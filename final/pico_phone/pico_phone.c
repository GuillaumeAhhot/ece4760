// One-way audio over WiFi: the TX board sends its mic, the RX board plays it on the DAC.
// Same file on both boards. Set AUDIO_TX below (1 = mic board, 0 = speaker board).
// GPIO22 jumper to 3V3 picks WiFi role (AP or station). It is independent of TX/RX.

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/spi.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/watchdog.h"
#include "lwip/netif.h"
#include "lwip/udp.h"
#include "lwip/pbuf.h"
#include "dhcpserver.h"
#include "i2s_in.pio.h"

// ---------------- config ----------------
#define AUDIO_TX    0        // 1 = this board sends mic audio, 0 = this board plays it
#define GAIN_SHIFT  3        // playback gain = 2^GAIN_SHIFT (3 = x8). Keep low.

#define WIFI_SSID "picow_audio"
#define WIFI_PASS "password123"
#define AP_ADDR   "192.168.4.1"
#define MAX_TRIES 5
#define UDP_PORT  5004
#define PIN_ROLE  22                 // jumper to 3V3 = access point

// mic pins (PIO)
#define PIN_WS   10
#define PIN_SCK  11                  // must be PIN_WS + 1
#define PIN_SD   12
// DAC pins (SPI0)
#define PIN_DAC_CS   17
#define PIN_DAC_SCK  18
#define PIN_DAC_TX   19
#define SPI_PORT spi0
#define PIN_FRAME_TICK 3

#define FRAME_SAMPLES 128
#define FS_HZ         16000
#define SKIP_FRAMES   50             // ~400 ms, covers mic startup zeros
#define RING_FRAMES   16
#define PREFILL       4              // jitter buffer depth before playback starts (32 ms)

#define DAC_CFG_A   0x3000           // channel A, 1x gain, active
#define DAC_MID     (DAC_CFG_A | 2048)

#define LINK_TIMEOUT_US  3000000     // nothing heard for 3 s = link dead
#define REBOOT_AFTER_US  10000000
#define WDT_MS           8000

// ---------------- packet ----------------
typedef struct __attribute__((packed)) {
    uint16_t seq;                    // low 16 bits of the mic frame counter
    uint32_t tx_us;                  // sender clock, used only for jitter
    int16_t  s[FRAME_SAMPLES];       // 16-bit mic samples
} pkt_t;                             // 262 bytes
#define PKT_BYTES sizeof(pkt_t)

// ---------------- shared state ----------------
static volatile uint32_t peer_ip = 0;
static volatile uint32_t last_rx_us = 0;
static dhcp_server_t dhcp;
static bool is_ap;

// ---------------- TX side: mic ----------------
#if AUDIO_TX
static PIO pio = pio0;
static uint sm;
static int32_t mic_buf[2][FRAME_SAMPLES];
static int mch[2];
static volatile uint32_t mic_frames = 0;

static void mic_isr(void) {
    for (int i = 0; i < 2; i++) {
        if (dma_channel_get_irq0_status(mch[i])) {
            dma_channel_acknowledge_irq0(mch[i]);
            dma_channel_set_write_addr(mch[i], mic_buf[i], false);
            gpio_xor_mask(1u << PIN_FRAME_TICK);
            mic_frames++;
        }
    }
}

static void audio_init(void) {
    gpio_init(PIN_FRAME_TICK);
    gpio_set_dir(PIN_FRAME_TICK, GPIO_OUT);

    uint32_t sys = clock_get_hz(clk_sys);
    float div = (float)sys / (FS_HZ * 128.0f);

    sm = pio_claim_unused_sm(pio, true);
    uint off = pio_add_program(pio, &i2s_in_program);
    pio_sm_config c = i2s_in_program_get_default_config(off);
    sm_config_set_in_pins(&c, PIN_SD);
    sm_config_set_sideset_pins(&c, PIN_WS);
    sm_config_set_in_shift(&c, false, true, 32);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
    sm_config_set_clkdiv(&c, div);
    pio_gpio_init(pio, PIN_SD);
    pio_gpio_init(pio, PIN_WS);
    pio_gpio_init(pio, PIN_SCK);
    pio_sm_set_consecutive_pindirs(pio, sm, PIN_SD, 1, false);
    pio_sm_set_consecutive_pindirs(pio, sm, PIN_WS, 2, true);
    pio_sm_init(pio, sm, off, &c);

    mch[0] = dma_claim_unused_channel(true);
    mch[1] = dma_claim_unused_channel(true);
    for (int i = 0; i < 2; i++) {
        dma_channel_config d = dma_channel_get_default_config(mch[i]);
        channel_config_set_transfer_data_size(&d, DMA_SIZE_32);
        channel_config_set_read_increment(&d, false);
        channel_config_set_write_increment(&d, true);
        channel_config_set_dreq(&d, pio_get_dreq(pio, sm, false));
        channel_config_set_chain_to(&d, mch[1 - i]);
        dma_channel_configure(mch[i], &d, mic_buf[i], &pio->rxf[sm], FRAME_SAMPLES, false);
        dma_channel_set_irq0_enabled(mch[i], true);
    }
    // shared handler: the WiFi driver may also use DMA interrupts
    irq_add_shared_handler(DMA_IRQ_0, mic_isr, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
    irq_set_enabled(DMA_IRQ_0, true);

    dma_channel_start(mch[0]);
    pio_sm_set_enabled(pio, sm, true);
    printf("mic started, clk_sys=%lu PIO div=%.5f\n", (unsigned long)sys, div);
}
#endif

// ---------------- RX side: jitter buffer + DAC ----------------
#if !AUDIO_TX
static uint16_t ring[RING_FRAMES][FRAME_SAMPLES];     // DAC words, filled by the UDP callback
static volatile uint32_t ring_wr = 0, ring_rd = 0;
static volatile uint32_t underruns = 0, overruns = 0;
static volatile bool buffering = true;                // true = waiting to refill to PREFILL

static uint16_t dac_buf[2][FRAME_SAMPLES];
static int dch[2];

// producer (UDP callback): s == NULL pushes a silent frame
static void ring_push_frame(const int16_t *s) {
    if (ring_wr - ring_rd >= RING_FRAMES) { overruns++; return; }
    uint16_t *dst = ring[ring_wr % RING_FRAMES];
    for (int i = 0; i < FRAME_SAMPLES; i++) {
        if (s) {
            int32_t v = ((int32_t)s[i] << GAIN_SHIFT) >> 4;   // 16-bit -> 12-bit with gain
            if (v > 2047) v = 2047;
            if (v < -2048) v = -2048;
            dst[i] = DAC_CFG_A | (uint16_t)(v + 2048);
        } else {
            dst[i] = DAC_MID;
        }
    }
    ring_wr++;                                        // publish last
}

// consumer (DMA ISR)
static void ring_pop_into(uint16_t *dst) {
    uint32_t fill = ring_wr - ring_rd;
    if (buffering && fill >= PREFILL) buffering = false;
    if (!buffering && fill > 0) {
        for (int i = 0; i < FRAME_SAMPLES; i++) dst[i] = ring[ring_rd % RING_FRAMES][i];
        ring_rd++;
    } else {
        for (int i = 0; i < FRAME_SAMPLES; i++) dst[i] = DAC_MID;
        if (!buffering) { underruns++; buffering = true; }   // ran dry: rebuffer
    }
}

static void dac_isr(void) {
    for (int i = 0; i < 2; i++) {
        if (dma_channel_get_irq1_status(dch[i])) {
            dma_channel_acknowledge_irq1(dch[i]);
            dma_channel_set_read_addr(dch[i], dac_buf[i], false);
            ring_pop_into(dac_buf[i]);
        }
    }
}

static void audio_init(void) {
    uint32_t sys = clock_get_hz(clk_sys);

    spi_init(SPI_PORT, 20 * 1000 * 1000);
    spi_set_format(SPI_PORT, 16, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(PIN_DAC_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_DAC_TX, GPIO_FUNC_SPI);
    gpio_set_function(PIN_DAC_CS, GPIO_FUNC_SPI);

    int timer = dma_claim_unused_timer(true);
    uint32_t den = sys / (FS_HZ / 2);                 // rate = sys * 2 / den = 16 kHz
    dma_timer_set_fraction(timer, 2, den);

    dch[0] = dma_claim_unused_channel(true);
    dch[1] = dma_claim_unused_channel(true);
    for (int i = 0; i < 2; i++) {
        dma_channel_config d = dma_channel_get_default_config(dch[i]);
        channel_config_set_transfer_data_size(&d, DMA_SIZE_16);
        channel_config_set_read_increment(&d, true);
        channel_config_set_write_increment(&d, false);
        channel_config_set_dreq(&d, dma_get_timer_dreq(timer));
        channel_config_set_chain_to(&d, dch[1 - i]);
        dma_channel_configure(dch[i], &d, &spi_get_hw(SPI_PORT)->dr, dac_buf[i],
                              FRAME_SAMPLES, false);
        dma_channel_set_irq1_enabled(dch[i], true);
    }
    irq_add_shared_handler(DMA_IRQ_1, dac_isr, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
    irq_set_enabled(DMA_IRQ_1, true);

    // DAC starts right away, playing silence until the ring has PREFILL frames
    ring_pop_into(dac_buf[0]);
    ring_pop_into(dac_buf[1]);
    dma_channel_start(dch[0]);
    printf("DAC started, timer rate=%llu Hz\n", (unsigned long long)sys * 2 / den);
}
#endif

// ---------------- receive stats ----------------
static volatile uint32_t rx_data = 0, lost = 0, reord = 0, resyncs = 0, bad_len = 0;
static volatile uint32_t jitter_us = 0, max_gap_us = 0;
static volatile bool have_prev = false;
static uint16_t prev_seq;
static uint32_t prev_arr, prev_tx;
static pkt_t rxp;                                     // static: keeps the callback stack small

static void recv_cb(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                    const ip_addr_t *addr, u16_t port) {
    uint32_t arr = time_us_32();
    uint32_t sender = ip4_addr_get_u32(ip_2_ip4(addr));
    uint32_t me = ip4_addr_get_u32(netif_ip4_addr(netif_default));
    if (sender != me) {
        peer_ip = sender;                             // any packet tells us the peer
        last_rx_us = arr;
        if (p->tot_len == PKT_BYTES) {
            pbuf_copy_partial(p, &rxp, sizeof(rxp), 0);
            bool play = true;
            if (have_prev) {
                int16_t gap = (int16_t)(rxp.seq - (uint16_t)(prev_seq + 1));
                if (gap > 0 && gap < 1000) {
                    lost += gap;
#if !AUDIO_TX
                    for (int k = 0; k < gap && k < 3; k++) ring_push_frame(NULL);  // keep timing
#endif
                } else if (gap < 0 && gap > -1000) {
                    reord++;
                    play = false;                     // late packet: drop it
                } else if (gap != 0) {
                    resyncs++;
                }
                int32_t d = (int32_t)((arr - prev_arr) - (rxp.tx_us - prev_tx));
                uint32_t ad = d < 0 ? -d : d;
                jitter_us += ((int32_t)ad - (int32_t)jitter_us) / 16;
                if (arr - prev_arr > max_gap_us) max_gap_us = arr - prev_arr;
            }
            if (play) {
                rx_data++;
                prev_seq = rxp.seq; prev_arr = arr; prev_tx = rxp.tx_us; have_prev = true;
#if !AUDIO_TX
                ring_push_frame(rxp.s);
#endif
            }
        } else if (p->tot_len != 5) {
            bad_len++;                                // 5 bytes = hello
        }
    }
    pbuf_free(p);
}

// ---------------- WiFi helpers ----------------
static int station_join(uint32_t timeout_ms) {
    int err = -1;
    for (int attempt = 1; attempt <= MAX_TRIES && err; attempt++) {
        watchdog_update();
        printf("Connect attempt %d to %s ...\n", attempt, WIFI_SSID);
        err = cyw43_arch_wifi_connect_timeout_ms(
            WIFI_SSID, WIFI_PASS, CYW43_AUTH_WPA2_AES_PSK, timeout_ms);
        if (err) printf("  failed, error %d\n", err);
    }
    return err;
}

static void reboot_now(void) {
    watchdog_reboot(0, 0, 100);
    while (true) tight_loop_contents();
}

int main(void) {
    stdio_init_all();
    gpio_init(PIN_ROLE);
    gpio_set_dir(PIN_ROLE, GPIO_IN);
    gpio_pull_down(PIN_ROLE);
    sleep_ms(3000);

    is_ap = gpio_get(PIN_ROLE);
    printf("audio role: %s\n", AUDIO_TX ? "TX (mic)" : "RX (speaker)");

    if (cyw43_arch_init()) { printf("cyw43 init failed\n"); return 1; }

    if (is_ap) {
        cyw43_arch_enable_ap_mode(WIFI_SSID, WIFI_PASS, CYW43_AUTH_WPA2_AES_PSK);
        ip4_addr_t gw, mask;
        ipaddr_aton(AP_ADDR, &gw);
        IP4_ADDR(&mask, 255, 255, 255, 0);
        dhcp_server_init(&dhcp, &gw, &mask);
        printf("WIFI: access point, ssid=%s, ip=%s\n", WIFI_SSID, AP_ADDR);
    } else {
        cyw43_arch_enable_sta_mode();
        if (station_join(15000)) { printf("giving up, rebooting\n"); reboot_now(); }
        printf("WIFI: station, my IP: %s\n", ip4addr_ntoa(netif_ip4_addr(netif_default)));
    }

    struct udp_pcb *pcb = udp_new();
    cyw43_arch_lwip_begin();
    udp_bind(pcb, IP_ANY_TYPE, UDP_PORT);
    udp_recv(pcb, recv_cb, NULL);
    cyw43_arch_lwip_end();

    audio_init();                                     // after WiFi is up

    uint32_t last_print = time_us_32(), last_hello = 0, last_blink = 0;
    uint32_t tx_ok = 0, tx_err = 0, last_rx = 0, dead_events = 0, dead_since = 0;
    uint32_t seen = 0, mic_missed = 0;
    bool dead = false, led = false;

    watchdog_enable(WDT_MS, 1);

    while (true) {
        watchdog_update();
        uint32_t now_us = time_us_32();

        // ---- link watchdog ----
        if (peer_ip != 0 && (int32_t)(now_us - last_rx_us) > LINK_TIMEOUT_US) {
            printf("LINK DEAD, restarting discovery\n");
            dead_events++;
            cyw43_arch_lwip_begin();
            peer_ip = 0;
            have_prev = false;
            cyw43_arch_lwip_end();
            if (!is_ap && cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA) != CYW43_LINK_UP) {
                printf("station link down, rejoining\n");
                if (station_join(6000)) { printf("rejoin failed, rebooting\n"); reboot_now(); }
            }
            dead = true;
            dead_since = last_print = time_us_32();
            continue;
        }
        if (dead && peer_ip != 0) { printf("link recovered\n"); dead = false; }
        if (dead && peer_ip == 0 && (now_us - dead_since) > REBOOT_AFTER_US) {
            printf("no recovery after 10 s, rebooting\n");
            reboot_now();
        }

        // ---- hello: TX only while peer unknown; RX always (so TX can always find it) ----
#if AUDIO_TX
        bool want_hello = (peer_ip == 0);
        uint32_t hello_period = 1000000;
#else
        bool want_hello = true;
        uint32_t hello_period = 500000;
#endif
        if (want_hello && now_us - last_hello >= hello_period) {
            last_hello = now_us;
            struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, 5, PBUF_RAM);
            if (p) {
                memcpy(p->payload, "hello", 5);
                cyw43_arch_lwip_begin();
                udp_sendto(pcb, p, IP_ADDR_BROADCAST, UDP_PORT);
                cyw43_arch_lwip_end();
                pbuf_free(p);
            }
        }

        // ---- TX: one packet per finished mic frame ----
#if AUDIO_TX
        uint32_t n = mic_frames;
        if (n != seen) {
            if (n - seen > 1) mic_missed += n - seen - 1;
            int32_t *b = mic_buf[(n - 1) & 1];
            seen = n;
            if (seen > SKIP_FRAMES && peer_ip != 0) {
                struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, PKT_BYTES, PBUF_RAM);
                if (p) {
                    pkt_t *h = (pkt_t *)p->payload;
                    h->seq = (uint16_t)n;             // gaps = missed mic frames
                    h->tx_us = now_us;
                    for (int i = 0; i < FRAME_SAMPLES; i++) {
                        int32_t s24 = ((int32_t)((uint32_t)b[i] << 1)) >> 8;
                        h->s[i] = (int16_t)(s24 >> 8);
                    }
                    ip_addr_t dst; ip_addr_set_ip4_u32(&dst, peer_ip);
                    cyw43_arch_lwip_begin();
                    err_t e = udp_sendto(pcb, p, &dst, UDP_PORT);
                    cyw43_arch_lwip_end();
                    pbuf_free(p);
                    if (e == ERR_OK) tx_ok++; else tx_err++;
                } else tx_err++;
            }
        }
#endif

        // ---- stats once per second ----
        if (now_us - last_print >= 1000000) {
            last_print = now_us;
            uint32_t rc = rx_data;
#if AUDIO_TX
            printf("TX sent=%lu err=%lu micmiss=%lu | heard=%lu dead=%lu\n",
                   (unsigned long)tx_ok, (unsigned long)tx_err, (unsigned long)mic_missed,
                   (unsigned long)rc, (unsigned long)dead_events);
#else
            printf("RX rx=%lu (+%lu/s) lost=%lu reord=%lu resync=%lu | ring=%lu under=%lu over=%lu | jitter=%.2fms maxgap=%.1fms dead=%lu\n",
                   (unsigned long)rc, (unsigned long)(rc - last_rx), (unsigned long)lost,
                   (unsigned long)reord, (unsigned long)resyncs,
                   (unsigned long)(ring_wr - ring_rd), (unsigned long)underruns,
                   (unsigned long)overruns, jitter_us / 1000.0f, max_gap_us / 1000.0f,
                   (unsigned long)dead_events);
#endif
            last_rx = rc;
            max_gap_us = 0;
        }

        if (now_us / 1000 - last_blink >= 500) {
            last_blink = now_us / 1000;
            led = !led;
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led);
        }
    }
}