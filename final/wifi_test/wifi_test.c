#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/watchdog.h"
#include "lwip/netif.h"
#include "lwip/udp.h"
#include "lwip/pbuf.h"
#include "dhcpserver.h"

#define WIFI_SSID "picow_audio"
#define WIFI_PASS "password123"      // WPA2 needs 8+ characters
#define AP_ADDR   "192.168.4.1"
#define MAX_TRIES 5
#define UDP_PORT  5004
#define PIN_ROLE  22                 // jumper to 3V3 = access point

#define PKT_BYTES  262               // 6 byte header + 128 samples x 2 bytes
#define PERIOD_US  8000              // 125 packets/s

#define LINK_TIMEOUT_US  2000000     // no packets for 2 s while streaming = link dead
#define REBOOT_AFTER_US  10000000    // dead and not recovered after 10 s = reboot
#define WDT_MS           8000        // hardware watchdog: reboot if loop stalls this long

typedef struct __attribute__((packed)) {
    uint16_t seq;
    uint32_t tx_us;                  // sender's clock, used only for jitter
    uint8_t  pad[PKT_BYTES - 6];     // filler standing in for audio
} pkt_t;

static volatile uint32_t peer_ip = 0;
static volatile uint32_t last_rx_us = 0;     // arrival time of the last packet from the peer
static volatile uint32_t rx_data = 0, lost = 0, reord = 0, bad_len = 0, resyncs = 0;
static volatile uint32_t jitter_us = 0, max_gap_us = 0;
static volatile bool have_prev = false;
static uint16_t prev_seq;
static uint32_t prev_arr, prev_tx;
static dhcp_server_t dhcp;           // must outlive main's setup
static bool is_ap;

// lwIP calls this for every packet that arrives on our port
static void recv_cb(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                    const ip_addr_t *addr, u16_t port) {
    uint32_t arr = time_us_32();
    uint32_t sender = ip4_addr_get_u32(ip_2_ip4(addr));
    uint32_t me = ip4_addr_get_u32(netif_ip4_addr(netif_default));
    if (sender != me) {                          // ignore our own broadcast
        peer_ip = sender;                        // hello or data, either tells us the peer
        last_rx_us = arr;                        // proof the link is alive
        if (p->tot_len == PKT_BYTES) {
            pkt_t h;
            pbuf_copy_partial(p, &h, sizeof(h), 0);
            rx_data++;
            if (have_prev) {
                int16_t gap = (int16_t)(h.seq - (uint16_t)(prev_seq + 1));
                if (gap > 0 && gap < 1000) lost += gap;
                else if (gap < 0 && gap > -1000) reord++;
                else if (gap != 0) resyncs++;    // huge jump: peer rebooted or counter wrapped
                // jitter: change in arrival spacing vs. send spacing
                int32_t d = (int32_t)((arr - prev_arr) - (h.tx_us - prev_tx));
                uint32_t ad = d < 0 ? -d : d;
                jitter_us += ((int32_t)ad - (int32_t)jitter_us) / 16;
                if (arr - prev_arr > max_gap_us) max_gap_us = arr - prev_arr;
            }
            prev_seq = h.seq; prev_arr = arr; prev_tx = h.tx_us; have_prev = true;
        } else if (p->tot_len != 5) {
            bad_len++;                           // 5 bytes = hello, anything else is unexpected
        }
    }
    pbuf_free(p);                                // always free
}

// Station only: join the AP's network, retrying. Returns 0 on success.
// Feeds the watchdog between tries (harmless if it isn't enabled yet).
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

    if (cyw43_arch_init()) { printf("cyw43 init failed\n"); return 1; }

    if (is_ap) {
        // Create the network, then run a DHCP server so stations get addresses
        cyw43_arch_enable_ap_mode(WIFI_SSID, WIFI_PASS, CYW43_AUTH_WPA2_AES_PSK);
        ip4_addr_t gw, mask;
        ipaddr_aton(AP_ADDR, &gw);
        IP4_ADDR(&mask, 255, 255, 255, 0);
        dhcp_server_init(&dhcp, &gw, &mask);
        printf("ROLE: access point, ssid=%s, ip=%s\n", WIFI_SSID, AP_ADDR);
    } else {
        cyw43_arch_enable_sta_mode();
        if (station_join(15000)) {                 // watchdog not enabled yet, long timeout is fine
            printf("giving up, rebooting\n");
            reboot_now();
        }
        // For the power-save-off run, uncomment (constant name unverified):
        // cyw43_wifi_pm(&cyw43_state, CYW43_NONE_PM);
        printf("ROLE: station, my IP: %s\n", ip4addr_ntoa(netif_ip4_addr(netif_default)));
    }

    // Open UDP port and register the receive callback
    struct udp_pcb *pcb = udp_new();
    cyw43_arch_lwip_begin();
    udp_bind(pcb, IP_ANY_TYPE, UDP_PORT);
    udp_recv(pcb, recv_cb, NULL);
    cyw43_arch_lwip_end();

    uint32_t next = time_us_32(), last_print = next, last_hello = 0;
    uint16_t seq = 0;
    uint32_t tx_ok = 0, tx_err = 0, last_rx = 0, last_blink = 0;
    uint32_t dead_events = 0, dead_since = 0;
    bool dead = false;
    bool led = false;

    watchdog_enable(WDT_MS, 1);        // reboot if not fed for 8 s; 1 = pause during debug

    while (true) {
        watchdog_update();             // feed it; a blocked loop stops feeding and the chip resets
        uint32_t now_us = time_us_32();

        // ---- link watchdog: streaming but nothing heard for 2 s ----
        if (peer_ip != 0 && (int32_t)(now_us - last_rx_us) > LINK_TIMEOUT_US) {
            printf("LINK DEAD (no packets for 2 s), restarting discovery\n");
            dead_events++;
            // reset stream state under the lwIP lock so the callback can't race us
            cyw43_arch_lwip_begin();
            peer_ip = 0;
            have_prev = false;
            cyw43_arch_lwip_end();
            seq = 0;

            if (!is_ap) {
                // rejoin only if the driver says the link is down
                // (cyw43_tcpip_link_status and CYW43_LINK_UP: names unverified)
                if (cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA) != CYW43_LINK_UP) {
                    printf("station link down, rejoining\n");
                    if (station_join(6000)) {          // 6 s per try keeps us under the 8 s watchdog
                        printf("rejoin failed, rebooting\n");
                        reboot_now();
                    }
                    printf("rejoined, my IP: %s\n",
                           ip4addr_ntoa(netif_ip4_addr(netif_default)));
                }
            }
            dead = true;
            now_us = time_us_32();
            dead_since = now_us;
            next = now_us;                 // the rejoin may have blocked, resync timers
            last_print = now_us;
            continue;
        }

        // ---- recovered: peer heard again ----
        if (dead && peer_ip != 0) {
            printf("link recovered\n");
            dead = false;
        }

        // ---- safety net: still no peer 10 s after a dead link, reboot ----
        if (dead && peer_ip == 0 && (now_us - dead_since) > REBOOT_AFTER_US) {
            printf("no recovery after 10 s, rebooting\n");
            reboot_now();
        }

        // every 8 ms: send either a hello (peer unknown) or a data packet (peer known)
        if ((int32_t)(now_us - next) >= 0) {
            // if we were blocked for more than 4 periods, skip the missed slots
            // instead of firing a burst of catch-up packets
            if ((int32_t)(now_us - next) > 4 * PERIOD_US) next = now_us;
            next += PERIOD_US;
            if (peer_ip == 0) {
                if (now_us - last_hello >= 1000000) {      // hello once per second
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
            } else {
                pkt_t h; memset(&h, 0, sizeof(h));
                h.seq = seq++; h.tx_us = now_us;
                struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, PKT_BYTES, PBUF_RAM);
                if (p) {
                    memcpy(p->payload, &h, sizeof(h));
                    ip_addr_t dst; ip_addr_set_ip4_u32(&dst, peer_ip);
                    cyw43_arch_lwip_begin();
                    err_t e = udp_sendto(pcb, p, &dst, UDP_PORT);
                    cyw43_arch_lwip_end();
                    pbuf_free(p);
                    if (e == ERR_OK) tx_ok++; else tx_err++;
                } else tx_err++;
            }
        }

        // once per second: print stats
        if (now_us - last_print >= 1000000) {
            last_print = now_us;       // not "+= 1000000", so a stall doesn't cause a burst of prints
            uint32_t rc = rx_data;
            printf("tx=%lu err=%lu | rx=%lu (+%lu/s) lost=%lu reord=%lu resync=%lu badlen=%lu jitter=%.2fms maxgap=%.1fms dead=%lu\n",
                   (unsigned long)tx_ok, (unsigned long)tx_err, (unsigned long)rc,
                   (unsigned long)(rc - last_rx), (unsigned long)lost, (unsigned long)reord,
                   (unsigned long)resyncs, (unsigned long)bad_len,
                   jitter_us / 1000.0f, max_gap_us / 1000.0f, (unsigned long)dead_events);
            last_rx = rc;
            max_gap_us = 0;                                // worst gap per second
        }

        // blink LED (on the WiFi chip) so you can see the loop is alive
        if (now_us / 1000 - last_blink >= 500) {
            last_blink = now_us / 1000;
            led = !led;
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led);
        }
    }
}