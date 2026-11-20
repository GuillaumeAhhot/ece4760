// wifi_diag: Pico<->Pico WiFi stall diagnostics. Replaces wifi_test.c (no audio).
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/watchdog.h"
#include "lwip/netif.h"
#include "lwip/udp.h"
#include "lwip/pbuf.h"
#include "dhcpserver.h"

// ===== experiment flags: change ONE per run. The CFG line at boot records them =====
#define CFG_HOTSPOT  0   // 0 = Pico AP (GPIO22 jumper) + Pico station, 1 = both join the phone hotspot
#define CFG_PM       0   // 0 = driver default, 1 = CYW43_PERFORMANCE_PM, 2 = no powersave (maintainer's value)
#define CFG_LED      1   // 1 = blink LED with cyw43_arch_gpio_put every 500 ms
#define CFG_RSSI     1   // 1 = station reads RSSI once a second (one extra chip call per second)
#define CFG_SCAN     1   // 1 = station scans nearby networks once at boot
#ifndef ARCH_POLL
#define ARCH_POLL    0   // set in CMakeLists.txt, not here
#endif
// =====================================================================================

#if CFG_HOTSPOT
#define NET_SSID "your_hotspot_name"
#define NET_PASS "your_password"
#else
#define NET_SSID "picow_audio"
#define NET_PASS "password123"
#endif
#define AP_ADDR   "192.168.4.1"
#define MAX_TRIES 5
#define UDP_PORT  5004
#define PIN_ROLE  22                 // jumper to 3V3 = access point (ignored if CFG_HOTSPOT)
#define PIN_TRIG  4                  // scope trigger: pulses on a failed or slow send

#define PKT_BYTES  262
#define PERIOD_US  8000
#define SLOW_SEND_US     5000
#define LINK_TIMEOUT_US  2000000
#define REBOOT_AFTER_US  10000000
#define WDT_MS           8000

typedef struct __attribute__((packed)) {
    uint16_t seq;
    uint32_t tx_us;
    uint8_t  pad[PKT_BYTES - 6];     // pad[0] carries the sender's RSSI (int8)
} pkt_t;

static volatile uint32_t peer_ip = 0, last_rx_us = 0;
static volatile uint32_t rx_data = 0, lost = 0, reord = 0, bad_len = 0, resyncs = 0;
static volatile uint32_t jitter_us = 0, max_gap_us = 0;
static volatile bool have_prev = false;
static volatile int8_t peer_rssi = 0;
static uint16_t prev_seq;
static uint32_t prev_arr, prev_tx;
static dhcp_server_t dhcp;
static bool is_ap;

static void trig(void) { gpio_put(PIN_TRIG, 1); busy_wait_us(50); gpio_put(PIN_TRIG, 0); }

static void recv_cb(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                    const ip_addr_t *addr, u16_t port) {
    uint32_t arr = time_us_32();
    uint32_t sender = ip4_addr_get_u32(ip_2_ip4(addr));
    uint32_t me = ip4_addr_get_u32(netif_ip4_addr(netif_default));
    if (sender != me) {
        peer_ip = sender;
        last_rx_us = arr;
        if (p->tot_len == PKT_BYTES) {
            pkt_t h;
            pbuf_copy_partial(p, &h, sizeof(h), 0);
            rx_data++;
            peer_rssi = (int8_t)h.pad[0];
            if (have_prev) {
                int16_t gap = (int16_t)(h.seq - (uint16_t)(prev_seq + 1));
                if (gap > 0 && gap < 1000) lost += gap;
                else if (gap < 0 && gap > -1000) reord++;
                else if (gap != 0) resyncs++;
                int32_t d = (int32_t)((arr - prev_arr) - (h.tx_us - prev_tx));
                uint32_t ad = d < 0 ? -d : d;
                jitter_us += ((int32_t)ad - (int32_t)jitter_us) / 16;
                if (arr - prev_arr > max_gap_us) max_gap_us = arr - prev_arr;
            }
            prev_seq = h.seq; prev_arr = arr; prev_tx = h.tx_us; have_prev = true;
        } else if (p->tot_len != 5) {
            bad_len++;
        }
    }
    pbuf_free(p);
}

#if CFG_SCAN
// names from pico-examples picow_wifi_scan (unverified for your SDK)
static int scan_cb(void *env, const cyw43_ev_scan_result_t *r) {
    if (r) printf("SCAN ch=%u rssi=%d ssid=%.*s\n", r->channel, r->rssi, r->ssid_len, r->ssid);
    return 0;
}
static void scan_once(void) {
    cyw43_wifi_scan_options_t opts = {0};
    if (cyw43_wifi_scan(&cyw43_state, &opts, NULL, scan_cb)) { printf("SCAN failed to start\n"); return; }
    absolute_time_t until = make_timeout_time_ms(5000);
    while (cyw43_wifi_scan_in_progress(&cyw43_state) && !time_reached(until)) {
#if ARCH_POLL
        cyw43_arch_poll();
#endif
        sleep_ms(50);
    }
}
#endif

static int station_join(uint32_t timeout_ms) {
    int err = -1;
    for (int attempt = 1; attempt <= MAX_TRIES && err; attempt++) {
        watchdog_update();
        printf("Connect attempt %d to %s ...\n", attempt, NET_SSID);
        err = cyw43_arch_wifi_connect_timeout_ms(NET_SSID, NET_PASS, CYW43_AUTH_WPA2_AES_PSK, timeout_ms);
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
    gpio_init(PIN_ROLE); gpio_set_dir(PIN_ROLE, GPIO_IN); gpio_pull_down(PIN_ROLE);
    gpio_init(PIN_TRIG); gpio_set_dir(PIN_TRIG, GPIO_OUT);
    sleep_ms(3000);

    is_ap = CFG_HOTSPOT ? false : gpio_get(PIN_ROLE);
    printf("CFG hotspot=%d pm=%d led=%d rssi=%d scan=%d arch=%s role=%s pkt=%d\n",
           CFG_HOTSPOT, CFG_PM, CFG_LED, CFG_RSSI, CFG_SCAN,
           ARCH_POLL ? "poll" : "background", is_ap ? "AP" : "STA", PKT_BYTES);
    if (watchdog_caused_reboot()) printf("BOOT: after watchdog reset\n");

    if (cyw43_arch_init()) { printf("cyw43 init failed\n"); return 1; }

    if (is_ap) {
        cyw43_arch_enable_ap_mode(NET_SSID, NET_PASS, CYW43_AUTH_WPA2_AES_PSK);
        ip4_addr_t gw, mask;
        ipaddr_aton(AP_ADDR, &gw);
        IP4_ADDR(&mask, 255, 255, 255, 0);
        dhcp_server_init(&dhcp, &gw, &mask);
        printf("WIFI: access point, ssid=%s, ip=%s\n", NET_SSID, AP_ADDR);
    } else {
        cyw43_arch_enable_sta_mode();
#if CFG_SCAN
        scan_once();
#endif
        if (station_join(15000)) { printf("giving up, rebooting\n"); reboot_now(); }
        printf("WIFI: station, my IP: %s\n", ip4addr_ntoa(netif_ip4_addr(netif_default)));
    }

    // power save (constants as quoted by the pico-sdk maintainer, unverified for your SDK)
#if CFG_PM == 1
    cyw43_wifi_pm(&cyw43_state, CYW43_PERFORMANCE_PM);
#elif CFG_PM == 2
    cyw43_wifi_pm(&cyw43_state, cyw43_pm_value(CYW43_NO_POWERSAVE_MODE, 20, 1, 1, 1));
#endif

    struct udp_pcb *pcb = udp_new();
    cyw43_arch_lwip_begin();
    udp_bind(pcb, IP_ANY_TYPE, UDP_PORT);
    udp_recv(pcb, recv_cb, NULL);
    cyw43_arch_lwip_end();

    uint32_t next = time_us_32(), last_print = next, last_hello = 0, prev_loop = 0;
    uint16_t seq = 0;
    uint32_t tx_ok = 0, tx_err = 0, last_rx = 0, last_blink = 0;
    uint32_t dead_events = 0, dead_since = 0;
    uint32_t send_max = 0, slow_sends = 0, led_max = 0, loop_max = 0;
    int last_err = 0;
    int8_t my_rssi = 0;
    bool dead = false, led = false;

    watchdog_enable(WDT_MS, 1);

    while (true) {
        watchdog_update();
#if ARCH_POLL
        cyw43_arch_poll();
#endif
        uint32_t now_us = time_us_32();
        if (prev_loop && now_us - prev_loop > loop_max) loop_max = now_us - prev_loop;
        prev_loop = now_us;

        // ---- link watchdog ----
        if (peer_ip != 0 && (int32_t)(now_us - last_rx_us) > LINK_TIMEOUT_US) {
            printf("LINK DEAD, restarting discovery\n");
            dead_events++;
            cyw43_arch_lwip_begin();
            peer_ip = 0; have_prev = false;
            cyw43_arch_lwip_end();
            seq = 0;
            if (!is_ap && cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA) != CYW43_LINK_UP) {
                printf("station link down, rejoining\n");
                if (station_join(6000)) { printf("rejoin failed, rebooting\n"); reboot_now(); }
            }
            dead = true;
            now_us = time_us_32();
            dead_since = now_us; next = now_us; last_print = now_us; prev_loop = 0;
            continue;
        }
        if (dead && peer_ip != 0) { printf("link recovered\n"); dead = false; }
        if (dead && peer_ip == 0 && (now_us - dead_since) > REBOOT_AFTER_US) {
            printf("no recovery after 10 s, rebooting\n");
            reboot_now();
        }

        // ---- every 8 ms: hello (peer unknown) or data packet (peer known) ----
        if ((int32_t)(now_us - next) >= 0) {
            if ((int32_t)(now_us - next) > 4 * PERIOD_US) next = now_us;
            next += PERIOD_US;
            if (peer_ip == 0) {
                if (now_us - last_hello >= 1000000) {
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
                h.seq = seq++; h.tx_us = now_us; h.pad[0] = (uint8_t)my_rssi;
                struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, PKT_BYTES, PBUF_RAM);
                if (p) {
                    memcpy(p->payload, &h, sizeof(h));
                    ip_addr_t dst; ip_addr_set_ip4_u32(&dst, peer_ip);
                    uint32_t t0 = time_us_32();               // includes waiting for the driver lock
                    cyw43_arch_lwip_begin();
                    err_t e = udp_sendto(pcb, p, &dst, UDP_PORT);
                    cyw43_arch_lwip_end();
                    uint32_t dt = time_us_32() - t0;
                    pbuf_free(p);
                    if (dt > send_max) send_max = dt;
                    if (e == ERR_OK) tx_ok++; else { tx_err++; last_err = e; }
                    if (e != ERR_OK || dt > SLOW_SEND_US) { slow_sends++; trig(); }
                } else tx_err++;
            }
        }

        // ---- once per second: machine-readable stats line ----
        if (now_us - last_print >= 1000000) {
            last_print = now_us;
#if CFG_RSSI
            if (!is_ap) {
                int32_t r;
                cyw43_arch_lwip_begin();
                int rc = cyw43_wifi_get_rssi(&cyw43_state, &r);
                cyw43_arch_lwip_end();
                if (rc == 0) my_rssi = (int8_t)r;
            }
#endif
            uint32_t rc2 = rx_data;
            printf("S t=%lu tx=%lu err=%lu lasterr=%d rx=%lu d=%lu lost=%lu reord=%lu resync=%lu "
                   "jit=%.2f maxgap=%.1f smax=%.1f slow=%lu ledmax=%.1f loopmax=%.1f rssi=%d prssi=%d dead=%lu\n",
                   (unsigned long)to_ms_since_boot(get_absolute_time()),
                   (unsigned long)tx_ok, (unsigned long)tx_err, last_err,
                   (unsigned long)rc2, (unsigned long)(rc2 - last_rx),
                   (unsigned long)lost, (unsigned long)reord, (unsigned long)resyncs,
                   jitter_us / 1000.0f, max_gap_us / 1000.0f, send_max / 1000.0f,
                   (unsigned long)slow_sends, led_max / 1000.0f, loop_max / 1000.0f,
                   (int)my_rssi, (int)peer_rssi, (unsigned long)dead_events);
            last_rx = rc2;
            max_gap_us = 0; send_max = 0; led_max = 0; loop_max = 0;
        }

#if CFG_LED
        if (now_us / 1000 - last_blink >= 500) {
            last_blink = now_us / 1000;
            led = !led;
            uint32_t t0 = time_us_32();
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led);
            uint32_t dt = time_us_32() - t0;
            if (dt > led_max) led_max = dt;
        }
#endif
    }
}