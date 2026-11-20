#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
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

static volatile uint32_t rx_count = 0;
static volatile uint32_t peer_ip = 0;
static dhcp_server_t dhcp;           // must outlive main's setup

static void recv_cb(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                    const ip_addr_t *addr, u16_t port) {
    uint32_t sender = ip4_addr_get_u32(ip_2_ip4(addr));
    uint32_t me = ip4_addr_get_u32(netif_ip4_addr(netif_default));
    if (sender != me) {              // ignore our own broadcast
        peer_ip = sender;
        rx_count++;
    }
    pbuf_free(p);
}

int main(void) {
    stdio_init_all();
    gpio_init(PIN_ROLE);
    gpio_set_dir(PIN_ROLE, GPIO_IN);
    gpio_pull_down(PIN_ROLE);
    sleep_ms(3000);

    bool is_ap = gpio_get(PIN_ROLE);

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
        int err = -1;
        for (int attempt = 1; attempt <= MAX_TRIES && err; attempt++) {
            printf("Connect attempt %d to %s ...\n", attempt, WIFI_SSID);
            err = cyw43_arch_wifi_connect_timeout_ms(
                WIFI_SSID, WIFI_PASS, CYW43_AUTH_WPA2_AES_PSK, 15000);
            if (err) printf("  failed, error %d\n", err);
        }
        if (err) { printf("giving up\n"); return 1; }
        printf("ROLE: station, my IP: %s\n", ip4addr_ntoa(netif_ip4_addr(netif_default)));
    }

    struct udp_pcb *pcb = udp_new();
    cyw43_arch_lwip_begin();
    udp_bind(pcb, IP_ANY_TYPE, UDP_PORT);
    udp_recv(pcb, recv_cb, NULL);
    cyw43_arch_lwip_end();

    uint32_t seen = 0, sent = 0, last_send = 0, last_blink = 0;
    bool led = false;

    while (true) {
        uint32_t now = to_ms_since_boot(get_absolute_time());

        if (now - last_send >= 1000) {          // broadcast "hello" once a second
            last_send = now;
            struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, 5, PBUF_RAM);
            if (p) {
                memcpy(p->payload, "hello", 5);
                cyw43_arch_lwip_begin();
                err_t e = udp_sendto(pcb, p, IP_ADDR_BROADCAST, UDP_PORT);
                cyw43_arch_lwip_end();
                pbuf_free(p);
                if (e == ERR_OK) sent++; else printf("send error %d\n", e);
            }
        }

        if (rx_count != seen) {
            seen = rx_count;
            ip4_addr_t a; ip4_addr_set_u32(&a, peer_ip);
            printf("heard peer %s (rx=%lu, sent=%lu)\n", ip4addr_ntoa(&a),
                   (unsigned long)seen, (unsigned long)sent);
        }

        if (now - last_blink >= 500) {
            last_blink = now;
            led = !led;
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led);
        }
    }
}