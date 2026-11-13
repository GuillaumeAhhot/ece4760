#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "i2s_in.pio.h"

#define PIN_WS   10
#define PIN_SCK  11          // must be PIN_WS + 1
#define PIN_SD   12
#define PIN_FRAME_TICK 3
#define FRAME_SAMPLES 128
#define FS_HZ    16000
#define SKIP_FRAMES 50       // ~400 ms, covers mic startup zeros

// Packet sent to the laptop (little endian, 268 bytes)
typedef struct __attribute__((packed)) {
    uint32_t sync;           // 0x55AA55AA -> bytes AA 55 AA 55
    uint32_t counter;        // DMA frame count, gaps = lost frames
    uint32_t stalls;         // PIO FIFO overruns so far
    int16_t  s[FRAME_SAMPLES];
} packet_t;

static PIO pio = pio0;
static uint sm;
static int32_t buf[2][FRAME_SAMPLES];
static int ch[2];
static volatile uint32_t frames_done = 0;
static packet_t pkt;

static void dma_isr(void) {
    for (int i = 0; i < 2; i++) {
        if (dma_channel_get_irq0_status(ch[i])) {
            dma_channel_acknowledge_irq0(ch[i]);
            dma_channel_set_write_addr(ch[i], buf[i], false);
            gpio_xor_mask(1u << PIN_FRAME_TICK);
            frames_done++;
        }
    }
}

int main(void) {
    stdio_init_all();
    stdio_set_translate_crlf(&stdio_usb, false);  // binary data: no \n -> \r\n
    sleep_ms(2000);
    // NOTE: no printf anywhere in this file. Text would corrupt the stream.

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

    ch[0] = dma_claim_unused_channel(true);
    ch[1] = dma_claim_unused_channel(true);
    for (int i = 0; i < 2; i++) {
        dma_channel_config d = dma_channel_get_default_config(ch[i]);
        channel_config_set_transfer_data_size(&d, DMA_SIZE_32);
        channel_config_set_read_increment(&d, false);
        channel_config_set_write_increment(&d, true);
        channel_config_set_dreq(&d, pio_get_dreq(pio, sm, false));
        channel_config_set_chain_to(&d, ch[1 - i]);
        dma_channel_configure(ch[i], &d, buf[i], &pio->rxf[sm], FRAME_SAMPLES, false);
        dma_channel_set_irq0_enabled(ch[i], true);
    }
    irq_set_exclusive_handler(DMA_IRQ_0, dma_isr);
    irq_set_enabled(DMA_IRQ_0, true);

    dma_channel_start(ch[0]);
    pio_sm_set_enabled(pio, sm, true);

    uint32_t seen = 0, stalls = 0;
    pkt.sync = 0x55AA55AA;

    while (true) {
        uint32_t n = frames_done;
        if (n == seen) continue;
        int32_t *b = buf[(n - 1) & 1];
        seen = n;
        if (seen <= SKIP_FRAMES) continue;

        // 24-bit sample (bits 30..7) -> top 16 bits, as signed 16-bit
        for (int i = 0; i < FRAME_SAMPLES; i++) {
            int32_t s24 = ((int32_t)((uint32_t)b[i] << 1)) >> 8;
            pkt.s[i] = (int16_t)(s24 >> 8);
        }

        if (pio->fdebug & (1u << (PIO_FDEBUG_RXSTALL_LSB + sm))) {
            stalls++;
            pio->fdebug = 1u << (PIO_FDEBUG_RXSTALL_LSB + sm);
        }

        pkt.counter = seen;
        pkt.stalls = stalls;
        stdio_put_string((const char *)&pkt, sizeof(pkt), false, false);
    }
}