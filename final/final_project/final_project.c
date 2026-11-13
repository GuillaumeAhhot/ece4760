#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/spi.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "i2s_in.pio.h"

// ---- mode: 1 = 1 kHz sine out of the DAC (step 2.1), 0 = mic passthrough (2.2) ----
#define TEST_SINE   1
#define GAIN_SHIFT  3        // mic gain = 2^GAIN_SHIFT (3 = x8). Keep low for speakers.

#define PIN_WS   10
#define PIN_SCK  11          // must be PIN_WS + 1
#define PIN_SD   12
#define PIN_DAC_CS   17
#define PIN_DAC_SCK  18
#define PIN_DAC_TX   19
#define SPI_PORT spi0
#define PIN_FRAME_TICK 3
#define FRAME_SAMPLES 128
#define FS_HZ    16000
#define SKIP_FRAMES 50       // ~400 ms, covers mic startup zeros
#define RING_FRAMES 8
#define PREFILL     4

#define DAC_CFG_A   0x3000   // channel A, 1x gain, active
#define DAC_MID     (DAC_CFG_A | 2048)

// ---------- mic side ----------
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

// ---------- ring: main (producer) -> DAC ISR (consumer) ----------
static uint16_t ring[RING_FRAMES][FRAME_SAMPLES];
static volatile uint32_t ring_wr = 0, ring_rd = 0;
static volatile uint32_t underruns = 0, overruns = 0;

// ---------- DAC side ----------
static uint16_t dac_buf[2][FRAME_SAMPLES];
static int dch[2];

static void ring_pop_into(uint16_t *dst) {
    if (ring_wr - ring_rd > 0) {
        for (int i = 0; i < FRAME_SAMPLES; i++) dst[i] = ring[ring_rd % RING_FRAMES][i];
        ring_rd++;
    } else {
        for (int i = 0; i < FRAME_SAMPLES; i++) dst[i] = DAC_MID;
        underruns++;
    }
}

static void dac_isr(void) {
    for (int i = 0; i < 2; i++) {
        if (dma_channel_get_irq1_status(dch[i])) {
            dma_channel_acknowledge_irq1(dch[i]);
            dma_channel_set_read_addr(dch[i], dac_buf[i], false);  // re-arm
            ring_pop_into(dac_buf[i]);   // the other channel is playing now
        }
    }
}

int main(void) {
    stdio_init_all();
    sleep_ms(2000);

    gpio_init(PIN_FRAME_TICK);
    gpio_set_dir(PIN_FRAME_TICK, GPIO_OUT);

    uint32_t sys = clock_get_hz(clk_sys);
    float div = (float)sys / (FS_HZ * 128.0f);
    printf("clk_sys=%lu PIO div=%.5f mode=%s\n", (unsigned long)sys, div,
           TEST_SINE ? "SINE" : "PASSTHROUGH");

    // ---- mic: PIO + DMA ping-pong (same as before) ----
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
    irq_set_exclusive_handler(DMA_IRQ_0, mic_isr);
    irq_set_enabled(DMA_IRQ_0, true);

    // ---- DAC: SPI0 + timer-paced DMA ping-pong ----
    spi_init(SPI_PORT, 20 * 1000 * 1000);
    spi_set_format(SPI_PORT, 16, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(PIN_DAC_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_DAC_TX, GPIO_FUNC_SPI);
    gpio_set_function(PIN_DAC_CS, GPIO_FUNC_SPI);   // hardware CS, as in the lab

    int timer = dma_claim_unused_timer(true);
    uint32_t den = sys / (FS_HZ / 2);               // rate = sys * 2 / den = 16 kHz
    dma_timer_set_fraction(timer, 2, den);
    printf("DAC timer rate = %llu Hz\n", (unsigned long long)sys * 2 / den);

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
    irq_set_exclusive_handler(DMA_IRQ_1, dac_isr);
    irq_set_enabled(DMA_IRQ_1, true);

    // ---- start the mic ----
    dma_channel_start(mch[0]);
    pio_sm_set_enabled(pio, sm, true);

    uint32_t seen = 0, missed = 0, sine_idx = 0;
    bool dac_started = false;
    static const int16_t sine16[16] = {   // 1 kHz at 16 kHz, 16 samples per cycle
        0, 383, 707, 924, 1000, 924, 707, 383, 0, -383, -707, -924, -1000, -924, -707, -383 };
    uint64_t last_print = time_us_64();

    while (true) {
        uint32_t n = mic_frames;
        if (n != seen) {
            if (n - seen > 1) missed += n - seen - 1;
            int32_t *b = mic_buf[(n - 1) & 1];
            seen = n;

            if (seen > SKIP_FRAMES) {
                if (ring_wr - ring_rd >= RING_FRAMES) {
                    overruns++;
                } else {
                    uint16_t *dst = ring[ring_wr % RING_FRAMES];
                    for (int i = 0; i < FRAME_SAMPLES; i++) {
                        int32_t v;
#if TEST_SINE
                        v = sine16[sine_idx++ & 15];
#else
                        int32_t s24 = ((int32_t)((uint32_t)b[i] << 1)) >> 8;
                        v = s24 >> (12 - GAIN_SHIFT);       // 12-bit signed with gain
                        if (v > 2047) v = 2047;
                        if (v < -2048) v = -2048;
#endif
                        dst[i] = DAC_CFG_A | (uint16_t)(v + 2048);
                    }
                    ring_wr++;
                }
            }

            if (!dac_started && (ring_wr - ring_rd) >= PREFILL) {
                ring_pop_into(dac_buf[0]);
                ring_pop_into(dac_buf[1]);
                dma_channel_start(dch[0]);
                dac_started = true;
            }
        }

        uint64_t now = time_us_64();
        if (now - last_print >= 1000000) {
            last_print = now;
            printf("mic=%lu missed=%lu ring=%lu under=%lu over=%lu\n",
                   (unsigned long)seen, (unsigned long)missed,
                   (unsigned long)(ring_wr - ring_rd),
                   (unsigned long)underruns, (unsigned long)overruns);
        }
    }
}