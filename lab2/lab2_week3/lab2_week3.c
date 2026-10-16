/**
 * ECE 4760 Lab 2 - Digital Galton Board
 * Based on Hunter Adams (vha3@cornell.edu) VGA and DMA demos
 *
 * HARDWARE CONNECTIONS
 *  - GPIO 16 ---> VGA Hsync
 *  - GPIO 17 ---> VGA Vsync
 *  - GPIO 18 ---> VGA Green lo-bit --> 470 ohm resistor --> VGA-Green
 *  - GPIO 19 ---> VGA Green hi-bit --> 330 ohm resistor --> VGA-Green
 *  - GPIO 20 ---> 330 ohm resistor ---> VGA-Blue
 *  - GPIO 21 ---> 330 ohm resistor ---> VGA-Red
 *    (1-bit color: only GPIO 19 carries pixels, the rest stay low)
 *  - GPIO 4/5/6/7 ---> SPI0 MISO/CS/SCK/MOSI; DAC uses CS/SCK/MOSI
 *  - MCP4822 VOUTA ---> amp
 *  - GPIO 27/28 ---> encoder A/B, encoder COM ---> GND
 *  - GPIO 15 ---> encoder switch, other switch pin ---> GND
 *  - GPIO 2 ---> scope: core 0 busy (low while waiting on core 1)
 *  - GPIO 3 ---> scope: core 1 busy
 *  - GPIO 25 ---> onboard LED (on when the 60 fps deadline is missed)
 *  - GND ---> VGA-GND
 *
 * RESOURCES USED
 *  - PIO state machines 0, 1, 2 on PIO0
 *  - 4 DMA channels (VGA library) + 2 DMA channels (audio)
 *  - DMA pacing timer 0, SPI0
 *  - Both cores, 2 semaphores
 *  - 76.8 kB RAM (2 framebuffers, 1 bit/pixel) + 400 kB (balls)
 *  - Ball loop runs from RAM
 */

#include "VGA/vga1_graphics.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/divider.h"
#include "pico/multicore.h"
#include "pico/sync.h"

#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/spi.h"
#include "hardware/clocks.h"
#include "hardware/pll.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "hardware/vreg.h"
#include "hardware/structs/qmi.h"

#include "pt_cornell_rp2040_v1_4.h"

// Force inlining so the hot path stays inside the RAM-resident ball loop.
#define HOT static inline __attribute__((always_inline))

/* Fixed-point arithmetic */

typedef int32_t fix15;

#define multfix15(a,b) \
  ((fix15)(((int64_t)(a) * (int64_t)(b)) >> 15))

#define float2fix15(a) ((fix15)((a) * 32768.0))
#define fix2float15(a) ((float)(a) / 32768.0)
#define absfix15(a)    abs(a)
#define int2fix15(a)   ((fix15)((int64_t)(a) * 32768LL))
#define fix2int15(a)   ((int)((a) >> 15))

#define divfix(a,b) \
  ((fix15)div_s64s64((int64_t)(a) * 32768LL, (int64_t)(b)))

/* Drawing (1 bit per pixel, bit 0 of each byte is the leftmost pixel) */

// Disc half-widths, from the top row to the bottom row.
static const int8_t ball_hw[9] = {
  2, 3, 4, 4, 4, 4, 4, 3, 2
};

static const int8_t peg_hw[13] = {
  2, 4, 5, 5, 6, 6, 6, 6, 6, 5, 5, 4, 2
};

// Edge pixels at each end of a row.
// Top and bottom rows are drawn across their full width.
static const int8_t ball_edge[9] = {
  4, 1, 1, 1, 1, 1, 1, 1, 4
};

static const int8_t peg_edge[13] = {
  4, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 4
};

// Outline bit pattern per row; bit i is pixel (x - r + i).
static uint32_t ball_rows[9];
static uint32_t peg_rows[13];

// Precompute outline patterns from the half-width and edge tables.
static void buildRows(
  uint32_t *rows, const int8_t *hw, const int8_t *edge, int r)
{
  for (int i = 0; i <= 2 * r; i++) {
    int left = r - hw[i];
    int width = 2 * hw[i];
    int n = edge[i];

    if (2 * n >= width) {
      rows[i] = ((1u << width) - 1) << left;
    } else {
      uint32_t ends = (1u << n) - 1;
      rows[i] = (ends << left) | (ends << (left + width - n));
    }
  }
}

// OR a precomputed sprite into the framebuffer: one shift per row, no branches.
HOT void drawSprite(int x, int y, const uint32_t *rows, int r)
{
  if (y < r || y > 479 - r || x < r || x > 639 - r)
    return;

  int sx = x - r;
  int sh = sx & 7;
  uint8_t *p = (uint8_t *)current_draw_buffer
               + VGA_ROW_BYTES * (y - r) + (sx >> 3);

  for (int i = 0; i <= 2 * r; i++, p += VGA_ROW_BYTES) {
    uint32_t m = rows[i] << sh;

    p[0] |= (uint8_t)m;
    p[1] |= (uint8_t)(m >> 8);

    // Pegs are 12 px wide and can span 3 bytes; folds away for balls.
    if (r > 4)
      p[2] |= (uint8_t)(m >> 16);
  }
}

// Light pixels [x, x + w) in one row.
static inline void hspan(uint8_t *row, int x, int w)
{
  int x1 = x + w - 1;
  int b0 = x >> 3;
  int b1 = x1 >> 3;
  uint8_t m0 = (uint8_t)(0xFF << (x & 7));
  uint8_t m1 = (uint8_t)(0xFF >> (7 - (x1 & 7)));

  if (b0 == b1) {
    row[b0] |= m0 & m1;
    return;
  }

  row[b0] |= m0;

  for (int b = b0 + 1; b < b1; b++)
    row[b] = 0xFF;

  row[b1] |= m1;
}

// Draw a rectangle outline; w and h are pixel counts.
static inline void drawRectOutlineFast(int x, int y, int w, int h)
{
  if (w <= 0 || h <= 0 ||
      x < 0 || y < 0 || x + w > 640 || y + h > 480)
    return;

  uint8_t *row = (uint8_t *)current_draw_buffer + VGA_ROW_BYTES * y;
  int lb = x >> 3;
  int rb = (x + w - 1) >> 3;
  uint8_t lm = (uint8_t)(1u << (x & 7));
  uint8_t rm = (uint8_t)(1u << ((x + w - 1) & 7));

  hspan(row, x, w);

  for (int i = 1; i < h - 1; i++) {
    row += VGA_ROW_BYTES;
    row[lb] |= lm;
    row[rb] |= rm;
  }

  if (h > 1)
    hspan(row + VGA_ROW_BYTES, x, w);
}

/* Audio */

#define sine_table_size 256
#define DAC_config_chan_A 0b0011000000000000

#define PIN_MISO 4
#define PIN_CS   5
#define PIN_SCK  6
#define PIN_MOSI 7
#define SPI_PORT spi0

int raw_sin[sine_table_size];
unsigned short DAC_data[sine_table_size];
unsigned short *address_pointer = &DAC_data[0];

int data_chan;
int ctrl_chan;

// Configure one-shot playback; the control channel rewinds the data channel.
void init_audio_dma()
{
  spi_init(SPI_PORT, 20000000);
  spi_set_format(SPI_PORT, 16, 0, 0, 0);

  gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
  gpio_set_function(PIN_CS, GPIO_FUNC_SPI);
  gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
  gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

  for (int i = 0; i < sine_table_size; i++) {
    raw_sin[i] = (int)(
      2047 * sin((float)i * 6.283 / (float)sine_table_size) + 2047
    );

    DAC_data[i] = DAC_config_chan_A | (raw_sin[i] & 0x0fff);
  }

  data_chan = dma_claim_unused_channel(true);
  ctrl_chan = dma_claim_unused_channel(true);

  dma_channel_config c = dma_channel_get_default_config(ctrl_chan);
  channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
  channel_config_set_read_increment(&c, false);
  channel_config_set_write_increment(&c, false);
  channel_config_set_chain_to(&c, data_chan);

  dma_channel_configure(
    ctrl_chan,
    &c,
    &dma_hw->ch[data_chan].read_addr,
    &address_pointer,
    1,
    false
  );

  dma_channel_config c2 = dma_channel_get_default_config(data_chan);
  channel_config_set_transfer_data_size(&c2, DMA_SIZE_16);
  channel_config_set_read_increment(&c2, true);
  channel_config_set_write_increment(&c2, false);

  // Approximately 50.4 kHz at a 300 MHz system clock.
  dma_timer_set_fraction(0, 0x000B, 0xffff);
  channel_config_set_dreq(&c2, 0x3b);

  // Leave the data channel unchained so playback stops after one table.
  dma_channel_configure(
    data_chan,
    &c2,
    &spi_get_hw(SPI_PORT)->dr,
    DAC_data,
    sine_table_size,
    false
  );
}

// Trigger the collision sound.
static inline void play_thunk()
{
  dma_channel_start(ctrl_chan);
}

/* Board geometry */

#define NUM_ROWS     16
#define PEG_TOP_Y    40
#define PEG_X0       320
#define PEG_DX       38
#define PEG_DY       19
#define PEG_RADIUS   6
#define BALL_RADIUS  4

#define EXIT_Y \
  (PEG_TOP_Y + (NUM_ROWS - 1) * PEG_DY + PEG_RADIUS + BALL_RADIUS)

#define MAX_BALLS       54000
#define START_BALLS     54000
#define SPAWN_PER_FRAME 100

// Extra balls given to core 1 (core 0 also does the clear, pegs and text).
// Tune with the scope on GPIO 2 and 3.
#define CORE1_EXTRA     0

fix15 GRAVITY = float2fix15(0.37);
fix15 BOUNCINESS = float2fix15(0.4);
fix15 COLLIDE_DIST;

/* User interface */

#define ENC_A      27
#define ENC_B      28
#define ENC_SW     15
#define LED_PIN    25
#define CORE0_GPIO 2
#define CORE1_GPIO 3

#define BALL_STEP   10
#define BOUNCE_STEP float2fix15(0.05)

enum {
  MODE_BALLS,
  MODE_BOUNCE,
  NUM_MODES
};

int mode = MODE_BALLS;
int target_balls = START_BALLS;

volatile int encoder_delta = 0;

// On A's falling edge, B's level determines the direction.
void encoder_isr(uint gpio, uint32_t events)
{
  if (gpio_get(ENC_B))
    encoder_delta++;
  else
    encoder_delta--;
}

/* Histogram */

#define NUM_BINS     (NUM_ROWS + 1)
#define BOTTOM_PEG_X (PEG_X0 - (NUM_ROWS - 1) * (PEG_DX / 2))
#define HIST_TEXT_Y  340
#define HIST_TOP     350
#define HIST_BOTTOM  480
#define HIST_H       (HIST_BOTTOM - HIST_TOP)
#define BAR_W        32

// Each core updates its own counters.
int hist[2][NUM_BINS];
int fallen[2];

// Cached histogram labels, refreshed once per second.
static uint8_t hist_text_pixels[VGA_ROW_BYTES * (HIST_TOP - HIST_TEXT_Y)];
static uint32_t hist_text_updated = 0;
static bool hist_text_dirty = true;

// Called while core 1 is idle.
void resetHistogram()
{
  memset(hist, 0, sizeof(hist));
  fallen[0] = fallen[1] = 0;
  hist_text_dirty = true;
}

// Assign an exiting ball to one of the 17 bins.
static inline void recordBin(fix15 x, int core)
{
  int px = fix2int15(x);
  int bin;

  if (px < BOTTOM_PEG_X) {
    bin = 0;
  } else {
    bin = (px - BOTTOM_PEG_X) / PEG_DX + 1;

    if (bin > NUM_BINS - 1)
      bin = NUM_BINS - 1;
  }

  hist[core][bin]++;
  fallen[core]++;
}

// Update bars each frame and regenerate count labels once per second.
void drawHistogram()
{
  char cnt[12];
  int sum[NUM_BINS];
  int maxv = 0;

  for (int b = 0; b < NUM_BINS; b++) {
    sum[b] = hist[0][b] + hist[1][b];

    if (sum[b] > maxv)
      maxv = sum[b];
  }

  uint32_t now = time_us_32();
  char *text_area = current_draw_buffer + VGA_ROW_BYTES * HIST_TEXT_Y;

  if (hist_text_dirty ||
      (uint32_t)(now - hist_text_updated) >= 1000000u) {
    // Clear the label strip before rendering the new counts.
    memset(text_area, 0, sizeof(hist_text_pixels));

    for (int b = 0; b < NUM_BINS; b++) {
      snprintf(cnt, sizeof(cnt), "%d", sum[b]);

      drawTextAscii(
        b * PEG_DX + 2, HIST_TEXT_Y, cnt, WHITE, BLACK
      );
    }

    memcpy(hist_text_pixels, text_area, sizeof(hist_text_pixels));

    hist_text_updated = now;
    hist_text_dirty = false;
  } else {
    // Restore the cached labels after the frame clear.
    memcpy(text_area, hist_text_pixels, sizeof(hist_text_pixels));
  }

  if (maxv == 0)
    return;

  for (int b = 0; b < NUM_BINS; b++) {
    int h = (int)((int64_t)sum[b] * HIST_H / maxv);

    if (h > 0)
      drawRectOutlineFast(b * PEG_DX, HIST_BOTTOM - h, BAR_W, h);
  }
}

/* Balls */

// Eight bytes per ball; physics calculations use widened fix15 values.
typedef struct {
  int16_t x, y;     // 1/32 pixel, range -1024 to just below 1024.
  int16_t vx, vy;   // 1/512 pixel/frame, range -64 to just below 64.
} ball_t;

typedef struct {
  fix15 x, y, vx, vy;
} ball_state_t;

#define POS_LOAD(p)  ((fix15)(p) * 1024)
#define VEL_LOAD(v)  ((fix15)(v) * 64)

// Round to nearest when initializing a ball.
#define POS_STORE(p) ((int16_t)(((p) + (1 << 9)) >> 10))
#define VEL_STORE(v) ((int16_t)(((v) + (1 << 5)) >> 6))

#define POS_TO_PX(p) ((p) >> 5)

// Entries below active_count are live.
ball_t balls[MAX_BALLS];
int active_count = 0;

static uint32_t rng_state[2] = {
  0x12345678,
  0x9E3779B9
};

// Separate random state for each core.
static inline uint32_t rng(int core)
{
  uint32_t x = rng_state[core];

  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;

  return rng_state[core] = x;
}

// Drop a ball from the top with a small random horizontal velocity.
void spawnBall(int i, int core)
{
  ball_t *b = &balls[i];
  int32_t launch = (int32_t)(rng(core) & 0xffffu) - 32768;

  b->x = POS_STORE(int2fix15(PEG_X0));
  b->y = 0;
  b->vx = VEL_STORE(launch >> 2);
  b->vy = 0;
}

// Draw the triangular array of peg outlines.
void drawPegs()
{
  for (int r = 0; r < NUM_ROWS; r++) {
    int row_x0 = PEG_X0 - r * (PEG_DX / 2);
    int row_y = PEG_TOP_Y + r * PEG_DY;

    for (int k = 0; k <= r; k++)
      drawSprite(row_x0 + k * PEG_DX, row_y, peg_rows, PEG_RADIUS);
  }
}

// intermediate_term is 2x the impact speed, so this is 1 px/frame.
#define IMPACT_MIN int2fix15(2)

// Resolve an overlap with peg k in row r.
HOT void collidePeg(ball_state_t *s, int r, int k)
{
  fix15 peg_x = int2fix15(
    PEG_X0 - r * (PEG_DX / 2) + k * PEG_DX
  );
  fix15 peg_y = int2fix15(PEG_TOP_Y + r * PEG_DY);

  fix15 dx = s->x - peg_x;
  fix15 dy = s->y - peg_y;

  fix15 ax = absfix15(dx);
  fix15 ay = absfix15(dy);

  if (ax >= COLLIDE_DIST || ay >= COLLIDE_DIST)
    return;

  fix15 largest = (ax > ay) ? ax : ay;
  fix15 smallest = (ax > ay) ? ay : ax;

  // Alpha = 123/128, beta = 51/128.
  fix15 distance = (123 * largest + 51 * smallest) >> 7;

  if (distance == 0 || distance >= COLLIDE_DIST)
    return;

  // Share one 32-bit division between both normal components.
  uint32_t inv_distance = (1u << 30) / (uint32_t)distance;

  // Unsigned products preserve the full 32-bit magnitude.
  fix15 normal_x = (fix15)(
    ((uint32_t)ax * inv_distance) >> 15
  );
  fix15 normal_y = (fix15)(
    ((uint32_t)ay * inv_distance) >> 15
  );

  if (dx < 0)
    normal_x = -normal_x;

  if (dy < 0)
    normal_y = -normal_y;

  fix15 intermediate_term = -2 * (
    multfix15(normal_x, s->vx) +
    multfix15(normal_y, s->vy)
  );

  // The push-out distance is an integer number of pixels.
  s->x = peg_x + normal_x * (PEG_RADIUS + BALL_RADIUS + 1);
  s->y = peg_y + normal_y * (PEG_RADIUS + BALL_RADIUS + 1);

  if (intermediate_term > 0) {
      s->vx += multfix15(normal_x, intermediate_term);
      s->vy += multfix15(normal_y, intermediate_term);

      if (intermediate_term > IMPACT_MIN) {
        play_thunk();

        s->vx = multfix15(BOUNCINESS, s->vx);
        s->vy = multfix15(BOUNCINESS, s->vy);
      }
    }
}

// Advance one frame: 0 = active, 1 = bottom exit, 2 = screen escape.
HOT int updateBall(ball_state_t *s)
{
  if (s->y > int2fix15(EXIT_Y))
    return 1;

  s->x += s->vx;
  s->y += s->vy;

  // Return 2 for balls that leave the screen.
  if (s->x < 0 || s->x > int2fix15(639) ||
      s->y < 0 || s->y > int2fix15(479))
    return 2;

  // Nearest row only: rows are 19 px apart and the collision range is 10,
  // so a ball can't touch pegs in two rows at once. A graze in the 1 px
  // band between rows is caught one frame later.
  int px = fix2int15(s->x);
  int py = fix2int15(s->y);
  int r = (py - PEG_TOP_Y + PEG_DY / 2) / PEG_DY;

  if (r >= 0 && r < NUM_ROWS) {
    int rel = px - (PEG_X0 - r * (PEG_DX / 2)) + PEG_DX / 2;

    if (rel >= 0) {
      int k = rel / PEG_DX;

      if (k <= r)
        collidePeg(s, r, k);
    }
  }

  s->vy += GRAVITY;

  return 0;
}

// Poll the button and apply accumulated encoder movement once per frame.
void handleInput()
{
  static bool sw_prev = true;
  bool sw = gpio_get(ENC_SW);

  if (sw_prev && !sw)
    mode = (mode + 1) % NUM_MODES;

  sw_prev = sw;

  uint32_t irq = save_and_disable_interrupts();
  int d = encoder_delta;
  encoder_delta = 0;
  restore_interrupts(irq);

  if (d == 0)
    return;

  bool changed = false;

  if (mode == MODE_BALLS) {
    int n = target_balls + d * BALL_STEP;

    if (n < 0)
      n = 0;

    if (n > MAX_BALLS)
      n = MAX_BALLS;

    changed = (n != target_balls);
    target_balls = n;
  } else if (mode == MODE_BOUNCE) {
    fix15 v = BOUNCINESS + d * BOUNCE_STEP;

    if (v < 0)
      v = 0;

    if (v > int2fix15(1))
      v = int2fix15(1);

    changed = (v != BOUNCINESS);
    BOUNCINESS = v;
  }

  if (changed)
    resetHistogram();
}

// Update and draw each core's share of the balls (runs from RAM).
static void __not_in_flash_func(processBalls)(int start, int end, int core)
{
  for (int i = start; i < end; i++) {
    ball_t *b = &balls[i];

    ball_state_t s = {
      POS_LOAD(b->x), POS_LOAD(b->y),
      VEL_LOAD(b->vx), VEL_LOAD(b->vy)
    };

    int result = updateBall(&s);

    if (result != 0) {
      if (result == 1)
        recordBin(s.x, core);

      spawnBall(i, core);
      continue;
    } else {
      uint32_t r = rng(core);

      // Random rounding preserves small changes on average.
      b->x = (int16_t)(
        (s.x + (fix15)(r & 1023u)) >> 10
      );
      b->y = (int16_t)(
        (s.y + (fix15)((r >> 10) & 1023u)) >> 10
      );
      b->vx = (int16_t)(
        (s.vx + (fix15)((r >> 20) & 63u)) >> 6
      );
      b->vy = (int16_t)(
        (s.vy + (fix15)((r >> 26) & 63u)) >> 6
      );

      // Prevent exact-center lockups: 1/512 pixel per frame.
      if (b->vx == 0)
        b->vx = (r & 1u) ? 1 : -1;
    }

    drawSprite(POS_TO_PX(b->x), POS_TO_PX(b->y), ball_rows, BALL_RADIUS);
  }
}

/* Multicore animation */

semaphore_t sem_start, sem_done;
volatile int core1_start, core1_end;

// Core 1 processes its assigned range, then signals completion.
// GPIO 3 is high while it works.
static PT_THREAD(protothread_anim1(struct pt *pt))
{
  PT_BEGIN(pt);

  while (1) {
    PT_SEM_SDK_WAIT(pt, &sem_start);

    gpio_put(CORE1_GPIO, 1);
    processBalls(core1_start, core1_end, 1);
    gpio_put(CORE1_GPIO, 0);

    PT_SEM_SDK_SIGNAL(pt, &sem_done);
  }

  PT_END(pt);
}

// Start the worker thread on core 1.
void core1_main()
{
  pt_add_thread(protothread_anim1);
  pt_schedule_start;
}

// Core 0 prepares each frame and waits for both ball ranges to finish.
// GPIO 2 is high while core 0 works and drops while it waits on core 1.
static PT_THREAD(protothread_anim(struct pt *pt))
{
  PT_BEGIN(pt);

  // These values must survive protothread yields.
  static char buf[40];
  static uint32_t frame_us = 0;
  static uint32_t t_start;

  while (1) {
    PT_YIELD_UNTIL(pt, draw_start_signal());

    t_start = time_us_32();
    gpio_put(CORE0_GPIO, 1);

    handleInput();

    clearLowFrame(0, BLACK);
    drawPegs();
    drawHistogram();

    // Retire excess balls or gradually add new ones.
    if (active_count > target_balls)
      active_count = target_balls;

    for (int s = 0;
         s < SPAWN_PER_FRAME && active_count < target_balls;
         s++) {
      spawnBall(active_count, 0);
      active_count++;
    }

    // Core 0 takes [0, split), core 1 takes [split, active_count).
    int split = (active_count - CORE1_EXTRA) / 2;

    if (split < 0)
      split = 0;

    core1_start = split;
    core1_end = active_count;

    PT_SEM_SDK_SIGNAL(pt, &sem_start);

    processBalls(0, split, 0);

    gpio_put(CORE0_GPIO, 0);
    PT_SEM_SDK_WAIT(pt, &sem_done);
    gpio_put(CORE0_GPIO, 1);

    sprintf(buf, "Balls: %d", target_balls);
    drawTextAscii(20, 10, buf, WHITE, BLACK);

    sprintf(buf, "Bounciness: %.2f", fix2float15(BOUNCINESS));
    drawTextAscii(20, 20, buf, WHITE, BLACK);

    sprintf(buf, "Fallen: %d", fallen[0] + fallen[1]);
    drawTextAscii(20, 30, buf, WHITE, BLACK);

    sprintf(
      buf, "Time: %lu s",
      (unsigned long)(to_ms_since_boot(get_absolute_time()) / 1000)
    );
    drawTextAscii(20, 40, buf, WHITE, BLACK);

    sprintf(
      buf, "Mode: %s",
      (mode == MODE_BALLS) ? "BALLS" : "BOUNCINESS"
    );
    drawTextAscii(20, 50, buf, WHITE, BLACK);

    sprintf(buf, "Frame: %lu us", (unsigned long)frame_us);
    drawTextAscii(20, 60, buf, WHITE, BLACK);

    // Report the time spent computing this frame.
    frame_us = time_us_32() - t_start;
    gpio_put(LED_PIN, frame_us > 16667);
    gpio_put(CORE0_GPIO, 0);
  }

  PT_END(pt);
}

// Slow the flash clock before raising the system clock (runs from RAM).
static void __no_inline_not_in_flash_func(set_flash_divider)(uint div)
{
  uint32_t t = qmi_hw->m[0].timing;
  t = (t & ~QMI_M0_TIMING_CLKDIV_BITS) | (div << QMI_M0_TIMING_CLKDIV_LSB);
  qmi_hw->m[0].timing = t;
}

// Initialize hardware and start both animation threads.
int main()
{
  vreg_set_voltage(VREG_VOLTAGE_1_30);
  sleep_ms(10);
  set_flash_divider(4);
  set_sys_clock_khz(400000, true);
  stdio_init_all();
  initVGA();

  COLLIDE_DIST = int2fix15(PEG_RADIUS + BALL_RADIUS);

  buildRows(ball_rows, ball_hw, ball_edge, BALL_RADIUS);
  buildRows(peg_rows, peg_hw, peg_edge, PEG_RADIUS);

  init_audio_dma();

  gpio_init(ENC_A);
  gpio_set_dir(ENC_A, GPIO_IN);
  gpio_pull_up(ENC_A);

  gpio_init(ENC_B);
  gpio_set_dir(ENC_B, GPIO_IN);
  gpio_pull_up(ENC_B);

  gpio_init(ENC_SW);
  gpio_set_dir(ENC_SW, GPIO_IN);
  gpio_pull_up(ENC_SW);

  gpio_init(CORE0_GPIO);
  gpio_set_dir(CORE0_GPIO, GPIO_OUT);
  gpio_put(CORE0_GPIO, 0);

  gpio_init(CORE1_GPIO);
  gpio_set_dir(CORE1_GPIO, GPIO_OUT);
  gpio_put(CORE1_GPIO, 0);

  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);
  gpio_put(LED_PIN, 0);

  gpio_set_irq_enabled_with_callback(
    ENC_A,
    GPIO_IRQ_EDGE_FALL,
    true,
    &encoder_isr
  );

  sem_init(&sem_start, 0, 1);
  sem_init(&sem_done, 0, 1);

  multicore_reset_core1();
  multicore_launch_core1(&core1_main);

  pt_add_thread(protothread_anim);
  pt_schedule_start;
}