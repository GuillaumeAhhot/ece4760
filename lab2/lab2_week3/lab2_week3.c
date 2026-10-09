/**
 * ECE 4760 Lab 2 - Digital Galton Board
 * Based on Hunter Adams (vha3@cornell.edu) VGA and DMA demos
 *
 * HARDWARE CONNECTIONS
 *  - GPIO 16 ---> VGA Hsync
 *  - GPIO 17 ---> VGA Vsync
 *  - GPIO 18 ---> VGA Green lo-bit --> 470 ohm resistor --> VGA_Green
 *  - GPIO 19 ---> VGA Green hi_bit --> 330 ohm resistor --> VGA_Green
 *  - GPIO 20 ---> 330 ohm resistor ---> VGA-Blue
 *  - GPIO 21 ---> 330 ohm resistor ---> VGA-Red
 *  - GPIO 4/5/6/7 ---> MCP4822 MISO/CS/SCK/SDI, VOUTA ---> amp
 *  - GPIO 27/28 ---> encoder A/B, encoder COM ---> GND
 *  - GPIO 15 ---> encoder switch, other switch pin ---> GND
 *  - GPIO 2  ---> scope (high while computing a frame)
 *  - GPIO 25 ---> onboard LED (on when the 60 fps deadline is missed)
 *  - GND ---> VGA-GND
 *
 * RESOURCES USED
 *  - PIO state machines 0, 1, 2 on PIO0
 *  - 4 DMA channels (VGA library) + 2 DMA channels (audio)
 *  - DMA pacing timer 0, SPI0
 *  - Both cores, 2 semaphores
 *  - 307.2 kB RAM (2 framebuffers) + 180 kB (ball array)
 */

#include "VGA/vga16_graphics_v3.h"
#include <stdio.h>
#include <stdlib.h>
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
#include "pt_cornell_rp2040_v1_4.h"

// === fixed point ================================================
typedef signed int fix15 ;
#define multfix15(a,b) ((fix15)((((signed long long)(a))*((signed long long)(b)))>>15))
#define float2fix15(a) ((fix15)((a)*32768.0))
#define fix2float15(a) ((float)(a)/32768.0)
#define absfix15(a) abs(a)
#define int2fix15(a) ((fix15)(a << 15))
#define fix2int15(a) ((int)(a >> 15))
#define divfix(a,b) (fix15)(div_s64s64( (((signed long long)(a)) << 15), ((signed long long)(b))))

// integer sqrt (static: the VGA library has its own copy)
static int32_t sqrt_i32(int32_t v) {
    uint32_t b = 1<<30, q = 0, r = v;
    while (b > r)
        b >>= 2;
    while( b > 0 ) {
        uint32_t t = q + b;
        q >>= 1;
        if( r >= t ) {
            r -= t;
            q += b;
        }
        b >>= 2;
    }
    return q;
}

// === drawing ====================================================
// disc half-widths per row, top to bottom (all widths even)
static const int8_t ball_hw[9]  = {2, 3, 4, 4, 4, 4, 4, 3, 2} ;
static const int8_t peg_hw[13]  = {2, 4, 5, 5, 6, 6, 6, 6, 6, 5, 5, 4, 2} ;

extern char *current_draw_buffer ;

// filled disc written straight into the framebuffer (2 px per byte)
static inline void drawDiscFast(int x, int y, const int8_t *hw, int r, char color)
{
  if (y < r || y > 479 - r || x < r || x > 639 - r) return ;
  char both = color | (color << 4) ;
  char *row = current_draw_buffer + 320*(y - r) ;

  for (int i = 0; i <= 2*r; i++, row += 320) {
    int x0 = x - hw[i] ;
    int n  = 2*hw[i] ;
    char *p = row + (x0 >> 1) ;
    if (x0 & 1) {                            // odd start: half bytes at both ends
      *p = (*p & 0x0F) | (color << 4) ; p++ ;
      for (int j = 0; j < (n-2)/2; j++) *p++ = both ;
      *p = (*p & 0xF0) | color ;
    } else {                                 // even start: whole bytes only
      for (int j = 0; j < n/2; j++) *p++ = both ;
    }
  }
}

// === audio ======================================================
#define sine_table_size 256

int raw_sin[sine_table_size] ;
unsigned short DAC_data[sine_table_size] ;
unsigned short * address_pointer = &DAC_data[0] ;

#define DAC_config_chan_A 0b0011000000000000

#define PIN_MISO 4
#define PIN_CS   5
#define PIN_SCK  6
#define PIN_MOSI 7
#define SPI_PORT spi0

int data_chan ;
int ctrl_chan ;

// SPI + two DMA channels: ctrl_chan rewinds data_chan, data_chan plays the table once
void init_audio_dma() {
    spi_init(SPI_PORT, 20000000) ;
    spi_set_format(SPI_PORT, 16, 0, 0, 0);

    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS,   GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    for (int i=0; i<sine_table_size; i++){
        raw_sin[i] = (int)(2047 * sin((float)i*6.283/(float)sine_table_size) + 2047);
        DAC_data[i] = DAC_config_chan_A | (raw_sin[i] & 0x0fff) ;
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
    dma_timer_set_fraction(0, 0x000B, 0xffff) ;   // ~50 kHz at 300 MHz
    channel_config_set_dreq(&c2, 0x3b);
    // no chain_to here: plays once instead of looping

    dma_channel_configure(
        data_chan,
        &c2,
        &spi_get_hw(SPI_PORT)->dr,
        DAC_data,
        sine_table_size,
        false
    );
}

static inline void play_thunk() {
    dma_channel_start(ctrl_chan) ;
}

// === board geometry (Fig. 2) ====================================
#define NUM_ROWS     16
#define PEG_TOP_Y    40
#define PEG_X0       320
#define PEG_DX       38
#define PEG_DY       19
#define PEG_RADIUS   6
#define BALL_RADIUS  4
#define EXIT_Y       (PEG_TOP_Y + (NUM_ROWS-1)*PEG_DY + PEG_RADIUS + BALL_RADIUS)

#define MAX_BALLS       20000
#define START_BALLS     10000
#define SPAWN_PER_FRAME 10

fix15 GRAVITY = float2fix15(0.37) ;
fix15 BOUNCINESS = float2fix15(0.3) ;
fix15 COLLIDE_DIST ;

// === user interface =============================================
#define ENC_A    27
#define ENC_B    28
#define ENC_SW   15
#define LED_PIN  25
#define ISR_GPIO 2

#define BALL_STEP    10
#define BOUNCE_STEP  float2fix15(0.05)

enum { MODE_BALLS, MODE_BOUNCE, NUM_MODES } ;
int mode = MODE_BALLS ;
int target_balls = START_BALLS ;

volatile int encoder_delta = 0 ;

// A falling edge: B's level gives direction
void encoder_isr(uint gpio, uint32_t events)
{
  if (gpio_get(ENC_B)) encoder_delta++ ;
  else                 encoder_delta-- ;
}

// === histogram ==================================================
// 17 bins: 15 gaps + one outside each end
#define NUM_BINS     (NUM_ROWS + 1)
#define BOTTOM_PEG_X (PEG_X0 - (NUM_ROWS-1)*(PEG_DX/2))
#define HIST_TEXT_Y  340
#define HIST_TOP     350
#define HIST_BOTTOM  480
#define HIST_H       (HIST_BOTTOM - HIST_TOP)
#define BAR_W        32

// one copy per core, summed when drawn
int hist[2][NUM_BINS] ;
int fallen[2] ;

void resetHistogram()
{
  memset(hist, 0, sizeof(hist)) ;
  fallen[0] = fallen[1] = 0 ;
}

// map exit x to a bin
static inline void recordBin(fix15 x, int core)
{
  int px = fix2int15(x) ;
  int bin ;
  if (px < BOTTOM_PEG_X) bin = 0 ;
  else {
    bin = (px - BOTTOM_PEG_X) / PEG_DX + 1 ;
    if (bin > NUM_BINS - 1) bin = NUM_BINS - 1 ;
  }
  hist[core][bin]++ ;
  fallen[core]++ ;
}

// counts + bars, tallest bar fills the area (core 1 is idle here)
void drawHistogram()
{
  static char cnt[8] ;
  int sum[NUM_BINS] ;
  int maxv = 0 ;
  for (int b = 0; b < NUM_BINS; b++) {
    sum[b] = hist[0][b] + hist[1][b] ;
    if (sum[b] > maxv) maxv = sum[b] ;
  }
  for (int b = 0; b < NUM_BINS; b++) {
    sprintf(cnt, "%d", sum[b]) ;
    drawTextAscii(b*PEG_DX + 2, HIST_TEXT_Y, cnt, WHITE, BLACK) ;
    if (maxv == 0) continue ;
    int h = sum[b] * HIST_H / maxv ;
    if (h > 0) fillRect(b*PEG_DX, HIST_BOTTOM - h, BAR_W, h, MED_GREEN) ;
  }
}

// === balls ======================================================
// stored 16-bit (8 B/ball), widened to fix15 for physics
typedef struct {
  int16_t x, y ;     // Q10.5: 1/32 px, +-1024 px
  int16_t vx, vy ;   // Q6.9: 1/512 px/frame, +-64 px/frame
} ball_t ;

typedef struct {
  fix15 x, y, vx, vy ;
} ball_state_t ;

// stores round to nearest (a plain shift biases balls left/up)
#define POS_LOAD(p)   (((fix15)(p)) << 10)
#define POS_STORE(p)  ((int16_t)(((p) + (1 << 9)) >> 10))
#define VEL_LOAD(v)   (((fix15)(v)) << 6)
#define VEL_STORE(v)  ((int16_t)(((v) + (1 << 5)) >> 6))
#define POS_TO_PX(p)  ((p) >> 5)

#define NO_PEG 255   // peg ids are 0..135

// balls[0 .. active_count-1] are live
ball_t  balls[MAX_BALLS] ;
uint8_t last_peg[MAX_BALLS] ;
int active_count = 0 ;

// xorshift32, one state per core
static uint32_t rng_state[2] = {0x12345678, 0x9E3779B9} ;

static inline uint32_t rng(int core)
{
  uint32_t x = rng_state[core] ;
  x ^= x << 13 ; x ^= x >> 17 ; x ^= x << 5 ;
  return rng_state[core] = x ;
}

// drop from the top with a small random vx
void spawnBall(int i, int core)
{
  ball_t *b = &balls[i] ;
  b->x  = POS_STORE(int2fix15(PEG_X0)) ;
  b->y  = 0 ;
  b->vx = VEL_STORE((fix15)((rng(core) & 0xffff) - 32768) >> 2) ;
  b->vy = 0 ;
  last_peg[i] = NO_PEG ;
}

void drawPegs()
{
  for (int r = 0; r < NUM_ROWS; r++) {
    int row_x0 = PEG_X0 - r*(PEG_DX/2) ;
    int row_y  = PEG_TOP_Y + r*PEG_DY ;
    for (int k = 0; k <= r; k++) {
      drawDiscFast(row_x0 + k*PEG_DX, row_y, peg_hw, PEG_RADIUS, WHITE) ;
    }
  }
}

// bounce ball s off peg k of row r if they overlap
static void collidePeg(ball_state_t *s, uint8_t *lp, int r, int k)
{
  fix15 peg_x = int2fix15(PEG_X0 - r*(PEG_DX/2) + k*PEG_DX) ;
  fix15 peg_y = int2fix15(PEG_TOP_Y + r*PEG_DY) ;

  fix15 dx = s->x - peg_x ;
  fix15 dy = s->y - peg_y ;

  if ((absfix15(dx) >= COLLIDE_DIST) || (absfix15(dy) >= COLLIDE_DIST)) return ;

  // distance in 1/256 px to avoid truncation and divide-by-zero
  int32_t dx8 = dx >> 7 ;
  int32_t dy8 = dy >> 7 ;
  fix15 distance = sqrt_i32(dx8*dx8 + dy8*dy8) << 7 ;

  if (distance == 0 || distance >= COLLIDE_DIST) return ;

  fix15 normal_x = divfix(dx, distance) ;
  fix15 normal_y = divfix(dy, distance) ;

  fix15 intermediate_term = -2 * (multfix15(normal_x, s->vx) + multfix15(normal_y, s->vy)) ;

  // move just outside the peg
  s->x = peg_x + multfix15(normal_x, COLLIDE_DIST + int2fix15(1)) ;
  s->y = peg_y + multfix15(normal_y, COLLIDE_DIST + int2fix15(1)) ;

  // reflect only if moving into the peg
  if (intermediate_term > 0) {
    s->vx = s->vx + multfix15(normal_x, intermediate_term) ;
    s->vy = s->vy + multfix15(normal_y, intermediate_term) ;
  }

  // thunk + energy loss once per new peg
  uint8_t peg_id = r*(r+1)/2 + k ;
  if (*lp != peg_id) {
    *lp = peg_id ;
    play_thunk() ;
    s->vx = multfix15(BOUNCINESS, s->vx) ;
    s->vy = multfix15(BOUNCINESS, s->vy) ;
  }
}

// one physics step; returns 1 if the ball cleared the bottom row
int updateBall(ball_state_t *s, uint8_t *lp)
{
  if (s->y > int2fix15(EXIT_Y)) return 1 ;

  s->x = s->x + s->vx ;
  s->y = s->y + s->vy ;

  // only the nearest peg in the 2 nearest rows can be in range
  int px = fix2int15(s->x) ;
  int py = fix2int15(s->y) ;
  int r0 = (py - PEG_TOP_Y) / PEG_DY ;

  for (int r = r0; r <= r0 + 1; r++) {
    if (r < 0 || r >= NUM_ROWS) continue ;
    int rel = px - (PEG_X0 - r*(PEG_DX/2)) + PEG_DX/2 ;
    if (rel < 0) continue ;
    int k = rel / PEG_DX ;
    if (k > r) continue ;
    collidePeg(s, lp, r, k) ;
  }

  // walls
  if (s->x < int2fix15(BALL_RADIUS) && s->vx < 0)        s->vx = -s->vx ;
  if (s->x > int2fix15(639 - BALL_RADIUS) && s->vx > 0)  s->vx = -s->vx ;
  if (s->y < int2fix15(BALL_RADIUS) && s->vy < 0)        s->vy = -s->vy ;

  s->vy = s->vy + GRAVITY ;
  return 0 ;
}

// button cycles mode, encoder adjusts the selected parameter (polled per frame)
void handleInput()
{
  static bool sw_prev = true ;
  bool sw = gpio_get(ENC_SW) ;
  if (sw_prev && !sw) mode = (mode + 1) % NUM_MODES ;
  sw_prev = sw ;

  uint32_t irq = save_and_disable_interrupts() ;
  int d = encoder_delta ;
  encoder_delta = 0 ;
  restore_interrupts(irq) ;
  if (d == 0) return ;

  bool changed = false ;

  if (mode == MODE_BALLS) {
    int n = target_balls + d*BALL_STEP ;
    if (n < 0) n = 0 ;
    if (n > MAX_BALLS) n = MAX_BALLS ;
    changed = (n != target_balls) ;
    target_balls = n ;
  }
  else if (mode == MODE_BOUNCE) {
    fix15 v = BOUNCINESS + d*BOUNCE_STEP ;
    if (v < 0) v = 0 ;
    if (v > int2fix15(1)) v = int2fix15(1) ;
    changed = (v != BOUNCINESS) ;
    BOUNCINESS = v ;
  }

  if (changed) resetHistogram() ;
}

// update + draw balls [start, end)
static void processBalls(int start, int end, int core)
{
  for (int i = start; i < end; i++) {
    ball_t *b = &balls[i] ;
    ball_state_t s = { POS_LOAD(b->x), POS_LOAD(b->y),
                       VEL_LOAD(b->vx), VEL_LOAD(b->vy) } ;

    if (updateBall(&s, &last_peg[i])) {
      recordBin(s.x, core) ;
      spawnBall(i, core) ;
    } else {
      b->x  = POS_STORE(s.x) ;
      b->y  = POS_STORE(s.y) ;
      b->vx = VEL_STORE(s.vx) ;
      b->vy = VEL_STORE(s.vy) ;
    }
    drawDiscFast(POS_TO_PX(b->x), POS_TO_PX(b->y), ball_hw, BALL_RADIUS, CYAN) ;
  }
}

// === multicore ==================================================
semaphore_t sem_start, sem_done ;
volatile int core1_start, core1_end ;

// core 1: wait for go, do the upper half of the balls, signal done
static PT_THREAD (protothread_anim1(struct pt *pt))
{
    PT_BEGIN(pt);
    while(1) {
      PT_SEM_SDK_WAIT(pt, &sem_start) ;
      processBalls(core1_start, core1_end, 1) ;
      PT_SEM_SDK_SIGNAL(pt, &sem_done) ;
    }
    PT_END(pt);
}

void core1_main()
{
  pt_add_thread(protothread_anim1) ;
  pt_schedule_start ;
}

// core 0: one frame per VGA buffer swap
static PT_THREAD (protothread_anim(struct pt *pt))
{
    PT_BEGIN(pt);

    // static: locals don't survive a protothread yield
    static char buf[40] ;
    static uint32_t frame_us = 0 ;
    static uint32_t t_start ;

    while(1) {
      PT_YIELD_UNTIL(pt, draw_start_signal()) ;
      t_start = time_us_32() ;
      gpio_put(ISR_GPIO, 1);

      handleInput() ;

      clearLowFrame(0, BLACK);
      drawPegs() ;
      drawHistogram() ;

      // match active count to target
      if (active_count > target_balls) active_count = target_balls ;
      for (int s = 0; s < SPAWN_PER_FRAME && active_count < target_balls; s++) {
        spawnBall(active_count, 0) ;
        active_count++ ;
      }

      // split balls between cores
      int half = active_count / 2 ;
      core1_start = half ;
      core1_end   = active_count ;
      PT_SEM_SDK_SIGNAL(pt, &sem_start) ;
      processBalls(0, half, 0) ;
      PT_SEM_SDK_WAIT(pt, &sem_done) ;

      // stats
      sprintf(buf, "Balls: %d", target_balls) ;
      drawTextAscii(20, 10, buf, WHITE, BLACK);
      sprintf(buf, "Bounciness: %.2f", fix2float15(BOUNCINESS)) ;
      drawTextAscii(20, 20, buf, WHITE, BLACK);
      sprintf(buf, "Fallen: %d", fallen[0] + fallen[1]) ;
      drawTextAscii(20, 30, buf, WHITE, BLACK);
      sprintf(buf, "Time: %d s", to_ms_since_boot(get_absolute_time()) / 1000) ;
      drawTextAscii(20, 40, buf, WHITE, BLACK);
      sprintf(buf, "Mode: %s", (mode == MODE_BALLS) ? "BALLS" : "BOUNCINESS") ;
      drawTextAscii(20, 50, buf, WHITE, BLACK);
      sprintf(buf, "Frame: %lu us", frame_us) ;
      drawTextAscii(20, 60, buf, WHITE, BLACK);

      // LED on if the 60 fps deadline was missed
      frame_us = time_us_32() - t_start ;
      gpio_put(LED_PIN, frame_us > 16667) ;
      gpio_put(ISR_GPIO, 0);
    }
  PT_END(pt);
}

int main(){
  set_sys_clock_khz(300000, true) ;
  stdio_init_all() ;
  initVGA() ;

  COLLIDE_DIST = int2fix15(PEG_RADIUS + BALL_RADIUS) ;

  init_audio_dma() ;

  gpio_init(ENC_A) ;
  gpio_set_dir(ENC_A, GPIO_IN) ;
  gpio_pull_up(ENC_A) ;

  gpio_init(ENC_B) ;
  gpio_set_dir(ENC_B, GPIO_IN) ;
  gpio_pull_up(ENC_B) ;

  gpio_init(ENC_SW) ;
  gpio_set_dir(ENC_SW, GPIO_IN) ;
  gpio_pull_up(ENC_SW) ;

  gpio_init(ISR_GPIO) ;
  gpio_set_dir(ISR_GPIO, GPIO_OUT) ;
  gpio_put(ISR_GPIO, 0) ;

  gpio_init(LED_PIN) ;
  gpio_set_dir(LED_PIN, GPIO_OUT) ;
  gpio_put(LED_PIN, 0) ;

  gpio_set_irq_enabled_with_callback(ENC_A, GPIO_IRQ_EDGE_FALL, true, &encoder_isr) ;

  // start core 1
  sem_init(&sem_start, 0, 1) ;
  sem_init(&sem_done, 0, 1) ;
  multicore_reset_core1() ;
  multicore_launch_core1(&core1_main) ;

  pt_add_thread(protothread_anim);
  pt_schedule_start ;
}