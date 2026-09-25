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
 *  - GND ---> VGA-GND
 *
 * RESOURCES USED
 *  - PIO state machines 0, 1, 2 on PIO0
 *  - 4 DMA channels (VGA library) + 2 DMA channels (audio)
 *  - DMA pacing timer 0, SPI0
 *  - 153.6 kBytes of RAM (pixel color data)
 */

#include "VGA/vga16_graphics_v3.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/divider.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/spi.h"
#include "hardware/clocks.h"
#include "hardware/pll.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
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

// static to avoid a duplicate-symbol clash with the copy in vga16_graphics_v3.c
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
    // (X/Y)*sys_clk. At 150 MHz this is ~52.6 kHz. Recompute if you overclock.
    dma_timer_set_fraction(0, 0x0017, 0xffff) ;
    channel_config_set_dreq(&c2, 0x3b);
    // No chain_to on the data channel: that is what makes this one-shot
    // instead of the looping sine in the demo.

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
#define PEG_TOP_Y    40      // y of the top peg
#define PEG_X0       320     // x of the top peg
#define PEG_DX       38      // horizontal separation
#define PEG_DY       19      // vertical separation
#define PEG_RADIUS   6
#define BALL_RADIUS  4

#define MAX_BALLS    2000

fix15 GRAVITY = float2fix15(0.37) ;
fix15 BOUNCINESS = float2fix15(0.25) ;
fix15 COLLIDE_DIST ;

// === histogram ==================================================
// 17 bins: 15 gaps between the 16 bottom-row pegs, plus one outside each end.
// Bottom-row pegs sit at x = 35 + 38k, so bin b spans x = 38b - 3 .. 38b + 35
// and is centered at x = 38b + 16.
#define NUM_BINS     (NUM_ROWS + 1)
#define BOTTOM_PEG_X (PEG_X0 - (NUM_ROWS-1)*(PEG_DX/2))   // 35
#define HIST_TOP     345
#define HIST_BOTTOM  480
#define HIST_H       (HIST_BOTTOM - HIST_TOP)
#define BAR_W        32

int hist[NUM_BINS] ;
int hist_max = 0 ;
int total_fallen = 0 ;

static inline void recordBin(fix15 x)
{
  int px = fix2int15(x) ;
  int bin ;
  if (px < BOTTOM_PEG_X) bin = 0 ;
  else {
    bin = (px - BOTTOM_PEG_X) / PEG_DX + 1 ;
    if (bin > NUM_BINS - 1) bin = NUM_BINS - 1 ;
  }
  hist[bin]++ ;
  if (hist[bin] > hist_max) hist_max = hist[bin] ;
  total_fallen++ ;
}

void drawHistogram()
{
  if (hist_max == 0) return ;
  for (int b = 0; b < NUM_BINS; b++) {
    // tallest bin always fills the full space under the board
    int h = hist[b] * HIST_H / hist_max ;
    if (h > 0) fillRect(b*PEG_DX, HIST_BOTTOM - h, BAR_W, h, MED_GREEN) ;
  }
}

// === rotary encoder =============================================
#define ENC_A 27
#define ENC_B 28

volatile int encoder_count = 1000 ;   // target ball count, starts at 10

// One interrupt per detent: on A's falling edge, B's level gives direction.
void encoder_isr(uint gpio, uint32_t events)
{
  if (gpio_get(ENC_B)) { if (encoder_count < MAX_BALLS) encoder_count++ ; }
  else                 { if (encoder_count > 0)         encoder_count-- ; }
}

// === balls ======================================================
typedef struct {
  fix15 x, y, vx, vy ;
  int last_peg ;
} ball_t ;

// Packed array: balls[0 .. active_count-1] are live, everything above is unused.
ball_t balls[MAX_BALLS] ;
int active_count = 0 ;

void spawnBall(ball_t *b)
{
  b->x = int2fix15(PEG_X0) ;
  b->y = int2fix15(0) ;
  b->vx = (fix15)((rand() & 0xffff) - 32768) >> 2 ;
  b->vy = 0 ;
  b->last_peg = -1 ;
}

void drawPegs()
{
  for (int r = 0; r < NUM_ROWS; r++) {
    int row_x0 = PEG_X0 - r*(PEG_DX/2) ;
    int row_y  = PEG_TOP_Y + r*PEG_DY ;
    for (int k = 0; k <= r; k++) {
      fillCircle(row_x0 + k*PEG_DX, row_y, PEG_RADIUS, WHITE) ;
    }
  }
}

// Collide ball b against peg k of row r, if they overlap.
static void collidePeg(ball_t *b, int r, int k)
{
  fix15 peg_x = int2fix15(PEG_X0 - r*(PEG_DX/2) + k*PEG_DX) ;
  fix15 peg_y = int2fix15(PEG_TOP_Y + r*PEG_DY) ;

  fix15 dx = b->x - peg_x ;
  fix15 dy = b->y - peg_y ;

  if ((absfix15(dx) >= COLLIDE_DIST) || (absfix15(dy) >= COLLIDE_DIST)) return ;

  // distance in 1/256 px units, then back to fix15; avoids the whole-pixel
  // truncation (and divide-by-zero) of doing the sqrt in integer pixels
  int32_t dx8 = dx >> 7 ;
  int32_t dy8 = dy >> 7 ;
  fix15 distance = sqrt_i32(dx8*dx8 + dy8*dy8) << 7 ;

  if (distance == 0 || distance >= COLLIDE_DIST) return ;

  fix15 normal_x = divfix(dx, distance) ;
  fix15 normal_y = divfix(dy, distance) ;

  fix15 intermediate_term = -2 * (multfix15(normal_x, b->vx) + multfix15(normal_y, b->vy)) ;

  // push the ball back outside the collision radius so it can't stick
  b->x = peg_x + multfix15(normal_x, COLLIDE_DIST + int2fix15(1)) ;
  b->y = peg_y + multfix15(normal_y, COLLIDE_DIST + int2fix15(1)) ;

  // only reflect if the ball is moving into the peg
  if (intermediate_term > 0) {
    b->vx = b->vx + multfix15(normal_x, intermediate_term) ;
    b->vy = b->vy + multfix15(normal_y, intermediate_term) ;
  }

  // sound and energy loss only on a new peg
  int peg_id = r*NUM_ROWS + k ;
  if (b->last_peg != peg_id) {
    b->last_peg = peg_id ;
    play_thunk() ;
    b->vx = multfix15(BOUNCINESS, b->vx) ;
    b->vy = multfix15(BOUNCINESS, b->vy) ;
  }
}

// Returns 1 if the ball fell out the bottom this frame.
int updateBall(ball_t *b)
{
  // y increases downward, so falling off the bottom is a large y
  if (b->y > int2fix15(480)) return 1 ;

  b->x = b->x + b->vx ;
  b->y = b->y + b->vy ;

  // Instead of testing all 136 pegs, compute which pegs could be touching.
  // Rows are 19 px apart and the collision distance is 10 px, so only the
  // row above and below the ball can be in range. Within a row pegs are
  // 38 px apart, so only the nearest one can be in range.
  int px = fix2int15(b->x) ;
  int py = fix2int15(b->y) ;
  int r0 = (py - PEG_TOP_Y) / PEG_DY ;

  for (int r = r0; r <= r0 + 1; r++) {
    if (r < 0 || r >= NUM_ROWS) continue ;
    int rel = px - (PEG_X0 - r*(PEG_DX/2)) + PEG_DX/2 ;
    if (rel < 0) continue ;
    int k = rel / PEG_DX ;
    if (k > r) continue ;
    collidePeg(b, r, k) ;
  }

  // side and top walls
  if (b->x < int2fix15(BALL_RADIUS) && b->vx < 0)        b->vx = -b->vx ;
  if (b->x > int2fix15(639 - BALL_RADIUS) && b->vx > 0)  b->vx = -b->vx ;
  if (b->y < int2fix15(BALL_RADIUS) && b->vy < 0)        b->vy = -b->vy ;

  b->vy = b->vy + GRAVITY ;
  return 0 ;
}

#define ISR_GPIO 2

static PT_THREAD (protothread_anim(struct pt *pt))
{
    PT_BEGIN(pt);

    static char buf[40] ;

    while(1) {
      PT_YIELD_UNTIL(pt, draw_start_signal()) ;
      gpio_put(ISR_GPIO, 1);
      clearLowFrame(0, BLACK);
      drawPegs() ;
      drawHistogram() ;

      int target = encoder_count ;

      // count went down: highest-index balls retire immediately
      if (active_count > target) active_count = target ;

      // count went up: add one ball per frame so they don't stack
      if (active_count < target) {
        spawnBall(&balls[active_count]) ;
        active_count++ ;
      }

      for (int i = 0; i < active_count; i++) {
        ball_t *b = &balls[i] ;
        if (updateBall(b)) {
          recordBin(b->x) ;
          spawnBall(b) ;
        }
        fillCircle(fix2int15(b->x), fix2int15(b->y), BALL_RADIUS, CYAN) ;
      }

      sprintf(buf, "Balls: %d / %d", active_count, target) ;
      drawTextAscii(20, 20, buf, WHITE, BLACK);
      sprintf(buf, "Fallen: %d", total_fallen) ;
      drawTextAscii(20, 30, buf, WHITE, BLACK);
      sprintf(buf, "Time: %d s", to_ms_since_boot(get_absolute_time()) / 1000) ;
      drawTextAscii(20, 40, buf, WHITE, BLACK);
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

    gpio_init(ISR_GPIO);
    gpio_set_dir(ISR_GPIO, GPIO_OUT);
    gpio_put(ISR_GPIO, 0);

  gpio_set_irq_enabled_with_callback(ENC_A, GPIO_IRQ_EDGE_FALL, true, &encoder_isr) ;

  pt_add_thread(protothread_anim);

  pt_schedule_start ;
}