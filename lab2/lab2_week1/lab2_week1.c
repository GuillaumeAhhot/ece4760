/**
 * ECE 4760 Lab 2, Week 1 - Digital Galton Board
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

// === rotary encoder =============================================
#define ENC_A 27
#define ENC_B 28

volatile int encoder_count = 0 ;

// One interrupt per detent: on A's falling edge, B's level gives direction.
void encoder_isr(uint gpio, uint32_t events)
{
  if (gpio_get(ENC_B)) encoder_count++ ;
  else                 encoder_count-- ;
}

// === Galton board parameters (Fig. 2) ===========================
fix15 GRAVITY = float2fix15(0.37) ;
fix15 BOUNCINESS = float2fix15(0.5) ;

#define PEG_RADIUS   6
#define BALL_RADIUS  4

fix15 COLLIDE_DIST ;

char color = WHITE ;

fix15 ball_x, ball_y, ball_vx, ball_vy ;
fix15 peg_x, peg_y ;
int last_peg ;

void spawnBall()
{
  ball_x = int2fix15(320) ;
  ball_y = int2fix15(0) ;
  ball_vx = (fix15)((rand() & 0xffff) - 32768) >> 2 ;
  ball_vy = 0 ;
  last_peg = -1 ;
}

void spawnPeg()
{
  peg_x = int2fix15(320) ;
  peg_y = int2fix15(240) ;
}

void updateBall(fix15* x, fix15* y, fix15* vx, fix15* vy)
{
  // y increases downward, so falling off the bottom is a large y
  if (*y > int2fix15(480)) {
    spawnBall() ;
    return ;
  }

  *x = *x + *vx ;
  *y = *y + *vy ;

  fix15 dx = *x - peg_x ;
  fix15 dy = *y - peg_y ;

  // bounding-box reject before the expensive distance calc
  if ((absfix15(dx) < COLLIDE_DIST) && (absfix15(dy) < COLLIDE_DIST)) {

    fix15 distance = int2fix15(sqrt_i32(fix2int15(multfix15(dx,dx)) + fix2int15(multfix15(dy,dy)))) ;

    if (distance < COLLIDE_DIST) {

      fix15 normal_x = divfix(dx, distance) ;
      fix15 normal_y = divfix(dy, distance) ;

      fix15 intermediate_term = -2 * (multfix15(normal_x, *vx) + multfix15(normal_y, *vy)) ;

      // push the ball back outside the collision radius so it can't stick
      *x = peg_x + multfix15(normal_x, COLLIDE_DIST + int2fix15(1)) ;
      *y = peg_y + multfix15(normal_y, COLLIDE_DIST + int2fix15(1)) ;

      // only reflect if the ball is moving into the peg
      if (intermediate_term > 0) {
        *vx = *vx + multfix15(normal_x, intermediate_term) ;
        *vy = *vy + multfix15(normal_y, intermediate_term) ;
      }

      // sound and energy loss fire once per contact, not every frame touching
      if (last_peg != 0) {
        last_peg = 0 ;
        play_thunk() ;
        *vx = multfix15(BOUNCINESS, *vx) ;
        *vy = multfix15(BOUNCINESS, *vy) ;
      }
    }
  }
  else {
    last_peg = -1 ;
  }

  *vy = *vy + GRAVITY ;
}

static PT_THREAD (protothread_anim(struct pt *pt))
{
    PT_BEGIN(pt);

    spawnPeg() ;
    spawnBall() ;

    while(1) {
      PT_YIELD_UNTIL(pt, draw_start_signal()) ;
      clearLowFrame(0, BLACK);
      updateBall(&ball_x, &ball_y, &ball_vx, &ball_vy) ;
      fillCircle(fix2int15(peg_x), fix2int15(peg_y), PEG_RADIUS, WHITE);
      fillCircle(fix2int15(ball_x), fix2int15(ball_y), BALL_RADIUS, color);

      static char buf[20] ;
      sprintf(buf, "Count: %d", encoder_count) ;
      drawTextAscii(20, 20, buf, WHITE, BLACK);
    }
  PT_END(pt);
}

int main(){
  set_sys_clock_khz(150000, true) ;
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

  gpio_set_irq_enabled_with_callback(ENC_A, GPIO_IRQ_EDGE_FALL, true, &encoder_isr) ;

  pt_add_thread(protothread_anim);

  pt_schedule_start ;
}