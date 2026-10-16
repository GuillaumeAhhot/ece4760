/**
 * 1-bit-per-pixel VGA driver
 * Adapted from Hunter Adams / Bruce Land's vga16_graphics_v3
 */

#include <stdint.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"

#include "hsync.pio.h"
#include "vsync.pio.h"
#include "rgb1.pio.h"

#include "vga1_graphics.h"
#include "font_ascii_characters.h"

#define H_ACTIVE   655              // active + front porch - 1
#define V_ACTIVE   479              // active lines - 1
#define RGB_ACTIVE 639              // pixels per line - 1

#define VGA_BUFFER_COUNT 38400      // 640 * 480 / 8

unsigned char vga_buffer_0[VGA_BUFFER_COUNT] ;
unsigned char vga_buffer_1[VGA_BUFFER_COUNT] ;

char *current_draw_buffer ;

// Tables the DMA ring cycles through to swap buffers every frame.
char *draw_buffer[4]            __attribute__ ((aligned (16))) ;
char *pointer_display_buffer[4] __attribute__ ((aligned (16))) ;
int   start_flag_array[4]       __attribute__ ((aligned (16))) ;

int start_flag = 0 ;

// Load the PIO programs and start the DMA ring that streams frames.
void initVGA()
{
  PIO pio = pio0 ;

  uint hsync_offset = pio_add_program(pio, &hsync_program) ;
  uint vsync_offset = pio_add_program(pio, &vsync_program) ;
  uint rgb_offset   = pio_add_program(pio, &rgb1_program) ;

  uint hsync_sm = 0 ;
  uint vsync_sm = 1 ;
  uint rgb_sm   = 2 ;
  pio_sm_claim(pio, hsync_sm) ;
  pio_sm_claim(pio, vsync_sm) ;
  pio_sm_claim(pio, rgb_sm) ;

  hsync_program_init(pio, hsync_sm, hsync_offset, HSYNC) ;
  vsync_program_init(pio, vsync_sm, vsync_offset, VSYNC) ;
  rgb1_program_init(pio, rgb_sm, rgb_offset, LO_GRN, RGB_ACTIVE) ;

  current_draw_buffer = (char *)vga_buffer_0 ;

  // Display one buffer while drawing the other.
  draw_buffer[0] = (char *)vga_buffer_1 ;
  draw_buffer[1] = (char *)vga_buffer_0 ;
  draw_buffer[2] = (char *)vga_buffer_1 ;
  draw_buffer[3] = (char *)vga_buffer_0 ;

  pointer_display_buffer[0] = (char *)vga_buffer_0 ;
  pointer_display_buffer[1] = (char *)vga_buffer_1 ;
  pointer_display_buffer[2] = (char *)vga_buffer_0 ;
  pointer_display_buffer[3] = (char *)vga_buffer_1 ;

  for (int i = 0; i < 4; i++)
    start_flag_array[i] = 1 ;

  int rgb_data_chan  = dma_claim_unused_channel(true) ;
  int set_disp_chan  = dma_claim_unused_channel(true) ;
  int set_draw_chan  = dma_claim_unused_channel(true) ;
  int set_start_chan = dma_claim_unused_channel(true) ;

  // Pixel bytes to the RGB state machine, paced by its FIFO.
  dma_channel_config c0 = dma_channel_get_default_config(rgb_data_chan) ;
  channel_config_set_transfer_data_size(&c0, DMA_SIZE_8) ;
  channel_config_set_read_increment(&c0, true) ;
  channel_config_set_write_increment(&c0, false) ;
  channel_config_set_dreq(&c0, DREQ_PIO0_TX2) ;
  channel_config_set_chain_to(&c0, set_disp_chan) ;

  dma_channel_configure(rgb_data_chan, &c0, &pio->txf[rgb_sm],
                        vga_buffer_0, VGA_BUFFER_COUNT, false) ;

  // Point the data channel at the next display buffer.
  dma_channel_config c1 = dma_channel_get_default_config(set_disp_chan) ;
  channel_config_set_transfer_data_size(&c1, DMA_SIZE_32) ;
  channel_config_set_read_increment(&c1, true) ;
  channel_config_set_write_increment(&c1, false) ;
  channel_config_set_chain_to(&c1, set_draw_chan) ;
  channel_config_set_ring(&c1, false, 4) ;

  dma_channel_configure(set_disp_chan, &c1,
                        &dma_hw->ch[rgb_data_chan].read_addr,
                        pointer_display_buffer, 1, false) ;

  // Update current_draw_buffer.
  c1 = dma_channel_get_default_config(set_draw_chan) ;
  channel_config_set_transfer_data_size(&c1, DMA_SIZE_32) ;
  channel_config_set_read_increment(&c1, true) ;
  channel_config_set_write_increment(&c1, false) ;
  channel_config_set_chain_to(&c1, set_start_chan) ;
  channel_config_set_ring(&c1, false, 4) ;

  dma_channel_configure(set_draw_chan, &c1, &current_draw_buffer,
                        draw_buffer, 1, false) ;

  // Raise start_flag for the drawing thread, then restart the data channel.
  c1 = dma_channel_get_default_config(set_start_chan) ;
  channel_config_set_transfer_data_size(&c1, DMA_SIZE_32) ;
  channel_config_set_read_increment(&c1, true) ;
  channel_config_set_write_increment(&c1, false) ;
  channel_config_set_chain_to(&c1, rgb_data_chan) ;
  channel_config_set_ring(&c1, false, 4) ;

  dma_channel_configure(set_start_chan, &c1, &start_flag,
                        start_flag_array, 1, false) ;

  pio_sm_put_blocking(pio, hsync_sm, H_ACTIVE) ;
  pio_sm_put_blocking(pio, vsync_sm, V_ACTIVE) ;

  pio_enable_sm_mask_in_sync(pio, (1u << hsync_sm) | (1u << vsync_sm) | (1u << rgb_sm)) ;
  dma_start_channel_mask(1u << rgb_data_chan) ;
}

// Returns 1 once per buffer swap, then clears the flag.
int draw_start_signal(void)
{
  if (start_flag == 1) {
    start_flag = 0 ;
    return 1 ;
  }
  return 0 ;
}

// Fill everything from row `top` down.
void clearLowFrame(short top, short c)
{
  memset(current_draw_buffer + VGA_ROW_BYTES * top, c ? 0xFF : 0x00,
         VGA_BUFFER_COUNT - VGA_ROW_BYTES * top) ;
}

void drawPixel(short x, short y, char color)
{
  if (x < 0 || x > 639 || y < 0 || y > 479) return ;

  char *p = current_draw_buffer + VGA_ROW_BYTES * y + (x >> 3) ;

  if (color) *p |=  (1 << (x & 7)) ;
  else       *p &= ~(1 << (x & 7)) ;
}

// Horizontal line of w pixels starting at x.
void drawHLine(int x, int y, int w, char color)
{
  if (y < 0 || y > 479 || w < 1) return ;
  if (x < 0) { w += x ; x = 0 ; }
  if (x + w > 640) w = 640 - x ;
  if (w < 1) return ;

  uint8_t *row = (uint8_t *)current_draw_buffer + VGA_ROW_BYTES * y ;
  int x1 = x + w - 1 ;
  int b0 = x >> 3 ;
  int b1 = x1 >> 3 ;
  uint8_t m0 = (uint8_t)(0xFF << (x & 7)) ;
  uint8_t m1 = (uint8_t)(0xFF >> (7 - (x1 & 7))) ;

  if (b0 == b1) m0 &= m1 ;

  if (color) {
    row[b0] |= m0 ;
    if (b1 != b0) {
      memset(row + b0 + 1, 0xFF, b1 - b0 - 1) ;
      row[b1] |= m1 ;
    }
  } else {
    row[b0] &= ~m0 ;
    if (b1 != b0) {
      memset(row + b0 + 1, 0x00, b1 - b0 - 1) ;
      row[b1] &= ~m1 ;
    }
  }
}

// Font rows store the leftmost pixel in bit 7; the framebuffer uses bit 0.
static inline uint8_t rev8(uint8_t b)
{
  b = (uint8_t)((b >> 4) | (b << 4)) ;
  b = (uint8_t)(((b & 0xCC) >> 2) | ((b & 0x33) << 2)) ;
  b = (uint8_t)(((b & 0xAA) >> 1) | ((b & 0x55) << 1)) ;
  return b ;
}

// 6x7 ASCII text. Only lit pixels are drawn; colors are ignored.
int drawTextAscii(short x, short y, char *str, char color, char bgcolor)
{
  if (x < 0 || y < 0 || y > 470) return 0 ;

  int count = 0 ;
  uint8_t *base = (uint8_t *)current_draw_buffer + VGA_ROW_BYTES * y ;

  while (*str) {
    if (x + 6 > 639) return count ;

    unsigned char c = (unsigned char)*str++ ;
    uint8_t *p = base + (x >> 3) ;
    int sh = x & 7 ;

    for (int i = 0; i < 7; i++, p += VGA_ROW_BYTES) {
      uint32_t m = (uint32_t)(rev8(asciifont[c * 7 + i]) & 0x3F) << sh ;
      p[0] |= (uint8_t)m ;
      p[1] |= (uint8_t)(m >> 8) ;
    }

    count++ ;
    x += 6 ;
  }
  return count ;
}