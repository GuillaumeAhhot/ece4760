/**
 * 1-bit-per-pixel 640x480 VGA driver (double buffered, 60 fps)
 * Adapted from Hunter Adams / Bruce Land's vga16_graphics_v3
 *
 * Rows are 80 bytes; bit 0 of each byte is the leftmost of its 8 pixels.
 * The pixel bit drives GPIO 19 (green high bit), so the display is
 * green on black. Any nonzero color draws a lit pixel.
 */

#ifndef VGA1_GRAPHICS_H
#define VGA1_GRAPHICS_H

enum vga_pins {HSYNC=16, VSYNC, LO_GRN, HI_GRN, BLUE_PIN, RED_PIN} ;

// Kept so existing code compiles; only zero vs nonzero matters.
enum colors {BLACK, DARK_GREEN, MED_GREEN, GREEN,
             DARK_BLUE, BLUE, LIGHT_BLUE, CYAN,
             RED, DARK_ORANGE, ORANGE, YELLOW,
             MAGENTA, PINK, LIGHT_PINK, WHITE} ;

#define VGA_ROW_BYTES 80

extern char *current_draw_buffer ;

void initVGA(void) ;
int  draw_start_signal(void) ;
void clearLowFrame(short top, short c) ;
void drawPixel(short x, short y, char color) ;
void drawHLine(int x, int y, int w, char color) ;
int  drawTextAscii(short x, short y, char *str, char color, char bgcolor) ;

#endif