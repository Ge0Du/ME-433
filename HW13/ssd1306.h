#ifndef SSD1306_H
#define SSD1306_H

#include <stdint.h>
#include <stdbool.h>

// SSD1306 I2C address (typically 0x3C or 0x3D). Most 128×64 modules use 0x3C.
#define SSD1306_ADDR       0x3C

// Display size (in pixels)
#define SSD1306_WIDTH      128
#define SSD1306_HEIGHT     64

// Framebuffer size = WIDTH × HEIGHT/8 (since each byte holds 8 vertical pixels)
#define SSD1306_BUFFER_SIZE (SSD1306_WIDTH * (SSD1306_HEIGHT/8))

// Initialize the OLED
void ssd1306_init();

// Clear the buffer (does not push to display; call ssd1306_show() afterward)
void ssd1306_clear();

// Draw a single pixel (x ∈ [0,127], y ∈ [0,63], color = 0 or 1)
void ssd1306_draw_pixel(int x, int y, int color);

// Draw a line between (x0,y0) and (x1,y1), color = 0/1
void ssd1306_draw_line(int x0, int y0, int x1, int y1, int color);

// Send the framebuffer to the display
void ssd1306_show();

#endif // SSD1306_H
