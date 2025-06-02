#include "ssd1306.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include <stdlib.h>

#define I2C_PORT    i2c0

static uint8_t buffer[SSD1306_BUFFER_SIZE];


static void ssd1306_command_list(const uint8_t *cmds, size_t len) {
    uint8_t buf[len+1];
    buf[0] = 0x00;
    for (size_t i = 0; i < len; i++) {
        buf[i+1] = cmds[i];
    }
    i2c_write_blocking(I2C_PORT, SSD1306_ADDR, buf, (int)(len+1), false);
}

void ssd1306_init() {
    sleep_ms(100);

    const uint8_t init_seq[] = {
        0xAE,                // DISPLAY_OFF
        0xD5, 0x80,          // SET_DISPLAY_CLOCK_DIV, the suggested ratio 0x80
        0xA8, 0x3F,          // SET_MULTIPLEX, 0x3F = 63 → height=64
        0xD3, 0x00,          // SET_DISPLAY_OFFSET, no offset
        0x40,                // SET_START_LINE to line 0
        0x8D, 0x14,          // CHARGE_PUMP: enable (0x10=disable, 0x14=enable)
        0x20, 0x00,          // MEMORY_MODE: 0x00 = horizontal addressing
        0xA1,                // SEGMENT_REMAP: column address 127 is mapped to SEG0
        0xC8,                // COM_SCAN_DEC: scan from COM[N–1] to COM0
        0xDA, 0x12,          // SET_COM_PINS: config for 128×64 (0x12)
        0x81, 0xCF,          // SET_CONTRAST: 0xCF
        0xD9, 0xF1,          // SET_PRECHARGE: pre‐charge period
        0xDB, 0x40,          // SET_VCOM_DETECT: ~0.83 × VCC
        0xA4,                // DISPLAY_ALL_ON_RESUME: resume RAM content
        0xA6,                // NORMAL_DISPLAY (A7=invert)
        0xAF                 // DISPLAY_ON
    };
    ssd1306_command_list(init_seq, sizeof(init_seq));

    // Clear buffer and push to display
    ssd1306_clear();
    ssd1306_show();
}

void ssd1306_clear() {
    for (int i = 0; i < SSD1306_BUFFER_SIZE; i++) {
        buffer[i] = 0x00;
    }
}

// Set/clear a pixel in the framebuffer
void ssd1306_draw_pixel(int x, int y, int color) {
    if (x < 0 || x >= SSD1306_WIDTH || y < 0 || y >= SSD1306_HEIGHT) return;
    int page = y / 8;             // which byte row
    int bit_pos = y & 0x07;       // which bit within that byte
    int index = x + (page * SSD1306_WIDTH);
    if (color) {
        buffer[index] |=  (1 << bit_pos);
    } else {
        buffer[index] &= ~(1 << bit_pos);
    }
}

// Simple Bresenham’s line algorithm (8‐bit coords)
void ssd1306_draw_line(int x0, int y0, int x1, int y1, int color) {
    int dx =  abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy, e2;

    while (true) {
        ssd1306_draw_pixel(x0, y0, color);
        if (x0 == x1 && y0 == y1) break;
        e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y0 += sy;
        }
    }
}

// Send the entire framebuffer to the SSD1306 (horizontal addressing).
void ssd1306_show() {
    // We’ll write in “pages” of 8 vertical pixels at a time. There are 8 pages (64/8).
    for (uint8_t page = 0; page < 8; page++) {
        uint8_t cmd_seq[] = {
            (uint8_t)(0xB0 | page),    // Set page address (B0h to B7h)
            0x00,                      // Set lower column address = 0
            0x10                       // Set higher column address = 0
        };
        ssd1306_command_list(cmd_seq, 3);

        // Now send 128 bytes of data for this page:
        uint8_t buf[1 + SSD1306_WIDTH];
        buf[0] = 0x40;  // Control byte = 0x40 means “next bytes are data”
        // Copy one page of data
        for (int i = 0; i < SSD1306_WIDTH; i++) {
            buf[1 + i] = buffer[page * SSD1306_WIDTH + i];
        }
        i2c_write_blocking(I2C_PORT, SSD1306_ADDR, buf, (int)(1 + SSD1306_WIDTH), false);
    }
}
