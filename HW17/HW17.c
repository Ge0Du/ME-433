/**
 * 3-Wheel Differential Drive with OV7670 Camera (D0–D7 on GPIO 17–24)
 * and NeoPixel Illumination on GPIO 16 (two LEDs), with NeoPixel driven on Core 1.
 *
 * No USB-CDC dependency: this code does not wait for stdio.
 *
 * Files in this project:
 *   • CMakeLists.txt      ← CMake build file (unchanged from earlier)
 *   • HW17.c              ← this main application (motors, camera, Core 1 spawn)
 *   • ov7670_regs.h       ← extern declarations for OV7670_init[] & OV7670_rgb[]
 *   • ov7670_regs.c       ← definitions of those arrays
 */

#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/multicore.h"
#include "ov7670_regs.h"   // ← extern declarations for OV7670_init[] & OV7670_rgb[]

// ──────────────────────────────────────────────────────────────────────────────
// Add these two prototypes so the compiler knows about them before use:
// ──────────────────────────────────────────────────────────────────────────────
void    OV7670_write_register(uint8_t reg, uint8_t value);
uint8_t OV7670_read_register(uint8_t reg);

// --------------------------------
// === CAMERA DEFINES & GLOBALS ===
// --------------------------------

// I²C for OV7670
#define I2C_PORT    i2c1
#define I2C_SDA     14
#define I2C_SCL     15

// Camera data pins on GP 17…24
#define D0 13
#define D1 16
#define D2 12
#define D3 17
#define D4 11
#define D5 18
#define D6 8
#define D7 19

// VSYNC / HSYNC / MCLK / PCLK / RST
#define VS   28     // VSYNC
#define HS   9     // HSYNC
#define MCLK 10    // 10 MHz clock via PWM
#define PCLK 27   // Pixel clock
#define RST  22    // Reset line (active LOW)

#define IMAGESIZEX 80
#define IMAGESIZEY 60

// Raw RGB565 buffer (2 bytes/pixel)
static volatile uint8_t cameraData[IMAGESIZEX * IMAGESIZEY * 2];

// Converted 8-bit R/G/B arrays
typedef struct {
    uint32_t index;
    uint8_t r[IMAGESIZEX * IMAGESIZEY];
    uint8_t g[IMAGESIZEX * IMAGESIZEY];
    uint8_t b[IMAGESIZEX * IMAGESIZEY];
} cameraImage_t;
static volatile cameraImage_t picture;

// IRQ-flags & counters
static volatile uint8_t  saveImage     = 0;    // 1 = capture requested
static volatile uint8_t  startImage    = 0;    // 1 = VSYNC seen
static volatile uint8_t  startCollect  = 0;    // 1 = HSYNC seen on valid row
static volatile uint32_t rawIndex      = 0;    // byte offset into cameraData
static volatile uint32_t hsCount       = 0;    // row count
static volatile uint32_t vsCount       = 0;    // pixel-clock ticks in a row

// Prototypes
void init_camera_pins();
void init_camera();
void setSaveImage(uint32_t s)       { saveImage = (uint8_t)s; }
uint32_t getSaveImage()             { return (uint32_t)saveImage; }
void convertImage();

// GPIO interrupt callback for VS, HS, PCLK
void gpio_callback(uint gpio, uint32_t events) {
    if (gpio == VS) {
        // VSYNC falling → start of a new frame
        if (saveImage) {
            rawIndex     = 0;
            hsCount      = 0;
            vsCount      = 0;
            startImage   = 1;
            startCollect = 0;
        }
    }
    else if (gpio == HS) {
        // HSYNC rising → start of a new row
        if (saveImage && startImage) {
            startCollect = 1;
            hsCount++;
            if (hsCount == IMAGESIZEY) {
                // Collected all rows → stop
                saveImage     = 0;
                startImage    = 0;
                startCollect  = 0;
                hsCount       = 0;
            }
        }
    }
    else if (gpio == PCLK) {
        // PCLK rising → sample one byte from D0..D7
        if (saveImage && startImage && startCollect) {
            vsCount++;
            uint8_t bit0 = gpio_get(D0);
            uint8_t bit1 = gpio_get(D1);
            uint8_t bit2 = gpio_get(D2);
            uint8_t bit3 = gpio_get(D3);
            uint8_t bit4 = gpio_get(D4);
            uint8_t bit5 = gpio_get(D5);
            uint8_t bit6 = gpio_get(D6);
            uint8_t bit7 = gpio_get(D7);
            uint8_t pixel_byte = (bit7 << 7) | (bit6 << 6) | (bit5 << 5) | (bit4 << 4)
                               | (bit3 << 3) | (bit2 << 2) | (bit1 << 1) |  bit0;
            cameraData[rawIndex++] = pixel_byte;

            // If buffer full, stop capturing
            if (rawIndex >= IMAGESIZEX * IMAGESIZEY * 2) {
                saveImage     = 0;
                startImage    = 0;
                startCollect  = 0;
            }
            // End of a row (2 bytes × IMAGESIZEX)
            if (vsCount == IMAGESIZEX * 2) {
                startCollect = 0;
                vsCount      = 0;
            }
        }
    }
}

// ----------------------------------
// === MOTOR & BUTTON DEFINES / CODE ===
// ----------------------------------

// Left motor (flipped wiring, DRV8833)
#define LEFT_IN1_PIN   0   // PWM slice 0, channel A
#define LEFT_IN2_PIN   1   // PWM slice 0, channel B

// Right motor (normal wiring, DRV8833)
#define RIGHT_IN1_PIN  2   // PWM slice 1, channel A
#define RIGHT_IN2_PIN  3   // PWM slice 1, channel B

// Button on GP 26 (active-LOW, pull-up)
#define BUTTON_PIN     26

static const uint16_t LEFT_WRAP  = 65535;  // ~1.9 kHz
static const uint16_t RIGHT_WRAP = 65535;  // ~1.9 kHz
#define DUTY_MIN   0
#define DUTY_MAX 100

// Drive left motor “forward physically” (swap channels because wiring is flipped)
static void set_left_motor(int duty) {
    uint slice = pwm_gpio_to_slice_num(LEFT_IN1_PIN);
    if (duty < DUTY_MIN) duty = DUTY_MIN;
    if (duty > DUTY_MAX) duty = DUTY_MAX;
    uint16_t level = (uint16_t)(((uint32_t)LEFT_WRAP * (uint32_t)duty) / 100);

    if (duty > 0) {
        // Flipped: IN1 = 0, IN2 = PWM
        pwm_set_chan_level(slice, PWM_CHAN_A, 0);      // LEFT_IN1_PIN
        pwm_set_chan_level(slice, PWM_CHAN_B, level);  // LEFT_IN2_PIN
    } else {
        // Stop/coast
        pwm_set_chan_level(slice, PWM_CHAN_A, 0);
        pwm_set_chan_level(slice, PWM_CHAN_B, 0);
    }
}

// Drive right motor normally (IN1 = PWM, IN2 = 0)
static void set_right_motor(int duty) {
    uint slice = pwm_gpio_to_slice_num(RIGHT_IN1_PIN);
    if (duty < DUTY_MIN) duty = DUTY_MIN;
    if (duty > DUTY_MAX) duty = DUTY_MAX;
    uint16_t level = (uint16_t)(((uint32_t)RIGHT_WRAP * (uint32_t)duty) / 100);

    if (duty > 0) {
        pwm_set_chan_level(slice, PWM_CHAN_A, level);  // RIGHT_IN1_PIN
        pwm_set_chan_level(slice, PWM_CHAN_B, 0);      // RIGHT_IN2_PIN
    } else {
        pwm_set_chan_level(slice, PWM_CHAN_A, 0);
        pwm_set_chan_level(slice, PWM_CHAN_B, 0);
    }
}

// ----------------------------------
// === WS2812B (NeoPixel) ON CORE 1 ===
// ----------------------------------

// NeoPixel data on GP 16 (two chained LEDs)
#define NEOPIXEL_PIN 16

// Timing constants (125 MHz clock ≈ 8 ns/cycle)
static const int T0H = 50;   // ~400 ns
static const int T0L = 106;  // ~850 ns
static const int T1H = 100;  // ~800 ns
static const int T1L = 56;   // ~450 ns

// Send one 24-bit GRB pixel to WS2812 (no interrupts disabled here).
static void put_pixel(uint32_t grb) {
    for (int8_t i = 23; i >= 0; i--) {
        if (grb & (1u << i)) {
            // '1' bit: ~800 ns HIGH, ~450 ns LOW
            gpio_put(NEOPIXEL_PIN, 1);
            for (int j = 0; j < T1H; j++) __asm__ volatile("nop");
            gpio_put(NEOPIXEL_PIN, 0);
            for (int j = 0; j < T1L; j++) __asm__ volatile("nop");
        } else {
            // '0' bit: ~400 ns HIGH, ~850 ns LOW
            gpio_put(NEOPIXEL_PIN, 1);
            for (int j = 0; j < T0H; j++) __asm__ volatile("nop");
            gpio_put(NEOPIXEL_PIN, 0);
            for (int j = 0; j < T0L; j++) __asm__ volatile("nop");
        }
    }
    // Latch: hold low for >50 μs
    gpio_put(NEOPIXEL_PIN, 0);
    sleep_us(60);
}

// Light two chained WS2812 LEDs solid white
static void set_two_pixels_white() {
    // GRB = {G=255, R=255, B=255} → 0xFF_FF_FF
    uint32_t white = (255u << 16) | (255u << 8) | 255u;
    put_pixel(white);
    put_pixel(white);
}

// This function will run on CORE 1
void core1_entry() {
    // 1) Initialize GP16 as NeoPixel data pin
    gpio_init(NEOPIXEL_PIN);
    gpio_set_dir(NEOPIXEL_PIN, GPIO_OUT);

    // 2) Immediately light both LEDs white
    set_two_pixels_white();

    // 3) Now just idle, or you could wait on FIFO for future color changes.
    while (1) {
        tight_loop_contents();
    }
}

// ----------------------------------
// === MAIN (runs on CORE 0) ===
// ----------------------------------

int main() {
    // 1) Launch Core 1 for NeoPixel
    multicore_launch_core1(core1_entry);

    // 2) Button = GP26, input + pull-up
    gpio_init(BUTTON_PIN);
    gpio_set_dir(BUTTON_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_PIN);

    // 3) Motor PWM initialization
    //    Left motor (flipped) on PWM slice 0
    gpio_set_function(LEFT_IN1_PIN, GPIO_FUNC_PWM);
    gpio_set_function(LEFT_IN2_PIN, GPIO_FUNC_PWM);
    {
        uint slice = pwm_gpio_to_slice_num(LEFT_IN1_PIN);
        pwm_set_wrap(slice, LEFT_WRAP);
        pwm_set_enabled(slice, true);
    }
    //    Right motor (normal) on PWM slice 1
    gpio_set_function(RIGHT_IN1_PIN, GPIO_FUNC_PWM);
    gpio_set_function(RIGHT_IN2_PIN, GPIO_FUNC_PWM);
    {
        uint slice = pwm_gpio_to_slice_num(RIGHT_IN1_PIN);
        pwm_set_wrap(slice, RIGHT_WRAP);
        pwm_set_enabled(slice, true);
    }

    // 4) Initialize camera pins + OV7670
    init_camera_pins();

    // 5) Main loop: toggle “running” via button, perform line-follow when running
    bool running = false;
    while (true) {
        // Button pressed? (active-LOW)
        if (gpio_get(BUTTON_PIN) == 0) {
            sleep_ms(20);  // debounce
            if (gpio_get(BUTTON_PIN) == 0) {
                running = !running;
                if (!running) {
                    // Stop motors when leaving line-follow mode
                    set_left_motor(0);
                    set_right_motor(0);
                }
                // Wait until button released
                while (gpio_get(BUTTON_PIN) == 0) {
                    sleep_ms(10);
                }
                sleep_ms(50);
            }
        }

        if (running) {
            // 6) Capture one frame
            setSaveImage(1);
            while (getSaveImage() == 1) {
                tight_loop_contents();
            }

            // 7) Convert raw RGB565 → 8-bit R/G/B
            convertImage();

            // 8) Analyze “non-white” pixel counts on left vs. right half
            const int white_threshold = 16;
            int left_nonwhite  = 0;
            int right_nonwhite = 0;
            for (uint32_t idx = 0; idx < IMAGESIZEX * IMAGESIZEY; idx++) {
                bool is_white = (picture.r[idx] >= white_threshold &&
                                 picture.g[idx] >= white_threshold &&
                                 picture.b[idx] >= white_threshold);
                if (!is_white) {
                    uint32_t x = idx % IMAGESIZEX;
                    if (x < (IMAGESIZEX / 2)) left_nonwhite++;
                    else                    right_nonwhite++;
                }
            }

            // 9) Steering logic
            int base_speed = 100;
            int turn_speed = 80;  // slow side to 80% duty for turning
            if (left_nonwhite > right_nonwhite + 20) {
                // Left side “off-line” → turn right
                set_left_motor(turn_speed);
                set_right_motor(base_speed);
            } else if (right_nonwhite > left_nonwhite + 20) {
                // Right side “off-line” → turn left
                set_left_motor(base_speed);
                set_right_motor(turn_speed);
            } else {
                // Both sides mostly white → drive straight
                set_left_motor(base_speed);
                set_right_motor(base_speed);
            }

            // ~10 FPS
            sleep_ms(100);
        } else {
            sleep_ms(10);
        }
    }
    return 0;
}

// -----------------------------
// Camera Pin + OV7670 Setup
// -----------------------------

void init_camera_pins() {
    // 1) Data pins D0..D7 = GP 17..24, all inputs
    for (int pin = D0; pin <= D7; pin++) {
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_IN);
    }

    // 2) RST = GP 12 as output; hold HIGH after reset
    gpio_init(RST);
    gpio_set_dir(RST, GPIO_OUT);
    gpio_put(RST, 1);

    // 3) MCLK = GP 10 → ~10 MHz PWM 
    gpio_set_function(MCLK, GPIO_FUNC_PWM);
    {
        uint slice = pwm_gpio_to_slice_num(MCLK);
        pwm_set_clkdiv(slice, 2.0f);   // divide by 2 → 62.5 MHz
        pwm_set_wrap(slice, 3);        // counter 0..3 → 4 steps → 15.625 MHz
        pwm_set_enabled(slice, true);
        pwm_set_chan_level(slice, PWM_CHAN_A, 2);  // 50% duty (2/4)
    }

    sleep_ms(50);

    // 4) Initialize I²C1 @100 kHz on GP 14/15
    i2c_init(I2C_PORT, 100 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // 5) Software + hardware reset + register writes
    init_camera();

    // 6) VSYNC = GP 8 (input, falling-edge triggers new frame)
    gpio_init(VS);
    gpio_set_dir(VS, GPIO_IN);
    gpio_set_irq_enabled_with_callback(VS, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    // 7) HSYNC = GP 9 (input, rising-edge triggers new row)
    gpio_init(HS);
    gpio_set_dir(HS, GPIO_IN);
    gpio_set_irq_enabled(HS, GPIO_IRQ_EDGE_RISE, true);

    // 8) PCLK = GP 11 (input, rising-edge samples data)
    gpio_init(PCLK);
    gpio_set_dir(PCLK, GPIO_IN);
    gpio_set_irq_enabled(PCLK, GPIO_IRQ_EDGE_RISE, true);
}

void init_camera() {
    // Hardware reset: pulse RST low
    gpio_put(RST, 0);
    sleep_ms(1);
    gpio_put(RST, 1);
    sleep_ms(50);

    // Software reset (COM7 = 0x80)
    OV7670_write_register(0x12, 0x80);
    sleep_ms(50);

    // PLL / clock config for 30 FPS
    OV7670_write_register(0x11, 1);  // CLKRC = 1 (divide by 1)
    OV7670_write_register(0x6B, 0);  // DBLV   = 0 (no PLL)

    // RGB565 format (COM7, RGB444, COM15)
    for (int i = 0; i < 3; i++) {
        OV7670_write_register(OV7670_rgb[i][0], OV7670_rgb[i][1]);
    }

    // Write all 93 registers from OV7670_init[][]
    for (int i = 0; i < 93; i++) {
        OV7670_write_register(OV7670_init[i][0], OV7670_init[i][1]);
    }
}

// I²C write to OV7670
void OV7670_write_register(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    i2c_write_blocking(I2C_PORT, 0x21, buf, 2, false);
    sleep_ms(1);
}

// Convert raw RGB565 → 8-bit R/G/B arrays
void convertImage() {
    picture.index = 0;
    for (int i = 0; i < IMAGESIZEX * IMAGESIZEY * 2; i += 2) {
        uint8_t hi = cameraData[i];
        uint8_t lo = cameraData[i + 1];
        // hi = RRRRRGGG, lo = GGGBBBBB
        picture.r[picture.index] = hi >> 3;                                // 5 bits → 0..31
        picture.g[picture.index] = ((hi & 0x07) << 3) | ((lo & 0xE0) >> 5); // 6 bits → 0..63
        picture.b[picture.index] = lo & 0x1F;                               // 5 bits → 0..31
        picture.index++;
    }
}
