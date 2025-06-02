/**
 * 3-Wheel Differential Drive with OV7670 Camera (D0–D7 on GPIO 13,16,12,17,11,18,8,19)
 * and NeoPixel Illumination on GPIO 27 (two LEDs), NeoPixel driven on Core 1.
 *
 * This version includes:
 *   • USB‐CDC stdio initialization + wait‐for‐host at startup
 *   • printf()‐based debug messages sprinkled throughout
 *   • HSYNC/PCLK interrupts enabled only during frame capture to avoid spurious IRQ floods
 *
 * Project files:
 *   • CMakeLists.txt      ← must link pico_stdio_usb, pico_multicore, hardware_pwm, hardware_i2c, hardware_gpio
 *   • HW18.c              ← this file (motors, camera, Core 1 NeoPixel, debug prints)
 *   • ov7670_regs.h       ← extern declarations for OV7670_init[][] & OV7670_rgb[][]
 *   • ov7670_regs.c       ← definitions of those arrays
 */

#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"       // for stdio_init_all(), stdio_usb_connected()
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/multicore.h"

#include "ov7670_regs.h"

// ──────────────────────────────────────────────────────────────────────────────
// Prototypes for camera I2C functions (declared here so compiler knows them):
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

// Camera data pins on GP13, GP16, GP12, GP17, GP11, GP18, GP8, GP19:
#define D0 13
#define D1 16
#define D2 12
#define D3 17
#define D4 11
#define D5 18
#define D6 8
#define D7 19

// VSYNC / HSYNC / MCLK / PCLK / RST / PWDN
#define VS     28    // VSYNC (falling-edge starts new frame)
#define HS     9     // HSYNC (rising-edge starts new row)
#define MCLK   10    // generate ~15.6 MHz PWM for camera clock
#define PCLK   27    // Pixel clock from camera
#define RST    22    // Reset line (active LOW)
#define PWDN   4     // Power-down line (driven low → camera on; driven high → camera off)

// Sensor geometry
#define IMAGESIZEX 80
#define IMAGESIZEY 60

// Raw RGB565 buffer (2 bytes per pixel)
static volatile uint8_t cameraData[IMAGESIZEX * IMAGESIZEY * 2];

// Converted 8-bit R/G/B arrays
typedef struct {
    uint32_t index;
    uint8_t  r[IMAGESIZEX * IMAGESIZEY];
    uint8_t  g[IMAGESIZEX * IMAGESIZEY];
    uint8_t  b[IMAGESIZEX * IMAGESIZEY];
} cameraImage_t;
static volatile cameraImage_t picture;

// IRQ-flags & counters
static volatile uint8_t  saveImage     = 0;    // 1 = capture requested
static volatile uint8_t  startImage    = 0;    // 1 = in a frame (VSYNC seen)
static volatile uint8_t  startCollect  = 0;    // 1 = in a valid row (HSYNC seen)
static volatile uint32_t rawIndex      = 0;    // byte offset into cameraData
static volatile uint32_t hsCount       = 0;    // how many rows collected so far
static volatile uint32_t vsCount       = 0;    // how many pixel-clock ticks in current row

// Prototypes
void init_camera_pins();
void init_camera();
void setSaveImage(uint32_t s)       { saveImage = (uint8_t)s; }
uint32_t getSaveImage()             { return (uint32_t)saveImage; }
void convertImage();

// GPIO interrupt callback for VS, HS, and PCLK
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
        if (!(saveImage && startImage)) return;
        startCollect = 1;
        hsCount++;
        if (hsCount == IMAGESIZEY) {
            // Collected all rows → end capture
            saveImage     = 0;
            startImage    = 0;
            startCollect  = 0;
            hsCount       = 0;
        }
    }
    else if (gpio == PCLK) {
        // PCLK rising → sample one byte on D0..D7
        if (!(saveImage && startImage && startCollect)) return;
        vsCount++;
        // read each data bit individually:
        uint8_t bit0 = gpio_get(D0);
        uint8_t bit1 = gpio_get(D1);
        uint8_t bit2 = gpio_get(D2);
        uint8_t bit3 = gpio_get(D3);
        uint8_t bit4 = gpio_get(D4);
        uint8_t bit5 = gpio_get(D5);
        uint8_t bit6 = gpio_get(D6);
        uint8_t bit7 = gpio_get(D7);
        uint8_t pixel_byte = (bit7 << 7)
                           | (bit6 << 6)
                           | (bit5 << 5)
                           | (bit4 << 4)
                           | (bit3 << 3)
                           | (bit2 << 2)
                           | (bit1 << 1)
                           |  bit0;
        cameraData[rawIndex++] = pixel_byte;

        // If buffer is full, stop capture
        if (rawIndex >= IMAGESIZEX * IMAGESIZEY * 2) {
            saveImage     = 0;
            startImage    = 0;
            startCollect  = 0;
        }
        // End of a row (2 bytes per pixel × IMAGESIZEX)
        if (vsCount == IMAGESIZEX * 2) {
            startCollect = 0;
            vsCount      = 0;
        }
    }
}


// ----------------------------------
// === MOTOR & BUTTON DEFINES / CODE ===
// ----------------------------------

// Left motor (wiring is flipped on DRV8833)
#define LEFT_IN1_PIN   0    // PWM slice 0, channel A
#define LEFT_IN2_PIN   1    // PWM slice 0, channel B

// Right motor (normal DRV8833 wiring)
#define RIGHT_IN1_PIN  2    // PWM slice 1, channel A
#define RIGHT_IN2_PIN  3    // PWM slice 1, channel B

// Button on GP20 (active-LOW input with pull-up)
#define BUTTON_PIN     20

static const uint16_t LEFT_WRAP  = 65535;   // ~1.9 kHz
static const uint16_t RIGHT_WRAP = 65535;   // ~1.9 kHz
#define DUTY_MIN   0
#define DUTY_MAX 100

// Drive left motor “forward physically” (swap channels because wiring is flipped)
static void set_left_motor(int duty) {
    uint slice = pwm_gpio_to_slice_num(LEFT_IN1_PIN);
    if (duty < DUTY_MIN) duty = DUTY_MIN;
    if (duty > DUTY_MAX) duty = DUTY_MAX;
    uint16_t level = (uint16_t)(((uint32_t)LEFT_WRAP * (uint32_t)duty) / 100);

    if (duty > 0) {
        // Flipped hardware: IN1 = 0, IN2 = PWM
        pwm_set_chan_level(slice, PWM_CHAN_A, 0);      // LEFT_IN1_PIN
        pwm_set_chan_level(slice, PWM_CHAN_B, level);  // LEFT_IN2_PIN
    } else {
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

// NeoPixel data pin on GP27 (two chained WS2812B LEDs)
#define NEOPIXEL_PIN 27

// Timing constants (125 MHz clock ≈ 8 ns per cycle)
static const int T0H = 50;   // ~400 ns high for “0”
static const int T0L = 106;  // ~850 ns low  for “0”
static const int T1H = 100;  // ~800 ns high for “1”
static const int T1L = 56;   // ~450 ns low  for “1”

// Send one 24-bit GRB pixel to the WS2812 chain. No interrupt disable.
static void put_pixel(uint32_t grb) {
    for (int8_t i = 23; i >= 0; i--) {
        if (grb & (1u << i)) {
            // ‘1’ bit: ~800 ns HIGH, ~450 ns LOW
            gpio_put(NEOPIXEL_PIN, 1);
            for (int j = 0; j < T1H; j++) __asm__ volatile("nop");
            gpio_put(NEOPIXEL_PIN, 0);
            for (int j = 0; j < T1L; j++) __asm__ volatile("nop");
        } else {
            // ‘0’ bit: ~400 ns HIGH, ~850 ns LOW
            gpio_put(NEOPIXEL_PIN, 1);
            for (int j = 0; j < T0H; j++) __asm__ volatile("nop");
            gpio_put(NEOPIXEL_PIN, 0);
            for (int j = 0; j < T0L; j++) __asm__ volatile("nop");
        }
    }
    // Latch: hold low >50 μs
    gpio_put(NEOPIXEL_PIN, 0);
    sleep_us(60);
}

// Light both chained WS2812 LEDs solid white
static void set_two_pixels_white() {
    uint32_t white = (255u << 16) | (255u << 8) | 255u;  // GRB = {G=255, R=255, B=255}
    put_pixel(white);
    put_pixel(white);
}

// Entry point for Core 1 (drives NeoPixel)
void core1_entry() {
    // 1) Initialize GP27 for NeoPixel
    gpio_init(NEOPIXEL_PIN);
    gpio_set_dir(NEOPIXEL_PIN, GPIO_OUT);

    // 2) Immediately light two LEDs white
    set_two_pixels_white();

    // 3) Idle in a tight loop thereafter
    while (1) {
        tight_loop_contents();
    }
}


// ----------------------------------
// === MAIN (runs on CORE 0)        ===
// ----------------------------------

int main() {
    // ——————————————————————————————————————————————————————————
    // 0) USB-CDC stdio init + wait for host
    // ——————————————————————————————————————————————————————————
    stdio_init_all();
    printf("\n--- HW18: waiting for USB stdio connection ---\n");
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }
    printf("USB stdio connected; starting application.\n");

    // 1) Launch Core 1 so it drives the NeoPixel independently
    multicore_launch_core1(core1_entry);
    printf("Launched Core 1 for NeoPixel.\n");

    // 2) Configure button on GP20 (input + pull-up)
    gpio_init(BUTTON_PIN);
    gpio_set_dir(BUTTON_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_PIN);

    // 3) Motor PWM initialization
    printf("Configuring motor PWM slices...\n");
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
    printf("Motors configured.\n");

    // 4) Initialize camera pins + OV7670 (but only VSYNC IRQ now)
    printf("Initializing camera pins and OV7670 registers...\n");
    init_camera_pins();
    printf("Camera initialization complete.\n");

    // 5) Main loop: toggle “running” via button, perform line-follow when running
    bool running = false;
    printf("Entering main loop; press button to start/stop.\n");
    while (true) {
        // Debug print the button state each iteration
        printf("BUTTON = %d\n", gpio_get(BUTTON_PIN));

        // If button pressed (active-LOW):
        if (gpio_get(BUTTON_PIN) == 0) {
            sleep_ms(20);  // debounce
            if (gpio_get(BUTTON_PIN) == 0) {
                running = !running;
                printf("Button pressed → running = %s\n", running ? "true" : "false");
                if (!running) {
                    // Stop motors when leaving line-follow mode
                    set_left_motor(0);
                    set_right_motor(0);
                    printf("Motors stopped.\n");
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
            printf("Requesting new frame capture...\n");
            setSaveImage(1);

            while (getSaveImage() == 1) {
                tight_loop_contents();
            }


            printf("Frame capture complete → converting...\n");

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
            printf("Non-white counts: Left = %4d, Right = %4d\n", left_nonwhite, right_nonwhite);

            // 9) Steering logic
            int base_speed = 100;
            int turn_speed = 80;  // slow side to 80% duty for turning
            if (left_nonwhite > right_nonwhite + 20) {
                // Left side “off-line” → turn right
                set_left_motor(turn_speed);
                set_right_motor(base_speed);
                printf("Turning RIGHT (L=%d, R=%d)\n", turn_speed, base_speed);
            } else if (right_nonwhite > left_nonwhite + 20) {
                // Right side “off-line” → turn left
                set_left_motor(base_speed);
                set_right_motor(turn_speed);
                printf("Turning LEFT (L=%d, R=%d)\n", base_speed, turn_speed);
            } else {
                // Both sides mostly white → drive straight
                set_left_motor(base_speed);
                set_right_motor(base_speed);
                printf("Driving STRAIGHT (L=%d, R=%d)\n", base_speed, base_speed);
            }

            // throttle to ~10 FPS
            sleep_ms(100);
        } else {
            // not running → tiny sleep
            sleep_ms(10);
        }
    }
    // (unreachable)
    return 0;
}


// -----------------------------
// Camera Pin + OV7670 Setup
// -----------------------------
void init_camera_pins() {
    printf(" → Setting up camera GPIO pins...\n");
    // 1) Data pins D0..D7 = GP13,16,12,17,11,18,8,19 → inputs
    gpio_init(D0);  gpio_set_dir(D0, GPIO_IN);
    gpio_init(D1);  gpio_set_dir(D1, GPIO_IN);
    gpio_init(D2);  gpio_set_dir(D2, GPIO_IN);
    gpio_init(D3);  gpio_set_dir(D3, GPIO_IN);
    gpio_init(D4);  gpio_set_dir(D4, GPIO_IN);
    gpio_init(D5);  gpio_set_dir(D5, GPIO_IN);
    gpio_init(D6);  gpio_set_dir(D6, GPIO_IN);
    gpio_init(D7);  gpio_set_dir(D7, GPIO_IN);

    // 2) RST = GP22 as output, hold HIGH normally
    gpio_init(RST);
    gpio_set_dir(RST, GPIO_OUT);
    gpio_put(RST, 1);

    // 3) PWDN = GP4 as output, drive low to keep camera ON
    gpio_init(PWDN);
    gpio_set_dir(PWDN, GPIO_OUT);
    gpio_put(PWDN, 0);

    // 4) MCLK = GP10 → PWM @ ~15.625 MHz
    gpio_set_function(MCLK, GPIO_FUNC_PWM);
    {
        uint slice = pwm_gpio_to_slice_num(MCLK);
        pwm_set_clkdiv(slice, 2.0f);   // 125 MHz / 2 → 62.5 MHz
        pwm_set_wrap(slice, 3);        // counter 0..3 → 4 steps → 62.5 MHz / 4 = 15.625 MHz
        pwm_set_enabled(slice, true);
        pwm_set_chan_level(slice, PWM_CHAN_A, 2);  // 50% duty cycle (2/4)
    }
    sleep_ms(50);

    // 5) I²C1 @100 kHz on GP14/GP15
    printf(" → Initializing I²C1 @ 100 kHz on GP14/GP15...\n");
    i2c_init(I2C_PORT, 100 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // 6) Software + hardware reset + register writes
    init_camera();
    printf(" → OV7670 registers written.\n");

    // 7) VSYNC = GP28 (input, falling-edge triggers new frame)
    gpio_init(VS);
    gpio_set_dir(VS, GPIO_IN);
    gpio_pull_down(VS);  // ensure stable level when camera not driving
    gpio_set_irq_enabled_with_callback(VS, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    // 8) HSYNC = GP9 (input, rising-edge triggers new row) ← do NOT enable IRQ yet
      gpio_init(HS);
    gpio_set_dir(HS, GPIO_IN);
    gpio_pull_down(HS);
    gpio_set_irq_enabled(HS, GPIO_IRQ_EDGE_RISE, true);

    // 9) PCLK = GP27 (input, rising-edge samples data) ← do NOT enable IRQ yet
    gpio_init(PCLK);
    gpio_set_dir(PCLK, GPIO_IN);
    gpio_pull_down(PCLK);
    gpio_set_irq_enabled(PCLK, GPIO_IRQ_EDGE_RISE, true);

    printf(" → VSYNC IRQ enabled; HSYNC/PCLK IRQs will be enabled during capture.\n");
}

void init_camera() {
    printf("   → Hardware resetting OV7670...\n");
    // Hardware reset: pulse RST low briefly
    gpio_put(RST, 0);
    sleep_ms(1);
    gpio_put(RST, 1);
    sleep_ms(50);

    printf("   → Software reset: writing COM7 = 0x80...\n");
    // Software reset (COM7 = 0x80)
    OV7670_write_register(0x12, 0x80);
    sleep_ms(50);

    printf("   → Setting PLL/clock registers...\n");
    // PLL / clock config for ~30 FPS
    OV7670_write_register(OV7670_REG_CLKRC, 1);  // CLKRC = 1 (divide by 1)
    OV7670_write_register(OV7670_REG_DBLV, 0);   // DBLV  = 0 (no PLL)

    printf("   → Setting RGB565 mode...\n");
    // RGB565 format
    for (int i = 0; i < 12; i++) {
        OV7670_write_register(OV7670_rgb[i][0], OV7670_rgb[i][1]);
    }

    printf("   → Writing all %d init registers...\n", 92);
    // Write all 92 registers from OV7670_init[][]
    for (int i = 0; i < 92; i++) {
        OV7670_write_register(OV7670_init[i][0], OV7670_init[i][1]);
    }
    sleep_ms(300);  // allow camera to settle
    printf("   → OV7670 register block complete.\n");
}

// I²C write to OV7670
void OV7670_write_register(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = { reg, value };
    i2c_write_blocking(I2C_PORT, OV7670_ADDR, buf, 2, false);
    sleep_ms(1);
}

// I²C read from OV7670 (not used here, but included for completeness)
uint8_t OV7670_read_register(uint8_t reg) {
    uint8_t val;
    i2c_write_blocking(I2C_PORT, OV7670_ADDR, &reg, 1, false);
    i2c_read_blocking(I2C_PORT, OV7670_ADDR, &val, 1, false);
    return val;
}

// Convert raw RGB565 → 8-bit R/G/B arrays
void convertImage() {
    picture.index = 0;
    for (int i = 0; i < IMAGESIZEX * IMAGESIZEY * 2; i += 2) {
        uint8_t hi = cameraData[i + 1];   // hi byte
        uint8_t lo = cameraData[i];       // lo byte
        // hi = RRRRRGGG, lo = GGGBBBBB
        picture.r[picture.index] = (uint8_t)((hi >> 3) << 3);                      // 5 bits → 0..255
        picture.g[picture.index] = (uint8_t)((((hi & 0x07) << 3) | (lo >> 5)) << 2); // 6 bits → 0..255
        picture.b[picture.index] = (uint8_t)((lo & 0x1F) << 3);                     // 5 bits → 0..255
        picture.index++;
    }
}
