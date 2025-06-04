/**
 * 3-Wheel Differential Drive with OV7670 Camera (D0–D7 on GPIO 13,16,12,17,11,18,8,19)
 * and NeoPixel Illumination on GPIO 27 (two LEDs), NeoPixel driven on Core 1.
 *
 * This version includes:
 *   • USB‐CDC stdio initialization + wait‐for‐host at startup
 *   • printf()‐based debug messages sprinkled throughout
 *   • HSYNC/PCLK interrupts enabled only during frame capture to avoid spurious IRQ floods
 *   • Centroid‐of‐white‐pixels steering with:
 *       1) an adjustable “calibrated center” column (instead of fixed 40);
 *       2) a deadzone defined as ±X% of image width around that center;
 *       3) **discrete** MIN_SPEED/MAX_SPEED outside the deadzone (no quadratic).
 *
 * You can adjust:
 *   • CALIBRATED_CENTER  — the column (0..79) you consider to be “perfectly centered.”
 *   • DEADZONE_RATIO      — fraction of image width (0..1) defining ± deadzone around CALIBRATED_CENTER.
 *   • MAX_SPEED           — PWM duty for the “fast” wheel when turning or driving straight (0..100).
 *   • MIN_SPEED           — PWM duty for the “slow” wheel when turning (0..MAX_SPEED).
 *
 * Project files:
 *   • CMakeLists.txt      ← must link pico_stdio_usb, pico_multicore, hardware_pwm, hardware_i2c, hardware_gpio
 *   • HW18.c              ← this file (updated to use cam.c / cam.h)
 *   • cam.h               ← camera‐pin macros + prototypes
 *   • cam.c               ← camera ISR, init_camera(), convertImage(), etc.
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

#include "cam.h"                   // camera‐pin defines, init_camera_pins(), setSaveImage(), getSaveImage(), convertImage(), etc.

// ──────────────────────────────────────────────────────────────────────────────
// Adjustable steering parameters:
// ──────────────────────────────────────────────────────────────────────────────

// If your camera sees “perfect alignment” (e.g. full-white column) at x = 40, set this to 40.
// Must be between 0 and IMAGESIZEX-1 (i.e., 0..79).
#define CALIBRATED_CENTER  40

// Fraction of the full image width where robot drives straight.
// e.g. 0.05 ⇒ ±2 pixels around CALIBRATED_CENTER (5% of 80 = 4 total).
#define DEADZONE_RATIO     0.10f

// PWM duty for the “fast” wheel when turning or driving straight (0..100).
#define MAX_SPEED          60

// PWM duty for the “slow” wheel when turning (0..MAX_SPEED).
// If you want sharper turns, make this smaller. Must be ≤ MAX_SPEED.
#define MIN_SPEED           20

#define LEFT_CALIBRATION   1.05f

#define MIN_WHITE_RATIO     0.05f   // Example: 5% white pixels required to consider a valid line

// ──────────────────────────────────────────────────────────────────────────────
// Send a crude 80×60 ASCII map over USB-CDC:
//   “#” = pixel ≥ threshold; “.” = pixel < threshold.
// You’ll see 60 lines of 80 characters each in your serial monitor.
// ──────────────────────────────────────────────────────────────────────────────
static void dumpThresholdedASCII(int threshold) {
    for (int y = 0; y < IMAGESIZEY; y++) {
        for (int x = 0; x < IMAGESIZEX; x++) {
            int idx = y * IMAGESIZEX + x;
            bool is_white = (
                picture.r[idx] >= threshold &&
                picture.g[idx] >= threshold &&
                picture.b[idx] >= threshold
            );
            putchar(is_white ? '#' : '.');
        }
        putchar('\n');
    }
    putchar('\n');
    fflush(stdout);
}

// ----------------------------------
// === MOTOR & BUTTON DEFINES / CODE ===
// ----------------------------------

// Left motor (wiring is flipped on DRV8833)
#define LEFT_IN1_PIN   0    // PWM slice 0, channel A
#define LEFT_IN2_PIN   1    // PWM slice 0, channel B

// Right motor (normal DRV8833 wiring)
#define RIGHT_IN1_PIN  2    // PWM slice 1, channel A
#define RIGHT_IN2_PIN  4    // PWM slice 1, channel B

// Button on GP20 (active-LOW input with pull-up)
#define BUTTON_PIN     20

static const uint16_t LEFT_WRAP  = 65535;   // ~1.9 kHz
static const uint16_t RIGHT_WRAP = 65535;   // ~1.9 kHz
#define DUTY_MIN   0
#define DUTY_MAX 100

// Drive left motor “forward physically” (swap channels because wiring is flipped)
static void set_left_motor(int duty) {
    float scaled = (float)duty * LEFT_CALIBRATION;
    int adjusted = (int)scaled;
    uint slice = pwm_gpio_to_slice_num(LEFT_IN1_PIN);
    if (adjusted < DUTY_MIN) adjusted = DUTY_MIN;
    if (adjusted > DUTY_MAX) adjusted = DUTY_MAX;
    uint16_t level = (uint16_t)(((uint32_t)LEFT_WRAP * (uint32_t)adjusted) / 100);

    if (adjusted > 0) {
        // Flipped hardware: IN1 = PWM, IN2 = 0  
        pwm_set_chan_level(slice, PWM_CHAN_A, level);  // LEFT_IN1_PIN
        pwm_set_chan_level(slice, PWM_CHAN_B, 0);      // LEFT_IN2_PIN
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
#define NEOPIXEL_PIN 5

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
    // Latch: hold low >50 µs
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
    for (int i = 0; i < 100; i++) {
        printf("•• VSYNC reads: %d\n", gpio_get(VS));
        sleep_ms(50);
    }
    printf("•• Done polling VSYNC.\n");
    printf("Camera initialization complete.\n");

    // 5) Main loop: toggle “running” via button, perform line-follow when running
    bool running = false;
    printf("Entering main loop; press button to start/stop.\n");
    while (true) {
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
            setSaveImage(1);
            while (getSaveImage() == 1) {
                tight_loop_contents();
            }

            // 7) Convert raw RGB565 → 8-bit R/G/B
            convertImage();

            // 8) === centroid‐based steering with custom center, no quadratic ===
            const int white_threshold = 200;
            dumpThresholdedASCII(white_threshold);
            uint32_t sum_x = 0;
            int white_count = 0;
            for (uint32_t idx = 0; idx < IMAGESIZEX * IMAGESIZEY; idx++) {
                bool is_white = (picture.r[idx] >= white_threshold &&
                                 picture.g[idx] >= white_threshold &&
                                 picture.b[idx] >= white_threshold);
                if (is_white) {
                    uint32_t x = idx % IMAGESIZEX;
                    sum_x += x;
                    white_count++;
                }
            }
            float white_ratio = (float)white_count / (IMAGESIZEX * IMAGESIZEY);
            printf("White pixel ratio: %.3f (threshold: %.3f)\n", white_ratio, MIN_WHITE_RATIO);
            int last_direction = 0;
            // If too few white pixels, ignore centroid and continue to flip/sweep logic
            if (white_ratio < MIN_WHITE_RATIO) {
                // Not enough white pixels → treat like no line
                if (last_direction < 0) {
                    // Last turn was left → now sweep RIGHT
                    set_left_motor(MAX_SPEED);
                    set_right_motor(MIN_SPEED);
                    printf("Too few white pixels → SWEEP RIGHT (L=%d, R=%d)\n", MAX_SPEED, MIN_SPEED);
                    last_direction = +1;
                } else if (last_direction > 0) {
                    // Last turn was right → now sweep LEFT
                    set_left_motor(MIN_SPEED);
                    set_right_motor(MAX_SPEED);
                    printf("Too few white pixels → SWEEP LEFT (L=%d, R=%d)\n", MIN_SPEED, MAX_SPEED);
                    last_direction = -1;
                } else {
                    // Never saw a line → stop
                    set_left_motor(0);
                    set_right_motor(0);
                    printf("Too few white pixels & no last_direction → STOP\n");
                }
                // Skip the centroid-based steering altogether
                sleep_ms(100);
                continue;
            }
            if (white_count > 0) {
                float centroid_x = (float)sum_x / (float)white_count; // [0..79]

                // Compute deadzone boundaries based on CALIBRATED_CENTER and DEADZONE_RATIO
                float half_dead = (IMAGESIZEX * DEADZONE_RATIO) / 2.0f; // e.g. 80*0.05/2 = 2.0
                float center = (float)CALIBRATED_CENTER;               // 40.0
                float center_min = center - half_dead;                 // 40 - 2 = 38
                float center_max = center + half_dead;                 // 40 + 2 = 42

                if (centroid_x >= center_min && centroid_x <= center_max) {
                    // within deadzone → go straight
                    set_left_motor(MAX_SPEED);
                    set_right_motor(MAX_SPEED);
                    printf(
                      "Centroid %.2f in deadzone [%.2f..%.2f] → STRAIGHT (L=%d, R=%d)\n",
                      centroid_x, center_min, center_max,
                      MAX_SPEED, MAX_SPEED
                    );
                    last_direction = 0;
                } else if (centroid_x < center_min) {
                    // line is left of calibrated center → turn LEFT
                    set_left_motor(MAX_SPEED);
                    set_right_motor(MIN_SPEED);
                    printf(
                      "Centroid %.2f < %.2f → TURN LEFT (L=%d, R=%d)\n",
                      centroid_x, center_min, MIN_SPEED, MAX_SPEED
                    );
                    last_direction = -1;
                } else {
                    // centroid_x > center_max → turn RIGHT
                    set_left_motor(MIN_SPEED);
                    set_right_motor(MAX_SPEED);
                    printf(
                      "Centroid %.2f > %.2f → TURN RIGHT (L=%d, R=%d)\n",
                      centroid_x, center_max, MAX_SPEED, MIN_SPEED
                    );
                    last_direction = +1;
                }
            } else {
                // No white pixels → flip from the last non‐straight turn:
                if (last_direction < 0) {
                    // Last turn was left → now sweep RIGHT
                    set_left_motor(MAX_SPEED);
                    set_right_motor(MIN_SPEED);
                    printf("No line → FLIP → SWEEP RIGHT (L=%d, R=%d)\n", MAX_SPEED, MIN_SPEED);
                    last_direction = +1;
                } else if (last_direction > 0) {
                    // Last turn was right → now sweep LEFT
                    set_left_motor(MIN_SPEED);
                    set_right_motor(MAX_SPEED);
                    printf("No line → FLIP → SWEEP LEFT (L=%d, R=%d)\n", MIN_SPEED, MAX_SPEED);
                    last_direction = -1;
                } else {
                    // last_direction = 0 (never saw a line) → stop
                    set_left_motor(0);
                    set_right_motor(0);
                    printf("No line & no last_direction → STOP (L=0, R=0)\n");
                }
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
