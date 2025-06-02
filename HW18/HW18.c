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
 *   • HW18.c              ← this file
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
#define PWDN   3     // Power-down line (driven low → camera on; driven high → camera off)

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
static volatile bool startedOnce = false;

// Remember last turn direction: -1=left, +1=right, 0=straight/unknown
static int last_direction = 0;

// Prototypes
void init_camera_pins();
void init_camera();
void setSaveImage(uint32_t s)       { saveImage = (uint8_t)s; }
uint32_t getSaveImage()             { return (uint32_t)saveImage; }
void convertImage();
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
            putchar( is_white ? '#' : '.' );
        }
        putchar('\n');
    }
    putchar('\n');
    fflush(stdout);
}

// GPIO interrupt callback for VS, HS, and PCLK
void gpio_callback(uint gpio, uint32_t events) {
    // *************************************************
    // 1) VSYNC falling-edge → start a new frame
    // *************************************************
    if (gpio == VS) {
        // Only “arm” HSYNC & PCLK on the first VSYNC after saveImage=1
        if (saveImage && !startedOnce) {
            rawIndex     = 0;
            hsCount      = 0;
            vsCount      = 0;
            startImage   = 1;   // “we’re inside a frame now”
            startCollect = 0;   // and not yet collecting row data
            startedOnce  = true;

            // ***** NOW enable HSYNC & PCLK IRQs for this frame *****
            gpio_set_irq_enabled(HS,   GPIO_IRQ_EDGE_RISE, true);
            gpio_set_irq_enabled(PCLK, GPIO_IRQ_EDGE_RISE, true);
        }
    }
    // *************************************************
    // 2) HSYNC rising-edge → new row in current frame
    // *************************************************
    else if (gpio == HS) {
        // Only collect HSYNC if we are actively capturing
        if (!(saveImage && startedOnce)) return;
        startCollect = 1;     // next PCLK pulses belong to this row
        hsCount++;
    }
    // *************************************************
    // 3) PCLK rising-edge → sample one camera byte
    // *************************************************
    else if (gpio == PCLK) {
        // Only collect pixel bytes if (saveImage && startedOnce && startCollect)
        if (!(saveImage && startedOnce && startCollect)) return;

        vsCount++;
        // Read the 8 data lines (D0..D7) in one go:
        uint8_t byteIn = (gpio_get(D7) << 7)
                       | (gpio_get(D6) << 6)
                       | (gpio_get(D5) << 5)
                       | (gpio_get(D4) << 4)
                       | (gpio_get(D3) << 3)
                       | (gpio_get(D2) << 2)
                       | (gpio_get(D1) << 1)
                       |  gpio_get(D0);

        cameraData[rawIndex++] = byteIn;

        // End of a row if we’ve seen 2×IMAGESIZEX PCLK pulses
        if (vsCount == IMAGESIZEX * 2) {
            vsCount      = 0;
            startCollect = 0;
        }

        // Once we have stored all 80×60×2 bytes, end capture:
        if (rawIndex >= IMAGESIZEX * IMAGESIZEY * 2) {
            saveImage    = 0;
            startImage   = 0;
            startCollect = 0;
            startedOnce  = false;

            // ***** Immediately disable HSYNC & PCLK IRQs *****
            gpio_set_irq_enabled(HS,   GPIO_IRQ_EDGE_RISE, false);
            gpio_set_irq_enabled(PCLK, GPIO_IRQ_EDGE_RISE, false);
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
    if (adjusted < DUTY_MIN) duty = DUTY_MIN;
    if (adjusted > DUTY_MAX) duty = DUTY_MAX;
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

void printImage(void) {
    // Stream all 80×60 pixels as: “x r g b\n” or “r g b\n” in row-major order.
    // We’ll just send “r g b\n” for each of the 4800 pixels (one text line per pixel).
    for (uint32_t i = 0; i < IMAGESIZEX * IMAGESIZEY; i++) {
        // print each pixel’s R, G, B as decimal 0..255
        printf("%3u %3u %3u\n",
               picture.r[i],
               picture.g[i],
               picture.b[i]
        );
    }
    // (Optional) a blank line to mark “end of frame”
    putchar('\n');
    fflush(stdout);
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
            saveImage    = 1;
            startedOnce  = false;   // ensure VSYNC ISR will arm HSYNC/PCLK

            while (getSaveImage() == 1) {
                tight_loop_contents();
            }

            // 7) Convert raw RGB565 → 8-bit R/G/B
            convertImage();
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

    // 9) PCLK = GP27 (input, rising-edge samples data) ← do NOT enable IRQ yet
    gpio_init(PCLK);
    gpio_set_dir(PCLK, GPIO_IN);
    gpio_pull_down(PCLK);
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

    printf("   → Setting RGB565 mode (12 registers)...\n");
    // RGB565 format
    for (int i = 0; i < 12; i++) {
        OV7670_write_register(OV7670_rgb[i][0], OV7670_rgb[i][1]);
    }

    printf("   → Writing all %d init registers...\n", 92);
    // Write all 92 registers from OV7670_init[][]
    for (int i = 0; i < 92; i++) {
        printf("     → Writing reg 0x%02X = 0x%02X …\n",
               OV7670_init[i][0], OV7670_init[i][1]);
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
        uint8_t lo = cameraData[i];       // low byte  = GGGBBBBB
        uint8_t hi = cameraData[i + 1];   // high byte = RRRRRGGG

        // Extract and scale each channel from RGB565:
        //   R = hi[7:3], G = (hi[2:0]<<3) | (lo[7:5]), B = lo[4:0]
        picture.r[picture.index] = (uint8_t)((hi >> 3) << 3);                        // 5 bits → 0..255
        picture.g[picture.index] = (uint8_t)((((hi & 0x07) << 3) | (lo >> 5)) << 2); // 6 bits → 0..255
        picture.b[picture.index] = (uint8_t)((lo & 0x1F) << 3);                      // 5 bits → 0..255
        picture.index++;
    }
}
