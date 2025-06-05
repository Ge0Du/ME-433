#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/multicore.h"

#include "ov7670_regs.h"
#define CALIBRATED_CENTER  40
#define DEADZONE_RATIO     0.10f
#define MAX_SPEED          60
#define MIN_SPEED           20
#define LEFT_CALIBRATION   1.05f
#define MIN_WHITE_RATIO     0.05f

void    OV7670_write_register(uint8_t reg, uint8_t value);
uint8_t OV7670_read_register(uint8_t reg);

#define I2C_PORT    i2c1
#define I2C_SDA     14
#define I2C_SCL     15

#define D0 13
#define D1 16
#define D2 12
#define D3 17
#define D4 11
#define D5 18
#define D6 8
#define D7 19

#define VS     28
#define HS     9
#define MCLK   10
#define PCLK   27
#define RST    22
#define PWDN   3

#define IMAGESIZEX 80
#define IMAGESIZEY 60

static volatile uint8_t cameraData[IMAGESIZEX * IMAGESIZEY * 2];

typedef struct {
    uint32_t index;
    uint8_t  r[IMAGESIZEX * IMAGESIZEY];
    uint8_t  g[IMAGESIZEX * IMAGESIZEY];
    uint8_t  b[IMAGESIZEX * IMAGESIZEY];
} cameraImage_t;
static volatile cameraImage_t picture;

static volatile uint8_t  saveImage     = 0;
static volatile uint8_t  startImage    = 0;
static volatile uint8_t  startCollect  = 0;
static volatile uint32_t rawIndex      = 0;
static volatile uint32_t hsCount       = 0;
static volatile uint32_t vsCount       = 0;
static volatile bool startedOnce = false;

static int last_direction = 0;

void init_camera_pins();
void init_camera();
void setSaveImage(uint32_t s)       { saveImage = (uint8_t)s; }
uint32_t getSaveImage()             { return (uint32_t)saveImage; }
void convertImage();

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

void gpio_callback(uint gpio, uint32_t events) {
    if (gpio == VS) {
        if (saveImage && !startedOnce) {
            rawIndex     = 0;
            hsCount      = 0;
            vsCount      = 0;
            startImage   = 1;
            startCollect = 0;
            startedOnce  = true;
            gpio_set_irq_enabled(HS,   GPIO_IRQ_EDGE_RISE, true);
            gpio_set_irq_enabled(PCLK, GPIO_IRQ_EDGE_RISE, true);
        }
    }
    else if (gpio == HS) {
        if (!(saveImage && startedOnce)) return;
        startCollect = 1;
        hsCount++;
    }
    else if (gpio == PCLK) {
        if (!(saveImage && startedOnce && startCollect)) return;
        vsCount++;
        uint8_t byteIn = (gpio_get(D7) << 7)
                       | (gpio_get(D6) << 6)
                       | (gpio_get(D5) << 5)
                       | (gpio_get(D4) << 4)
                       | (gpio_get(D3) << 3)
                       | (gpio_get(D2) << 2)
                       | (gpio_get(D1) << 1)
                       |  gpio_get(D0);

        cameraData[rawIndex++] = byteIn;

        if (vsCount == IMAGESIZEX * 2) {
            vsCount      = 0;
            startCollect = 0;
        }

        if (rawIndex >= IMAGESIZEX * IMAGESIZEY * 2) {
            saveImage    = 0;
            startImage   = 0;
            startCollect = 0;
            startedOnce  = false;
            gpio_set_irq_enabled(HS,   GPIO_IRQ_EDGE_RISE, false);
            gpio_set_irq_enabled(PCLK, GPIO_IRQ_EDGE_RISE, false);
        }
    }
}

#define LEFT_IN1_PIN   0
#define LEFT_IN2_PIN   1
#define RIGHT_IN1_PIN  2
#define RIGHT_IN2_PIN  4
#define BUTTON_PIN     20

static const uint16_t LEFT_WRAP  = 65535;
static const uint16_t RIGHT_WRAP = 65535;
#define DUTY_MIN   0
#define DUTY_MAX 100

static void set_left_motor(int duty) {
    float scaled = (float)duty * LEFT_CALIBRATION;
    int adjusted = (int)scaled;
    uint slice = pwm_gpio_to_slice_num(LEFT_IN1_PIN);
    if (adjusted < DUTY_MIN) duty = DUTY_MIN;
    if (adjusted > DUTY_MAX) duty = DUTY_MAX;
    uint16_t level = (uint16_t)(((uint32_t)LEFT_WRAP * (uint32_t)adjusted) / 100);

    if (adjusted > 0) {
        pwm_set_chan_level(slice, PWM_CHAN_A, level);
        pwm_set_chan_level(slice, PWM_CHAN_B, 0);
    } else {
        pwm_set_chan_level(slice, PWM_CHAN_A, 0);
        pwm_set_chan_level(slice, PWM_CHAN_B, 0);
    }
}

static void set_right_motor(int duty) {
    uint slice = pwm_gpio_to_slice_num(RIGHT_IN1_PIN);
    if (duty < DUTY_MIN) duty = DUTY_MIN;
    if (duty > DUTY_MAX) duty = DUTY_MAX;
    uint16_t level = (uint16_t)(((uint32_t)RIGHT_WRAP * (uint32_t)duty) / 100);

    if (duty > 0) {
        pwm_set_chan_level(slice, PWM_CHAN_A, level);
        pwm_set_chan_level(slice, PWM_CHAN_B, 0);
    } else {
        pwm_set_chan_level(slice, PWM_CHAN_A, 0);
        pwm_set_chan_level(slice, PWM_CHAN_B, 0);
    }
}

#define NEOPIXEL_PIN 5

static const int T0H = 50;
static const int T0L = 106;
static const int T1H = 100;
static const int T1L = 56;

static void put_pixel(uint32_t grb) {
    for (int8_t i = 23; i >= 0; i--) {
        if (grb & (1u << i)) {
            gpio_put(NEOPIXEL_PIN, 1);
            for (int j = 0; j < T1H; j++) __asm__ volatile("nop");
            gpio_put(NEOPIXEL_PIN, 0);
            for (int j = 0; j < T1L; j++) __asm__ volatile("nop");
        } else {
            gpio_put(NEOPIXEL_PIN, 1);
            for (int j = 0; j < T0H; j++) __asm__ volatile("nop");
            gpio_put(NEOPIXEL_PIN, 0);
            for (int j = 0; j < T0L; j++) __asm__ volatile("nop");
        }
    }
    gpio_put(NEOPIXEL_PIN, 0);
    sleep_us(60);
}

static void set_two_pixels_white() {
    uint32_t white = (255u << 16) | (255u << 8) | 255u;
    put_pixel(white);
    put_pixel(white);
}

void core1_entry() {
    gpio_init(NEOPIXEL_PIN);
    gpio_set_dir(NEOPIXEL_PIN, GPIO_OUT);
    set_two_pixels_white();
    while (1) {
        tight_loop_contents();
    }
}

void printImage(void) {
    for (uint32_t i = 0; i < IMAGESIZEX * IMAGESIZEY; i++) {
        printf("%3u %3u %3u\n",
               picture.r[i],
               picture.g[i],
               picture.b[i]
        );
    }
    putchar('\n');
    fflush(stdout);
}

int main() {
    stdio_init_all();
    printf("\n--- HW18: waiting for USB stdio connection ---\n");

    printf("USB stdio connected; starting application.\n");

    multicore_launch_core1(core1_entry);
    printf("Launched Core 1 for NeoPixel.\n");

    gpio_init(BUTTON_PIN);
    gpio_set_dir(BUTTON_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_PIN);

    printf("Configuring motor PWM slices...\n");
    gpio_set_function(LEFT_IN1_PIN, GPIO_FUNC_PWM);
    gpio_set_function(LEFT_IN2_PIN, GPIO_FUNC_PWM);
    {
        uint slice = pwm_gpio_to_slice_num(LEFT_IN1_PIN);
        pwm_set_wrap(slice, LEFT_WRAP);
        pwm_set_enabled(slice, true);
    }
    gpio_set_function(RIGHT_IN1_PIN, GPIO_FUNC_PWM);
    gpio_set_function(RIGHT_IN2_PIN, GPIO_FUNC_PWM);
    {
        uint slice = pwm_gpio_to_slice_num(RIGHT_IN1_PIN);
        pwm_set_wrap(slice, RIGHT_WRAP);
        pwm_set_enabled(slice, true);
    }
    printf("Motors configured.\n");

    printf("Initializing camera pins and OV7670 registers...\n");
    init_camera_pins();
    for (int i = 0; i < 100; i++) {
        printf("•• VSYNC reads: %d\n", gpio_get(VS));
        sleep_ms(50);
    }
    printf("•• Done polling VSYNC.\n");
    printf("Camera initialization complete.\n");

    bool running = false;
    printf("Entering main loop; press button to start/stop.\n");
    while (true) {
        if (gpio_get(BUTTON_PIN) == 0) {
            sleep_ms(20);
            if (gpio_get(BUTTON_PIN) == 0) {
                running = !running;
                printf("Button pressed → running = %s\n", running ? "true" : "false");
                if (!running) {
                    set_left_motor(0);
                    set_right_motor(0);
                    printf("Motors stopped.\n");
                }
                while (gpio_get(BUTTON_PIN) == 0) {
                    sleep_ms(10);
                }
                sleep_ms(50);
            }
        }

        if (running) {
            saveImage    = 1;
            startedOnce  = false;

            while (getSaveImage() == 1) {
                tight_loop_contents();
            }

            convertImage();
            convertImage();
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

            if (white_ratio < MIN_WHITE_RATIO) {
                if (last_direction < 0) {
                    set_left_motor(MAX_SPEED);
                    set_right_motor(MIN_SPEED);
                    printf("Too few white pixels → SWEEP RIGHT (L=%d, R=%d)\n", MAX_SPEED, MIN_SPEED);
                    last_direction = +1;
                } else if (last_direction > 0) {
                    set_left_motor(MIN_SPEED);
                    set_right_motor(MAX_SPEED);
                    printf("Too few white pixels → SWEEP LEFT (L=%d, R=%d)\n", MIN_SPEED, MAX_SPEED);
                    last_direction = -1;
                } else {
                    set_left_motor(0);
                    set_right_motor(0);
                    printf("Too few white pixels & no last_direction → STOP\n");
                }
                continue;
            }
            if (white_count > 0) {
                float centroid_x = (float)sum_x / (float)white_count;
                float half_dead = (IMAGESIZEX * DEADZONE_RATIO) / 2.0f;
                float center = (float)CALIBRATED_CENTER;
                float center_min = center - half_dead;
                float center_max = center + half_dead;

                if (centroid_x >= center_min && centroid_x <= center_max) {
                    set_left_motor(MAX_SPEED);
                    set_right_motor(MAX_SPEED);
                    printf(
                      "Centroid %.2f in deadzone [%.2f..%.2f] → STRAIGHT (L=%d, R=%d)\n",
                      centroid_x, center_min, center_max,
                      MAX_SPEED, MAX_SPEED
                    );
                    last_direction = 0;
                } else if (centroid_x < center_min) {
                    set_left_motor(MAX_SPEED);
                    set_right_motor(MIN_SPEED);
                    printf(
                      "Centroid %.2f < %.2f → TURN LEFT (L=%d, R=%d)\n",
                      centroid_x, center_min, MIN_SPEED, MAX_SPEED
                    );
                    last_direction = -1;
                } else {
                    set_left_motor(MIN_SPEED);
                    set_right_motor(MAX_SPEED);
                    printf(
                      "Centroid %.2f > %.2f → TURN RIGHT (L=%d, R=%d)\n",
                      centroid_x, center_max, MAX_SPEED, MIN_SPEED
                    );
                    last_direction = +1;
                }
            } else {
                if (last_direction < 0) {
                    set_left_motor(MAX_SPEED);
                    set_right_motor(MIN_SPEED);
                    printf("No line → FLIP → SWEEP RIGHT (L=%d, R=%d)\n", MAX_SPEED, MIN_SPEED);
                    last_direction = +1;
                } else if (last_direction > 0) {
                    set_left_motor(MIN_SPEED);
                    set_right_motor(MAX_SPEED);
                    printf("No line → FLIP → SWEEP LEFT (L=%d, R=%d)\n", MIN_SPEED, MAX_SPEED);
                    last_direction = -1;
                } else {
                    set_left_motor(0);
                    set_right_motor(0);
                    printf("No line & no last_direction → STOP (L=0, R=0)\n");
                }
            }
            sleep_ms(100);
        } else {
            sleep_ms(10);
        }
    }
    return 0;
}

void init_camera_pins() {
    printf(" → Setting up camera GPIO pins...\n");
    gpio_init(D0);  gpio_set_dir(D0, GPIO_IN);
    gpio_init(D1);  gpio_set_dir(D1, GPIO_IN);
    gpio_init(D2);  gpio_set_dir(D2, GPIO_IN);
    gpio_init(D3);  gpio_set_dir(D3, GPIO_IN);
    gpio_init(D4);  gpio_set_dir(D4, GPIO_IN);
    gpio_init(D5);  gpio_set_dir(D5, GPIO_IN);
    gpio_init(D6);  gpio_set_dir(D6, GPIO_IN);
    gpio_init(D7);  gpio_set_dir(D7, GPIO_IN);

    gpio_init(RST);
    gpio_set_dir(RST, GPIO_OUT);
    gpio_put(RST, 1);

    gpio_init(PWDN);
    gpio_set_dir(PWDN, GPIO_OUT);
    gpio_put(PWDN, 0);

    gpio_set_function(MCLK, GPIO_FUNC_PWM);
    {
        uint slice = pwm_gpio_to_slice_num(MCLK);
        pwm_set_clkdiv(slice, 2.0f);
        pwm_set_wrap(slice, 3);
        pwm_set_enabled(slice, true);
        pwm_set_chan_level(slice, PWM_CHAN_A, 2);
    }
    sleep_ms(50);

    printf(" → Initializing I²C1 @ 100 kHz on GP14/GP15...\n");
    i2c_init(I2C_PORT, 100 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    init_camera();
    printf(" → OV7670 registers written.\n");

    gpio_init(VS);
    gpio_set_dir(VS, GPIO_IN);
    gpio_pull_down(VS);
    gpio_set_irq_enabled_with_callback(VS, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    gpio_init(HS);
    gpio_set_dir(HS, GPIO_IN);
    gpio_pull_down(HS);

    gpio_init(PCLK);
    gpio_set_dir(PCLK, GPIO_IN);
    gpio_pull_down(PCLK);
}

void init_camera() {
    printf("   → Hardware resetting OV7670...\n");
    gpio_put(RST, 0);
    sleep_ms(1);
    gpio_put(RST, 1);
    sleep_ms(50);

    printf("   → Software reset: writing COM7 = 0x80...\n");
    OV7670_write_register(0x12, 0x80);
    sleep_ms(50);

    printf("   → Setting PLL/clock registers...\n");
    OV7670_write_register(OV7670_REG_CLKRC, 1);
    OV7670_write_register(OV7670_REG_DBLV, 0);

    printf("   → Setting RGB565 mode (12 registers)...\n");
    for (int i = 0; i < 12; i++) {
        OV7670_write_register(OV7670_rgb[i][0], OV7670_rgb[i][1]);
    }

    printf("   → Writing all %d init registers...\n", 92);
    for (int i = 0; i < 92; i++) {
        printf("     → Writing reg 0x%02X = 0x%02X …\n",
               OV7670_init[i][0], OV7670_init[i][1]);
        OV7670_write_register(OV7670_init[i][0], OV7670_init[i][1]);
    }

    sleep_ms(300);
    printf("   → OV7670 register block complete.\n");
}

void OV7670_write_register(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = { reg, value };
    i2c_write_blocking(I2C_PORT, OV7670_ADDR, buf, 2, false);
    sleep_ms(1);
}

uint8_t OV7670_read_register(uint8_t reg) {
    uint8_t val;
    i2c_write_blocking(I2C_PORT, OV7670_ADDR, &reg, 1, false);
    i2c_read_blocking(I2C_PORT, OV7670_ADDR, &val, 1, false);
    return val;
}

void convertImage() {
    picture.index = 0;
    for (int i = 0; i < IMAGESIZEX * IMAGESIZEY * 2; i += 2) {
        uint8_t lo = cameraData[i];
        uint8_t hi = cameraData[i + 1];

        picture.r[picture.index] = (uint8_t)((hi >> 3) << 3);
        picture.g[picture.index] = (uint8_t)((((hi & 0x07) << 3) | (lo >> 5)) << 2);
        picture.b[picture.index] = (uint8_t)((lo & 0x1F) << 3);
        picture.index++;
    }
}
