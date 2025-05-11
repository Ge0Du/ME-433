#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/pio.h"
#include "ws2812.pio.h"

// === CONFIGURATION ===
#define SERVO_PIN 15
#define LED_PIN 17
#define NUM_PIXELS 4

// === STRUCTS ===
typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} wsColor;

// === FUNCTION PROTOTYPES ===
void servo_pwm_init();
void servo_set_angle(int angle);
void ws2812_init(PIO pio, int sm, uint pin);
void put_pixel(PIO pio, int sm, uint32_t pixel_grb);
uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b);
wsColor HSBtoRGB(float hue, float sat, float brightness);

// === SERVO FUNCTIONS ===
void servo_pwm_init() {
    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(SERVO_PIN);
    pwm_set_clkdiv(slice, 64.0f); // 150 MHz / 64 = ~2.34 MHz
    pwm_set_wrap(slice, 46875);   // 2.34 MHz / 46875 = 50 Hz
    pwm_set_enabled(slice, true);
}

void servo_set_angle(int angle) {
    float min_duty = 0.5f / 20.0f; // 2.5%
    float max_duty = 2.5f / 20.0f; // 12.5%
    float duty_frac = min_duty + (max_duty - min_duty) * (angle / 180.0f);
    uint level = (uint)(duty_frac * 46875);
    pwm_set_gpio_level(SERVO_PIN, level);
}

// === WS2812 FUNCTIONS ===
void ws2812_init(PIO pio, int sm, uint pin) {
    uint offset = pio_add_program(pio, &ws2812_program);
    ws2812_program_init(pio, sm, offset, pin, 800000, false);
}

void put_pixel(PIO pio, int sm, uint32_t pixel_grb) {
    pio_sm_put_blocking(pio, sm, pixel_grb << 8u);
}

uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint32_t)g << 16) | ((uint32_t)r << 8) | b;
}

wsColor HSBtoRGB(float hue, float sat, float brightness) {
    float red = 0.0, green = 0.0, blue = 0.0;
    if (sat == 0.0) {
        red = green = blue = brightness;
    } else {
        if (hue >= 360.0f) hue = 0.0f;
        int slice = (int)(hue / 60.0f);
        float frac = (hue / 60.0f) - slice;
        float p = brightness * (1 - sat);
        float q = brightness * (1 - sat * frac);
        float t = brightness * (1 - sat * (1 - frac));
        switch (slice) {
            case 0: red = brightness; green = t; blue = p; break;
            case 1: red = q; green = brightness; blue = p; break;
            case 2: red = p; green = brightness; blue = t; break;
            case 3: red = p; green = q; blue = brightness; break;
            case 4: red = t; green = p; blue = brightness; break;
            case 5: red = brightness; green = p; blue = q; break;
        }
    }
    wsColor color = {
        .r = (uint8_t)(red * 255),
        .g = (uint8_t)(green * 255),
        .b = (uint8_t)(blue * 255)
    };
    return color;
}

// === MAIN ===
int main() {
    stdio_init_all();
    servo_pwm_init();

    PIO pio = pio0;
    int sm = 0;
    ws2812_init(pio, sm, LED_PIN);

    while (true) {
        for (int t = 0; t < 5000; t += 20) {
            float hue_base = (t / 5000.0f) * 360.0f;
            for (int i = 0; i < NUM_PIXELS; i++) {
                float hue = fmodf(hue_base + i * (360.0f / NUM_PIXELS), 360.0f);
                wsColor c = HSBtoRGB(hue, 1.0f, 0.3f);
                put_pixel(pio, sm, urgb_u32(c.r, c.g, c.b));
            }

            int angle;
            if (t <= 2500)
                angle = (t * 180) / 2500;
            else
                angle = 180 - ((t - 2500) * 180 / 2500);
            servo_set_angle(angle);

            sleep_ms(20);
        }
    }
}
