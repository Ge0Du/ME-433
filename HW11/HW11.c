#include <stdio.h>
#include "pico/stdlib.h"
#include "tusb.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include <math.h>

#define PIN_UP      2
#define PIN_DOWN    3
#define PIN_LEFT    4
#define PIN_RIGHT   5
#define PIN_MODE    6
#define LED_PIN     25

#define SPEED_LEVELS 4
const uint16_t speed_thresholds[SPEED_LEVELS] = {0, 300, 800, 1500};
const int speed_values[SPEED_LEVELS] = {1, 2, 4, 6};

bool mode_circle = false;
absolute_time_t mode_toggle_time;
absolute_time_t button_times[4];

void setup_gpio() {
    for (int pin = PIN_UP; pin <= PIN_MODE; ++pin) {
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_IN);
        gpio_pull_up(pin);
    }
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
}

int get_speed_us(uint i) {
    if (!gpio_get(i)) {
        int64_t dt = absolute_time_diff_us(button_times[i - PIN_UP], get_absolute_time());
        for (int j = SPEED_LEVELS - 1; j >= 0; j--) {
            if (dt >= speed_thresholds[j]) return speed_values[j];
        }
    } else {
        button_times[i - PIN_UP] = get_absolute_time();
    }
    return 0;
}

void send_hid_report_mouse(int8_t dx, int8_t dy) {
    if (!tud_hid_ready()) return;
    uint8_t report[4] = {0, dx, dy, 0};  // Buttons, dx, dy, wheel
    tud_hid_report(1, report, sizeof(report));
}

void hid_task() {
    static absolute_time_t last_time;
    static float theta = 0;

    if (absolute_time_diff_us(last_time, get_absolute_time()) < 10000) return;
    last_time = get_absolute_time();

    if (!gpio_get(PIN_MODE)) {
        if (absolute_time_diff_us(mode_toggle_time, get_absolute_time()) > 300000) {
            mode_circle = !mode_circle;
            gpio_put(LED_PIN, mode_circle);
            mode_toggle_time = get_absolute_time();
        }
    }

    if (mode_circle) {
        int8_t dx = (int8_t)(3 * cosf(theta));
        int8_t dy = (int8_t)(3 * sinf(theta));
        theta += 0.2f;
        send_hid_report_mouse(dx, dy);
    } else {
        int8_t dx = get_speed_us(PIN_RIGHT) - get_speed_us(PIN_LEFT);
        int8_t dy = get_speed_us(PIN_DOWN) - get_speed_us(PIN_UP);
        send_hid_report_mouse(dx, dy);
    }
}

int main() {
    stdio_init_all();
    tusb_init();
    setup_gpio();

    while (true) {
        tud_task();   // USB stack
        hid_task();   // Mouse control
    }
}
