#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"

#define PIN_A     0   // IN1
#define PIN_B     1   // IN2

// Duty-cycle range (signed percent)
#define DUTY_MIN  -100
#define DUTY_MAX   100

int main() {
    stdio_init_all();

    while (!stdio_usb_connected()) {
        sleep_ms(50);
    }
    printf("USB connected! Starting motor control...\n");

    // Configure GPIO0/1 for PWM
    gpio_set_function(PIN_A, GPIO_FUNC_PWM);
    gpio_set_function(PIN_B, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(PIN_A);

    const uint16_t wrap = 65535;
    pwm_set_wrap(slice, wrap);

    pwm_set_enabled(slice, true);

    printf("Fixed wrap = %u (PWM ≈ %.1f Hz)\n", 
           wrap, 125e6f / (1.0f * (wrap + 1)));
    printf("Press '+' / '-' over USB to adjust duty. Starting at 0%%.\n");

    int duty = 0;  // current duty in signed percent

    while (true) {
        int c = getchar_timeout_us(0);
        if (c == '+') {
            if (duty < DUTY_MAX) duty++;
            printf("Duty = %d%%\n", duty);
        } else if (c == '-') {
            if (duty > DUTY_MIN) duty--;
            printf("Duty = %d%%\n", duty);
        }

        // Compute compare value from |duty|% of wrap
        uint16_t level = (uint16_t)(((uint32_t)wrap * (uint32_t)abs(duty)) / 100);

        if (duty > 0) {
            // Forward: IN1 = PWM, IN2 = 0
            pwm_set_chan_level(slice, PWM_CHAN_A, level);
            pwm_set_chan_level(slice, PWM_CHAN_B, 0);
        } else if (duty < 0) {
            // Reverse: IN1 = 0, IN2 = PWM
            pwm_set_chan_level(slice, PWM_CHAN_A, 0);
            pwm_set_chan_level(slice, PWM_CHAN_B, level);
        } else {
            // Stop/coast: both low
            pwm_set_chan_level(slice, PWM_CHAN_A, 0);
            pwm_set_chan_level(slice, PWM_CHAN_B, 0);
        }

        sleep_ms(10);
    }

    return 0;
}
