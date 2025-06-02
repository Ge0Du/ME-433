/**
 * 3-Wheel Differential Drive Robot (Left + Right Motors + Caster)
 *
 * Each side’s motor is driven by a DRV8833 H-bridge. We assume:
 *   • Left motor IN1 → GPIO 0, IN2 → GPIO 1  (PWM Slice 0, CH A/B)
 *   • Right motor IN1 → GPIO 2, IN2 → GPIO 3 (PWM Slice 1, CH A/B)
 *
 * The caster wheel is passive and does not require any code.
 *
 * This example:
 *   1. Initializes PWM on both left and right motor pins.
 *   2. Defines helper functions set_left_motor(duty) and set_right_motor(duty)
 *      where duty ∈ [−100 .. +100] (%). Positive → forward, negative → reverse.
 *   3. In main(), runs a simple demo sequence:
 *        • Drive forward at 50% for 2 seconds
 *        • Stop for 1 second
 *        • Drive backward at 50% for 2 seconds
 *        • Stop for 1 second
 *        • Turn left in place (left motor backward, right motor forward) at 50% for 1.5 s
 *        • Stop 1 s
 *        • Turn right in place (left forward, right backward) at 50% for 1.5 s
 *        • Finally stop
 *
 * You can adapt the delays or duty‐cycle values as needed.
 */

#include <stdio.h>
#include <stdlib.h>          // for abs()
#include "pico/stdlib.h"
#include "hardware/pwm.h"

// --- Pin Definitions ---
// Left motor IN1, IN2
#define LEFT_IN1_PIN   0     // PWM slice 0, channel A
#define LEFT_IN2_PIN   1     // PWM slice 0, channel B
// Right motor IN1, IN2
#define RIGHT_IN1_PIN  2     // PWM slice 1, channel A
#define RIGHT_IN2_PIN  3     // PWM slice 1, channel B

// Duty-cycle bounds (signed percent)
#define DUTY_MIN     -100
#define DUTY_MAX      100

// Helper: set a single motor’s PWM outputs given a slice number and signed duty (−100..+100)
static void set_motor_pwm(uint slice_num, pwm_gpio_id pwm_chanA, pwm_gpio_id pwm_chanB, int duty) {
    // Clamp duty to [DUTY_MIN .. DUTY_MAX]
    if (duty > DUTY_MAX) duty = DUTY_MAX;
    if (duty < DUTY_MIN) duty = DUTY_MIN;

    // Compute absolute compare value as (|duty|/100) * wrap
    uint16_t wrap = pwm_get_wrap(slice_num);
    uint16_t level = (uint16_t)(((uint32_t)wrap * (uint32_t)abs(duty)) / 100);

    if (duty > 0) {
        // Forward: channel A = PWM, channel B = 0
        pwm_set_chan_level(slice_num, PWM_CHAN_A, level);
        pwm_set_chan_level(slice_num, PWM_CHAN_B, 0);
    }
    else if (duty < 0) {
        // Reverse: channel A = 0, channel B = PWM
        pwm_set_chan_level(slice_num, PWM_CHAN_A, 0);
        pwm_set_chan_level(slice_num, PWM_CHAN_B, level);
    }
    else {
        // Stop/coast: both low
        pwm_set_chan_level(slice_num, PWM_CHAN_A, 0);
        pwm_set_chan_level(slice_num, PWM_CHAN_B, 0);
    }
}

// Set left motor duty (signed %)
static void set_left_motor(int duty) {
    uint slice = pwm_gpio_to_slice_num(LEFT_IN1_PIN);
    set_motor_pwm(slice, LEFT_IN1_PIN, LEFT_IN2_PIN, duty);
}

// Set right motor duty (signed %)
static void set_right_motor(int duty) {
    uint slice = pwm_gpio_to_slice_num(RIGHT_IN1_PIN);
    set_motor_pwm(slice, RIGHT_IN1_PIN, RIGHT_IN2_PIN, duty);
}

int main() {
    stdio_init_all();
    // Wait for USB-CDC to connect (for debugging prints)
    while (!stdio_usb_connected()) {
        sleep_ms(50);
    }
    printf("\n3-Wheel Differential Drive Demo Starting\n");

    // ---- PWM Initialization for Left Motor (Slice 0) ----
    gpio_set_function(LEFT_IN1_PIN, GPIO_FUNC_PWM);
    gpio_set_function(LEFT_IN2_PIN, GPIO_FUNC_PWM);
    uint left_slice = pwm_gpio_to_slice_num(LEFT_IN1_PIN);

    // Explicitly set wrap so we know the PWM frequency (default divider = 1.0)
    // Here we use wrap=65535 → f_PWM ≈ 1.9 kHz
    pwm_set_wrap(left_slice, 65535);
    pwm_set_enabled(left_slice, true);

    // ---- PWM Initialization for Right Motor (Slice 1) ----
    gpio_set_function(RIGHT_IN1_PIN, GPIO_FUNC_PWM);
    gpio_set_function(RIGHT_IN2_PIN, GPIO_FUNC_PWM);
    uint right_slice = pwm_gpio_to_slice_num(RIGHT_IN1_PIN);

    // Use same wrap for the right motor
    pwm_set_wrap(right_slice, 65535);
    pwm_set_enabled(right_slice, true);

    // Brief pause so messages appear
    sleep_ms(200);
    printf("Both PWM slices initialized at ~1.9 kHz (wrap = 65535)\n");

    // --- Demo Sequence ---
    // 1) Drive forward at 50% for 2 s
    printf("Drive FORWARD at 50%% for 2 seconds\n");
    set_left_motor(50);
    set_right_motor(50);
    sleep_ms(2000);

    // 2) Stop for 1 s
    printf("Stop for 1 second\n");
    set_left_motor(0);
    set_right_motor(0);
    sleep_ms(1000);

    // 3) Drive backward at 50% for 2 s
    printf("Drive BACKWARD at 50%% for 2 seconds\n");
    set_left_motor(-50);
    set_right_motor(-50);
    sleep_ms(2000);

    // 4) Stop for 1 s
    printf("Stop for 1 second\n");
    set_left_motor(0);
    set_right_motor(0);
    sleep_ms(1000);

    // 5) Turn LEFT in place at 50% for 1.5 s
    //    (Left motor reverse, Right motor forward)
    printf("Turn LEFT in place at 50%% for 1.5 seconds\n");
    set_left_motor(-50);
    set_right_motor(50);
    sleep_ms(1500);

    // 6) Stop for 1 s
    printf("Stop for 1 second\n");
    set_left_motor(0);
    set_right_motor(0);
    sleep_ms(1000);

    // 7) Turn RIGHT in place at 50% for 1.5 s
    //    (Left motor forward, Right motor reverse)
    printf("Turn RIGHT in place at 50%% for 1.5 seconds\n");
    set_left_motor(50);
    set_right_motor(-50);
    sleep_ms(1500);

    // 8) Final stop
    printf("Final STOP\n");
    set_left_motor(0);
    set_right_motor(0);

    // End of demo; loop forever
    printf("Demo complete. Halting.\n");
    while (true) {
        tight_loop_contents();
    }

    return 0;
}
