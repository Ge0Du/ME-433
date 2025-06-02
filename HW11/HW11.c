/*
 * Pico USB Mouse Example with Two Modes
 * - Manual: 4-directional buttons control the cursor
 * - Circle: automatic circular motion
 * Mode switch toggled by a fifth button, with an LED indicator.
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "bsp/board_api.h"
#include "tusb.h"
#include "hardware/gpio.h"
#include "usb_descriptors.h"

// Button and LED pin assignments
#define UP_BUTTON    15
#define DOWN_BUTTON  17
#define LEFT_BUTTON  14
#define RIGHT_BUTTON 16
#define MODE_BUTTON  13
#define MODE_LED     25

// Mouse deltas
static int8_t x_delta = 0;
static int8_t y_delta = 0;

// Mode state
static bool circle_mode = false;
static absolute_time_t circle_start_time;

// Button press timestamps for acceleration
typedef struct {
  bool pressed;
  absolute_time_t press_time;
} button_state_t;

button_state_t up_btn, down_btn, left_btn, right_btn;

// Function Prototypes
void init_gpio(void);
void update_manual_deltas(void);
void update_circle_deltas(void);
void hid_task(void);

int main(void) {
  board_init();
  tud_init(BOARD_TUD_RHPORT);

  init_gpio();

  // Wait for USB enumeration
  puts("DEBUG: Waiting for USB connection...");
  while (!tud_mounted()) {
    tud_task();
    sleep_ms(10);
  }
  puts("DEBUG: USB connected!");

  while (1) {
    tud_task();

    // Mode toggle
    if (!gpio_get(MODE_BUTTON)) {
      sleep_ms(30); // Debounce
      if (!gpio_get(MODE_BUTTON)) {
        circle_mode = !circle_mode;
        gpio_put(MODE_LED, circle_mode);
        circle_start_time = get_absolute_time();

        // Reset deltas when switching modes
        x_delta = 0;
        y_delta = 0;

        printf("DEBUG: Switched to %s mode\n", circle_mode ? "CIRCLE" : "MANUAL");

        // Wait for release
        while (!gpio_get(MODE_BUTTON)) {}
        sleep_ms(30);
      }
    }

    // Update deltas based on mode
    if (circle_mode) {
      update_circle_deltas();
    } else {
      update_manual_deltas();
    }

    hid_task();
  }
}

void init_gpio(void) {
  // Initialize button GPIOs
  uint pins[] = { UP_BUTTON, DOWN_BUTTON, LEFT_BUTTON, RIGHT_BUTTON, MODE_BUTTON };
  for (int i = 0; i < 5; i++) {
    gpio_init(pins[i]);
    gpio_set_dir(pins[i], GPIO_IN);
    gpio_pull_up(pins[i]);
  }

  // Initialize mode LED
  gpio_init(MODE_LED);
  gpio_set_dir(MODE_LED, GPIO_OUT);

  puts("DEBUG: GPIO initialized");
}

void update_manual_deltas(void) {
  // Check each button
  struct {
    uint pin;
    int8_t *delta;
    int8_t direction; // +1 or -1
    button_state_t *state;
    const char *name;
  } buttons[] = {
    { UP_BUTTON, &y_delta, -1, &up_btn, "UP" },
    { DOWN_BUTTON, &y_delta,  1, &down_btn, "DOWN" },
    { LEFT_BUTTON, &x_delta, -1, &left_btn, "LEFT" },
    { RIGHT_BUTTON, &x_delta, 1, &right_btn, "RIGHT" },
  };

  for (int i = 0; i < 4; i++) {
    bool pressed = (gpio_get(buttons[i].pin) == 0);

    if (pressed && !buttons[i].state->pressed) {
      // Just pressed
      *(buttons[i].delta) = buttons[i].direction;
      buttons[i].state->press_time = get_absolute_time();
      buttons[i].state->pressed = true;

      printf("DEBUG: Button %s pressed\n", buttons[i].name);
    } else if (pressed && buttons[i].state->pressed) {
      // Held down — acceleration
      int elapsed = absolute_time_diff_us(buttons[i].state->press_time, get_absolute_time()) / 1000;
      int8_t level = 1;

      if (elapsed > 1500) level = 4;
      else if (elapsed > 1000) level = 3;
      else if (elapsed > 500) level = 2;

      *(buttons[i].delta) = buttons[i].direction * level;
    } else if (!pressed && buttons[i].state->pressed) {
      // Released
      *(buttons[i].delta) = 0;
      buttons[i].state->pressed = false;

      printf("DEBUG: Button %s released\n", buttons[i].name);
    }
  }

  // Debug cursor deltas
  if (x_delta || y_delta) {
    printf("DEBUG: Cursor delta: x=%d, y=%d\n", x_delta, y_delta);
  }
}

void update_circle_deltas(void) {
  int elapsed_ms = to_ms_since_boot(get_absolute_time()) - to_ms_since_boot(circle_start_time);
  float angle = (elapsed_ms / 1000.0f); // Radians per second

  x_delta = (int8_t)(3 * cosf(angle));
  y_delta = (int8_t)(3 * sinf(angle));

  printf("DEBUG: Circle mode deltas: x=%d, y=%d\n", x_delta, y_delta);
}

// USB HID reporting
void send_hid_report(uint8_t report_id) {
  if (!tud_hid_ready()) return;

  switch (report_id) {
    case REPORT_ID_MOUSE:
      tud_hid_mouse_report(REPORT_ID_MOUSE, 0x00, x_delta, y_delta, 0, 0);
      break;
    default:
      break;
  }
}

void hid_task(void) {
  const uint32_t interval_ms = 10;
  static uint32_t last_ms = 0;
  if (board_millis() - last_ms < interval_ms) return;
  last_ms += interval_ms;

  send_hid_report(REPORT_ID_MOUSE);
}

// USB callbacks (unchanged from default examples)
void tud_mount_cb(void)     { puts("DEBUG: USB mounted"); }
void tud_umount_cb(void)   { puts("DEBUG: USB unmounted"); }
void tud_suspend_cb(bool remote) { (void)remote; puts("DEBUG: USB suspended"); }
void tud_resume_cb(void)   { puts("DEBUG: USB resumed"); }
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer, uint16_t bufsize) { (void)instance; (void)report_id; (void)report_type; (void)buffer; (void)bufsize; }
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen) { (void)instance; (void)report_id; (void)report_type; (void)buffer; (void)reqlen; return 0; }
