#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "shared.h"

void core1_entry();

int main() {
    stdio_init_all();
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }

    multicore_launch_core1(core1_entry);

    char cmd;
    while (true) {
        printf("\nEnter command:\n 0: read A0\n 1: LED ON\n 2: LED OFF\n> ");
        scanf(" %c", &cmd);

        if (cmd == '0') {
            multicore_fifo_push_blocking(FLAG_READ_ADC);
            multicore_fifo_pop_blocking(); // wait for Core 1
            printf("Voltage on A0: %.2f V\n", adc_voltage);
        } else if (cmd == '1') {
            multicore_fifo_push_blocking(FLAG_LED_ON);
        } else if (cmd == '2') {
            multicore_fifo_push_blocking(FLAG_LED_OFF);
        } else {
            printf("Invalid command.\n");
        }
    }
}
