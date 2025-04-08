#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"

int main() {
    stdio_init_all();
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }
    printf("Start!\n");
    gpio_init(12); // LED PIN NEED TO UPDATE TO IRL
    gpio_init(13); // BUTTON PIN NEED TO UPDATE TO IRL
    gpio_set_dir(12, GPIO_OUT);
    gpio_set_dir(13, GPIO_IN);
    
    adc_init(); // init the adc module
    adc_gpio_init(26); // set ADC0 pin to be adc input instead of GPIO
    adc_select_input(0); // select to read from ADC0


    gpio_put(12, 1);
    while (!gpio_get(13)) {
        sleep_ms(10);
    }
    char message[100];
    while (1) {
        printf("Please enter a number of analog samples to take, from 1 to 100: \r\n ");
        scanf("%s", message);
        int number = atoi(message);
        for (int i = 0; i < number; i ++) {
            uint16_t result = adc_read();
            float voltage = result * 3.3f / 4095.0f;
            printf("Sample %d: %.3f V\n", i + 1, voltage);
            sleep_ms(10);
       }
    }
}