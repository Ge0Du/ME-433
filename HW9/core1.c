#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "pico/multicore.h"
#include "shared.h"

volatile float adc_voltage = 0;

void core1_entry() {
    // Init ADC
    adc_init();
    adc_gpio_init(26); // A0 is GPIO26
    adc_select_input(0); // Select ADC channel 0 (A0)

    // Init LED
    gpio_init(15);
    gpio_set_dir(15, GPIO_OUT);
    gpio_put(15, 0);

    while (true) {
        uint32_t cmd = multicore_fifo_pop_blocking();

        if (cmd == FLAG_READ_ADC) {
            uint16_t raw = adc_read();
            adc_voltage = (3.3f * raw) / 4095.0f;
            multicore_fifo_push_blocking(FLAG_READY);
        } else if (cmd == FLAG_LED_ON) {
            gpio_put(15, 1);
        } else if (cmd == FLAG_LED_OFF) {
            gpio_put(15, 0);
        }
    }
}
