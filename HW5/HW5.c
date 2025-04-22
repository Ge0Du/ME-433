#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"

#define SPI_PORT spi0
#define PIN_SCK  6
#define PIN_MOSI 7
#define PIN_MISO 4
#define DAC_PIN_CS 9

#define WRITE_CMD 0x02
#define READ_CMD  0x03
#define MODE_CMD  0x01
#define SEQ_MODE  0x40

#define SRAM_PIN_CS 10
#define VREF 3.3f

union FloatBytes {
    float f;
    uint8_t b[4];
};

void cs_select(uint cs_pin) {
    gpio_put(cs_pin, 0);
}

void cs_deselect(uint cs_pin) {
    gpio_put(cs_pin, 1);
}

void spi_init_all() {
    spi_init(SPI_PORT, 1 * 1000 * 1000); // 1 MHz
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);

    gpio_init(SRAM_PIN_CS);
    gpio_set_dir(SRAM_PIN_CS, GPIO_OUT);
    gpio_put(SRAM_PIN_CS, 1);

    gpio_init(DAC_PIN_CS);
    gpio_set_dir(DAC_PIN_CS, GPIO_OUT);
    gpio_put(DAC_PIN_CS, 1);

    printf("SPI initialized.\n");
}

void spi_ram_init() {
    cs_select(SRAM_PIN_CS);
    uint8_t cmd[] = { MODE_CMD, SEQ_MODE };
    spi_write_blocking(SPI_PORT, cmd, 2);
    cs_deselect(SRAM_PIN_CS);
    printf("SRAM set to sequential mode.\n");
}

void spi_ram_write_float(uint16_t addr, float val) {
    union FloatBytes fb;
    fb.f = val;
    uint8_t cmd[3] = { WRITE_CMD, addr >> 8, addr & 0xFF };

    cs_select(SRAM_PIN_CS);
    spi_write_blocking(SPI_PORT, cmd, 3);
    spi_write_blocking(SPI_PORT, fb.b, 4);
    cs_deselect(SRAM_PIN_CS);

    printf("Wrote float %.4f to SRAM address 0x%04X\n", val, addr);
}

float spi_ram_read_float(uint16_t addr) {
    union FloatBytes fb;
    uint8_t cmd[3] = { READ_CMD, addr >> 8, addr & 0xFF };

    cs_select(SRAM_PIN_CS);
    spi_write_blocking(SPI_PORT, cmd, 3);
    spi_read_blocking(SPI_PORT, 0, fb.b, 4);
    cs_deselect(SRAM_PIN_CS);

    printf("Read float %.4f from SRAM address 0x%04X\n", fb.f, addr);
    return fb.f;
}

void writeDAC(int channel, float volts) {
    if (volts < 0) volts = 0;
    if (volts > VREF) volts = VREF;

    uint16_t val = (uint16_t)((volts / VREF) * 4095.0f);
    uint16_t packet = 0b0011000000000000 | (channel << 15) | (val & 0x0FFF);
    uint8_t buf[2] = { packet >> 8, packet & 0xFF };

    cs_select(DAC_PIN_CS);
    spi_write_blocking(SPI_PORT, buf, 2);
    cs_deselect(DAC_PIN_CS);

    printf("DAC write: Channel %d, Voltage %.4f V (Raw: %u)\n", channel, volts, val);
}

int main() {
    stdio_init_all();
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }
    printf("Pico started. Initializing peripherals...\n");

    spi_init_all();
    spi_ram_init();

    const int num_samples = 1000;
    printf("Writing %d sine wave samples to external SRAM...\n", num_samples);

    for (int i = 0; i < num_samples; i++) {
        float angle = 2.0f * M_PI * i / num_samples;
        float v = 1.65f * sinf(angle) + 1.65f;
        spi_ram_write_float(i * 4, v);
    }

    printf("Sine wave data preloaded.\n");
    while (true) {
        printf("Starting 1Hz sine wave output cycle...\n");
        for (int i = 0; i < num_samples; i++) {
            float v = spi_ram_read_float(i * 4);
            writeDAC(0, v);
            if (i % 200 == 0) {
                printf("Sample %4d: %.4f V\n", i, v);
            }
            sleep_us(1000); // 1ms per sample
        }
        printf("Completed one full sine wave cycle.\n");
    }

}