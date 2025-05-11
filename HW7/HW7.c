#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "ssd1306.h"
#include "font.h"

// I2C defines
// This example will use I2C0 on GPIO8 (SDA) and GPIO9 (SCL) running at 400KHz.
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define I2C_PORT i2c0
#define I2C_SDA 12
#define I2C_SCL 13

// Prototypes
void drawMessage(int x, int y, char * m);
void drawLetter(int x, int y, char c);

int main()
{
    stdio_init_all();

    // I2C Initialisation. Using it at 400Khz.
    i2c_init(I2C_PORT, 400*1000);
    
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    // For more examples of I2C use see https://github.com/raspberrypi/pico-examples/tree/master/i2c

    // Initialises ssd1306 and clears the screen
    ssd1306_setup();
    ssd1306_clear();
    ssd1306_update();

    // Initialise the ADC0
    adc_init(); // init the adc module
    adc_gpio_init(28); // set ADC0 pin to be adc input instead of GPIO
    adc_select_input(2); // select to read from ADC0

    while (true) {
        int i = 1;
        while (1) {
            float time_passed_in_seconds = to_us_since_boot(get_absolute_time()) / 1000000.0;
            float fps = i / time_passed_in_seconds;
            float voltage = (adc_read() / 4096.0) * 3.3;
            char message[30];

            sprintf(message, "Voltage = %f", voltage);
            drawMessage(5,5,message);

            sprintf(message, "fps = %f", fps);
            drawMessage(5,20,message);
            ssd1306_update();

            i++;
        }
        char message[50];
        sprintf(message, "hello world");
        drawMessage(10,20,message);
        ssd1306_update();
        sleep_ms(1000);
    }
}

void drawMessage(int x, int y, char * m) // m is array of characters in message
{
    int i = 0;
    while (m[i] != 0){ // At the end of the sprintf, there is a "null character = 0", so this detects that
        drawLetter(x + (i * 6), y, m[i]); // Using i*6 so that the letters have more seperation
        i++;
    }
}

void drawLetter(int x, int y, char c){
    for (int j=0; j<5; j++){ // Iterates through the columms from left to right
        char col = ASCII[c-0x20] [j]; // -0x20 because the ASCII table starts at 0x20
        for (int i =0; i<8; i++){ // Iterates through the rows from top to bottom
            char bit = (col >>i) & 0b1; // For each bit, right shift and & it to check if it's 1
            ssd1306_drawPixel(x+j, y+i, bit); // Add the bit to draw pixel.
        }
    }
}