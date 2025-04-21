#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/spi.h"

#define SPI_PORT spi0
#define SCK_PIN_NUM 6
#define SDI_PIN_NUM 7
#define CS_PIN_NUM 9
#define PI 3.142
#define FREQ 2

void spi_initialize(void);
static inline void cs_select(uint cs_pin);
static inline void cs_deselect(uint cs_pin);
void writeDAC(int channel, float volts);

int main()
{
    stdio_init_all();
    spi_initialize();
    float t=0;
    float v = 0;

    while (1){
        for (int i=0;i<200;i++){
            v = 1.65*sin(FREQ*t*2.0*PI) + 1.65;
            writeDAC(0,v);
            v = fabs(3.3*(fmod(FREQ*t,2)-1));
            writeDAC(1,v);
            t=t+0.002;
            sleep_ms(2);
        }
        printf("Data written!\r\n");
        //sleep_ms(0.01);
    }
}

void spi_initialize(void){
    spi_init(SPI_PORT,1000*1000); // set the SPI 0 at 12 kHz and connect to GPIOs
    gpio_set_function(SCK_PIN_NUM, GPIO_FUNC_SPI); // set the SCK to GP6 (pin 9)
    gpio_set_function(SDI_PIN_NUM, GPIO_FUNC_SPI); // set the SDI to GP7 (pin 10) 

    gpio_init(CS_PIN_NUM); // set CS to GP9 (pin 12)
    gpio_set_dir(CS_PIN_NUM, GPIO_OUT); // set GP9 to be output pin for chip select
    gpio_put(CS_PIN_NUM,1); // initialize GP9 to high
}

static inline void cs_select(uint cs_pin) {
    asm volatile("nop \n nop \n nop"); // FIXME
    gpio_put(cs_pin, 0);
    asm volatile("nop \n nop \n nop"); // FIXME
}

static inline void cs_deselect(uint cs_pin) {
    asm volatile("nop \n nop \n nop"); // FIXME
    gpio_put(cs_pin, 1);
    asm volatile("nop \n nop \n nop"); // FIXME
}

void writeDAC(int channel, float volts){
    uint8_t data[2];
    int len = 2;

    if (volts < 0){
        volts = 0;
    }

    if (volts > 3.3){
        volts = 3.3;
    }

    uint16_t v_bin = (uint16_t)((volts / 3.3) * 1023.0);
    
    data[0] = 0;
    data[1] = 0;
    data[0] = data[0] | (channel<<7);
    data[0] = data[0] | (0b111<<4);
    data[0] = data[0] | (v_bin>>6);
    data[1] = data[1] | (v_bin << 2);

    //data[0] = 0b01111000;
    //data[1] = 0b00000000;

    cs_select(CS_PIN_NUM);
    spi_write_blocking(SPI_PORT, data, len); // where data is a uint8_t array with length len
    cs_deselect(CS_PIN_NUM);
}