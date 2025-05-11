#ifndef SHARED_H
#define SHARED_H

#include <stdint.h>

#define FLAG_READ_ADC 0
#define FLAG_LED_ON   1
#define FLAG_LED_OFF  2
#define FLAG_READY    3

extern volatile float adc_voltage;

#endif
