#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdlib.h>
#include "ssd1306.h"

#define MPU6050_ADDR         0x68
#define MPU6050_WHO_AM_I     0x75
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_GYRO_CONFIG  0x1B
#define MPU6050_ACCEL_XOUT_H 0x3B

static const float ACCEL_SCALE = 16384.0f;  
static const float GYRO_SCALE  = 16.4f;     

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t temp;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} mpu6050_raw_t;

#define I2C_BUS    i2c0
#define SDA_PIN    0  
#define SCL_PIN    1  
#define I2C_BAUD   400000

static bool mpu6050_write_reg(uint8_t reg, uint8_t data) {
    uint8_t buf[2] = { reg, data };
    int ret = i2c_write_blocking(I2C_BUS, MPU6050_ADDR, buf, 2, false);
    return (ret == 2);
}

static bool mpu6050_read_reg(uint8_t reg, uint8_t *out) {
    int ret = i2c_write_blocking(I2C_BUS, MPU6050_ADDR, &reg, 1, true);
    if (ret != 1) return false;
    ret = i2c_read_blocking(I2C_BUS, MPU6050_ADDR, out, 1, false);
    return (ret == 1);
}

static bool mpu6050_read_all(mpu6050_raw_t *raw) {
    uint8_t cmd = MPU6050_ACCEL_XOUT_H;
    uint8_t buf[14];
    int ret = i2c_write_blocking(I2C_BUS, MPU6050_ADDR, &cmd, 1, true);
    if (ret != 1) return false;

    ret = i2c_read_blocking(I2C_BUS, MPU6050_ADDR, buf, 14, false);
    if (ret != 14) return false;

    raw->accel_x = (int16_t)((buf[0] << 8) | buf[1]);
    raw->accel_y = (int16_t)((buf[2] << 8) | buf[3]);
    raw->accel_z = (int16_t)((buf[4] << 8) | buf[5]);
    raw->temp    = (int16_t)((buf[6] << 8) | buf[7]);
    raw->gyro_x  = (int16_t)((buf[8] << 8) | buf[9]);
    raw->gyro_y  = (int16_t)((buf[10] << 8) | buf[11]);
    raw->gyro_z  = (int16_t)((buf[12] << 8) | buf[13]);

    return true;
}

static void mpu6050_init() {
    sleep_ms(100); 

    uint8_t who = 0;
    if (!mpu6050_read_reg(MPU6050_WHO_AM_I, &who) || (who != 0x68 && who != 0x98)) {
        while (true) {
            gpio_put(PICO_DEFAULT_LED_PIN, 1);
            sleep_ms(200);
            gpio_put(PICO_DEFAULT_LED_PIN, 0);
            sleep_ms(200);
        }
    }

    mpu6050_write_reg(MPU6050_PWR_MGMT_1, 0x00);
    sleep_ms(10);

    mpu6050_write_reg(MPU6050_ACCEL_CONFIG, 0x00);
    sleep_ms(10);

    mpu6050_write_reg(MPU6050_GYRO_CONFIG, 0x18);
    sleep_ms(10);
}


static void i2c_init_bus() {
    i2c_init(I2C_BUS, I2C_BAUD);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
}


static void draw_accel_vector_on_oled(int16_t raw_ax, int16_t raw_ay) {
    const int cx = SSD1306_WIDTH  / 2; // 64
    const int cy = SSD1306_HEIGHT / 2; // 32

    float ax_g = (float)raw_ax / ACCEL_SCALE; // ±1.0 at ±2g
    float ay_g = (float)raw_ay / ACCEL_SCALE;

    const float max_len = 30.0f;

    int ex = cx + (int)(ax_g * max_len);
    int ey = cy - (int)(ay_g * max_len); 

    ssd1306_clear();

    ssd1306_draw_line(cx, cy, ex, ey, 1);

    ssd1306_draw_line(cx - 5, cy, cx + 5, cy, 1);
    ssd1306_draw_line(cx, cy - 5, cx, cy + 5, 1);

    ssd1306_show();
}


int main() {
    stdio_init_all();  
    sleep_ms(200);   

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    i2c_init_bus();
    printf("Scanning I2C bus…\n");
    for (uint8_t addr = 1; addr < 127; addr++) {
        // try a zero-byte read; if it returns >= 0, a device ACK’d it
        int ret = i2c_read_blocking(I2C_BUS, addr, NULL, 0, false);
        if (ret >= 0) {
            printf("  Found device @ 0x%02X\n", addr);
        }
    }
    printf("Scan complete.\n");
    ssd1306_init();

    mpu6050_init();

    printf("MPU6050 + OLED demo: tilt to draw vectors\r\n");
    
    mpu6050_raw_t raw;
    while (true) {
        if (mpu6050_read_all(&raw)) {
            draw_accel_vector_on_oled(raw.accel_x, raw.accel_y);
        } else {
            gpio_put(PICO_DEFAULT_LED_PIN, 1);
            sleep_ms(10);
            gpio_put(PICO_DEFAULT_LED_PIN, 0);
        }
        sleep_ms(10); // ~100 Hz update
    }

    return 0;
}
