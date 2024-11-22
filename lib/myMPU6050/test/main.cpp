#include "esp32-hal-i2c.h"
#include <stdio.h>

#define MPU6050_I2C_ADDR 0x68
#define MPU6050_REG_PWR_MGMT_1 0x6B
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_TEMP_OUT_H 0x41
#define MPU6050_REG_GYRO_XOUT_H 0x43

// Function to initialize the MPU6050
esp_err_t mpu6050_init(uint8_t i2c_num) {
    uint8_t data = 0;
    esp_err_t err;

    // Wake up the MPU6050 (clear sleep bit)
    data = 0x00;
    err = i2cWrite(i2c_num, MPU6050_I2C_ADDR, &data, 1, MPU6050_REG_PWR_MGMT_1);
    if (err != ESP_OK) {
        printf("Failed to write to PWR_MGMT_1 register\n");
        return err;
    }

    return ESP_OK;
}

// Function to read raw temperature data
esp_err_t mpu6050_read_temperature(uint8_t i2c_num, int16_t *temp_out) {
    uint8_t temp_data[2];
    esp_err_t err;

    err = i2cRead(i2c_num, MPU6050_I2C_ADDR, temp_data, 2, MPU6050_REG_TEMP_OUT_H, NULL);
    if (err != ESP_OK) {
        printf("Failed to read temperature\n");
        return err;
    }

    *temp_out = ((int16_t)temp_data[0] << 8) | temp_data[1];
    return ESP_OK;
}

// Function to read accelerometer data
esp_err_t mpu6050_read_accelerometer(uint8_t i2c_num, int16_t *accel_x, int16_t *accel_y, int16_t *accel_z) {
    uint8_t accel_data[6];
    esp_err_t err;

    err = i2cRead(i2c_num, MPU6050_I2C_ADDR, accel_data, 6, MPU6050_REG_ACCEL_XOUT_H, NULL);
    if (err != ESP_OK) {
        printf("Failed to read accelerometer\n");
        return err;
    }

    *accel_x = ((int16_t)accel_data[0] << 8) | accel_data[1];
    *accel_y = ((int16_t)accel_data[2] << 8) | accel_data[3];
    *accel_z = ((int16_t)accel_data[4] << 8) | accel_data[5];

    return ESP_OK;
}

// Function to read gyroscope data
esp_err_t mpu6050_read_gyroscope(uint8_t i2c_num, int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z) {
    uint8_t gyro_data[6];
    esp_err_t err;

    err = i2cRead(i2c_num, MPU6050_I2C_ADDR, gyro_data, 6, MPU6050_REG_GYRO_XOUT_H, NULL);
    if (err != ESP_OK) {
        printf("Failed to read gyroscope\n");
        return err;
    }

    *gyro_x = ((int16_t)gyro_data[0] << 8) | gyro_data[1];
    *gyro_y = ((int16_t)gyro_data[2] << 8) | gyro_data[3];
    *gyro_z = ((int16_t)gyro_data[4] << 8) | gyro_data[5];

    return ESP_OK;
}
