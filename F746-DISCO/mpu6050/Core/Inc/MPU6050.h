/*
 * MPU6050.h
 *
 *  Created on: Oct 22, 2025
 *      Author: ADMIN
 */

#ifndef __MPU6050_H__
#define __MPU6050_H__

#include "main.h"
#include <stdint.h>

#define MPU6050_ADDR        (0x68 << 1)   // 7-bit 0x68 => 8-bit HAL address 0xD0 (shifted left)
#define MPU6050_WHO_AM_I    0x75
#define MPU6050_PWR_MGMT_1  0x6B
#define MPU6050_SMPRT_DIV   0x19
#define MPU6050_CONFIG      0x1A
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_ACCEL_XOUT_H 0x3B

typedef struct {
    int16_t acc_x_raw;
    int16_t acc_y_raw;
    int16_t acc_z_raw;
    int16_t temp_raw;
    int16_t gyro_x_raw;
    int16_t gyro_y_raw;
    int16_t gyro_z_raw;

    float acc_x_g;
    float acc_y_g;
    float acc_z_g;
    float temperature_c;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
} MPU6050_Data_t;

extern MPU6050_Data_t MPU6050_Data;

// Init and read APIs
int MPU6050_Init(void); // returns 0 on success, non-zero on fail
int MPU6050_ReadAll(MPU6050_Data_t *out); // reads raw data and converts to physical units

#endif // __MPU6050_H__
