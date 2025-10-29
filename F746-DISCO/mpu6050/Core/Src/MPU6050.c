/*
 * MPU6050.c
 *
 *  Created on: Oct 22, 2025
 *      Author: ADMIN
 */
#include "MPU6050.h"
#include "main.h"
#include "stdio.h"
#include "string.h"

MPU6050_Data_t MPU6050_Data;

static const float ACC_SENS_2G = 16384.0f;  // LSB/g for ±2g
static const float GYRO_SENS_250DPS = 131.0f; // LSB/(°/s) for ±250 dps

// Helper: read bytes
static HAL_StatusTypeDef mpu_read_bytes(uint8_t reg, uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 1000);
}

// Helper: write byte
static HAL_StatusTypeDef mpu_write_byte(uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 500);
}

int MPU6050_Init(void)
{
    uint8_t who = 0;
    // read WHO_AM_I
    if (mpu_read_bytes(MPU6050_WHO_AM_I, &who, 1) != HAL_OK) {
        return -1;
    }
    if (who != 0x68) {
        return -2;
    }

    // Wake up device: write 0 to PWR_MGMT_1
    if (mpu_write_byte(MPU6050_PWR_MGMT_1, 0x00) != HAL_OK) return -3;
    HAL_Delay(50);

    // Sample rate divider (SMPLRT_DIV) -> sample rate = GyroRate/(1+SMPLRT_DIV)
    // If DLPF disabled (default), gyro output rate = 8kHz. To get 1kHz -> SMPLRT_DIV=7
    mpu_write_byte(MPU6050_SMPRT_DIV, 0x07);

    // DLPF config (CONFIG) -> keep 0 (DLPF disabled)
    mpu_write_byte(MPU6050_CONFIG, 0x00);

    // Gyro full scale ±250 dps
    mpu_write_byte(MPU6050_GYRO_CONFIG, 0x00);

    // Accel full scale ±2g
    mpu_write_byte(MPU6050_ACCEL_CONFIG, 0x00);

    HAL_Delay(50);

    return 0;
}

int MPU6050_ReadAll(MPU6050_Data_t *out)
{
    uint8_t buf[14];
    if (mpu_read_bytes(MPU6050_ACCEL_XOUT_H, buf, 14) != HAL_OK) {
        return -1;
    }

    out->acc_x_raw = (int16_t)((buf[0] << 8) | buf[1]);
    out->acc_y_raw = (int16_t)((buf[2] << 8) | buf[3]);
    out->acc_z_raw = (int16_t)((buf[4] << 8) | buf[5]);

    out->temp_raw  = (int16_t)((buf[6] << 8) | buf[7]);

    out->gyro_x_raw = (int16_t)((buf[8]  << 8) | buf[9]);
    out->gyro_y_raw = (int16_t)((buf[10] << 8) | buf[11]);
    out->gyro_z_raw = (int16_t)((buf[12] << 8) | buf[13]);

    // Convert to physical units
    out->acc_x_g = ((float)out->acc_x_raw) / ACC_SENS_2G;
    out->acc_y_g = ((float)out->acc_y_raw) / ACC_SENS_2G;
    out->acc_z_g = ((float)out->acc_z_raw) / ACC_SENS_2G;

    out->temperature_c = ((float)out->temp_raw) / 340.0f + 36.53f;

    out->gyro_x_dps = ((float)out->gyro_x_raw) / GYRO_SENS_250DPS;
    out->gyro_y_dps = ((float)out->gyro_y_raw) / GYRO_SENS_250DPS;
    out->gyro_z_dps = ((float)out->gyro_z_raw) / GYRO_SENS_250DPS;

    return 0;
}



