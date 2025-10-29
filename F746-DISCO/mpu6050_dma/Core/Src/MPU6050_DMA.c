/*
 * MPU6050_DMA.c
 *
 *  Created on: Oct 22, 2025
 *      Author: ADMIN
 */
#include "MPU6050_DMA.h"
#include "main.h"
#include <string.h>

/* Externals created by CubeMX in main.c / main.h */
extern I2C_HandleTypeDef hi2c1;

/* DMA buffer and flags */
static uint8_t mpu_dma_buf[14];
static volatile uint8_t mpu_dma_done = 0;
static volatile uint8_t mpu_dma_error = 0;
static MPU6050_Data_t mpu_last_data;

/* Sensitivity constants for FS = ±2g and ±250 dps (as in init) */
static const float ACC_LSB_PER_G = 16384.0f;    // ±2g
static const float GYRO_LSB_PER_DPS = 131.0f;   // ±250°/s

int MPU6050_DMA_Init(void)
{
    uint8_t who = 0;
    HAL_StatusTypeDef hs;

    /* read WHO_AM_I (blocking) */
    hs = HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &who, 1, 200);
    if (hs != HAL_OK) return -1;
    if (who != 0x68) return -2;

    /* Wake up device */
    uint8_t data;
    data = 0x00; // clear sleep bit
    hs = HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &data, 1, 200);
    if (hs != HAL_OK) return -3;
    HAL_Delay(20);

    /* sample rate divider for 1kHz sample: SMPLRT_DIV = 7 -> 8k/(1+7)=1k */
    data = 0x07;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_SMPRT_DIV, I2C_MEMADD_SIZE_8BIT, &data, 1, 200);

    /* config: DLPF off */
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_CONFIG, I2C_MEMADD_SIZE_8BIT, &data, 1, 200);

    /* gyro FS = ±250 dps */
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_GYRO_CONFIG, I2C_MEMADD_SIZE_8BIT, &data, 1, 200);

    /* accel FS = ±2g */
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, &data, 1, 200);

    /* enable Data Ready interrupt (optional) */
    data = 0x01; // data ready
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_INT_ENABLE, I2C_MEMADD_SIZE_8BIT, &data, 1, 200);

    HAL_Delay(10);
    mpu_dma_done = 0;
    mpu_dma_error = 0;
    memset(&mpu_last_data, 0, sizeof(mpu_last_data));

    return 0;
}

HAL_StatusTypeDef MPU6050_DMA_StartRead(void)
{
    mpu_dma_done = 0;
    mpu_dma_error = 0;
    /* Non-blocking DMA read 14 bytes */
    return HAL_I2C_Mem_Read_DMA(&hi2c1, MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, mpu_dma_buf, 14);
}

uint8_t MPU6050_DMA_DataReady(void)
{
    return mpu_dma_done;
}

void MPU6050_DMA_ProcessData(MPU6050_Data_t *out)
{
    if (!mpu_dma_done) return;

    /* copy raw */
    out->acc_x_raw = (int16_t)((mpu_dma_buf[0] << 8) | mpu_dma_buf[1]);
    out->acc_y_raw = (int16_t)((mpu_dma_buf[2] << 8) | mpu_dma_buf[3]);
    out->acc_z_raw = (int16_t)((mpu_dma_buf[4] << 8) | mpu_dma_buf[5]);
    out->temp_raw  = (int16_t)((mpu_dma_buf[6] << 8) | mpu_dma_buf[7]);
    out->gyro_x_raw = (int16_t)((mpu_dma_buf[8] << 8) | mpu_dma_buf[9]);
    out->gyro_y_raw = (int16_t)((mpu_dma_buf[10] << 8) | mpu_dma_buf[11]);
    out->gyro_z_raw = (int16_t)((mpu_dma_buf[12] << 8) | mpu_dma_buf[13]);

    /* convert */
    out->acc_x_g = ((float)out->acc_x_raw) / ACC_LSB_PER_G;
    out->acc_y_g = ((float)out->acc_y_raw) / ACC_LSB_PER_G;
    out->acc_z_g = ((float)out->acc_z_raw) / ACC_LSB_PER_G;

    out->temperature_c = ((float)out->temp_raw) / 340.0f + 36.53f;

    out->gyro_x_dps = ((float)out->gyro_x_raw) / GYRO_LSB_PER_DPS;
    out->gyro_y_dps = ((float)out->gyro_y_raw) / GYRO_LSB_PER_DPS;
    out->gyro_z_dps = ((float)out->gyro_z_raw) / GYRO_LSB_PER_DPS;

    /* store last */
    memcpy((void*)&mpu_last_data, out, sizeof(MPU6050_Data_t));

    /* clear done flag if you want single-shot; keep it set for consumer to read */
    // mpu_dma_done = 0; // do not clear here; consumer may clear or call StartRead again
}

MPU6050_Data_t *MPU6050_DMA_GetLastData(void)
{
    return (MPU6050_Data_t *)&mpu_last_data;
}

/* This function should be called from HAL_I2C_MemRxCpltCallback in stm32xx_hal_msp.c or weak callback */
void MPU6050_DMA_I2C_RxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == &hi2c1) {
        mpu_dma_done = 1;
    }
}

/* Optional: error callback */
void MPU6050_DMA_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == &hi2c1) {
        mpu_dma_error = 1;
        mpu_dma_done = 0;
    }
}


