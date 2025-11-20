/*
 * MPU6050_dma.c
 *
 *  Created on: Nov 10, 2025
 *      Author: ADMIN
 */
#include "MPU6050_dma.h"
#include "main.h"
#include <string.h>
#include "cmsis_os2.h"

extern I2C_HandleTypeDef hi2c1;

static uint8_t mpu_dma_buf[14];
static volatile uint8_t mpu_dma_done = 0;
static volatile uint8_t mpu_dma_error = 0;
static volatile uint8_t mpu_in_progress = 0;
static MPU6050_Data_t mpu_last_data;

/* Sensitivity constants for FS = ±2g and ±250 dps (as in init) */
static const float ACC_LSB_PER_G = 16384.0f;    // ±2g
static const float GYRO_LSB_PER_DPS = 131.0f;   // ±250°/s

static osSemaphoreId_t mpu_sem = NULL;
static osMutexId_t mpu_i2c_mutex = NULL;

int MPU6050_DMA_Init(void)
{
    uint8_t who = 0;
    HAL_StatusTypeDef hs;

    hs = HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &who, 1, 200);
    if (hs != HAL_OK) return -1;
    if (who != 0x68) return -2;

    uint8_t data;
    data = 0x00;
    hs = HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &data, 1, 200);
    if (hs != HAL_OK) return -3;
    HAL_Delay(20);

    data = 0x07;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_SMPRT_DIV, I2C_MEMADD_SIZE_8BIT, &data, 1, 200);
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_CONFIG, I2C_MEMADD_SIZE_8BIT, &data, 1, 200);
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_GYRO_CONFIG, I2C_MEMADD_SIZE_8BIT, &data, 1, 200);
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, &data, 1, 200);

    data = 0x01;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_INT_ENABLE, I2C_MEMADD_SIZE_8BIT, &data, 1, 200);

    HAL_Delay(10);
    mpu_dma_done = 0;
    mpu_dma_error = 0;
    memset(&mpu_last_data, 0, sizeof(mpu_last_data));
    return 0;
}
void MPU6050_SetSemaphore(osSemaphoreId_t sem) { mpu_sem = sem; }

void MPU6050_SetI2CMutex(osMutexId_t m) { mpu_i2c_mutex = m; }

HAL_StatusTypeDef MPU6050_DMA_StartRead(void)
{
    mpu_dma_done = 0;
    mpu_dma_error = 0;
    mpu_in_progress = 1;
    if (mpu_i2c_mutex) {
    	if (osMutexAcquire(mpu_i2c_mutex, 100) != osOK) return HAL_ERROR;
    }
    return HAL_I2C_Mem_Read_DMA(&hi2c1, MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, mpu_dma_buf, 14);
}

uint8_t MPU6050_DMA_DataReady(void) { return mpu_dma_done; }

void MPU6050_DMA_ProcessData(MPU6050_Data_t *out)
{
    if (!mpu_dma_done) return;
    out->acc_x_raw = (int16_t)((mpu_dma_buf[0] << 8) | mpu_dma_buf[1]);
    out->acc_y_raw = (int16_t)((mpu_dma_buf[2] << 8) | mpu_dma_buf[3]);
    out->acc_z_raw = (int16_t)((mpu_dma_buf[4] << 8) | mpu_dma_buf[5]);
    out->temp_raw  = (int16_t)((mpu_dma_buf[6] << 8) | mpu_dma_buf[7]);
    out->gyro_x_raw = (int16_t)((mpu_dma_buf[8] << 8) | mpu_dma_buf[9]);
    out->gyro_y_raw = (int16_t)((mpu_dma_buf[10] << 8) | mpu_dma_buf[11]);
    out->gyro_z_raw = (int16_t)((mpu_dma_buf[12] << 8) | mpu_dma_buf[13]);

    out->acc_x_g = ((float)out->acc_x_raw) / ACC_LSB_PER_G;
    out->acc_y_g = ((float)out->acc_y_raw) / ACC_LSB_PER_G;
    out->acc_z_g = ((float)out->acc_z_raw) / ACC_LSB_PER_G;

    out->temperature_c = ((float)out->temp_raw) / 340.0f + 36.53f;

    out->gyro_x_dps = ((float)out->gyro_x_raw) / GYRO_LSB_PER_DPS;
    out->gyro_y_dps = ((float)out->gyro_y_raw) / GYRO_LSB_PER_DPS;
    out->gyro_z_dps = ((float)out->gyro_z_raw) / GYRO_LSB_PER_DPS;

    memcpy((void*)&mpu_last_data, out, sizeof(MPU6050_Data_t));
    /* leave mpu_dma_done set; consumer may clear */
}

MPU6050_Data_t *MPU6050_DMA_GetLastData(void) { return (MPU6050_Data_t *)&mpu_last_data; }

/* Called from main HAL_I2C_MemRxCpltCallback forwarding */
void MPU6050_DMA_I2C_RxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if (mpu_i2c_mutex) osMutexRelease(mpu_i2c_mutex);
    (void)hi2c;
    if (!mpu_in_progress) return;
    mpu_in_progress = 0;
    mpu_dma_done = 1;
    if (mpu_sem) osSemaphoreRelease(mpu_sem);
}

/* Error */
void MPU6050_DMA_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
	if (mpu_i2c_mutex) osMutexRelease(mpu_i2c_mutex);
    (void)hi2c;
    mpu_in_progress = 0;
    mpu_dma_error = 1;
    if (mpu_sem) osSemaphoreRelease(mpu_sem);
}
