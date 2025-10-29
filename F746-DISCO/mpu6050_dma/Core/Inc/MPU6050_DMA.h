/*
 * MPU6050_DMA.h
 *
 *  Created on: Oct 22, 2025
 *      Author: ADMIN
 */

#ifndef __MPU6050_DMA_H__
#define __MPU6050_DMA_H__

#include "main.h"
#include <stdint.h>

#define MPU6050_ADDR            0xD0  // 0x68 << 1 (AD0 = GND)
#define MPU6050_WHO_AM_I        0x75
#define MPU6050_PWR_MGMT_1      0x6B
#define MPU6050_SMPRT_DIV       0x19
#define MPU6050_CONFIG          0x1A
#define MPU6050_GYRO_CONFIG     0x1B
#define MPU6050_ACCEL_CONFIG    0x1C
#define MPU6050_INT_ENABLE      0x38
#define MPU6050_INT_STATUS      0x3A
#define MPU6050_ACCEL_XOUT_H    0x3B

typedef struct {
    int16_t acc_x_raw, acc_y_raw, acc_z_raw;
    int16_t temp_raw;
    int16_t gyro_x_raw, gyro_y_raw, gyro_z_raw;

    float acc_x_g, acc_y_g, acc_z_g;
    float temperature_c;
    float gyro_x_dps, gyro_y_dps, gyro_z_dps;
} MPU6050_Data_t;

/* Initialize sensor (blocking config writes). Returns 0 ok, negative error */
int MPU6050_DMA_Init(void);

/* Start a non-blocking DMA read of 14 bytes (ACCEL..GYRO). Returns HAL status. */
HAL_StatusTypeDef MPU6050_DMA_StartRead(void);

/* Query if DMA read completed (set by callback). */
uint8_t MPU6050_DMA_DataReady(void);

/* Process raw DMA buffer -> fill out structure (call after ready). */
void MPU6050_DMA_ProcessData(MPU6050_Data_t *out);

/* Get pointer to latest data (thread-safe-ish for single consumer) */
MPU6050_Data_t *MPU6050_DMA_GetLastData(void);

/* To be called in HAL_I2C_MemRxCpltCallback (file defined weak in HAL): */
void MPU6050_DMA_I2C_RxCpltCallback(I2C_HandleTypeDef *hi2c);

/* To be called in HAL_I2C_ErrorCallback if you want to catch errors */
void MPU6050_DMA_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c);

#endif /* __MPU6050_DMA_H__ */

