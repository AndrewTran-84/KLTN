/* ADS1115_dma.h */
#ifndef __ADS1115_DMA_H__
#define __ADS1115_DMA_H__

#include "stm32f7xx_hal.h"
#include "cmsis_os2.h"
#include <stdint.h>

#define ADS1115_ADDRESS_GND   0x48  // 7-bit
#define ADS1115_REG_CONVERSION 0x00
#define ADS1115_REG_CONFIG     0x01
#define ADS1115_REG_LO_THRESH  0x02
#define ADS1115_REG_HI_THRESH  0x03

typedef enum {
    ADS_PGA_6_144 = 0,
    ADS_PGA_4_096,
    ADS_PGA_2_048,
    ADS_PGA_1_024,
    ADS_PGA_0_512,
    ADS_PGA_0_256
} ADS1115_PGA_t;

typedef enum {
    ADS_DR_8SPS   = 0,
    ADS_DR_16SPS,
    ADS_DR_32SPS,
    ADS_DR_64SPS,
    ADS_DR_128SPS,
    ADS_DR_250SPS,
    ADS_DR_475SPS,
    ADS_DR_860SPS
} ADS1115_DR_t;

/* init: store hi2c and config sensor */
void ADS1115_Init(I2C_HandleTypeDef *hi2c, uint8_t addr7bit, ADS1115_PGA_t pga, ADS1115_DR_t dr);

/* Start DMA read into user-supplied 2-byte buffer (non-blocking).
 * sets internal in_progress flag and returns HAL status */
int ADS1115_StartRead_DMA(uint8_t *buf2);

/* Blocking wrapper: start DMA then wait on semaphore (if set) or busy-wait.
 * timeout_ms: ms to wait. returns 0 success, -1 error/timeout */
int ADS1115_ReadRaw_DMA_Blocking(int16_t *raw_out, uint32_t timeout_ms);

int ADS1115_ReadVoltage_DMA_Blocking(float *volts_out, uint32_t timeout_ms);

/* Setter: provide semaphore created by main (CMSIS os).
 * If not set, driver uses volatile flag 'ads_dma_done' */
void ADS1115_SetSemaphore(osSemaphoreId_t sem);

void ADS1115_SetI2CMutex(osMutexId_t m);

/* Called from HAL_I2C_MemRxCpltCallback in main (forwarding). */
void ADS1115_DMA_RxCpltCallback(I2C_HandleTypeDef *hi2c);

/* Called from HAL_I2C_ErrorCallback */
void ADS1115_DMA_ErrorCallback(I2C_HandleTypeDef *hi2c);

#endif /* __ADS1115_DMA_H__ */
