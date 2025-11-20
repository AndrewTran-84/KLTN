/*
 * MCP4725_dma.h
 *
 *  Created on: Nov 10, 2025
 *      Author: ADMIN
 */

#ifndef __MCP4725_DMA_H__
#define __MCP4725_DMA_H__

#include "stm32f7xx_hal.h"
#include "cmsis_os2.h"
#include <stdint.h>

/* Initialize: provide hi2c and 7-bit address */
int MCP4725_Init(I2C_HandleTypeDef *hi2c, uint8_t addr7bit);

/* Set output (0..4095) via DMA (non-blocking); blocking wrapper uses semaphore */
int MCP4725_WriteValue_DMA(uint16_t value, uint32_t timeout_ms);

int MCP4725_SetValue(uint16_t rawValue);


/* set semaphore created by main */
void MCP4725_SetSemaphore(osSemaphoreId_t sem);
void MCP4725_SetI2CMutex(osMutexId_t m);

/* forward from HAL callbacks */
void MCP4725_DMA_TxCpltCallback(I2C_HandleTypeDef *hi2c);
void MCP4725_DMA_RxCpltCallback(I2C_HandleTypeDef *hi2c);
void MCP4725_DMA_ErrorCallback(I2C_HandleTypeDef *hi2c);

#endif /* __MCP4725_DMA_H__ */
