/*
 * MCP4725_dma.c
 *
 *  Created on: Nov 10, 2025
 *      Author: ADMIN
 */
#include "MCP4725_dma.h"
#include <string.h>
#include <stdio.h>
#include "cmsis_os2.h"

static I2C_HandleTypeDef *mcp_hi2c = NULL;
static uint8_t mcp_addr8 = 0;
static osSemaphoreId_t mcp_sem = NULL;
static volatile uint8_t mcp_in_progress = 0;
static uint8_t mcp_tx_buf[3];
static osMutexId_t mcp_i2c_mutex = NULL;


int MCP4725_Init(I2C_HandleTypeDef *hi2c, uint8_t addr7bit)
{
    if (hi2c == NULL) return -1;
    mcp_hi2c = hi2c;
    mcp_addr8 = (uint8_t)(addr7bit << 1);
    return 0;
}

void MCP4725_SetSemaphore(osSemaphoreId_t sem) { mcp_sem = sem; }
void MCP4725_SetI2CMutex(osMutexId_t m) { mcp_i2c_mutex = m; }

/* value 0..4095. Datasheet: fast mode write 12-bit DA is 3 bytes: C2..C0 + MSB + LSB
   We'll use fast mode: [C2 C1 C0 X X X X X] [D11..D4] [D3..D0 << 4] */
int MCP4725_WriteValue_DMA(uint16_t value, uint32_t timeout_ms)
{
    if (mcp_hi2c == NULL) return -1;
    if (value > 4095) value = 4095;

    /* prepare 3-byte fast write */
    mcp_tx_buf[0] = (uint8_t)((value >> 8) & 0x0F); /* bits D11..D8 in lower nibble */
    mcp_tx_buf[1] = (uint8_t)(value & 0xFF);

    mcp_in_progress = 1;
    if (mcp_i2c_mutex) {
        if (osMutexAcquire(mcp_i2c_mutex, 100) != osOK) {
            return -1;
        }
    }
    if (HAL_I2C_Master_Transmit_DMA(mcp_hi2c, mcp_addr8, mcp_tx_buf, 2) != HAL_OK) {
        mcp_in_progress = 0;
        return -1;
    }

    if (mcp_sem) {
        if (osSemaphoreAcquire(mcp_sem, timeout_ms) != osOK) {
            mcp_in_progress = 0;
            return -2;
        }
    } else {
        uint32_t t0 = HAL_GetTick();
        while (mcp_in_progress) {
            if ((HAL_GetTick() - t0) > timeout_ms) {
                mcp_in_progress = 0;
                return -2;
            }
            osDelay(1);
        }
    }
    return 0;
}

int MCP4725_SetValue(uint16_t rawValue)
{
    return MCP4725_WriteValue_DMA(rawValue, 200);
}
void MCP4725_DMA_TxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if (mcp_i2c_mutex) osMutexRelease(mcp_i2c_mutex);
    (void)hi2c;
    if (!mcp_in_progress) return;
    mcp_in_progress = 0;
    if (mcp_sem) osSemaphoreRelease(mcp_sem);
}

void MCP4725_DMA_RxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    MCP4725_DMA_TxCpltCallback(hi2c);
}

void MCP4725_DMA_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
	if (mcp_i2c_mutex) osMutexRelease(mcp_i2c_mutex);
    (void)hi2c;
    mcp_in_progress = 0;
    if (mcp_sem) osSemaphoreRelease(mcp_sem);
}
