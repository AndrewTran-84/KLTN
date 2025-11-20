/* ADS1115_dma.c */
#include "ADS1115_dma.h"
#include "cmsis_os2.h"
#include <string.h>
#include <stdio.h>

static float ads_get_fullscale_voltage(ADS1115_PGA_t p)
{
    switch (p) {
        case ADS_PGA_6_144: return 6.144f;
        case ADS_PGA_4_096: return 4.096f;
        case ADS_PGA_2_048: return 2.048f;
        case ADS_PGA_1_024: return 1.024f;
        case ADS_PGA_0_512: return 0.512f;
        case ADS_PGA_0_256: return 0.256f;
        default: return 2.048f;
    }
}

static I2C_HandleTypeDef *ads_hi2c = NULL;
static uint8_t ads_devaddr_8bit = 0;
static ADS1115_PGA_t ads_pga = ADS_PGA_2_048;
static ADS1115_DR_t ads_dr = ADS_DR_250SPS;

/* state */
volatile uint8_t ads_dma_done = 0;
static volatile uint8_t ads_in_progress = 0;
static osSemaphoreId_t ads_sem = NULL;
static osMutexId_t ads_i2c_mutex = NULL;

static int writeRegister16(uint8_t reg, uint16_t value)
{
    uint8_t data[2];
    data[0] = (uint8_t)(value >> 8);
    data[1] = (uint8_t)(value & 0xFF);
    if (HAL_I2C_Mem_Write(ads_hi2c, ads_devaddr_8bit, reg, I2C_MEMADD_SIZE_8BIT, data, 2, 200) != HAL_OK) {
        return -1;
    }
    return 0;
}
static int readRegister16_block(uint8_t reg, uint16_t *out)
{
    uint8_t data[2];
    if (HAL_I2C_Mem_Read(ads_hi2c, ads_devaddr_8bit, reg, I2C_MEMADD_SIZE_8BIT, data, 2, 200) != HAL_OK) {
        return -1;
    }
    *out = ((uint16_t)data[0] << 8) | data[1];
    return 0;
}

void ADS1115_Init(I2C_HandleTypeDef *hi2c, uint8_t addr7bit, ADS1115_PGA_t pga, ADS1115_DR_t dr)
{
    ads_hi2c = hi2c;
    ads_devaddr_8bit = (uint8_t)(addr7bit << 1);
    ads_pga = pga;
    ads_dr = dr;

    uint16_t cfg = 0;
    cfg |= (0x4 << 12);
    cfg |= ((uint16_t)((uint16_t)pga & 0x7) << 9);
    cfg |= ((uint16_t)((uint16_t)dr & 0x7) << 5);

    writeRegister16(ADS1115_REG_CONFIG, cfg);
    writeRegister16(ADS1115_REG_HI_THRESH, 0x7FFF);
    writeRegister16(ADS1115_REG_LO_THRESH, 0x8000);

    ads_dma_done = 0;
    ads_in_progress = 0;
}

void ADS1115_SetSemaphore(osSemaphoreId_t sem) { ads_sem = sem; }

void ADS1115_SetI2CMutex(osMutexId_t m) { ads_i2c_mutex = m; }

/* start DMA read into provided buffer (2 bytes). Non-blocking */
int ADS1115_StartRead_DMA(uint8_t *buf2)
{
    if (ads_hi2c == NULL) return -1;
    ads_dma_done = 0;
    ads_in_progress = 1;
    if (ads_i2c_mutex) {
    	if (osMutexAcquire(ads_i2c_mutex, 100) != osOK) {
    		return -1; /* không lấy được mutex trong 100ms */
    	}
    }
    if (HAL_I2C_Mem_Read_DMA(ads_hi2c, ads_devaddr_8bit, ADS1115_REG_CONVERSION, I2C_MEMADD_SIZE_8BIT, buf2, 2) != HAL_OK) {
        ads_in_progress = 0;
        return -1;
    }
    return 0;
}

int ADS1115_ReadRaw_DMA_Blocking(int16_t *raw_out, uint32_t timeout_ms)
{
    uint8_t buf[2];
    if (ADS1115_StartRead_DMA(buf) != 0) return -1;

    if (ads_sem != NULL) {
        if (osSemaphoreAcquire(ads_sem, timeout_ms) != osOK) {
            ads_in_progress = 0;
            return -1;
        }
    } else {
        uint32_t t0 = HAL_GetTick();
        while (!ads_dma_done) {
            if ((HAL_GetTick() - t0) > timeout_ms) {
                ads_in_progress = 0;
                return -1;
            }
            osDelay(1);
        }
    }

    ads_dma_done = 0;
    uint16_t tmp = ((uint16_t)buf[0] << 8) | buf[1];
    *raw_out = (int16_t)tmp;
    return 0;
}

int ADS1115_ReadVoltage_DMA_Blocking(float *volts_out, uint32_t timeout_ms)
{
    if (volts_out == NULL) return -1;
    int16_t raw = 0;
    if (ADS1115_ReadRaw_DMA_Blocking(&raw, timeout_ms) != 0) {
        return -1;
    }
    float fs = ads_get_fullscale_voltage(ads_pga);
    *volts_out = ((float)raw) * (fs / 32768.0f);
    return 0;
}
/* Called from main HAL_I2C_MemRxCpltCallback */
void ADS1115_DMA_RxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if (ads_i2c_mutex) osMutexRelease(ads_i2c_mutex);
    (void)hi2c;
    if (!ads_in_progress) return;
    ads_in_progress = 0;
    ads_dma_done = 1;
    if (ads_sem) {
        osSemaphoreRelease(ads_sem);
    }
}

void ADS1115_DMA_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
	if (ads_i2c_mutex) osMutexRelease(ads_i2c_mutex);
    (void)hi2c;
    ads_in_progress = 0;
    ads_dma_done = 0;
    if (ads_sem) {
        osSemaphoreRelease(ads_sem); /* wake waiter with error */
    }
}
