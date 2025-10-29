/*
 * ADS1115.c
 *
 *  Created on: Oct 22, 2025
 *      Author: ADMIN
 */
#include "ADS1115.h"
#include "string.h"
#include "stdio.h"

/* Internal state */
static I2C_HandleTypeDef *ads_hi2c = NULL;
static uint8_t ads_devaddr_8bit = 0;
static ADS1115_PGA_t ads_pga = ADS_PGA_2_048;
static ADS1115_DR_t ads_dr = ADS_DR_250SPS;

volatile uint8_t ads_dma_done = 0;
volatile uint8_t ads_ready_flag = 0; // set by EXTI when ALERT/RDY pulses

/* Helper: write 16-bit register (big-endian) */
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

/* Helper: blocking read 16-bit register */
static int readRegister16(uint8_t reg, uint16_t *out)
{
    uint8_t data[2];
    if (HAL_I2C_Mem_Read(ads_hi2c, ads_devaddr_8bit, reg, I2C_MEMADD_SIZE_8BIT, data, 2, 200) != HAL_OK) {
        return -1;
    }
    *out = ((uint16_t)data[0] << 8) | data[1];
    return 0;
}

/* Map PGA enum to full-scale voltage */
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

void ADS1115_Init(I2C_HandleTypeDef *hi2c, uint8_t addr7bit, ADS1115_PGA_t pga, ADS1115_DR_t dr)
{
    ads_hi2c = hi2c;
    ads_devaddr_8bit = (uint8_t)(addr7bit << 1);
    ads_pga = pga;
    ads_dr = dr;

    /* Build config register:
     * - OS = 0 (no effect in cont. mode)
     * - MUX = 100 (AIN0 single-ended)
     * - PGA = according to pga
     * - MODE = 0 (continuous conversion)
     * - DR = according to dr
     * - COMP_MODE = 0 (traditional)
     * - COMP_POL = 0 (active low)
     * - COMP_LAT = 0 (non-latching)
     * - COMP_QUE = 00 (assert after one conversion -> use as RDY pulse)
     *
     * Config bits layout: [15:0] OS(15) MUX(14:12) PGA(11:9) MODE(8) DR(7:5) COMP_MODE(4) COMP_POL(3)
     * COMP_LAT(2) COMP_QUE(1:0)
     */
    uint16_t cfg = 0;
    // MUX = 100 -> AIN0 single-ended
    cfg |= (0x4 << 12);
    // PGA
    cfg |= ((uint16_t)( ( (uint16_t)pga ) & 0x7) << 9);
    // MODE = 0 -> continuous
    // DR
    cfg |= ((uint16_t)( ( (uint16_t)dr ) & 0x7) << 5);
    // COMP_MODE = 0; COMP_POL = 0; COMP_LAT = 0;
    // COMP_QUE = 00 -> assert after one conversion
    cfg |= 0x0; // nothing extra

    // Write config
    writeRegister16(ADS1115_REG_CONFIG, cfg);

    // Optionally set thresholds to extremes (so comparator will assert every conversion)
    // set hi_thresh = 0x7FFF, lo_thresh = 0x8000
    writeRegister16(ADS1115_REG_HI_THRESH, 0x7FFF);
    writeRegister16(ADS1115_REG_LO_THRESH, 0x8000);

    /* Clear flags */
    ads_dma_done = 0;
    ads_ready_flag = 0;
}

/* Blocking read using blocking HAL (useful for debug) */
int ADS1115_ReadRaw_Blocking(int16_t *raw_out, uint32_t timeout_ms)
{
    uint16_t data;
    if (readRegister16(ADS1115_REG_CONVERSION, &data) != 0) return -1;
    *raw_out = (int16_t)data;
    (void)timeout_ms;
    return 0;
}

/* Start DMA read (non-blocking). Caller can wait for ads_dma_done flag or use blocking wrapper. */
static int ADS1115_StartRead_DMA(uint8_t *buf2)
{
    if (ads_hi2c == NULL) return -1;
    ads_dma_done = 0;
    if (HAL_I2C_Mem_Read_DMA(ads_hi2c, ads_devaddr_8bit, ADS1115_REG_CONVERSION, I2C_MEMADD_SIZE_8BIT, buf2, 2) != HAL_OK) {
        return -1;
    }
    return 0;
}

/* Blocking wrapper */
int ADS1115_ReadRaw_DMA_Blocking(int16_t *raw_out, uint32_t timeout_ms)
{
    uint8_t buf[2];
    if (ADS1115_StartRead_DMA(buf) != 0) return -1;

    uint32_t start = HAL_GetTick();
    while (!ads_dma_done) {
        if ((HAL_GetTick() - start) > timeout_ms) {
            return -1; // timeout
        }
    }
    ads_dma_done = 0;
    uint16_t tmp = ((uint16_t)buf[0] << 8) | buf[1];
    *raw_out = (int16_t)tmp;
    return 0;
}

int ADS1115_ReadVoltage_DMA_Blocking(float *volts_out, uint32_t timeout_ms)
{
    int16_t raw;
    if (ADS1115_ReadRaw_DMA_Blocking(&raw, timeout_ms) != 0) return -1;
    float fs = ads_get_fullscale_voltage(ads_pga);
    *volts_out = ((float)raw) * (fs / 32768.0f);
    return 0;
}

/* --- Callbacks to be placed in main project --- */
/* These should be present in your project and will be triggered by HAL: */

/* HAL_I2C_MemRxCpltCallback must be implemented in main (or weak) - we'll provide simple one in main.c */


