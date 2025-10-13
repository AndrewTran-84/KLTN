/*
 * ADS1015_ADS1115.c
 * DMA-enabled read for STM32 HAL + FreeRTOS
 */

#include "ADS1015_ADS1115.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include <string.h>

/* External objects from freertos.c */
extern SemaphoreHandle_t xADSSem;

/* Local static helper: writeRegister and blocking read remain */
static void writeRegister(ADS1xx5_I2C *i2c, uint8_t reg, uint16_t value) {
    uint8_t pData[3] = { reg, (uint8_t) (value >> 8), (uint8_t) (value & 0xFF) };
    HAL_I2C_Master_Transmit(i2c->hi2c, i2c->m_i2cAddress, pData, 3, 50);
}
/* forward declaration: readRegister is defined later in this file */
/* Read register implementation (fix: add actual definition) */
static uint16_t readRegister(ADS1xx5_I2C *i2c, uint8_t reg) {
    uint8_t regBuf = reg;
    if (HAL_I2C_Master_Transmit(i2c->hi2c, i2c->m_i2cAddress, &regBuf, 1, 10) != HAL_OK) {
        return 0; // or handle error
    }
    uint8_t pData[2] = {0, 0};
    if (HAL_I2C_Master_Receive(i2c->hi2c, i2c->m_i2cAddress, pData, 2, 10) != HAL_OK) {
        return 0; // or handle error
    }
    return (uint16_t)((pData[0] << 8) | pData[1]);
}

/* Blocking read (kept for compatibility) */
static uint16_t readRegisterBlocking(ADS1xx5_I2C *i2c, uint8_t reg) {
    HAL_I2C_Master_Transmit(i2c->hi2c, i2c->m_i2cAddress, &reg, 1, 50);
    uint8_t pData[2] = { 0, 0 };
    HAL_I2C_Master_Receive(i2c->hi2c, i2c->m_i2cAddress, pData, 2, 50);
    return ((pData[0] << 8) | pData[1]);
}

/* Check device ready */
static void ADSbegin(ADS1xx5_I2C *i2c) {
    if (HAL_I2C_IsDeviceReady(i2c->hi2c, i2c->m_i2cAddress, 3, 50) != HAL_OK)
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); // error indicator
}

/* Constructors */
void ADS1015(ADS1xx5_I2C *i2c, I2C_HandleTypeDef *hi2c, uint8_t i2cAddress) {
    i2c->hi2c = hi2c;
    i2c->m_i2cAddress = i2cAddress << 1;
    i2c->m_conversionDelay = ADS1015_CONVERSIONDELAY;
    i2c->m_bitShift = 4;
    i2c->m_gain = GAIN_TWOTHIRDS;
    ADSbegin(i2c);
}

void ADS1115(ADS1xx5_I2C *i2c, I2C_HandleTypeDef *hi2c, uint8_t i2cAddress) {
    i2c->hi2c = hi2c;
    i2c->m_i2cAddress = i2cAddress << 1;
    i2c->m_conversionDelay = ADS1115_CONVERSIONDELAY;
    i2c->m_bitShift = 0;
    i2c->m_gain = GAIN_TWOTHIRDS;
    ADSbegin(i2c);
}

void ADSsetGain(ADS1xx5_I2C *i2c, adsGain_t gain) { i2c->m_gain = gain; }
adsGain_t ADSgetGain(ADS1xx5_I2C *i2c) { return i2c->m_gain; }

/* Legacy single-ended blocking read (kept) */
uint16_t ADSreadADC_SingleEnded(ADS1xx5_I2C *i2c, uint8_t channel) {
    if (channel > 3) return 0;
    uint16_t config = ADS1015_REG_CONFIG_CQUE_NONE |
                      ADS1015_REG_CONFIG_CLAT_NONLAT |
                      ADS1015_REG_CONFIG_CPOL_ACTVLOW |
                      ADS1015_REG_CONFIG_CMODE_TRAD |
                      ADS1015_REG_CONFIG_DR_1600SPS |
                      ADS1015_REG_CONFIG_MODE_SINGLE;
    config |= i2c->m_gain;
    switch (channel) {
        case 0: config |= ADS1015_REG_CONFIG_MUX_SINGLE_0; break;
        case 1: config |= ADS1015_REG_CONFIG_MUX_SINGLE_1; break;
        case 2: config |= ADS1015_REG_CONFIG_MUX_SINGLE_2; break;
        case 3: config |= ADS1015_REG_CONFIG_MUX_SINGLE_3; break;
    }
    config |= ADS1015_REG_CONFIG_OS_SINGLE;
    writeRegister(i2c, ADS1015_REG_POINTER_CONFIG, config);
    HAL_Delay(i2c->m_conversionDelay);
    return readRegisterBlocking(i2c, ADS1015_REG_POINTER_CONVERT) >> i2c->m_bitShift;
}

/* New: DMA-based single-ended read
 * waitTicks => FreeRTOS ticks to wait for DMA completion (e.g. pdMS_TO_TICKS(50))
 */
int16_t ADSreadADC_SingleEnded_DMA(ADS1xx5_I2C *i2c, uint8_t channel, uint32_t wait_ms) {
    if (channel > 3) return 0;

    uint16_t config = ADS1015_REG_CONFIG_CQUE_NONE |
                      ADS1015_REG_CONFIG_CLAT_NONLAT |
                      ADS1015_REG_CONFIG_CPOL_ACTVLOW |
                      ADS1015_REG_CONFIG_CMODE_TRAD |
                      ADS1015_REG_CONFIG_DR_1600SPS |
                      ADS1015_REG_CONFIG_MODE_SINGLE;
    config |= i2c->m_gain;
    switch (channel) {
        case 0: config |= ADS1015_REG_CONFIG_MUX_SINGLE_0; break;
        case 1: config |= ADS1015_REG_CONFIG_MUX_SINGLE_1; break;
        case 2: config |= ADS1015_REG_CONFIG_MUX_SINGLE_2; break;
        case 3: config |= ADS1015_REG_CONFIG_MUX_SINGLE_3; break;
    }
    config |= ADS1015_REG_CONFIG_OS_SINGLE;

    /* Write config (blocking; short) */
    writeRegister(i2c, ADS1015_REG_POINTER_CONFIG, config);

    /* Start DMA read of conversion register (2 bytes) */
    uint8_t buf[2];
    if (HAL_I2C_Mem_Read_DMA(i2c->hi2c, i2c->m_i2cAddress, ADS1015_REG_POINTER_CONVERT,
                             I2C_MEMADD_SIZE_8BIT, buf, 2) != HAL_OK) {
        return 0; // error
    }

    /* If semaphore absent, fallback to blocking read */
    if (xADSSem == NULL) {
        HAL_Delay(i2c->m_conversionDelay);
        return (int16_t)(readRegister(i2c, ADS1015_REG_POINTER_CONVERT) >> i2c->m_bitShift);
    }

    /* convert wait_ms to ticks and wait */
    TickType_t waitTicks = pdMS_TO_TICKS(wait_ms);
    if (xSemaphoreTake(xADSSem, waitTicks) != pdTRUE) {
        /* timeout -> fallback blocking */
        HAL_Delay(i2c->m_conversionDelay);
        return (int16_t)(readRegister(i2c, ADS1015_REG_POINTER_CONVERT) >> i2c->m_bitShift);
    }

    uint16_t res = ((uint16_t)buf[0] << 8) | buf[1];
    if (i2c->m_bitShift)
        res = res >> i2c->m_bitShift;

    return (int16_t)res;
}


/* Differential reads kept (blocking) */
int16_t ADSreadADC_Differential_0_1(ADS1xx5_I2C *i2c) {
    uint16_t config = ADS1015_REG_CONFIG_CQUE_NONE | ADS1015_REG_CONFIG_CLAT_NONLAT |
                      ADS1015_REG_CONFIG_CPOL_ACTVLOW | ADS1015_REG_CONFIG_CMODE_TRAD |
                      ADS1015_REG_CONFIG_DR_1600SPS | ADS1015_REG_CONFIG_MODE_SINGLE;
    config |= i2c->m_gain;
    config |= ADS1015_REG_CONFIG_MUX_DIFF_0_1;
    config |= ADS1015_REG_CONFIG_OS_SINGLE;
    writeRegister(i2c, ADS1015_REG_POINTER_CONFIG, config);
    HAL_Delay(i2c->m_conversionDelay);
    uint16_t res = readRegisterBlocking(i2c, ADS1015_REG_POINTER_CONVERT) >> i2c->m_bitShift;
    if (i2c->m_bitShift == 0) return (int16_t)res;
    if (res > 0x07FF) res |= 0xF000;
    return (int16_t)res;
}

int16_t ADSreadADC_Differential_2_3(ADS1xx5_I2C *i2c) {
    uint16_t config = ADS1015_REG_CONFIG_CQUE_NONE | ADS1015_REG_CONFIG_CLAT_NONLAT |
                      ADS1015_REG_CONFIG_CPOL_ACTVLOW | ADS1015_REG_CONFIG_CMODE_TRAD |
                      ADS1015_REG_CONFIG_DR_1600SPS | ADS1015_REG_CONFIG_MODE_SINGLE;
    config |= i2c->m_gain;
    config |= ADS1015_REG_CONFIG_MUX_DIFF_2_3;
    config |= ADS1015_REG_CONFIG_OS_SINGLE;
    writeRegister(i2c, ADS1015_REG_POINTER_CONFIG, config);
    HAL_Delay(i2c->m_conversionDelay);
    uint16_t res = readRegisterBlocking(i2c, ADS1015_REG_POINTER_CONVERT) >> i2c->m_bitShift;
    if (i2c->m_bitShift == 0) return (int16_t)res;
    if (res > 0x07FF) res |= 0xF000;
    return (int16_t)res;
}

/* Comparator start (kept blocking) */
void ADSstartComparator_SingleEnded(ADS1xx5_I2C *i2c, uint8_t channel, int16_t threshold) {
    uint16_t config = ADS1015_REG_CONFIG_CQUE_1CONV | ADS1015_REG_CONFIG_CLAT_LATCH |
                      ADS1015_REG_CONFIG_CPOL_ACTVLOW | ADS1015_REG_CONFIG_CMODE_TRAD |
                      ADS1015_REG_CONFIG_DR_1600SPS | ADS1015_REG_CONFIG_MODE_CONTIN;
    config |= i2c->m_gain;
    switch (channel) {
        case 0: config |= ADS1015_REG_CONFIG_MUX_SINGLE_0; break;
        case 1: config |= ADS1015_REG_CONFIG_MUX_SINGLE_1; break;
        case 2: config |= ADS1015_REG_CONFIG_MUX_SINGLE_2; break;
        case 3: config |= ADS1015_REG_CONFIG_MUX_SINGLE_3; break;
    }
    writeRegister(i2c, ADS1015_REG_POINTER_HITHRESH, threshold << i2c->m_bitShift);
    writeRegister(i2c, ADS1015_REG_POINTER_CONFIG, config);
}

/* ADSgetLastConversionResults wrapper (blocking) */
int16_t ADSgetLastConversionResults(ADS1xx5_I2C *i2c) {
    HAL_Delay(i2c->m_conversionDelay);
    uint16_t res = readRegisterBlocking(i2c, ADS1015_REG_POINTER_CONVERT) >> i2c->m_bitShift;
    if (i2c->m_bitShift == 0) return (int16_t)res;
    if (res > 0x07FF) res |= 0xF000;
    return (int16_t)res;
}
