/*
 * ADS1115.c
 * simple blocking ADS1115 HAL driver for STM32F7
   Non-DMA, polls OS bit for conversion complete.
 *  Created on: Oct 22, 2025
 *      Author: ADMIN
 */
#include "ADS1115.h"
#include <string.h>

// Use hi2c1 defined in main.h / CubeMX project
extern I2C_HandleTypeDef hi2c1;

#define ADS_I2C_ADDR  ADS1115_I2C_ADDR

// internal delays: minimal wait derived from data rate
static uint32_t dr_to_delay_ms(ADS1115_DR_t dr)
{
    switch(dr) {
        case ADS_DR_8SPS: return 125;   // 125ms
        case ADS_DR_16SPS: return 63;
        case ADS_DR_32SPS: return 32;
        case ADS_DR_64SPS: return 16;
        case ADS_DR_128SPS: return 8;
        case ADS_DR_250SPS: return 4;
        case ADS_DR_475SPS: return 3;
        case ADS_DR_860SPS: return 2;
        default: return 10;
    }
}

void ADS1115_Init(void)
{
    // nothing required; hi2c1 must be initialized by CubeMX
}

// Low-level register access (16-bit registers big-endian)
uint16_t ADS1115_readRegister(uint8_t reg)
{
    uint8_t buf[2] = {0};
    if (HAL_I2C_Mem_Read(&hi2c1, ADS_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, 2, 1000) != HAL_OK) {
        return 0;
    }
    return (uint16_t)((buf[0] << 8) | buf[1]);
}

void ADS1115_writeRegister(uint8_t reg, uint16_t value)
{
    uint8_t buf[2];
    buf[0] = (uint8_t)(value >> 8);
    buf[1] = (uint8_t)(value & 0xFF);
    HAL_I2C_Mem_Write(&hi2c1, ADS_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, 2, 1000);
}

int16_t ADS1115_readConversionRegister(void)
{
    uint16_t raw = ADS1115_readRegister(ADS1115_REG_CONVERSION);
    return (int16_t)raw;
}

// Build config bits and start single-shot conversion
static void ADS1115_startSingleShot(uint16_t config)
{
    // write config (start single conversion by setting OS=1)
    ADS1115_writeRegister(ADS1115_REG_CONFIG, config);
}

// Utility to wait until OS bit (bit 15) is set in config (conversion complete)
static int ADS1115_waitForConversion(uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < timeout_ms) {
        uint16_t cfg = ADS1115_readRegister(ADS1115_REG_CONFIG);
        // OS bit (bit15) = 1 when conversion complete (for single-shot)
        if (cfg & 0x8000) return 0;
        HAL_Delay(1);
    }
    return -1; // timeout
}

// channel: 0..3 single ended, pga: see enum, dr: data rate
int16_t ADS1115_readSingleEnded(uint8_t channel, ADS1115_PGA_t pga, ADS1115_DR_t dr, uint32_t timeout_ms)
{
    if (channel > 3) return 0;
    // MUX for single-ended:
    // 100: AIN0-GND ; 101: AIN1-GND ; 110: AIN2-GND ; 111: AIN3-GND
    uint16_t mux;
    switch (channel) {
        case 0: mux = 0x4; break;
        case 1: mux = 0x5; break;
        case 2: mux = 0x6; break;
        default: mux = 0x7; break;
    }

    // config: OS=1 (start), MUX[14:12] = mux, PGA[11:9] = pga, MODE[8]=1 (single-shot),
    // DR[7:5]=dr, comparator disable bits = 000
    uint16_t config = 0;
    config |= (1 << 15);           // OS = 1 -> start single conversion
    config |= (mux & 0x7) << 12;   // MUX
    config |= ( ( (uint16_t)pga & 0x7) << 9 );
    config |= (1 << 8);            // MODE = single-shot
    config |= ( ( (uint16_t)dr & 0x7 ) << 5 );
    // comparator off (bits [4:0] -> 00011? choose disable: COMP_QUE = 11 to disable)
    config |= 0x0003; // COMP_QUE = 11 (disable comparator)

    ADS1115_startSingleShot(config);

    // wait for conversion
    uint32_t delay_ms = dr_to_delay_ms(dr) + 2;
    if (timeout_ms < delay_ms) timeout_ms = delay_ms + 10;
    if (ADS1115_waitForConversion(timeout_ms) != 0) {
        // timeout, but attempt to read raw anyway
    }

    int16_t raw = ADS1115_readConversionRegister();
    return raw;
}

// For differential, muxBits should be one of:
// 0: AIN0-AIN1, 1: AIN0-AIN3, 2: AIN1-AIN3, 3: AIN2-AIN3
int16_t ADS1115_readDifferential(uint8_t muxBits, ADS1115_PGA_t pga, ADS1115_DR_t dr, uint32_t timeout_ms)
{
    if (muxBits > 3) muxBits = 0;
    uint16_t mux = muxBits; // 000..011 maps directly to MUX for diff reading

    uint16_t config = 0;
    config |= (1 << 15); // OS
    config |= ( (mux & 0x7) << 12 );
    config |= ( ( (uint16_t)pga & 0x7 ) << 9 );
    config |= (1 << 8); // single-shot
    config |= ( ( (uint16_t)dr & 0x7 ) << 5 );
    config |= 0x0003; // comparator disable

    ADS1115_startSingleShot(config);

    uint32_t delay_ms = dr_to_delay_ms(dr) + 2;
    if (timeout_ms < delay_ms) timeout_ms = delay_ms + 10;
    if (ADS1115_waitForConversion(timeout_ms) != 0) {
        // timeout
    }

    int16_t raw = ADS1115_readConversionRegister();
    return raw;
}

float ADS1115_rawToVoltage(int16_t raw, ADS1115_PGA_t pga)
{
    float fs = 2.048f; // default
    switch (pga) {
        case ADS_PGA_6_144: fs = 6.144f; break;
        case ADS_PGA_4_096: fs = 4.096f; break;
        case ADS_PGA_2_048: fs = 2.048f; break;
        case ADS_PGA_1_024: fs = 1.024f; break;
        case ADS_PGA_0_512: fs = 0.512f; break;
        case ADS_PGA_0_256: fs = 0.256f; break;
    }
    // ADS1115 conversion outputs signed integer -32768..32767 representing -FS..+FS (approx)
    float lsb = fs / 32768.0f;
    return (float)raw * lsb;
}

