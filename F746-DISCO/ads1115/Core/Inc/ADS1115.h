/*
 * ADS1115.h
 *
 *  Created on: Oct 22, 2025
 *      Author: ADMIN
 */

#ifndef __ADS1115_H__
#define __ADS1115_H__

#include "main.h"   // expects extern I2C_HandleTypeDef hi2c1
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// I2C device address (ADDR pin = GND)
#define ADS1115_ADDR_7BIT     0x48
#define ADS1115_I2C_ADDR      (ADS1115_ADDR_7BIT << 1)  // HAL expects 8-bit (7-bit << 1)

// Registers
#define ADS1115_REG_CONVERSION    0x00
#define ADS1115_REG_CONFIG        0x01
#define ADS1115_REG_LO_THRESH     0x02
#define ADS1115_REG_HI_THRESH     0x03

// PGA choices (bits [11:9] in config)
typedef enum {
    ADS_PGA_6_144 = 0, // +/-6.144V
    ADS_PGA_4_096 = 1, // +/-4.096V
    ADS_PGA_2_048 = 2, // +/-2.048V (default)
    ADS_PGA_1_024 = 3, // +/-1.024V
    ADS_PGA_0_512 = 4, // +/-0.512V
    ADS_PGA_0_256 = 5  // +/-0.256V
} ADS1115_PGA_t;

// Data rate choices (bits [7:5])
typedef enum {
    ADS_DR_8SPS   = 0,
    ADS_DR_16SPS  = 1,
    ADS_DR_32SPS  = 2,
    ADS_DR_64SPS  = 3,
    ADS_DR_128SPS = 4, // default
    ADS_DR_250SPS = 5,
    ADS_DR_475SPS = 6,
    ADS_DR_860SPS = 7
} ADS1115_DR_t;

// prototypes
void ADS1115_Init(void); // currently a placeholder (no hardware init needed)
uint16_t ADS1115_readRegister(uint8_t reg);
void ADS1115_writeRegister(uint8_t reg, uint16_t value);

// reading helpers
int16_t  ADS1115_readConversionRegister(void);

// Single-shot read (single-ended AINx vs GND)
int16_t ADS1115_readSingleEnded(uint8_t channel, ADS1115_PGA_t pga, ADS1115_DR_t dr, uint32_t timeout_ms);

// Single-shot differential read (AINp - AINn). e.g. AIN0-AIN1
int16_t ADS1115_readDifferential(uint8_t muxBits, ADS1115_PGA_t pga, ADS1115_DR_t dr, uint32_t timeout_ms);

// utility: convert raw signed to volts (depends on PGA)
float ADS1115_rawToVoltage(int16_t raw, ADS1115_PGA_t pga);

#ifdef __cplusplus
}
#endif

#endif // __ADS1115_H__
