/*
 * ADS1115.h
 *
 *  Created on: Oct 22, 2025
 *      Author: ADMIN
 */

#ifndef __ADS1115_H__
#define __ADS1115_H__

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#define ADS1115_ADDRESS_GND   0x48  // ADDR -> GND (7-bit)
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

/* Initialize ADS1115 with chosen PGA and data rate.
 * hi2c must be assigned externally (we store pointer internally).
 * addr7bit: 7-bit I2C address (e.g. ADS1115_ADDRESS_GND).
 * alert_pin config: user must wire ALERT/RDY to a GPIO EXTI (example PA2).
 */
void ADS1115_Init(I2C_HandleTypeDef *hi2c, uint8_t addr7bit, ADS1115_PGA_t pga, ADS1115_DR_t dr);

/* Start a DMA read of the conversion register (non-blocking);
 * wrapper below blocks until DMA complete or timeout_ms elapsed.
 * raw_out receives signed 16-bit sample.
 * return 0 on success, -1 on timeout/error.
 */
int ADS1115_ReadRaw_DMA_Blocking(int16_t *raw_out, uint32_t timeout_ms);

/* Utility: return voltage in volts according to PGA selection used in init.
 * returns 0 on success, -1 on error.
 */
int ADS1115_ReadVoltage_DMA_Blocking(float *volts_out, uint32_t timeout_ms);

/* Lower-level blocking read (uses HAL I2C blocking) useful for debug */
int ADS1115_ReadRaw_Blocking(int16_t *raw_out, uint32_t timeout_ms);

#endif // __ADS1115_H__
