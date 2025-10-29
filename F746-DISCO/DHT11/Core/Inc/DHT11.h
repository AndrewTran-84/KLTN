/*
 * DHT11.h
 *
 *  Created on: Oct 29, 2025
 *      Author: ADMIN
 * DHT.h - DHT11 / DHT22 helper for STM32F7 (uses DWT cycle counter for delays)
 *
 * Configure:
 *  - DHT_PORT, DHT_PIN below (default PA0)
 *  - #define TYPE_DHT11  or #define TYPE_DHT22
 *
 * Usage:
 *  - DHT_DataTypedef d;
 *  - if (DWT_Delay_Init() == 0) DHT_GetData(&d);
 */

#ifndef INC_DHT_H_
#define INC_DHT_H_

#include "stm32f7xx_hal.h"

typedef struct {
    float Temperature;
    float Humidity;
    uint8_t valid; /* 1 = valid, 0 = checksum fail / no response */
} DHT_DataTypedef;

/* Select sensor type: uncomment one */
#define TYPE_DHT11
//#define TYPE_DHT22

/* Pin selection: default ARDUINO_A0 -> PA0 */
#ifndef DHT_PORT
#define DHT_PORT GPIOA
#endif
#ifndef DHT_PIN
#define DHT_PIN  GPIO_PIN_0
#endif

/* API */
uint32_t DWT_Delay_Init(void); /* init cycle counter, returns 0 if OK */
void DHT_GetData(DHT_DataTypedef *DHT_Data); /* fills struct, sets valid flag */

#endif /* INC_DHT_H_ */
