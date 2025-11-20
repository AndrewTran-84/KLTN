/*
 * DHT11.h
 *
 *  Created on: Nov 10, 2025
 *      Author: ADMIN
 */

#ifndef INC_DHT11_H_
#define INC_DHT11_H_

#include "stm32f7xx_hal.h"

typedef struct {
    float Temperature;
    float Humidity;
    uint8_t valid;
} DHT_DataTypedef;

#define TYPE_DHT11

#ifndef DHT_PORT
#define DHT_PORT GPIOG   /* change default to GPIOG for your PG7 mapping */
#endif
#ifndef DHT_PIN
#define DHT_PIN  GPIO_PIN_7
#endif

uint32_t DWT_Delay_Init(void);
void DHT_GetData(DHT_DataTypedef *DHT_Data);


#endif /* INC_DHT11_H_ */
