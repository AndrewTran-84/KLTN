/*
 * LoRa_dma.h
 *
 *  Created on: Nov 10, 2025
 *      Author: ADMIN
 */

#ifndef INC_LORA_DMA_H_
#define INC_LORA_DMA_H_

#include "stdint.h"
#include "stm32f7xx_hal.h"
#include "cmsis_os2.h"

extern SPI_HandleTypeDef hspi2;
extern UART_HandleTypeDef huart1;

#ifndef LORA_NSS_GPIO_Port
#define LORA_NSS_GPIO_Port  GPIOI
#define LORA_NSS_Pin        GPIO_PIN_0
#endif
#ifndef LORA_RST_GPIO_Port
#define LORA_RST_GPIO_Port  GPIOA
#define LORA_RST_Pin        GPIO_PIN_15
#endif
#ifndef LORA_DIO0_GPIO_Port
#define LORA_DIO0_GPIO_Port GPIOI
#define LORA_DIO0_Pin       GPIO_PIN_2
#endif

int  LoRa_Init(void);
void LoRa_Reset(void);
int  LoRa_SetFrequency(uint32_t freqHz);
int  LoRa_Transmit_DMA(uint8_t *buffer, uint16_t len, uint32_t timeout_ms);
int  LoRa_receive_ready(void);
int  LoRa_read_payload(uint8_t *dst, uint8_t maxlen);

/* Setter for tx semaphore created in main */
void LoRa_SetTxSemaphore(osSemaphoreId_t sem);

/* Callbacks to be called from HAL handlers */
void LoRa_SPI_TxCplt_Callback(SPI_HandleTypeDef *hspi);
void LoRa_DIO0_Callback(void);


#endif /* INC_LORA_DMA_H_ */
