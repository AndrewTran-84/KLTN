/*
 * DHT11.c
 *
 *  Created on: Nov 10, 2025
 *      Author: ADMIN
 */

#include "DHT11.h"

/* set pin as open-drain output, pull-up */
static void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;  /* open-drain */
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}
static void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}
uint32_t DWT_Delay_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0;
    for (volatile int i=0;i<10;i++) __NOP();
    if (DWT->CYCCNT) return 0;
    return 1;
}
static void delay_us(uint32_t microseconds)
{
    uint32_t clk_cycle_start = DWT->CYCCNT;
    uint32_t cycles = microseconds * (HAL_RCC_GetHCLKFreq() / 1000000UL);
    while ((DWT->CYCCNT - clk_cycle_start) < cycles) { __NOP(); }
}
static void DHT_Start(void)
{
    /* Ensure GPIO clocks enabled in CubeMX for port used. If not, enable manually: */
    if (DHT_PORT == GPIOA) __HAL_RCC_GPIOA_CLK_ENABLE();
    else if (DHT_PORT == GPIOB) __HAL_RCC_GPIOB_CLK_ENABLE();
    else if (DHT_PORT == GPIOC) __HAL_RCC_GPIOC_CLK_ENABLE();
    else if (DHT_PORT == GPIOD) __HAL_RCC_GPIOD_CLK_ENABLE();
    else if (DHT_PORT == GPIOE) __HAL_RCC_GPIOE_CLK_ENABLE();
    else if (DHT_PORT == GPIOF) __HAL_RCC_GPIOF_CLK_ENABLE();
    else if (DHT_PORT == GPIOG) __HAL_RCC_GPIOG_CLK_ENABLE();
    else if (DHT_PORT == GPIOH) __HAL_RCC_GPIOH_CLK_ENABLE();

    Set_Pin_Output(DHT_PORT, DHT_PIN);
    HAL_GPIO_WritePin(DHT_PORT, DHT_PIN, GPIO_PIN_RESET);

#if defined(TYPE_DHT11)
    delay_us(18000);
#elif defined(TYPE_DHT22)
    delay_us(1200);
#endif

    HAL_GPIO_WritePin(DHT_PORT, DHT_PIN, GPIO_PIN_SET);
    delay_us(30);
    Set_Pin_Input(DHT_PORT, DHT_PIN);
}
static int DHT_Check_Response(void)
{
    int response = 0;
    delay_us(40);
    if (HAL_GPIO_ReadPin(DHT_PORT, DHT_PIN) == GPIO_PIN_RESET) {
        delay_us(80);
        if (HAL_GPIO_ReadPin(DHT_PORT, DHT_PIN) == GPIO_PIN_SET) response = 1;
        else response = 0;
    }
    /* wait for line to go low again (end of response) */
    uint32_t tstart = DWT->CYCCNT;
    uint32_t timeoutCycles = (HAL_RCC_GetHCLKFreq() / 1000UL) * 5; /* 5 ms max */
    while (HAL_GPIO_ReadPin(DHT_PORT, DHT_PIN) == GPIO_PIN_SET) {
        if ((DWT->CYCCNT - tstart) > timeoutCycles) break;
    }
    return response;
}
static uint8_t DHT_ReadByte(void)
{
    uint8_t result = 0;
    for (uint8_t i = 0; i < 8; i++) {
        /* wait for pin to go high */
        uint32_t tstart = DWT->CYCCNT;
        uint32_t timeout = (HAL_RCC_GetHCLKFreq() / 1000000UL) * 1000; /* 1 ms */
        while (HAL_GPIO_ReadPin(DHT_PORT, DHT_PIN) == GPIO_PIN_RESET) {
            if ((DWT->CYCCNT - tstart) > timeout) return 0; /* timeout */
        }

        /* pin is high; measure how long */
        delay_us(40); /* after ~40us, high=>1 low=>0 per DHT timing */
        if (HAL_GPIO_ReadPin(DHT_PORT, DHT_PIN) == GPIO_PIN_SET) {
            result |= (1 << (7 - i));
            /* wait for pin to go low before next bit */
            tstart = DWT->CYCCNT;
            while (HAL_GPIO_ReadPin(DHT_PORT, DHT_PIN) == GPIO_PIN_SET) {
                if ((DWT->CYCCNT - tstart) > timeout) break;
            }
        } else {
            /* bit is 0: nothing to set (already zero) */
        }
    }
    return result;
}
/* Public API */
void DHT_GetData(DHT_DataTypedef *DHT_Data)
{
    if (DHT_Data == NULL) return;
    DHT_Data->valid = 0;
    /* start comms */
    DHT_Start();
    if (!DHT_Check_Response()) {
        DHT_Data->valid = 0;
        return;
    }

    uint8_t Rh_byte1 = DHT_ReadByte();
    uint8_t Rh_byte2 = DHT_ReadByte();
    uint8_t Temp_byte1 = DHT_ReadByte();
    uint8_t Temp_byte2 = DHT_ReadByte();
    uint8_t SUM = DHT_ReadByte();

    uint16_t calc_sum = (uint16_t)Rh_byte1 + (uint16_t)Rh_byte2 + (uint16_t)Temp_byte1 + (uint16_t)Temp_byte2;
    if ((uint8_t)(calc_sum & 0xFF) != SUM) {
        DHT_Data->valid = 0;
        return;
    }

#if defined(TYPE_DHT11)
    DHT_Data->Humidity = (float)Rh_byte1;
    DHT_Data->Temperature = (float)Temp_byte1;
#elif defined(TYPE_DHT22)
    int16_t rawHum = (int16_t)((Rh_byte1 << 8) | Rh_byte2);
    int16_t rawTmp = (int16_t)((Temp_byte1 << 8) | Temp_byte2);
    DHT_Data->Humidity = rawHum / 10.0f;
    /* Temperature sign bit */
    if (rawTmp & 0x8000) {
        rawTmp = rawTmp & 0x7FFF;
        DHT_Data->Temperature = - (rawTmp / 10.0f);
    } else {
        DHT_Data->Temperature = rawTmp / 10.0f;
    }
#endif

    DHT_Data->valid = 1;
    return;
}
