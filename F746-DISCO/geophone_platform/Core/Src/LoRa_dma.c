/*
 * LoRa_dma.c
 *
 *  Created on: Nov 10, 2025
 *      Author: ADMIN
 */

#include "LoRa_dma.h"
#include <string.h>
#include <stdio.h>
#include "cmsis_os2.h"
#include "main.h"

extern UART_HandleTypeDef huart1;
extern SPI_HandleTypeDef hspi2;

/* SX127x registers (subset used) */
#define REG_FIFO                    0x00
#define REG_OP_MODE                 0x01
#define REG_FRF_MSB                 0x06
#define REG_FRF_MID                 0x07
#define REG_FRF_LSB                 0x08
#define REG_PA_CONFIG               0x09
#define REG_LNA                     0x0C
#define REG_FIFO_ADDR_PTR           0x0D
#define REG_FIFO_TX_BASE_ADDR       0x0E
#define REG_FIFO_RX_BASE_ADDR       0x0F
#define REG_FIFO_RX_CURRENT_ADDR    0x10
#define REG_IRQ_FLAGS               0x12
#define REG_RX_NB_BYTES             0x13
#define REG_PKT_RSSI_VALUE          0x1A
#define REG_MODEM_CONFIG1           0x1D
#define REG_MODEM_CONFIG2           0x1E
#define REG_PAYLOAD_LENGTH          0x22
#define REG_DIO_MAPPING1            0x40
#define REG_VERSION                 0x42

/* Modes */
#define MODE_LONG_RANGE_MODE        0x80
#define MODE_SLEEP                  0x00
#define MODE_STDBY                  0x01
#define MODE_TX                     0x03
#define MODE_RXCONTINUOUS           0x05
#define MODE_RXSINGLE               0x06

/* IRQ flags */
#define IRQ_TX_DONE_MASK            0x08
#define IRQ_RX_DONE_MASK            0x40
#define IRQ_PAYLOAD_CRC_ERROR_MASK  0x20

static volatile uint8_t tx_done = 0;
static volatile uint8_t rx_done = 0;
static volatile uint8_t rx_crc_error = 0;
static volatile uint8_t lora_tx_in_progress = 0;

static uint8_t lora_tx_buf[256 + 1];

static osSemaphoreId_t lora_tx_sem = NULL;


static inline void CS_LOW(void)  { HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET); }
static inline void CS_HIGH(void) { HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET);   }

/* low-level singleWrite/read kept as before but used only in task context (not ISR) */

static int LoRa_singleWrite(uint8_t addr, uint8_t value)
{
    uint8_t tx[2];
    tx[0] = addr | 0x80;
    tx[1] = value;
    CS_LOW();
    if (HAL_SPI_Transmit(&hspi2, tx, 2, 100) != HAL_OK) { CS_HIGH(); return -1; }
    CS_HIGH();
    return 0;
}
static int LoRa_singleRead(uint8_t addr, uint8_t *value)
{
    uint8_t tx = addr & 0x7F;
    uint8_t rx = 0;
    CS_LOW();
    if (HAL_SPI_Transmit(&hspi2, &tx, 1, 50) != HAL_OK) { CS_HIGH(); return -1; }
    if (HAL_SPI_Receive(&hspi2, &rx, 1, 100) != HAL_OK) { CS_HIGH(); return -1; }
    CS_HIGH();
    *value = rx;
    return 0;
}

static int LoRa_readBurst(uint8_t addr, uint8_t *dst, uint16_t len)
{
    uint8_t a = addr & 0x7F;
    CS_LOW();
    if (HAL_SPI_Transmit(&hspi2, &a, 1, 50) != HAL_OK) { CS_HIGH(); return -1; }
    if (HAL_SPI_Receive(&hspi2, dst, len, 200) != HAL_OK) { CS_HIGH(); return -1; }
    CS_HIGH();
    return 0;
}

//* prepare and start DMA transmit (addr + payload) */
static int LoRa_writeBurst_DMA(uint8_t addr, const uint8_t *src, uint16_t len)
{
    if (len + 1 > sizeof(lora_tx_buf)) return -2;
    lora_tx_buf[0] = addr | 0x80;
    memcpy(&lora_tx_buf[1], src, len);

    tx_done = 0;
    lora_tx_in_progress = 1;
    CS_LOW();
    if (HAL_SPI_Transmit_DMA(&hspi2, lora_tx_buf, len + 1) != HAL_OK) {
        CS_HIGH();
        lora_tx_in_progress = 0;
        return -1;
    }
    return 0;
}

void LoRa_SetTxSemaphore(osSemaphoreId_t sem) { lora_tx_sem = sem; }

/* callback from HAL (forwarded from main HAL_SPI_TxCpltCallback) */
void LoRa_SPI_TxCplt_Callback(SPI_HandleTypeDef *hspi)
{
    if (hspi != &hspi2) return;
    /* release CS and set flags */
    CS_HIGH();
    tx_done = 1;
    lora_tx_in_progress = 0;
    if (lora_tx_sem) osSemaphoreRelease(lora_tx_sem);
}

void LoRa_DIO0_Callback(void)
{
    uint8_t irq = 0;
    if (LoRa_singleRead(REG_IRQ_FLAGS, &irq) == 0)
    {
        LoRa_singleWrite(REG_IRQ_FLAGS, irq);
        if (irq & IRQ_TX_DONE_MASK) {
            tx_done = 1;
            if (lora_tx_sem) osSemaphoreRelease(lora_tx_sem);
        }
        if (irq & IRQ_RX_DONE_MASK) {
            if (irq & IRQ_PAYLOAD_CRC_ERROR_MASK) rx_crc_error = 1;
            else rx_done = 1;
        }
    }
}
void LoRa_Reset(void)
{
    HAL_GPIO_WritePin(LORA_RST_GPIO_Port, LORA_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(5);
    HAL_GPIO_WritePin(LORA_RST_GPIO_Port, LORA_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(5);
}
int LoRa_Init(void)
{
    LoRa_Reset();
    HAL_Delay(10);

    uint8_t version = 0;
    if (LoRa_singleRead(REG_VERSION, &version) != 0) return -1;
    if (version != 0x12 && version != 0x22) {
        char msg[80];
        snprintf(msg, sizeof(msg), "LoRa: wrong version 0x%02X\r\n", version);
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        return -2;
    }

    LoRa_singleWrite(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
    HAL_Delay(10);

    LoRa_singleWrite(REG_FIFO_TX_BASE_ADDR, 0x00);
    LoRa_singleWrite(REG_FIFO_RX_BASE_ADDR, 0x00);

    LoRa_singleWrite(REG_MODEM_CONFIG1, 0x72);
    LoRa_singleWrite(REG_MODEM_CONFIG2, 0x74);

    LoRa_singleWrite(REG_PA_CONFIG, 0x80 | 0x70);
    LoRa_singleWrite(REG_DIO_MAPPING1, 0x00);

    LoRa_singleWrite(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
    HAL_Delay(10);
    return 0;
}

int LoRa_SetFrequency(uint32_t freqHz)
{
    uint64_t frf = ((uint64_t)freqHz << 19) / 32000000ULL;
    uint8_t frf_bytes[3];
    frf_bytes[0] = (frf >> 16) & 0xFF;
    frf_bytes[1] = (frf >> 8) & 0xFF;
    frf_bytes[2] = frf & 0xFF;
    if (LoRa_singleWrite(REG_FRF_MSB, frf_bytes[0]) != 0) return -1;
    if (LoRa_singleWrite(REG_FRF_MID, frf_bytes[1]) != 0) return -2;
    if (LoRa_singleWrite(REG_FRF_LSB, frf_bytes[2]) != 0) return -3;
    return 0;
}

int LoRa_Transmit_DMA(uint8_t *buffer, uint16_t len, uint32_t timeout_ms)
{
    if (len == 0 || len > 255) return -1;
    tx_done = 0;
    LoRa_singleWrite(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
    HAL_Delay(1);
    LoRa_singleWrite(REG_FIFO_ADDR_PTR, 0x00);
    if (LoRa_writeBurst_DMA(REG_FIFO, buffer, len) != 0) return -2;
    LoRa_singleWrite(REG_PAYLOAD_LENGTH, (uint8_t)len);
    LoRa_singleWrite(REG_IRQ_FLAGS, 0xFF);
    LoRa_singleWrite(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);

    if (lora_tx_sem) {
        if (osSemaphoreAcquire(lora_tx_sem, timeout_ms) != osOK) {
            /* timeout */
            HAL_SPI_Abort(&hspi2);
            CS_HIGH();
            return -3;
        }
    } else {
        uint32_t t0 = HAL_GetTick();
        while (!tx_done) {
            if ((HAL_GetTick() - t0) > timeout_ms) {
                HAL_SPI_Abort(&hspi2);
                CS_HIGH();
                return -3;
            }
            osDelay(1);
        }
    }
    tx_done = 0;
    return 0;
}
int LoRa_receive_ready(void) { return rx_done && !rx_crc_error; }

int LoRa_read_payload(uint8_t *dst, uint8_t maxlen)
{
    if (!rx_done) return -1;
    rx_done = 0;
    uint8_t nb = 0;
    if (LoRa_singleRead(REG_RX_NB_BYTES, &nb) != 0) return -2;
    if (nb > maxlen) nb = maxlen;
    uint8_t cur = 0;
    if (LoRa_singleRead(REG_FIFO_RX_CURRENT_ADDR, &cur) != 0) return -3;
    LoRa_singleWrite(REG_FIFO_ADDR_PTR, cur);
    if (LoRa_readBurst(REG_FIFO, dst, nb) != 0) return -4;
    LoRa_singleWrite(REG_IRQ_FLAGS, 0xFF);
    return (int)nb;
}
/* Integrate with HAL: forwarded from main.c HAL handlers */
//void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
//{
//    LoRa_SPI_TxCplt_Callback(hspi);
//}
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//    if (GPIO_Pin == LORA_DIO0_Pin) LoRa_DIO0_Callback();
//}
