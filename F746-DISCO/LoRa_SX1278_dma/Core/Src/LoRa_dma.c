/**
 * LoRa_dma.c  -- minimal SX127x driver using SPI2 + HAL DMA TX + DIO0 exti
 * - uses extern hspi2 (CubeMX)
 * - uses software NSS on PI0 (LORA_NSS_Pin)
 *
 * Notes:
 * - CubeMX must configure: SPI2 (master, 8-bit, CPOL=0, CPHA=0, NSS Soft),
 *   DMA for SPI2_TX and NVIC for DMA stream.
 * - Configure PI0 as GPIO output (set HIGH idle), PI2 as EXTI rising, PA15 as GPIO output for RST.
 *
 * This is a compact implementation for testing TX/RX flow.
 */

#include "LoRa_dma.h"
#include <string.h>
#include <stdio.h>

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

/* local state */
static volatile uint8_t tx_done = 0;
static volatile uint8_t rx_done = 0;
static volatile uint8_t rx_crc_error = 0;

/* temporary buffer for DMA transmit (addr + payload) */
static uint8_t lora_tx_buf[256 + 1]; /* support up to 256 payload (SX1278 max 255) */

/* helper: set NSS low/high */
static inline void CS_LOW(void)  { HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET); }
static inline void CS_HIGH(void) { HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET);   }

/* low-level read/write SPI (blocking) */
static int LoRa_singleWrite(uint8_t addr, uint8_t value)
{
    uint8_t tx[2];
    tx[0] = addr | 0x80; /* write: MSB=1 */
    tx[1] = value;
    CS_LOW();
    if (HAL_SPI_Transmit(&hspi2, tx, 2, 100) != HAL_OK) { CS_HIGH(); return -1; }
    CS_HIGH();
    return 0;
}

static int LoRa_singleRead(uint8_t addr, uint8_t *value)
{
    uint8_t tx = addr & 0x7F; /* read: MSB=0 */
    uint8_t rx = 0;
    CS_LOW();
    if (HAL_SPI_Transmit(&hspi2, &tx, 1, 100) != HAL_OK) { CS_HIGH(); return -1; }
    if (HAL_SPI_Receive(&hspi2, &rx, 1, 100) != HAL_OK) { CS_HIGH(); return -1; }
    CS_HIGH();
    *value = rx;
    return 0;
}

/* block read N bytes from reg (addr is FIFO or reg address) */
static int LoRa_readBurst(uint8_t addr, uint8_t *dst, uint16_t len)
{
    uint8_t a = addr & 0x7F;
    CS_LOW();
    if (HAL_SPI_Transmit(&hspi2, &a, 1, 50) != HAL_OK) { CS_HIGH(); return -1; }
    if (HAL_SPI_Receive(&hspi2, dst, len, 200) != HAL_OK) { CS_HIGH(); return -1; }
    CS_HIGH();
    return 0;
}

/* block write N bytes to reg (addr is FIFO or reg address) using HAL_SPI_Transmit_DMA */
static int LoRa_writeBurst_DMA(uint8_t addr, const uint8_t *src, uint16_t len)
{
    if (len + 1 > sizeof(lora_tx_buf)) return -2;
    /* prepare buffer: [addr|0x80] [data...] */
    lora_tx_buf[0] = addr | 0x80;
    memcpy(&lora_tx_buf[1], src, len);

    tx_done = 0;
    CS_LOW();
    /* transmit (addr + data) via DMA */
    if (HAL_SPI_Transmit_DMA(&hspi2, lora_tx_buf, len + 1) != HAL_OK)
    {
        CS_HIGH();
        return -1;
    }
    /* Wait (blocking) for completion or timeout; you can change to non-blocking if using RTOS */
    uint32_t t0 = HAL_GetTick();
    while (!tx_done)
    {
        if ((HAL_GetTick() - t0) > 2000) /* 2s timeout */
        {
            /* abort DMA */
            HAL_SPI_Abort(&hspi2);
            CS_HIGH();
            return -2;
        }
    }
    /* CS_HIGH is set in callback */
    return 0;
}

/* exported callback (to be called from application HAL_SPI_TxCpltCallback) */
void LoRa_SPI_TxCplt_Callback(SPI_HandleTypeDef *hspi)
{
    /* ensure this is the SPI we used */
    if (hspi == &hspi2)
    {
        /* end of DMA transfer -> release CS and mark done */
        CS_HIGH();
        tx_done = 1;
    }
}

/* exported callback for DIO0 (to be called from HAL_GPIO_EXTI_Callback) */
void LoRa_DIO0_Callback(void)
{
    /* mark rx or tx - further processing in main or LoRa_read_payload */
    uint8_t irq = 0;
    if (LoRa_singleRead(REG_IRQ_FLAGS, &irq) == 0)
    {
        /* clear IRQs */
        LoRa_singleWrite(REG_IRQ_FLAGS, irq);
        if (irq & IRQ_TX_DONE_MASK) {
            tx_done = 1;
        }
        if (irq & IRQ_RX_DONE_MASK) {
            if (irq & IRQ_PAYLOAD_CRC_ERROR_MASK) rx_crc_error = 1;
            else rx_done = 1;
        }
    }
}

/* initialize LoRa radio */
int LoRa_Init(void)
{
    /* reset chip */
    LoRa_Reset();
    HAL_Delay(10);

    uint8_t version = 0;
    if (LoRa_singleRead(REG_VERSION, &version) != 0) return -1;
    if (version != 0x12 && version != 0x22) { /* 0x12 for SX1276/7/8; some variants different */
        char msg[64];
        snprintf(msg, sizeof(msg), "LoRa: wrong version 0x%02X\r\n", version);
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        return -2;
    }

    /* put in sleep to allow configuration */
    LoRa_singleWrite(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
    HAL_Delay(10);

    /* set FIFO base addresses (defaults OK but set anyway) */
    LoRa_singleWrite(REG_FIFO_TX_BASE_ADDR, 0x00);
    LoRa_singleWrite(REG_FIFO_RX_BASE_ADDR, 0x00);

    /* Modem config: bandwidth, coding rate, implicit header */
    /* Here set reasonable defaults: BW125k, CR4/5, Explicit header mode */
    LoRa_singleWrite(REG_MODEM_CONFIG1, 0x72); /* BW=125kHz, CR=4/5, Explicit */
    LoRa_singleWrite(REG_MODEM_CONFIG2, 0x74); /* SF=7, CRC on (bit2) */

    /* PA config */
    LoRa_singleWrite(REG_PA_CONFIG, 0x80 | 0x70); /* PA_BOOST, OutputPower=0x7 (max) */

    /* set DIO0 mapped to RxDone/TxDone (DIO0 = 0x00 mapping default) */
    LoRa_singleWrite(REG_DIO_MAPPING1, 0x00);

    /* set to standby */
    LoRa_singleWrite(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
    HAL_Delay(10);

    return 0;
}

void LoRa_Reset(void)
{
    /* active low reset: toggle RST pin */
    HAL_GPIO_WritePin(LORA_RST_GPIO_Port, LORA_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(5);
    HAL_GPIO_WritePin(LORA_RST_GPIO_Port, LORA_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(5);
}

/* set frequency in Hz (e.g., 433000000) */
int LoRa_SetFrequency(uint32_t freqHz)
{
    /* FRF = freq / (32e6 / 2^19) */
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

/* Transmit payload using DMA (blocking wait with timeout for simplicity) */
/* returns 0 on success, negative on error */
int LoRa_Transmit_DMA(uint8_t *buffer, uint16_t len, uint32_t timeout_ms)
{
    if (len == 0 || len > 255) return -1;

    tx_done = 0;

    /* set to standby */
    LoRa_singleWrite(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
    HAL_Delay(1);

    /* set FIFO addr ptr to tx base */
    LoRa_singleWrite(REG_FIFO_ADDR_PTR, 0x00);
    /* write payload to FIFO via DMA */
    if (LoRa_writeBurst_DMA(REG_FIFO, buffer, len) != 0) return -2;

    /* set payload length */
    LoRa_singleWrite(REG_PAYLOAD_LENGTH, (uint8_t)len);

    /* clear IRQ flags */
    LoRa_singleWrite(REG_IRQ_FLAGS, 0xFF);

    /* set mode to TX */
    LoRa_singleWrite(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);

    /* wait for tx_done (set either by DIO0 interrupt or DMA callback) */
    uint32_t start = HAL_GetTick();
    while (!tx_done)
    {
        if ((HAL_GetTick() - start) > timeout_ms) {
            /* timeout */
            return -3;
        }
    }
    /* tx_done cleared by caller if needed */
    return 0;
}

/* check if a packet is ready (set by DIO0 callback) */
int LoRa_receive_ready(void)
{
    return rx_done && !rx_crc_error;
}

/* read payload into dst, return num bytes or negative on error */
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

    /* clear irq flags */
    LoRa_singleWrite(REG_IRQ_FLAGS, 0xFF);

    return (int)nb;
}

/* Simple wrapper for application: must be called from HAL_SPI_TxCpltCallback */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    /* bubble event to library */
    LoRa_SPI_TxCplt_Callback(hspi);
}

/* Application should call HAL_GPIO_EXTI_Callback on EXTI line; we provide a small handler */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == LORA_DIO0_Pin) {
        LoRa_DIO0_Callback();
    }
}
