/* main.c - ADS1115 read + LoRa transmit + UART status messages */
#include "main.h"
#include "spi.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#include "LoRa.h"
#include "ADS1015_ADS1115.h"

/* External peripheral handles (CubeMX generated) */
extern SPI_HandleTypeDef hspi2;
extern I2C_HandleTypeDef hi2c2;
extern UART_HandleTypeDef huart1;

void SystemClock_Config(void);

/* LoRa object and ADS object */
LoRa myLoRa;
ADS1xx5_I2C ads1115;

#define ADS1115_ADDR ADS_ADDR_GND  // nhu b?n dã d?nh (ADDR -> GND)

/* Helper to send C-string to UART1 */
static void uart_print(const char *s)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)s, strlen(s), HAL_MAX_DELAY);
}

/* Helper to send formatted string to UART1 */
static void uart_printf(const char *fmt, ...)
{
    char buf[128];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n > 0) {
        if (n > (int)sizeof(buf)) n = sizeof(buf);
        HAL_UART_Transmit(&huart1, (uint8_t*)buf, n, HAL_MAX_DELAY);
    }
}


int main(void)
{
    HAL_Init();
    SystemClock_Config();

    /* Initialize peripherals (CubeMX generated functions) */
    MX_GPIO_Init();
    MX_SPI2_Init();
    MX_I2C2_Init();
    MX_USART1_UART_Init();

    /* Quick UART test */
    uart_print("=== STM32 LoRa + ADS1115 Sender ===\r\n");

    /* --- Initialize ADS1115 object --- */
    ADS1115(&ads1115, &hi2c2, ADS1115_ADDR);
    ADSsetGain(&ads1115, GAIN_ONE); // gain x1 -> ±4.096V
    HAL_Delay(10);

    /* --- Initialize LoRa object --- */
    myLoRa = newLoRa();

    /* Map hardware pins - ensure these macros exist in gpio.h from CubeMX */
    myLoRa.CS_port = NSS_GPIO_Port;      // PB12
    myLoRa.CS_pin  = NSS_Pin;
    myLoRa.reset_port = RST_GPIO_Port;   // PB0
    myLoRa.reset_pin  = RST_Pin;
    myLoRa.DIO0_port  = DIO0_GPIO_Port;  // PB1
    myLoRa.DIO0_pin   = DIO0_Pin;
    myLoRa.hSPIx = &hspi2;

    /* Optional: perform hardware reset via library function */
    LoRa_reset(&myLoRa);
    HAL_Delay(50);

    /* Initialize LoRa module */
    uint16_t lres = LoRa_init(&myLoRa);
    if (lres == LORA_OK) {
        uart_print("LoRa init: OK\r\n");
    } else {
        uart_printf("LoRa init: FAILED (code=%u)\r\n", (unsigned)lres);
        /* continue anyway so you can debug ADS1115 or try reinit later */
    }

    /* Main loop: read ADS1115, send via LoRa, print UART status */
    while (1)
    {
        /* Read ADC (single-ended channel 0) */
        uint16_t raw = ADSreadADC_SingleEnded(&ads1115, 0);

        /* Convert to voltage: ADS1115 with gain GAIN_ONE => LSB = 0.125mV (per header notes)
           In your earlier code you used: voltage = (adc_value * 4.096) / 32767.0;
           That formula assumes 15-bit signed? For consistency use same formula:
        */
        float voltage = (raw * 4.096f) / 32767.0f;

        /* Prepare payload (ASCII) */
        char payload[64];
        int payload_len = snprintf(payload, sizeof(payload), "ADS: %u, V:%.3f", raw, voltage);
        if (payload_len <= 0) payload_len = 0;
        if (payload_len > (int)sizeof(payload)) payload_len = sizeof(payload);

        /* Transmit via LoRa (timeout 2000 ms) */
        uint8_t tx_ok = 0;
        if (lres == LORA_OK) {
            tx_ok = LoRa_transmit(&myLoRa, (uint8_t*)payload, (uint8_t)payload_len, 2000);
        } else {
            /* If LoRa init failed earlier, try re-init once before sending */
            uint16_t try_init = LoRa_init(&myLoRa);
            if (try_init == LORA_OK) {
                tx_ok = LoRa_transmit(&myLoRa, (uint8_t*)payload, (uint8_t)payload_len, 2000);
                lres = try_init;
            } else {
                tx_ok = 0;
            }
        }

        /* UART message as requested */
        char uart_msg[128];
        if (tx_ok) {
            /* "Ðã g?i (Giá tr? ADS1115) v? Gateway" */
            int n = snprintf(uart_msg, sizeof(uart_msg), "Sent (%.3f V) to Gateway\r\n", voltage);
            if (n > 0) HAL_UART_Transmit(&huart1, (uint8_t*)uart_msg, n, HAL_MAX_DELAY);
        } else {
            int n = snprintf(uart_msg, sizeof(uart_msg), "Unsuccessfully Sent (%.3f V)\r\n", voltage);
            if (n > 0) HAL_UART_Transmit(&huart1, (uint8_t*)uart_msg, n, HAL_MAX_DELAY);
        }

        /* Wait before next measurement/transmit */
        HAL_Delay(1000); // g?i 1g/s b?n có th? ch?nh
    }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
