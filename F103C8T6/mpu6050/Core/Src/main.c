/* main_modified.c - polling MPU6050 and sending with HAL_UART_Transmit */
#include "main.h"
#include "stdio.h"
#include "string.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "MPU6050.h"

/* Private variables ---------------------------------------------------------*/
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USART1_UART_Init();

    /* UART quick test */
    {
        char msg[] = "=== UART TEST: STM32 ready ===\r\n";
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    }

    /* I2C device check (MPU6050) */
    if (HAL_I2C_IsDeviceReady(&hi2c1, MPU6050_ADDR, 3, 100) == HAL_OK) {
        char ok[] = "I2C: MPU6050 detected\r\n";
        HAL_UART_Transmit(&huart1, (uint8_t*)ok, strlen(ok), HAL_MAX_DELAY);
    } else {
        char nok[] = "I2C: MPU6050 NOT detected - check wiring/pullups/addr\r\n";
        HAL_UART_Transmit(&huart1, (uint8_t*)nok, strlen(nok), HAL_MAX_DELAY);
        /* Continue anyway to help debugging */
    }

    /* Initialize MPU6050 */
    MPU6050_Initialization();

    /* Main loop: poll sensor and send data over UART (direct HAL transmit) */
    while (1)
    {
        MPU6050_ProcessData(&MPU6050);

        char buf[160];
        int len = snprintf(buf, sizeof(buf),
            "Accel: X=%.2f g, Y=%.2f g, Z=%.2f g\r\n"
            "Gyro : X=%.2f dps, Y=%.2f dps, Z=%.2f dps\r\n"
            "Temp : %.2f C\r\n\r\n",
            MPU6050.acc_x, MPU6050.acc_y, MPU6050.acc_z,
            MPU6050.gyro_x, MPU6050.gyro_y, MPU6050.gyro_z,
            MPU6050.temperature);

        if (len > 0) {
            if (len > (int)sizeof(buf)) len = sizeof(buf);
            HAL_UART_Transmit(&huart1, (uint8_t*)buf, len, HAL_MAX_DELAY);
        }

        HAL_Delay(100);
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
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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