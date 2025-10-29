/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "tim.h"
#include "dac.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "dma.h"
#include "ads1115.h"
#include "DHT.h"
#include "MPU6050.h"
#include "LoRa.h"
#include "sd_functions.h"
#include "sd_benchmark.h"
#include "sd_diskio_spi.h"
#include "sd_spi.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
DHT_Name DHT1;
LoRa myLoRa;
uint16_t adc_value;
float voltage = 3.0; // DAC 3.0V
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* FreeRTOS queues */
QueueHandle_t dataQueue;

/* Tasks prototypes */
void AcquisitionTask(void *pvParameters);
void LoggingTask(void *pvParameters);
void CommunicationTask(void *pvParameters);
void HealthTask(void *pvParameters);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	// Cau hình DAC
  HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint32_t)( (3.0 / 3.3) * 4095 ));
	
	DHT_Init(&DHT1, DHT11, &htim4, GPIOA, GPIO_PIN_0);
  MPU6050_Initialization();
  ADS1115_Init();
  SD_SPI_Init(); // SD card
  sd_mount();
  myLoRa.hSPIx = &hspi2;
  myLoRa.CS_port = GPIOB;
  myLoRa.CS_pin = GPIO_PIN_12;
  myLoRa.reset_port = GPIOB;
  myLoRa.reset_pin = GPIO_PIN_0;
  LoRa_reset(&myLoRa);
  LoRa_init(&myLoRa);
  LoRa_startReceiving(&myLoRa);
	
	dataQueue = xQueueCreate(10, sizeof(float));
	
	//task
	xTaskCreate(AcquisitionTask, "Acquisition", 128, NULL, 3, NULL); // High priority
  xTaskCreate(LoggingTask, "Logging", 128, NULL, 2, NULL); // Medium priority
  xTaskCreate(CommunicationTask, "Communication", 128, NULL, 1, NULL); // Low priority
  xTaskCreate(HealthTask, "Health", 128, NULL, 1, NULL); // Low priority

  vTaskStartScheduler();
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
/* Acquisition Task: Thu thap du lieu khi trigger */
void AcquisitionTask(void *pvParameters)
{
  for (;;) {
    if (xSemaphoreTake(triggerSemaphore, portMAX_DELAY) == pdTRUE) {
      Read_ADS1115_DMA();
      Read_MPU6050_DMA();
      DHT_ReadTempHum(&DHT1);

      xQueueSend(dataQueue, &voltage, pdMS_TO_TICKS(10));
    }
  }
}

/* Logging Task: Luu du lieu vao SD card */
void LoggingTask(void *pvParameters)
{
  for (;;) {
    float data;
    if (xQueueReceive(dataQueue, &data, portMAX_DELAY) == pdTRUE) {
      sd_write_file("data.txt", "Voltage: %.2f\n", data); // Ví d?
    }
  }
}

/* Communication Task: Gui du lieu qua LoRa, xu ly remote config */
void CommunicationTask(void *pvParameters)
{
  for (;;) {
    float data;
    if (xQueueReceive(dataQueue, &data, pdMS_TO_TICKS(100)) == pdTRUE) {
      uint8_t txData[10];
      sprintf((char*)txData, "%.2f", data);
      LoRa_transmit(&myLoRa, txData, strlen((char*)txData), 2000);
    }

    uint8_t rxData[10] = {0};
    uint8_t len = LoRa_receive(&myLoRa, rxData, 10);
    if (len > 0) {
      float newDAC = atof((char*)rxData);
      HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint32_t)((newDAC / 3.3) * 4095));
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

/* Health Task: Kiem tra he th?ng dinh ky */
void HealthTask(void *pvParameters)
{
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_0) { // PB0
    xSemaphoreGiveFromISR(triggerSemaphore, NULL); 
  }
}
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
