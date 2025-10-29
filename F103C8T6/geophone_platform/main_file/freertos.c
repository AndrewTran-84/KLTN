/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "co_detection.h"
#include "MCP4725.h"
#include "ADS1015_ADS1115.h"
#include "LoRa.h"
#include "sd_functions.h"
#include "DHT.h"
#include "delay_timer.h"
#include "MPU6050.h"
#include <stdio.h> 
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    ADS1xx5_I2C ads;
    MCP4725 mcp;
    LoRa lora;
    DHT_Name dht;
    Struct_MPU6050 mpu;
    float geophoneValue;     // From ADS
    float temp, hum;         // From DHT
    float accX, accY, accZ, gyroX, gyroY, gyroZ; // From MPU
    float eventConfidence;   // Co-detection confidence (0-1)
    uint8_t isValidEvent;    // Event validation result
} SensorData;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADS_I2C_ADDR 0x48
#define MCP_I2C_ADDR MCP4725A0_ADDR_A00
#define LORA_FREQ 433
#define MPU_INT_PORT GPIOB
#define MPU_INT_PIN GPIO_PIN_5
#define DHT_TYPE DHT11
#define DHT_PORT GPIOA
#define DHT_PIN GPIO_PIN_0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
extern I2C_HandleTypeDef hi2c2;
extern TIM_HandleTypeDef htim4;
extern SPI_HandleTypeDef hspi2;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osThreadId_t SensorTaskHandle;
osThreadId_t SDTaskHandle;
osThreadId_t LoRaTaskHandle;
osMessageQueueId_t SensorQueueHandle; // Queue for sensor data to SD/LoRa

SensorData sensors;
float thresholdVoltage = 1.5f; // Default threshold 1.5V
/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartSensorTask(void *argument);
void StartSDTask(void *argument);
void StartLoRaTask(void *argument);
void ReadSensors(SensorData *data);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

int16_t ADSgetLastConversionResults(ADS1xx5_I2C *i2c);  // Declaration d? fix warning
/* USER CODE END FunctionPrototypes */

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  SensorQueueHandle = osMessageQueueNew(1, sizeof(SensorData), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* USER CODE BEGIN Init Sensors (Di chuy?n t? main.c) */
  // Initialize ADS1115
  ADS1115(&sensors.ads, &hi2c2, ADS_I2C_ADDR);
  ADSsetGain(&sensors.ads, GAIN_TWOTHIRDS); // +/- 6.144V

  // Initialize MCP4725 (threshold DAC)
  sensors.mcp = MCP4725_init(&hi2c2, MCP_I2C_ADDR, 3.3f);
  if (MCP4725_isConnected(&sensors.mcp) == 0) {
      while(1); // Error
  }
  MCP4725_setVoltage(&sensors.mcp, thresholdVoltage, MCP4725_REGISTER_MODE, MCP4725_POWER_DOWN_OFF);

  // Initialize DHT11
  DHT_Init(&sensors.dht, DHT_TYPE, &htim4, DHT_PORT, DHT_PIN);

  // Initialize MPU6050
  MPU6050_Initialization();

  // Initialize LoRa
  sensors.lora.hSPIx = &hspi2;
  sensors.lora.CS_port = GPIOB;
  sensors.lora.CS_pin = GPIO_PIN_12;
  sensors.lora.reset_port = GPIOB;
  sensors.lora.reset_pin = GPIO_PIN_0;
  LoRa_reset(&sensors.lora);
  if (LoRa_init(&sensors.lora) != LORA_OK) {
      while(1); // Error
  }
  LoRa_startReceiving(&sensors.lora);

  // Mount SD
  if (sd_mount() != FR_OK) {
      while(1); // Error
  }
  /* USER CODE END Init Sensors */

  /* Create the thread(s) */
  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  const osThreadAttr_t SensorTask_attributes = {
    .name = "SensorTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t) osPriorityHigh,
  };
  SensorTaskHandle = osThreadNew(StartSensorTask, NULL, &SensorTask_attributes);

  const osThreadAttr_t SDTask_attributes = {
    .name = "SDTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t) osPriorityNormal,
  };
  SDTaskHandle = osThreadNew(StartSDTask, NULL, &SDTask_attributes);

  const osThreadAttr_t LoRaTask_attributes = {
    .name = "LoRaTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t) osPriorityNormal,
  };
  LoRaTaskHandle = osThreadNew(StartLoRaTask, NULL, &LoRaTask_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void StartSensorTask(void *argument)
{
  for(;;)
  {
    osThreadSuspend(osThreadGetId()); // Suspend until wake-up

    // On wake, read sensors
    ReadSensors(&sensors);

    // Only proceed if co-detection confirms valid event
    if (sensors.isValidEvent) {
        // Put data to queue for SD and LoRa
        osMessageQueuePut(SensorQueueHandle, &sensors, 0, 0);

        // Resume other tasks
        osThreadResume(SDTaskHandle);
        osThreadResume(LoRaTaskHandle);
        
        printf("Valid seismic event detected! Confidence: %.2f\n", sensors.eventConfidence);
    } else {
        printf("False positive filtered out\n");
    }

    osDelay(10); // Brief delay before next potential event
  }
}

void StartSDTask(void *argument)
{
  SensorData data;
  for(;;)
  {
    if (osMessageQueueGet(SensorQueueHandle, &data, NULL, osWaitForever) == osOK) {
      // Log to SD (CSV format)
      char logStr[256];
      sprintf(logStr, "%u,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
              HAL_GetTick(), data.geophoneValue, data.temp, data.hum,
              data.accX, data.accY, data.accZ, data.gyroX, data.gyroY, data.gyroZ);
      sd_write_file("geophone_log.csv", logStr);
    }
  }
}

void StartLoRaTask(void *argument)
{
  SensorData data;
  for(;;)
  {
    // Receive from gateway
    uint8_t rxBuf[10];
    uint8_t rxLen = LoRa_receive(&sensors.lora, rxBuf, 10);
    if (rxLen >= 2) {
      // Update threshold (2 bytes float)
      thresholdVoltage = (float)((rxBuf[0] << 8) | rxBuf[1]) / 1000.0f;
      MCP4725_setVoltage(&sensors.mcp, thresholdVoltage, MCP4725_REGISTER_MODE, MCP4725_POWER_DOWN_OFF);
    }

    // Send data on event
    if (osMessageQueueGet(SensorQueueHandle, &data, NULL, 0) == osOK) {
      // Pack data (simplified, adjust size)
      uint8_t txBuf[20] = {0}; // Example: geophone, temp, hum, accXYZ, gyroXYZ
      // Fill txBuf with data... (ví d?: memcpy ho?c cast float to bytes)
      *(float*)&txBuf[0] = data.geophoneValue;
      *(float*)&txBuf[4] = data.temp;
      // ... thêm các tru?ng khác
      LoRa_transmit(&sensors.lora, txBuf, 20, 100);
    }

    osDelay(50);
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_2) { // PA2 wake-up from event
      osThreadResume(SensorTaskHandle); // Resume sensor task
  }
  if (GPIO_Pin == GPIO_PIN_1) { // PB1 LoRa DIO0
      osThreadResume(LoRaTaskHandle); // Handle LoRa RX
  }
  if (GPIO_Pin == GPIO_PIN_5) { // PB5 MPU INT
      // Data ready, read MPU in sensor task
      osThreadResume(SensorTaskHandle);
  }
}

void ReadSensors(SensorData *data)
{
  // Read ADS1115 (geophone digitized)
  ADSstartComparator_SingleEnded(&data->ads, 0, (int16_t)(thresholdVoltage / 6.144 * 32768));
  HAL_Delay(10);
  int16_t adcRaw = ADSgetLastConversionResults(&data->ads);
  data->geophoneValue = (float)adcRaw * (6.144 / 32768); // Convert to voltage

  // Read DHT11
  DHT_ReadTempHum(&data->dht);
  data->temp = data->dht.Temp;
  data->hum = data->dht.Humi;

  // Read MPU6050
  MPU6050_ProcessData(&data->mpu);
  data->accX = data->mpu.acc_x;
  data->accY = data->mpu.acc_y;
  data->accZ = data->mpu.acc_z;
  data->gyroX = data->mpu.gyro_x;
  data->gyroY = data->mpu.gyro_y;
  data->gyroZ = data->mpu.gyro_z;

  // CO-DETECTION: Validate seismic event using multiple sensors
  uint32_t currentTime = HAL_GetTick();
  uint8_t isValidEvent = CO_IsValidSeismicEvent(
      data->geophoneValue, 
      data->accX, data->accY, data->accZ,
      data->gyroX, data->gyroY, data->gyroZ,
      currentTime
  );

  // Calculate event confidence for logging
  float eventConfidence = CO_CalculateEventConfidence(
      data->geophoneValue,
      CO_CalculateAccelMagnitude(data->accX, data->accY, data->accZ),
      CO_CalculateGyroMagnitude(data->gyroX, data->gyroY, data->gyroZ)
  );

  // Add confidence to data structure for logging
  data->eventConfidence = eventConfidence;
  data->isValidEvent = isValidEvent;

  // Only proceed if this is a valid seismic event
  if (!isValidEvent) {
      // Reset detection state for next event
      CO_ResetDetection();
      return; // Don't process false positives
  }

  // Valid event - reset detection for next event
  CO_ResetDetection();
}
/* USER CODE END Application */