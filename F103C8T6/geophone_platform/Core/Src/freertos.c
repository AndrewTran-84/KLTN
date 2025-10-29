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
#include <string.h>
#include "semphr.h"
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
extern SPI_HandleTypeDef hspi1; // SD SPI
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osThreadId_t SensorTaskHandle;
osThreadId_t SDTaskHandle;
osThreadId_t LoRaTaskHandle;
osMessageQueueId_t SensorQueueHandle; // Queue for sensor data to SD/LoRa

/* Synchronization objects */
SemaphoreHandle_t xMPUSem = NULL;
SemaphoreHandle_t xADSSem = NULL;
osMutexId_t i2c2_mutex = NULL;

/* SD batching buffer */
#define SD_BATCH_BUFFER_SIZE 4096
static char sdBatchBuf[SD_BATCH_BUFFER_SIZE];
static size_t sdBatchIdx = 0;
static uint32_t sdLastFlushTick = 0;
static const uint32_t SD_FLUSH_MS = 5000; // flush at least every 5s

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

int16_t ADSgetLastConversionResults(ADS1xx5_I2C *i2c);  // for compatibility
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
  /* create mutex for I2C2 bus (ADS1115 + MCP4725) */
  i2c2_mutex = osMutexNew(NULL);
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* create binary semaphores for MPU and ADS DMA completion */
  xMPUSem = xSemaphoreCreateBinary();
  xADSSem = xSemaphoreCreateBinary();
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  SensorQueueHandle = osMessageQueueNew(8, sizeof(SensorData), NULL); // queue depth 8
  /* USER CODE END RTOS_QUEUES */

  /* USER CODE BEGIN Init Sensors (moved from main) */
  // Initialize ADS1115
  ADS1115(&sensors.ads, &hi2c2, ADS_I2C_ADDR);
  ADSsetGain(&sensors.ads, GAIN_TWOTHIRDS); // +/- 6.144V

  // Initialize MCP4725 (threshold DAC)
  sensors.mcp = MCP4725_init(&hi2c2, MCP_I2C_ADDR, 3.3f);
  if (MCP4725_isConnected(&sensors.mcp) == 0) {
      while(1); // Error
  }
  /* Protect I2C2 with mutex */
  if (i2c2_mutex) osMutexAcquire(i2c2_mutex, osWaitForever);
  MCP4725_setVoltage(&sensors.mcp, thresholdVoltage, MCP4725_REGISTER_MODE, MCP4725_POWER_DOWN_OFF);
  if (i2c2_mutex) osMutexRelease(i2c2_mutex);

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

  /* initialize sd batch */
  sdBatchIdx = 0;
  sdLastFlushTick = HAL_GetTick();
  /* USER CODE END Init Sensors */

  /* Create the thread(s) */
  /* USER CODE BEGIN RTOS_THREADS */
  const osThreadAttr_t SensorTask_attributes = {
    .name = "SensorTask",
    .stack_size = 2048,    // increased
    .priority = (osPriority_t) osPriorityHigh,
  };
  SensorTaskHandle = osThreadNew(StartSensorTask, NULL, &SensorTask_attributes);

  const osThreadAttr_t SDTask_attributes = {
    .name = "SDTask",
    .stack_size = 1536,
    .priority = (osPriority_t) osPriorityNormal,
  };
  SDTaskHandle = osThreadNew(StartSDTask, NULL, &SDTask_attributes);

  const osThreadAttr_t LoRaTask_attributes = {
    .name = "LoRaTask",
    .stack_size = 1024,
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

/* SensorTask: waits for flags from EXTI or MPU data-ready, then reads sensors */
void StartSensorTask(void *argument)
{
  for(;;)
  {
    /* Wait for wake flags: 0x1 = PA2 event wake; 0x2 = MPU data ready */
    uint32_t flags = osThreadFlagsWait(0x0003, osFlagsWaitAny, osWaitForever);
    if (flags == osFlagsError) {
        osDelay(10);
        continue;
    }

    /* Read sensors on event (or MPU interrupt) */
    ReadSensors(&sensors);

    if (sensors.isValidEvent) {
        /* Enqueue for SD + LoRa (non-blocking, drop if full) */
        osMessageQueuePut(SensorQueueHandle, &sensors, 0, 0);

        /* also append formatted CSV into internal batch buffer (thread-local) */
        char line[256];
        int len = snprintf(line, sizeof(line),
						"%lu,%.3f,%.2f,%.2f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f\n",
						(unsigned long)HAL_GetTick(), sensors.geophoneValue, sensors.temp, sensors.hum,
						sensors.accX, sensors.accY, sensors.accZ, sensors.gyroX, sensors.gyroY, sensors.gyroZ);

        /* append to sdBatchBuf with mutex-like simple protection (disable interrupts briefly) */
        taskENTER_CRITICAL();
        if ((sdBatchIdx + len) < SD_BATCH_BUFFER_SIZE) {
            memcpy(&sdBatchBuf[sdBatchIdx], line, len);
            sdBatchIdx += len;
        } else {
            /* buffer full -> force flush by setting index to size so SDTask flushes */
            sdBatchIdx = SD_BATCH_BUFFER_SIZE;
        }
        taskEXIT_CRITICAL();

        /* Signal SDTask (use thread flag) */
        osThreadFlagsSet(SDTaskHandle, 0x1);

        /* Wake LoRaTask to send */
        osThreadFlagsSet(LoRaTaskHandle, 0x1);

        printf("Valid seismic event detected! Confidence: %.2f\n", sensors.eventConfidence);
    } else {
        /* false positive */
        CO_ResetDetection();
        printf("False positive filtered out\n");
    }

    /* short cooldown */
    osDelay(10);
  }
}

/* SDTask: batch write buffer to SD periodically or on event */
void StartSDTask(void *argument)
{
  FRESULT res;
  FIL logFile;

  /* Open file once in append mode */
  res = f_open(&logFile, "geophone_log.csv", FA_OPEN_ALWAYS | FA_WRITE);
	if (res == FR_OK) {
    /* move to end for append */
    f_lseek(&logFile, f_size(&logFile));
	}
  if (res != FR_OK) {
      /* create if not exists */
      res = f_open(&logFile, "geophone_log.csv", FA_CREATE_ALWAYS | FA_WRITE);
      if (res != FR_OK) {
          // cannot open -> fatal or retry
      }
  } else {
      /* seek end to append */
      f_lseek(&logFile, f_size(&logFile));
  }

  for(;;)
  {
    /* Wait either new data flag or timeout for periodic flush */
    uint32_t flags = osThreadFlagsWait(0x1, osFlagsWaitAny, pdMS_TO_TICKS(1000)); // wake max 1s

    uint32_t now = HAL_GetTick();
    int doFlush = 0;

    taskENTER_CRITICAL();
    if (sdBatchIdx > 0 && (sdBatchIdx >= 1024 || (now - sdLastFlushTick) >= SD_FLUSH_MS || sdBatchIdx >= SD_BATCH_BUFFER_SIZE)) {
        doFlush = 1;
    }
    taskEXIT_CRITICAL();

    if (doFlush) {
        /* write buffer */
        UINT bw;
        FRESULT fres = f_write(&logFile, sdBatchBuf, sdBatchIdx, &bw);
        if (fres != FR_OK) {
            // handle error: try remount or reopen
            printf("SD write error: %d\n", fres);
        } else {
            f_sync(&logFile);
            // reset buffer
            taskENTER_CRITICAL();
            sdBatchIdx = 0;
            sdLastFlushTick = HAL_GetTick();
            taskEXIT_CRITICAL();
        }
    }
    /* small delay to yield */
    osDelay(10);
  }
}

/* LoRaTask: handle inbound threshold update and outbound transmit */
void StartLoRaTask(void *argument)
{
  SensorData data;
  for(;;)
  {
    /* wait for RX interrupt flag or periodic poll */
    osThreadFlagsWait(0x1, osFlagsWaitAny, pdMS_TO_TICKS(200)); // wake when DIO0 occurred

    /* Check for received packet (non-blocking) */
    uint8_t rxBuf[10];
    uint8_t rxLen = LoRa_receive(&sensors.lora, rxBuf, 10); // LoRa_receive reads internal IRQ flags
    if (rxLen >= 2) {
      /* update threshold (two bytes -> mV scaled as earlier) */
      uint16_t raw = (rxBuf[0] << 8) | rxBuf[1];
      float newThreshold = (float)raw / 1000.0f;

      /* Protect I2C2 while updating MCP4725 */
      if (i2c2_mutex) osMutexAcquire(i2c2_mutex, osWaitForever);
      MCP4725_setVoltage(&sensors.mcp, newThreshold, MCP4725_REGISTER_MODE, MCP4725_POWER_DOWN_OFF);
      if (i2c2_mutex) osMutexRelease(i2c2_mutex);

      thresholdVoltage = newThreshold;
      printf("Threshold updated via LoRa: %.3f V\n", thresholdVoltage);
    }

    /* If there's a sensor sample queued for transmit, send it */
    if (osMessageQueueGet(SensorQueueHandle, &data, NULL, 0) == osOK) {
      uint8_t txBuf[32] = {0};
      /* simple packing: 4 bytes float each - be careful on endianness */
      memcpy(&txBuf[0], &data.geophoneValue, 4);
      memcpy(&txBuf[4], &data.temp, 4);
      memcpy(&txBuf[8], &data.hum, 4);
      memcpy(&txBuf[12], &data.accX, 4);
      memcpy(&txBuf[16], &data.accY, 4);
      memcpy(&txBuf[20], &data.accZ, 4);
      // keep rest zeros
      LoRa_transmit(&sensors.lora, txBuf, 24, 200);
    }

    osDelay(50);
  }
}

/* EXTI callback: set thread flags (ISR-safe) */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if (GPIO_Pin == GPIO_PIN_2) { // PA2 wake-up
      osThreadFlagsSet(SensorTaskHandle, 0x1);
  }
  if (GPIO_Pin == GPIO_PIN_1) { // PB1 LoRa DIO0
      osThreadFlagsSet(LoRaTaskHandle, 0x1);
  }
  if (GPIO_Pin == GPIO_PIN_5) { // PB5 MPU INT (data ready)
      osThreadFlagsSet(SensorTaskHandle, 0x2);
  }
}

/* ReadSensors: use ADS DMA read + protect I2C2 with mutex */
void ReadSensors(SensorData *data)
{
  /* ADS1115 comparator set and DMA read */
  if (i2c2_mutex) osMutexAcquire(i2c2_mutex, osWaitForever);

  ADSstartComparator_SingleEnded(&data->ads, 0, (int16_t)(thresholdVoltage / 6.144f * 32768.0f));
  /* use DMA read and wait ticks */
  int16_t adcRaw = ADSreadADC_SingleEnded_DMA(&data->ads, 0, pdMS_TO_TICKS(100));
  data->geophoneValue = (float)adcRaw * (6.144f / 32768.0f);

  if (i2c2_mutex) osMutexRelease(i2c2_mutex);

  /* Read DHT (blocking but rare) */
  DHT_ReadTempHum(&data->dht);
  data->temp = data->dht.Temp;
  data->hum = data->dht.Humi;

  /* Read MPU6050 (DMA read inside) */
  MPU6050_ProcessData(&data->mpu);
  data->accX = data->mpu.acc_x;
  data->accY = data->mpu.acc_y;
  data->accZ = data->mpu.acc_z;
  data->gyroX = data->mpu.gyro_x;
  data->gyroY = data->mpu.gyro_y;
  data->gyroZ = data->mpu.gyro_z;

  /* Co-detection validation */
  uint32_t currentTime = HAL_GetTick();
  uint8_t isValidEvent = CO_IsValidSeismicEvent(
      data->geophoneValue,
      data->accX, data->accY, data->accZ,
      data->gyroX, data->gyroY, data->gyroZ,
      currentTime
  );

  float eventConfidence = CO_CalculateEventConfidence(
      data->geophoneValue,
      CO_CalculateAccelMagnitude(data->accX, data->accY, data->accZ),
      CO_CalculateGyroMagnitude(data->gyroX, data->gyroY, data->gyroZ)
  );

  data->eventConfidence = eventConfidence;
  data->isValidEvent = isValidEvent;

  if (!isValidEvent) {
      /* reset state, do not queue */
      CO_ResetDetection();
      return;
  }

  CO_ResetDetection();
}

/* USER CODE END Application */
