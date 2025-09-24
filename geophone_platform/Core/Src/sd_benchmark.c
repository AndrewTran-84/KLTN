/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         : main.c
  * @brief        : Event-based Geophone Platform with Co-detection
  ******************************************************************************
  * Based on thesis: Event-based Geophone Platform with Co-detection (Akos Pasztor, 2018)
  * - Ultra-low power, event-triggered seismic sensing.
  * - Wake on event (PA2 from comparator).
  * - Read geophone (ADS1115), accel/gyro (MPU6050), temp/hum (DHT11).
  * - Log to SD, send to gateway via LoRa.
  * - Receive threshold updates from gateway, set MCP4725.
  * - Co-detection: Use MPU alongside geophone for validation.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"
#include "MCP4725.h"
#include "ADS1015_ADS1115.h"
#include "LoRa.h"
#include "sd_functions.h"
#include "DHT.h"
#include "delay_timer.h"
#include "MPU6050.h"
#include <stdio.h>  // Fix sprintf and printf

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef struct {
    ADS1xx5_I2C ads;
    MCP4725 mcp;
    LoRa lora;
    DHT_Name dht;
    Struct_MPU6050 mpu;
    float geophoneValue; // From ADS
    float temp, hum;     // From DHT
    float accX, accY, accZ, gyroX, gyroY, gyroZ; // From MPU
} SensorData;

/* Private define ------------------------------------------------------------*/
#define ADS_I2C_ADDR 0x48
#define MCP_I2C_ADDR MCP4725A0_ADDR_A00
#define LORA_FREQ 433
#define MPU_INT_PORT GPIOB
#define MPU_INT_PIN GPIO_PIN_5
#define DHT_TYPE DHT11
#define DHT_PORT GPIOA
#define DHT_PIN GPIO_PIN_0

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern TIM_HandleTypeDef htim4;

osThreadId_t SensorTaskHandle;
osThreadId_t SDTaskHandle;
osThreadId_t LoRaTaskHandle;
osMessageQueueId_t SensorQueueHandle; // Queue for sensor data to SD/LoRa

SensorData sensors;
float thresholdVoltage = 1.5f; // Default threshold 1.5V

/* USER CODE BEGIN PV */

// Declare prototype to fix warning
int16_t ADSgetLastConversionResults(ADS1xx5_I2C *i2c);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
void StartSensorTask(void *argument);
void StartSDTask(void *argument);
void StartLoRaTask(void *argument);

/* USER CODE BEGIN PFP */
void ReadSensors(SensorData *data);
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
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
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

  /* USER CODE END 2 */

  /* Call init function for FREERTOS */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* Infinite loop */
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
  ADSstartComparator_SingleEnded(&data->ads, 0, (int16_t)(thresholdVoltage / 6.144 * 32768)); // Adjust for gain
  HAL_Delay(10);
  int16_t adcRaw = ADSgetLastConversionResults(&data->ads);
  data->geophoneValue = (float)adcRaw * (6.144 / 32768); // Convert to voltage

  // Read DHT11
  DHT_ReadTempHum(&data->dht);
  data->temp = data->dht.Temp;
  data->hum = data->dht.Humi;

  // Read MPU6050 (DMA used in lib)
  MPU6050_ProcessData(&data->mpu);
  data->accX = data->mpu.acc_x;
  data->accY = data->mpu.acc_y;
  data->accZ = data->mpu.acc_z;
  data->gyroX = data->mpu.gyro_x;
  data->gyroY = data->mpu.gyro_y;
  data->gyroZ = data->mpu.gyro_z;

  // Co-detection: Validate geophone event with MPU accel (e.g., if accel > threshold)
  if (fabs(data->accX) < 0.1 && fabs(data->accY) < 0.1 && fabs(data->accZ - 1.0) < 0.1) {
      // Event valid (no movement from other sources), proceed
  } else {
      // False positive, ignore or log
  }
}

/* USER CODE END 4 */

/* Private application code ----------------------------------------------------*/
/* USER CODE BEGIN Application */
void StartSensorTask(void *argument)
{
  for(;;)
  {
    osThreadSuspend(osThreadGetId()); // Suspend until wake-up

    // On wake, read sensors
    ReadSensors(&sensors);

    // Put data to queue for SD and LoRa
    osMessageQueuePut(SensorQueueHandle, &sensors, 0, 0);

    // Resume other tasks
    osThreadResume(SDTaskHandle);
    osThreadResume(LoRaTaskHandle);

    osDelay(100); // Sampling interval or back to sleep
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

void MX_FREERTOS_Init(void)
{
  // Create queue
  SensorQueueHandle = osMessageQueueNew(1, sizeof(SensorData), NULL);

  /* SensorTask */
  const osThreadAttr_t SensorTask_attributes = {
    .name = "SensorTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t) osPriorityHigh,
  };
  SensorTaskHandle = osThreadNew(StartSensorTask, NULL, &SensorTask_attributes);

  /* SDTask */
  const osThreadAttr_t SDTask_attributes = {
    .name = "SDTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t) osPriorityNormal,
  };
  SDTaskHandle = osThreadNew(StartSDTask, NULL, &SDTask_attributes);

  /* LoRaTask */
  const osThreadAttr_t LoRaTask_attributes = {
    .name = "LoRaTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t) osPriorityNormal,
  };
  LoRaTaskHandle = osThreadNew(StartLoRaTask, NULL, &LoRaTask_attributes);
}
/* USER CODE END Application */