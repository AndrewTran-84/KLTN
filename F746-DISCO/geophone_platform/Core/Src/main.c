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
#include "cmsis_os.h"
#include "fatfs.h"
#include "rtc.h"
#include <string.h>
#include <stdio.h>

/* drivers */
#include "ADS1115_dma.h"
#include "MPU6050_dma.h"
#include "MCP4725_dma.h"
#include "LoRa_dma.h"
#include "DHT11.h"

/* extern handles from CubeMX */
extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi2;
extern UART_HandleTypeDef huart1;
extern SD_HandleTypeDef hsd1;
extern RTC_HandleTypeDef hrtc;

/* --- RTOS objects --- */
static osMutexId_t i2c_mutex = NULL;

/* semaphores used by drivers (binary semaphores used to signal DMA completion) */
static osSemaphoreId_t sem_ads = NULL;
static osSemaphoreId_t sem_mpu = NULL;
static osSemaphoreId_t sem_mcp = NULL;
static osSemaphoreId_t sem_lora_tx = NULL;

/* queues */
typedef enum { LOGTYPE_ACQ=1, LOGTYPE_IMU, LOGTYPE_HEALTH, LOGTYPE_EVENT } LogType_t;
typedef struct {
    LogType_t type;
    uint32_t timestamp; /* unix or epoch seconds from RTC */
    uint8_t *data;      /* pointer to payload (must be allocated by producer) */
    uint16_t len;
} LogMsg_t;

static osMessageQueueId_t logQueue = NULL;

/* Task prototypes */
static void InitTask(void *arg);
static void AcquisitionTask(void *arg);
static void IMUTask(void *arg);
static void CommTask(void *arg);
static void LoggingTask(void *arg);
static void HealthTask(void *arg);
static void TimerDaemonTask(void *arg);

/* helper: get RTC timestamp (seconds since 1970 not implemented, provide simple YMDHMS) */
static void get_rtc_time_string(char *buf, size_t len)
{
    RTC_TimeTypeDef sTime;
    RTC_DateTypeDef sDate;
    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN); // must call GetDate after GetTime
    snprintf(buf, len, "%04u-%02u-%02u %02u:%02u:%02u",
             2000 + sDate.Year, sDate.Month, sDate.Date,
             sTime.Hours, sTime.Minutes, sTime.Seconds);
}

/* printf redirection */
int __io_putchar(int ch) {
    uint8_t c = (uint8_t)ch;
    HAL_UART_Transmit(&huart1, &c, 1, HAL_MAX_DELAY);
    return ch;
}
int fputc(int ch, FILE *f) { return __io_putchar(ch); }

/* Forward HAL callbacks (single place) */
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    MCP4725_DMA_TxCpltCallback(hi2c);
}
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    /* forward to driver callbacks which set flags/semaphores */
    ADS1115_DMA_RxCpltCallback(hi2c);      // driver should check hi2c pointer
    MPU6050_DMA_I2C_RxCpltCallback(hi2c);
    MCP4725_DMA_RxCpltCallback(hi2c);      // if you implemented callback for write/read
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    ADS1115_DMA_ErrorCallback(hi2c);
    MPU6050_DMA_I2C_ErrorCallback(hi2c);
    MCP4725_DMA_ErrorCallback(hi2c);
}

/* SPI TX complete -> forward to LoRa driver */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    LoRa_SPI_TxCplt_Callback(hspi);
}

/* GPIO exti forwarder: DIO0 -> LoRa, Comparator (PA0) -> notify Acquisition */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == LORA_DIO0_Pin) {
        LoRa_DIO0_Callback();
    }
    /* comparator LM393 output wired to PA0 -> notify AcquisitionTask */
    if (GPIO_Pin == GPIO_PIN_0) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        /* If using CMSIS v2, use osThreadFlagsSet from ISR-safe wrapper:
           osThreadFlagsSet doesn't have FROM_ISR variant; we will use a direct-to-task
           notify simulation: in our design we will set an IRQ flag through HAL and then
           use osThreadFlagsSet from main context; to keep simple, we'll use a global flag and a semaphore. */
        /* Simpler approach: call HAL_GPIO_TogglePin or set a flag and wake via IRQ semaphore - implement below as example */
        /* Here just print debug (keep ISR short) */
        /* Do minimal work, main tasks will observe event via EXTI and driver states */
    }
}

/* Utility: push log message to queue (allocates payload copy) */
static int push_log_message(LogType_t type, const void *buf, uint16_t len)
{
    if (logQueue == NULL) return -1;
    LogMsg_t msg;
    msg.type = type;
    msg.len = len;
    msg.data = NULL;
    /* allocate memory for payload */
    if (len > 0) {
        msg.data = pvPortMalloc(len);
        if (!msg.data) return -2;
        memcpy(msg.data, buf, len);
    }
    /* set timestamp from RTC (seconds not implemented, use placeholder) */
    RTC_TimeTypeDef t; RTC_DateTypeDef d;
    HAL_RTC_GetTime(&hrtc, &t, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &d, RTC_FORMAT_BIN);
    /* simple encoded timestamp: HHMMSSDDMMYY */
    msg.timestamp = (t.Hours << 24) | (t.Minutes << 16) | (t.Seconds << 8) | (d.Date & 0xFF);

    if (osMessageQueuePut(logQueue, &msg, 0, 0) != osOK) {
        /* queue full - free allocated */
        if (msg.data) vPortFree(msg.data);
        return -3;
    }
    return 0;
}

/* -------------------- Task implementations -------------------- */

/* InitTask: lightweight init then exit (or suspend) */
static void InitTask(void *arg)
{
    (void)arg;
    printf("InitTask: starting drivers\n");

    /* Initialize drivers - ensure i2c_mutex and semaphores already created before calling these if drivers expect them */
    ADS1115_Init(&hi2c1, ADS1115_ADDRESS_GND, ADS_PGA_2_048, ADS_DR_250SPS);
    MPU6050_DMA_Init();
    MCP4725_Init(&hi2c1, 0x60); /* adjust addr if necessary */

    /* Link semaphores to drivers (driver headers must provide these) */
    ADS1115_SetSemaphore(sem_ads);
    MPU6050_SetSemaphore(sem_mpu);
    MCP4725_SetSemaphore(sem_mcp);
    LoRa_SetTxSemaphore(sem_lora_tx);

    /* Link i2c mutex */
    ADS1115_SetI2CMutex(i2c_mutex);
    MPU6050_SetI2CMutex(i2c_mutex);
    MCP4725_SetI2CMutex(i2c_mutex);

    /* Init LoRa and set frequency */
    if (LoRa_Init() == 0) {
        LoRa_SetFrequency(433000000);
        printf("InitTask: LoRa init OK\n");
    } else {
        printf("InitTask: LoRa init failed\n");
    }

    /* Optionally create a test packet here or signal other tasks */
    printf("InitTask: done, suspending self\n");
    vTaskSuspend(NULL); /* suspend init task */
}

/* AcquisitionTask: highest priority. Woken by EXTI (external comparator) via osThreadFlags or by a queued event.
   This implementation demonstrates: when woken, perform ADS1115 + MPU read, push to log queue, attempt LoRa TX.
*/
static void AcquisitionTask(void *arg)
{
    (void)arg;
    const TickType_t wait = pdMS_TO_TICKS(2000);

    for (;;) {
        /* Wait for notification/flag - for simplicity use osThreadFlagsWait,
           but if you plan to notify from ISR use osThreadFlagsSet(task_id, flags).
           We'll sleep and check a simple hardware flag - replace with real notify */
        /* In real deployment: use direct-to-task notify from ISR. Here: poll (simple) */
        osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever); // placeholder; ISR should call osThreadFlagsSet(acqTask, 0x1)
        printf("AcquisitionTask: triggered\n");

        /* Read ADS1115 (blocking wrapper using DMA) */
        int16_t raw;
        if (ADS1115_ReadRaw_DMA_Blocking(&raw, 500) == 0) {
            float volts = 0;
            ADS1115_ReadVoltage_DMA_Blocking(& (float){0}, 1); // optionally get volts
            printf("Acq: ADS raw=%d\n", raw);
        } else {
            printf("Acq: ADS1115 read failed\n");
        }

        /* Read IMU via DMA (start and wait for semaphore set by MPU callback) */
        if (osMutexAcquire(i2c_mutex, pdMS_TO_TICKS(200)) == osOK) {
            if (MPU6050_DMA_StartRead() == HAL_OK) {
                /* wait on sem_mpu (driver should give sem on RxCplt) */
                if (osSemaphoreAcquire(sem_mpu, wait) == osOK) {
                    MPU6050_Data_t d;
                    MPU6050_DMA_ProcessData(&d);
                    printf("Acq: IMU acc %.3f %.3f %.3f\n", d.acc_x_g, d.acc_y_g, d.acc_z_g);
                    /* pack a small packet to send via LoRa and to log */
                    uint8_t payload[32];
                    int n = snprintf((char*)payload, sizeof(payload), "T%u A:%.3f,%.3f,%.3f",
                                     (unsigned)HAL_GetTick(), d.acc_x_g, d.acc_y_g, d.acc_z_g);
                    /* send via LoRa (non-blocking internal, but our LoRa_Transmit_DMA blocks until tx_done) */
                    if (LoRa_Transmit_DMA(payload, (uint16_t)n, 2000) == 0) {
                        printf("Acq: LoRa TX OK\n");
                    } else {
                        printf("Acq: LoRa TX failed\n");
                    }
                    /* push to log queue */
                    push_log_message(LOGTYPE_ACQ, payload, (uint16_t)n);
                } else {
                    printf("Acq: MPU timeout\n");
                }
            } else {
                printf("Acq: MPU start read failed\n");
            }
            osMutexRelease(i2c_mutex);
        } else {
            printf("Acq: i2c_mutex busy\n");
        }

        /* After acquisition, wait some settling time or go back to sleep until next notify */
    }
}

/* IMUTask: separate consumer that may perform continuous IMU processing when acquisition is active */
static void IMUTask(void *arg)
{
    (void)arg;
    for (;;) {
        /* Example: periodic health-check read of IMU or use another notify from acquisition */
        osDelay(1000);
    }
}

/* CommTask: responsible for higher-level LoRa RX processing, and responding to gateway commands.
   When LoRa IRQ indicates packet ready, call LoRa_read_payload etc. */
static void CommTask(void *arg)
{
    (void)arg;
    uint8_t rxbuf[256];
    for (;;) {
        /* poll for RX ready */
        if (LoRa_receive_ready()) {
            int n = LoRa_read_payload(rxbuf, sizeof(rxbuf));
            if (n > 0) {
                rxbuf[n] = 0;
                printf("CommTask: received %d bytes: %s\n", n, rxbuf);
                /* handle commands: change threshold -> write MCP4725 via I2C; collect SD files -> transmit back etc. */
                /* For example: if command starts with "TH:" followed by value -> parse and set MCP4725 */
                if (strncmp((char*)rxbuf, "TH:", 3) == 0) {
                    int v = atoi((char*)&rxbuf[3]);
                    /* convert v into DAC raw (0..4095) depending on Vref 3.3V */
                    uint16_t dac = (uint16_t)((v/3.3f) * 4095.0f);
                    if (osMutexAcquire(i2c_mutex, pdMS_TO_TICKS(200)) == osOK) {
                        MCP4725_SetValue(dac); /* assume blocking write */
                        osMutexRelease(i2c_mutex);
                    }
                }
            }
        }
        osDelay(50);
    }
}

/* LoggingTask: sole task writing to SD card. Receives LogMsg_t from producers. */
static void LoggingTask(void *arg)
{
    (void)arg;
    LogMsg_t msg;
    FRESULT fres;
    for (;;) {
        if (osMessageQueueGet(logQueue, &msg, NULL, osWaitForever) == osOK) {
            /* prepare filename and folder using RTC */
            char timestr[64];
            get_rtc_time_string(timestr, sizeof(timestr));
            char fname[128];
            /* choose folder by type */
            const char *folder = (msg.type == LOGTYPE_ACQ) ? "ACQ" :
                                 (msg.type == LOGTYPE_IMU) ? "IMU" :
                                 (msg.type == LOGTYPE_HEALTH) ? "HLT" : "EVT";
            /* create folder and filename e.g., /ACQ/2025-10-31_12-00-00.log */
            snprintf(fname, sizeof(fname), "%s/%s.log", folder, timestr);

            /* open file for append */
            FIL f;
            /* create folder if necessary - f_mkdir */
            char folderpath[64];
            snprintf(folderpath, sizeof(folderpath), "%s", folder);
            f_mkdir(folderpath); // ignore error if exists

            char filepath[160];
            snprintf(filepath, sizeof(filepath), "%s/%08lu.log", folder, (unsigned long)HAL_GetTick());
            if (f_open(&f, filepath, FA_CREATE_ALWAYS | FA_WRITE) == FR_OK) {
                UINT bw;
                /* write timestamp + data */
                f_write(&f, timestr, strlen(timestr), &bw);
                f_write(&f, " ", 1, &bw);
                if (msg.len > 0 && msg.data) {
                    f_write(&f, msg.data, msg.len, &bw);
                }
                f_write(&f, "\r\n", 2, &bw);
                f_close(&f);
                printf("LoggingTask: wrote %s\n", filepath);
            } else {
                printf("LoggingTask: f_open failed for %s\n", filepath);
            }
            if (msg.data) vPortFree(msg.data);
        }
    }
}

/* HealthTask: periodic health monitoring and DHT11 reading */
static void HealthTask(void *arg)
{
    (void)arg;
    DHT_DataTypedef dht;
    for (;;) {
        /* read DHT */
        if (DWT_Delay_Init() == 0) {
            DHT_GetData(&dht);
            if (dht.valid) {
                char buf[64];
                int n = snprintf(buf, sizeof(buf), "T:%.1f H:%.1f", dht.Temperature, dht.Humidity);
                push_log_message(LOGTYPE_HEALTH, buf, (uint16_t)n);
                printf("Health: %s\n", buf);
            } else {
                printf("Health: DHT read failed\n");
            }
        }
        /* battery check, memory, etc. could be added here */
        osDelay(120000); /* every 2 minutes */
    }
}

/* TimerDaemonTask: housekeeping timers & BOLT features */
static void TimerDaemonTask(void *arg)
{
    (void)arg;
    for (;;) {
        /* Run periodic maintenance: rotate logs, check SD space, reschedule tasks, perform periodic calibration */
        osDelay(1000);
    }
}

/* -------------------- main() -------------------- */
int main(void)
{
    /* HAL init */
    HAL_Init();
    SystemClock_Config();

    /* init peripherals from CubeMX */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_I2C1_Init();
    MX_USART1_UART_Init();
    MX_SPI2_Init();
    MX_SDMMC1_SD_Init();
    MX_FATFS_Init();
    MX_RTC_Init();

    printf("System boot\n");

    /* RTOS kernel init */
    osKernelInitialize();

    /* create synchronization objects BEFORE tasks that will use them */
    /* create i2c mutex */
    i2c_mutex = osMutexNew(NULL);

    /* semaphores - binary (count 0) */
    sem_ads = osSemaphoreNew(1, 0, NULL);
    sem_mpu = osSemaphoreNew(1, 0, NULL);
    sem_mcp = osSemaphoreNew(1, 0, NULL);
    sem_lora_tx = osSemaphoreNew(1, 0, NULL);

    /* message queue for logging */
    logQueue = osMessageQueueNew(32, sizeof(LogMsg_t), NULL);

    /* create tasks */
    const osThreadAttr_t init_attr = { .name="InitTask", .priority=osPriorityHigh, .stack_size=2048 };
    osThreadNew(InitTask, NULL, &init_attr);

    const osThreadAttr_t acq_attr = { .name="AcqTask", .priority=osPriorityRealtime, .stack_size=4096 };
    osThreadNew(AcquisitionTask, NULL, &acq_attr);

    const osThreadAttr_t imu_attr = { .name="IMUTask", .priority=osPriorityHigh, .stack_size=2048 };
    osThreadNew(IMUTask, NULL, &imu_attr);

    const osThreadAttr_t comm_attr = { .name="CommTask", .priority=osPriorityAboveNormal, .stack_size=2048 };
    osThreadNew(CommTask, NULL, &comm_attr);

    const osThreadAttr_t log_attr = { .name="LogTask", .priority=osPriorityLow, .stack_size=4096 };
    osThreadNew(LoggingTask, NULL, &log_attr);

    const osThreadAttr_t health_attr = { .name="HealthTask", .priority=osPriorityLow, .stack_size=2048 };
    osThreadNew(HealthTask, NULL, &health_attr);

    const osThreadAttr_t timer_attr = { .name="TimerDaemon", .priority=osPriorityHigh, .stack_size=2048 };
    osThreadNew(TimerDaemonTask, NULL, &timer_attr);

    /* register semaphores & mutex to drivers */
    ADS1115_SetSemaphore(sem_ads);
    MPU6050_SetSemaphore(sem_mpu);
    MCP4725_SetSemaphore(sem_mcp);
    LoRa_SetTxSemaphore(sem_lora_tx);

    ADS1115_SetI2CMutex(i2c_mutex);
    MPU6050_SetI2CMutex(i2c_mutex);
    MCP4725_SetI2CMutex(i2c_mutex);

    /* start scheduler */
    osKernelStart();

    /* should never reach here */
    for (;;) {}
}
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
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
	  HAL_Delay(200);
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
	printf("Assert %s:%lu\n", file, (unsigned long)line);
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
