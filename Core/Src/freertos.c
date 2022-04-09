/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "printf.h"
#include "bmpxx80.h"
#include "bh1750.h"
#include "i2c.h"
#include "usart.h"
#include "dwt_stm32_delay.h"
#include "DHT11.h"
#include "iwdg.h"
#include "NRF24L0plus.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MEASURING_INTERVAL 	3000//ms
#define BLINKING_INTERVAL 	1000//ms



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
typedef enum
{
	bmp_temp,
	bmp_pressure,
	dht_temp,
	dht_humidity,
	bh_light
}id_enumerating;
#define SUM_OF_ID 31

typedef struct
{
	uint8_t data_ID;
	uint32_t value;
} Measured_t;

/*typedef struct
{
	uint8_t	bmp_temperature	:1;
	uint8_t	bmp_pressure	:1;
	uint8_t	dht_temperature	:1;
	uint8_t	dht_humidity	:1;
	uint8_t	bh_light		:1;
	uint8_t	not_used01		:3;
} dataReady_status_t;
*/


/* USER CODE END Variables */
/* Definitions for BMP_Task */
osThreadId_t BMP_TaskHandle;
const osThreadAttr_t BMP_Task_attributes = {
  .name = "BMP_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for BH_Task */
osThreadId_t BH_TaskHandle;
const osThreadAttr_t BH_Task_attributes = {
  .name = "BH_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LedRed_task */
osThreadId_t LedRed_taskHandle;
const osThreadAttr_t LedRed_task_attributes = {
  .name = "LedRed_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for DHT11_Task */
osThreadId_t DHT11_TaskHandle;
const osThreadAttr_t DHT11_Task_attributes = {
  .name = "DHT11_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SenderTask */
osThreadId_t SenderTaskHandle;
const osThreadAttr_t SenderTask_attributes = {
  .name = "SenderTask",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for IWDG_Task */
osThreadId_t IWDG_TaskHandle;
const osThreadAttr_t IWDG_Task_attributes = {
  .name = "IWDG_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for NRF24Task */
osThreadId_t NRF24TaskHandle;
const osThreadAttr_t NRF24Task_attributes = {
  .name = "NRF24Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for QueueToSend */
osMessageQueueId_t QueueToSendHandle;
const osMessageQueueAttr_t QueueToSend_attributes = {
  .name = "QueueToSend"
};
/* Definitions for MutexPrint */
osMutexId_t MutexPrintHandle;
const osMutexAttr_t MutexPrint_attributes = {
  .name = "MutexPrint"
};
/* Definitions for MutexI2C1 */
osMutexId_t MutexI2C1Handle;
const osMutexAttr_t MutexI2C1_attributes = {
  .name = "MutexI2C1"
};
/* Definitions for MutexSPI1 */
osMutexId_t MutexSPI1Handle;
const osMutexAttr_t MutexSPI1_attributes = {
  .name = "MutexSPI1"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartBMP_Task(void *argument);
void StartBH_Task(void *argument);
void StartLedRed_task(void *argument);
void StartDHT11Task(void *argument);
void StartSenderTask(void *argument);
void StartIWDGTask(void *argument);
void StartNRF24Task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
	while(1)
			{;}
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void)
{
   /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
	while(1)
		{;}
}
/* USER CODE END 5 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of MutexPrint */
  MutexPrintHandle = osMutexNew(&MutexPrint_attributes);

  /* creation of MutexI2C1 */
  MutexI2C1Handle = osMutexNew(&MutexI2C1_attributes);

  /* creation of MutexSPI1 */
  MutexSPI1Handle = osMutexNew(&MutexSPI1_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of QueueToSend */
  QueueToSendHandle = osMessageQueueNew (5, sizeof(Measured_t), &QueueToSend_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of BMP_Task */
  BMP_TaskHandle = osThreadNew(StartBMP_Task, NULL, &BMP_Task_attributes);

  /* creation of BH_Task */
  BH_TaskHandle = osThreadNew(StartBH_Task, NULL, &BH_Task_attributes);

  /* creation of LedRed_task */
  LedRed_taskHandle = osThreadNew(StartLedRed_task, NULL, &LedRed_task_attributes);

  /* creation of DHT11_Task */
  DHT11_TaskHandle = osThreadNew(StartDHT11Task, NULL, &DHT11_Task_attributes);

  /* creation of SenderTask */
  SenderTaskHandle = osThreadNew(StartSenderTask, NULL, &SenderTask_attributes);

  /* creation of IWDG_Task */
  IWDG_TaskHandle = osThreadNew(StartIWDGTask, NULL, &IWDG_Task_attributes);

  /* creation of NRF24Task */
  NRF24TaskHandle = osThreadNew(StartNRF24Task, NULL, &NRF24Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */

  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartBMP_Task */
/**
  * @brief  Function implementing the BMP_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartBMP_Task */
void StartBMP_Task(void *argument)
{
  /* USER CODE BEGIN StartBMP_Task */
	Measured_t bmp;
	osMutexAcquire(MutexI2C1Handle, osWaitForever);
	BMP280_Init(&hi2c1, BMP280_TEMPERATURE_16BIT, BMP280_STANDARD, BMP280_FORCEDMODE);
	osMutexRelease(MutexI2C1Handle);

  /* Infinite loop */
  for(;;)
  {
	  osMutexAcquire(MutexI2C1Handle, osWaitForever);
	  bmp.data_ID = bmp_temp;
	  bmp.value = BMP280_ReadTemperature();
	  osMutexRelease(MutexI2C1Handle);
	  osMessageQueuePut(QueueToSendHandle, &bmp, 0, osWaitForever);

	  osMutexAcquire(MutexI2C1Handle, osWaitForever);
	  bmp.data_ID = bmp_pressure;
	  bmp.value = BMP280_ReadPressure();
	  osMutexRelease(MutexI2C1Handle);
	  osMessageQueuePut(QueueToSendHandle, &bmp, 0, osWaitForever);

	  osDelay(MEASURING_INTERVAL);
  }
  /* USER CODE END StartBMP_Task */
}

/* USER CODE BEGIN Header_StartBH_Task */
/**
* @brief Function implementing the BH_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBH_Task */
void StartBH_Task(void *argument)
{
  /* USER CODE BEGIN StartBH_Task */
	Measured_t measuring;
	measuring.data_ID = bh_light;

	osMutexAcquire(MutexI2C1Handle, osWaitForever);
	if(BH1750_OK == BH1750_Init(&hi2c1))
	{
		printf("BH init OK!\n\r");
	}else
	{
		printf("BH init error!\n\r");
	}

	BH1750_SetMode(CONTINUOUS_HIGH_RES_MODE);
	osMutexRelease(MutexI2C1Handle);

  /* Infinite loop */
  for(;;)
  {
	  osMutexAcquire(MutexI2C1Handle, osWaitForever);
	  if(BH1750_OK == BH1750_ReadLight(&measuring.value))
	  {
		  osMessageQueuePut(QueueToSendHandle, &measuring, 0, osWaitForever);
		  //printf("Light: %d lx\n\r",BH1750_lx_value);
	  }else
	  {
		  osMutexAcquire(MutexPrintHandle, osWaitForever);
		  printf("BH error!\n\r");
		  osMutexRelease(MutexPrintHandle);
	  }
	  osMutexRelease(MutexI2C1Handle);

	  osDelay(MEASURING_INTERVAL);
  }
  /* USER CODE END StartBH_Task */
}

/* USER CODE BEGIN Header_StartLedRed_task */
/**
* @brief Function implementing the LedRed_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLedRed_task */
void StartLedRed_task(void *argument)
{
  /* USER CODE BEGIN StartLedRed_task */
  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_TogglePin(Led_Red_GPIO_Port,Led_Red_Pin);
	  osDelay(BLINKING_INTERVAL);
  }
  /* USER CODE END StartLedRed_task */
}

/* USER CODE BEGIN Header_StartDHT11Task */
/**
* @brief Function implementing the DHT11_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDHT11Task */
void StartDHT11Task(void *argument)
{
  /* USER CODE BEGIN StartDHT11Task */
	//uint32_t DHT_temperature,DHT_humidity;
	DWT_Delay_Init();
	uint8_t temperature;
	uint8_t humidity;
	Measured_t dht;
  /* Infinite loop */
  for(;;)
  {

	  DHT11_Start();
	  taskENTER_CRITICAL();
	  DHT11_TempAndHumidity((uint16_t*)&temperature, (uint16_t*)&humidity);
	  taskEXIT_CRITICAL();

	  dht.data_ID = dht_temp;
	  dht.value = temperature;
	  osMessageQueuePut(QueueToSendHandle, &dht, 0, osWaitForever);

	  dht.data_ID = dht_humidity;
	  dht.value = humidity;
	  osMessageQueuePut(QueueToSendHandle, (Measured_t*) &dht,  0, osWaitForever);

	  osDelay(MEASURING_INTERVAL);
  }
  /* USER CODE END StartDHT11Task */
}

/* USER CODE BEGIN Header_StartSenderTask */
/**
* @brief Function implementing the SenderTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSenderTask */
void StartSenderTask(void *argument)
{
  /* USER CODE BEGIN StartSenderTask */
	Measured_t received;

	int8_t temperature_BMP;
	uint32_t pressure_BMP;
	uint32_t pressure_afterComa;
	int8_t temperature_DHT;
	uint8_t humidity_DHT;
	uint32_t light_BH;

	uint8_t dataReady_status = 0;


  /* Infinite loop */
  for(;;)
  {
	osMessageQueueGet(QueueToSendHandle, &received, 0, osWaitForever);

		switch (received.data_ID)
		{
		case bmp_temp:
			temperature_BMP = received.value;

			dataReady_status |=(1<<bmp_temp);
			break;

		case bmp_pressure:
			pressure_afterComa = received.value;	//to use less variables
			pressure_BMP = pressure_afterComa / 100;
			pressure_afterComa = pressure_afterComa %100;

			dataReady_status |=(1<<bmp_pressure);
			break;

		case dht_humidity:
			humidity_DHT = received.value;

			dataReady_status |=(1<<dht_humidity);
			break;

		case dht_temp:
			temperature_DHT = received.value;

			dataReady_status |=(1<<dht_temp);
			break;

		case bh_light:
			light_BH = received.value;

			dataReady_status |=(1<<bh_light);
			break;

		default:
			printf("\nError in switch case!!\n\r");
		}

		if (dataReady_status == SUM_OF_ID)
		{

			if (GPIO_PIN_SET == HAL_GPIO_ReadPin(RAIN_GPIO_Port, RAIN_Pin))
			{
			osMutexAcquire(MutexPrintHandle,osWaitForever);
			printf("{\"temp\":%d, \"hum\":%d, \"press\":%d.%d,"
					"\"light\":%d, \"batt\":4.58, \"rain\":1}\n\r",
					temperature_BMP, humidity_DHT,
					pressure_BMP, pressure_afterComa,
					light_BH);
			osMutexRelease(MutexPrintHandle);
			} else
			{
			osMutexAcquire(MutexPrintHandle,osWaitForever);
			printf("{\"temp\":%d, \"hum\":%d, \"press\":%d.%d,"
					"\"light\":%d, \"batt\":4.58, \"rain\":0}\n\r",
					temperature_BMP, humidity_DHT,
					pressure_BMP, pressure_afterComa,
					light_BH);
			osMutexRelease(MutexPrintHandle);
			}
			dataReady_status = 0;
		}
  }
  /* USER CODE END StartSenderTask */
}

/* USER CODE BEGIN Header_StartIWDGTask */
/**
* @brief Function implementing the IWDG_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartIWDGTask */
void StartIWDGTask(void *argument)
{
  /* USER CODE BEGIN StartIWDGTask */
  /* Infinite loop */
  for(;;)
  {
	  osDelay(1800);
	  HAL_IWDG_Refresh(&hiwdg);
  }
  /* USER CODE END StartIWDGTask */
}

/* USER CODE BEGIN Header_StartNRF24Task */
/**
* @brief Function implementing the NRF24Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartNRF24Task */
void StartNRF24Task(void *argument)
{
  /* USER CODE BEGIN StartNRF24Task */
	nrf24_CE_enable();
	uint8_t pipes[2];

	nrf24_WriteReg(CONFIG,PWR_UP);
	nrf24_WriteReg(EN_RXADDR,PIPE4_EN);

  /* Infinite loop */
  for(;;)
  {
	  if(HAL_OK != nrf24_ReadReg(CONFIG, pipes))
	  {
		  printf("siuuuuuuuuuuu\n\r");
	  } else
	  {
		  osMutexAcquire(MutexPrintHandle, osWaitForever);
		  printf("NRF24 enabled pipes1: %d\n\r",pipes[0]);
		  printf("NRF24 enabled pipes2: %d\n\r",pipes[1]);
		  osMutexRelease(MutexPrintHandle);
	  }

	  osDelay(1000);
  }
  /* USER CODE END StartNRF24Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void _putchar(char character)
{
	HAL_UART_Transmit(&huart2, (uint8_t *) &character, 1, 1000);
  // send char to console etc.
}

/* USER CODE END Application */

