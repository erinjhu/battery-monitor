/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticQueue_t osStaticMessageQDef_t;
typedef StaticSemaphore_t osStaticMutexDef_t;
typedef StaticSemaphore_t osStaticSemaphoreDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

typedef enum
{
  MSG_TYPE_VOLTAGE,
  MSG_TYPE_BUTTON,
  MSG_TYPE_ERROR
} MsgType_t;

typedef enum
{
  ERR_CODE_SUCCESS = 0,
  ERR_CODE_UNKNOWN = 1,
  ERR_CODE_INVALID_ARG = 2,
  ERR_CODE_QUEUE_FULL = 3,
  ERR_CODE_MUTEX_TIMEOUT = 4,
  ERR_CODE_BUFF_TOO_SMALL = 5,
  ERR_CODE_LOG_MSG_SILENCED = 6,
  ERR_CODE_INVALID_STATE = 7,
  ERR_CODE_UNSUPPORTED_EVENT = 8,
  ERR_CODE_BUFF_OVERFLOW = 9,
  ERR_CODE_SEMAPHORE_TIMEOUT = 10,
  ERR_CODE_SEMAPHORE_FULL = 11,
  ERR_CODE_QUEUE_EMPTY = 12,
  ERR_CODE_NOT_MUTEX_OWNER = 13,
  ERR_CODE_PERSISTENT_CORRUPTED = 14,
  ERR_CODE_FAILED_UNPACK = 15,
  ERR_CODE_FAILED_PACK = 16,
  ERR_CODE_INVALID_STATE_TRANSITION = 17,
  ERR_CODE_FREERTOS_ASSERT_FAIL = 18,
  ERR_CODE_FAILED_STACK_CANARY = 19,
  ERR_CODE_ADC_TIMEOUT = 20,
} ErrorCode_t;

typedef struct 
{
  MsgType_t type;  
  union {
    float value;  
    uint8_t str[8];
    ErrorCode_t errCode;
  };
  
} UARTMsg_t;


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */



/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for xTaskSensor */
osThreadId_t xTaskSensorHandle;
uint32_t xTaskSensorBuffer[ 128 ];
osStaticThreadDef_t xTaskSensorControlBlock;
const osThreadAttr_t xTaskSensor_attributes = {
  .name = "xTaskSensor",
  .cb_mem = &xTaskSensorControlBlock,
  .cb_size = sizeof(xTaskSensorControlBlock),
  .stack_mem = &xTaskSensorBuffer[0],
  .stack_size = sizeof(xTaskSensorBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for xTaskAlarm */
osThreadId_t xTaskAlarmHandle;
uint32_t xTaskAlarmBuffer[ 128 ];
osStaticThreadDef_t xTaskAlarmControlBlock;
const osThreadAttr_t xTaskAlarm_attributes = {
  .name = "xTaskAlarm",
  .cb_mem = &xTaskAlarmControlBlock,
  .cb_size = sizeof(xTaskAlarmControlBlock),
  .stack_mem = &xTaskAlarmBuffer[0],
  .stack_size = sizeof(xTaskAlarmBuffer),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for xTaskUART */
osThreadId_t xTaskUARTHandle;
uint32_t xTaskUARTBuffer[ 128 ];
osStaticThreadDef_t xTaskUARTControlBlock;
const osThreadAttr_t xTaskUART_attributes = {
  .name = "xTaskUART",
  .cb_mem = &xTaskUARTControlBlock,
  .cb_size = sizeof(xTaskUARTControlBlock),
  .stack_mem = &xTaskUARTBuffer[0],
  .stack_size = sizeof(xTaskUARTBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for xUARTQueue */
osMessageQueueId_t xUARTQueueHandle;
uint8_t xUARTQueueBuffer[ 16 * sizeof( UARTMsg_t ) ];
osStaticMessageQDef_t xUARTQueueControlBlock;
const osMessageQueueAttr_t xUARTQueue_attributes = {
  .name = "xUARTQueue",
  .cb_mem = &xUARTQueueControlBlock,
  .cb_size = sizeof(xUARTQueueControlBlock),
  .mq_mem = &xUARTQueueBuffer,
  .mq_size = sizeof(xUARTQueueBuffer)
};
/* Definitions for xMutex */
osMutexId_t xMutexHandle;
osStaticMutexDef_t xMutexControlBlock;
const osMutexAttr_t xMutex_attributes = {
  .name = "xMutex",
  .cb_mem = &xMutexControlBlock,
  .cb_size = sizeof(xMutexControlBlock),
};
/* Definitions for xBinSem */
osSemaphoreId_t xBinSemHandle;
osStaticSemaphoreDef_t xBinSemControlBlock;
const osSemaphoreAttr_t xBinSem_attributes = {
  .name = "xBinSem",
  .cb_mem = &xBinSemControlBlock,
  .cb_size = sizeof(xBinSemControlBlock),
};
/* USER CODE BEGIN PV */
volatile float fBatteryVoltage;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);
void vTaskSensor(void *argument);
void vTaskAlarm(void *argument);
void vTaskUART(void *argument);

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
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  uint16_t readVal;
  char tx_buffer[20];
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of xMutex */
  xMutexHandle = osMutexNew(&xMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of xBinSem */
  xBinSemHandle = osSemaphoreNew(1, 0, &xBinSem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of xUARTQueue */
  xUARTQueueHandle = osMessageQueueNew (16, sizeof(UARTMsg_t), &xUARTQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of xTaskSensor */
  xTaskSensorHandle = osThreadNew(vTaskSensor, NULL, &xTaskSensor_attributes);

  /* creation of xTaskAlarm */
  xTaskAlarmHandle = osThreadNew(vTaskAlarm, NULL, &xTaskAlarm_attributes);

  /* creation of xTaskUART */
  xTaskUARTHandle = osThreadNew(vTaskUART, NULL, &xTaskUART_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // code to verify that adc works

  //   printf("test print\r\n");
	//  HAL_ADC_Start(&hadc1);
	//  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	//  readVal = HAL_ADC_GetValue(&hadc1);

	//  sprintf(tx_buffer, "%hu\r\n", readVal);
	//  HAL_UART_Transmit(&huart2, (uint8_t*)tx_buffer, strlen(tx_buffer), HAL_MAX_DELAY);
	//  HAL_Delay(10);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA6 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_vTaskSensor */
/**
* @brief Function implementing the xTaskSensor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vTaskSensor */
void vTaskSensor(void *argument)
{
  /* USER CODE BEGIN vTaskSensor */
  HAL_UART_Transmit(&huart2, (uint8_t*)"Sensor Task Started\r\n", 21, 100);
  /* Infinite loop */
  osStatus_t errCode;
  for(;;)
  {
    HAL_ADC_Start(&hadc1);


    HAL_StatusTypeDef halStatus = HAL_ADC_PollForConversion(&hadc1,100);
    if (halStatus == HAL_TIMEOUT)
    {
      UARTMsg_t errMsg = {.type = MSG_TYPE_ERROR, .errCode = ERR_CODE_ADC_TIMEOUT};
      osMessageQueuePut(xUARTQueueHandle, &errMsg, 0U, 0); // timeout = 0 --> return immediately
      continue; // skip the rest of this loop iteration
    }
    else if (halStatus != HAL_OK) 
    {
      UARTMsg_t errMsg = {.type = MSG_TYPE_ERROR, .errCode = ERR_CODE_UNKNOWN};
      osMessageQueuePut(xUARTQueueHandle, &errMsg, 0U, 0);
      return;
    }

    errCode = osMutexAcquire(xMutexHandle,100);
    if (errCode == osErrorTimeout)
    {
      UARTMsg_t errMsg = {.type = MSG_TYPE_ERROR, .errCode = ERR_CODE_MUTEX_TIMEOUT};
      osMessageQueuePut(xUARTQueueHandle, &errMsg, 0U, 0);
      continue; 
    }
    else if (errCode != osOK) 
    {
      UARTMsg_t errMsg = {.type = MSG_TYPE_ERROR, .errCode = ERR_CODE_UNKNOWN};
      osMessageQueuePut(xUARTQueueHandle, &errMsg, 0U, 0);
      return;
    }

    uint32_t adcRaw = HAL_ADC_GetValue(&hadc1);
    fBatteryVoltage = (adcRaw * 3.3f) / 4095.0f;

    UARTMsg_t batteryMessage = {.type = MSG_TYPE_VOLTAGE, .value = fBatteryVoltage};
    osMessageQueuePut(xUARTQueueHandle, &batteryMessage, 0U, 10);
    osSemaphoreRelease(xBinSemHandle);
    osMutexRelease(xMutexHandle);
    osDelay(100);
  }
  /* USER CODE END vTaskSensor */
}

/* USER CODE BEGIN Header_vTaskAlarm */
/**
* @brief Function implementing the xTaskAlarm thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vTaskAlarm */
void vTaskAlarm(void *argument)
{
  /* USER CODE BEGIN vTaskAlarm */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END vTaskAlarm */
}

/* USER CODE BEGIN Header_vTaskUART */
/**
* @brief Function implementing the xTaskUART thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vTaskUART */
void vTaskUART(void *argument)
{
  /* USER CODE BEGIN vTaskUART */
  HAL_UART_Transmit(&huart2, (uint8_t*)"UART Task Started\r\n", 19, 100);
  UARTMsg_t msg;
  /* Infinite loop */
  for(;;)
  {
    if (osMessageQueueGet(xUARTQueueHandle,&msg,NULL,osWaitForever) == osOK)
    // use osWaitForever to block the task until a msg arrives in the queue
    {
      char buffer[64];
      if (msg.type == MSG_TYPE_VOLTAGE)
      {
        // snprintf(buffer, sizeof(buffer), "Voltage: %u.%02u V\r\n", msg.value);
        snprintf(buffer, sizeof(buffer), "Raw: %u\r\n", (unsigned int)(msg.value * 1000));
        HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 100);
      }
      else if (msg.type == MSG_TYPE_ERROR)
      {
        snprintf(buffer, sizeof(buffer), "Error code: %d \r\n", msg.errCode);
        HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 100);
      }
      else if (msg.type == MSG_TYPE_BUTTON)
      {
        snprintf(buffer, sizeof(buffer), "Button pressed");
        HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 100);
      }
    }
  }
  /* USER CODE END vTaskUART */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
