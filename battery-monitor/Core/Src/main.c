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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_tasks.h"
#include "bsp.h"
#include "globals.h"
#include "gpio.h"
#include "interrupts.h"
#include "messages.h"
#include "env_mgr.h"
#include "voltage_mgr.h"
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




/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

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
// osThreadId_t xTaskAlarmHandle;
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
osStaticMutexDef_t xMutexControlBlock;
const osMutexAttr_t xMutex_attributes = {
  .name = "xMutex",
  .cb_mem = &xMutexControlBlock,
  .cb_size = sizeof(xMutexControlBlock),
};
/* Definitions for xBinSem */
osStaticSemaphoreDef_t xBinSemControlBlock;
const osSemaphoreAttr_t xBinSem_attributes = {
  .name = "xBinSem",
  .cb_mem = &xBinSemControlBlock,
  .cb_size = sizeof(xBinSemControlBlock),
};
/* USER CODE BEGIN PV */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void); 


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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  BMP180_Init();
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
  //xBinSemHandle = osSemaphoreNew(1, 0, &xBinSem_attributes);

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
  xTaskSensorHandle = osThreadNew(vTaskVoltageMgr, NULL, &xTaskSensor_attributes);

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
  //uint16_t readVal;
  //char tx_buffer[20];
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









/* USER CODE BEGIN 4 */

/* USER CODE END 4 */







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
