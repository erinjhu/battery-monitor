#include "app_tasks.h"
#include "main.h"
#include "cmsis_os.h"
#include "globals.h"
#include "gpio.h"
#include "messages.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h" 







osThreadId_t xTaskUARTHandle;
uint32_t xTaskUARTBuffer[ 128 ];
StaticTask_t xTaskUARTControlBlock;
const osThreadAttr_t xTaskUART_attributes = {
  .name = "xTaskUART",
  .cb_mem = &xTaskUARTControlBlock,
  .cb_size = sizeof(xTaskUARTControlBlock),
  .stack_mem = &xTaskUARTBuffer[0],
  .stack_size = sizeof(xTaskUARTBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};

void vTaskUART(void *argument)
{
  /* USER CODE BEGIN vTaskUART */
  
  HAL_StatusTypeDef halErrCode;
  osStatus_t cmsisErrCode;
  RETURN_IF_ERROR_CODE_HAL(HAL_UART_Transmit(&huart2, (uint8_t*)"UART Task Started\r\n", 19, 100), &healthFlags.uart);
  UARTMsg_t msg;
  /* Infinite loop */
  for(;;)
  {
    RETURN_IF_ERROR_CODE_CMSIS(osMessageQueueGet(xUARTQueueHandle, &msg, NULL, osWaitForever), &healthFlags.uart);
    // use osWaitForever to block the task until a msg arrives in the queue  
    char buffer[64];
    if (msg.type == MSG_TYPE_VOLTAGE)
    {
      // snprintf(buffer, sizeof(buffer), "Voltage: %u.%02u V\r\n", msg.value);
      snprintf(buffer, sizeof(buffer), "Raw: %u\r\n", (unsigned int)(msg.value * 1000));
      RETURN_IF_ERROR_CODE_HAL(HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 100), &healthFlags.uart);
    }
    else if (msg.type == MSG_TYPE_ERROR)
    {
      snprintf(buffer, sizeof(buffer), "Error code: %d \r\n", msg.errCode);
      RETURN_IF_ERROR_CODE_HAL(HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 100), &healthFlags.uart);
    }
    else if (msg.type == MSG_TYPE_BUTTON)
    {
      snprintf(buffer, sizeof(buffer), "Button pressed");
      RETURN_IF_ERROR_CODE_HAL(HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 100), &healthFlags.uart);
    }
    healthFlags.uart = HEALTH_OK;
  }
  /* USER CODE END vTaskUART */
}

osThreadId_t xTaskAlarmHandle;
uint32_t xTaskAlarmBuffer[ 128 ];
StaticTask_t xTaskAlarmControlBlock;
const osThreadAttr_t xTaskAlarm_attributes = {
  .name = "xTaskAlarm",
  .cb_mem = &xTaskAlarmControlBlock,
  .cb_size = sizeof(xTaskAlarmControlBlock),
  .stack_mem = &xTaskAlarmBuffer[0],
  .stack_size = sizeof(xTaskAlarmBuffer),
  .priority = (osPriority_t) osPriorityHigh,
};

void vTaskAlarm(void *argument)
{
  /* USER CODE BEGIN vTaskAlarm */
  HAL_StatusTypeDef halErrCode;
  osStatus_t cmsisErrCode;
  RETURN_IF_ERROR_CODE_HAL(HAL_UART_Transmit(&huart2, (uint8_t*)"Alarm Task Started\r\n", 21, 100), &healthFlags.alarm);
  
  /* Infinite loop */
  for(;;)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Wait for notification
    // RETURN_IF_ERROR_CODE(osSemaphoreAcquire(xBinSemHandle, HAL_MAX_DELAY));
    RETURN_IF_ERROR_CODE_CMSIS(osMutexAcquire(xMutexHandle,100), &healthFlags.alarm);
    float batteryVoltage = fBatteryVoltage;
    RETURN_IF_ERROR_CODE_CMSIS(osMutexRelease(xMutexHandle), &healthFlags.alarm);
    if (batteryVoltage > fThresholdVoltage)
    {
      RETURN_IF_ERROR_CODE_HAL(HAL_UART_Transmit(&huart2, (uint8_t*)"Turn on green LED\r\n", 30, 100), &healthFlags.alarm);
      HAL_GPIO_WritePin(GPIOA, GPIO_GREEN_LED, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOA, GPIO_RED_LED, GPIO_PIN_RESET);
    } 
    else
    {
      RETURN_IF_ERROR_CODE_HAL(HAL_UART_Transmit(&huart2, (uint8_t*)"Turn on red LED\r\n", 30, 100), &healthFlags.alarm);
      HAL_GPIO_WritePin(GPIOA, GPIO_RED_LED, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOA, GPIO_GREEN_LED, GPIO_PIN_RESET);
    }
    healthFlags.alarm = HEALTH_OK;
  }
  /* USER CODE END vTaskAlarm */
}


osThreadId_t xTaskWatchdogHandle;
uint32_t xTaskWatchdogBuffer[ 128 ];
StaticTask_t xTaskWatchdogControlBlock;
const osThreadAttr_t xTaskWatchdog_attributes = {
  .name = "xTaskWatchdog",
  .cb_mem = &xTaskWatchdogControlBlock,
  .cb_size = sizeof(xTaskWatchdogControlBlock),
  .stack_mem = &xTaskWatchdogBuffer[0],
  .stack_size = sizeof(xTaskWatchdogBuffer),
  .priority = (osPriority_t) osPriorityHigh,
};

void vTaskWatchdog(void *argument)
{
  /* USER CODE BEGIN vTaskUART */
  HAL_StatusTypeDef halErrCode;
  MsgType_t type;
  RETURN_IF_ERROR_CODE_HAL(HAL_UART_Transmit(&huart2, (uint8_t*)"Watchdog Task\r\n", 19, 100), &healthFlags.watchdog);
  UARTMsg_t msg;
  /* Infinite loop */
  for(;;)
  {
    if (healthFlags.uart == HEALTH_OK && healthFlags.alarm == HEALTH_OK && healthFlags.bmp180 == HEALTH_OK && healthFlags.button == HEALTH_OK && healthFlags.env_mgr == HEALTH_OK && healthFlags.voltage_mgr == HEALTH_OK)
    {
      LOG_FROM_TASK(ERR_CODE_WATCHDOG_HEALTHY, MSG_TYPE_HEALTH, "watchdog healthy");
      RETURN_IF_ERROR_CODE_HAL(HAL_IWDG_Refresh(&hiwdg), &healthFlags.watchdog);
    }
    osDelay(50);
  }
}