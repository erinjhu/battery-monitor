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
#include "tim.h"




#define MAX_VOLTAGE 3300
#define PERIOD 99


osThreadId_t xTaskUARTHandle;
uint32_t xTaskUARTBuffer[ 512 ];
StaticTask_t xTaskUARTControlBlock;
const osThreadAttr_t xTaskUART_attributes = {
  .name = "xTaskUART",
  .cb_mem = &xTaskUARTControlBlock,
  .cb_size = sizeof(xTaskUARTControlBlock),
  .stack_mem = &xTaskUARTBuffer[0],
  .stack_size = sizeof(xTaskUARTBuffer),
  .priority = (osPriority_t) osPriorityHigh,
};

void vTaskUART(void *argument)
{
  /* USER CODE BEGIN vTaskUART */
  static int lastError = -1;
  HAL_StatusTypeDef halErrCode;
  osStatus_t cmsisErrCode;
  // RETURN_IF_ERROR_CODE_HAL(HAL_UART_Transmit(&huart2, (uint8_t*)"UART Task Started\r\n", 19, 100), &healthFlags.uart);
  // const char *message = "UART Task Started\r\n";
  // RETURN_IF_ERROR_CODE_HAL(HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), HAL_MAX_DELAY), &healthFlags.uart);
  UARTMsg_t msg;
  /* Infinite loop */
  for(;;)
  {
    RETURN_IF_ERROR_CODE_CMSIS(osMessageQueueGet(xUARTQueueHandle, &msg, NULL, osWaitForever), &healthFlags.uart);
    // use osWaitForever to block the task until a msg arrives in the queue  
    char buffer[256];
    if (msg.type == MSG_TYPE_VOLTAGE)
    {
      // snprintf(buffer, sizeof(buffer), "Voltage: %u.%02u V\r\n", msg.value);
      snprintf(buffer, sizeof(buffer), "[MSG_TYPE_VOLTAGE] Raw: %u\r\n", (unsigned int)(msg.value * 1000));
      RETURN_IF_ERROR_CODE_HAL(HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 100), &healthFlags.uart);
    }
    else if (msg.type == MSG_TYPE_ERROR && msg.errCode != lastError)
    {
      lastError = msg.errCode;
      // snprintf(buffer, sizeof(buffer), "Raw: %u\r\n", (unsigned int)(msg.value * 1000));
      // snprintf(buffer, sizeof(buffer),
      // "Error code: %d\r\nFile: %s\r\nFunc: %s\r\nLine: %d\r\nMsg: %s\r\n",
      // msg.errCode,
      // msg.file ? msg.file : "?",   // handle NULL pointers
      // msg.func ? msg.func : "?",
      // msg.line,
      // msg.msg);
      // RETURN_IF_ERROR_CODE_HAL(HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 100), &healthFlags.uart);
    }
    else if (msg.type == MSG_TYPE_BUTTON)
    {
      snprintf(buffer, sizeof(buffer), "[MSG_TYPE_BUTTON] Button pressed\r\n");
      RETURN_IF_ERROR_CODE_HAL(HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 100), &healthFlags.uart);
    }
    else if (msg.type == MSG_TYPE_HEALTH)
    {
      snprintf(buffer, sizeof(buffer), "[MSG_TYPE_HEALTH] %s \r\n", msg.msg);
      RETURN_IF_ERROR_CODE_HAL(HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 100), &healthFlags.uart);
    }
    healthFlags.uart = HEALTH_OK;
  }
  /* USER CODE END vTaskUART */
}

osThreadId_t xTaskAlarmHandle;
uint32_t xTaskAlarmBuffer[ 256 ];
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
  const char *msg = "Alarm Task Started\r\n";
  // RETURN_IF_ERROR_CODE_HAL(HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY), &healthFlags.uart);
  LOG_FROM_TASK(ERR_CODE_SUCCESS, MSG_TYPE_HEALTH, "Alarm Task Started");
  /* Infinite loop */
  for(;;)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Wait for notification
    // RETURN_IF_ERROR_CODE(osSemaphoreAcquire(xBinSemHandle, HAL_MAX_DELAY));
    RETURN_IF_ERROR_CODE_CMSIS(osMutexAcquire(xMutexHandle,100), &healthFlags.alarm);
    float batteryVoltage = fBatteryVoltage;
    RETURN_IF_ERROR_CODE_CMSIS(osMutexRelease(xMutexHandle), &healthFlags.alarm);
    int compareValue = (int)(batteryVoltage / (float)MAX_VOLTAGE) * PERIOD;
   
    if (batteryVoltage > fThresholdVoltage)
    {
      LOG_FROM_TASK(ERR_CODE_SUCCESS, MSG_TYPE_HEALTH, "Green LED ON");
      // RETURN_IF_ERROR_CODE_HAL(HAL_UART_Transmit(&huart2, (uint8_t*)"Turn on green LED\r\n", 30, 100), &healthFlags.alarm);
      HAL_GPIO_WritePin(GPIOA, GPIO_GREEN_LED, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOA, GPIO_RED_LED, GPIO_PIN_RESET);
      
    } 
    else
    {
      LOG_FROM_TASK(ERR_CODE_SUCCESS, MSG_TYPE_HEALTH, "Red LED ON");
      // HAL_GPIO_WritePin(GPIOA, GPIO_RED_LED, GPIO_PIN_SET);
      // HAL_GPIO_WritePin(GPIOA, GPIO_GREEN_LED, GPIO_PIN_RESET);
      // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOA, GPIO_GREEN_LED, GPIO_PIN_RESET);
      HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 105);
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
  .priority = (osPriority_t) osPriorityNormal,
};

void vTaskWatchdog(void *argument)
{
  /* USER CODE BEGIN vTaskUART */
  HAL_StatusTypeDef halErrCode;
  MsgType_t type;
  LOG_FROM_TASK(ERR_CODE_SUCCESS, MSG_TYPE_HEALTH, "Watchdog Task Started");
  UARTMsg_t msg;
  /* Infinite loop */
  for(;;)
  {
    // logAllHealthFlags();
    if (healthFlags.uart == HEALTH_OK && healthFlags.alarm == HEALTH_OK && healthFlags.bmp180 == HEALTH_OK && healthFlags.button == HEALTH_OK && healthFlags.env_mgr == HEALTH_OK && healthFlags.voltage_mgr == HEALTH_OK)
    {
      LOG_FROM_TASK(ERR_CODE_WATCHDOG_HEALTHY, MSG_TYPE_HEALTH, "watchdog healthy");
      RETURN_IF_ERROR_CODE_HAL(HAL_IWDG_Refresh(&hiwdg), &healthFlags.watchdog);
    }
    osDelay(50);
  }
}