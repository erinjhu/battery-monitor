#include "app_tasks.h"
#include "main.h"
#include "cmsis_os.h"
#include "globals.h"
#include "gpio.h"
#include "messages.h"

void StartDefaultTask(void *argument)
{
    for(;;)
    {
        osDelay(1000);
    }
}

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

void vTaskAlarm(void *argument)
{
  /* USER CODE BEGIN vTaskAlarm */
  HAL_UART_Transmit(&huart2, (uint8_t*)"Alarm Task Started\r\n", 21, 100);
  osStatus_t errCode;
  /* Infinite loop */
  for(;;)
  {
    
    errCode = osSemaphoreAcquire(xBinSemHandle, HAL_MAX_DELAY);
    if (errCode != osOK)
    {
      UARTMsg_t errMsg = {.type = MSG_TYPE_ERROR, .errCode = ERR_CODE_UNKNOWN};
      osMessageQueuePut(xUARTQueueHandle, &errMsg, 0U, 0);
      return;
    }
    errCode = osMutexAcquire(xMutexHandle,100);
    if (errCode != osOK)
    {
      UARTMsg_t errMsg = {.type = MSG_TYPE_ERROR, .errCode = ERR_CODE_UNKNOWN};
      osMessageQueuePut(xUARTQueueHandle, &errMsg, 0U, 0);
      return;
    }
    float batteryVoltage = fBatteryVoltage;
    osMutexRelease(xMutexHandle);
    if (batteryVoltage > fThresholdVoltage)
    {
      HAL_UART_Transmit(&huart2, (uint8_t*)"Turn on green LED\r\n", 30, 100);
      HAL_GPIO_WritePin(GPIOA, GPIO_GREEN_LED, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOA, GPIO_RED_LED, GPIO_PIN_RESET);
    } 
    else
    {
      HAL_UART_Transmit(&huart2, (uint8_t*)"Turn on red LED\r\n", 30, 100);
      HAL_GPIO_WritePin(GPIOA, GPIO_RED_LED, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOA, GPIO_GREEN_LED, GPIO_PIN_RESET);
    }
  }
  /* USER CODE END vTaskAlarm */
}