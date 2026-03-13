#include "voltage_mgr.h"
#include "main.h"
#include "cmsis_os.h"
#include "globals.h"
#include "messages.h"

void vTaskVoltageMgr(void *argument)
{
  /* USER CODE BEGIN vTaskVoltageMgr */
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
  /* USER CODE END vTaskVoltageMgr */
}


