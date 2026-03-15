#include "voltage_mgr.h"
#include "main.h"
#include "cmsis_os.h"
#include "globals.h"
#include "messages.h"
#include "FreeRTOS.h"
#include "task.h"

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

void vTaskVoltageMgr(void *argument)
{
  /* USER CODE BEGIN vTaskVoltageMgr */
  HAL_UART_Transmit(&huart2, (uint8_t*)"Sensor Task Started\r\n", 21, 100);
  /* Infinite loop */
  osStatus_t errCode;
  for(;;)
  {
    HAL_ADC_Start(&hadc1);

    RETURN_IF_ERROR_CODE(HAL_ADC_PollForConversion(&hadc1,100));
    RETURN_IF_ERROR_CODE(osMutexAcquire(xMutexHandle,100));

    uint32_t adcRaw = HAL_ADC_GetValue(&hadc1);
    fBatteryVoltage = (adcRaw * 3.3f) / 4095.0f;
    UARTMsg_t batteryMessage = {.type = MSG_TYPE_VOLTAGE, .value = fBatteryVoltage};
    RETURN_IF_ERROR_CODE(osMessageQueuePut(xUARTQueueHandle, &batteryMessage, 0U, 10));
    // osSemaphoreRelease(xBinSemHandle);
    xTaskNotifyGive(xTaskAlarm);
    RETURN_IF_ERROR_CODE(osMutexRelease(xMutexHandle));
    osDelay(100);
  }
  /* USER CODE END vTaskVoltageMgr */
}


