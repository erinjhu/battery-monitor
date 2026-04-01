#include "main.h"
#include "cmsis_os.h"
#include "globals.h"
#include "messages.h"
#include "FreeRTOS.h"
#include "task.h"
#include "iwdg.h"
#include "usart.h"
#include "adc.h"
#include "app_tasks.h"



#define ADC_BUFFER_SIZE 8

static uint32_t adcRingBuffer[ADC_BUFFER_SIZE] = {0};
static uint8_t adcBufferIndex = 0;
static uint32_t adcRunningSum = 0;
static uint8_t isBufferFull = 0; 

uint32_t filterADCReading(uint32_t newRawValue) {
    // Subtract the oldest reading from the sum
    adcRunningSum -= adcRingBuffer[adcBufferIndex];

    adcRingBuffer[adcBufferIndex] = newRawValue;
    adcRunningSum += newRawValue;

    adcBufferIndex++;
    if (adcBufferIndex >= ADC_BUFFER_SIZE) {
        adcBufferIndex = 0;
        isBufferFull = 1; 
    }

    if (!isBufferFull) {
        // check if index is 0. if it is 0, divide by 1
        return adcRunningSum / (adcBufferIndex == 0 ? 1 : adcBufferIndex); 
    } else {
        return adcRunningSum / ADC_BUFFER_SIZE; 
    }
}

osThreadId_t xTaskVoltageMgrHandle;
uint32_t xTaskVoltageMgrBuffer[ 128 ];
StaticTask_t xTaskVoltageMgrControlBlock;
const osThreadAttr_t xTaskVoltageMgr_attributes = {
  .name = "xTaskVoltageMgr",
  .cb_mem = &xTaskVoltageMgrControlBlock,
  .cb_size = sizeof(xTaskVoltageMgrControlBlock),
  .stack_mem = &xTaskVoltageMgrBuffer[0],
  .stack_size = sizeof(xTaskVoltageMgrBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};

void vTaskVoltageMgr(void *argument)
{
  // HAL_UART_Transmit(&huart2, (uint8_t*)"Sensor Task Started\r\n", 21, 100);
  LOG_FROM_TASK(ERR_CODE_SUCCESS, MSG_TYPE_HEALTH, "Sensor Task Started");
  /* Infinite loop */
  HAL_StatusTypeDef halErrCode;
  osStatus_t cmsisErrCode;
  for(;;)
  {
    HAL_ADC_Start(&hadc1);

    RETURN_IF_ERROR_CODE_HAL(HAL_ADC_PollForConversion(&hadc1,100), &healthFlags.voltage_mgr);
    RETURN_IF_ERROR_CODE_CMSIS(osMutexAcquire(xMutexHandle,100), &healthFlags.voltage_mgr);

    uint32_t adcRaw = filterADCReading(HAL_ADC_GetValue(&hadc1));
    fBatteryVoltage = (adcRaw * 3.3f) / 4095.0f;
    UARTMsg_t batteryMessage = {.type = MSG_TYPE_VOLTAGE, .value = fBatteryVoltage};
    RETURN_IF_ERROR_CODE_CMSIS(osMessageQueuePut(xUARTQueueHandle, &batteryMessage, 0U, 10), &healthFlags.voltage_mgr);
    // osSemaphoreRelease(xBinSemHandle);
    xTaskNotifyGive(xTaskAlarmHandle);
    RETURN_IF_ERROR_CODE_CMSIS(osMutexRelease(xMutexHandle), &healthFlags.voltage_mgr);
    healthFlags.voltage_mgr = HEALTH_OK;
    osDelay(100);
  }
}


