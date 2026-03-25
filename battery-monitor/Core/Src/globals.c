#include "globals.h"
#include "messages.h"



volatile float fBatteryVoltage = 0.0f;
volatile float fBatteryTemperature = 0.0f;
volatile float fBatteryPressure = 0.0f;
const float fThresholdVoltage = 1.5f;
volatile uint32_t currentTime = 0;
volatile uint32_t previousTime = 0;
SystemHealth_t healthFlags;


osMessageQueueId_t xUARTQueueHandle;
uint8_t xUARTQueueBuffer[ 16 * sizeof( UARTMsg_t ) ];
StaticQueue_t xUARTQueueControlBlock;
const osMessageQueueAttr_t xUARTQueue_attributes = {
  .name = "xUARTQueue",
  .cb_mem = &xUARTQueueControlBlock,
  .cb_size = sizeof(xUARTQueueControlBlock),
  .mq_mem = &xUARTQueueBuffer,
  .mq_size = sizeof(xUARTQueueBuffer)
};

osMutexId_t xMutexHandle;
StaticSemaphore_t xMutexControlBlock;
const osMutexAttr_t xMutex_attributes = {
  .name = "xMutex",
  .cb_mem = &xMutexControlBlock,
  .cb_size = sizeof(xMutexControlBlock),
};

osMessageQDef_t xBinSemHandle;
StaticSemaphore_t xBinSemControlBlock;
const osSemaphoreAttr_t xBinSem_attributes = {
  .name = "xBinSem",
  .cb_mem = &xBinSemControlBlock,
  .cb_size = sizeof(xBinSemControlBlock),
};