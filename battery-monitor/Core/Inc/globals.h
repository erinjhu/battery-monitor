
#ifndef GLOBALS_H
#define GLOBALS_H

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

// variables
extern volatile float fBatteryVoltage;
extern volatile float fBatteryTemperature;
extern volatile float fBatteryPressure;
extern const float fThresholdVoltage;
// button debouncing
extern volatile uint32_t currentTime;
extern volatile uint32_t previousTime;
// freertos objects
extern osMessageQueueId_t xUARTQueueHandle;
// extern osSemaphoreId_t xBinSemHandle;
extern osMutexId_t xMutexHandle;
extern osThreadId_t xTaskAlarm;



#endif // GLOBALS_H