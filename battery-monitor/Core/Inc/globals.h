
#ifndef GLOBALS_H
#define GLOBALS_H

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

// variables
extern volatile float fBatteryVoltage;
extern const float fThresholdVoltage;
// button debouncing
extern volatile uint32_t currentTime;
extern volatile uint32_t previousTime;
// adc
extern ADC_HandleTypeDef hadc1;
// uart
extern UART_HandleTypeDef huart2;
// i2c
extern I2C_HandleTypeDef hi2c1;
// freertos objects
extern osMessageQueueId_t xUARTQueueHandle;
extern osSemaphoreId_t xBinSemHandle;
extern osMutexId_t xMutexHandle;


#endif // GLOBALS_H