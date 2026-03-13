#include "globals.h"

volatile float fBatteryVoltage = 0.0f;
const float fThresholdVoltage = 1.5f;
volatile uint32_t currentTime = 0;
volatile uint32_t previousTime = 0;

ADC_HandleTypeDef hadc1;
UART_HandleTypeDef huart2;
osMessageQueueId_t xUARTQueueHandle;
osSemaphoreId_t xBinSemHandle;
osMutexId_t xMutexHandle;
I2C_HandleTypeDef hi2c1;