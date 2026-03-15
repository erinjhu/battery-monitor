#include "env_mgr.h"
#include "main.h"
#include "cmsis_os.h"
#include "bmp180.h"
#include "messages.h"

void vTaskEnvMgr(void *argument)
{
    // BMP180 sensor logic here
    for(;;)
    {
        BMP180_Init();
        float temp = BMP180_GetTemperature();
        float pressure = BMP180_GetPressure();
        RETURN_IF_ERROR_CODE(osMutexAcquire(xMutexHandle,100));
        fBatteryTemperature = temp;
        fBatteryPressure = pressure;
        UARTMsg_t tempMessage = {.type = MSG_TYPE_TEMP, .value = fBatteryTemperature};
        UARTMsg_t pressureMessage = {.type = MSG_TYPE_PRESSURE, .value = fBatteryPressure};
        RETURN_IF_ERROR_CODE(osMessageQueuePut(xUARTQueueHandle, &tempMessage, 0U, 10));
        RETURN_IF_ERROR_CODE(osMessageQueuePut(xUARTQueueHandle, &pressureMessage, 0U,10));
        xTaskNotifyGive(xTaskAlarm);
        RETURN_IF_ERROR_CODE(osMutexRelease(xMutexHandle));
        osDelay(1000);
    }
}