#include "env_mgr.h"
#include "main.h"
#include "cmsis_os.h"
#include "bmp180.h"
#include "messages.h"
#include "globals.h"
#include "iwdg.h"

void vTaskEnvMgr(void *argument)
{
    // BMP180 sensor logic here
    for(;;)
    {
        healthFlags.env_mgr = HEALTH_ERROR;
        BMP180_Init();
        float temp = BMP180_GetTemperature();
        float pressure = BMP180_GetPressure();
        RETURN_IF_ERROR_CODE_HEALTH(osMutexAcquire(xMutexHandle,100), &healthFlags.env_mgr);
        fBatteryTemperature = temp;
        fBatteryPressure = pressure;
        UARTMsg_t tempMessage = {.type = MSG_TYPE_TEMP, .value = fBatteryTemperature};
        UARTMsg_t pressureMessage = {.type = MSG_TYPE_PRESSURE, .value = fBatteryPressure};
        RETURN_IF_ERROR_CODE_HEALTH(osMessageQueuePut(xUARTQueueHandle, &tempMessage, 0U, 10), &healthFlags.env_mgr);
        RETURN_IF_ERROR_CODE_HEALTH(osMessageQueuePut(xUARTQueueHandle, &pressureMessage, 0U,10), &healthFlags.env_mgr);
        xTaskNotifyGive(xTaskAlarm);
        RETURN_IF_ERROR_CODE_HEALTH(osMutexRelease(xMutexHandle), &healthFlags.env_mgr);
        healthFlags.env_mgr = HEALTH_OK;
        osDelay(1000);
    }
}