#include "main.h"
#include "cmsis_os.h"
#include "bmp180.h"
#include "messages.h"
#include "globals.h"
#include "iwdg.h"
#include "app_tasks.h"

osThreadId_t xTaskEnvMgrHandle;
uint32_t xTaskEnvMgrBuffer[ 128 ];
StaticTask_t xTaskEnvMgrControlBlock;
const osThreadAttr_t xTaskEnvMgr_attributes = {
  .name = "xTaskEnvMgr",
  .cb_mem = &xTaskEnvMgrControlBlock,
  .cb_size = sizeof(xTaskEnvMgrControlBlock),
  .stack_mem = &xTaskEnvMgrBuffer[0],
  .stack_size = sizeof(xTaskEnvMgrBuffer),
  .priority = (osPriority_t) osPriorityHigh,
};

void vTaskEnvMgr(void *argument)
{
    osStatus_t cmsisErrCode;
    // BMP180 sensor logic here
    for(;;)
    {
        healthFlags.env_mgr = HEALTH_ERROR;
        BMP180_Init();
        float temp = BMP180_GetTemperature();
        float pressure = BMP180_GetPressure();
        RETURN_IF_ERROR_CODE_CMSIS(osMutexAcquire(xMutexHandle,100), &healthFlags.env_mgr);
        fBatteryTemperature = temp;
        fBatteryPressure = pressure;
        UARTMsg_t tempMessage = {.type = MSG_TYPE_TEMP, .value = fBatteryTemperature};
        UARTMsg_t pressureMessage = {.type = MSG_TYPE_PRESSURE, .value = fBatteryPressure};
        RETURN_IF_ERROR_CODE_CMSIS(osMessageQueuePut(xUARTQueueHandle, &tempMessage, 0U, 10), &healthFlags.env_mgr);
        RETURN_IF_ERROR_CODE_CMSIS(osMessageQueuePut(xUARTQueueHandle, &pressureMessage, 0U,10), &healthFlags.env_mgr);
        xTaskNotifyGive(xTaskAlarmHandle);
        RETURN_IF_ERROR_CODE_CMSIS(osMutexRelease(xMutexHandle), &healthFlags.env_mgr);
        healthFlags.env_mgr = HEALTH_OK;
        osDelay(1000);
    }
}