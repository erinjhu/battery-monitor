#include "env_mgr.h"
#include "main.h"
#include "cmsis_os.h"
#include "bmp180.h"

void vTaskEnvMgr(void *argument)
{
    // BMP180 sensor logic here
    for(;;)
    {
        BMP180_Init();
        float temp = BMP180_GetTemperature();
        float pressure = BMP180_GetPressure();
        osDelay(1000);
    }
}