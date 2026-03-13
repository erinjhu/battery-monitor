#include "env_mgr.h"
#include "main.h"
#include "cmsis_os.h"

void vTaskEnvMgr(void *argument)
{
    // BMP180 sensor logic here
    for(;;)
    {
        // Read sensor, process data, etc.
        osDelay(1000);
    }
}