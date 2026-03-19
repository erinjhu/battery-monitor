#ifndef ENV_MGR_H
#define ENV_MGR_H

#include "cmsis_os.h"


extern osThreadId_t xTaskEnvMgrHandle;


void vTaskEnvMgr(void *argument);

#endif // ENV_MGR_H