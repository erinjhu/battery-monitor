#ifndef ENV_MGR_H
#define ENV_MGR_H

extern osThreadId_t xTaskEnvMgrHandle;


void vTaskEnvMgr(void *argument);

#endif // ENV_MGR_H