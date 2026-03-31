#ifndef APP_TASKS_H
#define APP_TASKS_H

#include "cmsis_os.h"

extern osThreadId_t xTaskAlarmHandle;
extern osThreadId_t xTaskWatchdogHandle;
extern osThreadId_t xTaskUARTHandle;
extern osThreadId_t xTaskEnvMgrHandle;
extern osThreadId_t xTaskVoltageMgrHandle;
extern osThreadId_t xTaskPWMController;

void vTaskUART(void *argument);
void vTaskAlarm(void *argument);
void vTaskWatchdog(void *argument);
void vTaskEnvMgr(void *argument);
void vTaskVoltageMgr(void *argument);
void vTaskPWMController(void *argument);


extern const osThreadAttr_t xTaskAlarm_attributes;
extern const osThreadAttr_t xTaskWatchdog_attributes;
extern const osThreadAttr_t xTaskUART_attributes;
extern const osThreadAttr_t xTaskEnvMgr_attributes;
extern const osThreadAttr_t xTaskVoltageMgr_attributes;
extern const osThreadAttr_t xUARTQueue_attributes;
extern const osThreadAttr_t xMutex_attributes;
extern const osThreadAttr_t xTaskPWMController_attributes;



#endif // APP_TASKS_H