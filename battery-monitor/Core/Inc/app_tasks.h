#ifndef APP_TASKS_H
#define APP_TASKS_H

#include "cmsis_os.h"

extern osThreadId_t xTaskAlarm;
extern osThreadId_t xTaskWatchdog;


void vTaskUART(void *argument);
void vTaskAlarm(void *argument);
void vTaskWatchdog(void *argument);

#endif // APP_TASKS_H