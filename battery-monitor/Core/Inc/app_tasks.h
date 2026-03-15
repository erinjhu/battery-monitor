#ifndef APP_TASKS_H
#define APP_TASKS_H

extern osThreadId_t xTaskAlarm;
extern osThreadId_t xTaskWatchdog;


void StartDefaultTask(void *argument);
void vTaskUART(void *argument);
void vTaskAlarm(void *argument);
void vTaskWatchdog(void *argument);

#endif // APP_TASKS_H