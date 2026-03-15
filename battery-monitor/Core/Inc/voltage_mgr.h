#ifndef VOLTAGE_MGR_H
#define VOLTAGE_MGR_H

void vTaskVoltageMgr(void *argument);

extern osThreadId_t xTaskSensorHandle;


#endif // VOLTAGE_MGR_H