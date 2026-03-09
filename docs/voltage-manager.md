# Voltage Monitor

This feature measures the voltage and turns on a green or red LED.

![Flow chart for tasks](../images/battery-monitor-flowchart.jpg)

## Tasks

|Name|Priority|Stack Size|Periodicity/Trigger|Description|
|--|--|--|--|--|
|`static TaskHandle_t xTaskSensor`|Low|128 words|100 ms|Monitor voltage using ADC|
|`staticTaskHandle_t xTaskAlarm`|High|128 words|Event-driven (sensor update)|Control LEDs|
|`static TaskHandle_t xTaskUART`|Medium|128 words|Event-driven (sensor update)|Transmit data|

- **Static:** limit variable's scope to `main.c` file for encapsulation

## Concurrency and Synchronization

|Component|Description|
|--|--|
|`SemaphoreHandle_t xMutex`|Manage shared access for `fBatteryVoltage`|
|`SemaphoreHandle_t xBinSemaphore`|Wake up `TaskAlarm` only when there is new data|
|`QueueHandle_t xUARTQueue`|Queue to send messages to `TaskUART`|

- Why does TaskSensor gives the semaphore before giving the mutex?
  - Prevent tasks from fighting over the mutex before they are "synced" by the semaphore
  - Give the semaphore first to let the scheduler organize the ready tasks 
- Why statically allocate tasks, mutexes, and semaphores?
  - Ensures determinism, no risk of runtime memory allocation failure

## Variables

|Variable|Description|
|--|--|
|`volatile float fBatteryVoltage`|Battery voltage read by ADC|
|`const float fThresholdVoltage`|Threshold for lighting up green/red LED|
|`volatile uint32_t currentTime`|Button debouncing|
|`volatile uint32_t prevTime`|Button debouncing|

- **Volatile:** ensure compiler doesn't optimize read/writes

## Messages

```c
typedef enum {
    MSG_TYPE_VOLTAGE,
    MSG_TYPE_BUTTON
} MsgType_t;

typedef struct {
    MsgType_t type;    
    float value;       
} UARTMsg_t;
```

