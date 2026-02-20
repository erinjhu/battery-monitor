## Intro

This file documents concepts I've been learning.

### Resources Used
- [CircuitGator HQ's STM32 Tutorials](https://youtube.com/playlist?list=PLGh4659DkyarOFZVtnah4KORCJzuPbWg_&si=7iypIOyyFD7IGzBB)
- [ControllersTech's FreeRTOS Tutorials](https://youtube.com/playlist?list=PLfIJKC1ud8gj1t2y36sabPT4YcKzmN_5D&si=Jtzcy8ACzQ3xHdPo)

## GPIO

### Blinking LED

Turn LED ON/OFF

```C
/* USER CODE BEGIN 3 */
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1); // PB5
HAL_Delay(1000); // 1000 ms
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0); // PB5
HAL_Delay(1000); // 1000 ms
```

Toggle pin

```C
/* USER CODE BEGIN 3 */
HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
HAL_Delay(1000); // 1000 ms
```


## PWM

### PWM Frequency

Since $f_{\text{clock}}$ is usually too fast for PWM, use the prescaler and period values to obtain the desired $f_{\text{PWM}}$.

$f_{\text{PWM}} = \frac{f_\text{clock}}{(1 + \text{Prescaler})(1 + \text{Period})}$

| Variable           | Description                                                                                                                                                                           |
| ------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| $f_{\text{PWM}}$   | number of full on-off cycles completed per second (desired clock to run PWM signal)                                                                                                   |
| $f_{\text{clock}}$ | number of full on-off cycles completed per second (given clock; usually depends on the device)                                                                                        |
| $\text{Prescaler}$ | factor to divide the system clock by to get a slower value. use 1 + prescaler because there is a tick when the counter is at 0.                                                       |
| $\text{Period}$    | number of timer counts (ticks). e.g. if $\text{Period}=99$, there will be 100 ticks (timer at 0 for one tick, timer at 1 for one tick, ... , timer at 99 for one tick, repeat from 0) |

### Duty Cycle

The compare register value is the number of ticks when the desired duty cycle will be completed for a certain period.

$\text{Compare Register Value}=\frac{\text{Duty Cycle} }{100} \cdot \text{Period}$

| Variable                        | Description                                                                           |
| ------------------------------- | ------------------------------------------------------------------------------------- |
| $\text{Compare Register Value}$ | when the counter reaches the compare register value, switch the signal from ON to OFF |
| $\text{Duty Cycle}$             | % of the period that the signal is ON                                                 |
| $\text{Period}$                 | number of ticks in one full cycle                                                     |


### Change LED Brightness

* After each iteration of the first for loop:
	* $\uparrow$ compare value $\rightarrow$ $\uparrow$ duty cycle $\rightarrow$ $\uparrow$ fraction of the period where LED is ON, making it appear brighter
* After each iteration of the second for loop:
	* $\downarrow$ compare value $\rightarrow$ $\downarrow$ duty cycle $\rightarrow$ $\downarrow$ fraction of the period where LED is ON, making it appear dimmer

```C
/* USER CODE BEGIN 2 */
HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
/* USER CODE END 2 */
/* Infinite loop */
/* USER CODE BEGIN WHILE */
while (1) {
/* USER CODE END WHILE */
/* USER CODE BEGIN 3 */
	for(int i = 0; i < 210; i++) {
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, i);
		HAL_Delay(2);
	}
	for(int i = 210; i > 0; i--) {
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, i);
		HAL_Delay(2);
	}
}
/* USER CODE END 3 */
```
## ADC

### Potentiometer

- Acts as a resistor if the outer two terminals are connected
- Middle terminal is a wiper

### Simple Potentiometer Circuit

* Connect the potentiometer to 3.3V, GND, and the ADC pin
* The code below prints `readVal` which is a number between 0 (ground) to 4095 (reference voltage, which is 3.3V)

```C
while (1){
/* USER CODE END WHILE */
/* USER CODE BEGIN 3 */
	printf("test print\r\n");
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	readVal = HAL_ADC_GetValue(&hadc1);
	sprintf(tx_buffer, "%hu\r\n", readVal);
	HAL_UART_Transmit(&huart2, (uint8_t*)tx_buffer, strlen(tx_buffer), HAL_MAX_DELAY);
	HAL_Delay(10);
}
/* USER CODE END 3 */
```

$V=\frac{\text{readVal}}{2^n-1} \cdot V_{\text{ref}}$

| Variable         | Description                                      |
| ---------------- | ------------------------------------------------ |
| $n$              | ADC resolution; the STM32 board has a 12-bit ADC |
| $V_{\text{ref}}$ | reference voltage (3.3V)                         |
| $\text{readVal}$ | raw value read from ADC					      |
| $\frac{\text{readVal}}{2^n-1}$ | normalized value (from 0 to 1)|

To use the potentiometer to change the LED brightness

```C
/* USER CODE BEGIN 3 */

printf("test print\r\n");
HAL_ADC_Start(&hadc1);
HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
readVal = HAL_ADC_GetValue(&hadc1);
sprintf(tx_buffer, "%hu\r\n", readVal);
HAL_UART_Transmit(&huart2, (uint8_t*)tx_buffer, strlen(tx_buffer), HAL_MAX_DELAY);
HAL_Delay(10);
uint32_t pwmVal = (readVal * 209) / 4095;
// 4095 is the max val
// 209 is the counter period
__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwmVal);
}
```
### ADC Polling

* Code repeatedly checks a peripheral for an event or data
* For ADC polling, the code waits for the ADC conversion to finish and then reads the value

Put this in the main infinite while loop and replace the parameters.

```c
HAL_ADC_Start(ADC_HandleTypeDef *hadc);
HAL_ADC_PollForConversion*ADC_HandleTypeDef *hadc, uint32_t Timeout);
// timeout is how long the function should wait for the ADC conversion to complete
// if the conversion doesn't finish within the timeout, the function will return with a timeout error
// can use HAL_MAX_DELAY
HAL_ADC_GetValue(ADC_HandleTypeDef *hadc)
```


## UART

- Asynchronous - doesn't need clock
- Duplex communication - can send and receive data simultaneously

Use `uint8_t` because UART transmits one byte/char at a time

```c
uint8_t data[] = "Data to transmit on UART\n";
HAL_UART_Transmit(&huart2, data, sizeof(data), 500);
```

### Data

Made of...
- Start bit
- Data bits
- Parity bit
- Stop bit

Baud rate:
- Number of bits transmitted per second
- Both sender and receiver must have the same baud rate

Settings used

| Parameter   | Value                     |
| ----------- | ------------------------- |
| Baud Rate   | 115200 Bits/s             |
| Word Length | 8 Bits (including Parity) |
| Stop Bits   | 1                         |

When the microcontroller receives data from the computer, it will trigger an interrupt

### Key Press Interrupt to Change LED Colour

**Private variables section (top of `main.c`)**

1. Initialize the starting message to send on UART.
```c
char tx_data[] = "Select colour: R, G, or B: \r\n";
char rx_data[1];
```

2. Transmit the data across UART.

```c
HAL_UART_Transmit(&huart2, tx_data, sizeof(tx_data), HAL_MAX_DELAY);
```

**Infinite loop**

Receive the data from UART. This function configures the hardware to wait for incoming data from a key press. 

When you press a key, the hardware triggers an interrupt, and the HAL library will call the callback function.

```c
HAL_UART_Receive_IT(&huart2, rx_data, sizeof(rx_data));
```

**Callback function**

Pressing a character triggers an interrupt, and the callback function is called. 

```c
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(!strcmp(rx_data, "R")) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 0);
		char msg[] = "Red selected\r\n";
		HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	}
}
```

## FreeRTOS

### Tasks

When setting up the project in STM32CubeMX, change `USE_PREEMPTION` to `Enabled`.
  - Allows higher priority tasks to take control from lower priority tasks
  - Importance: prevent delays to critical tasks 


| Code   | Description                     |
| ----------- | ------------------------- |
| `osThreadId taskHandleName;`   | set up the task handle (variable that stores a reference to a thread/task)       |
| `void taskInitializationFunction(void const* argument)`; | main function of a task |
|`osThreadDef(Task2, Task2_init, osPriorityNormal, 0, 128);`|create the thread and set the name, entry point, priority, instances (0 = 1 instance), and stack size (bytes)|
| `Task2Handle = osThreadCreate(osThread(Task2), NULL);`|assign the task to the handle |

For the task functions, put an infinite loop since it doesn't return a value

### Multiple Tasks Printing to UART



Each task function:

```c
void Task2_init(void const * argument)
{
  /* USER CODE BEGIN Task2_init */
  /* Infinite loop */
  for(;;)
  {
    uint8_t data[] = "Hello from Task2\r\n";
	HAL_UART_Transmit(&huart2, data, sizeof(data), 500);
    osDelay(1000);
  }
  /* USER CODE END Task2_init */
}
```

All the tasks use the same UART. Make the delay the same for each task.
- If all 3 tasks have the same priority, each task will transmit every 3 seconds.
  - The scheduler will run them using round-robin (tasks take turns; each turn is equal)
  - The time slice for each turn allocated to each task's turn depends on the config settings
- If you make them all have different priorities, all 3 will appear to print at the same time.
  - Calling osDelay(1000) makes a task go to sleep
  - Run the highest priority task, then it goes to sleep. Then the next priority task will run.
  - Since printing happens quickly, the scheduler will go to the next priority task immediately after


### Scheduling

`osKernelStart();` 
- starts the RTOS kernel scheduler, which executes the created tasks/threads
- RTOS takes over CPU and runs tasks/threads according to priorities
- code in `main()` won't run unless tasks are deleted or scheduler is stopped

When you call `osDelay()` in a task, it blocks that task. The scheduler can switch to other ready tasks.

**Block task** 

Scheduler can switch to other ready tasks. If $f=1\text{kHz}$ then $1\text{ tick}=1\text{ms}$.

|Code|Description|
|--|--|
|`osDelay(x)`|delay for x ticks starting from current time|
|`osDelayUntil(&refTime, x)`|delay task until tick count reaches refTime + x|


### Managing Tasks

| Code   | Description                     |
| ----------- | ------------------------- |
| `osThreadSuspend(osThreadId thread_id)`   | Suspend |
|`osThreadResume(osThreadId thread_id)`|Resume|
|`osThreadTerminate(osThreadId thread_id)`|Terminate (can't be resumed after)|
|`osThreadSuspendAll(osThreadId thread_id)`|Suspend all|
|`osThreadResumeAll(osThreadId thread_id)`|Resume all|
|`osThreadYield()`|Pass control to next ready thread|
|`osThreadSetPriority(osThreadId thread_id, osPriority priority)`|Set priority|

|`osDelay()`|`osThreadSuspend()`|
|--|--|
|Pause task for specific amount of time. Task automatically becomes ready after and scheduler will resume task.|Pause task until you explicitly resume it with `osThreadResume()`.|

- When you call `osDelay()` in a task's function, it will yeild the CPU for that amount of time
  - After the delay, the scheduler will run whatever task has a higher priority

### Binary Semaphore

|Code|Description|
|--|--|
|`osSemaphoreId osSemaphoreCreate(const osSemaphoreDef_t * semaphore_def, int32_t count)`|Create semaphore. For binary semaphore, count = 1|
|`int32_t osSemaphoreWait(osSemaphoreId semaphore_id, uint32_t millisec)`|Block the task until the semaphore is available or the timeout has passed|
|`osStatus osSemaphoreRelease(osSemaphoreId semaphore_id)`|Release semaphore|

**Priority Inversion**

- High priority task is blocked because a lower priority task has the semaphore.
- Need to wait until low priority task releases semaphore

At any time, the scheduler will run the highest priority task that is ready or running.

**Priority Inheritance**

- A lower priority task temporarily becomes a higher priority
- Prevents priority inversion
- Allows the lower priority task to finish and then release the semaphore
- Lower priority task doesn't get pre-empted before it releases the semaphore

### Counting Semaphore

The semaphore doesn't have to be released by the task that acquired it

## Queue

### Simple Queue Tutorial
- Create tasks:
  - High priority sender
  - Low priority sender
  - Receiver


```c
BaseType_t xTaskCreate(
    TaskFunction_t pvTaskCode,    // Pointer to the task entry function
    const char * const pcName,    // Name of the task (for debugging)
    configSTACK_DEPTH_TYPE usStackDepth, // Stack size in words, not bytes
    void *pvParameters,           // Parameters to pass to the task
    UBaseType_t uxPriority,       // Task priority
    TaskHandle_t *pxCreatedTask   // Pointer to return the task handle (can be NULL if you don't need to reference/interact with task after creating it)
);
```

Create queue
```c
xQueueCreate(
    UBaseType_t uxQueueLength,   // The maximum number of items the queue can hold
    UBaseType_t uxItemSize       // The size (in bytes) of each item in the queue
);
```

```c
uint32_t TickDelay = pdMS_To_TICKS(2000); // Conver milliseconds to # of FreeRTOS ticks
// # of ticks per ms depends on the tick rate (config settings)
vTaskDelay(TickDelay);
```

```c
BaseType_t xQueueSend(
    QueueHandle_t xQueue,   // The queue to which the item will be sent
    const void * pvItemToQueue, // Pointer to the item to send
    TickType_t xTicksToWait     // How long to wait if the queue is full (in ticks)
);
```


```c
int sprintf(
	char *str, // pointer to buffer to store the string
	const char *format, // the actual string with formats such as %d
	... // values for the formats; e.g. an int variable for %d
);

```

```c
BaseType_t xQueueReceive(
    QueueHandle_t xQueue,     // The queue to receive from
    void *pvBuffer,           // Pointer to buffer to store the received item
    TickType_t xTicksToWait   // How long to wait if the queue is empty (in ticks)
);
```

## Timers

### One-Shot Timer
- Do callback function once
- Will not automatically restart (need to restart automatically)

### Auto-Reload Timer
- Will automatically restart callback function; callback function will run periodically

## Info 



### FreeRTOS
- RTOS kernel
- Used for multitasking

#### Prefixes

x: function returns a value, which is usually BaseType_t (status/result)
v: function returns void; no return value

`BaseType_t` is the most efficient int type for the target architecture (usually int or long)

### CMSIS
- Like a wrapper 
- Standardized APIs to work with different RTOS kernels (FreeRTOS, etc.)

### Kernel
- Part of an OS
- Manages tasks, scheduling, memory, hardware access

### OS
- Includes the kernel
- Also includes drivers, libraries, user interfaces
