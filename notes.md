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

## UART

- Asynchronous - doesn't need clock
- Duplex communication - can send and receive data simultaneously

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



