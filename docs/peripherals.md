# Peripherals

## ADC
- **Resolution:** 12 bits
- **Sampling rate:**
- **Trigger source:** software

## UART
- **Baud rate:** 115200 bits/s
- **Word length:** 8 bits
- **Parity:** no parity 
- **Stop bits:** 1

## GPIO

|Pin|Component|Description|
|--|--|--|
|PA5|GPIO_Output|Write to green LED|
|PA6|GPIO_Output|Write to red LED|
|PC13|GPIO_Input|Button press and interrupt|
|PA2|USART_Tx|Transmit data from STM32|
|PA3|USART_Rx|Transmit data to STM32|
|PC1|ADC1_IN11|Read voltage value|
|PB6|I2C1_SCL|I2C serial clock|
|PB7|I2C1_SDA|I2C serial data|
|3.3V|n/a|Power the potentiometer and LEDs|
|GND|n/a|Ground|



**SysTick**
- Hardware timer
- Generates interrupt at regular interval
- Used for button debouncing (`HAL_GetTick()`) and sleeping tasks (`osDelay()`)