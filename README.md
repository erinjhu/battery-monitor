# battery-monitor
Real-time battery monitor made with STM32 and FreeRTOS

# Progress

## Test GPIO and ADC

- [ ] GPIO
	- [ ] Assemble basic LED circuit
	- [ ] Code the LED to blink
- [ ] ADC
	- [ ] Connect potentiometer, ADC, and voltage divider
	- [ ] Code 
		- [ ] Initialize ADC registers
		- [ ] Polling function
			- [ ] Convert raw value to voltage
			- [ ] Turn on green LED if V > 2.0 V, red otherwise
	- [ ] Assemble and test circuit

## Task Setup

- [ ] Set up FreeRTOS
- [ ] Migrate testing code to RTOS tasks
	- [ ] Read background info on volatile keyword
	- [ ] Set stack size
	- [ ] Make global variable for battery voltage
	- [ ] `Task_Sensor`: monitor voltage
	- [ ] `Task_Alarm`: compare voltage to desired threshold and turn on LEDs accordingly
	- [ ] Test code

## Implement Mutexes and Interrupts

- [ ] Implement data protection with mutexes
	- [ ] Read background info on mutexes, semaphores, priority inversion
	- [ ] `Task_Sensor`: lock mutex before writing, unlock after
	- [ ] `Task_Alarm`: lock mutex before reading
- [ ] Implement interrupts and signaling with semaphores
	- [ ] Read background info on ISRs and using semaphores for events
	- [ ] Connect button to circuit
	- [ ] Configure button GPIO pin as external interrupt
	- [ ] Write interrupt service routine (ISR) 
- [ ] Implement UART communication
	- [ ] Read background info on parsing packets, checking for start of frame, validating checksum
	- [ ] Create task to send voltage via UART
