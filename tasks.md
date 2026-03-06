# battery-monitor
Real-time battery monitor made with STM32 and FreeRTOS

# Progress

## Test GPIO and ADC

- [x] GPIO
	- [x] Assemble basic LED circuit
	- [x] Code the LED to blink
- [x] ADC
	- [x] Connect potentiometer ADC, and LEDs
	- [x] Code 
		- [x] Polling function
			- [x] Convert raw value to voltage
			- [x] Turn on green LED if V > 2.0 V, red otherwise
	- [x] Assemble and test circuit

## Task Setup

- [x] Set up FreeRTOS
- [x] Learn how to use FreeRTOS tasks
- [x] Migrate testing code to RTOS tasks
	- [x] Read background info on volatile keyword
	- [x] Set stack size
	- [x] Make global variable for battery voltage
	- [x] `Task_Sensor`: monitor voltage
	- [x] `Task_Alarm`: compare voltage to desired threshold and turn on LEDs accordingly
	- [x] Test code

## Implement Mutexes and Interrupts

- [x] Implement data protection with mutexes
	- [x] Read background info on mutexes, semaphores, priority inversion
	- [x] `Task_Sensor`: lock mutex before writing, unlock after
	- [x] `Task_Alarm`: lock mutex before reading
- [x] Implement interrupts and signaling with semaphores
	- [x] Read background info on ISRs and using semaphores for events
	- [x] Connect button to circuit
	- [x] Configure button GPIO pin as external interrupt
	- [x] Write interrupt service routine (ISR) 
- [x] Implement UART communication
	- [x] Read background info on parsing packets, checking for start of frame, validating checksum
	- [x] Create task to send voltage via UART

