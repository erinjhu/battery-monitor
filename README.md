# Battery Monitor

Embedded STM32 project for monitoring battery voltage, controlling LEDs, and communicating via UART. Uses FreeRTOS for multitasking and demonstrates key embedded concepts.

## Table of Contents
1. [Project Overview](#project-overview)
2. [Documentation Index](#documentation-index)
3. [Version History](#version-history)
4. [Next Steps](#next-steps)

## Documentation Index
- [Voltage Manager](docs/voltage-manager.md)
- [Peripherals](docs/peripherals.md)
- [Interrupts](docs/interrupts.md)

## Project Overview
This project monitors battery voltage using an STM32 microcontroller, displays status with LEDs, and transmits data over UART. It demonstrates:
- ADC voltage measurement
- FreeRTOS tasks, mutexes, semaphores, and queues
- Button debouncing and external interrupts
- UART communication

![Flow chart for tasks](images/battery-monitor-flowchart.jpg)

## Version History

|Version|Date|Description|
|--|--|--|
|3|2026-03|Integrate BMP180 sensor with I2C, monitor tasks with watchdog, implement direct task notifications, and use software timer for status LED|
|2|2026-02|Turn on LED based on voltage in ADC, output to UART, and control battery variable with mutex and semaphore|
|1|2026-01|Implement tutorials for basic ADC, semaphore, mutex, queue, tasks, and timer|

## Next Steps
- BME280: I2C
- 74HC595: SPI, 7-segment display
- Buzzer: PWM tones
