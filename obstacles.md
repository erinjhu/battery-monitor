# Obstacles

## Mar 24, 2026
- Only the default task was printing to the putty terminal. Put breakpoint into MX_FREERTOS_Init(). osThreadNew() valid handle for default task but NULL for the other ones. Not enough memory for the statically allocated buffer for the task.
    - Control Block (TCB): memory to manage the task's state, stack pointer, and priority
- Forgot to create the queue and mutex; made the handles but didn't call the functions to create them
- Did not know that tasks are not allowed to return in FreeRTOS
- Multiple tasks trying to use the same UART handle messing up the terminal prints

## Mar 18, 2026
- Did not know that you can compile in the STM32 IDE without the board attached.
- Forgot to create the threads, so nothing was running.
- Forgetting to define extern variables after declaring them

## Mar 14, 2026
- Use bit shifting instead of pow() from math.h.

## Mar 13, 2026
- Was using large breadboard. Forgot that the power rails don't run across the entire breadboard. VIN and GND of the BMP180 were connected to the half of the breadboard that wasn't connected to the STM32.
- Wrote BMP180 calibration function. Was trying to test it but forgot to call it in main.