# Obstacles

pain.

## Mar 26, 2026
- Accidental git restore .
- Couldn't figure out why updated outputs weren't printing. Had to clean the project
- Running code before uart was initialized
- Watchdog task was logging all the health flags, causing the alarm task to not turn on the LEDs. Could be due to filling up the UART queue, causing the watchdog task to block.
- Not receiving any data from the BMP180. Changed pins to GPIO to turn on LEDs and pins seem to be working fine
- Was reading pinout upside down so connected to wrong pin.
- Had correct prescaler and period but not the right frequency, so didn't show pulsing LED; LED was just staying on

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