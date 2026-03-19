# Obstacles

## Mar 18, 2026
- Did not know that you can compile in the STM32 IDE without the board attached.
- Forgot to create the threads, so nothing was running.
- Forgetting to define extern variables after declaring them

## Mar 14, 2026
- Use bit shifting instead of pow() from math.h.

## Mar 13, 2026
- Was using large breadboard. Forgot that the power rails don't run across the entire breadboard. VIN and GND of the BMP180 were connected to the half of the breadboard that wasn't connected to the STM32.
- Wrote BMP180 calibration function. Was trying to test it but forgot to call it in main.