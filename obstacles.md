# Obstacles

## Mar 13, 2026
- Was using large breadboard. Forgot that the power rails don't run across the entire breadboard. VIN and GND of the BMP180 were connected to the half of the breadboard that wasn't connected to the STM32.
- Wrote BMP180 calibration function. Was trying to test it but forgot to call it in main.