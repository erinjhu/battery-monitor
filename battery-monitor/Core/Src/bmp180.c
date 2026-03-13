#include "bmp180.h"
#include "globals.h"




// Private instance of the calibration struct
BMP180_CalibData_t calib_data;

void BMP180_Init(void) {
    BMP180_ReadCalibrationData();
}

void BMP180_ReadCalibrationData(void) {
    // Read from I2C
    uint8_t calib_raw[22];
    HAL_I2C_Mem_Read(&hi2c1, BMP180_I2C_DEVICE_ADDR, BMP180_REG_AC1_MSB, I2C_MEMADD_SIZE_8BIT, &calib_raw, 22, 100);
    // Combine the bytes and put them into the struct
    calib_data.ac1 = (int16_t)((calib_raw[0] << 8) | calib_raw[1]);
    calib_data.ac2 = (int16_t)((calib_raw[2] << 8) | calib_raw[3]);
    calib_data.ac3 = (int16_t)((calib_raw[4] << 8) | calib_raw[5]);
    calib_data.ac4 = (uint16_t)((calib_raw[6] << 8) | calib_raw[7]);
    calib_data.ac5 = (uint16_t)((calib_raw[8] << 8) | calib_raw[9]);
    calib_data.ac6 = (uint16_t)((calib_raw[10] << 8) | calib_raw[11]);
    calib_data.b1  = (int16_t)((calib_raw[12] << 8) | calib_raw[13]);
    calib_data.b2  = (int16_t)((calib_raw[14] << 8) | calib_raw[15]);
    calib_data.mb  = (int16_t)((calib_raw[16] << 8) | calib_raw[17]);
    calib_data.mc  = (int16_t)((calib_raw[18] << 8) | calib_raw[19]);
    calib_data.md  = (int16_t)((calib_raw[20] << 8) | calib_raw[21]);
}

float BMP180_GetTemperature(void) {
    // Write command to control register
    // Delay task for 4.5 ms
    // Read raw UT bits
    // Apply compensation math
    return 0.0f; // Return final Celsius value
}

float BMP180_GetPressure(void) {
    // Write command to control register
    // Delay task based on oversampling setting
    // Read raw UP bits
    // Apply compensation math using intermediate temperature values
    return 0.0f; // Return final pressure value
}