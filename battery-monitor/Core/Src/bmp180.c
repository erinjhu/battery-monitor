#include "bmp180.h"
#include "globals.h"
#include "messages.h"
#include <math.h>



// Private instance of the calibration struct
BMP180_CalibData_t calib_data;

void BMP180_Init(void) {
    uint8_t chip_id = 0;
    HAL_StatusTypeDef errCode;
    RETURN_IF_ERROR_CODE(HAL_I2C_Mem_Read(&hi2c1, BMP180_I2C_DEVICE_ADDR, 0xD0, I2C_MEMADD_SIZE_8BIT, &chip_id, 1, 100));
    if (chip_id == 0x55) {
        BMP180_ReadCalibrationData();
    }
    else
    {
    	chip_id = 0xA; // dummy value that isn't 0 or 0x55
        UARTMsg_t errMsg = {.type = MSG_TYPE_ERROR, .errCode = ERR_CODE_FAILED_BMP180_INIT};
        osMessageQueuePut(xUARTQueueHandle, &errMsg, 0U, 0);
        return;
    }
}

void BMP180_ReadCalibrationData(void) {
    // Read from I2C
    uint8_t calib_raw[22];
    HAL_StatusTypeDef errCode;
    RETURN_IF_ERROR_CODE(HAL_I2C_Mem_Read(&hi2c1, BMP180_I2C_DEVICE_ADDR, BMP180_REG_AC1_MSB, I2C_MEMADD_SIZE_8BIT, calib_raw, 22, 100));
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
    HAL_StatusTypeDef errCode;
    RETURN_IF_ERROR_CODE(HAL_I2C_Mem_Write(&hi2c, BMP180_I2C_DEVICE_ADDR, BMP180_REG_CONTROl, I2C_MEMADD_SIZE_8BIT, BMP180_TEMP_CMD, ));
    // Delay task for 4.5 ms
    osDelay(5);
    // Read raw UT bits
    // Apply compensation math
    return BMP180_CalcTrueTemp(BMP180_ReadUncompTemp()); // Return final Celsius value
}

float BMP180_ReadUncompTemp(void) {
    uint8_t msb = 0x0;
    uint8_t lsb = 0x0;
    uint16_t uncomp_temp = 0x0;
    RETURN_IF_ERROR_CODE(HAL_I2C_Mem_Read(&hi2c,BMP180_I2C_DEVICE_ADDR,BMP180_REG_TEMP_MSB,I2C_MEMADD_SIZE_8BIT,&msb,1,100));
    RETURN_IF_ERROR_CODE(HAL_I2C_Mem_Read(&hi2c,BMP180_I2C_DEVICE_ADDR,BMP180_REG_TEMP_LSB,I2C_MEMADD_SIZE_8BIT,&lsb,1,100));
    return (msb << 8) + lsb;
}

float BMP180_CalcTrueTemp(uint16_t uncomp_temp) {
    // uint32_t rec in data sheet; prevents overflow when bitshifting and multiplying
    int32_t x1 = ((int32_t)uncomp_temp - (int32_t)calib_data.ac6) * (int32_t)calib_data.ac5 >> 15;
    int32_t x2 = ((int32_t)calib_data.mc << 11) / (x1 + calib_data.md);
    int32_t b5 = x1 + x2;
    float temp = ((b5 + 8) >> 4) / 10.0f; 
    return temp;
}

float BMP180_GetPressure(void) {
    // Write command to control register
    // Delay task based on oversampling setting
    // Read raw UP bits
    // Apply compensation math using intermediate temperature values
    return 0.0f; // Return final pressure value
}
