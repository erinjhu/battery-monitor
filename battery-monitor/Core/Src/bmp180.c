#include "bmp180.h"
#include "globals.h"
#include "messages.h"
#include "i2c.h"


BMP180_CalibData_t calib_data;
static const uint8_t bmp180_conv_time[4] = {5, 8, 14, 26};
static int32_t b5;

void BMP180_ReadCalibrationData(void);
static float BMP180_ReadUncompTemp(void);
static float BMP180_ReadUncompPressure(void);
static float BMP180_CalcTruePressure(float uncomp_pressure);
static float BMP180_CalcTrueTemp(float uncomp_temp);

void BMP180_Init(void) {
    uint8_t chip_id = 0;
    HAL_StatusTypeDef halErrCode;
    RETURN_IF_ERROR_CODE_HAL(HAL_I2C_Mem_Read(&hi2c1, BMP180_I2C_DEVICE_ADDR, 0xD0, I2C_MEMADD_SIZE_8BIT, &chip_id, 1, 100), &healthFlags.bmp180);
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
    HAL_StatusTypeDef halErrCode;
    // Read from I2C
    uint8_t calib_raw[22];
    RETURN_IF_ERROR_CODE_HAL(HAL_I2C_Mem_Read(&hi2c1, BMP180_I2C_DEVICE_ADDR, BMP180_REG_AC1_MSB, I2C_MEMADD_SIZE_8BIT, calib_raw, 22, 100), &healthFlags.bmp180);
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
    HAL_StatusTypeDef halErrCode;
    uint8_t temp_cmd = BMP180_TEMP_CMD;
    RETURN_IF_ERROR_CODE_HAL(HAL_I2C_Mem_Write(&hi2c1, BMP180_I2C_DEVICE_ADDR, BMP180_REG_CONTROl, I2C_MEMADD_SIZE_8BIT, &temp_cmd, 1,100), &healthFlags.bmp180);
    // Delay task for 4.5 ms
    osDelay(5);
    // Read raw UT bits
    // Apply compensation math
    return BMP180_CalcTrueTemp(BMP180_ReadUncompTemp()); // Return final Celsius value
}

static float BMP180_ReadUncompTemp(void) {
    HAL_StatusTypeDef halErrCode;
    uint8_t msb = 0x0;
    uint8_t lsb = 0x0;
    uint16_t uncomp_temp = 0x0;
    RETURN_IF_ERROR_CODE_HAL(HAL_I2C_Mem_Read(&hi2c1,BMP180_I2C_DEVICE_ADDR,BMP180_REG_DATA_MSB,I2C_MEMADD_SIZE_8BIT,&msb,1,100), &healthFlags.bmp180);
    RETURN_IF_ERROR_CODE_HAL(HAL_I2C_Mem_Read(&hi2c1,BMP180_I2C_DEVICE_ADDR,BMP180_REG_DATA_LSB,I2C_MEMADD_SIZE_8BIT,&lsb,1,100), &healthFlags.bmp180);
    return (msb << 8) + lsb;
}

static float BMP180_CalcTrueTemp(float uncomp_temp) {
    // uint32_t rec in data sheet; prevents overflow when bitshifting and multiplying
    int32_t x1 = ((int32_t)uncomp_temp - (int32_t)calib_data.ac6) * (int32_t)calib_data.ac5 >> 15;
    int32_t x2 = ((int32_t)calib_data.mc << 11) / (x1 + calib_data.md);
    b5 = x1 + x2;
    float temp = ((b5 + 8) >> 4) / 10.0f; 
    return temp;
}

float BMP180_GetPressure(void) {
    // Write command to control register
    uint8_t pressure_cmd = BMP180_PRESSURE_CMD;
    HAL_StatusTypeDef halErrCode;
    RETURN_IF_ERROR_CODE_HAL(HAL_I2C_Mem_Write(&hi2c1, BMP180_I2C_DEVICE_ADDR, BMP180_REG_CONTROl, I2C_MEMADD_SIZE_8BIT, &pressure_cmd,1,100), &healthFlags.bmp180);
    // Delay task based on oversampling setting
    osDelay(bmp180_conv_time[BMP180_OSS]);
    // Read raw UP bits
    // Apply compensation math using intermediate temperature values
    return BMP180_CalcTruePressure(BMP180_ReadUncompPressure()); // Return final pressure value
}

static float BMP180_ReadUncompPressure(void) {
    HAL_StatusTypeDef halErrCode;
    uint8_t msb = 0x0;
    uint8_t lsb = 0x0;
    uint8_t xlsb = 0x0;
    RETURN_IF_ERROR_CODE_HAL(HAL_I2C_Mem_Read(&hi2c1,BMP180_I2C_DEVICE_ADDR,BMP180_REG_DATA_MSB,I2C_MEMADD_SIZE_8BIT,&msb,1,100), &healthFlags.bmp180);
    RETURN_IF_ERROR_CODE_HAL(HAL_I2C_Mem_Read(&hi2c1,BMP180_I2C_DEVICE_ADDR,BMP180_REG_DATA_LSB,I2C_MEMADD_SIZE_8BIT,&lsb,1,100), &healthFlags.bmp180);
    RETURN_IF_ERROR_CODE_HAL(HAL_I2C_Mem_Read(&hi2c1,BMP180_I2C_DEVICE_ADDR,BMP180_REG_DATA_XLSB,I2C_MEMADD_SIZE_8BIT,&xlsb,1,100), &healthFlags.bmp180);
    return ((msb << 16) + (lsb << 8) + xlsb) >> (8 - BMP180_OSS);
}

static float BMP180_CalcTruePressure(float uncomp_pressure) {
    // int32_t rec in data sheet; prevents overflow when bitshifting and multiplying
    int32_t b6 = b5 - 4000;
    int32_t x1 = (calib_data.b2 * (b6 * (b6 >> 12))) >> 11;
    int32_t x2 = calib_data.ac2 * (b6 >> 11);
    int32_t x3 = x1 + x2;
    int32_t b3 = ((((int32_t)calib_data.ac1 * 4 + x3) << BMP180_OSS) + 2) / 4;
    x1 = (calib_data.ac3 * (b6 >> 13));
    x2 = (calib_data.b1 * ((b6 * b6) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    uint32_t b4 = (calib_data.ac4 * (uint32_t)(x3 + 32768)) >> 15;
    uint32_t b7 = ((uint32_t)uncomp_pressure - b3) * (50000 >> BMP180_OSS);

    int32_t p;
    if (b7 < 0x80000000) {
        p = (b7 * 2) / b4;
    } else {
        p = (b7 / b4) * 2;
    }

    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    p = p + ((x1 + x2 + 3791) >> 4);
    return (float)p; // Pressure in Pa
}