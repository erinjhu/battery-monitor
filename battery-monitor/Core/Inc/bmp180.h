#ifndef BMP180_H
#define BMP180_H

#include <stdint.h>

// EEPROM register addresses for calibration data
#define BMP180_REG_AC1_MSB  0xAA
#define BMP180_REG_AC1_LSB  0xAB
#define BMP180_REG_AC2_MSB  0xAC
#define BMP180_REG_AC2_LSB  0xAD
#define BMP180_REG_AC3_MSB  0xAE
#define BMP180_REG_AC3_LSB  0xAF
#define BMP180_REG_AC4_MSB  0xB0
#define BMP180_REG_AC4_LSB  0xB1
#define BMP180_REG_AC5_MSB  0xB2
#define BMP180_REG_AC5_LSB  0xB3
#define BMP180_REG_AC6_MSB  0xB4
#define BMP180_REG_AC6_LSB  0xB5
#define BMP180_REG_B1_MSB   0xB6
#define BMP180_REG_B1_LSB   0xB7
#define BMP180_REG_B2_MSB   0xB8
#define BMP180_REG_B2_LSB   0xB9
#define BMP180_REG_MB_MSB   0xBA
#define BMP180_REG_MB_LSB   0xBB
#define BMP180_REG_MC_MSB   0xBC
#define BMP180_REG_MC_LSB   0xBD
#define BMP180_REG_MD_MSB   0xBE
#define BMP180_REG_MD_LSB   0xBF

// Control register
#define BMP180_REG_CONTROl      0xF4
#define BMP180_TEMP_CMD         0x2E
#define BMP180_PRESSURE_CMD     (int)(0x34 + (BMP180_OSS << 6))

// I2C addresses
#define BMP180_I2C_READ_ADDR 0xEF
#define BMP180_I2C_WRITE_ADDR 0xEE
#define BMP180_I2C_DEVICE_ADDR 0xEE

// Temperature/pressure data registers
#define BMP180_REG_DATA_MSB 0xF6
#define BMP180_REG_DATA_LSB 0xF7
#define BMP180_REG_DATA_XLSB 0xF8


// Pressure OSS
#define BMP180_OSS          0x1 // manually change this





// Struct to hold the 11 calibration variables
typedef struct {
    int16_t  ac1;
    int16_t  ac2;
    int16_t  ac3;
    uint16_t ac4;  // Unsigned
    uint16_t ac5;  // Unsigned
    uint16_t ac6;  // Unsigned
    int16_t  b1;
    int16_t  b2;
    int16_t  mb;
    int16_t  mc;
    int16_t  md;
} BMP180_CalibData_t;

void BMP180_Init(void);
void BMP180_ReadCalibrationData(void);
float BMP180_GetTemperature(void);
float BMP180_GetPressure(void);

#endif // BMP180_H