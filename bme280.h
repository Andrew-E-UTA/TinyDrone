// BMP280 Library
// Servando Olvera

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// BME280 Device

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#ifndef BME280_H_
#define BME280_H_

#include <stdint.h>
#include <stdbool.h>

// Register Map
#define CHIP_ID_REG         0xD0
#define CTRL_HUM_REG        0xF2
#define STATUS_REG          0xF3
#define CTRL_MEAS_REG       0xF4
#define CONFIG_REG          0xF5
#define PRES_MSB_REG        0xF7
#define PRES_LSB_REG        0xF8
#define PRES_XLBS_REG       0xF9
#define TEMP_MSB_REG        0xFA
#define TEMP_LSB_REG        0xFB
#define TEMP_XLSB_REG       0xFC
#define HUM_MSB_REG         0xFD
#define HUM_LBS_REG         0xFE
#define DIG_T1_PMTR_REG     0x88
#define DIG_P1_PMTR_REG     0x8E
#define DIG_H1_PMTR_REG     0xA1
#define DIG_H2_PMTR_REG     0xE1

// Register Bit Descriptions
#define HUM_OVERSAMP_X1     0x01
#define HUM_OVERSAMP_X2     0x02
#define HUM_OVERSAMP_X4     0x03
#define HUM_OVERSAMP_X8     0x04
#define HUM_OVERSAMP_X16    0x05

#define TEMP_OVERSAMP_X1    0x20
#define TEMP_OVERSAMP_X2    0x40
#define TEMP_OVERSAMP_X4    0x60
#define TEMP_OVERSAMP_X8    0x80
#define TEMP_OVERSAMP_X16   0xA0

#define PRES_OVERSAMP_X1    0x04
#define PRES_OVERSAMP_X2    0x08
#define PRES_OVERSAMP_X4    0x0C
#define PRES_OVERSAMP_X8    0x10
#define PRES_OVERSAMP_X16   0x14

#define SLEEP_MODE          0x00
#define FORCE_MODE          0x01
#define NORMAL_MODE         0x03

#define T_STANDBY_0_5_MS    0x00
#define T_STANDBY_62_5_MS   0x20
#define T_STANDBY_125_MS    0x40
#define T_STANDBY_250_MS    0x60
#define T_STANDBY_500_MS    0x80
#define T_STANDBY_1000_MS   0xA0
#define T_STANDBY_10_MS     0xC0
#define T_STANDBY_20_MS     0xE0

#define FILTER_COEF_OFF     0x00
#define FILTER_COEF_2       0x04
#define FILTER_COEF_4       0x08
#define FILTER_COEF_8       0x0C
#define FILTER_COEF_16      0x10

// Other
#define DEVICE_ADD          0x76
#define CHIP_ID             0x60

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void getCalibrationParameters();
void initBme280();
uint32_t compensatePres(int32_t presAdc);
int32_t compensateTemp(int32_t tempAdc);
uint32_t compensateHum(int32_t humAdc);
void getBmeData(float *tempC, float *pres, float *hum);

#endif
