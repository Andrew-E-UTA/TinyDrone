// BME280 Library
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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "gpio.h"
#include "clock.h"
#include "wait.h"
#include "uart0.h"
#include "i2c1.h"
#include "bme280.h"

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

BmeCalibrationParams getCalibrationParameters() {
    uint8_t digT[6];
    uint8_t digP[18];
    uint8_t digH[8];
    BmeCalibrationParams params;

    // Read registers with calibration parameters
    readI2c1Registers(DEVICE_ADD, DIG_T1_PMTR_REG, digT, 6);
    readI2c1Registers(DEVICE_ADD, DIG_P1_PMTR_REG, digP, 18);
    readI2c1Registers(DEVICE_ADD, DIG_H1_PMTR_REG, &digH[0], 1);
    readI2c1Registers(DEVICE_ADD, DIG_H2_PMTR_REG, &digH[1], 7);

    // Form parameters accordingly for pressure
    params.digP1 = (digP[1] << 8) | digP[0];
    params.digP2 = (digP[3] << 8) | digP[2];
    params.digP3 = (digP[5] << 8) | digP[4];
    params.digP4 = (digP[7] << 8) | digP[6];
    params.digP5 = (digP[9] << 8) | digP[8];
    params.digP6 = (digP[11] << 8) | digP[10];
    params.digP7 = (digP[13] << 8) | digP[12];
    params.digP8 = (digP[15] << 8) | digP[14];
    params.digP9 = (digP[17] << 8) | digP[16];

    // Form parameters accordingly for humidity
    params.digH1 = digH[0];
    params.digH2 = (digH[2] << 8) | digH[1];
    params.digH3 = digH[3];
    params.digH4 = (digH[4] << 4) | (digH[5] & 0x0F);
    params.digH5 = (digH[6] << 4) | (digH[5] >> 4);
    params.digH6 = digH[7];

    return params;
}

BmeCalibrationParams initBme280() {
    uint8_t chipId;

    // Ensure Device Address is the correct one
    if(!pollI2c1Address(DEVICE_ADD)) {
        putsUart0("Device NOT Responsive\n");
        return (BmeCalibrationParams){};
    }

    // Ensure chip ID is the expected one
    chipId = readI2c1Register(DEVICE_ADD, CHIP_ID_REG);
    if(chipId != CHIP_ID) {
        putsUart0("Wrong ID Device\n");
        return (BmeCalibrationParams){};
    }

    // Configure Device Registers
    writeI2c1Register(DEVICE_ADD, CTRL_HUM_REG, HUM_OVERSAMP_X1);
    writeI2c1Register(DEVICE_ADD, CTRL_MEAS_REG, (PRES_OVERSAMP_X1 | NORMAL_MODE));
    writeI2c1Register(DEVICE_ADD, CONFIG_REG, (T_STANDBY_1000_MS | FILTER_COEF_OFF));

    // Obtain calibration parameters after configuring device
    return getCalibrationParameters();
}

//------------------------------------------------------------------------------------------------------
// Functions below were sourced from BME280 Datasheet, Section 4.2.3
// Manufacturer: Bosch Sensortec
// URL: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf
//------------------------------------------------------------------------------------------------------

// Compute the intermediate value
static float getTFine(int32_t tempAdc, const BmeCalibrationParams *params) {
    float var1 = ((float)tempAdc) / 16384.0f - ((float)params->digT1) / 1024.0f;
    float var2 = (((float)tempAdc) / 131072.0f - ((float)params->digT1) / 8192.0f) * ((float)params->digT2);
    return var1 * ((float)params->digT3) + var2;
}

// Compensated temperature in °C
static float compensateTemp(float t_fine) {
    return t_fine / 5120.0f;
}

// Compensated pressure in Pa
static float compensatePres(int32_t presAdc, float t_fine, const BmeCalibrationParams *params) {
    float var1 = (t_fine / 2.0f) - 64000.0f;
    float var2 = var1 * var1 * ((float)params->digP6) / 32768.0f;
    var2 = var2 + var1 * ((float)params->digP5) * 2.0f;
    var2 = (var2 / 4.0f) + (((float)params->digP4) * 65536.0f);
    var1 = (((float)params->digP3) * var1 * var1 / 524288.0f + ((float)params->digP2) * var1) / 524288.0f;
    var1 = (1.0f + var1 / 32768.0f) * ((float)params->digP1);
    if (var1 == 0.0f) return 0.0f;

    float p = 1048576.0f - (float)presAdc;
    p = (p - var2 / 4096.0f) * 6250.0f / var1;
    var1 = ((float)params->digP9) * p * p / 2147483648.0f;
    var2 = p * ((float)params->digP8) / 32768.0f;
    p = p + (var1 + var2 + ((float)params->digP7)) / 16.0f;

    return p; // pressure in Pa
}

// Compensated relative humidity in %
static float compensateHum(int32_t humAdc, float t_fine, const BmeCalibrationParams *params) {
    float var1 = t_fine - 76800.0f;
    float var2 = ((float)params->digH4) * 64.0f + ((float)params->digH5) / 16384.0f * var1;
    float var3 = (float)humAdc - var2;
    float var4 = ((float)params->digH2) / 65536.0f * (1.0f + ((float)params->digH6) / 67108864.0f * var1 * (1.0f + ((float)params->digH3) / 67108864.0f * var1));
    float var5 = 1.0f + ((float)params->digH3) / 67108864.0f * var1;
    float var6 = var3 * var4 / (var5 * 32768.0f);
    float hum = var6 * (1.0f - ((float)params->digH1) * var6 / 32768.0f);

    if (hum > 100.0f) hum = 100.0f;
    if (hum < 0.0f)   hum = 0.0f;
    return hum;
}

//------------------------------------------------------------------------------------------------------
// Functions above were sourced from BME280 Datasheet, Section 4.2.3
// Manufacturer: Bosch Sensortec
// URL: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf
//------------------------------------------------------------------------------------------------------

void getBmeData(float *tempC, float *pres, float *hum, BmeCalibrationParams *params) {
    uint8_t presRaw[3];
    uint8_t tempRaw[3];
    uint8_t humRaw[2];

    // Read raw sensor data
    readI2c1Registers(DEVICE_ADD, PRES_MSB_REG, presRaw, 3);
    readI2c1Registers(DEVICE_ADD, TEMP_MSB_REG, tempRaw, 3);
    readI2c1Registers(DEVICE_ADD, HUM_MSB_REG, humRaw, 2);

    // Form 20-bit ADC values
    int32_t presAdc = (presRaw[0] << 12) | (presRaw[1] << 4) | (presRaw[2] >> 4);
    int32_t tempAdc = (tempRaw[0] << 12) | (tempRaw[1] << 4) | (tempRaw[2] >> 4);
    int32_t humAdc  = (humRaw[0] << 8) | humRaw[1];

    // Compute t_fine from temperature ADC
    float t_fine = getTFine(tempAdc, params);

    // Convert to physical values
    *tempC = compensateTemp(t_fine);                            // °C
    *pres  = compensatePres(presAdc, t_fine, params) / 100.0f;  // hPa (Pa / 100)
    *hum   = compensateHum(humAdc, t_fine, params);             // %RH
}

