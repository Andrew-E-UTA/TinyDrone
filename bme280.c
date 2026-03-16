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

// Calibration Parameters
uint16_t digP1;
int16_t digP2, digP3, digP4, digP5, digP6, digP7, digP8, digP9;

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void getCalibrationParameters() {
    uint8_t digP[18];

    // Read registers with calibration parameters
    readI2c1Registers(DEVICE_ADD, DIG_P1_PMTR_REG, digP, 18);

    // Form parameters accordingly for pressure
    digP1 = (digP[1] << 8) | digP[0];
    digP2 = (digP[3] << 8) | digP[2];
    digP3 = (digP[5] << 8) | digP[4];
    digP4 = (digP[7] << 8) | digP[6];
    digP5 = (digP[9] << 8) | digP[8];
    digP6 = (digP[11] << 8) | digP[10];
    digP7 = (digP[13] << 8) | digP[12];
    digP8 = (digP[15] << 8) | digP[14];
    digP9 = (digP[17] << 8) | digP[16];
}

void initBme280() {
    uint8_t chipId;

    // Ensure Device Address is the correct one
    if(!pollI2c1Address(DEVICE_ADD)) {
        putsUart0("Device NOT Responsive\n");
        return;
    }

    // Ensure chip ID is the expected one
    chipId = readI2c1Register(DEVICE_ADD, CHIP_ID_REG);
    if(chipId != CHIP_ID) {
        putsUart0("Wrong ID Device\n");
        return;
    }

    // Configure Device Registers
    writeI2c1Register(DEVICE_ADD, CTRL_HUM_REG, HUM_OVERSAMP_X1);
    writeI2c1Register(DEVICE_ADD, CTRL_MEAS_REG, (PRES_OVERSAMP_X1 | NORMAL_MODE));
    writeI2c1Register(DEVICE_ADD, CONFIG_REG, (T_STANDBY_1000_MS | FILTER_COEF_OFF));

    // Obtain calibration parameters after configuring device
    getCalibrationParameters();
}

//------------------------------------------------------------------------------------------------------
// Functions below were sourced from BME280 Datasheet, Section 4.2.3
// Manufacturer: Bosch Sensortec
// URL: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf
//------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------
// Functions above were sourced from BME280 Datasheet, Section 4.2.3
// Manufacturer: Bosch Sensortec
// URL: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf
//------------------------------------------------------------------------------------------------------

void getBmeData(float *tempC, float *pres, float *hum) {
    uint8_t presRaw[3];

    int32_t presAdc;

    // Read raw temperature, pressure & humidity values (up to 20-bits in length)
    readI2c1Registers(DEVICE_ADD, PRES_MSB_REG, presRaw, 3);

    // Form ADC values respectively
    presAdc = (presRaw[0] << 12) | (presRaw[1] << 4) | (presRaw[2] >> 4);

    // Use compensation formulas from data sheet to convert values to useful values
    //TODO: return fixed point pressure
}


