// Tasks
// J Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

//C-Std Lib.
#include <stdint.h>
#include <stdbool.h>

//TM4C-Core
#include "tm4c123gh6pm.h"
#include "wait.h"
#include "asm.h"

//TM4C-Peripheral drivers
#include "nvic.h"
#include "uart0.h"
#include "gpio.h"
#include "i2c1.h"
#include "spi1.h"

//Device Drivers
#include "shell.h"
#include "kernel.h"
#include "tasks.h"
#include "nrf24l01.h"
#define MPU6050_IMPLEMENTATION
#include "mpu6050.h"
#define QMC5883P_IMPLEMENTATION
#include "qmc5883p.h"
#include "bme280.h"

#define MPU6050_INT     PORTE,4
#define QMC5883P_INT    PORTB,5
#define BLUE PORTF,2

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------
void delayMs(uint16_t ms) {
    waitMicrosecond(ms*1000);
}

void initTaskHw(void) {
    initUart0();
    setUart0BaudRate(115200, 40e6);
    initI2c1();
    initSpi1(USE_SSI_RX);
    setSpi1BaudRate(4e6, 40e6);
    setSpi1Mode(0, 0);

    enablePort(PORTF);
    selectPinPushPullOutput(BLUE);
    setPinValue(BLUE, 0);

    //MPU6050 (Gyro & Accelerometer)
    mpu_init();
    enablePort(PORTE);
    selectPinDigitalInput(MPU6050_INT);
    selectPinInterruptHighLevel(MPU6050_INT);
    enablePinInterrupt(MPU6050_INT);
//    enableNvicInterrupt(INT_GPIOE);

    //QMC5883P (Magnetometer)
    qmc_init();
    enablePort(PORTB);
    selectPinDigitalInput(QMC5883P_INT);
    selectPinInterruptHighLevel(QMC5883P_INT);
    enablePinInterrupt(QMC5883P_INT);
//    enableNvicInterrupt(INT_GPIOB);

    //BME280 (Barometer)
    initBme280();

    //NRF (Wireless Transceiver)
    nrfInit();
    nrfSetTxMode(1, (uint8_t[]){0x11,0x22,0x33,0x44,0x55});
}

/* ============================================================================
 *  ESTIMATE ATTITUDE   (1kHz)          (Priority: 0)
 * ============================================================================
 *  Description: This process will read the accelerometer and gyro sensor every time step
 *  amd use Mahoney Sensor Fusion Algorithm to compute attitude estimateion
 *
 *  Control Flow:
 *      1) Read Sensors
             - accel and gyro read
             - use last mag/baro data (updates slower)
 *      2) Estimate attitude
 *      3) Correct attitude
 *      4) Post AttitudeReady
 *      5) Sleep 1ms
 *
 *  Mutexes Used:
 *      1) I2CBus           (Communicate)
 *      2) MagData          (Read)
 *      3) BaroData         (Read)
 *
 *  Semaphores Used:
 *      1) AttitudeReady    (Post)
 * */


/* ============================================================================
 * MOTOR CONTROL        (1kHz)          (Priority: 0)
 * ============================================================================
 *  Description: Runs the PID loop for the motors given the attitude setpoints
 *  and sets the pwm compare values for the esc.
 *
 *  Control Flow:
 *      1) Wait AttitudeReady
 *      2) PID Step
 *      3) Update PWMS
 *
 *  Semaphores Used:
 *      2) AttitudeReady    (Wait)
 * */


/* ============================================================================
 * MAG & BARO           (100Hz)         (Priority: 3)
 * ============================================================================
 *  Description: Read the Magnetometer and Barometer
 *
 *  Control Flow:
 *      1) Read mag & baro
 *      2) Sleep 10ms
 *
 *  Mutexes Used:
 *      1) I2CBus           (communicate)
 *      2) MagData          (write)
 *      3) BaroData         (write)
 * */


/* ============================================================================
 * RC INPUT             (50hz)          (Priority: 2)
 * ============================================================================
 *  Description: Receive Controller Input update PID setpoints
 *
 *  Control Flow:
 *      1) Read Rx Data
 *      2) Extract:
 *           - Setpoints
 *           - NeedTelemSignal
 *      3) Update Setpoints
 *      4) If Need Telem -> post TxTelem
 *      5) Sleep 20ms
 *
 *  Mutexes Used:
 *      1) Radio            (Receive Mode)
 *
 *  Semaphore Used:
 *      2) TxTelem          (Post)
 *
 * */


/* ============================================================================
 * TELEMETRY            (1hz)           (Priority: 2)
 * ============================================================================
 *  Description: Transmit battery health telemetry
 *
 *  Control Flow:
 *      1) Wait TxTelem
 *      2) Transmit Battery Health
 *      3) Sleep 1000ms
 *
 *  Mutexes Used:
 *      1) Radio            (Transmit Mode)
 *
 *  Semaphore Used:
 *      2) TxTelem          (Wait)
 * */


/* ============================================================================
 * IDLE                 (??hz)          (Priority: 7)
 * ============================================================================*/
void idle(void) {
    for(;;) yield();
}
