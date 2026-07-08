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

/*
 * Sources:
 *  Mathematical Model of an IMU - nitinjsanket:
 *      https://nitinjsanket.github.io/tutorials/attitudeest/madgwick#madgwickfilt
 *
 *  Madgwick_Filter - bjohnsonfl:
 *      https://github.com/bjohnsonfl/Madgwick_Filter/blob/master/madgwickFilter.c
 *
 *
 *  Notes:
 *      With the clock at 80Mhz, trying to run the ahrs and pid both at 1kHz, that leaves 80k total clocks in 1kHz
 *
 *
 */

//C-Std Lib.
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>

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
#include "mpu6050.h"
#include "quaternion.h"
#include "qmc5883p.h"
#include "bme280.h"

#define MPU6050_INT     PORTE,4
#define QMC5883P_INT    PORTB,5
#define NRF24L01_INT    PORTB,5
#define NRF24L01_CSN    PORTD,1
#define NRF24L01_CE     PORTE,3

//Drone Specific Types
typedef struct { float pitch, roll, yaw; } Attitude;
typedef struct { float x, y, z; } MagData;
typedef struct { float x, y, z; } BaroData;

Attitude attitude;
MagData  mag_data;
BaroData baro_data;

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void gpioBIsr(void) {

}

void gpioEIsr(void) {

}

void initTaskHw(void) {

    //COMM PROTOCOLS (uart0, sp1, i2c1)
    initUart0();
    setUart0BaudRate(115200, 80e6);
    initI2c1();
    initSpi1(USE_SSI_RX);
    setSpi1BaudRate(4e6, 80e6);
    setSpi1Mode(0, 0);

    //GPIO (mpu, qmc, nrf interrupts)
    enablePort(PORTB);
    enablePort(PORTD);
    enablePort(PORTE);

    selectPinDigitalInput(QMC5883P_INT);
    selectPinDigitalInput(MPU6050_INT);
    selectPinDigitalInput(NRF24L01_INT);
    selectPinPushPullOutput(NRF24L01_CSN);
    selectPinPushPullOutput(NRF24L01_CE);

    selectPinInterruptHighLevel(MPU6050_INT);
    selectPinInterruptHighLevel(QMC5883P_INT);
    selectPinInterruptFallingEdge(NRF24L01_INT);

    enablePinInterrupt(MPU6050_INT);
    enablePinInterrupt(QMC5883P_INT);
    enablePinInterrupt(NRF24L01_INT);

    enableNvicInterrupt(INT_GPIOE);
    enableNvicInterrupt(INT_GPIOB);


    //MODULES
    qmc_init();
    mpu_init();
    initBme280();
    nrfInit();
    nrfSetTxMode(1, (uint8_t[]){0x11,0x22,0x33,0x44,0x55});
}

#define COUNT2SEC .0000000125 //80MHz
float deltaSeconds(void) {
    uint32_t counts = WTIMER0_TAV_R;
    WTIMER0_TAV_R = 0;
    return ((float)counts) * COUNT2SEC;
}

Vec3f gyroOffsets(void) {
    uint32_t i;
    MpuData m;
    Vec3f s = {0,0,0};
    for(i = 0; i < 1000; ++i) {
        m = mpu_read();
        s.x += m.g.x;
        s.y += m.g.y;
        s.z += m.g.z;
        waitMicrosecond(2e3);
    }
    s.x /= 1000;
    s.y /= 1000;
    s.z /= 1000;
    return s;
}

/* ============================================================================
 *  ESTIMATE ATTITUDE   (1kHz)          (Priority: 0)
 * ============================================================================
 *  Description: This process will read the sensors, update attitude, and run the pid loop for the motors
 *
 *  Control Flow:
 *      Wait on Mpu data ready semaphore (yields to other tasks while waiting), that is posted by interrupt
 *      reads mpu data
 *      polls qmc data ready (no interrupt)
 *      if qmc -> read qmc & marg ahrs
 *      else -> imu_ahrs
 *      pid update (with updated attitude and current setpoint)
 *      send motor control
 *
 *  Notes:
 *      no sleeping as the wait on data will provide the other tasks with time to run
 *      the mpu thus governs the speed in which this task runs (mpu6050 set at 1khz datarate)
 * */

#define GYRO_MSE    0.1
#define PI          3.141592
#define RAD2DEG     180.0/PI
#define WINDOW_SIZE 4
void task_ahrs_pid(void) {
    float mag_A[3][3]   =  {{0.00861610, 0.00195565, 0.00083649},
                           {0.00195565, 10.99483483, -0.22468925},
                           {0.00083649, -0.22468925, 10.5611354}};
    float mag_b [3]     =  {178.46403898, -0.03880254, -0.03374748};
    float accel_A[3][3] =  {{1.00550042, -0.00211106, -0.00102880},
                           {-0.00211106, 1.00552839, 0.00102497},
                           {-0.00102880, 0.00102497, 0.98906820}};
    float accel_b[3]    =  {0.02526037, 0.00277766, 0.01647459};
    Vec3f g_ofs = gyroOffsets();
    Vec3f window_a[WINDOW_SIZE] = {};
    Vec3f window_g[WINDOW_SIZE] = {};
    Vec3f window_m[WINDOW_SIZE] = {};
    uint8_t idx_a = 0, idx_g = 0, idx_m = 0;
    float dt_marg_s = 0.0f;
    Quaternion q_est = {1.0f, 0.0f, 0.0f, 0.0f};

    for(;;) {
    }
}


/* ============================================================================
 * RC INPUT             (50hz)          (Priority: 2)
 * ============================================================================
 *  Description: Receive Controller Input update PID setpoints
 *
 *  Control Flow:
 *      Sleep 20ms (50hz)
 *      read from nrf
 *
 * */
void task_receive_input(void) {
    for(;;) {

    }
}


/* ============================================================================
 * TELEMETRY            (1hz)           (Priority: 2)
 * ============================================================================
 *  Description: Transmit battery health telemetry
 *
 * */
void task_send_telem(void) {
    for(;;) {

    }
}


/* ============================================================================
 * IDLE                 (??hz)          (Priority: 7)
 * ============================================================================
*  Description: Do nothing task
*
*/
void idle(void) {
    for(;;) yield();
}
