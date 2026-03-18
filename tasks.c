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
 */

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
#define QUATERNION_IMPLEMENTATION
#include "quaternion.h"
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

    //Timer for integrating gyro
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R0;
    _delay_cycles(3);

    WTIMER0_CTL_R  &= ~(TIMER_CTL_TAEN);
    WTIMER0_CFG_R   = 0x4;
    WTIMER0_TAMR_R  = TIMER_TAMR_TAMR_1_SHOT | TIMER_TAMR_TACDIR;
    WTIMER0_TAV_R   = 0;
}

#define SECONDS_PER_COUNT   0.000000025
float delta(void) {
    static uint32_t count_now = 0.0, count_last= 0.0;
    count_last = count_now;
    count_now = WTIMER0_TAV_R;
    WTIMER0_TAV_R = 0;
    return (count_now - count_last) * SECONDS_PER_COUNT;
}

/* ============================================================================
 *  ESTIMATE ATTITUDE   (1kHz)          (Priority: 0)
 * ============================================================================
 *  Description: This process will read the accelerometer and gyro sensor every
 *  time step amd use Madgewick Sensor Fusion Algorithm to compute attitude
 *  estimation
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

#define GYRO_MSE    0.1
#define PI          3.141592
#define RAD2DEG     180.0/PI
void task_estimate_attitude(void) {
    MpuData m;
    Quaternion q_est, q_prev, q_accel, q_gyro, q_grad;
    float dt = 0.0;
    float f[3] = {0.0};//Objective Function
    float j[3][4] = {0.0};//Jacobian
    float beta = -sqrtf(3.0f / 4.0f) * GYRO_MSE;

    WTIMER0_CTL_R |= TIMER_CTL_TAEN;
    for(;;) {
        //Setup
        dt = delta();
        m = mpu_read();
        q_prev = q_est;

        //Estimate
        q_accel = (Quaternion){.w=0, .x=m.a.x, .y=m.a.y, .z=m.a.z};
        q_accel = quaternion_normalize(q_accel);
        q_gyro  = (Quaternion){.w=0, .x=m.g.x*.5, .y=m.g.y*.5, .z=m.g.z*.5};
        q_gyro = quaternion_hamilton(q_prev, q_gyro);

        //Objective Function
        f[0] = 2*(q_prev.x*q_prev.z - q_prev.w*q_prev.y) - q_accel.x;
        f[1] = 2*(q_prev.w*q_prev.x + q_prev.y*q_prev.z) - q_accel.y;
        f[2] = 2*(0.5 - q_prev.x*q_prev.x - q_prev.y*q_prev.y) - q_accel.z;

        //Jacobian
        j[0][0] = -2 * q_prev.y;
        j[0][1] =  2 * q_prev.z;
        j[0][2] = -2 * q_prev.w;
        j[0][3] =  2 * q_prev.x;

        j[1][0] =  2 * q_prev.x;
        j[1][1] =  2 * q_prev.w;
        j[1][2] =  2 * q_prev.z;
        j[1][3] =  2 * q_prev.y;

        j[2][0] =  0;
        j[2][1] = -4 * q_prev.x;
        j[2][2] = -4 * q_prev.y;
        j[2][3] =  0;

        //Gradiant (J' * F)
        q_grad.w = j[0][0]*f[0] + j[1][0]*f[1] + j[2][0]*f[2];
        q_grad.x = j[0][1]*f[0] + j[1][1]*f[1] + j[2][1]*f[2];
        q_grad.y = j[0][2]*f[0] + j[1][2]*f[1] + j[2][2]*f[2];
        q_grad.z = j[0][3]*f[0] + j[1][3]*f[1] + j[2][3]*f[2];
        q_grad = quaternion_normalize(q_grad);
        q_grad = quaternion_scalar(q_grad, beta);

        //TODO:Incorporate the Magnetometer data to compensate yaw

        //Fuse
        q_est = quaternion_add(q_gyro, q_grad);

        //Integrate
        q_est = quaternion_scalar(q_est, dt);

        //Apply
        q_est = quaternion_add(q_prev, q_est);
        q_est = quaternion_normalize(q_est);

        //Convert to attitude
        Attitude a = (Attitude) {
            .pitch= RAD2DEG * -asinf(2*q_est.x*q_est.z + 2*q_est.w*q_est.y),
            .roll=  RAD2DEG * atan2f(2*q_est.y*q_est.z - 2*q_est.w*q_est.x, 2*q_est.w*q_est.w - 2*q_est.x*q_est.x - 1),
            .yaw=   RAD2DEG * atan2f(2*q_est.x*q_est.y - 2*q_est.w*q_est.z, 2*q_est.w*q_est.w - 2*q_est.x*q_est.x - 1)

        };
        lock(mutex_attitude);
        global_write(mutex_attitude, &a);
        unlock(mutex_attitude);
        post(semaphore_attitude_ready);
        sleep(1);
    }
}

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
void task_control_motors(void) {
    for(;;) {

    }
}

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
void task_read_slow_sensors(void) {
    for(;;) {

    }
}


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
void task_receive_input(void) {
    for(;;) {

    }
}


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
void task_send_telem(void) {
    for(;;) {

    }
}


/* ============================================================================
 * IDLE                 (??hz)          (Priority: 7)
 * ============================================================================*/
void idle(void) {
    for(;;) yield();
}
