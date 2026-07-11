// spi2.h
// Andrew Espinoza

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL
// Target uC:       TM4C123GH6PM
// System Clock:    80 Mhz

// Hardware configuration:
// SPI2 Interface:
//   SCLK on PB4 (SSI2Clk)
//   ~CS  on PB5 (SSI2Fss)
//   MISO on PB6 (SSI2Rx)
//   MOSI on PB7 (SSI2Tx)

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "spi2.h"
#include "gpio.h"

// Pins
#define SSI2CLK PORTB,4     // Clock
#define SSI2FSS PORTB,5     // Frame/chip select
#define SSI2RX  PORTB,6     // Receive
#define SSI2TX  PORTB,7		// Transmit

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize SPI2
void initSpi2(uint32_t pinMask)
{
    // Enable clocks
    SYSCTL_RCGCSSI_R |= SYSCTL_RCGCSSI_R2;
    _delay_cycles(3);
    enablePort(PORTB);

    // Configure SSI2 pins for SPI configuration
    selectPinPushPullOutput(SSI2TX);				// sets transmit pin to be output
    setPinAuxFunction(SSI2TX, GPIO_PCTL_PB7_SSI2TX);		// set as spi - aux function
    selectPinPushPullOutput(SSI2CLK);				// sets clock pin to be outout
    setPinAuxFunction(SSI2CLK, GPIO_PCTL_PB4_SSI2CLK);		// sets as clock - aux function
    selectPinPushPullOutput(SSI2FSS);				// frame select is output
    if (pinMask & USE_SS2_FSS)					// is the hardware controlling fss
    {
        setPinAuxFunction(SSI2FSS, GPIO_PCTL_PB5_SSI2FSS);	// set fss to be an aux function
    }
    if (pinMask & USE_SS2_RX)					// is there a receiev pin
    {
        selectPinDigitalInput(SSI2RX);				// if so, set it to be an input
        setPinAuxFunction(SSI2RX, GPIO_PCTL_PB6_SSI2RX);	// receive pin is aux function
    }

    // Configure the SSI2 as a SPI master, mode 3, 8bit operation
    SSI2_CR1_R &= ~SSI_CR1_SSE;                        // turn off SSI2 to allow re-configuration
    SSI2_CR1_R = 0;                                    // select master mode in control reg, 3rd bit
    SSI2_CC_R = 0;                                     // select system clock as the clock source
						       // set to 0x05 for PIOSC (precision oscillator)
    SSI2_CR0_R = SSI_CR0_FRF_MOTO | SSI_CR0_DSS_8;     // set SR=0, 8-bit
						       // set Freescale SPI Frame Format &
						       // to 8 bit SSI Data Size 
}

// Set baud rate as function of instruction cycle frequency
void setSpi2BaudRate(uint32_t baudRate, uint32_t fcyc)
{
    uint32_t divisorTimes2 = (fcyc * 2) / baudRate;    // calculate divisor (r) times 2
    SSI2_CR1_R &= ~SSI_CR1_SSE;                        // turn off SSI2 to allow re-configuration
    SSI2_CPSR_R = (divisorTimes2 + 1) >> 1;            // round divisor to nearest integer
    SSI2_CR1_R |= SSI_CR1_SSE;                         // turn on SSI2
}

// Set mode
void setSpi2Mode(uint8_t polarity, uint8_t phase)
{
    SSI2_CR1_R &= ~SSI_CR1_SSE;                        // turn off SSI2 to allow re-configuration
    SSI2_CR0_R &= ~(SSI_CR0_SPH | SSI_CR0_SPO);        // set SPO and SPH as appropriate
						       // set as first edge capture, SPh = 0
						       // set as  steady state Low for clock, SPO = 0
    if (polarity)				// if the polarity is set, set SPO
    {
        SSI2_CR0_R |= SSI_CR0_SPO;		// set SPO = 1
        enablePinPullup(SSI2CLK);		// clock must be pullup if SPO = 1
    }
    else					// if polarity is off then disable pull up to save power
        disablePinPullup(SSI2CLK);
    if (phase)					// if phase has been set, set SPH
        SSI2_CR0_R |= SSI_CR0_SPH;

    SSI2_CR1_R |= SSI_CR1_SSE;                         // turn on SSI2
}

// Blocking function that writes data and waits until the tx buffer is empty
void writeSpi2Data(uint32_t data)
{
    SSI2_DR_R = data;
    while (SSI2_SR_R & SSI_SR_BSY);
}

// Reads data from the rx buffer after a write
uint32_t readSpi2Data()
{
    return SSI2_DR_R;
}
