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

#ifndef SPI2_H_
#define SPI2_H_

#define USE_SS2_FSS 1		// do we want the fss, either software or hardware controlled
#define USE_SS2_RX  2		// spi receive or not receieve

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void initSpi2(uint32_t pinMask);
void setSpi2BaudRate(uint32_t clockRate, uint32_t fcyc);
void setSpi2Mode(uint8_t polarity, uint8_t phase);
void writeSpi2Data(uint32_t data);
uint32_t readSpi2Data();

#endif
