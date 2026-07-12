// NVIC Library
// Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL
// Target uC:       TM4C123GH6PM
// System Clock:    -

// Hardware configuration: -

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#ifndef NVIC_H_
#define NVIC_H_

#include <stdint.h>

#define PIN_0           0x01
#define PIN_1           0x02
#define PIN_2           0x04
#define PIN_3           0x08
#define PIN_4           0x10
#define PIN_5           0x20
#define PIN_6           0x40
#define PIN_7           0x80

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void enableNvicInterrupt(uint8_t vectorNumber);
void disableNvicInterrupt(uint8_t vectorNumber);
void setNvicInterruptPriority(uint8_t vectorNumber, uint8_t priority);

#endif
