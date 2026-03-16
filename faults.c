// Shell functions
// J Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "faults.h"
#include "uart0.h"
#include "shell.h"
#include "asm.h"
#include "kernel.h"

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// REQUIRED: code this function
void mpuFaultIsr(void)
{
    char buffer[11] = {};
    Print("\nMPU fault in process ", .fg=Red, .end=' ');
    htoa(getTaskCurrent(), buffer);
    putsUart0(buffer);
    putcUart0('\n');
    putsUart0("MSP: ");
    htoa((uint32_t)getMsp(), buffer);
    putsUart0(buffer);
    putcUart0('\n');
    putsUart0("PSP: ");
    htoa((uint32_t)getPsp(), buffer);
    putsUart0(buffer);
    putcUart0('\n');
    putsUart0("MFault Flags: ");
    htoa(NVIC_FAULT_STAT_R, buffer);
    putsUart0(buffer);
    putcUart0('\n');
    putsUart0("HFault Flags: ");
    htoa(NVIC_HFAULT_STAT_R, buffer);
    putsUart0(buffer);
    putcUart0('\n');
    putsUart0("MMFAULTADDR:");
    htoa(NVIC_MM_ADDR_R, buffer);
    putsUart0(buffer);
    putcUart0('\n');
    putsUart0("Offending Instruction: ");
    htoa(*((uint32_t*)*(getPsp()+6)), buffer);
    putsUart0(buffer);
    putcUart0('\n');
    putsUart0("----==== Process Stack Dump ====----\n");

    putsUart0("R0: ");
    htoa(*(getPsp()+0), buffer);
    putsUart0(buffer);
    putcUart0('\n');
    putsUart0("R1: ");
    htoa(*(getPsp()+1), buffer);
    putsUart0(buffer);
    putcUart0('\n');
    putsUart0("R2: ");
    htoa(*(getPsp()+2), buffer);
    putsUart0(buffer);
    putcUart0('\n');
    putsUart0("R3: ");
    htoa(*(getPsp()+3), buffer);
    putsUart0(buffer);
    putcUart0('\n');
    putsUart0("R12: ");
    htoa(*(getPsp()+4), buffer);
    putsUart0(buffer);
    putcUart0('\n');
    putsUart0("LR: ");
    htoa(*(getPsp()+5), buffer);
    putsUart0(buffer);
    putcUart0('\n');
    putsUart0("PC: ");
    htoa(*(getPsp()+6), buffer);
    putsUart0(buffer);
    putcUart0('\n');
    putsUart0("xPSR: ");
    htoa(*(getPsp()+7), buffer);
    putsUart0(buffer);
    putcUart0('\n');

    for(;;);
}

// REQUIRED: code this function
void hardFaultIsr(void)
{
    char buffer[11] = {};
    putcUart0('\n');
    Print("\nHard fault in process ", .fg=Red, .end=' ');
    htoa(getTaskCurrent(), buffer);
    putsUart0(buffer);
    putcUart0('\n');
    putsUart0("MSP: ");
    htoa((uint32_t)getMsp(), buffer);
    putsUart0(buffer);
    putcUart0('\n');
    putsUart0("PSP: ");
    htoa((uint32_t)getPsp(), buffer);
    putsUart0(buffer);
    putcUart0('\n');
    putsUart0("MFault Flags: ");
    htoa(NVIC_FAULT_STAT_R, buffer);
    putsUart0(buffer);
    putcUart0('\n');
    putsUart0("HFault Flags: ");
    htoa(NVIC_HFAULT_STAT_R, buffer);
    putsUart0(buffer);
    putcUart0('\n');
    putsUart0("Offending Instruction: ");
    htoa(*((uint32_t*)*(getPsp()+6)), buffer);
    putsUart0(buffer);
    putcUart0('\n');
    putsUart0("----==== Process Stack Dump ====----\n");

    putsUart0("R0: ");
    htoa(*(getPsp()+0), buffer);
    putsUart0(buffer);
    putcUart0('\n');
    putsUart0("R1: ");
    htoa(*(getPsp()+1), buffer);
    putsUart0(buffer);
    putcUart0('\n');
    putsUart0("R2: ");
    htoa(*(getPsp()+2), buffer);
    putsUart0(buffer);
    putcUart0('\n');
    putsUart0("R3: ");
    htoa(*(getPsp()+3), buffer);
    putsUart0(buffer);
    putcUart0('\n');
    putsUart0("R12: ");
    htoa(*(getPsp()+4), buffer);
    putsUart0(buffer);
    putcUart0('\n');
    putsUart0("LR: ");
    htoa(*(getPsp()+5), buffer);
    putsUart0(buffer);
    putcUart0('\n');
    putsUart0("PC: ");
    htoa(*(getPsp()+6), buffer);
    putsUart0(buffer);
    putcUart0('\n');
    putsUart0("xPSR: ");
    htoa(*(getPsp()+7), buffer);
    putsUart0(buffer);
    putcUart0('\n');

    for(;;);
}

// REQUIRED: code this function
void busFaultIsr(void)
{
    char buffer[11] = {};
    Print("\nBus fault in process ", .fg=Red, .end=' ');
    htoa(getTaskCurrent(), buffer);
    putsUart0(buffer);

    for(;;);
}

// REQUIRED: code this function
void usageFaultIsr(void)
{
    char buffer[11] = {};
    Print("\nUsage fault in process ", .fg=Red, .end=' ');
    htoa(getTaskCurrent(), buffer);
    putsUart0(buffer);

    for(;;);
}

