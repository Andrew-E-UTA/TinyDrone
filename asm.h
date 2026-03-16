/*
 * asm.h
 *
 *  Created on: Oct 11, 2025
 *      Author: kyojin
 */

#ifndef ASM_H_
#define ASM_H_

#include <stdint.h>
#include <stdbool.h>

uint32_t* getPsp(void);
uint32_t* getMsp(void);
uint32_t* getTMPL(void);
uint32_t* getR0(void);
uint32_t  getxPsr(void);
void setAsp(void);
void setPsp(uint32_t* address);
void setTMPL(void);
void pushHwRegs(void);
void popHwRegs(void);


#endif /* ASM_H_ */
