.thumb

.text

	.def getPsp
getPsp:
	MRS R0, PSP	; Move from Special to Register
	BX LR

	.def getMsp
getMsp:
	MRS R0, MSP	; Move from Special to Register
	BX LR

	.def getR0
getR0:
	BX LR

	.def getXpsr
getXpsr:
	MRS R0, APSR	; Move from Special to Register
	BX LR

	.def setPsp
setPsp:
	MSR PSP, R0		; Set the PSP Register to whatever the value is in R0
	ISB
	BX LR

	.def setAsp
setAsp:				; Argument R0 = addr of where the psp should be
	MRS R0, CONTROL	; Move from Special to Register
	ORR R0, R0, #0x2 ; Set ASP bit ( bit 2 in CONTROL reg)
	MSR CONTROL, R0 ; Move from Register back to Special
	ISB				; Instruction Synch. Barrier (Makes sure to use the new PSP)
	BX LR

	.def setTMPL
setTMPL:
	MRS R0, CONTROL
	ORR R0, #0x1
	MSR CONTROL, R0
	ISB
	BX LR

	.def pushHwRegs
pushHwRegs:
	MRS  R0	 , PSP		 ; Set R0 to the PSP Register
	STR  R4  , [R0,#-4]!
	STR  R5  , [R0,#-4]!
	STR  R6  , [R0,#-4]!
	STR  R7  , [R0,#-4]!
	STR  R8  , [R0,#-4]!
	STR  R9  , [R0,#-4]!
	STR  R10 , [R0,#-4]!
	STR  R11 , [R0,#-4]!
	MOV  R1	 , #0xFFFD
	MOVT R1	 , #0xFFFF
	STR  R1	 , [R0,#-4]!
	VSTMDB R0!, {S0-S31}
	VMRS R1, FPSCR
	STR R1, [R0, #-4]!
	MSR  PSP , R0
	BX LR

	.def popHwRegs
popHwRegs:
	MRS R0 	, PSP
	LDR R1, [R0], #4
	VMSR FPSCR, R1
	VLDMIA R0!, {S0-S31}
	LDR R14 , [R0] , #4
	LDR R11 , [R0] , #4
	LDR R10 , [R0] , #4
	LDR R9 	, [R0] , #4
	LDR R8 	, [R0] , #4
	LDR R7 	, [R0] , #4
	LDR R6 	, [R0] , #4
	LDR R5 	, [R0] , #4
	LDR R4 	, [R0] , #4
	MSR PSP	, R0
	BX LR
