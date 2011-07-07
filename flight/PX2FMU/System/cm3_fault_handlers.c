/*
 * cm3_fault_handlers.c
 *
 *  Created on: Apr 24, 2011
 *      Author: msmith
 */

#include <stdint.h>
#include "dcc_stdio.h"
#include "stm32f2xx.h"

#define FAULT_TRAMPOLINE(_vec)										\
__attribute__((naked))												\
void																\
_vec##_Handler(void)												\
{																	\
     __asm(  ".syntax unified\n"									\
                     "MOVS   R0, #4  \n"							\
                     "MOV    R1, LR  \n"							\
                     "TST    R0, R1  \n"							\
                     "BEQ    1f    \n"								\
                     "MRS    R0, PSP \n"							\
                     "B      " #_vec "_Handler2      \n"			\
             "1:  \n"												\
                     "MRS    R0, MSP \n"							\
                     "B      " #_vec "_Handler2      \n"			\
             ".syntax divided\n");									\
}																	\
struct hack

struct cm3_frame {
	uint32_t	r0;
	uint32_t	r1;
	uint32_t	r2;
	uint32_t	r3;
	uint32_t	r12;
	uint32_t	lr;
	uint32_t	pc;
	uint32_t	psr;
};

FAULT_TRAMPOLINE(HardFault);
FAULT_TRAMPOLINE(BusFault);
FAULT_TRAMPOLINE(UsageFault);

/* this is a hackaround to avoid an issue where dereferencing SCB seems to result in bad codegen and a link error */
#define SCB_REG(_reg)	(*(uint32_t *)&(SCB->_reg))

void
HardFault_Handler2(struct cm3_frame *frame)
{
	dbg_write_str("\nHARD FAULT");
	dbg_write_hex32(frame->pc);
	dbg_write_char('\n');
	dbg_write_hex32(SCB_REG(HFSR));
	dbg_write_char('\n');
	for (;;);
}

void
BusFault_Handler2(struct cm3_frame *frame)
{
	dbg_write_str("\nBUS FAULT @ 0x");
	dbg_write_hex32(frame->pc);
	dbg_write_str("  CFSR 0x");
	dbg_write_hex32(SCB_REG(CFSR));
	dbg_write_str("  BFAR 0x");
	dbg_write_hex32(SCB_REG(BFAR));
	for (;;);
}

void
UsageFault_Handler2(struct cm3_frame *frame)
{
	dbg_write_str("\nUSAGE FAULT @ 0x");
	dbg_write_hex32(frame->pc);
	dbg_write_str("  CFSR 0x");
	dbg_write_hex32(SCB_REG(CFSR));
	for (;;);
}

