#ifndef PIOS_BOARD_H_
#define PIOS_BOARD_H_

#ifdef USE_STM32103CB_AHRS
#include "STM32103CB_AHRS.h"
#elif USE_STM3210E_OP
#include "STM3210E_OP.h"
#elif USE_STM32103CB_PIPXTREME
#include "STM32103CB_PIPXTREME_Rev1.h"
#elif USE_STM32103CB_CC_Rev1
#include "STM32103CB_CC_Rev1.h"
#elif USE_STM3210E_INS
#include "STM3210E_INS.h"
#elif USE_STM32F100C6T6B_PX2IO_Rev1
#include "STM32100C6_PX2IO_Rev1.h"
#elif USE_STM32205_PX2FMU_Rev1
#include "STM32205_PX2FMU_Rev1.h"
#else
#error Board definition has not been provided.
#endif

#endif /* PIOS_BOARD_H_ */
