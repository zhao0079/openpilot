/**
 ******************************************************************************
 * @addtogroup PIOS PIOS Core hardware abstraction layer
 * @{
 * @addtogroup PIOS_SYS System Functions
 * @brief PIOS System Initialization code
 * @{
 *
 * @file       pios_sys.h  
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * 	       Parts by Thorsten Klose (tk@midibox.org)
 * @brief      System and hardware Init functions header.
 * @see        The GNU Public License (GPL) Version 3
 *
 *****************************************************************************/
/* 
 * This program is free software; you can redistribute it and/or modify 
 * it under the terms of the GNU General Public License as published by 
 * the Free Software Foundation; either version 3 of the License, or 
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY 
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License 
 * for more details.
 * 
 * You should have received a copy of the GNU General Public License along 
 * with this program; if not, write to the Free Software Foundation, Inc., 
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#ifndef PIOS_SYS_H
#define PIOS_SYS_H

/* Public structures */
/* XXX this should really come from a SoC-specific sub-header */
struct pios_clock_cfg {
	/**
	 * RCC_PLLSource_HSI_Div2 always selects HSI/2 as the PLL
	 * clock source.
	 * On Value Line devices, use RCC_PLLSource_PREDIV1 and
	 * set refclock_prescale to a value between 1 and 16.  Otherwise
	 * select one of RCC_PLLSource_HSE_Div1 or RCC_PLLSource_HSE_Div2.
	 */
	uint32_t source;

	/**
	 * The reference clock frequency, not required if
	 * source is RCC_PLLSource_HSI_Div2.
	 */
	uint32_t refclock_frequency;

	/**
	 * The desired reference clock prescaler.
	 * For Value Line devices this can be 1-16,
	 * ignored for other devices.
	 */
	uint32_t refclock_prescale;

	/** The PLL multiplier value.  One of RCC_PLLMul_* */
	uint32_t pll_multiply;

	/** Divider from SYSCLK to HCLK, a value from RCC_SYSCLK_Div* */
	uint32_t hclk_prescale;

	/** Divider from HCLK to PCLK1, a value from RCC_HCLK_Div* */
	uint32_t pclk1_prescale;

	/** Divider from HCLK to PCLK2, a value from RCC_HCLK_Div* */
	uint32_t pclk2_prescale;

	/** Divider from PCLK to ADCCLK, a value from RCC_PCLK2_Div* */
	uint32_t adc_prescale;
};

/* Public Functions */
extern void PIOS_SYS_Init(void);
extern int32_t PIOS_SYS_Reset(void);
extern uint32_t PIOS_SYS_getCPUFlashSize(void);
extern int32_t PIOS_SYS_SerialNumberGetBinary(uint8_t *array);
extern int32_t PIOS_SYS_SerialNumberGet(char *str);

#endif /* PIOS_SYS_H */

/**
  * @}
  * @}
  */
