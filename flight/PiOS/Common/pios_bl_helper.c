/**
 ******************************************************************************
 * @addtogroup PIOS PIOS Core hardware abstraction layer
 * @{
 * @addtogroup PIOS_BOOTLOADER Functions
 * @brief HAL code to interface to the OpenPilot AHRS module
 * @{
 *
 * @file       pios_bl_helper.c
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @brief      Bootloader Helper Functions
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

/* Project Includes */
#include "pios.h"
#if defined(PIOS_INCLUDE_BL_HELPER)
#if defined(STM32F2XX)
#include "stm32f2xx_flash.h"
#define FALSE	0
#define TRUE	1
#else
#include <pios_board_info.h>
#include "stm32f10x_flash.h"
#endif
#endif

uint8_t *PIOS_BL_HELPER_FLASH_If_Read(uint32_t SectorAddress)
{
	return (uint8_t *) (SectorAddress);
}

#if defined(PIOS_INCLUDE_BL_HELPER_WRITE_SUPPORT)
uint8_t PIOS_BL_HELPER_FLASH_Ini()
{
	FLASH_Unlock();
	return 1;
}

#if defined(STM32F2XX)
static struct {
	uint16_t	sectorCode;
	uint32_t	baseAddress;
} _flash_sectors[] = {
		{ FLASH_Sector_0,  0x08000000 },
		{ FLASH_Sector_1,  0x08004000 },
		{ FLASH_Sector_2,  0x08008000 },
		{ FLASH_Sector_3,  0x0800c000 },
		{ FLASH_Sector_4,  0x08010000 },
		{ FLASH_Sector_5,  0x08020000 },
		{ FLASH_Sector_6,  0x08040000 },
		{ FLASH_Sector_7,  0x08060000 },
		{ FLASH_Sector_8,  0x08080000 },
		{ FLASH_Sector_9,  0x080a0000 },
		{ FLASH_Sector_10, 0x080c0000 },
		{ FLASH_Sector_11, 0x080e0000 },
		{ 0xffff,          0x08100000 }
};

PIOS_BL_HELPER_FLASH_Start()
{
	uint32_t		baseAddress = START_OF_USER_CODE;
	uint32_t		endAddress = START_OF_USER_CODE + SIZE_OF_CODE + SIZE_OF_DESCRIPTION;
	int				i;

	// if the range to be erased overlaps the sector, erase the sector
	for (i = 0; _flash_sectors[i].baseAddress != 0xffff; i++) {
		if ((_flash_sectors[i].baseAddress < endAddress) &&
				(_flash_sectors[i + 1].baseAddress > baseAddress)) {
			// use the 'safe' flash mode per OpenOCD
			if (FLASH_COMPLETE != FLASH_EraseSector(_flash_sectors[i].sectorCode, VoltageRange_2)) {
				return 0;
			}
		}
	}
	return 1;
}

#else

uint8_t PIOS_BL_HELPER_FLASH_Start()
{
	const struct pios_board_info * bdinfo = &pios_board_info_blob;
	uint32_t pageAdress = bdinfo->fw_base;
	uint8_t fail = FALSE;
	while ((pageAdress < (bdinfo->fw_base + bdinfo->fw_size + bdinfo->desc_size))
	       || (fail == TRUE)) {
		for (int retry = 0; retry < MAX_DEL_RETRYS; ++retry) {
			if (FLASH_ErasePage(pageAdress) == FLASH_COMPLETE) {
				fail = FALSE;
				break;
			} else {
				fail = TRUE;
			}

		}

#ifdef STM32F10X_HD
		pageAdress += 2048;
#elif defined (STM32F10X_MD)
		pageAdress += 1024;
#endif
	}

	return (fail == TRUE) ? 0 : 1;
}
#endif

uint32_t PIOS_BL_HELPER_CRC_Memory_Calc()
{
	const struct pios_board_info * bdinfo = &pios_board_info_blob;

	PIOS_BL_HELPER_CRC_Ini();
	CRC_ResetDR();
	CRC_CalcBlockCRC((uint32_t *) bdinfo->fw_base, (bdinfo->fw_size) >> 2);
	return CRC_GetCRC();
}

void PIOS_BL_HELPER_FLASH_Read_Description(uint8_t * array, uint8_t size)
{
	const struct pios_board_info * bdinfo = &pios_board_info_blob;
	uint8_t x = 0;
	if (size > bdinfo->desc_size) size = bdinfo->desc_size;
	for (uint32_t i = bdinfo->fw_base + bdinfo->fw_size; i < bdinfo->fw_base + bdinfo->fw_size + size; ++i) {
		array[x] = *PIOS_BL_HELPER_FLASH_If_Read(i);
		++x;
	}
}

void PIOS_BL_HELPER_CRC_Ini()
{
#if defined(STM32F2XX)
	// we run the F2XX with all used clocks on all the time
#else
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC, ENABLE);
#endif
}
#endif
