/*
 * pios_eeprom.h
 *
 *  Created on: May 1, 2011
 *      Author: Michael Smith
 */

#ifndef PIOS_EEPROM_H_
#define PIOS_EEPROM_H_

#include <pios_flashfs_objlist.h>

extern PIOS_FLASHFS_Driver	PIOS_EEPROM_Driver;

void PIOS_EEPROM_Attach(uint32_t i2c_bus, uint8_t address);
void PIOS_EEPROM_Init(void);

#endif /* PIOS_EEPROM_H_ */
