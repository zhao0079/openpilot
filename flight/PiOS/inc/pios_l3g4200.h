/*
 * pios_l3g4200.h
 *
 *  Created on: May 1, 2011
 *      Author: msmith
 */

#ifndef PIOS_L3G4200_H_
#define PIOS_L3G4200_H_

struct pios_l3g4200_data {
	int16_t x;
	int16_t y;
	int16_t z;
};

/* combined rate/bandwidth constants, obtained from the ST sample driver */
#define L3G4200_RATE_100Hz			((0<<6) | (0<<4))
#define L3G4200_RATE_200Hz			((1<<6) | (0<<4))
#define L3G4200_RATE_400Hz			((2<<6) | (1<<4))
#define L3G4200_RATE_800Hz			((3<<6) | (2<<4))

#define L3G4200_RANGE_250dps		(0<<4)
#define L3G4200_RANGE_500dps		(1<<4)
#define L3G4200_RANGE_2000dps		(3<<4)

void PIOS_L3G4200_SelectRate(uint8_t rate);
void PIOS_L3G4200_SetRange(uint8_t range);
void PIOS_L3G4200_Attach(uint32_t spi_id);
void PIOS_L3G4200_Init();
int8_t PIOS_L3G4200_ReadTemperature(void);
bool PIOS_L3G4200_Read(struct pios_l3g4200_data * data);

#endif /* PIOS_L3G4200_H_ */
