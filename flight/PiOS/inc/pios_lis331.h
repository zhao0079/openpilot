/*
 * pios_LIS331.h
 *
 *  Created on: May 1, 2011
 *      Author: msmith
 */

#ifndef PIOS_LIS331_H_
#define PIOS_LIS331_H_

struct pios_lis331_data {
	int16_t x;
	int16_t y;
	int16_t z;
};

#define LIS331_RATE_50Hz	(0<<3)
#define LIS331_RATE_100Hz	(1<<3)
#define LIS331_RATE_400Hz	(2<<3)
#define LIS331_RATE_1000Hz	(3<<3)

#define LIS331_RANGE_2G		(0<<4)
#define LIS331_RANGE_4G		(1<<4)
#define LIS331_RANGE_8G		(3<<4)

void PIOS_LIS331_SelectRate(uint8_t rate);
void PIOS_LIS331_SetRange(uint8_t range);
void PIOS_LIS331_Attach(uint32_t spi_id);
void PIOS_LIS331_Init();
bool PIOS_LIS331_Read(struct pios_lis331_data * data);

#endif /* PIOS_LIS331_H_ */
