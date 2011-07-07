/**
 ******************************************************************************
 * @addtogroup OpenPilotModules OpenPilot Modules
 * @{
 * @addtogroup PX2Test PX2 Test logic
 * @brief Hardware tests for the PX2 FMU
 * @{
 *
 * @file       px2test.c
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @brief      Module to handle all comms to the AHRS on a periodic basis.
 *
 * @see        The GNU Public License (GPL) Version 3
 *
 ******************************************************************************/
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

#include "pios.h"

static xTaskHandle testTaskHandle;

#define STACK_SIZE_BYTES		540						// XXX re-evaluate
#define TEST_TASK_PRIORITY		(tskIDLE_PRIORITY + 3)	// high

static void testTask(void *parameters);
static void	testAccel(void);
static void	testGyro(void);
static void	testMag(void);
static void	testBaro(void);
static void	testEEPROM(void);

#define puts(_s)				PIOS_COM_SendString(PIOS_COM_DEBUG, _s)
#define putln(_s)				puts(_s "\r\n")
#define print(_fmt, args...)	PIOS_COM_SendFormattedString(PIOS_COM_DEBUG, _fmt, ##args)
#define println(_fmt, args...)	PIOS_COM_SendFormattedString(PIOS_COM_DEBUG, _fmt "\r\n", ##args)

/**
 * Initialise the module, called on startup
 * \returns 0 on success or -1 if initialisation failed
 */
int32_t PX2TestInitialize(void)
{

	// Start the test task
	xTaskCreate(testTask, (signed char *)"Test", STACK_SIZE_BYTES/4, NULL, TEST_TASK_PRIORITY, &testTaskHandle);

	return 0;
}

/**
 * Module thread, should not return.
 */
static void testTask(void *parameters)
{
	putln("PX2 HARDWARE TEST");
	testAccel();
	testGyro();
	testMag();
	testBaro();
	testEEPROM();

	for (;;) {

	}
}

/**
 * Verify that the accelerometer can be initialised and that it is returning samples
 * at approximately the expected rate.
 */
static void
testAccel(void)
{
	struct pios_lis331_data		sample;
	int							samples;

	puts("LIS331...");
	PIOS_LIS331_Init();
	puts(" init OK");
	PIOS_LIS331_SelectRate(LIS331_RATE_400Hz);
	PIOS_LIS331_SetRange(LIS331_RANGE_8G);
	puts(" config OK");

	samples = 0;
	for (int i = 0; i < 100; i++) {
		if (PIOS_LIS331_Read(&sample))
			samples++;
		PIOS_DELAY_WaitmS(1);
	}
	puts(" sampling");
	if (samples < 30) {
		println(" FAIL: undersample(%d)", samples);
	} else if (samples > 50) {
		println(" FAIL: oversample(%d)", samples);
	} else {
		println(" OK", samples);
	}
}

/**
 * Verify that the gyro can be initialised and that it is returning samples
 * at approximately the expected rate.
 */
static void
testGyro(void)
{
	struct pios_l3g4200_data	sample;
	int							samples;

	puts("L3G4200...");
	PIOS_L3G4200_Init();
	puts(" init OK");
	PIOS_L3G4200_SelectRate(L3G4200_RATE_400Hz);
	PIOS_L3G4200_SetRange(L3G4200_RANGE_2000dps);
	puts(" config OK");

	samples = 0;
	for (int i = 0; i < 100; i++) {
		if (PIOS_L3G4200_Read(&sample))
			samples++;
		PIOS_DELAY_WaitmS(1);
	}
	puts(" sampling");
	if (samples < 30) {
		println(" FAIL: undersample(%d)", samples);
	} else if (samples > 50) {
		println(" FAIL: oversample(%d)", samples);
	} else {
		println(" OK", samples);
	}
}

/**
 * Verify that the magnetometer can be initialised and that it is returning samples
 * at approximately the expected rate.
 */
static void
testMag(void)
{
	int16_t			sample[3];
	int				samples;

	puts("HMC5883...");
	PIOS_HMC5883_Init();
	puts(" init OK");
#if 0
	/* don't do this for now - it makes the I2C bus sad */
	if (!PIOS_HMC5883_Test()) {
		putln(" FAIL: selftest");
		return;
	}
	puts(" selftest OK");
#endif

	samples = 0;
	for (int i = 0; i < 100; i++) {
		if (PIOS_HMC5883_NewDataAvailable()) {
			PIOS_HMC5883_ReadMag(sample);
			samples++;
		}
		PIOS_DELAY_WaitmS(10);
	}
	puts(" sampling");
	if (samples < 10) {
		println(" FAIL: undersample(%d)", samples);
	} else if (samples > 20) {
		println(" FAIL: oversample(%d)", samples);
	} else {
		println(" OK", samples);
	}
}

/**
 * Verify that the barometric pressure sensor can be initialised and that it is returning
 * data that looks approximately correct.
 */
static void
testBaro(void)
{
	int16_t	temperature;
	int32_t	pressure;

	puts("BMP085...");
	PIOS_BMP085_Init();
	puts(" init OK");

	PIOS_BMP085_StartADC(TemperatureConv);
	PIOS_BMP085_ReadADC();
	temperature = PIOS_BMP085_GetTemperature();
	puts(" temperature");
	if (temperature < 50) {
		print(" FAIL: undertemp(%d)", temperature);
	} else if (temperature > 500) {
		print(" FAIL: overtemp(%d)", temperature);
	} else {
		puts(" OK");
	}

	PIOS_BMP085_StartADC(PressureConv);
	PIOS_BMP085_ReadADC();
	pressure = PIOS_BMP085_GetPressure();
	puts(" pressure");
	if (pressure < 80000) {
		println(" FAIL: underpressure(%d)", pressure);
	} else if (pressure > 12000) {
		println(" FAIL: overpressure(%d)", pressure);
	} else {
		putln(" OK");
	}
}

/**
 * Verify that the EEPROM can be read and written.
 */
static void
testEEPROM(void)
{
	uint8_t	saved[4];
	uint8_t	buf[4];
	uint32_t addr;

	puts("EEPROM...");
	PIOS_EEPROM_Init();
	addr = PIOS_I2C_EEPROM_SIZE - 4;

	if (!PIOS_EEPROM_Read(addr, saved, 4)) {
		putln("FAIL: backup");
		return;
	}
	puts(" backup OK");
	buf[0] = ~saved[0];
	buf[1] = ~saved[1];
	buf[2] = ~saved[2];
	buf[3] = ~saved[3];
	if (!PIOS_EEPROM_Write(addr, buf, 4)) {
		putln(" FAIL: write");
		return;
	}
	puts(" write OK");
	buf[0] = buf[1] = buf[2] = buf[3] = 0x5a;
	if (!PIOS_EEPROM_Read(addr, buf, 4)) {
		putln("FAIL: readback");
		return;
	}
	if ((buf[0] != (uint8_t)~saved[0]) ||
		(buf[1] != (uint8_t)~saved[1]) ||
		(buf[2] != (uint8_t)~saved[2]) ||
		(buf[3] != (uint8_t)~saved[3])) {
		println(" FAIL: miscompare (read:expect) %02x:%02x %02x:%02x %02x:%02x %02x:%02x",
				buf[0], (uint8_t)~saved[0],
				buf[1], (uint8_t)~saved[1],
				buf[2], (uint8_t)~saved[2],
				buf[3], (uint8_t)~saved[3]);
		return;
	}
	puts(" compare OK");
	if (!PIOS_EEPROM_Write(addr, saved, 4)) {
		putln(" FAIL: restore");
		return;
	}
	putln(" restore OK");
}
