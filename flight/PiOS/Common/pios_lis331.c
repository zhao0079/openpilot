/*
 * pios_LIS331.c
 *
 *  Created on: May 1, 2011
 *      Author: Michael Smith
 *
 *  Based on the ADXL345 driver.
 */

#include <pios.h>

#define DIR_READ		(1<<7)
#define DIR_WRITE		(0<<7)
#define ADDR_INCREMENT	(1<<6)

#define ADDR_WHO_AM_I			0x0f
#define WHO_I_AM					0x32

#define ADDR_CTRL_REG1			0x20		/* sample rate constants are in the public header */
#define REG1_POWER_NORMAL			(1<<5)
#define REG1_Z_ENABLE				(1<<2)
#define REG1_Y_ENABLE				(1<<1)
#define REG1_X_ENABLE				(1<<0)

#define ADDR_CTRL_REG2			0x21

#define ADDR_CTRL_REG3			0x22

#define ADDR_CTRL_REG4			0x23
#define REG4_BDU					(1<<7)
#define REG4_BIG_ENDIAN				(1<<6)
#define REG4_SPI_3WIRE				(1<<0)

#define ADDR_CTRL_REG5			0x24

#define ADDR_HP_FILTER_RESET	0x25
#define ADDR_REFERENCE			0x26
#define ADDR_STATUS_REG			0x27
#define STATUS_ZYXOR				(1<<7)
#define SATAUS_ZOR					(1<<6)
#define STATUS_YOR					(1<<5)
#define STATUS_XOR					(1<<4)
#define STATUS_ZYXDA				(1<<3)
#define STATUS_ZDA					(1<<2)
#define STATUS_YDA					(1<<1)
#define STATUS_XDA					(1<<0)

#define ADDR_OUT_X				0x28	/* 16 bits */
#define ADDR_OUT_Y				0x2A	/* 16 bits */
#define ADDR_OUT_Z				0x2C	/* 16 bits */

static uint32_t bus_handle;

/**
 * @brief Claim the SPI bus for the accel communications and select this chip
 */
static void
claim_bus()
{
	PIOS_SPI_ClaimBus(bus_handle);
	PIOS_SPI_RC_PinSet(bus_handle, SPI_CS_ACCEL, 0);
}

/**
 * @brief Release the SPI bus for the accel communications and end the transaction
 */
static void
release_bus()
{
	PIOS_SPI_RC_PinSet(bus_handle, SPI_CS_ACCEL, 1);
	PIOS_SPI_ReleaseBus(bus_handle);
}

/**
 * @brief Write a single register in the device.
 */
static void
write_reg(uint8_t address, uint8_t data)
{
	uint8_t cmd[2] = { address | DIR_WRITE, data };

	claim_bus();
	PIOS_SPI_TransferBlock(bus_handle, cmd, NULL, sizeof(cmd), NULL);
	release_bus();
}

/**
 * @brief Read a single register in the device
 */
static uint8_t
read_reg(uint8_t address)
{
	uint8_t	cmd[2] = {address | DIR_READ, 0};
	uint8_t data[2];

	claim_bus();
	PIOS_SPI_TransferBlock(bus_handle, cmd, data, sizeof(cmd), NULL);
	release_bus();

	return data[1];
}

/**
 * @brief Select the sampling rate of the chip
 * 
 * This also puts it into high power mode
 */
void
PIOS_LIS331_SelectRate(uint8_t rate)
{
	write_reg(ADDR_CTRL_REG1,
			(rate & (3<<3)) | REG1_POWER_NORMAL | REG1_Z_ENABLE | REG1_Y_ENABLE | REG1_X_ENABLE);
}

/**
 * @brief Set the range of the accelerometer. Also enforces 4-wire SPI and coherent LSB/MSB sample updates.
 */
void
PIOS_LIS331_SetRange(uint8_t range)
{
	write_reg(ADDR_CTRL_REG4, (range & (3<<4)) | REG4_BDU);
}

/**
 * @brief Connect to the correct SPI bus
 */
void
PIOS_LIS331_Attach(uint32_t spi_id)
{
	bus_handle = spi_id;
}

/**
 * @brief Initialize with sane default settings
 */
void
PIOS_LIS331_Init()
{
	/* run a who-am-I to verify that the device is present */
	PIOS_Assert(read_reg(ADDR_WHO_AM_I) == WHO_I_AM);

	write_reg(ADDR_CTRL_REG2, 0);			/* disable interrupt-generating high-pass filters */
	write_reg(ADDR_CTRL_REG3, 0);			/* no interrupts - we don't use them */
	write_reg(ADDR_CTRL_REG5, 0);			/* disable wake-on-interrupt */

	PIOS_LIS331_SetRange(LIS331_RANGE_8G);
	PIOS_LIS331_SelectRate(LIS331_RATE_400Hz);	/* takes device out of low-power mode */
}

/**
 * @brief Read a single set of values from the x y z channels
 * @returns true if the samples are new data, false if they are stale
 */
bool
PIOS_LIS331_Read(struct pios_lis331_data * data)
{
	struct {					/* status register and data as read back from the device */
		uint8_t		cmd;
		uint8_t		status;
		int16_t		x;
		int16_t		y;
		int16_t		z;
	} __attribute__((packed))	report;
	
	report.cmd = ADDR_STATUS_REG | DIR_READ | ADDR_INCREMENT;

	/* exchange the report structure with the device */
	claim_bus();
	PIOS_SPI_TransferBlock(bus_handle, (uint8_t *)&report, (uint8_t *)&report, sizeof(report), NULL);
	release_bus();
	
	data->x = report.x;
	data->y = report.y;
	data->z = report.z;

	return report.status & STATUS_ZYXDA;
}
