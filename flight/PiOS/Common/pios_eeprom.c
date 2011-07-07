/*
 * pios_eeprom.c
 *
 *  Created on: May 1, 2011
 *      Author: msmith
 *
 * Generic I2C EEPROM support.
 */

#include <pios.h>

#ifndef PIOS_I2C_EEPROM_SIZE
# error Must define PIOS_I2C_EEPROM_SIZE
#endif
#define EEPROM_PAGE_SIZE		64		/* XXX is this truly constant? */
#define EEPROM_SECTOR_SIZE				0x100

#if 0
#define debug(fmt, args...)  PIOS_COM_SendFormattedString(PIOS_COM_DEBUG, "EE: " fmt "\r\n", ##args)
#define hexdump(p, count)	_hexdump(p, count)
#else
#define debug(fmt, args...)
#define hexdump(p, count)
#endif


uint32_t	bus_handle;
uint8_t		bus_address;
int			address_byte_count;

static void test(void);
static void _hexdump(void *p, int count) __attribute__((used));



static int8_t PIOS_EEPROM_Erase(uint32_t address);
static int8_t PIOS_EEPROM_Write(uint32_t address, uint8_t *data, uint16_t count);
static int8_t PIOS_EEPROM_Read(uint32_t address, uint8_t *data, uint16_t count);

PIOS_FLASHFS_Driver PIOS_EEPROM_Driver = {
		EEPROM_SECTOR_SIZE,
		PIOS_EEPROM_Erase,
		PIOS_EEPROM_Write,
		PIOS_EEPROM_Read
};

void
PIOS_EEPROM_Attach(uint32_t i2c_bus, uint8_t address)
{
	bus_handle = i2c_bus;
	bus_address = address;
}

void
PIOS_EEPROM_Init(void)
{
	debug("init");
	if (PIOS_I2C_EEPROM_SIZE <= (1<<16)) {
		address_byte_count = 2;
	} else if (PIOS_I2C_EEPROM_SIZE <= (1<<24)) {
		address_byte_count = 3;
	} else {
		address_byte_count = 4;		// rather unlikely
	}
	if (0)
		test();
}

static int8_t
PIOS_EEPROM_Erase(uint32_t address)
{
	uint8_t	blank[64];
	int count;

	debug("erase 0x%x", address);

	memset(blank, 0, sizeof(blank));
	for (count = 0; count < EEPROM_SECTOR_SIZE; count += sizeof(blank), address += sizeof(blank)) {
		if (PIOS_EEPROM_Write(address, blank, sizeof(blank))) {
			debug("erase error");
			return -1;
		}
	}
	return 0;
}

static int8_t
PIOS_EEPROM_Write(uint32_t address, uint8_t *data, uint16_t count)
{
	int run_length;
	uint8_t	*p = (uint8_t *)data;
	uint8_t	cmd[4 + EEPROM_PAGE_SIZE];
	struct pios_i2c_txn txn_list[] = {
			{
					.info = __func__,
					.addr = bus_address,
					.rw = PIOS_I2C_TXN_WRITE,
					.buf = &cmd[4 - address_byte_count]
			}
	};

	debug("write 0x%x/%d", address, count);
	hexdump(data, count);

	if ((address + count) > PIOS_I2C_EEPROM_SIZE)
		return -1;

	/* batch writes to avoid over-wearing pages */
	while (count) {
		/* maximum run length before crossing a page boundary */
		run_length = (EEPROM_PAGE_SIZE - (address % EEPROM_PAGE_SIZE));
		if (run_length > count)
			run_length = count;

		/* build a 4-byte address packet - buffer offset above compensates for the actual device */
		cmd[0] = address >> 24;
		cmd[1] = address >> 16;
		cmd[2] = address >> 8;
		cmd[3] = address;
		memcpy(&cmd[4], p, run_length);

		txn_list[0].len = address_byte_count + run_length;
		if (!PIOS_I2C_Transfer(bus_handle, txn_list, NELEMENTS(txn_list))) {
			debug("write error");
			return -1;
		}

		p += run_length;
		count -= run_length;
		address += run_length;

		/* wait for the operation to complete - 5ms per sample datasheet */
		/* note that this will yield if possible */
		PIOS_DELAY_WaitmS(5);
	}
	return 0;
}


static int8_t
PIOS_EEPROM_Read(uint32_t address, uint8_t *data, uint16_t count)
{
	uint8_t	cmd[4];
	struct pios_i2c_txn txn_list[] = {
			{
					.info = __func__,
					.addr = bus_address,
					.rw = PIOS_I2C_TXN_WRITE,
					.len = 4 - address_byte_count,
					.buf = &cmd[4 - address_byte_count]
			},
			{
					.info = __func__,
					.addr = bus_address,
					.rw = PIOS_I2C_TXN_READ,
					.len = count,
					.buf = data
			}
	};

	debug("read 0x%x/%d", address, count);

	if ((address + count) > PIOS_I2C_EEPROM_SIZE)
		return -1;

	/* build a 4-byte address packet - buffer offset above compensates for the actual device */
	cmd[0] = address >> 24;
	cmd[1] = address >> 16;
	cmd[2] = address >> 8;
	cmd[3] = address;

	if (!PIOS_I2C_Transfer(bus_handle, txn_list, NELEMENTS(txn_list))) {
		debug("read error");
		return -1;
	}

	hexdump(data, count);
	return 0;
}

static void
_hexdump(void *p, int count)
{
	uint8_t	*b = (uint8_t *)p;

	while (count--)
		PIOS_COM_SendFormattedString(PIOS_COM_DEBUG, "%02x ", *b++);
	PIOS_COM_SendString(PIOS_COM_DEBUG, "\r\n");
}

static void
test(void)
{
	uint8_t	buf[128];
	int		x, y;

	for (y = 1; y < 128; y++)
		buf[y] = y;
	for (x = 0; x < 8; x++) {
		buf[0] = x;

		if (PIOS_EEPROM_Write(x * 128, buf, 128))
			debug("write error @ %d", x);
	}

	for (x = 0; x < 8; x++) {
		memset(buf, 0, 128);

		if (PIOS_EEPROM_Read(x * 128, buf, 128)) {
			debug("read error @ %d", x);
			continue;
		}

		if (buf[0] != x) {
			debug ("block mismatch %d should be %d", buf[0], x);
			continue;
		}
		for (y = 1; y < 128; y++) {
			if (buf[y] != y) {
				debug("data mismatch %d,%d should be %d", x, buf[y], y);
			}
		}
	}
	debug("PASS");
}
