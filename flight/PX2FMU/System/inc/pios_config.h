/**
 ******************************************************************************
 * @addtogroup OpenPilotSystem OpenPilot System
 * @{
 * @addtogroup OpenPilotCore OpenPilot Core
 * @{
 *
 * @file       pios_config.h  
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @brief      PiOS configuration header. 
 *             Central compile time config for the project.
 *             In particular, pios_config.h is where you define which PiOS libraries
 *             and features are included in the firmware.
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


#ifndef PIOS_CONFIG_H
#define PIOS_CONFIG_H

/* Enable/Disable PiOS Modules */
#define PIOS_INCLUDE_ADC
#define PIOS_INCLUDE_BMP085
#define PIOS_INCLUDE_COM
#define PIOS_INCLUDE_DELAY
#define PIOS_INCLUDE_EEPROM
#define PIOS_INCLUDE_EXTI
#define PIOS_INCLUDE_FLASH
#define PIOS_INCLUDE_FLASH_SECTOR_SETTINGS
#define PIOS_INCLUDE_FREERTOS
#define PIOS_INCLUDE_GPIO
#define PIOS_INCLUDE_HMC5883
#define PIOS_INCLUDE_I2C
#define PIOS_INCLUDE_IRQ
#define PIOS_INCLUDE_LED
#define PIOS_INCLUDE_PPM
#define PIOS_INCLUDE_SPI
#define PIOS_INCLUDE_SYS
#define PIOS_INCLUDE_USART
#define PIOS_INCLUDE_SDCARD

//#define PIOS_INCLUDE_USB_HID		// XXX needs PiOS support work
//#define PIOS_INCLUDE_HCSR04		// XXX needs PiOS support work
//#define PIOS_INCLUDE_WDG			// XXX needs PiOS support work

/* Defaults for Logging */
#define LOG_FILENAME 			"PIOS.LOG"
#define STARTUP_LOG_ENABLED		1

/* Enable a priority queue in telemetry */
#define PIOS_TELEM_PRIORITY_QUEUE

/* COM Module */
#define GPS_BAUDRATE			19200

#ifndef TELEM_BAUDRATE
#define TELEM_BAUDRATE			115200
#endif

#ifndef AUXUART_BAUDRATE
#define AUXUART_BAUDRATE		115200
#endif

/* Alarm Thresholds */
#define HEAP_LIMIT_WARNING		4000
#define HEAP_LIMIT_CRITICAL		1000
#define CPULOAD_LIMIT_WARNING		80
#define CPULOAD_LIMIT_CRITICAL		95

#endif /* PIOS_CONFIG_H */
/**
 * @}
 * @}
 */
