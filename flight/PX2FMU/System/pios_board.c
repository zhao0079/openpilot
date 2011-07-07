/**
 ******************************************************************************
 * @addtogroup OpenPilotSystem OpenPilot System
 * @{
 * @addtogroup OpenPilotCore OpenPilot Core
 * @{
 *
 * @file       pios_board.c
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @brief      Defines board specific static initializers for hardware for the OpenPilot board.
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

#include <pios.h>
#include <openpilot.h>
#include <uavobjectsinit.h>

#include <pios_usart_priv.h>
#include <pios_spi_priv.h>
#include <pios_com_priv.h>
#include <pios_ppm_priv.h>
#include "pios_adc_priv.h"
#include <pios_i2c_priv.h>

#ifdef PIOS_DEBUG
#include "dcc_stdio.h"
#endif

/* XXX this should be more comprehensively abstracted */
void PIOS_SPI_main_irq_handler(void);
void DMA2_Stream0_IRQ_Handler(void) __attribute__((alias("PIOS_SPI_main_irq_handler")));
void DMA2_Stream3_IRQ_Handler(void) __attribute__((alias("PIOS_SPI_main_irq_handler")));
const struct pios_spi_cfg pios_spi_main_cfg = SPI1_CONFIG(PIOS_SPI_main_irq_handler);

uint32_t pios_spi_main_id;
void PIOS_SPI_main_irq_handler(void)
{
  /* Call into the generic code to handle the IRQ for this specific device */
	PIOS_SPI_IRQ_Handler(pios_spi_main_id);
}


/*
 * ADC system
 */
void DMA2_Stream4_IRQHandler() __attribute__ ((alias("PIOS_ADC_handler")));
const struct pios_adc_cfg pios_adc_cfg = {
	.dma = {
		.irq = {
			.handler = PIOS_ADC_DMA_Handler,
			.flags   = (DMA_FLAG_TCIF4 | DMA_FLAG_TEIF4 | DMA_FLAG_HTIF4),
			.init    = {
				.NVIC_IRQChannel		= DMA2_Stream4_IRQn
			},
		},
		/* XXX there is secret knowledge here regarding the use of ADC1 by the pios_adc code */
		.rx = {
			.channel = DMA2_Stream4,	// stream0 may be used by SPI1
			.init    = {
				.DMA_Channel			= DMA_Channel_0,
				.DMA_PeripheralBaseAddr	= (uint32_t) & ADC1->DR
			},
		}
	}, 
	.half_flag = DMA_IT_HTIF4,
	.full_flag = DMA_IT_TCIF4,
};

struct pios_adc_dev pios_adc_devs[] = {
	{
		.cfg = &pios_adc_cfg,
		.callback_function = NULL,
		.data_queue = NULL
	},
};

uint8_t pios_adc_num_devices = NELEMENTS(pios_adc_devs);

void PIOS_ADC_handler() {
	PIOS_ADC_DMA_Handler();
}

/*
 * Telemetry USART
 */
void PIOS_USART_telem_irq_handler(void);
void USART2_IRQHandler() __attribute__ ((alias ("PIOS_USART_telem_irq_handler")));
const struct pios_usart_cfg pios_usart_telem_cfg = USART2_CONFIG(PIOS_COM_TELEM_BAUDRATE, PIOS_USART_telem_irq_handler);
static uint32_t pios_usart_telem_rf_id;
void PIOS_USART_telem_irq_handler(void)
{
	PIOS_USART_IRQ_Handler(pios_usart_telem_rf_id);
}

/*
 * GPS USART
 */
void PIOS_USART_gps_irq_handler(void);
void USART6_IRQHandler() __attribute__ ((alias ("PIOS_USART_gps_irq_handler")));
const struct pios_usart_cfg pios_usart_gps_cfg = USART6_CONFIG(PIOS_COM_GPS_BAUDRATE, PIOS_USART_gps_irq_handler);
static uint32_t pios_usart_gps_id;
void PIOS_USART_gps_irq_handler(void)
{
	PIOS_USART_IRQ_Handler(pios_usart_gps_id);
}

/*
 * AUX USART
 */
void PIOS_USART_aux_irq_handler(void);
void USART1_IRQHandler() __attribute__ ((alias ("PIOS_USART_aux_irq_handler")));
const struct pios_usart_cfg pios_usart_aux_cfg = USART1_CONFIG(PIOS_COM_AUX_BAUDRATE, PIOS_USART_aux_irq_handler);
static uint32_t pios_usart_aux_id;
void PIOS_USART_aux_irq_handler(void)
{
	PIOS_USART_IRQ_Handler(pios_usart_aux_id);
}

/*
 * Control USART
 */
void PIOS_USART_control_irq_handler(void);
void UART5_IRQHandler() __attribute__ ((alias ("PIOS_USART_control_irq_handler")));
const struct pios_usart_cfg pios_usart_control_cfg = UART5_CONFIG(PIOS_COM_CONTROL_BAUDRATE, PIOS_USART_control_irq_handler);
static uint32_t pios_usart_control_id;
void PIOS_USART_control_irq_handler(void)
{
	PIOS_USART_IRQ_Handler(pios_usart_control_id);
}


/*
 * PPM Input
 */
/* XXX this could do with better abstracting */
#if defined(PIOS_INCLUDE_PPM)
void TIM1_CC_IRQHandler();
void TIM1_CC_IRQHandler() __attribute__ ((alias ("PIOS_TIM1_CC_irq_handler")));
const struct pios_ppm_cfg pios_ppm_cfg = {
	.tim_base_init = {
		.TIM_Prescaler = (PIOS_MASTER_CLOCK / 1000000) - 1,
		.TIM_ClockDivision = TIM_CKD_DIV1,
		.TIM_CounterMode = TIM_CounterMode_Up,
		.TIM_Period = 0xFFFF,
		.TIM_RepetitionCounter = 0x0000,
	},
	.tim_ic_init = {
			.TIM_ICPolarity = TIM_ICPolarity_Rising,
			.TIM_ICSelection = TIM_ICSelection_DirectTI,
			.TIM_ICPrescaler = TIM_ICPSC_DIV1,
			.TIM_ICFilter = 0x0,
			.TIM_Channel = PIOS_PPM_TIM_CHANNEL,
	},
	.remap = PIOS_PPM_GPIO_REMAP,
	.port = PIOS_PPM_GPIO_PORT,
	.gpio_init = {
			.GPIO_Pin   = PIOS_PPM_GPIO_PIN,
			.GPIO_Mode  = GPIO_Mode_AF,
			.GPIO_Speed = GPIO_Speed_2MHz,
			.GPIO_OType = GPIO_OType_OD,
			.GPIO_PuPd  = GPIO_PuPd_UP
	},
	.irq = {
		.handler = TIM1_CC_IRQHandler,
		.init    = {
			.NVIC_IRQChannel                   = PIOS_PPM_TIM_IRQ,
			.NVIC_IRQChannelPreemptionPriority = PIOS_IRQ_PRIO_MID,
			.NVIC_IRQChannelSubPriority        = 0,
			.NVIC_IRQChannelCmd                = ENABLE,
			.NVIC_IRQChannel                   = PIOS_PPM_TIM_IRQ,
		},
	},
	.timer = PIOS_PPM_TIM,
	.ccr = PIOS_PPM_TIM_CCR,
};

void PIOS_TIM1_CC_irq_handler()
{
	PIOS_PPM_irq_handler();
}

#endif //PPM


/*
 * I2C Adapters
 */

/* XXX this should be more comprehensively abstracted */

/* I2C ESCs connected via the multi-connector */
void PIOS_I2C_esc_adapter_ev_irq_handler(void);
void PIOS_I2C_esc_adapter_er_irq_handler(void);
void I2C1_EV_IRQHandler() __attribute__ ((alias ("PIOS_I2C_esc_adapter_ev_irq_handler")));
void I2C1_ER_IRQHandler() __attribute__ ((alias ("PIOS_I2C_esc_adapter_er_irq_handler")));

const struct pios_i2c_adapter_cfg pios_i2c_esc_adapter_cfg = I2C1_CONFIG(PIOS_I2C_esc_adapter_ev_irq_handler,
																			PIOS_I2C_esc_adapter_er_irq_handler);

uint32_t pios_i2c_esc_adapter_id;
void PIOS_I2C_esc_adapter_ev_irq_handler(void)
{
	/* Call into the generic code to handle the IRQ for this specific device */
	PIOS_I2C_EV_IRQ_Handler(pios_i2c_esc_adapter_id);
}

void PIOS_I2C_esc_adapter_er_irq_handler(void)
{
  /* Call into the generic code to handle the IRQ for this specific device */
  PIOS_I2C_ER_IRQ_Handler(pios_i2c_esc_adapter_id);
}

/* Onboard sensor bus + IO comms */
void PIOS_I2C_main_adapter_ev_irq_handler(void);
void PIOS_I2C_main_adapter_er_irq_handler(void);
void I2C2_EV_IRQHandler() __attribute__ ((alias ("PIOS_I2C_main_adapter_ev_irq_handler")));
void I2C2_ER_IRQHandler() __attribute__ ((alias ("PIOS_I2C_main_adapter_er_irq_handler")));

const struct pios_i2c_adapter_cfg pios_i2c_main_adapter_cfg = I2C2_CONFIG(PIOS_I2C_main_adapter_ev_irq_handler,
																			 PIOS_I2C_main_adapter_er_irq_handler);

uint32_t pios_i2c_main_adapter_id;
void PIOS_I2C_main_adapter_ev_irq_handler(void)
{
	/* Call into the generic code to handle the IRQ for this specific device */
	PIOS_I2C_EV_IRQ_Handler(pios_i2c_main_adapter_id);
}

void PIOS_I2C_main_adapter_er_irq_handler(void)
{
  /* Call into the generic code to handle the IRQ for this specific device */
  PIOS_I2C_ER_IRQ_Handler(pios_i2c_main_adapter_id);
}

/* Off-board sensor bus */
void PIOS_I2C_external_adapter_ev_irq_handler(void);
void PIOS_I2C_external_adapter_er_irq_handler(void);
void I2C3_EV_IRQHandler() __attribute__ ((alias ("PIOS_I2C_external_adapter_ev_irq_handler")));
void I2C3_ER_IRQHandler() __attribute__ ((alias ("PIOS_I2C_external_adapter_er_irq_handler")));

const struct pios_i2c_adapter_cfg pios_i2c_external_adapter_cfg = I2C3_CONFIG(PIOS_I2C_external_adapter_ev_irq_handler,
																			 PIOS_I2C_external_adapter_er_irq_handler);

uint32_t pios_i2c_external_adapter_id;
void PIOS_I2C_external_adapter_ev_irq_handler(void)
{
	/* Call into the generic code to handle the IRQ for this specific device */
	PIOS_I2C_EV_IRQ_Handler(pios_i2c_external_adapter_id);
}

void PIOS_I2C_external_adapter_er_irq_handler(void)
{
  /* Call into the generic code to handle the IRQ for this specific device */
  PIOS_I2C_ER_IRQ_Handler(pios_i2c_external_adapter_id);
}


/*
 * COM interfaces
 */

extern const struct pios_com_driver pios_usb_com_driver;

uint32_t pios_com_telem_rf_id;
uint32_t pios_com_telem_usb_id;
uint32_t pios_com_gps_id;
uint32_t pios_com_aux_id;
uint32_t pios_com_control_id;


/**
 * PIOS_Board_Init()
 * initializes all the core subsystems on this specific hardware
 * called from System/openpilot.c
 */
void PIOS_Board_Init(void) {

	/* Debug services */
	PIOS_DEBUG_Init();

	/* Delay system */
	PIOS_DELAY_Init();

	/* Initialise USARTs */
	if (PIOS_USART_Init(&pios_usart_telem_rf_id, &pios_usart_telem_cfg) ||
		PIOS_COM_Init(&pios_com_telem_rf_id, &pios_usart_com_driver, pios_usart_telem_rf_id)) {
		PIOS_DEBUG_Assert(0);
	}
	if (PIOS_USART_Init(&pios_usart_gps_id, &pios_usart_gps_cfg) ||
		PIOS_COM_Init(&pios_com_gps_id, &pios_usart_com_driver, pios_usart_gps_id)) {
		PIOS_DEBUG_Assert(0);
	}
	if (PIOS_USART_Init(&pios_usart_aux_id, &pios_usart_aux_cfg) ||
		PIOS_COM_Init(&pios_com_aux_id, &pios_usart_com_driver, pios_usart_aux_id)) {
		PIOS_DEBUG_Assert(0);
	}
	if (PIOS_USART_Init(&pios_usart_control_id, &pios_usart_control_cfg) ||
		PIOS_COM_Init(&pios_com_control_id, &pios_usart_com_driver, pios_usart_control_id)) {
		PIOS_DEBUG_Assert(0);
	}
	PIOS_COM_SendString(PIOS_COM_DEBUG, "\r\n\r\nFMU starting: USART ");

	/* Internal settings support - must come up before UAVO */
	if (PIOS_I2C_Init(&pios_i2c_main_adapter_id, &pios_i2c_main_adapter_cfg)) {
		PIOS_DEBUG_Assert(0);
	}
	PIOS_EEPROM_Attach(PIOS_I2C_MAIN_ADAPTER, PIOS_I2C_EEPROM_ADDRESS);
	PIOS_EEPROM_Init();
	PIOS_FLASHFS_Init(&PIOS_EEPROM_Driver);

	/* Initialize UAVObject libraries */
	EventDispatcherInitialize();
	UAVObjInitialize();
	UAVObjectsInitializeAll();

	/* Initialize the alarms library */
	AlarmsInitialize();

	/* Initialize the task monitor library */
	TaskMonitorInitialize();

	/* GPIO init */
	PIOS_GPIO_Init();

	/* Initialise SPI */
	if (PIOS_SPI_Init(&pios_spi_main_id, &pios_spi_main_cfg)) {
		PIOS_DEBUG_Assert(0);
	}
	PIOS_L3G4200_Attach(pios_spi_main_id);
	PIOS_LIS331_Attach(pios_spi_main_id);

	/* XXX sdcard init here */

	PIOS_COM_SendString(PIOS_COM_DEBUG, "SPI ");

	if (PIOS_I2C_Init(&pios_i2c_esc_adapter_id, &pios_i2c_esc_adapter_cfg)) {
		PIOS_DEBUG_Assert(0);
	}
#if 0 /* XXX this will hang without pullups on I2C3 - busted for early FMU boards, only safe if IO is connected */
	/* XXX might want to sniff for pullups first... */
	if (PIOS_I2C_Init(&pios_i2c_external_adapter_id, &pios_i2c_external_adapter_cfg)) {
		PIOS_DEBUG_Assert(0);
	}
#endif

	PIOS_COM_SendString(PIOS_COM_DEBUG, "I2C ");
	PIOS_ADC_Init();
	PIOS_COM_SendString(PIOS_COM_DEBUG, "ADC ");
	PIOS_PPM_Init();
	PIOS_COM_SendString(PIOS_COM_DEBUG, "PPM ");

#if defined(PIOS_INCLUDE_USB_HID)
	PIOS_USB_HID_Init(0);
#if defined(PIOS_INCLUDE_COM)
	if (PIOS_COM_Init(&pios_com_telem_usb_id, &pios_usb_com_driver, 0)) {
		PIOS_DEBUG_Assert(0);
	}
	PIOS_COM_SendString(PIOS_COM_DEBUG, "USB ");
#endif	/* PIOS_INCLUDE_COM */
#endif  /* PIOS_INCLUDE_USB_HID */

	PIOS_IAP_Init();

#if defined(PIOS_INCLUDE_WDG)
	PIOS_WDG_Init();
#endif /* PIOS_INCLUDE_WDG */

	PIOS_COM_SendString(PIOS_COM_DEBUG, "Hardware init done.\r\n");
}

/**
 * @}
 */
