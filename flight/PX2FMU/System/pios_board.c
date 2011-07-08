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
#include <pios_adc_priv.h>
#include <pios_i2c_priv.h>
#include <pios_rcvr_priv.h>


#ifdef PIOS_DEBUG
#include "dcc_stdio.h"
#endif

/* XXX this should be more comprehensively abstracted */
void PIOS_SPI_main_irq_handler(void);
void DMA2_Stream0_IRQ_Handler(void) __attribute__((alias("PIOS_SPI_main_irq_handler")));
void DMA2_Stream3_IRQ_Handler(void) __attribute__((alias("PIOS_SPI_main_irq_handler")));
const struct pios_spi_cfg pios_spi_main_cfg = {
    .regs    = SPI1,
    .remap   = GPIO_AF_SPI1,
    .use_crc = false,
    .init    = {
        .SPI_Mode              = SPI_Mode_Master,
        .SPI_Direction         = SPI_Direction_2Lines_FullDuplex,
        .SPI_DataSize          = SPI_DataSize_8b,
        .SPI_NSS               = SPI_NSS_Soft,
        .SPI_FirstBit          = SPI_FirstBit_MSB,
        .SPI_CRCPolynomial     = 7,
        .SPI_CPOL              = SPI_CPOL_High,
        .SPI_CPHA              = SPI_CPHA_2Edge,
        .SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8,
    },
    .dma     = {
        /* .ahb_clk - not required */
        .irq = {
            .handler = NULL,
            .flags   = (DMA_IT_TCIF3 | DMA_IT_TEIF3 | DMA_IT_HTIF3),
            .init    = {
                .NVIC_IRQChannel                   = DMA2_Stream0_IRQn,
                .NVIC_IRQChannelPreemptionPriority = PIOS_IRQ_PRIO_HIGH,
                .NVIC_IRQChannelSubPriority        = 0,
                .NVIC_IRQChannelCmd                = ENABLE,
            },
        },
        .rx = {
            .channel = DMA2_Stream0,
            .init = {
                .DMA_Channel            = DMA_Channel_3,
                .DMA_PeripheralBaseAddr = (uint32_t)&(SPI1->DR),
                /* .DMA_Memory0BaseAddr */
                .DMA_DIR                = DMA_DIR_PeripheralToMemory,
                /* .DMA_BufferSize */
                .DMA_PeripheralInc      = DMA_PeripheralInc_Disable,
                /* .DMA_BufferSize */
                .DMA_PeripheralInc      = DMA_PeripheralInc_Disable,
                .DMA_MemoryInc          = DMA_MemoryInc_Enable,
                .DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte,
                .DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte,
                .DMA_Mode               = DMA_Mode_Normal,
                .DMA_Priority           = DMA_Priority_High,
                .DMA_FIFOMode           = DMA_FIFOMode_Disable,
                /* .DMA_FIFOThreshold */
                .DMA_MemoryBurst        = DMA_MemoryBurst_Single,
                .DMA_PeripheralBurst    = DMA_PeripheralBurst_Single,
            },
        },
        .tx = {
            .channel = DMA2_Stream3,
            .init = {
                .DMA_Channel            = DMA_Channel_3,
                /* .DMA_Memory0BaseAddr */
                .DMA_PeripheralBaseAddr = (uint32_t)&(SPI1->DR),
                .DMA_DIR                = DMA_DIR_MemoryToPeripheral,
                /* .DMA_BufferSize */
                .DMA_PeripheralInc      = DMA_PeripheralInc_Disable,
                .DMA_MemoryInc          = DMA_MemoryInc_Enable,
                .DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte,
                .DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte,
                .DMA_Mode               = DMA_Mode_Normal,
                .DMA_Priority           = DMA_Priority_High,
                .DMA_FIFOMode           = DMA_FIFOMode_Disable,
                /* .DMA_FIFOThreshold */
                .DMA_MemoryBurst        = DMA_MemoryBurst_Single,
                .DMA_PeripheralBurst    = DMA_PeripheralBurst_Single,
            },
        },
    },
    .sclk = {
        .gpio = GPIOA,
        .init = {
            .GPIO_Pin   = GPIO_Pin_5,
            .GPIO_Mode  = GPIO_Mode_AF,
            .GPIO_Speed = GPIO_Speed_100MHz,
            .GPIO_OType = GPIO_OType_PP,
            .GPIO_PuPd  = GPIO_PuPd_UP,
        },
    },
    .miso = {
        .gpio = GPIOA,
        .init = {
            .GPIO_Pin   = GPIO_Pin_6,
            .GPIO_Mode  = GPIO_Mode_AF,
            .GPIO_Speed = GPIO_Speed_50MHz,
            .GPIO_OType = GPIO_OType_PP,
            .GPIO_PuPd  = GPIO_PuPd_UP,
        },
    },
    .mosi = {
        .gpio = GPIOA,
        .init = {
            .GPIO_Pin   = GPIO_Pin_7,
            .GPIO_Mode  = GPIO_Mode_AF,
            .GPIO_Speed = GPIO_Speed_50MHz,
            .GPIO_OType = GPIO_OType_PP,
            .GPIO_PuPd  = GPIO_PuPd_UP,
        },
    },
    .slave_count = 2,
    .ssel = {
        {
            .gpio = GPIOC,
            .init = {
                .GPIO_Pin   = GPIO_Pin_14,
                .GPIO_Mode  = GPIO_Mode_OUT,
                .GPIO_Speed = GPIO_Speed_50MHz,
                .GPIO_OType = GPIO_OType_PP,
                .GPIO_PuPd  = GPIO_PuPd_UP,
            },
        },
        {
            .gpio = GPIOC,
            .init = {
                .GPIO_Pin   = GPIO_Pin_15,
                .GPIO_Mode  = GPIO_Mode_OUT,
                .GPIO_Speed = GPIO_Speed_50MHz,
                .GPIO_OType = GPIO_OType_PP,
                .GPIO_PuPd  = GPIO_PuPd_UP,
            },
        },
    },
};

uint32_t pios_spi_main_id;
void PIOS_SPI_main_irq_handler(void)
{
  /* Call into the generic code to handle the IRQ for this specific device */
	PIOS_SPI_IRQ_Handler(pios_spi_main_id);
}

/*
 * ADC system
 */
void PIOS_ADC_handler(void)
{
	PIOS_ADC_DMA_Handler();
}
void DMA2_Stream4_IRQHandler() __attribute__ ((alias("PIOS_ADC_handler")));
const struct pios_adc_cfg pios_adc_cfg = {
	.dma = {
		.irq = {
			.handler = NULL,
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

/*
 * Telemetry USART
 */
const struct pios_usart_cfg pios_usart_telem_cfg = USART2_CONFIG(PIOS_COM_TELEM_BAUDRATE);

/*
 * GPS USART
 */
const struct pios_usart_cfg pios_usart_gps_cfg = USART6_CONFIG(PIOS_COM_GPS_BAUDRATE);

/*
 * AUX USART
 */
const struct pios_usart_cfg pios_usart_aux_cfg = USART1_CONFIG(PIOS_COM_AUX_BAUDRATE);

/*
 * Control USART
 */
const struct pios_usart_cfg pios_usart_control_cfg = UART5_CONFIG(PIOS_COM_CONTROL_BAUDRATE);


/*
 * PPM Input
 */
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
			.TIM_Channel = TIM_Channel_2,
	},
	.remap = GPIO_AF_TIM1,
	.port = GPIOA,
	.gpio_init = {
			.GPIO_Pin   = GPIO_Pin_9,
			.GPIO_Mode  = GPIO_Mode_AF,
			.GPIO_Speed = GPIO_Speed_2MHz,
			.GPIO_OType = GPIO_OType_OD,
			.GPIO_PuPd  = GPIO_PuPd_UP
	},
	.irq = {
		.handler = NULL,
		.init    = {
			.NVIC_IRQChannelPreemptionPriority = PIOS_IRQ_PRIO_MID,
			.NVIC_IRQChannelSubPriority        = 0,
			.NVIC_IRQChannelCmd                = ENABLE,
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

const struct pios_i2c_adapter_cfg pios_i2c_esc_adapter_cfg = I2C1_CONFIG();

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

const struct pios_i2c_adapter_cfg pios_i2c_main_adapter_cfg = I2C2_CONFIG();

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

const struct pios_i2c_adapter_cfg pios_i2c_external_adapter_cfg = I2C3_CONFIG();

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

uint32_t pios_rcvr_channel_to_id_map[PIOS_RCVR_MAX_DEVS];
uint32_t pios_rcvr_max_channel;

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
void PIOS_Board_Init(void)
{
	uint32_t	usart_id;

	/* Debug services */
	PIOS_DEBUG_Init();

	/* Delay system */
	PIOS_DELAY_Init();

	/* Initialise USARTs */
	if (PIOS_USART_Init(&usart_id, &pios_usart_telem_cfg) ||
		PIOS_COM_Init(&pios_com_telem_rf_id, &pios_usart_com_driver, usart_id)) {
		PIOS_DEBUG_Assert(0);
	}
	if (PIOS_USART_Init(&usart_id, &pios_usart_gps_cfg) ||
		PIOS_COM_Init(&pios_com_gps_id, &pios_usart_com_driver, usart_id)) {
		PIOS_DEBUG_Assert(0);
	}
	if (PIOS_USART_Init(&usart_id, &pios_usart_aux_cfg) ||
		PIOS_COM_Init(&pios_com_aux_id, &pios_usart_com_driver, usart_id)) {
		PIOS_DEBUG_Assert(0);
	}
	if (PIOS_USART_Init(&usart_id, &pios_usart_control_cfg) ||
		PIOS_COM_Init(&pios_com_control_id, &pios_usart_com_driver, usart_id)) {
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
