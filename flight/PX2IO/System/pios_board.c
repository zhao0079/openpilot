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
#include <pios_com_priv.h>
#include <pios_ppm_priv.h>
#include <pios_servo_priv.h>
#include <pios_spektrum_priv.h>
#include <pios_usart_priv.h>

/*
 * Clocking
 */
const struct pios_clock_cfg px2io_clock_config = {
		.source				= RCC_PLLSource_HSI_Div2,	/* XXX temporary until we sort out the clock on IO */
		.refclock_frequency = 8 * 1000 * 1000,
		.refclock_prescale	= 2,
		.pll_multiply		= 6,
		.hclk_prescale		= RCC_SYSCLK_Div1,
		.pclk1_prescale		= RCC_HCLK_Div1,
		.pclk2_prescale		= RCC_HCLK_Div1,
		.adc_prescale		= RCC_PCLK2_Div2,
};

/*
 * AUX USART
 */
void PIOS_USART_aux_irq_handler(void);
void USART1_IRQHandler() __attribute__ ((alias ("PIOS_USART_aux_irq_handler")));
const struct pios_usart_cfg pios_usart_aux_cfg = {
  .regs = USART1,
  .init = {
	.USART_BaudRate            = PIOS_COM_AUX_BAUDRATE,
    .USART_BaudRate            = 115200,
    .USART_WordLength          = USART_WordLength_8b,
    .USART_Parity              = USART_Parity_No,
    .USART_StopBits            = USART_StopBits_1,
    .USART_HardwareFlowControl = USART_HardwareFlowControl_None,
    .USART_Mode                = USART_Mode_Rx | USART_Mode_Tx,
  },
  .irq = {
    .handler = PIOS_USART_aux_irq_handler,
    .init    = {
      .NVIC_IRQChannel                   = USART1_IRQn,
      .NVIC_IRQChannelPreemptionPriority = PIOS_IRQ_PRIO_MID,
      .NVIC_IRQChannelSubPriority        = 0,
      .NVIC_IRQChannelCmd                = ENABLE,
    },
  },
  .remap = GPIO_Remap_USART1,
  .rx   = {
    .gpio = GPIOA,
    .init = {
      .GPIO_Pin   = GPIO_Pin_10,
      .GPIO_Speed = GPIO_Speed_2MHz,
      .GPIO_Mode  = GPIO_Mode_IPU,
    },
  },
  .tx   = {
    .gpio = GPIOA,
    .init = {
      .GPIO_Pin   = GPIO_Pin_9,
      .GPIO_Speed = GPIO_Speed_2MHz,
      .GPIO_Mode  = GPIO_Mode_AF_PP,
    },
  },
};

static uint32_t pios_usart_aux_id;
void PIOS_USART_aux_irq_handler(void)
{
	PIOS_USART_IRQ_Handler(pios_usart_aux_id);
}

#if PIOS_INCLUDE_SPEKTRUM
# error Need Spektrum config
#endif

/**
 * Pios servo configuration structures
 *
 * Note: early prototypes do not have the last two channels available,
 * as they are routed elsewhere (as ADC inputs).
 */
const struct pios_servo_channel pios_servo_channels[] = {
	{	// channel 1
		.timer = TIM2,
		.port = GPIOA,
		.channel = TIM_Channel_1,
		.pin = GPIO_Pin_0,
	}, 
	{	// channel 2
		.timer = TIM2,
		.port = GPIOA,
		.channel = TIM_Channel_2,
		.pin = GPIO_Pin_1,
	}, 
	{	// channel 3
		.timer = TIM16,
		.port = GPIOB,
		.channel = TIM_Channel_1,
		.pin = GPIO_Pin_8,
	}, 
	{	// channel 4
		.timer = TIM17,
		.port = GPIOB,
		.channel = TIM_Channel_1,
		.pin = GPIO_Pin_9,
	}, 
	{	// channel 5
		.timer = TIM3,
		.port = GPIOA,
		.channel = TIM_Channel_1,
		.pin = GPIO_Pin_6,
	}, 
	{	// channel 6
		.timer = TIM3,
		.port = GPIOA,
		.channel = TIM_Channel_2,
		.pin = GPIO_Pin_7,
	}, 
	{	// channel 7
		.timer = TIM3,
		.port = GPIOB,
		.channel = TIM_Channel_3,
		.pin = GPIO_Pin_0,
	}, 
	{	// channel 8
		.timer = TIM3,
		.port = GPIOB,
		.channel = TIM_Channel_4,
		.pin = GPIO_Pin_1,
	}, 	
};

const struct pios_servo_cfg pios_servo_cfg = {
	.tim_base_init = {
		.TIM_Prescaler = (PIOS_MASTER_CLOCK / 1000000) - 1,
		.TIM_ClockDivision = TIM_CKD_DIV1,
		.TIM_CounterMode = TIM_CounterMode_Up,
		.TIM_Period = ((1000000 / PIOS_SERVO_UPDATE_HZ) - 1),
		.TIM_RepetitionCounter = 0x0000,
	},
	.tim_oc_init = {
		.TIM_OCMode = TIM_OCMode_PWM1,
		.TIM_OutputState = TIM_OutputState_Enable,
		.TIM_OutputNState = TIM_OutputNState_Disable,
		.TIM_Pulse = PIOS_SERVOS_INITIAL_POSITION,		
		.TIM_OCPolarity = TIM_OCPolarity_High,
		.TIM_OCNPolarity = TIM_OCPolarity_High,
		.TIM_OCIdleState = TIM_OCIdleState_Reset,
		.TIM_OCNIdleState = TIM_OCNIdleState_Reset,
	},
	.gpio_init = {
		.GPIO_Mode = GPIO_Mode_AF_PP,
		.GPIO_Speed = GPIO_Speed_2MHz,
	},
	.remap = 0,
	.channels = pios_servo_channels,
	.num_channels = NELEMENTS(pios_servo_channels),
};


#ifdef PIOS_INCLUDE_PPM
/*
 * PPM Input
 */
void TIM6_IRQHandler();
void TIM6_IRQHandler() __attribute__ ((alias ("PIOS_TIM6_irq_handler")));
const struct pios_ppmsv_cfg pios_ppmsv_cfg = {
	.tim_base_init = {
		.TIM_Prescaler = (PIOS_MASTER_CLOCK / 1000000) - 1,	/* For 1 uS accuracy */
		.TIM_ClockDivision = TIM_CKD_DIV1,
		.TIM_CounterMode = TIM_CounterMode_Up,
		.TIM_Period = ((1000000 / 25) - 1), /* 25 Hz */
		.TIM_RepetitionCounter = 0x0000,
	},
	.irq = {
		.handler = TIM6_IRQHandler,
		.init    = {
			.NVIC_IRQChannelPreemptionPriority = PIOS_IRQ_PRIO_MID,
			.NVIC_IRQChannelSubPriority        = 0,
			.NVIC_IRQChannelCmd                = ENABLE,
		},
	},
	.timer = TIM6,
	.ccr = TIM_IT_Update,
};

void PIOS_TIM6_irq_handler()
{
	PIOS_PPMSV_irq_handler();
}

void TIM1_CC_IRQHandler();
void TIM1_CC_IRQHandler() __attribute__ ((alias ("PIOS_TIM1_CC_irq_handler")));
const struct pios_ppm_cfg pios_ppm_cfg = {
	.tim_base_init = {
		.TIM_Prescaler = (PIOS_MASTER_CLOCK / 1000000) - 1,	/* For 1 uS accuracy */
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
	.gpio_init = {
			.GPIO_Mode = GPIO_Mode_IPD,
			.GPIO_Speed = GPIO_Speed_2MHz,
			.GPIO_Pin = GPIO_Pin_9,
	},
	.remap = 0,
	.irq = {
		.handler = TIM1_CC_IRQHandler,
		.init    = {
			.NVIC_IRQChannelPreemptionPriority = PIOS_IRQ_PRIO_MID,
			.NVIC_IRQChannelSubPriority        = 0,
			.NVIC_IRQChannelCmd                = ENABLE,
			.NVIC_IRQChannel = TIM1_CC_IRQn,
		},
	},
	.timer = TIM1,
	.port = GPIOA,
	.ccr = TIM_IT_CC2,
};

void PIOS_TIM1_CC_irq_handler()
{
	PIOS_PPM_irq_handler();
}
#endif

uint32_t pios_com_aux_id;
#if defined(PIOS_INCLUDE_SPEKTRUM)
uint32_t pios_com_spektrum_id;
#endif

/**
 * PIOS_Board_Init()
 */
void PIOS_Board_Init(void) {

	/* Debug services */
	PIOS_DEBUG_Init();

	/* Delay system */
	PIOS_DELAY_Init();	
	
#if defined(PIOS_INCLUDE_SPEKTRUM)
	/* SPEKTRUM init must come before comms */
	PIOS_SPEKTRUM_Init();

	if (PIOS_USART_Init(&pios_usart_spektrum_id, &pios_usart_spektrum_cfg)) {
		PIOS_DEBUG_Assert(0);
	}
	if (PIOS_COM_Init(&pios_com_spektrum_id, &pios_usart_com_driver, pios_usart_spektrum_id)) {
		PIOS_DEBUG_Assert(0);
	}
#endif

#if defined(PIOS_COM_AUX)
	if (PIOS_USART_Init(&pios_usart_aux_id, &pios_usart_aux_cfg)) {
		PIOS_DEBUG_Assert(0);
	}
	if (PIOS_COM_Init(&pios_com_aux_id, &pios_usart_com_driver, pios_usart_aux_id)) {
		PIOS_DEBUG_Assert(0);
	}
#endif
	PIOS_COM_SendFormattedString(PIOS_COM_AUX, "PX2IO starting...\r\n");

	PIOS_Servo_Init();
	PIOS_GPIO_Init();

#if defined(PIOS_INCLUDE_PPM)
	PIOS_PPM_Init();
#endif

	PIOS_WDG_Init();
}

/**
 * @}
 */
