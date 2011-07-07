 /**
 ******************************************************************************
 *
 * @file       pios_board.h
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @brief      Defines board hardware for the OpenPilot Version 1.1 hardware.
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


#ifndef STM32100C6_PX2IO_H_
#define STM32100C6_PX2IO_H_

//------------------------
// BOOTLOADER_SETTINGS
//------------------------

// XXX currently no bootloader for this board

//#define FUNC_ID					2
//#define HW_VERSION				69

//#define BOOTLOADER_VERSION			0
#define BOARD_TYPE						0x03	// XXX pick a value
#define BOARD_REVISION					0x01
#define MEM_SIZE						(32 * 1024)
#define SIZE_OF_DESCRIPTION				50
#define START_OF_USER_CODE				(uint32_t)0x08000000	// due to no bootloader
#define SIZE_OF_CODE					(uint32_t)(MEM_SIZE-(START_OF_USER_CODE-0x08000000)-SIZE_OF_DESCRIPTION)
#define HW_TYPE							2	// low-density (this will probably break things)
#define BOARD_READABLE					TRUE
#define BOARD_WRITABLA					TRUE
#define MAX_DEL_RETRYS					3

//------------------------
// Clock configuration
//------------------------
#define PIOS_CLOCK_CONFIG				px2io_clock_config

//------------------------
// WATCHDOG_SETTINGS
//------------------------
#define PIOS_WATCHDOG_TIMEOUT   		250
#define PIOS_WDG_REGISTER       	 	BKP_DR4
// XXX define watchdog bits here

//------------------------
// PIOS_LED
//------------------------
#define PIOS_LED_LED1_GPIO_PORT			GPIOB
#define PIOS_LED_LED1_GPIO_PIN			GPIO_Pin_14
#define PIOS_LED_LED1_GPIO_CLK			RCC_APB2Periph_GPIOB
#define PIOS_LED_LED2_GPIO_PORT         GPIOB
#define PIOS_LED_LED2_GPIO_PIN          GPIO_Pin_15
#define PIOS_LED_LED2_GPIO_CLK          RCC_APB2Periph_GPIOB
#define PIOS_LED_LED3_GPIO_PORT         GPIOB
#define PIOS_LED_LED3_GPIO_PIN          GPIO_Pin_9
#define PIOS_LED_LED3_GPIO_CLK          RCC_APB2Periph_GPIOB
#define PIOS_LED_NUM                    3
#define PIOS_LED_PORTS                  { PIOS_LED_LED1_GPIO_PORT, PIOS_LED_LED2_GPIO_PORT, PIOS_LED_LED3_GPIO_PORT}
#define PIOS_LED_PINS                   { PIOS_LED_LED1_GPIO_PIN, PIOS_LED_LED2_GPIO_PIN, PIOS_LED_LED3_GPIO_PIN }
#define PIOS_LED_CLKS                   { PIOS_LED_LED1_GPIO_CLK, PIOS_LED_LED2_GPIO_CLK, PIOS_LED_LED3_GPIO_CLK }

//-------------------------
// Delay Timer
//-------------------------
#define PIOS_DELAY_TIMER				TIM3
#define PIOS_DELAY_TIMER_RCC_FUNC		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE)

//-------------------------
// System Settings
//-------------------------
#define PIOS_MASTER_CLOCK				24000000
#define PIOS_PERIPHERAL_CLOCK			(PIOS_MASTER_CLOCK / 2)
#if defined(USE_BOOTLOADER)
#define PIOS_NVIC_VECTTAB_FLASH			(START_OF_USER_CODE)
#else
#define PIOS_NVIC_VECTTAB_FLASH			((uint32_t)0x08000000)
#endif

//-------------------------
// Interrupt Priorities
//-------------------------
#define PIOS_IRQ_PRIO_LOW				12		// lower than RTOS
#define PIOS_IRQ_PRIO_MID				8		// higher than RTOS
#define PIOS_IRQ_PRIO_HIGH				5		// for SPI, ADC, I2C etc...
#define PIOS_IRQ_PRIO_HIGHEST			4 		// for USART etc...

//-------------------------
// PIOS_USART
//-------------------------
#define PIOS_USART_MAX_DEVS				2

#define PIOS_USART_RX_BUFFER_SIZE       32
#define PIOS_USART_TX_BUFFER_SIZE       32

//-------------------------
// PIOS_COM
//
// See also pios_board.c
//-------------------------
#define PIOS_COM_MAX_DEVS				2

#define PIOS_COM_AUX_BAUDRATE			115200
extern uint32_t pios_com_aux_id;
#define PIOS_COM_AUX					(pios_com_aux_id)
#define PIOS_COM_DEBUG                  PIOS_COM_AUX

#define PIOS_COM_SPEKTRUM_BAUDRATE      115200
extern uint32_t pios_com_spektrum_id;
#define PIOS_COM_SPEKTRUM               (pios_com_spektrum_id)

// XXX spektrum UART is also S.Bus UART

//-------------------------
// Receiver PPM input
//-------------------------
#define PIOS_PPM_GPIO_PORT				GPIOA
#define PIOS_PPM_GPIO_PIN				GPIO_Pin_8
#define PIOS_PPM_GPIO_REMAP				GPIO_AF_TIM1
#define PIOS_PPM_TIM_CHANNEL			TIM_Channel_1
#define PIOS_PPM_TIM_CCR				TIM_IT_CC1
#define PIOS_PPM_TIM					TIM1
#define PIOS_PPM_TIM_IRQ				TIM1_CC_IRQn
#define PIOS_PPM_NUM_INPUTS				8  //Could be more if needed

//-------------------------
// ADC
//-------------------------
//#define PIOS_ADC_OVERSAMPLING_RATE		1
#define PIOS_ADC_USE_TEMP_SENSOR			1
#define PIOS_ADC_TEMP_SENSOR_ADC			ADC1
#define PIOS_ADC_TEMP_SENSOR_ADC_CHANNEL	1

#define PIOS_ADC_PIN1_GPIO_PORT			GPIOB
#define PIOS_ADC_PIN1_GPIO_PIN			GPIO_Pin_0
#define PIOS_ADC_PIN1_GPIO_CHANNEL		ADC_Channel_4	// XXX probably wrong
#define PIOS_ADC_PIN1_ADC				ADC1			// XXX probably wrong
#define PIOS_ADC_PIN1_ADC_NUMBER		1				// XXX probably wrong

#define PIOS_ADC_PIN2_GPIO_PORT			GPIOB
#define PIOS_ADC_PIN2_GPIO_PIN			GPIO_Pin_1
#define PIOS_ADC_PIN2_GPIO_CHANNEL		ADC_Channel_5	// XXX probably wrong
#define PIOS_ADC_PIN2_ADC				ADC1			// XXX probably wrong
#define PIOS_ADC_PIN2_ADC_NUMBER		2				// XXX probably wrong

#define PIOS_ADC_NUM_PINS				2

#define PIOS_ADC_PORTS					{ PIOS_ADC_PIN1_GPIO_PORT, PIOS_ADC_PIN2_GPIO_PORT }
#define PIOS_ADC_PINS					{ PIOS_ADC_PIN1_GPIO_PIN, PIOS_ADC_PIN2_GPIO_PIN }
#define PIOS_ADC_CHANNELS				{ PIOS_ADC_PIN1_GPIO_CHANNEL, PIOS_ADC_PIN2_GPIO_CHANNEL }
#define PIOS_ADC_MAPPING				{ PIOS_ADC_PIN1_ADC, PIOS_ADC_PIN2_ADC }
#define PIOS_ADC_CHANNEL_MAPPING		{ PIOS_ADC_PIN1_ADC_NUMBER, PIOS_ADC_PIN2_ADC_NUMBER }
#define PIOS_ADC_NUM_CHANNELS			(PIOS_ADC_NUM_PINS + PIOS_ADC_USE_TEMP_SENSOR) 
#define PIOS_ADC_NUM_ADC_CHANNELS		2
#define PIOS_ADC_USE_ADC2				0
#define PIOS_ADC_CLOCK_FUNCTION			RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE)
#define PIOS_ADC_ADCCLK					RCC_PCLK2_Div8
/* RCC_PCLK2_Div2: ADC clock = PCLK2/2 */
/* RCC_PCLK2_Div4: ADC clock = PCLK2/4 */
/* RCC_PCLK2_Div6: ADC clock = PCLK2/6 */
/* RCC_PCLK2_Div8: ADC clock = PCLK2/8 */
#define PIOS_ADC_SAMPLE_TIME			ADC_SampleTime_239Cycles5
/* Sample time: */
/* With an ADCCLK = 14 MHz and a sampling time of 239.5 cycles: */
/* Tconv = 239.5 + 12.5 = 252 cycles = 18�s */
/* (1 / (ADCCLK / CYCLES)) = Sample Time (�S) */
#define PIOS_ADC_IRQ_PRIO				PIOS_IRQ_PRIO_LOW

// Currently analog acquisition hard coded at 480 Hz
// PCKL2 = HCLK / 16
// ADCCLK = PCLK2 / 2
#define PIOS_ADC_RATE					(24.0e6 / 1.0 / 8.0 / 252.0 / (PIOS_ADC_NUM_CHANNELS >> PIOS_ADC_USE_ADC2))
#define PIOS_ADC_MAX_OVERSAMPLING       36

//-------------------------
// Servo outputs
//-------------------------
#define PIOS_SERVO_UPDATE_HZ            50
#define PIOS_SERVOS_INITIAL_POSITION    0		// default to sending no signal

//-------------------------
// GPIO
//-------------------------
#define PIOS_GPIO_1_PORT				GPIOA					// Relay 1
#define PIOS_GPIO_1_PIN					GPIO_Pin_11
#define PIOS_GPIO_1_GPIO_CLK			RCC_APB2Periph_GPIOA

#define PIOS_GPIO_2_PORT				GPIOA					// Relay 2
#define PIOS_GPIO_2_PIN					GPIO_Pin_12
#define PIOS_GPIO_2_GPIO_CLK			RCC_APB2Periph_GPIOA

#define PIOS_GPIO_3_PORT				GPIOC					// Accessory 1
#define PIOS_GPIO_3_PIN					GPIO_Pin_13
#define PIOS_GPIO_3_GPIO_CLK			RCC_APB2Periph_GPIOC

#define PIOS_GPIO_4_PORT				GPIOC					// Accessory 2
#define PIOS_GPIO_4_PIN					GPIO_Pin_14
#define PIOS_GPIO_4_GPIO_CLK			RCC_APB2Periph_GPIOC

#define PIOS_GPIO_PORTS					{ PIOS_GPIO_1_PORT,     PIOS_GPIO_2_PORT,     PIOS_GPIO_3_PORT,     PIOS_GPIO_4_PORT }
#define PIOS_GPIO_PINS					{ PIOS_GPIO_1_PIN,      PIOS_GPIO_2_PIN,      PIOS_GPIO_3_PIN,      PIOS_GPIO_4_PIN}
#define PIOS_GPIO_CLKS					{ PIOS_GPIO_1_GPIO_CLK, PIOS_GPIO_2_GPIO_CLK, PIOS_GPIO_3_GPIO_CLK, PIOS_GPIO_4_GPIO_CLK, }
#define PIOS_GPIO_NUM					4

#endif /* STM32100C6_PX2IO_H_ */
