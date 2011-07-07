
#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * THESE PARAMETERS ARE DESCRIBED WITHIN THE 'CONFIGURATION' SECTION OF THE
 * FreeRTOS API DOCUMENTATION AVAILABLE ON THE FreeRTOS.org WEB SITE. 
 *
 * See http://www.freertos.org/a00110.html.
 *----------------------------------------------------------*/

/**
  * @addtogroup PIOS PIOS
  * @{
  * @addtogroup FreeRTOS FreeRTOS
  * @{
  */

#define configCPU_CLOCK_HZ					(24UL * 1000 * 1000)		// should be PIOS_MASTER_CLOCK
#define configTICK_RATE_HZ					((portTickType)250)
#define configMAX_PRIORITIES				((unsigned portBASE_TYPE)5)
#define configMINIMAL_STACK_SIZE			((unsigned short )256)
#define configTOTAL_HEAP_SIZE				((size_t)(1 * 1024))
#define configMAX_TASK_NAME_LEN				(8)

#define configUSE_PREEMPTION				1
#define configUSE_IDLE_HOOK					0
#define configUSE_TICK_HOOK					0
#define configUSE_CO_ROUTINES 				0
#define configUSE_16_BIT_TICKS				0

/* Set the following definitions to 1 to include the API function, or zero
to exclude the API function. */

#define INCLUDE_vTaskPrioritySet			0
#define INCLUDE_uxTaskPriorityGet			0
#define INCLUDE_vTaskDelete					0
#define INCLUDE_vTaskCleanUpResources		0
#define INCLUDE_vTaskSuspend				0
#define INCLUDE_vTaskDelayUntil				0
#define INCLUDE_vTaskDelay					1

/* This is the raw value as per the Cortex-M3 NVIC.  Values can be 255
(lowest) to 1 (highest maskable) to 0 (highest non-maskable). */
#define configKERNEL_INTERRUPT_PRIORITY 		15 << 4	/* equivalent to NVIC priority 15 */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY 	 3 << 4	/* equivalent to NVIC priority  3 */

/* This is the value being used as per the ST library which permits 16
priority values, 0 to 15.  This must correspond to the
configKERNEL_INTERRUPT_PRIORITY setting.  Here 15 corresponds to the lowest
NVIC value of 255. */
#define configLIBRARY_KERNEL_INTERRUPT_PRIORITY	15

/**
  * @}
  */

#endif /* FREERTOS_CONFIG_H */
