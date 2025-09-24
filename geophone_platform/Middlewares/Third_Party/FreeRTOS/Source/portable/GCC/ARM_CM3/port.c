#ifndef PORTMACRO_H
#define PORTMACRO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*-----------------------------------------------------------
 * Port specific definitions.
 *
 * The settings in this file configure FreeRTOS correctly for the
 * given hardware and compiler.
 *
 * These settings should not be altered.
 *-----------------------------------------------------------
 */

/* Type definitions. */
#define portCHAR		char
#define portFLOAT		float
#define portDOUBLE		double
#define portLONG		long
#define portSHORT		short
#define portSTACK_TYPE	uint32_t
#define portBASE_TYPE	long

typedef portSTACK_TYPE StackType_t;
typedef long BaseType_t;
typedef unsigned long UBaseType_t;

#if( configUSE_16_BIT_TICKS == 1 )
	typedef uint16_t TickType_t;
	#define portMAX_DELAY ( TickType_t ) 0xffff
#else
	typedef uint32_t TickType_t;
	#define portMAX_DELAY ( TickType_t ) 0xffffffffUL

	/* 32-bit tick type on a 32-bit architecture, so reads of the tick count do
	not need to be guarded with a critical section. */
	#define portTICK_TYPE_IS_ATOMIC 1
#endif
/*-----------------------------------------------------------*/

/* Architecture specifics. */
#define portSTACK_GROWTH			( -1 )
#define portTICK_PERIOD_MS			( ( TickType_t ) 1000 / configTICK_RATE_HZ )
#define portBYTE_ALIGNMENT			8
/*-----------------------------------------------------------*/

/* Scheduler utilities. */
#define portYIELD() 															\
{																				\
	/* Set a PendSV to request a context switch. */								\
	portNVIC_INT_CTRL_REG = portNVIC_PENDSVSET_BIT;								\
																				\
	/* Barriers are normally not required but do ensure the code is completely	\
	completely within the specified behaviour for the architecture. */			\
	__asm volatile( "dsb" ::: "memory" );										\
	__asm volatile( "isb" );													\
}

#define portNVIC_INT_CTRL_REG		( * ( ( volatile uint32_t * ) 0xe000ed04 ) )
#define portNVIC_PENDSVSET_BIT		( 1UL << 28UL )
#define portEND_SWITCHING_ISR( xSwitchRequired ) if( xSwitchRequired != pdFALSE ) portYIELD()
#define portYIELD_FROM_ISR( x ) portEND_SWITCHING_ISR( x )
/*-----------------------------------------------------------*/

/* Critical section management. */
extern void vPortEnterCritical( void );
extern void vPortExitCritical( void );

#define portDISABLE_INTERRUPTS()				ulPortSetInterruptMask()
#define portENABLE_INTERRUPTS()					vPortClearInterruptMask( 0 )
#define portENTER_CRITICAL()					vPortEnterCritical()
#define portEXIT_CRITICAL()						vPortExitCritical()
#define portSET_INTERRUPT_MASK_FROM_ISR()		ulPortSetInterruptMask()
#define portCLEAR_INTERRUPT_MASK_FROM_ISR(x)	vPortClearInterruptMask(x)

/*-----------------------------------------------------------*/

/* Tickless idle/low power functionality. */
#ifndef portSUPPRESS_TICKS_AND_SLEEP
	extern void vPortSuppressTicksAndSleep( TickType_t xExpectedIdleTime );
	#define portSUPPRESS_TICKS_AND_SLEEP( xExpectedIdleTime ) vPortSuppressTicksAndSleep( xExpectedIdleTime )
#endif
/*-----------------------------------------------------------*/

/* Task function macros as described on the FreeRTOS.org WEB site.  These are
not necessary for to use this port.  They are defined so the common demo files
(which build with all the ports) will build. */
#define portTASK_FUNCTION_PROTO( vFunction, pvParameters ) void vFunction( void *pvParameters )
#define portTASK_FUNCTION( vFunction, pvParameters ) void vFunction( void *pvParameters )

/* Prototype of the FreeRTOS MPU ports. */
extern BaseType_t xIsPrivileged( void );

/* MPU regions. */
#define portUNPRIVILEGED_FLASH_REGION		( 0UL )
#define portPRIVILEGED_FLASH_REGION			( 1UL )
#define portPRIVILEGED_RAM_REGION			( 2UL )
#define portFIRST_CONFIGURABLE_REGION		( 3UL )
#define portLAST_CONFIGURABLE_REGION		( portFIRST_CONFIGURABLE_REGION + configTOTAL_MPU_REGIONS - 1UL )
#define portNUM_CONFIGURABLE_REGIONS		( ( portLAST_CONFIGURABLE_REGION - portFIRST_CONFIGURABLE_REGION ) + 1 )
#define portTOTAL_NUM_REGIONS				( portNUM_CONFIGURABLE_REGIONS + 1 ) /* Plus one to make space for the stack region. */

/* Device memory attributes used in MPU_MAIR registers.
 *
 * 8-bit values can be written to MPU_MAIR registers armv7-m.
 */
#define portMPU_NORMAL_MEMORY_NON_CACHEABLE				( 0x05 ) /* Normal memory, Non-cacheable, Bufferable */
#define portMPU_DEVICE_MEMORY_nGnRnE					( 0x00 ) /* Device memory, nGnRnE */

/* Attributes used in MPU_MAIR registers. */
#define portMPU_MAIR_ATTRIBUTE_FLASH					( portMPU_NORMAL_MEMORY_NON_CACHEABLE << ( portPRIVILEGED_FLASH_REGION * 8UL ) )
#define portMPU_MAIR_ATTRIBUTE_SRAM						( portMPU_NORMAL_MEMORY_NON_CACHEABLE << ( portPRIVILEGED_RAM_REGION * 8UL ) )
#define portMPU_MAIR_ATTRIBUTE_STACK					( portMPU_NORMAL_MEMORY_NON_CACHEABLE << ( portSTACK_REGION * 8UL ) )

/* Check if the MPU is present. */
#if( configUSE_MPU == 1 )
	#define portHAS_MPU 1
#else
	#define portHAS_MPU 0
#endif

/* The maximum 24-bit number.  It is needed because the uxTopUsedPriority
variable is declared as a 24-bit type to save space but cannot contain
the 32-bit top bit. */
#define portMAX_24_BIT_NUMBER				( 0xffffffUL )

#if( configUSE_PORT_OPTIMISED_TASK_SELECTION == 1 )
	/* Store/clear the ready priorities in a bit map.  The number of bits
	in the bit map is 24 as configMAX_PRIORITIES is one less than 32. */
	#define portRECORD_READY_PRIORITY( uxPriority, uxReadyPriorities ) ( uxReadyPriorities ) |= ( 1UL << ( uxPriority ) )
	#define portRESET_READY_PRIORITY( uxPriority, uxReadyPriorities ) ( uxReadyPriorities ) &= ~( 1UL << ( uxPriority ) )
	#define portGET_HIGHEST_PRIORITY( uxTopReadyPriority, uxReadyPriorities ) uxTopReadyPriority = ( portMAX_24_BIT_NUMBER - __builtin_clz( ( uxReadyPriorities ) ) )
#endif /* configUSE_PORT_OPTIMISED_TASK_SELECTION */

/*-----------------------------------------------------------*/

/* Only used when running in privileged mode.  Defines the high bit of the
priority mask that is used to determine whether or not a context switch
is required following a nested ISR.  If confiMAX_SYSCALL_INTERRUPT_PRIORITY
is set to 0 then nested ISRs must call portNestedFreeRTOSAPI() on exit to
ensure a context switch is performed if necessary.  See the demo application
for the Parrot AR.Drone for an example of how this is used. */
#define portPRIORITY_SHIFT (( 8 ) - configPRIO_BITS )

/* portYIELD_WITHIN_API() will not yield if the yield is disabled. */
static UBaseType_t xYieldDisable = pdFALSE;

portFORCE_INLINE static void vPortEnableVFP( void )
{
	__asm volatile
	(
		"	ldr.w r0, =0xE000ED88		\n" /* The FPU enable bits are in the CPACR. */
		"	ldr r1, [r0]				\n"
		"	orr r1, r1, #( 0xf << 20 )	\n" /* Enable CP10 and CP11 co-processors. */
		"	str r1, [r0]				\n"
		"	bx lr						\n"
		"	nop							\n"
	);
}

/*-----------------------------------------------------------*/

/* See header file for description. */
BaseType_t xPortIsAuthorizedToRequestOperation( void );

/*-----------------------------------------------------------*/

#define portNO_OPERATION ( ( StackType_t ) 0 )

/* See header file for description. */
StackType_t *pxPortInitialiseStack( StackType_t *pxTopOfStack, TaskFunction_t pxCode, void *pvParameters, BaseType_t xRunPrivileged )
{
StackType_t * pxOriginalTOS;

	pxOriginalTOS = pxTopOfStack;

	/* Simulate the stack frame as it would be created by a context switch
	interrupt. */

	/* Offset added to account for the way the MCU uses the stack on entry/exit
	of interrupts, and to ensure alignment. */
	pxTopOfStack -= 2;

	*pxTopOfStack = portINITIAL_XPSR;	/* xPSR */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) pxCode;	/* PC */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) portTASK_RETURN_ADDRESS;	/* LR */

	/* Save code space by skipping register initialisation. */
	pxTopOfStack -= 5;	/* R12, R3, R2 and R1. */
	*pxTopOfStack = ( StackType_t ) pvParameters;	/* R0 */

	/* A save of the registers that the compiler itself is saving is repeated
	several times solely to align the stack. */
	pxTopOfStack -= 8;	/* R11, R10, R9, R8, R7, R6, R5 and R4. */

	/* The MPU region registers are initialised to 0, which means no access
	permissions.  This being the case it is not necessary to save and restore
	them when on the stack of a task that is not running in privileged mode. */
	#if( configENFORCE_SYSTEM_CALLS_FROM_KERNEL_ONLY == 1 )
		pxTopOfStack -= 8;	/* MPU region registers. */
	#else
		if( xRunPrivileged == pdTRUE )
		{
			pxTopOfStack -= 8;	/* MPU region registers. */
		}
	#endif /* configENFORCE_SYSTEM_CALLS_FROM_KERNEL_ONLY */

	pxTopOfStack--;
	*pxTopOfStack = portINITIAL_CONTROL_IF_UNPRIVILEGED;
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) pxOriginalTOS; /* Stack used when calling taskYIELD_FROM_ISR (). */

	/* If configENFORCE_SYSTEM_CALLS_FROM_KERNEL_ONLY is set then system calls
	from user tasks are not permitted - so no need to initialise the EXC_RETURN
	value on the stack of a user task. */
	#if( configENFORCE_SYSTEM_CALLS_FROM_KERNEL_ONLY == 1 )
		( void ) xRunPrivileged;
	#else
		if( xRunPrivileged == pdTRUE )
		{
			pxTopOfStack--;
			*pxTopOfStack = portINITIAL_EXC_RETURN;
		}
	#endif

	return pxTopOfStack;
}

/*-----------------------------------------------------------*/

/* The real vPortSVCHandler is naked, so its prototype goes in portasm.s. */
extern void vPortSVCHandler( void );

/*-----------------------------------------------------------*/

/* This is a naked function. */
static void prvPortStartFirstTask( void )
{
	__asm volatile(
					" ldr r0, =0xE000ED08 	\n" /* Use the NVIC offset register to locate the stack. */
					" ldr r0, [r0] 			\n"
					" ldr r0, [r0] 			\n"
					" msr msp, r0			\n" /* Set the msp back to the start of the stack. */
					" mov r0, #0			\n" /* Clear the bit that indicates the FPU is in use, see comment above. */
					" msr control, r0		\n"
					" cpsie i				\n" /* Globally enable interrupts. */
					" cpsie f				\n"
					" dsb					\n"
					" isb					\n"
					" svc 0					\n" /* System call to start first task. */
					" nop					\n"
				);
}

/*-----------------------------------------------------------*/

void vPortEnterCritical( void )
{
	portDISABLE_INTERRUPTS();
	uxCriticalNesting++;

	/* This is not the interrupt safe version of the enter critical function so
	assert() if it is being called from an interrupt context.  Only API
	functions that end in "FromISR" can be used in an interrupt.  Only assert if
	the critical nesting count is 1 to protect against recursive calls if the
	assert function also uses a critical section. */
	if( uxCriticalNesting == 1 )
	{
		configASSERT( ( portNVIC_INT_CTRL_REG & portVECTACTIVE_MASK ) == 0 );
	}
}

/*-----------------------------------------------------------*/

void vPortExitCritical( void )
{
	configASSERT( uxCriticalNesting );
	uxCriticalNesting--;
	if( uxCriticalNesting == 0 )
	{
		portENABLE_INTERRUPTS();
	}
}

/*-----------------------------------------------------------*/

uint32_t ulPortSetInterruptMask( void )
{
uint32_t ulOriginalMask;

	__asm volatile( "mrs %0, basepri" : "=r" (ulOriginalMask) :: "memory" );

	__asm volatile( "msr basepri, %0" :: "r" ( configMAX_SYSCALL_INTERRUPT_PRIORITY ) : "memory" );

	return ulOriginalMask;
}

/*-----------------------------------------------------------*/

void vPortClearInterruptMask( uint32_t ulNewMaskValue )
{
	__asm volatile( "msr basepri, %0" :: "r" ( ulNewMaskValue ) : "memory" );
}

/*-----------------------------------------------------------*/

void vPortSetupTimerInterrupt( void )
{
	/* Calculate the constants required to configure the tick interrupt. */
	#if( configUSE_TICKLESS_IDLE == 1 )
	{
		ulTimerCountsForOneTick = ( configSYSTICK_CLOCK_HZ / configTICK_RATE_HZ );
		xMaximumPossibleSuppressedTicks = portMAX_24_BIT_NUMBER / ulTimerCountsForOneTick;
		ulStoppedTimerCompensation = portMISSED_COUNTS_FACTOR / ( configCPU_CLOCK_HZ / configSYSTICK_CLOCK_HZ );
	}
	#endif /* configUSE_TICKLESS_IDLE */

	/* Stop and clear the SysTick. */
	portNVIC_SYSTICK_CTRL_REG = 0UL;
	portNVIC_SYSTICK_CURRENT_VALUE_REG = 0UL;

	/* Configure SysTick to interrupt at the requested rate. */
	portNVIC_SYSTICK_LOAD_REG = ( configSYSTICK_CLOCK_HZ / configTICK_RATE_HZ ) - 1UL;
	portNVIC_SYSTICK_CTRL_REG = ( portNVIC_SYSTICK_CLK_BIT | portNVIC_SYSTICK_INT_BIT | portNVIC_SYSTICK_ENABLE_BIT );
}

/*-----------------------------------------------------------*/

uint32_t vPortGetIPSR( void )
{
uint32_t ulIPSR;

	__asm volatile( "mrs %0, ipsr" : "=r" (ulIPSR) );

	return ulIPSR;
}

/*-----------------------------------------------------------*/

#if( configASSERT_DEFINED == 1 )

	void vPortValidateInterruptPriority( void )
	{
	uint32_t ulCurrentInterrupt;
	uint8_t ucCurrentPriority;

		/* Obtain the number of the currently executing interrupt. */
		ulCurrentInterrupt = vPortGetIPSR();

		/* Is the interrupt number a user defined interrupt? */
		if( ulCurrentInterrupt >= portFIRST_USER_INTERRUPT_NUMBER )
		{
			/* Look up the interrupt's priority. */
			ucCurrentPriority = pcInterruptPriorityRegisters[ ulCurrentInterrupt ];

			/* The following assertion will fail if a service routine (ISR) for
			an interrupt that has been assigned a priority above
			configMAX_SYSCALL_INTERRUPT_PRIORITY calls an ISR safe FreeRTOS API
			function.  ISR safe FreeRTOS API functions must *only* be called
			from interrupts that have been assigned a priority at or below
			configMAX_SYSCALL_INTERRUPT_PRIORITY.

			Numerically low interrupt priority numbers represent logically high
			interrupt priorities, therefore the priority of the interrupt must
			be set to a value equal to or numerically *higher* than
			configMAX_SYSCALL_INTERRUPT_PRIORITY.

			Interrupts that	use the FreeRTOS API must not be left at their
			default priority of	zero as that is the highest possible priority,
			which is guaranteed to be above configMAX_SYSCALL_INTERRUPT_PRIORITY,
			and	therefore also guaranteed to be invalid.

			FreeRTOS maintains separate thread and ISR API functions to ensure
			interrupt entry is as fast and simple as possible.

			The following links provide detailed information:
			http://www.FreeRTOS.org/RTOS-Cortex-M3-M4.html
			http://www.FreeRTOS.org/FAQHelp.html */
			configASSERT( ucCurrentPriority >= ucMaxSysCallPriority );
		}

		/* Priority grouping:  The interrupt controller (NVIC) allows the bits
		that define each interrupt's priority to be split between bits that
		define the interrupt's pre-emption priority bits and bits that define
		the interrupt's sub-priority.  For simplicity all bits must be defined
		to be pre-emption priority bits.  The following assertion will fail if
		this is not the case (if some bits represent a sub-priority).

		If the application only uses CMSIS libraries for interrupt
		configuration then the correct setting can be achieved on all Cortex-M
		devices by calling NVIC_SetPriorityGrouping( 0 ); before starting the
		scheduler.  Note however that some vendor specific peripheral libraries
		assume a non-zero priority group setting, in which cases using a value
		of zero will result in unpredictable behaviour. */
		configASSERT( ( portAIRCR_REG & portPRIORITY_GROUP_MASK ) <= ulMaxPRIGROUPValue );
	}

#endif /* configASSERT_DEFINED */
