#ifndef PORTMACRO_H
#define PORTMACRO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Type definitions. */
#define portNVIC_SYSTICK_CLK_BIT  ( 1UL << 2UL )  // Processor clock (AHB) for STM32F103
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
	within the specified behaviour for the architecture. */						\
	__asm volatile( "dsb" ::: "memory" );										\
	__asm volatile( "isb" );													\
}

#define portNVIC_INT_CTRL_REG		( * ( ( volatile uint32_t * ) 0xe000ed04 ) )
#define portNVIC_PENDSVSET_BIT		( 1UL << 28UL )
#define portEND_SWITCHING_ISR( xSwitchRequired ) if( xSwitchRequired != pdFALSE ) portYIELD()
#define portYIELD_FROM_ISR( x ) portEND_SWITCHING_ISR( x )
/*-----------------------------------------------------------*/

/* Critical section management. */
#define portDISABLE_INTERRUPTS()				vPortRaiseBASEPRI()
#define portENABLE_INTERRUPTS()					vPortSetBASEPRI( 0 )
#define portENTER_CRITICAL()					vPortEnterCritical()
#define portEXIT_CRITICAL()						vPortExitCritical()
#define portSET_INTERRUPT_MASK_FROM_ISR()		ulPortRaiseBASEPRI()
#define portCLEAR_INTERRUPT_MASK_FROM_ISR(x)	vPortSetBASEPRI(x)

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

/* Prototype of the FreeRTOS tick handler.  This must be installed as the
handler for whichever peripheral is used to generate the RTOS tick. */
void xPortSysTickHandler( void );

/* Any task that uses the floating point unit MUST call vPortTaskUsesFPU()
before any floating point instructions are executed. */
void vPortTaskUsesFPU( void );
#define portTASK_USES_FLOATING_POINT() vPortTaskUsesFPU()

#define portLOWEST_INTERRUPT_PRIORITY ( ( ( unsigned char ) 0xff ) << (8 - configPRIO_BITS) )
#define portLOWEST_USABLE_INTERRUPT_PRIORITY ( portLOWEST_INTERRUPT_PRIORITY - 1 )

/* Architecture specific optimisations. */
#ifndef configUSE_PORT_OPTIMISED_TASK_SELECTION
	#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1
#endif

#if configUSE_PORT_OPTIMISED_TASK_SELECTION == 1

	/* Check the configuration. */
	#if( configMAX_PRIORITIES > 32 )
		#error configUSE_PORT_OPTIMISED_TASK_SELECTION can only be set to 1 when configMAX_PRIORITIES is less than or equal to 32.  It is very rare that a system requires more than 10 to 15 different priorities as tasks that share a priority will time slice.
	#endif

	/* Store/clear the ready priorities in a bit map. */
	#define portRECORD_READY_PRIORITY( uxPriority, uxReadyPriorities ) ( uxReadyPriorities ) |= ( 1UL << ( uxPriority ) )
	#define portRESET_READY_PRIORITY( uxPriority, uxReadyPriorities ) ( uxReadyPriorities ) &= ~( 1UL << ( uxPriority ) )

	/*-----------------------------------------------------------*/

	#define portGET_HIGHEST_PRIORITY( uxTopReadyPriority, uxReadyPriorities ) uxTopReadyPriority = ( 31UL - ( uint32_t ) __builtin_clz( ( uxReadyPriorities ) ) )

#endif /* configUSE_PORT_OPTIMISED_TASK_SELECTION */

#ifdef configASSERT
	void vPortValidateInterruptPriority( void );
	#define portASSERT_IF_INTERRUPT_PRIORITY_INVALID() 	vPortValidateInterruptPriority()
#endif

/* The number of bits to shift to get the priority from the IPSR value. */
#define portAPSR_PRIORITY_SHIFT		( 8UL - configPRIO_BITS )

#define portPRIGROUP_MASK			( 0x07UL << 8UL )
#define portPRIGROUP_SHIFT			( 8UL )

/* portSY_FULL_READ_WRITE is used as 3rd parameter to both DSB and ISB,
ensuring sufficient amount of instructions can execute between read and write
accesses. */
#define portSY_FULL_READ_WRITE		( 15UL )

/* Constants required to handle critical sections. */
#define portNO_CRITICAL_NESTING		( ( uint32_t ) 0 )

/* The variable used to hold the critical nesting value. */
extern volatile uint32_t ulCriticalNesting;

/* Inline functions to fix the errors. */
static inline void vPortRaiseBASEPRI( void )
{
uint32_t ulNewBASEPRI = configMAX_SYSCALL_INTERRUPT_PRIORITY;

	__asm volatile
	(
		"	msr basepri, %0	" :: "r" ( ulNewBASEPRI ) : "memory"
	);
	__asm volatile( "dsb" ::: "memory" );
	__asm volatile( "isb" ::: "memory" );
}

static inline void vPortSetBASEPRI( uint32_t ulBASEPRI )
{
	__asm volatile
	(
		"	msr basepri, %0	" :: "r" ( ulBASEPRI ) : "memory"
	);
}

static inline uint32_t ulPortRaiseBASEPRI( void )
{
uint32_t ulReturn, ulNewBASEPRI = configMAX_SYSCALL_INTERRUPT_PRIORITY;

	__asm volatile
	(
		"	mrs %0, basepri	" : "=r" ( ulReturn ) : : "memory"
	);
	__asm volatile
	(
		"	msr basepri, %0	" :: "r" ( ulNewBASEPRI ) : "memory"
	);
	__asm volatile( "dsb" ::: "memory" );
	__asm volatile( "isb" ::: "memory" );

	return ulReturn;
}

static inline void vPortEnterCritical( void )
{
	vPortRaiseBASEPRI();
	ulCriticalNesting++;
}

static inline void vPortExitCritical( void )
{
	if( ulCriticalNesting > portNO_CRITICAL_NESTING )
	{
		ulCriticalNesting--;
		if( ulCriticalNesting == portNO_CRITICAL_NESTING )
		{
			vPortSetBASEPRI( 0 );
		}
	}
}

#ifdef __cplusplus
}
#endif

#endif /* PORTMACRO_H */
