#ifndef PORTMACRO_H
#define PORTMACRO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Type definitions. */
#define portCHAR        char
#define portFLOAT       float
#define portDOUBLE      double
#define portLONG        long
#define portSHORT       short
#define portSTACK_TYPE  uint32_t
#define portBASE_TYPE   long

typedef portSTACK_TYPE StackType_t;
typedef long BaseType_t;
typedef unsigned long UBaseType_t;

#if( configUSE_16_BIT_TICKS == 1 )
typedef uint16_t TickType_t;
#define portMAX_DELAY ( TickType_t ) 0xffff
#else
typedef uint32_t TickType_t;
#define portMAX_DELAY ( TickType_t ) 0xffffffffUL
#endif

/*-----------------------------------------------------------*/

/* Architecture specifics. */
#define portSTACK_GROWTH			( -1 )
#define portTICK_PERIOD_MS			( ( TickType_t ) 1000 / configTICK_RATE_HZ )
#define portBYTE_ALIGNMENT			8

/*-----------------------------------------------------------*/

/* Scheduler utilities. */
#define portYIELD() 										\
{															\
	/* Set a PendSV to request a context switch. */		\
	portNVIC_INT_CTRL_REG = portNVIC_PENDSVSET_BIT;			\
	/* Barriers are normally not required but do ensure the code is completely	\
	within the specified behaviour for the architecture. */	\
	__asm volatile( "dsb" ::: "memory" );					\
	__asm volatile( "isb" );								\
}

#define portNVIC_INT_CTRL_REG		( * ( ( volatile uint32_t * ) 0xe000ed04 ) )
#define portNVIC_PENDSVSET_BIT		( 1UL << 28UL )

#define portEND_SWITCHING_ISR( xSwitchRequired ) if( xSwitchRequired != pdFALSE ) portYIELD()
#define portYIELD_FROM_ISR( x ) portEND_SWITCHING_ISR( x )

/*-----------------------------------------------------------*/

/* Critical section management. */
extern void vPortEnterCritical( void );
extern void vPortExitCritical( void );

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

/* Port specific optimisations. */
#ifndef configUSE_PORT_OPTIMISED_TASK_SELECTION
	#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1
#endif

#if configUSE_PORT_OPTIMISED_TASK_SELECTION == 1

	/* Check the configuration. */
	#if( configMAX_PRIORITIES > 32 )
		#error configUSE_PORT_OPTIMISED_TASK_SELECTION can only be set to 1 when configMAX_PRIORITIES is less than or equal to 32.  It is very rare that a system requires more than 10 to 15 difference priorities as tasks that share a priority will time slice.
	#endif

	/* Store/clear the ready priorities in a bit map. */
	#define portRECORD_READY_PRIORITY( uxPriority, uxReadyPriorities ) ( uxReadyPriorities ) |= ( 1UL << ( uxPriority ) )
	#define portRESET_READY_PRIORITY( uxPriority, uxReadyPriorities ) ( uxReadyPriorities ) &= ~( 1UL << ( uxPriority ) )

	/*-----------------------------------------------------------*/

	#define portGET_HIGHEST_PRIORITY( uxTopReadyPriority, uxReadyPriorities ) uxTopReadyPriority = ( 31UL - ( uint32_t ) __builtin_clz( ( uxReadyPriorities ) ) )

#endif /* configUSE_PORT_OPTIMISED_TASK_SELECTION */

/*-----------------------------------------------------------*/

/* Task function macros as described on the FreeRTOS.org WEB site.  These are
not necessary for to use this port.  They are defined so the common demo files
(which build with all the ports) will build. */
#define portTASK_FUNCTION_PROTO( vFunction, pvParameters ) void vFunction( void *pvParameters )
#define portTASK_FUNCTION( vFunction, pvParameters ) void vFunction( void *pvParameters )

/*-----------------------------------------------------------*/

#ifdef configASSERT
	void vPortValidateInterruptPriority( void );
	#define portASSERT_IF_INTERRUPT_PRIORITY_INVALID() 	vPortValidateInterruptPriority()
#endif

/* portNOP() is not required by this port. */
#define portNOP()

#define portINLINE __inline

#ifndef portFORCE_INLINE
	#define portFORCE_INLINE inline __attribute__(( always_inline ))
#endif

portFORCE_INLINE static BaseType_t xPortIsInsideInterrupt( void )
{
uint32_t ulCurrentInterrupt;
BaseType_t xReturn;

	/* Obtain the number of the currently executing interrupt. */
	__asm volatile( "mrs %0, ipsr" : "=r"( ulCurrentInterrupt ) :: "memory" );

	if( ulCurrentInterrupt == 0 )
	{
		xReturn = pdFALSE;
	}
	else
	{
		xReturn = pdTRUE;
	}

	return xReturn;
}

/*-----------------------------------------------------------*/

portFORCE_INLINE static void vPortRaiseBASEPRI( void )
{
uint32_t ulNewBASEPRI;

	__asm volatile
	(
		"	mov %0, %1												\n"	\
		"	msr basepri, %0										\n" \
		"	dsb														\n" \
		"	isb														\n" \
		: "=r" (ulNewBASEPRI) : "i" ( configMAX_SYSCALL_INTERRUPT_PRIORITY ) : "memory"
	);
}

/*-----------------------------------------------------------*/

portFORCE_INLINE static uint32_t ulPortRaiseBASEPRI( void )
{
uint32_t ulOriginalBASEPRI, ulNewBASEPRI;

	__asm volatile
	(
		"	mrs %0, basepri										\n" \
		"	mov %1, %2												\n"	\
		"	msr basepri, %1										\n" \
		"	dsb														\n" \
		"	isb														\n" \
		: "=r" (ulOriginalBASEPRI), "=r" (ulNewBASEPRI) : "i" ( configMAX_SYSCALL_INTERRUPT_PRIORITY ) : "memory"
	);

	/* This return will not be reached but is necessary to prevent compiler
	warnings. */
	return ulOriginalBASEPRI;
}
/*-----------------------------------------------------------*/

portFORCE_INLINE static void vPortSetBASEPRI( uint32_t ulNewMaskValue )
{
	__asm volatile
	(
		"	msr basepri, %0	" :: "r" ( ulNewMaskValue ) : "memory"
	);
}
/*-----------------------------------------------------------*/

#define portNVIC_SYSTICK_CTRL_REG				( * ( ( volatile uint32_t * ) 0xe000e010 ) )
#define portNVIC_SYSTICK_LOAD_REG				( * ( ( volatile uint32_t * ) 0xe000e014 ) )
#define portNVIC_SYSTICK_CURRENT_VALUE_REG		( * ( ( volatile uint32_t * ) 0xe000e018 ) )
#define portNVIC_SYSPRI2_REG					( * ( ( volatile uint32_t * ) 0xe000ed20 ) )
#define portNVIC_SYSTICK_INT_BIT				( 1UL << 1UL )
#define portNVIC_SYSTICK_ENABLE_BIT				( 1UL << 0UL )
#define portNVIC_SYSTICK_COUNT_FLAG_BIT			( 1UL << 16UL )
#define portNVIC_PENDSVCLEAR_BIT 				( 1UL << 27UL )
#define portNVIC_PEND_SYSTICK_CLEAR_BIT			( 1UL << 25UL )

#define portNVIC_PENDSV_PRI						( ( ( uint32_t ) configKERNEL_INTERRUPT_PRIORITY ) << 16UL )
#define portNVIC_SYSTICK_PRI					( ( ( uint32_t ) configKERNEL_INTERRUPT_PRIORITY ) << 24UL )

#ifdef __cplusplus
}
#endif

#endif /* PORTMACRO_H */
