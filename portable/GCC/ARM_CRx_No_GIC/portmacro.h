/*
 * FreeRTOS Kernel V10.5.1
 * Copyright (C) 2021 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * https://www.FreeRTOS.org
 * https://github.com/FreeRTOS
 *
 */

#ifndef PORTMACRO_H
    #define PORTMACRO_H

    #ifdef __cplusplus
        extern "C" {
    #endif

/*-----------------------------------------------------------
 * Port specific definitions.
 *
 * The settings in this file configure FreeRTOS correctly for the given hardware
 * and compiler.
 *
 * These settings should not be altered.
 *-----------------------------------------------------------
 */

/* Type definitions. */
    #define portCHAR          char
    #define portFLOAT         float
    #define portDOUBLE        double
    #define portLONG          long
    #define portSHORT         short
    #define portSTACK_TYPE    uint32_t
    #define portBASE_TYPE     long

    typedef portSTACK_TYPE   StackType_t;
    typedef long             BaseType_t;
    typedef unsigned long    UBaseType_t;

    typedef uint32_t         TickType_t;
    #define portMAX_DELAY              ( TickType_t ) 0xffffffffUL

/* 32-bit tick type on a 32-bit architecture, so reads of the tick count do
 * not need to be guarded with a critical section. */
    #define portTICK_TYPE_IS_ATOMIC    1

/*-----------------------------------------------------------*/

/* Hardware specifics. */
    #define portSTACK_GROWTH      ( -1 )
    #define portTICK_PERIOD_MS    ( ( TickType_t ) 1000 / configTICK_RATE_HZ )
    #define portBYTE_ALIGNMENT    8


/*-----------------------------------------------------------
* Critical section control
*----------------------------------------------------------*/
/* SVC numbers for various services. */
/* TODO FUTURE_SPLIT_MACRO: Actually use these when the portmacro.h split happens. */
    #define portSVC_YIELD               0
    #define portSVC_RAISE_PRIVILEGE     1
    #define portSVC_ENTER_CRITICAL      2
    #define portSVC_EXIT_CRITICAL       3
    #define portSVC_DISABLE_INTERRUPTS  4
    #define portSVC_ENABLE_INTERRUPTS   5
    #define portSVC_RESET_PRIVILEGE     6

    /* These are assembly functions */
    extern void vPortTaskUsesFPU( void );
    extern void vPortYield( void );
    extern void vPortRaisePrivilege( void );
    extern void vPortEnterCritical( void );
    extern void vPortExitCritical( void );
    extern void vPortEnableInterrupts( void );
    extern void vPortDisableInterrupts( void );
    extern void vPortResetPrivilege( void );
    extern BaseType_t xPortIsPrivileged( void );

/**
 * @brief Checks whether or not the processor is privileged.
 * The Privilege level is determined by checking if bits [4:0] of
 * the callers Current Program Status Register are in USER_MODE, 0x10
 * @return 1 if the processor is already privileged, 0 otherwise.
 */
    #define portIS_PRIVILEGED()     xPortIsPrivileged()

/**
 * @brief Raise an SVC request to raise privilege.
 * The Privilege level is raised by setting bits [4:0] of
 * the callers Current Program Status Register to SYS_MODE, 0x1F
 * @return 0 in R0 if the caller is in USER_MODE
 */
    #define portRAISE_PRIVILEGE()       vPortRaisePrivilege()

/**
 * @brief Raise an SVC request to lower privilege.
 * The Privilege level is lowered by setting bits [4:0] of
 * the callers Current Program Status Register to USER_MODE, 0x10
 */
    #define portRESET_PRIVILEGE()       vPortResetPrivilege();

/**
 * TODO POSSIBLE_IMPROVEMENTS: These functions are allowed to be called from user mode for the sake of demos.
 * These will need to be either removed from the port, or require a privileged task to call them
 */

    #define portENTER_CRITICAL()        vPortEnterCritical()
    #define portEXIT_CRITICAL()         vPortExitCritical()

    #define portDISABLE_INTERRUPTS()    vPortDisableInterrupts()
    #define portENABLE_INTERRUPTS()     vPortEnableInterrupts()

    /* Scheduler utilities. */
    #define portYIELD()                 vPortYield()
    #define portSYS_SSIR1_REG			( * ( ( volatile uint32_t * ) 0xFFFFFFB0 ) )
    #define portSYS_SSIR1_SSKEY			( 0x7500UL )

    #define portYIELD_WITHIN_API()		{ portSYS_SSIR1_REG = portSYS_SSIR1_SSKEY;  asm( " DSB " ); asm( " ISB " ); }
    #define portYIELD_FROM_ISR( x )		if( x != pdFALSE ){ portSYS_SSIR1_REG = portSYS_SSIR1_SSKEY;  ( void ) portSYS_SSIR1_REG; }

/*-----------------------------------------------------------*/

/* Task function macros as described on the FreeRTOS.org WEB site.  These are
 * not required for this port but included in case common demo code that uses these
 * macros is used. */
    #define portTASK_FUNCTION_PROTO( vFunction, pvParameters )    void vFunction( void * pvParameters )
    #define portTASK_FUNCTION( vFunction, pvParameters )          void vFunction( void * pvParameters )

/* Any task that uses the floating point unit MUST call vPortTaskUsesFPU()
 * before any floating point instructions are executed. */
    #define portTASK_USES_FLOATING_POINT()    vPortTaskUsesFPU()

/* Architecture specific optimisations. */
    #ifndef configUSE_PORT_OPTIMISED_TASK_SELECTION
        #define configUSE_PORT_OPTIMISED_TASK_SELECTION 1
    #endif

    #if configUSE_PORT_OPTIMISED_TASK_SELECTION == 1

	/* Generic helper function. */
	unsigned long ulPortCountLeadingZeros( unsigned long ulBitmap );

	/* Check the configuration. */
	#if( configMAX_PRIORITIES > 32 )
		#error configUSE_PORT_OPTIMISED_TASK_SELECTION can only be set to 1 when configMAX_PRIORITIES is less than or equal to 32.  It is very rare that a system requires more than 10 to 15 difference priorities as tasks that share a priority will time slice.
	#endif

/* Store/clear the ready priorities in a bit map. */
	#define portRECORD_READY_PRIORITY( uxPriority, uxReadyPriorities ) ( uxReadyPriorities ) |= ( 1UL << ( uxPriority ) )
	#define portRESET_READY_PRIORITY( uxPriority, uxReadyPriorities ) ( uxReadyPriorities ) &= ~( 1UL << ( uxPriority ) )

/*-----------------------------------------------------------*/

	#define portGET_HIGHEST_PRIORITY( uxTopPriority, uxReadyPriorities ) uxTopPriority = ( 31UL - ulPortCountLeadingZeros( ( uxReadyPriorities ) ) )

    #endif /* configUSE_PORT_OPTIMISED_TASK_SELECTION */

/* Prototype of the FreeRTOS tick handler.  This must be installed as the
 * handler for whichever peripheral is used to generate the RTOS tick. */
    void FreeRTOS_Tick_Handler( void );

    #define portLOWEST_INTERRUPT_PRIORITY           ( ( ( uint32_t ) configUNIQUE_INTERRUPT_PRIORITIES ) - 1UL )
    #define portLOWEST_USABLE_INTERRUPT_PRIORITY    ( portLOWEST_INTERRUPT_PRIORITY - 1UL )



/* Tickless idle/low power functionality. */
    #ifndef portSUPPRESS_TICKS_AND_SLEEP
        extern void vPortSuppressTicksAndSleep( TickType_t xExpectedIdleTime );
        #define portSUPPRESS_TICKS_AND_SLEEP( xExpectedIdleTime )    vPortSuppressTicksAndSleep( xExpectedIdleTime )
    #endif

    /* MPU specific constants.  */
    #define portUSING_MPU_WRAPPERS		            1
    #define portPRIVILEGE_BIT                       ( 0x80000000UL )

    #define portMPU_REGION_READ_WRITE				( 0x03UL << 8UL )
    #define portMPU_REGION_PRIVILEGED_READ_ONLY		( 0x05UL << 8UL )
    #define portMPU_REGION_READ_ONLY				( 0x06UL << 8UL )
    #define portMPU_REGION_PRIVILEGED_READ_WRITE	( 0x01UL << 8UL )

    #define portMPU_REGION_STRONGLY_ORDERED         ( 0x00UL )
    #define portMPU_REGION_DEVICE                   ( 0x01UL )
    #define portMPU_REGION_CACHEABLE_BUFFERABLE		( 0x03UL )
    #define portMPU_REGION_EXECUTE_NEVER			( 0x01UL << 12UL )

    #define portMPU_STRONGLYORDERED_SHAREABLE  ( 0x0000UL ) /**< Memory type strongly ordered and sharable */
    #define portMPU_DEVICE_SHAREABLE           ( 0x0001UL ) /**< Memory type device and sharable */
    #define portMPU_NORMAL_OIWTNOWA_NONSHARED  ( 0x0002UL ) /**< Memory type normal outer and inner write-through, no write allocate and non shared */
    #define portMPU_NORMAL_OIWBNOWA_NONSHARED  ( 0x0003UL ) /**< Memory type normal outer and inner write-back, no write allocate and non shared */
    #define portMPU_NORMAL_OIWTNOWA_SHARED     ( 0x0006UL ) /**< Memory type normal outer and inner write-through, no write allocate and shared */
    #define portMPU_NORMAL_OIWBNOWA_SHARED     ( 0x0007UL ) /**< Memory type normal outer and inner write-back, no write allocate and shared */
    #define portMPU_NORMAL_OINC_NONSHARED      ( 0x0008UL ) /**< Memory type normal outer and inner non-cachable and non shared */
    #define portMPU_NORMAL_OIWBWA_NONSHARED    ( 0x000BUL ) /**< Memory type normal outer and inner write-back, write allocate and non shared */
    #define portMPU_NORMAL_OINC_SHARED         ( 0x000CUL ) /**< Memory type normal outer and inner non-cachable and shared */
    #define portMPU_NORMAL_OIWBWA_SHARED       ( 0x000FUL ) /**< Memory type normal outer and inner write-back, write allocate and shared */
    #define portMPU_DEVICE_NONSHAREABLE        ( 0x0010UL ) /**< Memory type device and non sharable */

    #define portMPU_PRIV_NA_USER_NA_EXEC    ( 0x0000UL ) /**< Alias no access in privileged mode, no access in user mode and execute */
    #define portMPU_PRIV_RW_USER_NA_EXEC    ( 0x0100UL ) /**< Alias no read/write in privileged mode, no access in user mode and execute */
    #define portMPU_PRIV_RW_USER_RO_EXEC    ( 0x0200UL ) /**< Alias no read/write in privileged mode, read only in user mode and execute */
    #define portMPU_PRIV_RW_USER_RW_EXEC    ( 0x0300UL ) /**< Alias no read/write in privileged mode, read/write in user mode and execute */
    #define portMPU_PRIV_RO_USER_NA_EXEC    ( 0x0500UL ) /**< Alias no read only in privileged mode, no access in user mode and execute */
    #define portMPU_PRIV_RO_USER_RO_EXEC    ( 0x0600UL ) /**< Alias no read only in privileged mode, read only in user mode and execute */
    #define portMPU_PRIV_NA_USER_NA_NOEXEC  ( 0x1000UL ) /**< Alias no access in privileged mode, no access in user mode and no execution */
    #define portMPU_PRIV_RW_USER_NA_NOEXEC  ( 0x1100UL ) /**< Alias no read/write in privileged mode, no access in user mode and no execution */
    #define portMPU_PRIV_RW_USER_RO_NOEXEC  ( 0x1200UL ) /**< Alias no read/write in privileged mode, read only in user mode and no execution */
    #define portMPU_PRIV_RW_USER_RW_NOEXEC  ( 0x1300UL ) /**< Alias no read/write in privileged mode, read/write in user mode and no execution */
    #define portMPU_PRIV_RO_USER_NA_NOEXEC  ( 0x1500UL ) /**< Alias no read only in privileged mode, no access in user mode and no execution */
    #define portMPU_PRIV_RO_USER_RO_NOEXEC  ( 0x1600UL ) /**< Alias no read only in privileged mode, read only in user mode and no execution */

    #define portMPU_REGION_ENABLE					( 0x01UL )

    #define portMPU_TOTAL_REGIONS				    ( 12UL )

    /* MPU Sub Region region */
    #define portMPU_SUBREGION_0_DISABLE			    ( 0x1UL << 8UL )
    #define portMPU_SUBREGION_1_DISABLE			    ( 0x1UL << 9UL )
    #define portMPU_SUBREGION_2_DISABLE			    ( 0x1UL << 10UL )
    #define portMPU_SUBREGION_3_DISABLE			    ( 0x1UL << 11UL )
    #define portMPU_SUBREGION_4_DISABLE			    ( 0x1UL << 12UL )
    #define portMPU_SUBREGION_5_DISABLE			    ( 0x1UL << 13UL )
    #define portMPU_SUBREGION_6_DISABLE			    ( 0x1UL << 14UL )
    #define portMPU_SUBREGION_7_DISABLE			    ( 0x1UL << 15UL )

    /* MPU region sizes */
    #define portMPU_SIZE_32B						( 0x04UL << 1UL )
    #define portMPU_SIZE_64B						( 0x05UL << 1UL )
    #define portMPU_SIZE_128B						( 0x06UL << 1UL )
    #define portMPU_SIZE_256B						( 0x07UL << 1UL )
    #define portMPU_SIZE_512B						( 0x08UL << 1UL )
    #define portMPU_SIZE_1KB						( 0x09UL << 1UL )
    #define portMPU_SIZE_2KB						( 0x0AUL << 1UL )
    #define portMPU_SIZE_4KB						( 0x0BUL << 1UL )
    #define portMPU_SIZE_8KB						( 0x0CUL << 1UL )
    #define portMPU_SIZE_16KB                       ( 0x0DUL << 1UL )
    #define portMPU_SIZE_32KB                       ( 0x0EUL << 1UL )
    #define portMPU_SIZE_64KB                       ( 0x0FUL << 1UL )
    #define portMPU_SIZE_128KB                      ( 0x10UL << 1UL )
    #define portMPU_SIZE_256KB                      ( 0x11UL << 1UL )
    #define portMPU_SIZE_512KB                      ( 0x12UL << 1UL )
    #define portMPU_SIZE_1MB					    ( 0x13UL << 1UL )
    #define portMPU_SIZE_2MB					    ( 0x14UL << 1UL )
    #define portMPU_SIZE_4MB					    ( 0x15UL << 1UL )
    #define portMPU_SIZE_8MB					    ( 0x16UL << 1UL )
    #define portMPU_SIZE_16MB						( 0x17UL << 1UL )
    #define portMPU_SIZE_32MB						( 0x18UL << 1UL )
    #define portMPU_SIZE_64MB					    ( 0x19UL << 1UL )
    #define portMPU_SIZE_128MB					    ( 0x1AUL << 1UL )
    #define portMPU_SIZE_256MB					    ( 0x1BUL << 1UL )
    #define portMPU_SIZE_512MB					    ( 0x1CUL << 1UL )
    #define portMPU_SIZE_1GB					    ( 0x1DUL << 1UL )
    #define portMPU_SIZE_2GB					    ( 0x1EUL << 1UL )
    #define portMPU_SIZE_4GB					    ( 0x1FUL << 1UL )

    /* Default MPU regions */
    #define portUNPRIVILEGED_FLASH_REGION		    ( 0UL )
    #define portPRIVILEGED_FLASH_REGION			    ( 1UL )
    #define portPRIVILEGED_RAM_REGION			    ( 2UL )
    #define portGENERAL_PERIPHERALS_REGION		    ( 3UL )
    #define portSTACK_REGION					    ( 4UL )
    #define portFIRST_CONFIGURABLE_REGION	        ( 5UL )
    #define portLAST_CONFIGURABLE_REGION		    ( portMPU_TOTAL_REGIONS - 2UL )
    #define portPRIVILEGED_SYSTEM_REGION            ( portMPU_TOTAL_REGIONS - 1UL )
    #define portNUM_CONFIGURABLE_REGIONS		    ( ( portLAST_CONFIGURABLE_REGION - portFIRST_CONFIGURABLE_REGION ) + 1UL )
    #define portTOTAL_NUM_REGIONS				    ( portNUM_CONFIGURABLE_REGIONS + 1UL ) /* Plus one to make space for the stack region. */

    typedef struct MPU_REGION_REGISTERS
    {
        unsigned ulRegionBaseAddress;
        unsigned ulRegionSize;
        unsigned ulRegionAttribute;
    } xMPU_REGION_REGISTERS;

    /* Plus 1 to create space for the stack region. */
    typedef struct MPU_SETTINGS
    {
        xMPU_REGION_REGISTERS xRegion[ portTOTAL_NUM_REGIONS ];
    } xMPU_SETTINGS;

    #define portNOP()               __asm volatile ( "NOP" )
    #define portINLINE    __inline
    #define portMEMORY_BARRIER()    __asm volatile ( "" ::: "memory" )

    #ifdef __cplusplus
}     /* extern C */
    #endif


#endif /* PORTMACRO_H */
