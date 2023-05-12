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

/* Standard includes. */
#include <stdlib.h>

#define MPU_WRAPPERS_INCLUDED_FROM_API_FILE
/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#undef MPU_WRAPPERS_INCLUDED_FROM_API_FILE

#if configUSE_PORT_OPTIMISED_TASK_SELECTION == 1
    /* Check the configuration. */
    #if ( configMAX_PRIORITIES > 32 )
        #error configUSE_PORT_OPTIMISED_TASK_SELECTION can only be set to 1 when configMAX_PRIORITIES is less than or equal to 32.  It is very rare that a system requires more than 10 to 15 difference priorities as tasks that share a priority will time slice.
    #endif
#endif /* configUSE_PORT_OPTIMISED_TASK_SELECTION */

#ifndef configSETUP_TICK_INTERRUPT
    #error configSETUP_TICK_INTERRUPT() must be defined in FreeRTOSConfig.h to call the function that sets up the tick interrupt.
#endif

#ifndef configCLEAR_TICK_INTERRUPT
    #error configCLEAR_TICK_INTERRUPT must be defined in FreeRTOSConfig.h to clear which ever interrupt was used to generate the tick interrupt.
#endif

/* A critical section is exited when the critical section nesting count reaches
 * this value. */
#define portNO_CRITICAL_NESTING          ( ( uint32_t ) 0 )

/* Tasks are not created with a floating point context, but can be given a
 * floating point context after they have been created.  A variable is stored as
 * part of the tasks context that holds portNO_FLOATING_POINT_CONTEXT if the task
 * does not have an FPU context, or any other value if the task does have an FPU
 * context. */
#define portNO_FLOATING_POINT_CONTEXT    ( ( StackType_t ) 0 )

/**
 * TODO FUTURE_SPLIT_MACRO/POSSIBLE_IMPROVEMENTS: This isn't really setting an intial SPSR, it's setting a CPSR.
 * These values are actually just setting the mode bits to either be in system or privileged mode
 * when the task starts. When the portmacro.h split happens these should be either removed and
 * replaced with the mode name, or at least renamed to be accurate. Something like portINITIAL_TASK_MODE.
 * Lastly, when the RM46 starts up it enables a bit in the CSPR to hold on an ASYNC interrupt, which isn't done here.
 * It's rare to hit that interrupt, per ARMs documentation, but is this something that should be added?
*/
/* Constants required to setup the initial task context. */
#define portINITIAL_SPSR_IF_PRIVILEGED   ( ( StackType_t ) 0x1F )   /* Set task to System Mode by setting Bits [4:0] of the CPSR to System Mode */
#define portINITIAL_SPSR_IF_UNPRIVILEGED ( ( StackType_t ) 0x10 )   /* Set task to User Mode by setting Bits [4:0] of the CPSR to User Mode */
#define portTHUMB_MODE_BIT               ( ( StackType_t ) 0x20 )
#define portTHUMB_MODE_ADDRESS           ( 0x01UL )

/* Masks all bits in the APSR other than the mode bits. */
#define portAPSR_MODE_BITS_MASK          ( 0x1F )

/* The value of the mode bits in the APSR when the CPU is executing in user
 * mode. */
#define portAPSR_USER_MODE               ( 0x10 )

/* Let the user override the pre-loading of the initial LR with the address of
 * prvTaskExitError() in case it messes up unwinding of the stack in the
 * debugger. */
#ifdef configTASK_RETURN_ADDRESS
    #define portTASK_RETURN_ADDRESS    configTASK_RETURN_ADDRESS
#else
    #define portTASK_RETURN_ADDRESS    prvTaskExitError
#endif

/*-----------------------------------------------------------*/

/*
 * Starts the first task executing.  This function is necessarily written in
 * assembly code so is implemented in portASM.s.
 */
extern void vPortRestoreTaskContext( void );

/*
 * Used to catch tasks that attempt to return from their implementing function.
 */
static void prvTaskExitError( void );

/* MPU access routines defined in portASM.asm */
extern void prvMpuEnable( void );
extern void prvMpuDisable( void );
extern void prvMpuSetRegion( unsigned region, unsigned base, unsigned size, unsigned access );

/*-----------------------------------------------------------*/
/**
 * TODO GLOBAL_VARIABLE_RISKS: There are a few global variables that get their values
 * saved in their task on a context swap. However, if a user modified these variables
 * in another task's stack they could cause unknown side effects. Additionally, it is possible
 * to change these variables while running in user mode. A decision should be made regarding
 * how to properly protect these values.
 */
/* A variable is used to keep track of the critical section nesting.  This
 * variable has to be stored as part of the task context and must be initialised to
 * a non zero value to ensure interrupts don't inadvertently become unmasked before
 * the scheduler starts.  As it is stored as part of the task context it will
 * automatically be set to 0 when the first task is started. */
volatile uint32_t ulCriticalNesting = 9999UL;

/* Saved as part of the task context.  If ulPortTaskHasFPUContext is non-zero then
 * a floating point context must be saved and restored for the task. */
volatile uint32_t ulPortTaskHasFPUContext = pdFALSE;

/* Set to 1 to pend a context switch from an ISR. */
volatile uint32_t ulPortYieldRequired = pdFALSE;

/* Counts the interrupt nesting depth.  A context switch is only performed if
 * if the nesting depth is 0. */
volatile uint32_t ulPortInterruptNesting = 0UL;

/* TODO FUTURE_SPLIT_MACRO: Split the header file into different ones, one of which can be included in
 * the assembly file so these can just be defined once, instead of needing to load variables.
 * Need to loop over the number of MPU regions in the portRESTORE_CONTEXT macro
 * Defining it as a variable here to link against in the asm file.
 */
uint32_t ulTotalMPURegions              = portMPU_TOTAL_REGIONS;
uint32_t ulStackMPURegion               = portSTACK_REGION;
uint32_t ulLastConfigurableMPURegion    = portLAST_CONFIGURABLE_REGION;

/* Used in the asm file to clear an interrupt. */
__attribute__( ( used ) ) const uint32_t ulICCEOIR = configEOI_ADDRESS;

/*-----------------------------------------------------------*/

/*
 * See header file for description.
 */
#if ( portUSING_MPU_WRAPPERS == 1 )
StackType_t *pxPortInitialiseStack( StackType_t *pxTopOfStack,
                                    TaskFunction_t pxCode, 
                                    void *pvParameters, 
                                    BaseType_t xRunPrivileged )
#else
StackType_t *pxPortInitialiseStack( StackType_t *pxTopOfStack,
                                    TaskFunction_t pxCode,
                                     void *pvParameters )
#endif
{
    /* Setup the initial stack of the task.  The stack is set exactly as
     * expected by the portRESTORE_CONTEXT() macro.
     *
     * The fist real value on the stack is the status register, which is set for
     * system mode, with interrupts enabled.  A few NULLs are added first to ensure
     * GDB does not try decoding a non-existent return address. */
    *pxTopOfStack = ( StackType_t ) NULL;
    pxTopOfStack--;
    *pxTopOfStack = ( StackType_t ) NULL;
    pxTopOfStack--;
    *pxTopOfStack = ( StackType_t ) NULL;
    pxTopOfStack--;
    
	/* Set the status register for system or user mode, with interrupts enabled. */
#if ( portUSING_MPU_WRAPPERS == 1 )
	if( xRunPrivileged == pdTRUE )
	{
	    *pxTopOfStack = (portSTACK_TYPE) ( portINITIAL_SPSR_IF_PRIVILEGED );
	}
	else
#endif
	{
	    *pxTopOfStack = (portSTACK_TYPE) ( portINITIAL_SPSR_IF_UNPRIVILEGED );
	}


    if( ( ( uint32_t ) pxCode & portTHUMB_MODE_ADDRESS ) != 0x00UL )
    {
        /* The task will start in THUMB mode. */
        *pxTopOfStack |= portTHUMB_MODE_BIT;
    }

    pxTopOfStack--;

    /* Next the return address, which in this case is the start of the task. */
    *pxTopOfStack = ( StackType_t ) pxCode;
    pxTopOfStack--;

    /* Next all the registers other than the stack pointer. */
    *pxTopOfStack = ( StackType_t ) portTASK_RETURN_ADDRESS; /* R14 */
    pxTopOfStack--;
    *pxTopOfStack = ( StackType_t ) 0x12121212;              /* R12 */
    pxTopOfStack--;
    *pxTopOfStack = ( StackType_t ) 0x11111111;              /* R11 */
    pxTopOfStack--;
    *pxTopOfStack = ( StackType_t ) 0x10101010;              /* R10 */
    pxTopOfStack--;
    *pxTopOfStack = ( StackType_t ) 0x09090909;              /* R9 */
    pxTopOfStack--;
    *pxTopOfStack = ( StackType_t ) 0x08080808;              /* R8 */
    pxTopOfStack--;
    *pxTopOfStack = ( StackType_t ) 0x07070707;              /* R7 */
    pxTopOfStack--;
    *pxTopOfStack = ( StackType_t ) 0x06060606;              /* R6 */
    pxTopOfStack--;
    *pxTopOfStack = ( StackType_t ) 0x05050505;              /* R5 */
    pxTopOfStack--;
    *pxTopOfStack = ( StackType_t ) 0x04040404;              /* R4 */
    pxTopOfStack--;
    *pxTopOfStack = ( StackType_t ) 0x03030303;              /* R3 */
    pxTopOfStack--;
    *pxTopOfStack = ( StackType_t ) 0x02020202;              /* R2 */
    pxTopOfStack--;
    *pxTopOfStack = ( StackType_t ) 0x01010101;              /* R1 */
    pxTopOfStack--;
    *pxTopOfStack = ( StackType_t ) pvParameters;            /* R0 */
    pxTopOfStack--;

    /* The task will start with a critical nesting count of 0 as interrupts are
     * enabled. */
    *pxTopOfStack = portNO_CRITICAL_NESTING;
    pxTopOfStack--;

    /* The task will start without a floating point context.  A task that uses
     * the floating point hardware must call vPortTaskUsesFPU() before executing
     * any floating point instructions. */
    *pxTopOfStack = portNO_FLOATING_POINT_CONTEXT;

    return pxTopOfStack;
}

/*----------------------------------------------------------------------------*/

static unsigned long prvGetMPURegionSizeSetting( unsigned long ulActualSizeInBytes )
{
	unsigned long ulRegionSize, ulReturnValue = 4;

	/* 32 is the smallest region size, 31 is the largest valid value for
	ulReturnValue. */
	for( ulRegionSize = 32UL; ulReturnValue < 31UL; ( ulRegionSize <<= 1UL ) )
	{
		if( ulActualSizeInBytes <= ulRegionSize )
		{
			break;
		}
		else
		{
			ulReturnValue++;
		}
	}

	/* Shift the code by one before returning so it can be written directly
	into the the correct bit position of the attribute register. */
	return ulReturnValue << 1UL;
}

/*----------------------------------------------------------------------------*/

void vPortStoreTaskMPUSettings( xMPU_SETTINGS *xMPUSettings, const struct xMEMORY_REGION * const xRegions, StackType_t *pxBottomOfStack, uint32_t usStackDepth )
{
	long lIndex;
	unsigned long ul;
	if( xRegions == NULL )
	{
		/* No MPU regions are specified so allow access to all of the RAM. */
		xMPUSettings->xRegion[0].ulRegionBaseAddress = 0x08000000;
		xMPUSettings->xRegion[0].ulRegionSize        = portMPU_SIZE_512KB | portMPU_REGION_ENABLE;
		xMPUSettings->xRegion[0].ulRegionAttribute   = portMPU_PRIV_RW_USER_RW_EXEC | portMPU_NORMAL_OIWTNOWA_SHARED;

		/* Re-instate the privileged only RAM region as xRegion[ 0 ] will have
		just removed the privileged only parameters. */
		xMPUSettings->xRegion[1].ulRegionBaseAddress = 0x08000000;
		xMPUSettings->xRegion[1].ulRegionSize        = portMPU_SIZE_4KB | portMPU_REGION_ENABLE;
		xMPUSettings->xRegion[1].ulRegionAttribute   = portMPU_PRIV_RW_USER_NA_NOEXEC | portMPU_NORMAL_OIWTNOWA_SHARED;

		/* Invalidate all other regions. */
		for( ul = 2; ul <= portNUM_CONFIGURABLE_REGIONS; ul++ )
		{
			xMPUSettings->xRegion[ ul ].ulRegionBaseAddress = 0x00000000UL;
			xMPUSettings->xRegion[ ul ].ulRegionSize        = 0UL;
			xMPUSettings->xRegion[ ul ].ulRegionAttribute   = 0UL;
		}

	}
	else
	{
		/* This function is called automatically when the task is created - in
		which case the stack region parameters will be valid.  At all other
		times the stack parameters will not be valid and it is assumed that the
		stack region has already been configured. */

		if( usStackDepth > 0 )
		{
		    /* Define the region that allows access to the stack. */
			xMPUSettings->xRegion[0].ulRegionBaseAddress = (unsigned)pxBottomOfStack;
			xMPUSettings->xRegion[0].ulRegionSize        = prvGetMPURegionSizeSetting( (unsigned long)usStackDepth * (unsigned long) sizeof(portSTACK_TYPE) ) | portMPU_REGION_ENABLE;
			xMPUSettings->xRegion[0].ulRegionAttribute   = portMPU_REGION_EXECUTE_NEVER | portMPU_REGION_READ_WRITE | portMPU_NORMAL_OIWTNOWA_SHARED ;

		}
		lIndex = 0;

		for( ul = 1; ul <= portNUM_CONFIGURABLE_REGIONS; ul++ )
		{
			if( ( xRegions[ lIndex ] ).ulLengthInBytes > 0UL )
			{
				/* Translate the generic region definition contained in
				xRegions into the R4 specific MPU settings that are then
				stored in xMPUSettings. */
				xMPUSettings->xRegion[ul].ulRegionBaseAddress = (unsigned long) xRegions[lIndex].pvBaseAddress;
				xMPUSettings->xRegion[ul].ulRegionSize        = prvGetMPURegionSizeSetting( xRegions[ lIndex ].ulLengthInBytes ) | portMPU_REGION_ENABLE;;
				xMPUSettings->xRegion[ul].ulRegionAttribute   = xRegions[ lIndex ].ulParameters;
			}
			else
			{
				/* Invalidate the region. */
				xMPUSettings->xRegion[ ul ].ulRegionBaseAddress = 0x00000000UL;
				xMPUSettings->xRegion[ ul ].ulRegionSize        = 0UL;
				xMPUSettings->xRegion[ ul ].ulRegionAttribute   = 0UL;
			}
			lIndex++;
		}
	}
}

/*----------------------------------------------------------------------------*/

static void prvSetupDefaultMPU( void )
{
	/* Make sure MPU is disabled */
	prvMpuDisable();

	/* First setup the entire flash for unprivileged read only access. */
	prvMpuSetRegion(portUNPRIVILEGED_FLASH_REGION,  0x00000000, portMPU_SIZE_4MB | portMPU_REGION_ENABLE, portMPU_PRIV_RO_USER_RO_EXEC | portMPU_NORMAL_OIWTNOWA_SHARED);

	/* Setup the first 32K for privileged only access.  This is where the kernel code is placed. */
	prvMpuSetRegion(portPRIVILEGED_FLASH_REGION,  0x00000000, portMPU_SIZE_32KB | portMPU_REGION_ENABLE, portMPU_PRIV_RO_USER_NA_EXEC | portMPU_NORMAL_OIWTNOWA_SHARED);

	/* Setup the the entire RAM region for privileged read-write and unprivileged read only access */
	prvMpuSetRegion(portPRIVILEGED_RAM_REGION,  0x08000000, portMPU_SIZE_512KB | portMPU_REGION_ENABLE, portMPU_PRIV_RW_USER_RO_EXEC | portMPU_NORMAL_OIWTNOWA_SHARED);

	/* Default peripherals setup */
	prvMpuSetRegion(portGENERAL_PERIPHERALS_REGION,  0xF0000000,
					portMPU_SIZE_256MB | portMPU_REGION_ENABLE | portMPU_SUBREGION_1_DISABLE | portMPU_SUBREGION_2_DISABLE | portMPU_SUBREGION_3_DISABLE | portMPU_SUBREGION_4_DISABLE,
					portMPU_PRIV_RW_USER_RW_NOEXEC | portMPU_DEVICE_NONSHAREABLE);

	/* Privilege System Region setup */
	prvMpuSetRegion(portPRIVILEGED_SYSTEM_REGION,  0xFFF80000, portMPU_SIZE_512KB | portMPU_REGION_ENABLE, portMPU_PRIV_RW_USER_RO_NOEXEC | portMPU_DEVICE_NONSHAREABLE);
	
	/* Enable MPU */
	prvMpuEnable();
}

/*-----------------------------------------------------------*/

static void prvTaskExitError( void )
{
    /* A function that implements a task must not exit or attempt to return to
     * its caller as there is nothing to return to.  If a task wants to exit it
     * should instead call vTaskDelete( NULL ).
     *
     * Artificially force an assert() to be triggered if configASSERT() is
     * defined, then stop here so application writers can catch the error. */
    configASSERT( ulPortInterruptNesting == ~0UL );
    portDISABLE_INTERRUPTS();

    for( ; ; )
    {
    }
}

/*-----------------------------------------------------------*/

BaseType_t xPortStartScheduler( void )
{
    /* Configure the regions in the MPU that are common to all tasks. */
    prvSetupDefaultMPU();
    
    /* Start the timer that generates the tick ISR. */
    configSETUP_TICK_INTERRUPT();

    /* Reset the critical section nesting count read to execute the first task. */
    ulCriticalNesting = 0;
    
    /* Start the first task executing. */
    vPortRestoreTaskContext();

    /* Will only get here if vTaskStartScheduler() was called with the CPU in
     * a non-privileged mode or the binary point register was not set to its lowest
     * possible value.  prvTaskExitError() is referenced to prevent a compiler
     * warning about it being defined but not referenced in the case that the user
     * defines their own exit address. */
    ( void ) prvTaskExitError;
    return 0;
}
/*-----------------------------------------------------------*/

void vPortEndScheduler( void )
{
    /* Not implemented in ports where there is nothing to return to.
     * Artificially force an assert. */
    configASSERT( ulCriticalNesting == 1000UL );
}

/*-----------------------------------------------------------*/
void FreeRTOS_Tick_Handler( void )
{
    uint32_t ulInterruptStatus;

    ulInterruptStatus = portSET_INTERRUPT_MASK_FROM_ISR();

    /* Increment the RTOS tick. */
    if( xTaskIncrementTick() != pdFALSE )
    {
        ulPortYieldRequired = pdTRUE;
    }

    portCLEAR_INTERRUPT_MASK_FROM_ISR( ulInterruptStatus );

    configCLEAR_TICK_INTERRUPT();
}
/*-----------------------------------------------------------*/
