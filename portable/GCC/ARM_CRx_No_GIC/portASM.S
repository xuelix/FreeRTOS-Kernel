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
    .cpu cortex-r4
    .text
    .arm
    .syntax unified

    /* TODO POSSIBLE_IMPROVEMENTS: There are some different formatting styles in this file.
     * A style guide should be developed and made for FreeRTOS ports that states exactly
     * what a port's assembly file should look like. An example being how an assembly function's
     * comment block above the function should look, and how the end of the function should be marked.
     * Another important style decision will be max line length. As of now there is no set value for this.
    */

    /* TODO FUTURE_SPLIT_MACRO: When this split happens these should be defined in the
     * Split header function. There are places that these get used in the port.c file
    */
    /* These modes correspond to the values in the M field, bits [4:0] of the CPSR */
    /* Of note, the only mode not "Privileged" is User Mode */
    .set USER_MODE, 0x10    /* CPSR M field value for User Mode */
    .set FIQ_MODE,  0x11    /* CPSR M field value for Fast Interrupt Handler Mode */
    .set IRQ_MODE,  0x12    /* CPSR M field value for Interrupt Handler Mode */
    .set SVC_MODE,  0x13    /* CPSR M field value for Supervisor Mode */
    .set MON_MODE,  0x16    /* CPSR M field value for Monitor Mode */
    .set ABT_MODE,  0x17    /* CPSR M field value for Abort Mode */
    .set HYP_MODE,  0x1A    /* CPSR M field value for Hypervisor Mode */
    .set UND_MODE,  0x1B    /* CPSR M field value for Undefined Mode */
    .set SYS_MODE,  0x1f    /* CPSR M field value for System Mode */

    /* Variables and functions. */
    .extern pxCurrentTCB
    .extern vTaskSwitchContext
    .extern vApplicationIRQHandler
    .extern ulPortInterruptNesting
    .extern ulPortTaskHasFPUContext
    .extern ulICCEOIR
    .extern ulPortYieldRequired

    .global FreeRTOS_IRQ_Handler
    .global FreeRTOS_SVC_Handler
    .global vPortRestoreTaskContext

    .global prvMpuDisable
    .global prvMpuEnable
    .global prvMpuSetRegion

.macro portSAVE_CONTEXT

    DSB
    ISB

    /* The System Mode stack pointer will be the stack of the task being swapped out
     * Save the LR and SPSR onto the system mode stack before switching to
     * system mode to save the remaining system mode registers. */
    /* TODO TCB_CONTEXT_SAVING: When we swap to storing context in the TCB we will need to
     * get the context saving region from the TCB, move it to the SP, and then do this */
    SRSDB   SP!, #SYS_MODE                      /* Save the return state of the current caller onto the SYS_MODE stack */
    CPS     #SYS_MODE                           /* Change to system mode to access the registers of the task being swapped out */
    PUSH    {R0-R12, R14}                       /* Push User registers R0-R12, and the previous Link Register */

    /* Push the critical nesting count. */
    LDR     R2, ulCriticalNestingConst          /* Load the address of ulCriticalNesting */
    LDR     R1, [R2]                            /* Load the value of ulCriticalNestingConst into R1 */
    PUSH    {R1}                                /* Push the value of ulCriticalNestingConst onto the stack */

    /* TODO TCB_CONTEXT_SAVING: When we swap to storing context in the TCB we will always save the FPRs */
    /* If ulPortTaskHasFPUContext is not 0 then push the Floating Point Registers, 
     * Floating Point Status and Control Register, and the value of ulPortTaskHasFPUContext. */
    LDR     R2, ulPortTaskHasFPUContextConst    /* Load the address of ulPortTaskHasFPUContext */
    LDR     R3, [R2]                            /* Load the value of ulPortTaskHasFPUContextConst into R3 */
    CMP     R3, #0                              /* Check if ulPortTaskHasFPUContext is 0 */

    /* Save the floating point context, if any. */
    FMRXNE  R1, FPSCR                           /* If ulPortTaskHasFPUContext is not 0 load the FPSCR into R1*/
    VPUSHNE {D0-D15}                            /* if ulPortTaskHasFPUContext is not 0 push the 16 floating point registers onto the stack */
    PUSHNE  {R1}                                /* Push the value of FPSCR onto the stack */

    /* Save ulPortTaskHasFPUContext itself. */
    PUSH    {R3}                                /* Push the value of ulPortTaskHasFPUContext onto the stack */

    /* Save the stack pointer in the TCB. */
    LDR     R0, pxCurrentTCBConst               /* Load the address of pxCurrentTCB into R0 */
    LDR     R1, [R0]                            /* Load pxCurrentTCB into R1 */
    STR     SP, [R1]                            /* The first value of pxCurrentTCB is the stack pointer */

    .endm

/**********************************************************************/
.macro portRESTORE_CONTEXT

    /* TODO TCB_CONTEXT_SAVING: When the swap to storing context in the TCB happens this will need to
     * get the context saving region from the TCB, move it to the SP, and then restore from there */
    /* Set the SP to point to the stack of the task being restored. */
    LDR     R0, pxCurrentTCBConst                   /* Load the address of the TCB of the task to be restored */
    LDR     R1, [R0]                                /* Load the TCB into R1 */
    LDR     SP, [R1]                                /* First value in the TCB is the pointer to the task's stack */

    ADD     R12, R1, #4                             /* Set R12 to the second member of the TCB struct, xMPUSettings */
/* TODO FUTURE_SPLIT_MACRO: Replace with defined value in the split portmacro file */
    LDR     R5, ulLastConfigurableMPURegionConst    /* Upper bound for loop is portLAST_CONFIGURABLE_REGION */
    LDR     R5, [R5]                                /* Load the last configurable MPU region number into R5  */
    LDR     R4, ulStackMPURegionConst               /* Every task has its stack region, and then configurable regions after */
    LDR     R4, [R4]                                /* Load the first per-task MPU region number into R4 */
/* TODO POSSIBLE_IMPROVEMENTS: When creating a loop label in a macro it has to be a numeric label.
 * Maybe the entire "Load MPU values from the TCB and set them" should be its own function that gets called 
 */
123:    /* For (R4 = portSTACK_REGION ; R4 <=  portLAST_CONFIGURABLE_REGION; R4++ ) */
    LDMIA   R12!, {R1-R3}                           /* Load values of struct MPU_REGION_REGISTERS into R1-R3 */
    /* R1 Will hold ulRegionBaseAddress, R2 will hold ulRegionSize, R3 will hold ulRegionAttribute */
    MCR     p15, #0, R4, c6, c2, #0                 /* Use the value in R4 to Select the MPU region  */
    MCR     p15, #0, R1, c6, c1, #0                 /* Set the MPU region Base Address using ulRegionBaseAddress */
    MCR     p15, #0, R3, c6, c1, #4                 /* Set the MPU region Access Attributes using ulRegionAttribute */
    MCR     p15, #0, R2, c6, c1, #2                 /* Set the MPU region Size, and if the region is enabled using ulRegionSize */
    ADD     R4, R4, #1                              /* R4++ */
    CMP     R4, R5                                  /* R4 <= R5 */

    BLE     123b                                    /* R4 <= R5, loop again */
    /* R4 > portLAST_CONFIGURABLE_REGION, all regions have been restored */

    /* TODO TCB_CONTEXT_SAVING: When the swap to storing context in the TCB happens these will always be saved */
    /* Is there a floating point context to restore?  If the restored ulPortTaskHasFPUContext is zero then no. */
    LDR     R0, ulPortTaskHasFPUContextConst        /* Load address of ulPortTaskHasFPUContextConst into R0 */
    POP     {R1}                                    /* Pop the stored value of ulPortTaskHasFPUContext R1 */
    STR     R1, [R0]                                /* Load value of ulPortTaskHasFPUContext into its memory location */
    CMP     R1, #0                                  /* Determine if ulPortTaskHasFPUContextConst is 0 */

    /* Restore the floating point context, if any. */
    POPNE   {R0}                                    /* If ulPortTaskHasFPUContextConst is not 0, pop the FP Status and Control Register */
    VPOPNE  {D0-D15}                                /* Restore the Floating Point Registers */
    VMSRNE  FPSCR, R0                               /* Move the saved FPSCR value into the FPSCR */

    /* Restore the critical section nesting depth. */
    LDR     R0, ulCriticalNestingConst              /* Load the address of the ulCriticalNesting variable into R0 */
    POP     {R1}                                    /* Pop the previously saved value of ulCriticalNesting off the stack */
    STR     R1, [R0]                                /* Store the value of ulCriticalNesting into address of ulCriticalNesting */

    /* Restore all system mode registers other than the SP (which is already being used). */
    POP     {R0-R12, R14}                           /* Load the values for R0-R14 from the stack that were saved when saving context */

    /* Return to the task code, loading CPSR on the way. */
    RFEIA   SP!                                     /* Use the RFEIA instruction to load the previous Current Program Status and Control Register and Program Counter */

    .endm

/******************************************************************************
 * vPortRestoreTaskContext is used to start the scheduler.
 *****************************************************************************/
.align 4
.type vPortRestoreTaskContext, %function
vPortRestoreTaskContext:
    /* Just need to restore the context to start the first task. */
    portRESTORE_CONTEXT


/******************************************************************************
 * FreeRTOS_SVC_Handler is used to handle Supervisor Calls
 *****************************************************************************/
.align 4
.type FreeRTOS_SVC_Handler, %function
FreeRTOS_SVC_Handler:
    /* Upon entering here the LR, or R14, will hold the address of the following instruction.
     * The instruction can be inspected to determine what SVC # was raised.
    */
    STMFD   SP!, {R11,R12,LR}         /* Store previous R11, R12, and the next instruction that would be run to the SVC Stack */
    MRS     R12, SPSR                 /* The SPSR will be the CPSR of the calling task, store it in R12 */
    ANDS    R12, R12, #0x20           /* Thumb Mode is bit 5 of the CPSR. Clear all other bits */
    LDRBNE  R12, [LR, #-2]            /* If not in Thumb mode set R12 the calling instruction will be 0x2 before, access that to get the SVC # */
    LDRBEQ  R12, [LR, #-4]            /* If not in Thumb mode set R12 the calling instruction will be 0x4 before, access that to get the SVC # */
    LDR     R14,  table               /* Load the address of the SVC jump table into R14 */
    LDR     R12, [R14, R12, lsl #2]   /* Shift the value of R12 left by two, add it to R14, then load the value in that address into R12 */
    BLX     R12                       /* Jump to the operation specified by the SVC */
    LDMFD   SP!, {R11,R12,pc}^        /* Restore the previously saved R11, R12, then load the following instruction into the PC */

table:
    .word   jumpTable

jumpTable:
    .word   svcPortYield                 /* 0 - vPortYield */
    .word   svcRaisePrivilege            /* 1 - portRAISE_PRIVILEGE */
    .word   svcPortEnterCritical         /* 2 - vPortEnterCritical */
    .word   svcPortExitCritical          /* 3 - vPortExitCritical */
    .word   svcPortDisableInterrupts     /* 4 - vPortDisableInterrupts */
    .word   svcPortEnableInterrupts      /* 5 - vPortEnableInterrupts */
    .word   svcResetPrivilege            /* 6 - portRESET_PRIVILEGE */

/*-----------------------------------------------------------*/
/* Yield to another task. */

.weak vPortYieldProcessor
.type vPortYieldProcessor, %function
svcPortYield:
    /* Restore SP and LR before calling vPortYieldProcessor */
    LDMFD   SP!, {R11,R12,LR}

vPortYieldProcessor:
/* TODO POSSIBLE_IMPROVEMENTS: As part of saving the context the processor swaps
 * to SYS_MODE and sets the SP to be the current task's SP. This means
 * that when running vTaskSwitchContext() any variables needed are saved onto the
 * caller's stack. I think it might make sense to swap back out of SYS_MODE
 * or swap back to the SVC Mode SP to run vTaskSwitchContext()
 */
    /* Save the context of the current task and select a new task to run. */
    portSAVE_CONTEXT

    BL vTaskSwitchContext /* Select a new task to swap to */

    /* Restore the context of the task selected to execute. */
    portRESTORE_CONTEXT


/*-----------------------------------------------------------*/
/* Yield to another task from within the FreeRTOS API */

.weak vPortYieldWithinAPI
.type vPortYieldWithinAPI, %function                
vPortYieldWithinAPI:
/* TODO POSSIBLE_IMPROVEMENTS: As part of saving the context the processor swaps
 * to SYS_MODE and sets the SP to be the current task's SP. This means
 * that when running vTaskSwitchContext() any variables needed are saved onto the
 * caller's stack. I think it might make sense to swap back out of SYS_MODE
 * or swap back to the SVC Mode SP to run vTaskSwitchContext()
 */
    SUB     LR, LR, #4      /* This function is called by an IRQ, the IRQ increases the LR by 4 so account for this */
    portSAVE_CONTEXT        /* Save the context of the current task */

    MOVW    R0, #0xFFF4     /* Clear the SSI flags */
    MOVT    R0, #0xFFFF
    LDR     R0, [R0]

    /* Select the next task to execute. */
    BL      vTaskSwitchContext

    /* Restore the context of the task selected to execute. */
    portRESTORE_CONTEXT
        
/*-------------------------------------------------------------------------------*/ 
/*
 * svcRaisePrivilege
 * Must return zero in R0 if caller was in user mode
*/
svcRaisePrivilege:
    MRS     R12, SPSR               /* Move CPSR into R12 */
    ANDS    R0, R12, #0x0F          /* USER_MODE is 0x10, so clear other bits . Now if R0 will only be 0 if it was in user mode*/
    ORREQ   R12, R12, #SYS_MODE     /* Set the MODE bits to SYS_MODE */
    MSREQ   SPSR_c, R12             /* Assign the new value to SPSR */
    BX      R14

/*-------------------------------------------------------------------------------*/
/* svcPortEnterCritical */

svcPortEnterCritical:
/* TODO POSSIBLE_IMPROVEMENTS: These first three instructions are the same thing
 * that svcDisableInterrupts does. Is it worth doing a bl to that instead of this duplication?
 * Also this only disables the IRQ interrupts, not FIQs. Is this wrong?
 */
    MRS     R11, SPSR                       /* The user mode CPSR is in the SPSR */
    ORR     R11, R11, #0x80                 /* Set bit 7, which is the interrupt enable bit of the CPSR */
    MSR     SPSR_c, R11                     /* Move the modified user mode CPSR into the SPSR */
    LDR     R11, ulCriticalNestingConst     /* Load address of current critical nesting count */
    LDR     R12, [R11]                      /* Load value of current critical nesting count */
    ADD     R12, R12, #1                    /* Add one to ulCriticalNesting */
    STR     R12, [R11]                      /* Store the modified ulCriticalNesting back into RAM */
    BX      R14

/*-------------------------------------------------------------------------------*/
/* svcPortExitCritical */
        
svcPortExitCritical:
    LDR     R11, ulCriticalNestingConst     /* Load address of current critical nesting count */
    LDR     R12, [R11]                      /* Load value of current critical nesting count */
    CMP     R12, #0                         /* Check if the count is 0 */
    BXEQ    R14                             /* If ulCriticalNesting is 0 return to caller */
    SUBS    R12, R12, #1                    /* Subtract one from ulCriticalNesting */
    STR     R12, [R11]                      /* Store the modified ulCriticalNesting back into RAM */
    BXNE    R14                             /* If the subtraction caused an overflow then leave */
    B       svcPortEnableInterrupts         /* Go to svcPortEnableInterrupts to enable interrupts */
/*
 * TODO POSSIBLE_IMPROVEMENTS:
 * With correct dynamic prediction the above branch takes 1 CPU cycle
 * With incorrect dynamic predicition it takes 8 CPU cycles.
 * Need to determine if this will always be "Correct" or not
 * If it is likely to be incorrect the duplicated instructions for enabling
 * interrupts may be better due to timing.
 *  MRS     R11, SPSR
 *  BIC     R11, R11, #0x80
 *  MSR     SPSR_c, R11
 *  BX      R14
*/

/*-------------------------------------------------------------------------------*/
/* svcPortDisableInterrupts */
        
svcPortDisableInterrupts:
    MRS     R11, SPSR           /* The user mode CPSR is in the SPSR */
    ORR     R11, R11, #0x80     /* Set bit 7, which is the interrupt enable bit of the CPSR */
    MSR     SPSR_c, R11         /* Move the modified user mode CPSR into the SPSR */
    BX      R14                 /* Return to FreeRTOS_SVC_Handler */

/*-------------------------------------------------------------------------------*/
/* svcPortEnableInterrupts */
        
svcPortEnableInterrupts:
    MRS     R11, SPSR           /* The user mode CPSR is in the SPSR */
    BIC     R11, R11, #0x80     /* Clear bit 7, which is the interrupt enable bit of the CPSR */ 
    MSR     SPSR_c, R11         /* Move the modified user mode CPSR into the SPSR */
    BX      R14                 /* Return to FreeRTOS_SVC_Handler */

/*-------------------------------------------------------------------------------*/
/* svcResetPrivilege */
/* Must return zero in R0 if caller was in user mode */

svcResetPrivilege:
    MRS     R12, SPSR           /* The user mode CPSR is in SPSR */
    BIC     R12, R12, #0x0F     /* Clear the last 4 bits, which are the MODE bits */
    MSREQ   SPSR_c, R12         /* Move the new value into the SPSR */
    BX      R14                 /* Jump back */

/******************************************************************************
 * prvMpuSetRegion is used to set the base address, access attributes, and the size 
 * and enable bits of a selected MPU region.
 * void _mpuSetRegion(unsigned region, unsigned base, unsigned size, unsigned access)
 *****************************************************************************/
.align 4 
.type prvMpuSetRegion, %function  
prvMpuSetRegion:
    AND   R0,  R0, #15               /* Only 15 possible regions, drop all other bits */   
    MCR   p15, #0, R0, c6, c2, #0    /* Select the MPU Region selected by region */
    MCR   p15, #0, R1, c6, c1, #0    /* Set the Base Address to be base */
    MCR   p15, #0, R3, c6, c1, #4    /* Set the Access Attributes to be access */
    MCR   p15, #0, R2, c6, c1, #2    /* Set the Size and Enable bits to be size */
    BX    LR
        
/******************************************************************************
 * prvMpuEnable is used to set the Enable bit of the MPU Enable Register to 1.
 *****************************************************************************/
.align 4 
.type prvMpuEnable, %function
prvMpuEnable:
    MRC   p15, #0, R0, c1, c0, #0   /* Read the current MPU control register into R0 */
    ORR   R0,  R0, #1               /* Set the enable bit to high */
    DSB                             /* Data sync */
    MCR   p15, #0, R0, c1, c0, #0   /* Write out previous MPU control register with a high enable bit */
    ISB                             /* Instruction sync */
    BX    LR

/******************************************************************************
 * prvMpuDisable is used to set the Enable bit of the MPU Enable Register to 0.
 *****************************************************************************/
.align 4 
.type prvMpuDisable, %function        
prvMpuDisable:
    MRC   p15, #0, R0, c1, c0, #0   /* Read the MPU enable register values into R0 */ 
    BIC   R0,  R0, #1               /* Clear out all bits in R0 except for bit 1 */
    DSB                             /* Wait for all pending explicit data accesses to complete */
    MCR   p15, #0, R0, c1, c0, #0   /* Write out to the MPU Enable Register */  
    ISB                             /* Flushes the pipeline and prefetch buffer(s) in the processor. Ensures all following instructions are fetched from cache or memory. */
    BX    LR


/*-------------------------------------------------------------------------------*/
/* xPortIsPrivileged
 * Must return zero in R0 if caller was in user mode
 */
.align 4
.weak xPortIsPrivileged
.type xPortIsPrivileged, %function
xPortIsPrivileged:
    MRS     R0, cpsr           /* Load value of SPSR into R0 */
    AND     R0, R0, #0x1F      /* Get the relevant mode bits */
    CMP     R0, #USER_MODE     /* Check if we're in user mode */
    MOVEQ   R0, #0x0           /* If we are in user mode set R0 to 0 for the return */
    MOVNE   R0, #0x01          /* Otherwise set R0 to 1 for the return */
    blx      LR

/*-------------------------------------------------------------------------------*/
/* ulPortCountLeadingZeros */
.align 4
.weak ulPortCountLeadingZeros
.type ulPortCountLeadingZeros, %function
ulPortCountLeadingZeros:
    CLZ     R0, R0    /* Count the leadign zeros and return in R0 */
    BX      LR


/*-------------------------------------------------------------------------------*/
/* vPortTaskUsesFPU */
/* TODO TCB_CONTEXT_SAVING: When we swap to storing context in the TCB we always save the FPRs.
 * Meaning that this function can be removed.
 */
/* TODO POSSIBLE_IMPROVEMENTS: I chose to save and restore R11 and R12 here.
 * However, this should be called before a task starts running anything, or using its registers.
 * Could potentially remove the push/pop of these to save CPU/Memory cycles and size of the binary.
 */
.global vPortTaskUsesFPU
.align 4
.type vPortTaskUsesFPU, %function
.func
vPortTaskUsesFPU:
    PUSH    {R11, R12}                          /* Save values of R11 and R12 so we can use them */
    LDR     R12, ulPortTaskHasFPUContextConst   /* Load address of ulPortTaskHasFPUContext into R12*/
    MOV     R11, #1                             /* Move #1 into R11 */
    STR     R11, [r12]                          /* Store 1 into ulPortTaskHasFPUContext, marking the task as using the FPU */
    MOV     R11, #0                             /* Move 0 into R11 */
    FMXR    FPSCR, r11                          /* Set the Floating Point Status and Control Register to 0 to start with */
    POP     {R11, R12}                          /* Restore  previous R11 and R12 */
    BX      LR                                  /* Return to caller */
.endfunc

/* TODO FUTURE_SPLIT_MACRO: For now these just use magic numbers that line up to the jump table.
 * When the split happens these can be changed to defines. */
/******************************************************************************
 * SVC handler is used to yield.
 *****************************************************************************/

.align 4
.type vPortYield, %function
.global vPortYield
vPortYield:
	SVC 0   /* Make the SVC to swap tasks */
    /* This should not return, so no branch instruction */

/*-------------------------------------------------------------------------------*/
/* vPortRaisePrivilege */

.align 4
.type vPortRaisePrivilege, %function
.global vPortRaisePrivilege
vPortRaisePrivilege:
    SVC 1   /* Make the SVC to raise privilege */
    BX  LR  /* Return to caller */

/*-------------------------------------------------------------------------------*/
/* vPortEnterCritical */

.align 4
.type vPortEnterCritical, %function
.global vPortEnterCritical
vPortEnterCritical:
    SVC 2   /* Make the SVC to enter a critical section */
    BX  LR  /* Return to caller */

/*-------------------------------------------------------------------------------*/
/* vPortExitCritical */

.align 4
.type vPortExitCritical, %function
.global vPortExitCritical
vPortExitCritical:
    SVC 3   /* Make the SVC to exit a critical section */
    BX  LR  /* Return to caller */

/*-------------------------------------------------------------------------------*/
/* vPortDisableInterrupts */

.align 4
.type vPortDisableInterrupts, %function
.global vPortDisableInterrupts
vPortDisableInterrupts:
    SVC 4   /* Make the SVC to disable interrupts */
    BX  LR  /* Return to caller */

/*-------------------------------------------------------------------------------*/
/* vPortEnableInterrupts */

.align 4
.type vPortEnableInterrupts, %function
.global vPortEnableInterrupts
vPortEnableInterrupts:
    SVC 5   /* Make the SVC to enable interrupts */
    BX  LR  /* Return to caller */

/*-------------------------------------------------------------------------------*/
/* vPortRaisePrivilege */

.align 4
.type vPortResetPrivilege, %function
.global vPortResetPrivilege
vPortResetPrivilege:
    SVC 1   /* Make the SVC to raise privilege */
    BX  LR  /* Return to caller */

/******************************************************************************
 * vPortPreemptiveTick is used to save context, increment the tick count, and 
 * then switch tasks if neeeded
 *****************************************************************************/
.global vPortPreemptiveTick
.align 8
.type vPortPreemptiveTick, %function
.func
vPortPreemptiveTick:
/* TODO POSSIBLE_IMPROVEMENTS: As part of saving the context the processor swaps
 * to SYS_MODE and sets the SP to be the current task's SP. This means
 * that when running vTaskSwitchContext() any variables needed are saved onto the
 * caller's stack. I think it might make sense to swap back out of SYS_MODE
 * or swap back to the SVC Mode SP to run vTaskSwitchContext()
 */
    /* Return to the interrupted instruction. */
    SUB        LR, LR, #4

    /* Save the context of the current task. */
    portSAVE_CONTEXT

    /* Clear interrupt flag in RTI. */
    MOVW    R0, #0xFC88
    MOVT    R0, #0xFFFF
    MOV     R1, #1
    STR     R1, [R0]

    /* Increment the tick count, making any adjustments to the blocked lists
    that may be necessary. */
    BL      xTaskIncrementTick

    /* If xTaskIncrementTick returned non-zero then select the next task to execute. */
    CMP    R0, #0
    BLNE    vTaskSwitchContext

    /* Restore the context of the task selected to execute. */
    portRESTORE_CONTEXT
    .endfunc

/* TODO POSSIBLE_IMPROVEMENTS: I don't think this actually gets used anywhere. 
 * The current RM46 board/project has work around to trigger IRQs.
 * Might be worth looking into why it does it the way it does, and if possible just use this.
 * Additionally, entering FRQ mode saves R8-R14 by default. Maybe just using that mode instead would
 * Make all the above SVCs/IRQs a lot easier since then we can use R8-R12 as scratch registers? 
 */
.align 4
.type FreeRTOS_IRQ_Handler, %function
FreeRTOS_IRQ_Handler:
    /* Return to the interrupted instruction. */
    SUB     LR, LR, #4

    /* TODO POSSIBLE_IMPROVEMENTS: If this is kept this function could swap to using SRS/RFE instead. */
    /* Push the return address and SPSR. */
    PUSH    {LR}
    MRS     LR, SPSR
    PUSH    {LR}

    /* Change to supervisor mode to allow reentry. */
    CPS     #SVC_MODE

    /* Push used registers. */
    PUSH    {R0-R3, R12}

    /* Increment nesting count.  R3 holds the address of ulPortInterruptNesting
    for future use.  R1 holds the original ulPortInterruptNesting value for
    future use. */
    LDR     R3, ulPortInterruptNestingConst
    LDR     R1, [R3]
    ADD     R0, R1, #1
    STR     R0, [R3]

    /* Ensure bit 2 of the stack pointer is clear.  R2 holds the bit 2 value for
    future use. */
    MOV     R0, SP
    AND     R2, R0, #4
    SUB     SP, SP, R2

    /* Call the interrupt handler. */
    PUSH    {R0-R3, LR}
    LDR      R1, vApplicationIRQHandlerConst
    BLX      R1
    POP     {R0-R3, LR}
    ADD     SP, SP, R2

    CPSID    i
    DSB
    ISB

    /* Write to the EOI register. */
    LDR     R0, ulICCEOIRConst
    LDR     R2, [R0]
    STR     R0, [R2]

    /* Restore the old nesting count. */
    STR     R1, [R3]

    /* A context switch is never performed if the nesting count is not 0. */
    CMP     R1, #0
    BNE     exit_without_switch

    /* Did the interrupt request a context switch?  R1 holds the address of
    ulPortYieldRequired and R0 the value of ulPortYieldRequired for future use. */
    LDR     R1, ulPortYieldRequiredConst
    LDR     R0, [R1]
    CMP     R0, #0
    BNE     switch_before_exit

exit_without_switch:
    /* No context switch.  Restore used registers, LR_irq and SPSR before returning. */
    POP     {R0-R3, R12}
    CPS     #IRQ_MODE
    POP     {LR}
    MSR     SPSR_cxsf, LR
    POP     {LR}
    MOVS    PC, LR

switch_before_exit:
    /* A context swtich is to be performed. Clear the context switch pending flag. */
    MOV     R0, #0
    STR     R0, [R1]

    /* Restore used registers, LR-irq and SPSR before saving the context
    to the task stack. */
    POP     {R0-R3, R12}
    CPS     #IRQ_MODE
    POP     {LR}
    MSR     SPSR_cxsf, LR
    POP     {LR}
    portSAVE_CONTEXT

    /* Call the function that selects the new task to execute.
    vTaskSwitchContext() if vTaskSwitchContext() uses LDRD or STRD
    instructions, or 8 byte aligned stack allocated data.  LR does not need
    saving as a new LR will be loaded by portRESTORE_CONTEXT anyway. */
    LDR     R0, vTaskSwitchContextConst
    BLX     R0

    /* Restore the context of, and branch to, the task selected to execute next. */
    portRESTORE_CONTEXT

/* TODO GLOBAL_VARIABLE_RISKS: A few of these are variables that if modified by a user
 * could allow them to potentially elevate their permission, or break the system. */
/* TODO POSSIBLE_IMPROVEMENTS: Change these from "Const" to "Addr". Const is inaccurate. */
ulICCEOIRConst:                 .word ulICCEOIR
pxCurrentTCBConst:              .word pxCurrentTCB
ulCriticalNestingConst:         .word ulCriticalNesting
ulPortTaskHasFPUContextConst:   .word ulPortTaskHasFPUContext
vTaskSwitchContextConst:        .word vTaskSwitchContext
vApplicationIRQHandlerConst:    .word vApplicationIRQHandler
ulPortInterruptNestingConst:    .word ulPortInterruptNesting
ulPortYieldRequiredConst:       .word ulPortYieldRequired

/* TODO FUTURE_SPLIT_MACRO: These are currently extern variables declared in the port.c file
 * This is done to ensure that the correct regions are looped over.
 * However this approach requires loading the variable address, than the value.
 * When the portmacro.h file is split into two, these can be replaced with defined values 
 */
ulTotalMPURegionsConst:             .word   ulTotalMPURegions
ulStackMPURegionConst:              .word   ulStackMPURegion
ulLastConfigurableMPURegionConst:   .word   ulLastConfigurableMPURegion

.end
