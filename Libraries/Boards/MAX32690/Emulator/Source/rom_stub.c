/******************************************************************************
 * Copyright (C) 2023 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *
 ******************************************************************************/

#include "max32690.h"
#include "mxc_device.h"

/* The stack address is defined by the linker
 * It is typed as a function here to avoid compiler warnings
 */
extern void __StackTop(void);

void Reset_Handler(void);
void NMI_Handler_ROM(void);
void HardFault_Handler_ROM(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);

/* Create a vector table to locate at zero in the ROM for handling reset and startup */
__attribute__((section(".rom_vector"))) void (*const rom_vector[])(void) = {
    __StackTop, /* Top of Stack */
    Reset_Handler, /* Reset Handler */
    NMI_Handler_ROM, /* NMI Handler */
    HardFault_Handler_ROM, /* Hard Fault Handler */
    MemManage_Handler, /* MPU Fault Handler */
    BusFault_Handler, /* Bus Fault Handler */
    UsageFault_Handler, /* Usage Fault Handler */
    0, /* Reserved */
    0, /* Reserved */
    0, /* Reserved */
    0, /* Reserved */
    SVC_Handler, /* SVCall Handler */
    DebugMon_Handler, /* Debug Monitor Handler */
    0, /* Reserved */
    PendSV_Handler, /* PendSV Handler */
    SysTick_Handler, /* SysTick Handler */
};

/* This is needed to handle the NMI at POR */
__attribute__((section(".rom_handlers"))) void NMI_Handler_ROM(void)
{
    __NOP();
}

/* This is needed to handle the fault after initial programming */
__attribute__((section(".rom_handlers"))) void HardFault_Handler_ROM(void)
{
    NVIC_SystemReset();
}
