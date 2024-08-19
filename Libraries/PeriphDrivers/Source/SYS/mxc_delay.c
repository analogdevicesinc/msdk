/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. (now owned by 
 * Analog Devices, Inc.),
 * Copyright (C) 2023-2024 Analog Devices, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************/

/* **** Includes **** */
#include <stdint.h>
#include <stddef.h>
#include "mxc_device.h"
#include "mxc_delay.h"
#include "mxc_sys.h"

#ifdef __riscv

int MXC_Delay(uint32_t us)
{
    // Check if there is nothing to do
    if (us == 0) {
        return E_NO_ERROR;
    }

    // Calculate number of cycles needed.
    uint32_t ticks = (MXC_SYS_RiscVClockRate() / 1000000) * us;

    CSR_SetPCMR(0); // Turn off counter
    CSR_SetPCCR(0); // Clear counter register
    CSR_SetPCER(1); // Enable counting of cycles
    CSR_SetPCMR(3); // Turn on counter

    while (CSR_GetPCCR() < ticks) {
        // Wait for counter to reach the tick count.
    }
    return E_NO_ERROR;
}

int MXC_DelayAsync(uint32_t us, mxc_delay_complete_t callback)
{
    return E_NOT_SUPPORTED;
}

int MXC_DelayCheck(void)
{
    return E_NOT_SUPPORTED;
}

void MXC_DelayAbort(void) {}

#else

/* **** File Scope Variables **** */
static volatile int overflows = -1;
static uint32_t endtick;
static uint32_t ctrl_save;
static mxc_delay_complete_t cbFunc;

static void MXC_DelayInit(uint32_t us);
extern void SysTick_Handler(void);

/* ************************************************************************** */
__weak void SysTick_Handler(void)
{
    MXC_DelayHandler();
}

/* ************************************************************************** */
void MXC_DelayHandler(void)
{
    // Check and clear overflow flag
    if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) {
        // Decrement overflow flag if delay is still ongoing
        if (overflows > 0) {
            overflows--;
        } else {
            MXC_DelayAbort();

            if (cbFunc != NULL) {
                cbFunc(E_NO_ERROR);
                cbFunc = NULL;
            }
        }
    }
}

/* ************************************************************************** */
static void MXC_DelayInit(uint32_t us)
{
    uint32_t starttick, reload, ticks, lastticks;

    // Record the current tick value and clear the overflow flag
    starttick = SysTick->VAL;

    // Save the state of control register (and clear the overflow flag)
    ctrl_save = SysTick->CTRL & ~SysTick_CTRL_COUNTFLAG_Msk;

    // If the SysTick is not running, configure and start it
    if (!(SysTick->CTRL & SysTick_CTRL_ENABLE_Msk)) {
        SysTick->LOAD = SysTick_LOAD_RELOAD_Msk;
        SysTick->VAL = SysTick_VAL_CURRENT_Msk;
        SysTick->CTRL = SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_CLKSOURCE_Msk;
        starttick = SysTick_VAL_CURRENT_Msk;
        reload = SysTick_LOAD_RELOAD_Msk + 1;
    } else {
        reload = SysTick->LOAD + 1; // get the current reload value
    }

    // Calculate the total number of ticks to delay
    ticks = (uint32_t)(((uint64_t)us * (uint64_t)SystemCoreClock) / 1000000);

    // How many overflows of the SysTick will occur
    overflows = ticks / reload;

    // How many remaining ticks after the last overflow
    lastticks = ticks % reload;

    // Check if there will be another overflow due to the current value of the SysTick
    if (lastticks >= starttick) {
        overflows++;
        endtick = reload - (lastticks - starttick);
    } else {
        endtick = starttick - lastticks;
    }
}

/* ************************************************************************** */
int MXC_DelayAsync(uint32_t us, mxc_delay_complete_t callback)
{
    cbFunc = callback;

    // Check if timeout currently ongoing
    if (overflows > 0) {
        return E_BUSY;
    }

    // Check if there is nothing to do
    if (us == 0) {
        return E_NO_ERROR;
    }

    // Calculate the necessary delay and start the timer
    MXC_DelayInit(us);

    // Enable SysTick interrupt if necessary
    if (overflows > 0) {
        SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
    }

    return E_NO_ERROR;
}

/* ************************************************************************** */
int MXC_DelayCheck(void)
{
    // Check if timeout currently ongoing
    if (overflows < 0) {
        if (cbFunc != NULL) {
            cbFunc(E_NO_ERROR);
            cbFunc = NULL;
        }

        return E_NO_ERROR;
    }

    // Check the global values
    if ((overflows == 0) && (SysTick->VAL <= endtick)) {
        MXC_DelayAbort();

        if (cbFunc != NULL) {
            cbFunc(E_NO_ERROR);
            cbFunc = NULL;
        }

        return E_NO_ERROR;
    }

    return E_BUSY;
}

/* ************************************************************************** */
void MXC_DelayAbort(void)
{
    if (cbFunc != NULL) {
        cbFunc(E_ABORT);
        cbFunc = NULL;
    }

    SysTick->CTRL = ctrl_save;
    overflows = -1;
}

/* ************************************************************************** */
int MXC_Delay(uint32_t us)
{
    // Check if timeout currently ongoing
    if (overflows > 0) {
        return E_BUSY;
    }

    // Check if there is nothing to do
    if (us == 0) {
        return E_NO_ERROR;
    }

    // Calculate the necessary delay and start the timer
    MXC_DelayInit(us);

    // Wait for the number of overflows
    while (overflows > 0) {
        // If SysTick interrupts are enabled, COUNTFLAG will never be set here and
        // overflows will be decremented in the ISR. If SysTick interrupts are
        // disabled, overflows is decremented here.
        if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) {
            overflows--;
        }
    }

    // Wait for the counter value
    while (SysTick->VAL > endtick) {}

    MXC_DelayAbort();
    return E_NO_ERROR;
}

#endif // __riscv
