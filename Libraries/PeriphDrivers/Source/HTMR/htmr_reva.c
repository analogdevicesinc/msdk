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

#include <stdio.h>
#include <stddef.h>
#include <stdint.h>
#include "mxc_device.h"
#include "mxc_assert.h"
#include "mxc_lock.h"
#include "mxc_sys.h"
#include "mxc_delay.h"
#include "htmr_reva_regs.h"
#include "htmr.h"
#include "htmr_reva.h"

/* **** Definitions **** */
#define HTMR_CTRL_RESET_DEFAULT (0x8000UL)
#define BUSY_TIMEOUT (10000) // Timeout counts for the Busy bit
#define ASYNC_MODE (MXC_F_HTMR_REVA_CTRL_ACRE & htmr->ctrl)

#define MXC_HTMR_ALL_INT_FLAGS \
    (MXC_F_HTMR_REVA_CTRL_RDY | MXC_F_HTMR_REVA_CTRL_ALDF | MXC_F_HTMR_REVA_CTRL_ALSF)
#define MXC_HTMR_ALL_INT_ENABLES \
    (MXC_F_HTMR_REVA_CTRL_RDYE | MXC_F_HTMR_REVA_CTRL_ADE | MXC_F_HTMR_REVA_CTRL_ASE)

/* ***** Functions ***** */
int MXC_HTMR_RevA_Init(mxc_htmr_reva_regs_t *htmr, uint32_t sec, uint8_t ssec)
{
    if (htmr == NULL) {
        return E_NULL_PTR;
    }

    if (MXC_HTMR_CheckBusy((mxc_htmr_regs_t *)htmr)) {
        return E_BUSY;
    }

    htmr->ctrl = MXC_F_HTMR_REVA_CTRL_WE; //  Allow Writes

    if (MXC_HTMR_CheckBusy((mxc_htmr_regs_t *)htmr)) {
        return E_BUSY;
    }

    htmr->ctrl = HTMR_CTRL_RESET_DEFAULT; // Start with a Clean Register

    if (MXC_HTMR_CheckBusy((mxc_htmr_regs_t *)htmr)) {
        return E_BUSY;
    }

    htmr->ctrl |= MXC_F_HTMR_REVA_CTRL_WE; // Set Write Enable, allow writing to reg.

    if (MXC_HTMR_CheckBusy((mxc_htmr_regs_t *)htmr)) {
        return E_BUSY;
    }

    htmr->ssec = ssec;

    if (MXC_HTMR_CheckBusy((mxc_htmr_regs_t *)htmr)) {
        return E_BUSY;
    }

    htmr->sec = sec;

    if (MXC_HTMR_CheckBusy((mxc_htmr_regs_t *)htmr)) {
        return E_BUSY;
    }

    // If the peripheral clock is twice the frequency of the clock driving HTMR
    // we can enable Asynchronous read mode for htmr->sec and htmr->ssec registers
    if (PeripheralClock > (2 * IBRO_FREQ)) {
        htmr->ctrl |= MXC_F_HTMR_REVA_CTRL_ACRE;
    }

    htmr->ctrl &= ~MXC_F_HTMR_REVA_CTRL_WE; // Prevent Writing...

    return E_SUCCESS;
}

int MXC_HTMR_RevA_Start(mxc_htmr_reva_regs_t *htmr)
{
    if (htmr == NULL) {
        return E_NULL_PTR;
    }

    if (MXC_HTMR_CheckBusy((mxc_htmr_regs_t *)htmr)) {
        return E_BUSY;
    }

    htmr->ctrl |= MXC_F_HTMR_REVA_CTRL_WE; // Allow writing to registers

    if (MXC_HTMR_CheckBusy((mxc_htmr_regs_t *)htmr)) {
        return E_BUSY;
    }

    // Can only write if WE=1 and BUSY=0
    htmr->ctrl |= MXC_F_HTMR_REVA_CTRL_HTEN; // setting RTCE = 1

    if (MXC_HTMR_CheckBusy((mxc_htmr_regs_t *)htmr)) {
        return E_BUSY;
    }

    htmr->ctrl &= ~MXC_F_HTMR_REVA_CTRL_WE; // Prevent Writing...

    return E_SUCCESS;
}

int MXC_HTMR_RevA_Stop(mxc_htmr_reva_regs_t *htmr)
{
    if (htmr == NULL) {
        return E_NULL_PTR;
    }

    if (MXC_HTMR_CheckBusy((mxc_htmr_regs_t *)htmr)) {
        return E_BUSY;
    }

    htmr->ctrl |= MXC_F_HTMR_REVA_CTRL_WE; // Allow writing to registers

    if (MXC_HTMR_CheckBusy((mxc_htmr_regs_t *)htmr)) {
        return E_BUSY;
    }

    // Can only write if WE=1 and BUSY=0
    htmr->ctrl &= ~MXC_F_HTMR_REVA_CTRL_HTEN; // setting RTCE = 0

    if (MXC_HTMR_CheckBusy((mxc_htmr_regs_t *)htmr)) {
        return E_BUSY;
    }

    htmr->ctrl &= ~MXC_F_HTMR_REVA_CTRL_WE; // Prevent Writing...

    return E_SUCCESS;
}

int MXC_HTMR_RevA_GetShortCount(mxc_htmr_reva_regs_t *htmr)
{
    if (htmr == NULL) {
        return E_NULL_PTR;
    }

    // Don't bother calling CheckBusy() if we're in async mode
    if (!ASYNC_MODE) {
        if (MXC_HTMR_CheckBusy((mxc_htmr_regs_t *)htmr)) {
            return E_BUSY;
        }
    }

    return htmr->ssec;
}

int MXC_HTMR_RevA_GetLongCount(mxc_htmr_reva_regs_t *htmr)
{
    if (htmr == NULL) {
        return E_NULL_PTR;
    }

    // Don't bother calling CheckBusy() if we're in async mode
    if (!ASYNC_MODE) {
        if (MXC_HTMR_CheckBusy((mxc_htmr_regs_t *)htmr)) {
            return E_BUSY;
        }
    }

    return htmr->sec;
}

int MXC_HTMR_RevA_SetLongAlarm(mxc_htmr_reva_regs_t *htmr, uint32_t ras)
{
    // compare with maximum value
    if (ras > 0xFFFFF) {
        return E_BAD_PARAM;
    }

    if (MXC_HTMR_DisableInt((mxc_htmr_regs_t *)htmr, MXC_F_HTMR_REVA_CTRL_ADE) == E_BUSY) {
        return E_BUSY;
    }

    if (MXC_HTMR_CheckBusy((mxc_htmr_regs_t *)htmr)) {
        return E_BUSY;
    }

    htmr->ras = (ras << MXC_F_HTMR_REVA_RAS_RAS_POS) & MXC_F_HTMR_REVA_RAS_RAS;

    if (MXC_HTMR_EnableInt((mxc_htmr_regs_t *)htmr, MXC_F_HTMR_REVA_CTRL_ADE) == E_BUSY) {
        return E_BUSY;
    }

    return E_SUCCESS;
}

int MXC_HTMR_RevA_SetShortAlarm(mxc_htmr_reva_regs_t *htmr, uint32_t rssa)
{
    if (MXC_HTMR_DisableInt((mxc_htmr_regs_t *)htmr, MXC_F_HTMR_REVA_CTRL_ASE) == E_BUSY) {
        return E_BUSY;
    }

    if (MXC_HTMR_CheckBusy((mxc_htmr_regs_t *)htmr)) {
        return E_BUSY;
    }

    htmr->rssa = (rssa << MXC_F_HTMR_REVA_RSSA_RSSA_POS) & MXC_F_HTMR_REVA_RSSA_RSSA;

    if (MXC_HTMR_EnableInt((mxc_htmr_regs_t *)htmr, MXC_F_HTMR_REVA_CTRL_ASE) == E_BUSY) {
        return E_BUSY;
    }

    return E_SUCCESS;
}

int MXC_HTMR_RevA_CheckBusy(mxc_htmr_reva_regs_t *htmr)
{
    MXC_DelayAsync(MXC_DELAY_USEC(BUSY_TIMEOUT), NULL);

    while (htmr->ctrl & MXC_F_HTMR_CTRL_BUSY) {
        MXC_DelayAbort();

        if (MXC_DelayCheck() != E_BUSY) {
            return E_BUSY;
        }
    }

    return E_SUCCESS;
}

int MXC_HTMR_RevA_GetFlags(mxc_htmr_reva_regs_t *htmr)
{
    if (htmr == NULL) {
        return E_NULL_PTR;
    }

    return htmr->ctrl & MXC_HTMR_ALL_INT_FLAGS;
}

int MXC_HTMR_RevA_ClearFlags(mxc_htmr_reva_regs_t *htmr, int flags)
{
    if (MXC_HTMR_CheckBusy((mxc_htmr_regs_t *)htmr)) {
        return E_BUSY;
    }

    htmr->ctrl &= ~(flags & MXC_HTMR_ALL_INT_FLAGS);

    return E_SUCCESS;
}

int MXC_HTMR_RevA_EnableInt(mxc_htmr_reva_regs_t *htmr, uint32_t mask)
{
    if (MXC_HTMR_CheckBusy((mxc_htmr_regs_t *)htmr)) {
        return E_BUSY;
    }

    htmr->ctrl |= (mask & MXC_HTMR_ALL_INT_ENABLES); // Disable Long Interval Interrupt

    if (MXC_HTMR_CheckBusy((mxc_htmr_regs_t *)htmr)) {
        return E_BUSY;
    }

    return E_SUCCESS;
}

int MXC_HTMR_RevA_DisableInt(mxc_htmr_reva_regs_t *htmr, uint32_t mask)
{
    if (MXC_HTMR_CheckBusy((mxc_htmr_regs_t *)htmr)) {
        return E_BUSY;
    }

    htmr->ctrl &= ~(mask & MXC_HTMR_ALL_INT_ENABLES); // Disable Long Interval Interrupt

    if (MXC_HTMR_CheckBusy((mxc_htmr_regs_t *)htmr)) {
        return E_BUSY;
    }

    return E_SUCCESS;
}
