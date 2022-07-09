/* ****************************************************************************
 * Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
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
 *************************************************************************** */


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
#define HTMR_CTRL_RESET_DEFAULT     (0x8000UL)
#define BUSY_TIMEOUT                (10000)         // Timeout counts for the Busy bit
#define ASYNC_MODE                  (htmr->ctrl & MXC_F_HTMR_REVA_CTRL_ACRE)

#define MXC_HTMR_ALL_INT_FLAGS      (MXC_F_HTMR_REVA_CTRL_RDY  | MXC_F_HTMR_REVA_CTRL_ALDF | MXC_F_HTMR_REVA_CTRL_ALSF)
#define MXC_HTMR_ALL_INT_ENABLES    (MXC_F_HTMR_REVA_CTRL_RDYE | MXC_F_HTMR_REVA_CTRL_ADE  | MXC_F_HTMR_REVA_CTRL_ASE)

/* ***** Functions ***** */
int MXC_HTMR_RevA_Init (mxc_htmr_reva_regs_t *htmr, uint32_t sec, uint8_t ssec)
{
    if (htmr == NULL) {
        return E_NULL_PTR;
    }

    if (MXC_HTMR_CheckBusy ((mxc_htmr_regs_t*) htmr)) {
        return E_BUSY;
    }
    
    htmr->ctrl = MXC_F_HTMR_REVA_CTRL_WE;       //  Allow Writes
    
    if (MXC_HTMR_CheckBusy ((mxc_htmr_regs_t*) htmr)) {
        return E_BUSY;
    }
    
    htmr->ctrl = HTMR_CTRL_RESET_DEFAULT;  // Start with a Clean Register
    
    if (MXC_HTMR_CheckBusy ((mxc_htmr_regs_t*) htmr)) {
        return E_BUSY;
    }
    
    htmr->ctrl |= MXC_F_HTMR_REVA_CTRL_WE;      // Set Write Enable, allow writing to reg.
    
    if (MXC_HTMR_CheckBusy ((mxc_htmr_regs_t*) htmr)) {
        return E_BUSY;
    }
    
    htmr->ssec = ssec;
    
    if (MXC_HTMR_CheckBusy ((mxc_htmr_regs_t*) htmr)) {
        return E_BUSY;
    }
    
    htmr->sec = sec;
    
    if (MXC_HTMR_CheckBusy ((mxc_htmr_regs_t*) htmr)) {
        return E_BUSY;
    }
    
    // If the peripheral clock is twice the frequency of the clock driving HTMR
    // we can enable Asynchronous read mode for htmr->sec and htmr->ssec registers
    if (PeripheralClock > (2 * IBRO_FREQ)) {
        htmr->ctrl |= MXC_F_HTMR_REVA_CTRL_ACRE;
    }
    
    htmr->ctrl &= ~MXC_F_HTMR_REVA_CTRL_WE;       // Prevent Writing...
    
    return E_SUCCESS;
}

int MXC_HTMR_RevA_Start (mxc_htmr_reva_regs_t *htmr)
{
    if (htmr == NULL) {
        return E_NULL_PTR;
    }

    if (MXC_HTMR_CheckBusy ((mxc_htmr_regs_t*) htmr)) {
        return E_BUSY;
    }
    
    htmr->ctrl |= MXC_F_HTMR_REVA_CTRL_WE;       // Allow writing to registers
    
    if (MXC_HTMR_CheckBusy ((mxc_htmr_regs_t*) htmr)) {
        return E_BUSY;
    }
    
    // Can only write if WE=1 and BUSY=0
    htmr->ctrl |= MXC_F_HTMR_REVA_CTRL_HTEN;     // setting RTCE = 1
    
    if (MXC_HTMR_CheckBusy ((mxc_htmr_regs_t*) htmr)) {
        return E_BUSY;
    }
    
    htmr->ctrl &= ~MXC_F_HTMR_REVA_CTRL_WE;      // Prevent Writing...
    
    return E_SUCCESS;
}

int MXC_HTMR_RevA_Stop (mxc_htmr_reva_regs_t *htmr)
{
    if (htmr == NULL) {
        return E_NULL_PTR;
    }

    if (MXC_HTMR_CheckBusy ((mxc_htmr_regs_t*) htmr)) {
        return E_BUSY;
    }
    
    htmr->ctrl |= MXC_F_HTMR_REVA_CTRL_WE;      // Allow writing to registers
    
    if (MXC_HTMR_CheckBusy ((mxc_htmr_regs_t*) htmr)) {
        return E_BUSY;
    }
    
    // Can only write if WE=1 and BUSY=0
    htmr->ctrl &= ~MXC_F_HTMR_REVA_CTRL_HTEN;  // setting RTCE = 0
    
    if (MXC_HTMR_CheckBusy ((mxc_htmr_regs_t*) htmr)) {
        return E_BUSY;
    }
    
    htmr->ctrl &= ~MXC_F_HTMR_REVA_CTRL_WE;       // Prevent Writing...
    
    return E_SUCCESS;
}

int MXC_HTMR_RevA_GetShortCount (mxc_htmr_reva_regs_t *htmr)
{
    if (htmr == NULL) {
        return E_NULL_PTR;
    }
    
    // Don't bother calling CheckBusy() if we're in async mode
    if (!ASYNC_MODE) {
        if (MXC_HTMR_CheckBusy ((mxc_htmr_regs_t*) htmr)) {
            return E_BUSY;
        }
    }
    
    return htmr->ssec;
}

int MXC_HTMR_RevA_GetLongCount (mxc_htmr_reva_regs_t *htmr)
{
    if (htmr == NULL) {
        return E_NULL_PTR;
    }
    
    // Don't bother calling CheckBusy() if we're in async mode
    if (!ASYNC_MODE) {
        if (MXC_HTMR_CheckBusy ((mxc_htmr_regs_t*) htmr)) {
            return E_BUSY;
        }
    }
    
    return htmr->sec;
}

int MXC_HTMR_RevA_SetLongAlarm (mxc_htmr_reva_regs_t *htmr, uint32_t ras)
{
    // compare with maximum value
    if (ras > 0xFFFFF) {
        return E_BAD_PARAM;
    }
    
    if (MXC_HTMR_DisableInt ((mxc_htmr_regs_t*) htmr, MXC_F_HTMR_REVA_CTRL_ADE) == E_BUSY) {
        return E_BUSY;
    }
    
    if (MXC_HTMR_CheckBusy ((mxc_htmr_regs_t*) htmr)) {
        return E_BUSY;
    }
    
    htmr->ras = (ras << MXC_F_HTMR_RAS_RAS_POS) & MXC_F_HTMR_RAS_RAS;
    
    if (MXC_HTMR_EnableInt ((mxc_htmr_regs_t*) htmr, MXC_F_HTMR_REVA_CTRL_ADE) == E_BUSY) {
        return E_BUSY;
    }
    
    return E_SUCCESS;
}

int MXC_HTMR_RevA_SetShortAlarm (mxc_htmr_reva_regs_t *htmr, uint32_t rssa)
{
    if (MXC_HTMR_DisableInt ((mxc_htmr_regs_t*) htmr, MXC_F_HTMR_REVA_CTRL_ASE) == E_BUSY) {
        return E_BUSY;
    }
    
    if (MXC_HTMR_CheckBusy ((mxc_htmr_regs_t*) htmr)) {
        return E_BUSY;
    }
    
    htmr->rssa = (rssa << MXC_F_HTMR_RSSA_RSSA_POS) & MXC_F_HTMR_RSSA_RSSA;
    
    if (MXC_HTMR_EnableInt ((mxc_htmr_regs_t*) htmr, MXC_F_HTMR_REVA_CTRL_ASE) == E_BUSY) {
        return E_BUSY;
    }
    
    return E_SUCCESS;
}

int MXC_HTMR_RevA_CheckBusy (mxc_htmr_reva_regs_t *htmr)
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

int MXC_HTMR_RevA_GetFlags (mxc_htmr_reva_regs_t *htmr)
{
    if (htmr == NULL) {
        return E_NULL_PTR;
    }
    
    return htmr->ctrl & MXC_HTMR_ALL_INT_FLAGS;
}

int MXC_HTMR_RevA_ClearFlags (mxc_htmr_reva_regs_t *htmr, int flags)
{
    if (MXC_HTMR_CheckBusy ((mxc_htmr_regs_t*) htmr)) {
        return E_BUSY;
    }
    
    htmr->ctrl &= ~(flags & MXC_HTMR_ALL_INT_FLAGS);
    
    return E_SUCCESS;
}

int MXC_HTMR_RevA_EnableInt (mxc_htmr_reva_regs_t* htmr, uint32_t mask)
{

    if (MXC_HTMR_CheckBusy ((mxc_htmr_regs_t*) htmr)) {
        return E_BUSY;
    }
    
    htmr->ctrl |= (mask & MXC_HTMR_ALL_INT_ENABLES);    // Disable Long Interval Interrupt
    
    if (MXC_HTMR_CheckBusy ((mxc_htmr_regs_t*) htmr)) {
        return E_BUSY;
    }
    
    return E_SUCCESS;
}

int MXC_HTMR_RevA_DisableInt (mxc_htmr_reva_regs_t* htmr, uint32_t mask)
{

    if (MXC_HTMR_CheckBusy ((mxc_htmr_regs_t*) htmr)) {
        return E_BUSY;
    }
    
    htmr->ctrl &= ~(mask & MXC_HTMR_ALL_INT_ENABLES);    // Disable Long Interval Interrupt
    
    if (MXC_HTMR_CheckBusy ((mxc_htmr_regs_t*) htmr)) {
        return E_BUSY;
    }
    
    return E_SUCCESS;
}
