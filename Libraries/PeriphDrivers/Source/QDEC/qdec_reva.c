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
#include "mxc_device.h"
#include "mxc_errors.h"
#include "mxc_assert.h"
#include "mxc_sys.h"
#include "mxc_lock.h"
#include "qdec.h"
#include "qdec_reva.h"

#define QDEC_IE_MASK                                                                            \
    (MXC_F_QDEC_REVA_INTEN_INDEX | MXC_F_QDEC_REVA_INTEN_QERR | MXC_F_QDEC_REVA_INTEN_COMPARE | \
     MXC_F_QDEC_REVA_INTEN_MAXCNT | MXC_F_QDEC_REVA_INTEN_CAPTURE | MXC_F_QDEC_REVA_INTEN_DIR | \
     MXC_F_QDEC_REVA_INTEN_MOVE)

#define QDEC_IF_MASK                                                                            \
    (MXC_F_QDEC_REVA_INTFL_INDEX | MXC_F_QDEC_REVA_INTFL_QERR | MXC_F_QDEC_REVA_INTFL_COMPARE | \
     MXC_F_QDEC_REVA_INTFL_MAXCNT | MXC_F_QDEC_REVA_INTFL_CAPTURE | MXC_F_QDEC_REVA_INTFL_DIR | \
     MXC_F_QDEC_REVA_INTFL_MOVE)

static mxc_qdec_cb_t async_callback;

int MXC_QDEC_RevA_Init(mxc_qdec_reva_regs_t *qdec, mxc_qdec_req_t *req)
{
    // Disable QDEC to configure
    qdec->ctrl &= ~MXC_F_QDEC_REVA_CTRL_EN;

    // Set count mode
    qdec->ctrl |= ((req->mode << MXC_F_QDEC_REVA_CTRL_MODE_POS) & MXC_F_QDEC_REVA_CTRL_MODE);

    // Phase swap
    if (req->swap) {
        qdec->ctrl |= MXC_F_QDEC_REVA_CTRL_SWAP;
    } else {
        qdec->ctrl &= ~MXC_F_QDEC_REVA_CTRL_SWAP;
    }

    // Set sticky state
    qdec->ctrl |= ((req->sticky << MXC_F_QDEC_REVA_CTRL_STICKY_POS) & MXC_F_QDEC_REVA_CTRL_STICKY);

    qdec->ctrl |= ((req->clkdiv << MXC_F_QDEC_REVA_CTRL_PSC_POS) & MXC_F_QDEC_REVA_CTRL_PSC);

    // Set max and min count values
    MXC_QDEC_RevA_SetMaxCount(qdec, req->maxcnt);
    MXC_QDEC_RevA_SetInitial(qdec, req->initial);

    qdec->ctrl &= ~(MXC_F_QDEC_REVA_CTRL_RST_MAXCNT | MXC_F_QDEC_REVA_CTRL_RST_INDEX);

    // Reset on Max Count Match
    if (req->rst == MXC_QDEC_RST_ON_MAXCNT) {
        qdec->ctrl |= MXC_F_QDEC_REVA_CTRL_RST_MAXCNT;

        // Clear flag before enabling interrupt
        qdec->intfl |= MXC_F_QDEC_REVA_INTFL_MAXCNT;
        qdec->inten |= MXC_F_QDEC_REVA_INTEN_MAXCNT;

        // Reset on Index
    } else if (req->rst == MXC_QDEC_RST_ON_INDEX) {
        qdec->ctrl |= MXC_F_QDEC_REVA_CTRL_RST_INDEX;

        // Clear flag before enabling interrupt
        qdec->intfl |= MXC_F_QDEC_REVA_INTFL_INDEX;
        qdec->inten |= MXC_F_QDEC_REVA_INTEN_INDEX;

    } else {
        return E_BAD_PARAM;
    }

    // Enable capture or compare function before enabling
    if (req->func == MXC_QDEC_CAPTURE) {
        // Clear flag before enabling interrupt
        qdec->intfl |= MXC_F_QDEC_REVA_INTFL_CAPTURE;
        qdec->inten |= MXC_F_QDEC_REVA_INTEN_CAPTURE;

    } else if (req->func == MXC_QDEC_COMPARE) {
        MXC_QDEC_RevA_SetCompare(qdec, req->compare);

        // Clear flag before enabling interrupt
        qdec->intfl |= MXC_F_QDEC_REVA_INTFL_COMPARE;
        qdec->inten |= MXC_F_QDEC_REVA_INTEN_COMPARE;
    }

    // Save for callback
    if (req->callback != NULL) {
        async_callback = req->callback;
    }

    qdec->ctrl |= MXC_F_QDEC_REVA_CTRL_EN;

    return E_NO_ERROR;
}

int MXC_QDEC_RevA_Shutdown(mxc_qdec_reva_regs_t *qdec)
{
    qdec->ctrl &= ~MXC_F_QDEC_REVA_CTRL_EN;

    // Disable and Clear interupts
    qdec->inten = 0;
    qdec->intfl = QDEC_IF_MASK;

    // Clear registers
    qdec->ctrl = 0;
    qdec->maxcnt = 0xFFFFFFFF;
    qdec->initial = 0;

    return E_NO_ERROR;
}

void MXC_QDEC_RevA_EnableInt(mxc_qdec_reva_regs_t *qdec, uint32_t flags)
{
    int save_state = qdec->ctrl;

    // Disable QDEC to configure
    qdec->ctrl &= ~MXC_F_QDEC_REVA_CTRL_EN;

    // Clear flag before enabling interrupt
    qdec->intfl |= (flags & QDEC_IF_MASK);
    qdec->inten |= (flags & QDEC_IE_MASK);

    qdec->ctrl = save_state;
}

void MXC_QDEC_RevA_DisableInt(mxc_qdec_reva_regs_t *qdec, uint32_t flags)
{
    int save_state = qdec->ctrl;

    // Disable QDEC to configure
    qdec->ctrl &= ~MXC_F_QDEC_REVA_CTRL_EN;

    qdec->inten &= ~(flags & QDEC_IE_MASK);

    qdec->ctrl = save_state;
}

int MXC_QDEC_RevA_GetFlags(mxc_qdec_reva_regs_t *qdec)
{
    return (qdec->intfl & QDEC_IF_MASK);
}

void MXC_QDEC_RevA_ClearFlags(mxc_qdec_reva_regs_t *qdec, uint32_t flags)
{
    // Write 1 to clear flags
    qdec->intfl |= (flags & QDEC_IF_MASK);
}

void MXC_QDEC_RevA_SetMaxCount(mxc_qdec_reva_regs_t *qdec, uint32_t maxCount)
{
    int save_state = qdec->ctrl;

    // Disable QDEC to configure
    qdec->ctrl &= ~MXC_F_QDEC_REVA_CTRL_EN;

    qdec->maxcnt = maxCount;

    qdec->ctrl = save_state;
}

int MXC_QDEC_RevA_GetMaxCount(mxc_qdec_reva_regs_t *qdec)
{
    return qdec->maxcnt;
}

void MXC_QDEC_RevA_SetInitial(mxc_qdec_reva_regs_t *qdec, uint32_t initial)
{
    int save_state = qdec->ctrl;

    // Disable QDEC to configure
    qdec->ctrl &= ~MXC_F_QDEC_REVA_CTRL_EN;

    qdec->initial = initial;

    qdec->ctrl = save_state;
}

int MXC_QDEC_RevA_GetInitial(mxc_qdec_reva_regs_t *qdec)
{
    return qdec->initial;
}

void MXC_QDEC_RevA_SetCompare(mxc_qdec_reva_regs_t *qdec, uint32_t compare)
{
    int save_state = qdec->ctrl;

    // Disable QDEC to configure
    qdec->ctrl &= ~MXC_F_QDEC_REVA_CTRL_EN;

    qdec->compare = compare;

    qdec->ctrl = save_state;
}

int MXC_QDEC_RevA_GetCompare(mxc_qdec_reva_regs_t *qdec)
{
    return qdec->compare;
}

int MXC_QDEC_RevA_GetIndex(mxc_qdec_reva_regs_t *qdec)
{
    return qdec->index;
}

int MXC_QDEC_RevA_GetCapture(mxc_qdec_reva_regs_t *qdec)
{
    return qdec->capture;
}

int MXC_QDEC_RevA_Handler(mxc_qdec_reva_regs_t *qdec)
{
    uint32_t flags;

    // Clear Flags
    flags = MXC_QDEC_GetFlags();
    qdec->intfl = flags;

    if (async_callback != NULL) {
        async_callback(NULL, E_NO_ERROR);
    }

    return E_NO_ERROR;
}

// ************************************* Function to Read QDEC Data *******************************************
int MXC_QDEC_RevA_GetPosition(mxc_qdec_reva_regs_t *qdec)
{
    return qdec->position;
}

int MXC_QDEC_RevA_GetDirection(mxc_qdec_reva_regs_t *qdec)
{
    return (qdec->status & MXC_F_QDEC_REVA_STATUS_DIR);
}
