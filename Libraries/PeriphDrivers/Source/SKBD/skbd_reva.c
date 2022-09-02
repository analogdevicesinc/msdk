/* *****************************************************************************
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
 **************************************************************************** */

#include <stddef.h>
#include <string.h>
#include "mxc_assert.h"
#include "mxc_pins.h"
#include "mxc_lock.h"
#include "mxc_delay.h"
#include "mxc_device.h"
#include "mxc_errors.h"
#include "nvic_table.h"
#include "skbd.h"
#include "skbd_reva.h"

/* ***** SKBD context info ***** */
static mxc_skbd_reva_req_t mxc_skbd_req;

static void SKBD_RevA_IRQHandler(void)
{
    if (mxc_skbd_req.irq_handler) {
        mxc_skbd_req.irq_handler();
    }

    /* Acknowledge interrupt at platform level */
    NVIC_ClearPendingIRQ(SKB_IRQn);
}

int MXC_SKBD_RevA_PreInit(void)
{
    mxc_skbd_req.first_init = 0;

    NVIC_ClearPendingIRQ(SKB_IRQn);
    /* Attach vector */
    MXC_NVIC_SetVector(SKB_IRQn, SKBD_RevA_IRQHandler);

    return E_NO_ERROR;
}

int MXC_SKBD_RevA_Init(mxc_skbd_reva_regs_t* skbd, mxc_skbd_config_t config)
{
    int result = E_NO_ERROR;

    if (!mxc_skbd_req.first_init) {
        unsigned int temp;
        /* Number of output pins */
        unsigned int outputs = 0;

        /* Check for the I/O pin overlaps */
        if (config.outputs & config.inputs) {
            return MXC_SKBD_REVA_ERR_INVALID_PIN_CONFIGURATION;
        }

        /* I/O pin direction selection for the SKBD pins */
        /* Configure SKBD output pins */
        skbd->ctrl0 |= config.outputs;
        /* Configure SKBD input pins */
        skbd->ctrl0 &= ~(config.inputs);
        /* Memset like procedure */
        memset((unsigned char*)&mxc_skbd_req, 0x00, sizeof(mxc_skbd_req));
        /* Count the number of output SKBD lines */
        temp = config.outputs;

        while (temp) {
            temp &= (temp - 1);
            outputs++;
        }

        /* Configure the SKBD  */
        temp =
            ((config.reg_erase << MXC_F_SKBD_REVA_CTRL1_CLEAR_POS) | MXC_F_SKBD_REVA_CTRL1_AUTOEN);
        temp |= (config.debounce << MXC_F_SKBD_REVA_CTRL1_DBTM_POS) & MXC_F_SKBD_REVA_CTRL1_DBTM;
        temp |= (outputs << MXC_F_SKBD_REVA_CTRL1_OUTNB_POS) & MXC_F_SKBD_REVA_CTRL1_OUTNB;
        skbd->ctrl1 |= temp;

        while (!(skbd->sr & MXC_F_SKBD_REVA_SR_BUSY))
            ;

        /* Setup IRQ */
        if (config.irq_handler) {
            mxc_skbd_req.irq_handler = config.irq_handler;
        }

        /* To be done once only */
        mxc_skbd_req.first_init = 1;
        mxc_skbd_req.state      = MXC_SKBD_REVA_STATE_INITIALIZED;
    } else {
        result = MXC_SKBD_REVA_ERR_ALREAD_INITIALIZED;
    }

    return result;
}

int MXC_SKBD_RevA_EnableInterruptEvents(mxc_skbd_reva_regs_t* skbd, unsigned int events)
{
    int result = E_NO_ERROR;

    if (MXC_SKBD_REVA_STATE_INITIALIZED != mxc_skbd_req.state) {
        result = MXC_SKBD_REVA_ERR_NOT_INITIALIZED;
    } else {
        events &=
            (MXC_SKBD_REVA_EVENT_PUSH | MXC_SKBD_REVA_EVENT_RELEASE | MXC_SKBD_REVA_EVENT_OVERRUN);
        skbd->ier |= events;
    }

    return result;
}

extern inline int MXC_SKBD_RevA_DisableInterruptEvents(mxc_skbd_reva_regs_t* skbd,
                                                       unsigned int events)
{
    events &=
        (MXC_SKBD_REVA_EVENT_PUSH | MXC_SKBD_REVA_EVENT_RELEASE | MXC_SKBD_REVA_EVENT_OVERRUN);
    skbd->ier &= ~events;
    return E_NO_ERROR;
}

extern inline int MXC_SKBD_RevA_ClearInterruptStatus(mxc_skbd_reva_regs_t* skbd,
                                                     unsigned int status)
{
    status &= (MXC_SKBD_REVA_INTERRUPT_STATUS_PUSHIS | MXC_SKBD_REVA_INTERRUPT_STATUS_RELEASEIS |
               MXC_SKBD_REVA_INTERRUPT_STATUS_OVERIS);
    skbd->isr &= ~status;
    return E_NO_ERROR;
}

extern inline int MXC_SKBD_RevA_InterruptStatus(mxc_skbd_reva_regs_t* skbd, unsigned int* status)
{
    if (status == NULL) {
        return E_NULL_PTR;
    }

    *status = skbd->isr;
    return E_NO_ERROR;
}

int MXC_SKBD_RevA_ReadKeys(mxc_skbd_reva_regs_t* skbd, mxc_skbd_reva_keys_t* keys)
{
    volatile unsigned short* key;
    volatile unsigned int i = 0;
    volatile unsigned int temp;
    volatile unsigned int* key_reg;

    if (keys == NULL) {
        return E_NULL_PTR;
    }

    key     = (unsigned short*)&keys->key0_reva;
    key_reg = (unsigned int*)&skbd->evt[0];

    for (i = 0; i < MXC_SKBD_REVA_TOTAL_KEY_REGS; i++) {
        if (!(skbd->ctrl1 & MXC_F_SKBD_REVA_CTRL1_CLEAR) &&
            (skbd->ier & MXC_F_SKBD_REVA_IER_PUSHIE)) {
            if (!(*key_reg & (MXC_F_SKBD_REVA_EVT_PUSH | MXC_F_SKBD_REVA_EVT_READ))) {
                *key++ = ((*key_reg & MXC_F_SKBD_REVA_EVT_IOIN) |
                          ((*key_reg & MXC_F_SKBD_REVA_EVT_IOOUT) >> 1));
            }
        } else if (!(skbd->ctrl1 & MXC_F_SKBD_REVA_CTRL1_CLEAR) &&
                   (skbd->ier & MXC_F_SKBD_REVA_IER_RELEASEIE)) {
            temp = *key_reg;

            if ((temp & MXC_F_SKBD_REVA_EVT_PUSH) && !(temp & MXC_F_SKBD_REVA_EVT_READ)) {
                *key++ = ((*key_reg & MXC_F_SKBD_REVA_EVT_IOIN) |
                          ((*key_reg & MXC_F_SKBD_REVA_EVT_IOOUT) >> 1));
            }
        } else {
            temp = *key_reg;

            if (!(temp & MXC_F_SKBD_REVA_EVT_READ)) {
                *key++ =
                    ((temp & MXC_F_SKBD_REVA_EVT_IOIN) | ((temp & MXC_F_SKBD_REVA_EVT_IOOUT) >> 1));
            }
        }

        key_reg++;
    }

    return E_NO_ERROR;
}

int MXC_SKBD_RevA_Close(void)
{
    NVIC_DisableIRQ(SKB_IRQn);
    mxc_skbd_req.state      = MXC_SKBD_REVA_STATE_CLOSED;
    mxc_skbd_req.first_init = 0;
    return E_NO_ERROR;
}