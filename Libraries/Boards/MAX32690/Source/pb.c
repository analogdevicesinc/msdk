/*******************************************************************************
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
 * $Date: 2017-02-28 16:31:00 -0600 (Tue, 28 Feb 2017) $
 * $Revision: 26756 $
 *
 ******************************************************************************/

#include <stddef.h>
#include "mxc_device.h"
#include "mxc_assert.h"
#include "pb.h"
#include "pwrseq_regs.h"
#include "mcr_regs.h"

/* Save the callback state for GPIO4 */
static pb_callback gpio4_callback;
static int gpio4_pb;

/******************************************************************************/
int PB_Init(void)
{
    int retval = E_NO_ERROR;
    unsigned int i;

    // Enable pushbutton inputs
    for (i = 0; i < num_pbs; i++) {
        if (MXC_GPIO_GET_IDX(pb_pin[i].port) == 4) {
            MXC_MCR->gpio4_ctrl &= ~(MXC_F_MCR_GPIO4_CTRL_P40_OE | MXC_F_MCR_GPIO4_CTRL_P40_PE);
            MXC_MCR->gpio4_ctrl |= MXC_F_MCR_GPIO4_CTRL_P40_DO;
        } else if (MXC_GPIO_Config(&pb_pin[i]) != E_NO_ERROR) {
            retval = E_UNKNOWN;
        }
    }

    /* Initialize the GPIO4 callback state */
    gpio4_callback = NULL;
    gpio4_pb = -1;

    return retval;
}

/******************************************************************************/
int PB_RegisterCallback(unsigned int pb, pb_callback callback)
{
    MXC_ASSERT(pb < num_pbs);

    if (callback) {
        if (MXC_GPIO_GET_IDX(pb_pin[pb].port) == 4) {
            /* Save the GPIO4 callback and pb index */
            gpio4_callback = callback;
            gpio4_pb = pb;

            MXC_PWRSEQ->lpwken4 |= pb_pin[pb].mask;
            NVIC_EnableIRQ(GPIOWAKE_IRQn);

            return E_NO_ERROR;
        }

        // Register callback
        MXC_GPIO_RegisterCallback(&pb_pin[pb], callback, (void *)pb);

        // Configure and enable interrupt
        MXC_GPIO_IntConfig(&pb_pin[pb], MXC_GPIO_INT_FALLING);
        MXC_GPIO_EnableInt(pb_pin[pb].port, pb_pin[pb].mask);
        NVIC_EnableIRQ(MXC_GPIO_GET_IRQ(MXC_GPIO_GET_IDX(pb_pin[pb].port)));
    } else {
        if (MXC_GPIO_GET_IDX(pb_pin[pb].port) == 4) {
            /* Clear the GPIO4 callback and pb index */
            gpio4_callback = NULL;
            gpio4_pb = -1;

            NVIC_DisableIRQ(GPIOWAKE_IRQn);
            MXC_PWRSEQ->lpwken4 &= ~pb_pin[pb].mask;

            return E_NO_ERROR;
        }
        // Disable interrupt and clear callback
        MXC_GPIO_DisableInt(pb_pin[pb].port, pb_pin[pb].mask);
        MXC_GPIO_RegisterCallback(&pb_pin[pb], NULL, NULL);
    }

    return E_NO_ERROR;
}

/******************************************************************************/
int PB_RegisterCallbackRiseFall(unsigned int pb, pb_callback callback)
{
    MXC_ASSERT(pb < num_pbs);

    if (callback) {
        if (MXC_GPIO_GET_IDX(pb_pin[pb].port) == 4) {
            /* Save the GPIO4 callback and pb index */
            gpio4_callback = callback;
            gpio4_pb = pb;

            MXC_PWRSEQ->lpwken4 |= pb_pin[pb].mask;
            NVIC_EnableIRQ(GPIOWAKE_IRQn);

            return E_NO_ERROR;
        }
        // Register callback
        MXC_GPIO_RegisterCallback(&pb_pin[pb], callback, (void *)pb);

        // Configure and enable interrupt
        MXC_GPIO_IntConfig(&pb_pin[pb], MXC_GPIO_INT_BOTH);
        MXC_GPIO_EnableInt(pb_pin[pb].port, pb_pin[pb].mask);
        NVIC_EnableIRQ(MXC_GPIO_GET_IRQ(MXC_GPIO_GET_IDX(pb_pin[pb].port)));
    } else {
        if (MXC_GPIO_GET_IDX(pb_pin[pb].port) == 4) {
            /* Clear the GPIO4 callback and pb index */
            gpio4_callback = NULL;
            gpio4_pb = -1;

            NVIC_DisableIRQ(GPIOWAKE_IRQn);
            MXC_PWRSEQ->lpwken4 &= ~pb_pin[pb].mask;

            return E_NO_ERROR;
        }

        // Disable interrupt and clear callback
        MXC_GPIO_DisableInt(pb_pin[pb].port, pb_pin[pb].mask);
        MXC_GPIO_RegisterCallback(&pb_pin[pb], NULL, NULL);
    }

    return E_NO_ERROR;
}

//******************************************************************************
void GPIO0_IRQHandler(void)
{
    MXC_GPIO_Handler(MXC_GPIO_GET_IDX(MXC_GPIO0));
}

//******************************************************************************
void GPIO1_IRQHandler(void)
{
    MXC_GPIO_Handler(MXC_GPIO_GET_IDX(MXC_GPIO1));
}

//******************************************************************************
void GPIO2_IRQHandler(void)
{
    MXC_GPIO_Handler(MXC_GPIO_GET_IDX(MXC_GPIO2));
}

//******************************************************************************
void GPIO3_IRQHandler(void)
{
    MXC_GPIO_Handler(MXC_GPIO_GET_IDX(MXC_GPIO3));
}

//******************************************************************************
void GPIOWAKE_IRQHandler(void)
{
    /* Clear the interrupt status */
    MXC_PWRSEQ->lpwkst4 = pb_pin[gpio4_pb].mask;
    NVIC_ClearPendingIRQ(GPIOWAKE_IRQn);

    /* Call the saved callback if available */
    if (gpio4_callback != NULL) {
        gpio4_callback((void *)&pb_pin[gpio4_pb]);
    }
}

//******************************************************************************
void PB_IntEnable(unsigned int pb)
{
    MXC_ASSERT(pb < num_pbs);
    if (MXC_GPIO_GET_IDX(pb_pin[pb].port) == 4) {
        MXC_PWRSEQ->lpwkst4 = pb_pin[pb].mask;
        MXC_PWRSEQ->lpwken4 |= pb_pin[pb].mask;
        NVIC_EnableIRQ(GPIOWAKE_IRQn);
    } else {
        MXC_GPIO_EnableInt(pb_pin[pb].port, pb_pin[pb].mask);
    }
}

//******************************************************************************
void PB_IntDisable(unsigned int pb)
{
    MXC_ASSERT(pb < num_pbs);
    if (MXC_GPIO_GET_IDX(pb_pin[pb].port) == 4) {
        MXC_PWRSEQ->lpwken4 &= ~pb_pin[pb].mask;
        NVIC_DisableIRQ(GPIOWAKE_IRQn);
    } else {
        MXC_GPIO_DisableInt(pb_pin[pb].port, pb_pin[pb].mask);
    }
}

//******************************************************************************
void PB_IntClear(unsigned int pb)
{
    MXC_ASSERT(pb < num_pbs);
    if (MXC_GPIO_GET_IDX(pb_pin[pb].port) == 4) {
        MXC_PWRSEQ->lpwkst4 = pb_pin[pb].mask;
    } else {
        MXC_GPIO_ClearFlags(pb_pin[pb].port, pb_pin[pb].mask);
    }
}

//******************************************************************************
int PB_Get(unsigned int pb)
{
    MXC_ASSERT(pb < num_pbs);
    if (MXC_GPIO_GET_IDX(pb_pin[pb].port) == 4) {
        return !(MXC_MCR->gpio4_ctrl & MXC_F_MCR_GPIO4_CTRL_P40_IN);
    }

    return !MXC_GPIO_InGet(pb_pin[pb].port, pb_pin[pb].mask);
}
