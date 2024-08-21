/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. All Rights Reserved.
 * (now owned by Analog Devices, Inc.),
 * Copyright (C) 2023 Analog Devices, Inc. All Rights Reserved. This software
 * is proprietary to Analog Devices, Inc. and its licensors.
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

#include <stddef.h>
#include "board.h"
#include "mxc_device.h"
#include "mxc_assert.h"
#include "pb.h"

/******************************************************************************/
int PB_Init(void)
{
    int retval = E_NO_ERROR;
    unsigned int i;

    // Enable pushbutton inputs
    for (i = 0; i < num_pbs; i++) {
        if (MXC_GPIO_Config(&pb_pin[i]) != E_NO_ERROR) {
            retval = E_UNKNOWN;
        }
    }

    return retval;
}

/******************************************************************************/
int PB_RegisterCallback(unsigned int pb, pb_callback callback)
{
    if (pb >= num_pbs) {
        return E_BAD_PARAM;
    }

    if (callback) {
        // Register callback
        MXC_GPIO_RegisterCallback(&pb_pin[pb], callback, (void *)pb);

        // Configure and enable interrupt
        MXC_GPIO_IntConfig(&pb_pin[pb], MXC_GPIO_INT_FALLING);
        MXC_GPIO_EnableInt(pb_pin[pb].port, pb_pin[pb].mask);
        NVIC_EnableIRQ(MXC_GPIO_GET_IRQ(MXC_GPIO_GET_IDX(pb_pin[pb].port)));
    } else {
        // Disable interrupt and clear callback
        MXC_GPIO_DisableInt(pb_pin[pb].port, pb_pin[pb].mask);
        MXC_GPIO_RegisterCallback(&pb_pin[pb], NULL, NULL);
    }

    return E_NO_ERROR;
}

/******************************************************************************/
int PB_RegisterCallbackRiseFall(unsigned int pb, pb_callback callback)
{
    if (pb >= num_pbs) {
        return E_BAD_PARAM;
    }

    if (callback) {
        // Register callback
        MXC_GPIO_RegisterCallback(&pb_pin[pb], callback, (void *)&pb_pin[pb]);

        // Configure and enable interrupt
        MXC_GPIO_IntConfig(&pb_pin[pb], MXC_GPIO_INT_BOTH);
        MXC_GPIO_EnableInt(pb_pin[pb].port, pb_pin[pb].mask);
        NVIC_EnableIRQ(MXC_GPIO_GET_IRQ(MXC_GPIO_GET_IDX(pb_pin[pb].port)));
    } else {
        // Disable interrupt and clear callback
        MXC_GPIO_DisableInt(pb_pin[pb].port, pb_pin[pb].mask);
        MXC_GPIO_RegisterCallback(&pb_pin[pb], NULL, NULL);
    }

    return E_NO_ERROR;
}

//******************************************************************************
void PB_IntEnable(unsigned int pb)
{
    MXC_ASSERT(pb < num_pbs);
    MXC_GPIO_EnableInt(pb_pin[pb].port, pb_pin[pb].mask);
}

//******************************************************************************
void PB_IntDisable(unsigned int pb)
{
    MXC_ASSERT(pb < num_pbs);
    MXC_GPIO_DisableInt(pb_pin[pb].port, pb_pin[pb].mask);
}

//******************************************************************************
void PB_IntClear(unsigned int pb)
{
    MXC_ASSERT(pb < num_pbs);
    MXC_GPIO_ClearFlags(pb_pin[pb].port, pb_pin[pb].mask);
}

//******************************************************************************
int PB_Get(unsigned int pb)
{
    MXC_ASSERT(pb < num_pbs);
    return !MXC_GPIO_InGet(pb_pin[pb].port, pb_pin[pb].mask);
}

//******************************************************************************
int PB_IsPressedAny(void)
{
    int i = 0;

    for (i = 0; i < num_pbs; i++) {
        if (PB_Get(i)) {
            return 1;
        }
    }

    return 0;
}
