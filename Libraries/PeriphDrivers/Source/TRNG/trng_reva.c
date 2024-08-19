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

#include <stdlib.h>
#include <string.h>

#include "mxc_sys.h"
#include "mxc_device.h"
#include "mxc_errors.h"
#include "mxc_assert.h"
#include "mxc_lock.h"

#include "trng_regs.h"
#include "trng.h"
#include "trng_reva.h"

/***** Global Variables *****/

static mxc_trng_complete_t MXC_TRNG_Callback;

static uint32_t TRNG_count, TRNG_maxLength;
static uint8_t *TRNG_data;

/* ************************************************************************* */
/* Global Control/Configuration functions                                    */
/* ************************************************************************* */

int MXC_TRNG_RevA_Init(void)
{
    MXC_TRNG_DisableInt();
    return E_NO_ERROR;
}

void MXC_TRNG_RevA_EnableInt(mxc_trng_reva_regs_t *trng)
{
    trng->ctrl |= MXC_F_TRNG_REVA_CTRL_RNG_IE;
}

void MXC_TRNG_RevA_DisableInt(mxc_trng_reva_regs_t *trng)
{
    trng->ctrl &= ~MXC_F_TRNG_REVA_CTRL_RNG_IE;
}

int MXC_TRNG_RevA_Shutdown(void)
{
    return E_NO_ERROR;
}

void MXC_TRNG_RevA_Handler(mxc_trng_reva_regs_t *trng)
{
    uint32_t temp, remaining, i;
    mxc_trng_complete_t cb;

    // if this is last block, disable interrupt before reading trng->data
    if (TRNG_maxLength <= TRNG_count + 4) {
        trng->ctrl &= ~MXC_F_TRNG_REVA_CTRL_RNG_IE;
    }

    trng->ctrl |= MXC_S_TRNG_REVA_CTRL_RNG_ISC_CLEAR;
    remaining = (TRNG_maxLength - TRNG_count) / 4;

    if (remaining) {
        for (i = 0; i < 4; i++) {
            if (i >= remaining)
                break;

            temp = trng->data;
            memcpy(&(TRNG_data[TRNG_count]), (uint8_t *)(&temp), 4);
            TRNG_count += 4;
        }
    } else {
        memcpy(&(TRNG_data[TRNG_count]), (uint8_t *)(&temp), TRNG_maxLength & 0x03);
        TRNG_count += (TRNG_maxLength & 0x03);
    }

    if (TRNG_maxLength == TRNG_count) {
        cb = MXC_TRNG_Callback;
        cb(0, 0);
    }
}

/* ************************************************************************* */
/* True Random Number Generator(TRNG) functions                              */
/* ************************************************************************* */

int MXC_TRNG_RevA_RandomInt(mxc_trng_reva_regs_t *trng)
{
    while (!(trng->ctrl & MXC_S_TRNG_REVA_CTRL_RNG_IS_READY)) {}

    return (int)trng->data;
}

int MXC_TRNG_RevA_Random(uint8_t *data, uint32_t len)
{
    unsigned int i, temp;

    if (data == NULL) {
        return E_NULL_PTR;
    }

    for (i = 0; (i + 3) < len; i += 4) {
        temp = MXC_TRNG_RandomInt();
        memcpy(&(data[i]), (uint8_t *)(&temp), 4);
    }

    if (len & 0x03) {
        temp = MXC_TRNG_RandomInt();
        memcpy(&(data[i]), (uint8_t *)(&temp), len & 0x03);
    }

    return E_NO_ERROR;
}

void MXC_TRNG_RevA_RandomAsync(mxc_trng_reva_regs_t *trng, uint8_t *data, uint32_t len,
                               mxc_trng_complete_t callback)
{
    MXC_ASSERT(data && callback);

    if (len == 0) {
        return;
    }

    TRNG_data = data;
    TRNG_count = 0;
    TRNG_maxLength = len;
    MXC_TRNG_Callback = callback;

    // Enable interrupts
    trng->ctrl |= MXC_F_TRNG_REVA_CTRL_RNG_IE;
}

void MXC_TRNG_RevA_GenerateKey(mxc_trng_reva_regs_t *trng)
{
    /*Generate AES Key */
    trng->ctrl |= MXC_F_TRNG_REVA_CTRL_AESKG;

    while (trng->ctrl & MXC_F_TRNG_REVA_CTRL_AESKG) {}
}
