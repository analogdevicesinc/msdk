/**
 * @file
 * @brief   Trust Protection Unit driver.
 */

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

#include "mxc_device.h"
#include "mxc_errors.h"
#include "mxc_assert.h"
#include "mxc_sys.h"
#include "aes_revb.h"
#include "trng_revb.h"

/* ************************************************************************* */
/* Global Control/Configuration functions                                    */
/* ************************************************************************* */

int MXC_AES_Init(void)
{
#ifndef MSDK_NO_GPIO_CLK_INIT
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_AES);
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_TRNG);
#endif

    return MXC_AES_RevB_Init((mxc_aes_revb_regs_t *)MXC_AES, MXC_DMA);
}

void MXC_AES_EnableInt(uint32_t interrupt)
{
    MXC_AES_RevB_EnableInt((mxc_aes_revb_regs_t *)MXC_AES, interrupt);
}

void MXC_AES_DisableInt(uint32_t interrupt)
{
    MXC_AES_RevB_DisableInt((mxc_aes_revb_regs_t *)MXC_AES, interrupt);
}

int MXC_AES_IsBusy(void)
{
    return MXC_AES_RevB_IsBusy((mxc_aes_revb_regs_t *)MXC_AES);
}

int MXC_AES_Shutdown(void)
{
    int error = MXC_AES_RevB_Shutdown((mxc_aes_revb_regs_t *)MXC_AES);

    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_AES);

    return error;
}

void MXC_AES_DMACallback(int ch, int error)
{
    MXC_AES_RevB_DMACallback(ch, error);
}

void MXC_AES_GenerateKey(void)
{
    // Generating a random key is part of the TRNG block.
    MXC_TRNG_RevB_GenerateKey((mxc_trng_revb_regs_t *)MXC_TRNG);
}

void MXC_AES_SetKeySize(mxc_aes_keys_t key)
{
    MXC_AES_RevB_SetKeySize((mxc_aes_revb_regs_t *)MXC_AES, (mxc_aes_revb_keys_t)key);
}

mxc_aes_keys_t MXC_AES_GetKeySize(void)
{
    return MXC_AES_RevB_GetKeySize((mxc_aes_revb_regs_t *)MXC_AES);
}

void MXC_AES_FlushInputFIFO(void)
{
    MXC_AES_RevB_FlushInputFIFO((mxc_aes_revb_regs_t *)MXC_AES);
}

void MXC_AES_FlushOutputFIFO(void)
{
    MXC_AES_RevB_FlushOutputFIFO((mxc_aes_revb_regs_t *)MXC_AES);
}

void MXC_AES_Start(void)
{
    MXC_AES_RevB_Start((mxc_aes_revb_regs_t *)MXC_AES);
}

uint32_t MXC_AES_GetFlags(void)
{
    return MXC_AES_RevB_GetFlags((mxc_aes_revb_regs_t *)MXC_AES);
}

void MXC_AES_ClearFlags(uint32_t flags)
{
    MXC_AES_RevB_ClearFlags((mxc_aes_revb_regs_t *)MXC_AES, flags);
}

int MXC_AES_Generic(mxc_aes_req_t *req)
{
    return MXC_AES_RevB_Generic((mxc_aes_revb_regs_t *)MXC_AES, (mxc_aes_revb_req_t *)req);
}

int MXC_AES_Encrypt(mxc_aes_req_t *req)
{
    return MXC_AES_RevB_Encrypt((mxc_aes_revb_regs_t *)MXC_AES, (mxc_aes_revb_req_t *)req);
}

int MXC_AES_Decrypt(mxc_aes_req_t *req)
{
    return MXC_AES_RevB_Decrypt((mxc_aes_revb_regs_t *)MXC_AES, (mxc_aes_revb_req_t *)req);
}

int MXC_AES_TXDMAConfig(void *src_addr, int len)
{
    return MXC_AES_RevB_TXDMAConfig(src_addr, len, MXC_DMA);
}

int MXC_AES_RXDMAConfig(void *dest_addr, int len)
{
    return MXC_AES_RevB_RXDMAConfig(dest_addr, len, MXC_DMA);
}

int MXC_AES_GenericAsync(mxc_aes_req_t *req, uint8_t enc)
{
    return MXC_AES_RevB_GenericAsync((mxc_aes_revb_regs_t *)MXC_AES, (mxc_aes_revb_req_t *)req,
                                     enc);
}

int MXC_AES_EncryptAsync(mxc_aes_req_t *req)
{
    return MXC_AES_RevB_EncryptAsync((mxc_aes_revb_regs_t *)MXC_AES, (mxc_aes_revb_req_t *)req);
}

int MXC_AES_DecryptAsync(mxc_aes_req_t *req)
{
    return MXC_AES_RevB_DecryptAsync((mxc_aes_revb_regs_t *)MXC_AES, (mxc_aes_revb_req_t *)req);
}

void MXC_AES_SetExtKey(const void *key, mxc_aes_keys_t len)
{
    return MXC_AES_RevB_SetExtKey((mxc_aeskeys_revb_regs_t *)MXC_AESKEYS, key, len);
}
