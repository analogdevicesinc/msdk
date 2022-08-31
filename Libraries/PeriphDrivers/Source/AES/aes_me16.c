/**
 * @file
 * @brief   Trust Protection Unit driver.
 */

/* ****************************************************************************
 * Copyright (C) 2018 Maxim Integrated Products, Inc., All Rights Reserved.
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

#include "aes_revb.h"
#include "mxc_assert.h"
#include "mxc_device.h"
#include "mxc_errors.h"
#include "mxc_sys.h"

/* ************************************************************************* */
/* Global Control/Configuration functions                                    */
/* ************************************************************************* */

int MXC_AES_Init(void)
{
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_AES);
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_TRNG);

    MXC_AES_RevB_Init();

    return E_NO_ERROR;
}

void MXC_AES_EnableInt(uint32_t interrupt)
{
    MXC_AES_RevB_EnableInt(interrupt);
}

void MXC_AES_DisableInt(uint32_t interrupt)
{
    MXC_AES_RevB_DisableInt(interrupt);
}

int MXC_AES_IsBusy(void)
{
    return MXC_AES_RevB_IsBusy();
}

int MXC_AES_Shutdown(void)
{
    int error = MXC_AES_RevB_Shutdown();

    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_AES);

    return error;
}

void MXC_AES_DMACallback(int ch, int error)
{
    MXC_AES_RevB_DMACallback(ch, error);
}

void MXC_AES_GenerateKey(void)
{
    MXC_AES_RevB_GenerateKey();
}

void MXC_AES_SetKeySize(mxc_aes_keys_t key)
{
    MXC_AES_RevB_SetKeySize(key);
}

mxc_aes_keys_t MXC_AES_GetKeySize(void)
{
    return MXC_AES_RevB_GetKeySize();
}

void MXC_AES_FlushInputFIFO(void)
{
    MXC_AES_RevB_FlushInputFIFO();
}

void MXC_AES_FlushOutputFIFO(void)
{
    MXC_AES_RevB_FlushOutputFIFO();
}

void MXC_AES_Start(void)
{
    MXC_AES_RevB_Start();
}

uint32_t MXC_AES_GetFlags(void)
{
    return MXC_AES_RevB_GetFlags();
}

void MXC_AES_ClearFlags(uint32_t flags)
{
    MXC_AES_RevB_ClearFlags(flags);
}

int MXC_AES_Generic(mxc_aes_req_t* req)
{
    return MXC_AES_RevB_Generic(req);
}

int MXC_AES_Encrypt(mxc_aes_req_t* req)
{
    return MXC_AES_RevB_Encrypt(req);
}

int MXC_AES_Decrypt(mxc_aes_req_t* req)
{
    return MXC_AES_RevB_Decrypt(req);
}

int MXC_AES_TXDMAConfig(void* src_addr, int len)
{
    return MXC_AES_RevB_TXDMAConfig(src_addr, len);
}

int MXC_AES_RXDMAConfig(void* dest_addr, int len)
{
    return MXC_AES_RevB_RXDMAConfig(dest_addr, len);
}

int MXC_AES_GenericAsync(mxc_aes_req_t* req, uint8_t enc)
{
    return MXC_AES_RevB_GenericAsync(req, enc);
}

int MXC_AES_EncryptAsync(mxc_aes_req_t* req)
{
    return MXC_AES_RevB_EncryptAsync(req);
}

int MXC_AES_DecryptAsync(mxc_aes_req_t* req)
{
    return MXC_AES_RevB_DecryptAsync(req);
}

void MXC_AES_SetExtKey(const void* key, mxc_aes_keys_t len)
{
    return MXC_AES_RevB_SetExtKey(key, len);
}
