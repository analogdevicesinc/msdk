/* *****************************************************************************
 * Copyright (C) 2017 Maxim Integrated Products, Inc., All Rights Reserved.
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
 * $Date: 2018-12-18 15:37:22 -0600 (Tue, 18 Dec 2018) $
 * $Revision: 40072 $
 *
 **************************************************************************** */

/* **** Includes **** */
#include <stdint.h>
#include <stdio.h>
#include "mxc_errors.h"
#include "icc_regs.h"
#include "icc.h"
#include "icc_reva.h"
#include "icc_common.h"

/* **************************************************************************** */
int MXC_ICC_ID(mxc_icc_info_t cid)
{
    int err = 0;

    err += MXC_ICC_RevA_ID((mxc_icc_reva_regs_t *)MXC_ICC0, cid);
    err += MXC_ICC_RevA_ID((mxc_icc_reva_regs_t *)MXC_ICC1, cid);

    return err;
}

/* **************************************************************************** */
void MXC_ICC_Enable(void)
{
    MXC_ICC_RevA_Enable((mxc_icc_reva_regs_t *)MXC_ICC0);
    MXC_ICC_RevA_Enable((mxc_icc_reva_regs_t *)MXC_ICC1);
}

/* **************************************************************************** */
void MXC_ICC_Disable(void)
{
    MXC_ICC_RevA_Disable((mxc_icc_reva_regs_t *)MXC_ICC0);
    MXC_ICC_RevA_Enable((mxc_icc_reva_regs_t *)MXC_ICC1);
}

/* **************************************************************************** */
void MXC_ICC_Flush(void)
{
    MXC_ICC_Com_Flush();
}

/* **************************************************************************** */
int MXC_ICC_IDInst(mxc_icc_regs_t* icc, mxc_icc_info_t cid)
{
    return MXC_ICC_RevA_ID((mxc_icc_reva_regs_t *)icc, cid);
}

/* **************************************************************************** */
void MXC_ICC_EnableInst(mxc_icc_regs_t* icc)
{
    MXC_ICC_RevA_Enable((mxc_icc_reva_regs_t *)icc);
}

/* **************************************************************************** */
void MXC_ICC_DisableInst(mxc_icc_regs_t* icc)
{
    MXC_ICC_RevA_Disable((mxc_icc_reva_regs_t *)icc);
}

/* **************************************************************************** */
void MXC_ICC_FlushInst(mxc_icc_regs_t* icc)
{
    MXC_ICC_DisableInst(icc);
    MXC_ICC_EnableInst(icc);
}