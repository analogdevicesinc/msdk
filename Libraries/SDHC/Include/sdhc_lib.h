/**
 * @file
 * @brief      This file contains the function definitions for the
 *             Secure Digital High Capacity (SDHC) peripheral module.
 */

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
 * $Date: 2017-03-01 09:46:57 -0600 (Wed, 01 Mar 2017) $
 * $Revision: 26777 $
 *
 **************************************************************************** */

#ifndef LIBRARIES_SDHC_INCLUDE_SDHC_LIB_H_
#define LIBRARIES_SDHC_INCLUDE_SDHC_LIB_H_

/* **** Includes **** */
#include <string.h>
#include <stdio.h>
#include "mxc_device.h"
#include "mxc_assert.h"
#include "mxc_sys.h"
#include "sdhc.h"
#include "sdhc_resp_regs.h"

/**
 * @ingroup sdhc
 * @{
 */

/* **** Definitions **** */
typedef enum {
    MXC_SDHC_LIB_SINGLE_DATA = 1,
    MXC_SDHC_LIB_QUAD_DATA,
} mxc_sdhc_data_width;

typedef enum {
    MXC_SDHC_LIB_IDLE_STATE = 0,
    MXC_SDHC_LIB_READY_STATE = 1,
    MXC_SDHC_LIB_IDENT_STATE = 2,
    MXC_SDHC_LIB_STBY_STATE = 3,
    MXC_SDHC_LIB_TRAN_STATE = 4,
    MXC_SDHC_LIB_DATA_STATE = 5,
    MXC_SDHC_LIB_RCV_STATE = 6,
    MXC_SDHC_LIB_PRG_STATE = 7,
    MXC_SDHC_LIB_DIS_STATE = 8,
} mxc_sdhc_state;

typedef enum { CARD_NONE = 0, CARD_SDHC, CARD_MMC } mxc_sdhc_lib_card_type;

mxc_sdhc_lib_card_type MXC_SDHC_Lib_Get_Card_Type(void);

/* ************************************************************************** */
int MXC_SDHC_Lib_SetRCA(void);

/* ************************************************************************** */
int MXC_SDHC_Lib_GetCSD(mxc_sdhc_csd_regs_t *csd);

/* ************************************************************************** */
unsigned int MXC_SDHC_Lib_GetCapacity(mxc_sdhc_csd_regs_t *csd);

/* ************************************************************************** */
unsigned int MXC_SDHC_Lib_GetSectors(mxc_sdhc_csd_regs_t *csd);

/* ************************************************************************** */
int MXC_SDHC_Lib_GetBlockSize(mxc_sdhc_csd_regs_t *csd);

/* ************************************************************************** */
int MXC_SDHC_Lib_GetCurrentState(mxc_sdhc_state *state);

/* ************************************************************************** */
int MXC_SDHC_Lib_SetDsr(void);

/* ************************************************************************** */
int MXC_SDHC_Lib_SetBusWidth(mxc_sdhc_data_width bus_width);

/* ************************************************************************** */
int MXC_SDHC_Lib_InitCard(int retries);

/* ************************************************************************** */
void MXC_SDHC_Lib_Async_Handler(void);

/* ************************************************************************** */
int MXC_SDHC_Lib_Prepare_Trans(mxc_sdhc_data_width width);

/* ************************************************************************** */
int MXC_SDHC_Lib_Write(unsigned int dst_addr, void *src_addr, unsigned int cnt,
                       mxc_sdhc_data_width width);

/* ************************************************************************** */
int MXC_SDHC_Lib_Read(void *dst_addr, unsigned int src_addr, unsigned int cnt,
                      mxc_sdhc_data_width width);

/* ************************************************************************** */
int MXC_SDHC_Lib_WriteAsync(unsigned int dst_addr, void *src_addr, unsigned int cnt,
                            mxc_sdhc_data_width width, mxc_sdhc_callback_fn callback);

/* ************************************************************************** */
int MXC_SDHC_Lib_ReadAsync(void *dst_addr, unsigned int src_addr, unsigned int cnt,
                           mxc_sdhc_data_width width, mxc_sdhc_callback_fn callback);

/**@} end of group sdhc */

#endif // LIBRARIES_SDHC_INCLUDE_SDHC_LIB_H_
