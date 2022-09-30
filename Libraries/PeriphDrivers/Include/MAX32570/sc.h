/**
 * @file    sc.h
 * @brief   Smart Card (SC) interface functions and data types.
 */

/* ****************************************************************************
 * Copyright (C) Maxim Integrated Products, Inc., All Rights Reserved.
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

#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32570_SC_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32570_SC_H_

/* **** Includes **** */
#include "mxc_device.h"
#include "scn_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup scn Smart Card (SC)
 * @ingroup periphlibs
 * @{
 */

/* **** Definitions **** */
/**
 * Bitmasks for each smart card device
 */
typedef enum {
    MXC_SC_DEV_MIN = 0,
    MXC_SC_DEV0 = MXC_SC_DEV_MIN, /**< Smart Card 0 */
    MXC_SC_DEV1, /**< Smart Card 1 */
    MXC_SC_DEV_MAX = MXC_SC_DEV1,
    MXC_SC_DEV_COUNT /**< Number of Smart Card Devices */
} mxc_sc_id_t;

/* **** Structures **** */
/**
  * @brief      Structure for smart card register
  *
  */
typedef struct {
    mxc_scn_regs_t *reg_sc;
} mxc_sc_info_t;

/**
  * @brief      Structure to save smart card state
  *
  */
typedef struct {
    unsigned char first_init;
    mxc_sc_info_t sc[MXC_SC_DEV_COUNT];
} mxc_sc_context_t;

/* **** Function Prototypes **** */

/**
 * @brief       Inititalize Smart Card Interface
 *
 * @param       id Smart Card interface id
 * @return      Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int mxc_sc_init(mxc_sc_id_t id);

#ifdef __cplusplus
}
#endif

/**@} end of group sc */
#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32570_SC_H_
