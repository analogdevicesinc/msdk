/**
 * @file    icc.h
 * @brief   Instruction Controller Cache(ICC) function prototypes and data types.
 */

/******************************************************************************
 * Copyright (C) 2023 Maxim Integrated Products, Inc., All Rights Reserved.
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
 ******************************************************************************/

/* Define to prevent redundant inclusion */
#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32650_ICC_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32650_ICC_H_

/* **** Includes **** */
#include <stdint.h>
#include "icc_regs.h"
#include "max32650.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup icc Internal Cache Controller (ICC)
 * @ingroup periphlibs
 * @{
 */

/**
 * @brief Enumeration type for the Cache ID Register
 */
typedef enum {
    ICC_INFO_RELNUM, ///< Identifies the RTL release version
    ICC_INFO_PARTNUM, ///< Specifies the value of C_ID Port Number
    ICC_INFO_ID ///< Specifies the value of Cache ID
} mxc_icc_info_t;

/**
 * @brief   Reads ID information from the MXC_ICC0 Cache ID Register.
 * @param   cid     Selects what information to get from the MXC_ICC0 Cache ID Register
 * @retval  Returns the selected value from the MXC_ICC0 Cache ID Register.
 */
int MXC_ICC_ID(mxc_icc_info_t cid);

/**
 * @brief	Enables both of the instruction cache controllers.
 */
void MXC_ICC_Enable(void);

/**
 * @brief	Disables both of the instruction cache controllers.
 */
void MXC_ICC_Disable(void);

/**
 * @brief	Flushes both of the instruction cache controllers.
 */
void MXC_ICC_Flush(void);

/**
 * @brief   Reads ID information from one of the ICC's Cache ID Register.
 * @param   icc     Pointer ICC instance to get ID information from.
 * @param   cid     Selects what information to get from the Cache ID Register
 * @retval  Returns the selected value from the Cache ID Register.
 */
int MXC_ICC_IDInst(mxc_icc_regs_t *icc, mxc_icc_info_t cid);

/**
 * @brief   Enables one of the ICC's.
 * @param   icc     Pointer to ICC instance to enable.
 */
void MXC_ICC_EnableInst(mxc_icc_regs_t *icc);

/**
 * @brief   Disables one of the ICC's.
 * @param   icc     Pointer to ICC instance to disable.
 */
void MXC_ICC_DisableInst(mxc_icc_regs_t *icc);

/**
 * @brief   Flushes data from one of the ICC's.
 * @param   icc     Pointer to ICC instance to flush.
 */
void MXC_ICC_FlushInst(mxc_icc_regs_t *icc);

/**@} end of group icc */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32650_ICC_H_
