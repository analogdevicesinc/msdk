/* *****************************************************************************
 * Copyright (C) Analog Devices, All rights Reserved.
 *
 * This software is protected by copyright laws of the United States and
 * of foreign countries. This material may also be protected by patent laws
 * and technology transfer regulations of the United States and of foreign
 * countries. This software is furnished under a license agreement and/or a
 * nondisclosure agreement and may only be used or reproduced in accordance
 * with the terms of those agreements. Dissemination of this information to
 * any party or parties not specified in the license agreement and/or
 * nondisclosure agreement is expressly prohibited.
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
 **************************************************************************** */

/**
 * @file    dbb_registers.h
 * @brief   Mapping the register pointers to the base address.
 */

#ifndef MAX32665_INCLUDE_DBB_REGISTERS_H_
#define MAX32665_INCLUDE_DBB_REGISTERS_H_

/**************************************************************************************************
  Includes
**************************************************************************************************/

#include "mxc_device.h"
#include "dbb_ctrl_regs.h"
#include "dbb_rx_regs.h"
#include "dbb_tx_regs.h"
#include "dbb_rffe_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************************************
  Macros
**************************************************************************************************/

#define MXC_DBB_CTRL ((mxc_dbb_ctrl_regs_t *)MXC_BASE_BTLE_DBB_CTRL)
#define MXC_DBB_RX ((mxc_dbb_rx_regs_t *)MXC_BASE_BTLE_DBB_RX)
#define MXC_DBB_TX ((mxc_dbb_tx_regs_t *)MXC_BASE_BTLE_DBB_TX)
#define MXC_DBB_RFFE ((mxc_dbb_rffe_regs_t *)MXC_BASE_BTLE_DBB_EXT_RFFE)

#ifdef __cplusplus
};
#endif

#endif // MAX32665_INCLUDE_DBB_REGISTERS_H_
