/**
 * @file    mcr_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the MCR Peripheral Module.
 * @note    This file is @generated.
 * @ingroup mcr_registers
 */

/******************************************************************************
 * Copyright (C) 2023 Maxim Integrated Products, Inc., All rights Reserved.
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
 *
 ******************************************************************************/

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32570_INCLUDE_MCR_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32570_INCLUDE_MCR_REGS_H_

/* **** Includes **** */
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined (__ICCARM__)
  #pragma system_include
#endif

#if defined (__CC_ARM)
  #pragma anon_unions
#endif
/// @cond
/*
    If types are not defined elsewhere (CMSIS) define them here
*/
#ifndef __IO
#define __IO volatile
#endif
#ifndef __I
#define __I  volatile const
#endif
#ifndef __O
#define __O  volatile
#endif
#ifndef __R
#define __R  volatile const
#endif
/// @endcond

/* **** Definitions **** */

/**
 * @ingroup     mcr
 * @defgroup    mcr_registers MCR_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the MCR Peripheral Module.
 * @details     Misc Control.
 */

/**
 * @ingroup mcr_registers
 * Structure type to access the MCR Registers.
 */
typedef struct {
    __R  uint32_t rsv_0x0_0x7[2];
    __IO uint32_t pdown;                /**< <tt>\b 0x08:</tt> MCR PDOWN Register */
    __R  uint32_t rsv_0xc;
    __IO uint32_t ctrl;                 /**< <tt>\b 0x10:</tt> MCR CTRL Register */
    __IO uint32_t clkctrl;              /**< <tt>\b 0x14:</tt> MCR CLKCTRL Register */
    __IO uint32_t rtcrst;               /**< <tt>\b 0x18:</tt> MCR RTCRST Register */
    __IO uint32_t rtctrim;              /**< <tt>\b 0x1C:</tt> MCR RTCTRIM Register */
    __R  uint32_t rsv_0x20_0x5f[16];
    __IO uint32_t ldoctrl;              /**< <tt>\b 0x60:</tt> MCR LDOCTRL Register */
    __IO uint32_t pwrmonst;             /**< <tt>\b 0x64:</tt> MCR PWRMONST Register */
} mxc_mcr_regs_t;

/* Register offsets for module MCR */
/**
 * @ingroup    mcr_registers
 * @defgroup   MCR_Register_Offsets Register Offsets
 * @brief      MCR Peripheral Register Offsets from the MCR Base Peripheral Address.
 * @{
 */
#define MXC_R_MCR_PDOWN                    ((uint32_t)0x00000008UL) /**< Offset from MCR Base Address: <tt> 0x0008</tt> */
#define MXC_R_MCR_CTRL                     ((uint32_t)0x00000010UL) /**< Offset from MCR Base Address: <tt> 0x0010</tt> */
#define MXC_R_MCR_CLKCTRL                  ((uint32_t)0x00000014UL) /**< Offset from MCR Base Address: <tt> 0x0014</tt> */
#define MXC_R_MCR_RTCRST                   ((uint32_t)0x00000018UL) /**< Offset from MCR Base Address: <tt> 0x0018</tt> */
#define MXC_R_MCR_RTCTRIM                  ((uint32_t)0x0000001CUL) /**< Offset from MCR Base Address: <tt> 0x001C</tt> */
#define MXC_R_MCR_LDOCTRL                  ((uint32_t)0x00000060UL) /**< Offset from MCR Base Address: <tt> 0x0060</tt> */
#define MXC_R_MCR_PWRMONST                 ((uint32_t)0x00000064UL) /**< Offset from MCR Base Address: <tt> 0x0064</tt> */
/**@} end of group mcr_registers */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_PDOWN MCR_PDOWN
 * @brief    PDOWN Drive Strength
 * @{
 */
#define MXC_F_MCR_PDOWN_DS_POS                         0 /**< PDOWN_DS Position */
#define MXC_F_MCR_PDOWN_DS                             ((uint32_t)(0x3UL << MXC_F_MCR_PDOWN_DS_POS)) /**< PDOWN_DS Mask */

#define MXC_F_MCR_PDOWN_VSEL_POS                       2 /**< PDOWN_VSEL Position */
#define MXC_F_MCR_PDOWN_VSEL                           ((uint32_t)(0x1UL << MXC_F_MCR_PDOWN_VSEL_POS)) /**< PDOWN_VSEL Mask */

/**@} end of group MCR_PDOWN_Register */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_CTRL MCR_CTRL
 * @brief    Control Register
 * @{
 */
#define MXC_F_MCR_CTRL_PADPUP_RST_POS                  9 /**< CTRL_PADPUP_RST Position */
#define MXC_F_MCR_CTRL_PADPUP_RST                      ((uint32_t)(0x1UL << MXC_F_MCR_CTRL_PADPUP_RST_POS)) /**< CTRL_PADPUP_RST Mask */

#define MXC_F_MCR_CTRL_PADVDDIOHSEL_RST_POS            10 /**< CTRL_PADVDDIOHSEL_RST Position */
#define MXC_F_MCR_CTRL_PADVDDIOHSEL_RST                ((uint32_t)(0x1UL << MXC_F_MCR_CTRL_PADVDDIOHSEL_RST_POS)) /**< CTRL_PADVDDIOHSEL_RST Mask */

/**@} end of group MCR_CTRL_Register */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_CLKCTRL MCR_CLKCTRL
 * @brief    System Clock Control Register
 * @{
 */
#define MXC_F_MCR_CLKCTRL_ERTCO_PD_POS                 16 /**< CLKCTRL_ERTCO_PD Position */
#define MXC_F_MCR_CLKCTRL_ERTCO_PD                     ((uint32_t)(0x1UL << MXC_F_MCR_CLKCTRL_ERTCO_PD_POS)) /**< CLKCTRL_ERTCO_PD Mask */

#define MXC_F_MCR_CLKCTRL_ERTCO_EN_POS                 17 /**< CLKCTRL_ERTCO_EN Position */
#define MXC_F_MCR_CLKCTRL_ERTCO_EN                     ((uint32_t)(0x1UL << MXC_F_MCR_CLKCTRL_ERTCO_EN_POS)) /**< CLKCTRL_ERTCO_EN Mask */

/**@} end of group MCR_CLKCTRL_Register */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_RTCRST MCR_RTCRST
 * @brief    Reset Register.
 * @{
 */
#define MXC_F_MCR_RTCRST_RTC_POS                       0 /**< RTCRST_RTC Position */
#define MXC_F_MCR_RTCRST_RTC                           ((uint32_t)(0x1UL << MXC_F_MCR_RTCRST_RTC_POS)) /**< RTCRST_RTC Mask */

/**@} end of group MCR_RTCRST_Register */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_RTCTRIM MCR_RTCTRIM
 * @brief    RTC Trim Register.
 * @{
 */
#define MXC_F_MCR_RTCTRIM_X1_POS                       0 /**< RTCTRIM_X1 Position */
#define MXC_F_MCR_RTCTRIM_X1                           ((uint32_t)(0x1FUL << MXC_F_MCR_RTCTRIM_X1_POS)) /**< RTCTRIM_X1 Mask */

#define MXC_F_MCR_RTCTRIM_X2_POS                       8 /**< RTCTRIM_X2 Position */
#define MXC_F_MCR_RTCTRIM_X2                           ((uint32_t)(0x1FUL << MXC_F_MCR_RTCTRIM_X2_POS)) /**< RTCTRIM_X2 Mask */

/**@} end of group MCR_RTCTRIM_Register */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_LDOCTRL MCR_LDOCTRL
 * @brief    LDO Control Register.
 * @{
 */
#define MXC_F_MCR_LDOCTRL_0P9V_EN_POS                  0 /**< LDOCTRL_0P9V_EN Position */
#define MXC_F_MCR_LDOCTRL_0P9V_EN                      ((uint32_t)(0x1UL << MXC_F_MCR_LDOCTRL_0P9V_EN_POS)) /**< LDOCTRL_0P9V_EN Mask */

/**@} end of group MCR_LDOCTRL_Register */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_PWRMONST MCR_PWRMONST
 * @brief    LDO Control Register.
 * @{
 */
#define MXC_F_MCR_PWRMONST_PORZ_VLOSS_POS              0 /**< PWRMONST_PORZ_VLOSS Position */
#define MXC_F_MCR_PWRMONST_PORZ_VLOSS                  ((uint32_t)(0x1UL << MXC_F_MCR_PWRMONST_PORZ_VLOSS_POS)) /**< PWRMONST_PORZ_VLOSS Mask */

#define MXC_F_MCR_PWRMONST_PORZ_VBAT_POS               1 /**< PWRMONST_PORZ_VBAT Position */
#define MXC_F_MCR_PWRMONST_PORZ_VBAT                   ((uint32_t)(0x1UL << MXC_F_MCR_PWRMONST_PORZ_VBAT_POS)) /**< PWRMONST_PORZ_VBAT Mask */

#define MXC_F_MCR_PWRMONST_PORZ_VBB_POS                2 /**< PWRMONST_PORZ_VBB Position */
#define MXC_F_MCR_PWRMONST_PORZ_VBB                    ((uint32_t)(0x1UL << MXC_F_MCR_PWRMONST_PORZ_VBB_POS)) /**< PWRMONST_PORZ_VBB Mask */

#define MXC_F_MCR_PWRMONST_PORZ_VDDCA_POS              4 /**< PWRMONST_PORZ_VDDCA Position */
#define MXC_F_MCR_PWRMONST_PORZ_VDDCA                  ((uint32_t)(0x1UL << MXC_F_MCR_PWRMONST_PORZ_VDDCA_POS)) /**< PWRMONST_PORZ_VDDCA Mask */

#define MXC_F_MCR_PWRMONST_PORZ_VDDCB_POS              5 /**< PWRMONST_PORZ_VDDCB Position */
#define MXC_F_MCR_PWRMONST_PORZ_VDDCB                  ((uint32_t)(0x1UL << MXC_F_MCR_PWRMONST_PORZ_VDDCB_POS)) /**< PWRMONST_PORZ_VDDCB Mask */

#define MXC_F_MCR_PWRMONST_PORZ_VDDA_POS               6 /**< PWRMONST_PORZ_VDDA Position */
#define MXC_F_MCR_PWRMONST_PORZ_VDDA                   ((uint32_t)(0x1UL << MXC_F_MCR_PWRMONST_PORZ_VDDA_POS)) /**< PWRMONST_PORZ_VDDA Mask */

#define MXC_F_MCR_PWRMONST_PORZ_VDDB_POS               7 /**< PWRMONST_PORZ_VDDB Position */
#define MXC_F_MCR_PWRMONST_PORZ_VDDB                   ((uint32_t)(0x1UL << MXC_F_MCR_PWRMONST_PORZ_VDDB_POS)) /**< PWRMONST_PORZ_VDDB Mask */

#define MXC_F_MCR_PWRMONST_RSTZ_VDDCB_POS              9 /**< PWRMONST_RSTZ_VDDCB Position */
#define MXC_F_MCR_PWRMONST_RSTZ_VDDCB                  ((uint32_t)(0x1UL << MXC_F_MCR_PWRMONST_RSTZ_VDDCB_POS)) /**< PWRMONST_RSTZ_VDDCB Mask */

#define MXC_F_MCR_PWRMONST_RSTZ_VDDA_POS               10 /**< PWRMONST_RSTZ_VDDA Position */
#define MXC_F_MCR_PWRMONST_RSTZ_VDDA                   ((uint32_t)(0x1UL << MXC_F_MCR_PWRMONST_RSTZ_VDDA_POS)) /**< PWRMONST_RSTZ_VDDA Mask */

#define MXC_F_MCR_PWRMONST_RSTZ_VDDB_POS               11 /**< PWRMONST_RSTZ_VDDB Position */
#define MXC_F_MCR_PWRMONST_RSTZ_VDDB                   ((uint32_t)(0x1UL << MXC_F_MCR_PWRMONST_RSTZ_VDDB_POS)) /**< PWRMONST_RSTZ_VDDB Mask */

#define MXC_F_MCR_PWRMONST_RSTZ_VDDIO_POS              12 /**< PWRMONST_RSTZ_VDDIO Position */
#define MXC_F_MCR_PWRMONST_RSTZ_VDDIO                  ((uint32_t)(0x1UL << MXC_F_MCR_PWRMONST_RSTZ_VDDIO_POS)) /**< PWRMONST_RSTZ_VDDIO Mask */

#define MXC_F_MCR_PWRMONST_RSTZ_VDDIOH_POS             13 /**< PWRMONST_RSTZ_VDDIOH Position */
#define MXC_F_MCR_PWRMONST_RSTZ_VDDIOH                 ((uint32_t)(0x1UL << MXC_F_MCR_PWRMONST_RSTZ_VDDIOH_POS)) /**< PWRMONST_RSTZ_VDDIOH Mask */

#define MXC_F_MCR_PWRMONST_RSTZ_VBB_POS                14 /**< PWRMONST_RSTZ_VBB Position */
#define MXC_F_MCR_PWRMONST_RSTZ_VBB                    ((uint32_t)(0x1UL << MXC_F_MCR_PWRMONST_RSTZ_VBB_POS)) /**< PWRMONST_RSTZ_VBB Mask */

#define MXC_F_MCR_PWRMONST_RSTZ_LDO0P9V_POS            16 /**< PWRMONST_RSTZ_LDO0P9V Position */
#define MXC_F_MCR_PWRMONST_RSTZ_LDO0P9V                ((uint32_t)(0x1UL << MXC_F_MCR_PWRMONST_RSTZ_LDO0P9V_POS)) /**< PWRMONST_RSTZ_LDO0P9V Mask */

#define MXC_F_MCR_PWRMONST_RSTZ_VDDCA_POS              17 /**< PWRMONST_RSTZ_VDDCA Position */
#define MXC_F_MCR_PWRMONST_RSTZ_VDDCA                  ((uint32_t)(0x1UL << MXC_F_MCR_PWRMONST_RSTZ_VDDCA_POS)) /**< PWRMONST_RSTZ_VDDCA Mask */

#define MXC_F_MCR_PWRMONST_RSTZ_VCOREHV_POS            18 /**< PWRMONST_RSTZ_VCOREHV Position */
#define MXC_F_MCR_PWRMONST_RSTZ_VCOREHV                ((uint32_t)(0x1UL << MXC_F_MCR_PWRMONST_RSTZ_VCOREHV_POS)) /**< PWRMONST_RSTZ_VCOREHV Mask */

#define MXC_F_MCR_PWRMONST_RSTZ_VDDIOHV_POS            19 /**< PWRMONST_RSTZ_VDDIOHV Position */
#define MXC_F_MCR_PWRMONST_RSTZ_VDDIOHV                ((uint32_t)(0x1UL << MXC_F_MCR_PWRMONST_RSTZ_VDDIOHV_POS)) /**< PWRMONST_RSTZ_VDDIOHV Mask */

#define MXC_F_MCR_PWRMONST_RSTZ_VDDIOHHV_POS           20 /**< PWRMONST_RSTZ_VDDIOHHV Position */
#define MXC_F_MCR_PWRMONST_RSTZ_VDDIOHHV               ((uint32_t)(0x1UL << MXC_F_MCR_PWRMONST_RSTZ_VDDIOHHV_POS)) /**< PWRMONST_RSTZ_VDDIOHHV Mask */

#define MXC_F_MCR_PWRMONST_RSTZ_VNFCRX_POS             21 /**< PWRMONST_RSTZ_VNFCRX Position */
#define MXC_F_MCR_PWRMONST_RSTZ_VNFCRX                 ((uint32_t)(0x1UL << MXC_F_MCR_PWRMONST_RSTZ_VNFCRX_POS)) /**< PWRMONST_RSTZ_VNFCRX Mask */

#define MXC_F_MCR_PWRMONST_RSTZ_VNFCTX_POS             22 /**< PWRMONST_RSTZ_VNFCTX Position */
#define MXC_F_MCR_PWRMONST_RSTZ_VNFCTX                 ((uint32_t)(0x1UL << MXC_F_MCR_PWRMONST_RSTZ_VNFCTX_POS)) /**< PWRMONST_RSTZ_VNFCTX Mask */

#define MXC_F_MCR_PWRMONST_RSTZ_VNFC1V_POS             23 /**< PWRMONST_RSTZ_VNFC1V Position */
#define MXC_F_MCR_PWRMONST_RSTZ_VNFC1V                 ((uint32_t)(0x1UL << MXC_F_MCR_PWRMONST_RSTZ_VNFC1V_POS)) /**< PWRMONST_RSTZ_VNFC1V Mask */

/**@} end of group MCR_PWRMONST_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32570_INCLUDE_MCR_REGS_H_
