/**
 * @file    mcr_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the MCR Peripheral Module.
 * @note    This file is @generated.
 */

/******************************************************************************
 * Copyright (C) 2022 Maxim Integrated Products, Inc., All rights Reserved.
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
    __IO uint32_t eccen;                /**< <tt>\b 0x00:</tt> MCR ECCEN Register */
    __R  uint32_t rsv_0x4;
    __IO uint32_t pdown;                /**< <tt>\b 0x08:</tt> MCR PDOWN Register */
    __R  uint32_t rsv_0xc;
    __IO uint32_t ctrl;                 /**< <tt>\b 0x10:</tt> MCR CTRL Register */
    __IO uint32_t clkctrl;              /**< <tt>\b 0x14:</tt> MCR CLKCTRL Register */
    __IO uint32_t rtcrst;               /**< <tt>\b 0x18:</tt> MCR RTCRST Register */
    __IO uint32_t rtctrim;              /**< <tt>\b 0x1C:</tt> MCR RTCTRIM Register */
    __R  uint32_t rsv_0x20_0x5f[16];
    __IO uint32_t ldoctrl;              /**< <tt>\b 0x60:</tt> MCR LDOCTRL Register */
} mxc_mcr_regs_t;

/* Register offsets for module MCR */
/**
 * @ingroup    mcr_registers
 * @defgroup   MCR_Register_Offsets Register Offsets
 * @brief      MCR Peripheral Register Offsets from the MCR Base Peripheral Address.
 * @{
 */
#define MXC_R_MCR_ECCEN                    ((uint32_t)0x00000000UL) /**< Offset from MCR Base Address: <tt> 0x0000</tt> */
#define MXC_R_MCR_PDOWN                    ((uint32_t)0x00000008UL) /**< Offset from MCR Base Address: <tt> 0x0008</tt> */
#define MXC_R_MCR_CTRL                     ((uint32_t)0x00000010UL) /**< Offset from MCR Base Address: <tt> 0x0010</tt> */
#define MXC_R_MCR_CLKCTRL                  ((uint32_t)0x00000014UL) /**< Offset from MCR Base Address: <tt> 0x0014</tt> */
#define MXC_R_MCR_RTCRST                   ((uint32_t)0x00000018UL) /**< Offset from MCR Base Address: <tt> 0x0018</tt> */
#define MXC_R_MCR_RTCTRIM                  ((uint32_t)0x0000001CUL) /**< Offset from MCR Base Address: <tt> 0x001C</tt> */
#define MXC_R_MCR_LDOCTRL                  ((uint32_t)0x00000060UL) /**< Offset from MCR Base Address: <tt> 0x0060</tt> */
/**@} end of group mcr_registers */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_ECCEN MCR_ECCEN
 * @brief    ECC Enable Register
 * @{
 */
#define MXC_F_MCR_ECCEN_RAM0_POS                       0 /**< ECCEN_RAM0 Position */
#define MXC_F_MCR_ECCEN_RAM0                           ((uint32_t)(0x1UL << MXC_F_MCR_ECCEN_RAM0_POS)) /**< ECCEN_RAM0 Mask */

#define MXC_F_MCR_ECCEN_RAM1_POS                       1 /**< ECCEN_RAM1 Position */
#define MXC_F_MCR_ECCEN_RAM1                           ((uint32_t)(0x1UL << MXC_F_MCR_ECCEN_RAM1_POS)) /**< ECCEN_RAM1 Mask */

#define MXC_F_MCR_ECCEN_RAM2_POS                       2 /**< ECCEN_RAM2 Position */
#define MXC_F_MCR_ECCEN_RAM2                           ((uint32_t)(0x1UL << MXC_F_MCR_ECCEN_RAM2_POS)) /**< ECCEN_RAM2 Mask */

#define MXC_F_MCR_ECCEN_RAM3_POS                       3 /**< ECCEN_RAM3 Position */
#define MXC_F_MCR_ECCEN_RAM3                           ((uint32_t)(0x1UL << MXC_F_MCR_ECCEN_RAM3_POS)) /**< ECCEN_RAM3 Mask */

#define MXC_F_MCR_ECCEN_RAM4_POS                       4 /**< ECCEN_RAM4 Position */
#define MXC_F_MCR_ECCEN_RAM4                           ((uint32_t)(0x1UL << MXC_F_MCR_ECCEN_RAM4_POS)) /**< ECCEN_RAM4 Mask */

#define MXC_F_MCR_ECCEN_RAM5_POS                       5 /**< ECCEN_RAM5 Position */
#define MXC_F_MCR_ECCEN_RAM5                           ((uint32_t)(0x1UL << MXC_F_MCR_ECCEN_RAM5_POS)) /**< ECCEN_RAM5 Mask */

#define MXC_F_MCR_ECCEN_ICC_POS                        8 /**< ECCEN_ICC Position */
#define MXC_F_MCR_ECCEN_ICC                            ((uint32_t)(0x1UL << MXC_F_MCR_ECCEN_ICC_POS)) /**< ECCEN_ICC Mask */

#define MXC_F_MCR_ECCEN_ICCXIP_POS                     10 /**< ECCEN_ICCXIP Position */
#define MXC_F_MCR_ECCEN_ICCXIP                         ((uint32_t)(0x1UL << MXC_F_MCR_ECCEN_ICCXIP_POS)) /**< ECCEN_ICCXIP Mask */

#define MXC_F_MCR_ECCEN_FLASH0_POS                     11 /**< ECCEN_FLASH0 Position */
#define MXC_F_MCR_ECCEN_FLASH0                         ((uint32_t)(0x1UL << MXC_F_MCR_ECCEN_FLASH0_POS)) /**< ECCEN_FLASH0 Mask */

#define MXC_F_MCR_ECCEN_FLASH1_POS                     12 /**< ECCEN_FLASH1 Position */
#define MXC_F_MCR_ECCEN_FLASH1                         ((uint32_t)(0x1UL << MXC_F_MCR_ECCEN_FLASH1_POS)) /**< ECCEN_FLASH1 Mask */

/**@} end of group MCR_ECCEN_Register */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_PDOWN MCR_PDOWN
 * @brief    PDOWN Drive Strength
 * @{
 */
#define MXC_F_MCR_PDOWN_DS_POS                         0 /**< PDOWN_DS Position */
#define MXC_F_MCR_PDOWN_DS                             ((uint32_t)(0x3UL << MXC_F_MCR_PDOWN_DS_POS)) /**< PDOWN_DS Mask */

#define MXC_F_MCR_PDOWN_VSSEL_POS                      2 /**< PDOWN_VSSEL Position */
#define MXC_F_MCR_PDOWN_VSSEL                          ((uint32_t)(0x1UL << MXC_F_MCR_PDOWN_VSSEL_POS)) /**< PDOWN_VSSEL Mask */

/**@} end of group MCR_PDOWN_Register */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_CTRL MCR_CTRL
 * @brief    Control Register
 * @{
 */
#define MXC_F_MCR_CTRL_VDDCSW_EN_POS                   0 /**< CTRL_VDDCSW_EN Position */
#define MXC_F_MCR_CTRL_VDDCSW_EN                       ((uint32_t)(0x1UL << MXC_F_MCR_CTRL_VDDCSW_EN_POS)) /**< CTRL_VDDCSW_EN Mask */

#define MXC_F_MCR_CTRL_VDDCSW_POS                      1 /**< CTRL_VDDCSW Position */
#define MXC_F_MCR_CTRL_VDDCSW                          ((uint32_t)(0x3UL << MXC_F_MCR_CTRL_VDDCSW_POS)) /**< CTRL_VDDCSW Mask */

#define MXC_F_MCR_CTRL_USBSW_DIS_POS                   3 /**< CTRL_USBSW_DIS Position */
#define MXC_F_MCR_CTRL_USBSW_DIS                       ((uint32_t)(0x1UL << MXC_F_MCR_CTRL_USBSW_DIS_POS)) /**< CTRL_USBSW_DIS Mask */

#define MXC_F_MCR_CTRL_PADPUP_POS                      9 /**< CTRL_PADPUP Position */
#define MXC_F_MCR_CTRL_PADPUP                          ((uint32_t)(0x1UL << MXC_F_MCR_CTRL_PADPUP_POS)) /**< CTRL_PADPUP Mask */

#define MXC_F_MCR_CTRL_VDDIOHSEL_RST_POS               10 /**< CTRL_VDDIOHSEL_RST Position */
#define MXC_F_MCR_CTRL_VDDIOHSEL_RST                   ((uint32_t)(0x1UL << MXC_F_MCR_CTRL_VDDIOHSEL_RST_POS)) /**< CTRL_VDDIOHSEL_RST Mask */

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
 * @brief    Reset Registter.
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
#define MXC_F_MCR_RTCTRIM_TRIM_X1_POS                  0 /**< RTCTRIM_TRIM_X1 Position */
#define MXC_F_MCR_RTCTRIM_TRIM_X1                      ((uint32_t)(0x1FUL << MXC_F_MCR_RTCTRIM_TRIM_X1_POS)) /**< RTCTRIM_TRIM_X1 Mask */

#define MXC_F_MCR_RTCTRIM_TRIM_X2_POS                  8 /**< RTCTRIM_TRIM_X2 Position */
#define MXC_F_MCR_RTCTRIM_TRIM_X2                      ((uint32_t)(0x1FUL << MXC_F_MCR_RTCTRIM_TRIM_X2_POS)) /**< RTCTRIM_TRIM_X2 Mask */

/**@} end of group MCR_RTCTRIM_Register */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_LDOCTRL MCR_LDOCTRL
 * @brief    LDO Control Register.
 * @{
 */
#define MXC_F_MCR_LDOCTRL_LDO0P9_EN_POS                0 /**< LDOCTRL_LDO0P9_EN Position */
#define MXC_F_MCR_LDOCTRL_LDO0P9_EN                    ((uint32_t)(0x1UL << MXC_F_MCR_LDOCTRL_LDO0P9_EN_POS)) /**< LDOCTRL_LDO0P9_EN Mask */

/**@} end of group MCR_LDOCTRL_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32570_INCLUDE_MCR_REGS_H_
