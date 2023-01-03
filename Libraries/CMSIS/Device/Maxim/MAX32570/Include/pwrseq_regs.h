/**
 * @file    pwrseq_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the PWRSEQ Peripheral Module.
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32570_INCLUDE_PWRSEQ_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32570_INCLUDE_PWRSEQ_REGS_H_

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
 * @ingroup     pwrseq
 * @defgroup    pwrseq_registers PWRSEQ_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the PWRSEQ Peripheral Module.
 * @details     Power Sequencer / Low Power Control Register.
 */

/**
 * @ingroup pwrseq_registers
 * Structure type to access the PWRSEQ Registers.
 */
typedef struct {
    __IO uint32_t lpctrl;               /**< <tt>\b 0x00:</tt> PWRSEQ LPCTRL Register */
    __IO uint32_t lpwkfl0;              /**< <tt>\b 0x04:</tt> PWRSEQ LPWKFL0 Register */
    __IO uint32_t lpwken0;              /**< <tt>\b 0x08:</tt> PWRSEQ LPWKEN0 Register */
    __IO uint32_t lpwkfl1;              /**< <tt>\b 0x0C:</tt> PWRSEQ LPWKFL1 Register */
    __IO uint32_t lpwken1;              /**< <tt>\b 0x10:</tt> PWRSEQ LPWKEN1 Register */
    __IO uint32_t lpwkfl2;              /**< <tt>\b 0x14:</tt> PWRSEQ LPWKFL2 Register */
    __IO uint32_t lpwken2;              /**< <tt>\b 0x18:</tt> PWRSEQ LPWKEN2 Register */
    __IO uint32_t lpwkfl3;              /**< <tt>\b 0x1C:</tt> PWRSEQ LPWKFL3 Register */
    __IO uint32_t lpwken3;              /**< <tt>\b 0x20:</tt> PWRSEQ LPWKEN3 Register */
    __R  uint32_t rsv_0x24_0x2f[3];
    __IO uint32_t lppwkfl;              /**< <tt>\b 0x30:</tt> PWRSEQ LPPWKFL Register */
    __IO uint32_t lppwken;              /**< <tt>\b 0x34:</tt> PWRSEQ LPPWKEN Register */
    __R  uint32_t rsv_0x38_0x3f[2];
    __IO uint32_t lpmemsd;              /**< <tt>\b 0x40:</tt> PWRSEQ LPMEMSD Register */
    __IO uint32_t lpvddpd;              /**< <tt>\b 0x44:</tt> PWRSEQ LPVDDPD Register */
    __IO uint32_t gp0;                  /**< <tt>\b 0x48:</tt> PWRSEQ GP0 Register */
    __IO uint32_t gp1;                  /**< <tt>\b 0x4C:</tt> PWRSEQ GP1 Register */
} mxc_pwrseq_regs_t;

/* Register offsets for module PWRSEQ */
/**
 * @ingroup    pwrseq_registers
 * @defgroup   PWRSEQ_Register_Offsets Register Offsets
 * @brief      PWRSEQ Peripheral Register Offsets from the PWRSEQ Base Peripheral Address.
 * @{
 */
#define MXC_R_PWRSEQ_LPCTRL                ((uint32_t)0x00000000UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0000</tt> */
#define MXC_R_PWRSEQ_LPWKFL0               ((uint32_t)0x00000004UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0004</tt> */
#define MXC_R_PWRSEQ_LPWKEN0               ((uint32_t)0x00000008UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0008</tt> */
#define MXC_R_PWRSEQ_LPWKFL1               ((uint32_t)0x0000000CUL) /**< Offset from PWRSEQ Base Address: <tt> 0x000C</tt> */
#define MXC_R_PWRSEQ_LPWKEN1               ((uint32_t)0x00000010UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0010</tt> */
#define MXC_R_PWRSEQ_LPWKFL2               ((uint32_t)0x00000014UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0014</tt> */
#define MXC_R_PWRSEQ_LPWKEN2               ((uint32_t)0x00000018UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0018</tt> */
#define MXC_R_PWRSEQ_LPWKFL3               ((uint32_t)0x0000001CUL) /**< Offset from PWRSEQ Base Address: <tt> 0x001C</tt> */
#define MXC_R_PWRSEQ_LPWKEN3               ((uint32_t)0x00000020UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0020</tt> */
#define MXC_R_PWRSEQ_LPPWKFL               ((uint32_t)0x00000030UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0030</tt> */
#define MXC_R_PWRSEQ_LPPWKEN               ((uint32_t)0x00000034UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0034</tt> */
#define MXC_R_PWRSEQ_LPMEMSD               ((uint32_t)0x00000040UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0040</tt> */
#define MXC_R_PWRSEQ_LPVDDPD               ((uint32_t)0x00000044UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0044</tt> */
#define MXC_R_PWRSEQ_GP0                   ((uint32_t)0x00000048UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0048</tt> */
#define MXC_R_PWRSEQ_GP1                   ((uint32_t)0x0000004CUL) /**< Offset from PWRSEQ Base Address: <tt> 0x004C</tt> */
/**@} end of group pwrseq_registers */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_LPCTRL PWRSEQ_LPCTRL
 * @brief    Low Power Control Register.
 * @{
 */
#define MXC_F_PWRSEQ_LPCTRL_RAMRET_EN_POS              0 /**< LPCTRL_RAMRET_EN Position */
#define MXC_F_PWRSEQ_LPCTRL_RAMRET_EN                  ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPCTRL_RAMRET_EN_POS)) /**< LPCTRL_RAMRET_EN Mask */

#define MXC_F_PWRSEQ_LPCTRL_OVR_POS                    4 /**< LPCTRL_OVR Position */
#define MXC_F_PWRSEQ_LPCTRL_OVR                        ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPCTRL_OVR_POS)) /**< LPCTRL_OVR Mask */
#define MXC_V_PWRSEQ_LPCTRL_OVR_0_9V                   ((uint32_t)0x0UL) /**< LPCTRL_OVR_0_9V Value */
#define MXC_S_PWRSEQ_LPCTRL_OVR_0_9V                   (MXC_V_PWRSEQ_LPCTRL_OVR_0_9V << MXC_F_PWRSEQ_LPCTRL_OVR_POS) /**< LPCTRL_OVR_0_9V Setting */
#define MXC_V_PWRSEQ_LPCTRL_OVR_1_0V                   ((uint32_t)0x1UL) /**< LPCTRL_OVR_1_0V Value */
#define MXC_S_PWRSEQ_LPCTRL_OVR_1_0V                   (MXC_V_PWRSEQ_LPCTRL_OVR_1_0V << MXC_F_PWRSEQ_LPCTRL_OVR_POS) /**< LPCTRL_OVR_1_0V Setting */
#define MXC_V_PWRSEQ_LPCTRL_OVR_1_1V                   ((uint32_t)0x2UL) /**< LPCTRL_OVR_1_1V Value */
#define MXC_S_PWRSEQ_LPCTRL_OVR_1_1V                   (MXC_V_PWRSEQ_LPCTRL_OVR_1_1V << MXC_F_PWRSEQ_LPCTRL_OVR_POS) /**< LPCTRL_OVR_1_1V Setting */

#define MXC_F_PWRSEQ_LPCTRL_RETREG_EN_POS              8 /**< LPCTRL_RETREG_EN Position */
#define MXC_F_PWRSEQ_LPCTRL_RETREG_EN                  ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCTRL_RETREG_EN_POS)) /**< LPCTRL_RETREG_EN Mask */

#define MXC_F_PWRSEQ_LPCTRL_FASTWK_EN_POS              10 /**< LPCTRL_FASTWK_EN Position */
#define MXC_F_PWRSEQ_LPCTRL_FASTWK_EN                  ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCTRL_FASTWK_EN_POS)) /**< LPCTRL_FASTWK_EN Mask */

#define MXC_F_PWRSEQ_LPCTRL_BG_DIS_POS                 11 /**< LPCTRL_BG_DIS Position */
#define MXC_F_PWRSEQ_LPCTRL_BG_DIS                     ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCTRL_BG_DIS_POS)) /**< LPCTRL_BG_DIS Mask */

#define MXC_F_PWRSEQ_LPCTRL_VCOREPOR_DIS_POS           12 /**< LPCTRL_VCOREPOR_DIS Position */
#define MXC_F_PWRSEQ_LPCTRL_VCOREPOR_DIS               ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCTRL_VCOREPOR_DIS_POS)) /**< LPCTRL_VCOREPOR_DIS Mask */

#define MXC_F_PWRSEQ_LPCTRL_LDO_DIS_POS                16 /**< LPCTRL_LDO_DIS Position */
#define MXC_F_PWRSEQ_LPCTRL_LDO_DIS                    ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCTRL_LDO_DIS_POS)) /**< LPCTRL_LDO_DIS Mask */

#define MXC_F_PWRSEQ_LPCTRL_VCOREMON_DIS_POS           20 /**< LPCTRL_VCOREMON_DIS Position */
#define MXC_F_PWRSEQ_LPCTRL_VCOREMON_DIS               ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCTRL_VCOREMON_DIS_POS)) /**< LPCTRL_VCOREMON_DIS Mask */

#define MXC_F_PWRSEQ_LPCTRL_VRTCMON_DIS_POS            21 /**< LPCTRL_VRTCMON_DIS Position */
#define MXC_F_PWRSEQ_LPCTRL_VRTCMON_DIS                ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCTRL_VRTCMON_DIS_POS)) /**< LPCTRL_VRTCMON_DIS Mask */

#define MXC_F_PWRSEQ_LPCTRL_VDDAMON_DIS_POS            22 /**< LPCTRL_VDDAMON_DIS Position */
#define MXC_F_PWRSEQ_LPCTRL_VDDAMON_DIS                ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCTRL_VDDAMON_DIS_POS)) /**< LPCTRL_VDDAMON_DIS Mask */

#define MXC_F_PWRSEQ_LPCTRL_VDDIOMON_DIS_POS           23 /**< LPCTRL_VDDIOMON_DIS Position */
#define MXC_F_PWRSEQ_LPCTRL_VDDIOMON_DIS               ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCTRL_VDDIOMON_DIS_POS)) /**< LPCTRL_VDDIOMON_DIS Mask */

#define MXC_F_PWRSEQ_LPCTRL_VDDIOHMON_DIS_POS          24 /**< LPCTRL_VDDIOHMON_DIS Position */
#define MXC_F_PWRSEQ_LPCTRL_VDDIOHMON_DIS              ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCTRL_VDDIOHMON_DIS_POS)) /**< LPCTRL_VDDIOHMON_DIS Mask */

#define MXC_F_PWRSEQ_LPCTRL_VDDBMON_DIS_POS            27 /**< LPCTRL_VDDBMON_DIS Position */
#define MXC_F_PWRSEQ_LPCTRL_VDDBMON_DIS                ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCTRL_VDDBMON_DIS_POS)) /**< LPCTRL_VDDBMON_DIS Mask */

/**@} end of group PWRSEQ_LPCTRL_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_LPWKFL0 PWRSEQ_LPWKFL0
 * @brief    Low Power I/O Wakeup Status Register 0. This register indicates the low power
 *           wakeup status for GPIO0.
 * @{
 */
#define MXC_F_PWRSEQ_LPWKFL0_ALL_POS                   0 /**< LPWKFL0_ALL Position */
#define MXC_F_PWRSEQ_LPWKFL0_ALL                       ((uint32_t)(0x7FFFFFFFUL << MXC_F_PWRSEQ_LPWKFL0_ALL_POS)) /**< LPWKFL0_ALL Mask */

/**@} end of group PWRSEQ_LPWKFL0_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_LPWKEN0 PWRSEQ_LPWKEN0
 * @brief    Low Power I/O Wakeup Enable Register 0. This register enables low power wakeup
 *           functionality for GPIO0.
 * @{
 */
#define MXC_F_PWRSEQ_LPWKEN0_ALL_POS                   0 /**< LPWKEN0_ALL Position */
#define MXC_F_PWRSEQ_LPWKEN0_ALL                       ((uint32_t)(0x7FFFFFFFUL << MXC_F_PWRSEQ_LPWKEN0_ALL_POS)) /**< LPWKEN0_ALL Mask */

/**@} end of group PWRSEQ_LPWKEN0_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_LPPWKFL PWRSEQ_LPPWKFL
 * @brief    Low Power Peripheral Wakeup Status Register.
 * @{
 */
#define MXC_F_PWRSEQ_LPPWKFL_USBLS_POS                 0 /**< LPPWKFL_USBLS Position */
#define MXC_F_PWRSEQ_LPPWKFL_USBLS                     ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPPWKFL_USBLS_POS)) /**< LPPWKFL_USBLS Mask */

#define MXC_F_PWRSEQ_LPPWKFL_USBVBUS_POS               2 /**< LPPWKFL_USBVBUS Position */
#define MXC_F_PWRSEQ_LPPWKFL_USBVBUS                   ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWKFL_USBVBUS_POS)) /**< LPPWKFL_USBVBUS Mask */

#define MXC_F_PWRSEQ_LPPWKFL_SDMA0_POS                 3 /**< LPPWKFL_SDMA0 Position */
#define MXC_F_PWRSEQ_LPPWKFL_SDMA0                     ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWKFL_SDMA0_POS)) /**< LPPWKFL_SDMA0 Mask */

#define MXC_F_PWRSEQ_LPPWKFL_BACKUP_POS                16 /**< LPPWKFL_BACKUP Position */
#define MXC_F_PWRSEQ_LPPWKFL_BACKUP                    ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWKFL_BACKUP_POS)) /**< LPPWKFL_BACKUP Mask */

#define MXC_F_PWRSEQ_LPPWKFL_RESET_POS                 17 /**< LPPWKFL_RESET Position */
#define MXC_F_PWRSEQ_LPPWKFL_RESET                     ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWKFL_RESET_POS)) /**< LPPWKFL_RESET Mask */

#define MXC_F_PWRSEQ_LPPWKFL_SDMA1_POS                 18 /**< LPPWKFL_SDMA1 Position */
#define MXC_F_PWRSEQ_LPPWKFL_SDMA1                     ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWKFL_SDMA1_POS)) /**< LPPWKFL_SDMA1 Mask */

/**@} end of group PWRSEQ_LPPWKFL_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_LPPWKEN PWRSEQ_LPPWKEN
 * @brief    Low Power Peripheral Wakeup Enable Register.
 * @{
 */
#define MXC_F_PWRSEQ_LPPWKEN_USBLS_POS                 0 /**< LPPWKEN_USBLS Position */
#define MXC_F_PWRSEQ_LPPWKEN_USBLS                     ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPPWKEN_USBLS_POS)) /**< LPPWKEN_USBLS Mask */

#define MXC_F_PWRSEQ_LPPWKEN_USBVBUS_POS               2 /**< LPPWKEN_USBVBUS Position */
#define MXC_F_PWRSEQ_LPPWKEN_USBVBUS                   ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWKEN_USBVBUS_POS)) /**< LPPWKEN_USBVBUS Mask */

#define MXC_F_PWRSEQ_LPPWKEN_SDMA0_POS                 3 /**< LPPWKEN_SDMA0 Position */
#define MXC_F_PWRSEQ_LPPWKEN_SDMA0                     ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWKEN_SDMA0_POS)) /**< LPPWKEN_SDMA0 Mask */

#define MXC_F_PWRSEQ_LPPWKEN_SDMA1_POS                 18 /**< LPPWKEN_SDMA1 Position */
#define MXC_F_PWRSEQ_LPPWKEN_SDMA1                     ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWKEN_SDMA1_POS)) /**< LPPWKEN_SDMA1 Mask */

/**@} end of group PWRSEQ_LPPWKEN_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_LPMEMSD PWRSEQ_LPMEMSD
 * @brief    Low Power Memory Shutdown Control.
 * @{
 */
#define MXC_F_PWRSEQ_LPMEMSD_RAM0_POS                  0 /**< LPMEMSD_RAM0 Position */
#define MXC_F_PWRSEQ_LPMEMSD_RAM0                      ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPMEMSD_RAM0_POS)) /**< LPMEMSD_RAM0 Mask */

#define MXC_F_PWRSEQ_LPMEMSD_RAM1_POS                  1 /**< LPMEMSD_RAM1 Position */
#define MXC_F_PWRSEQ_LPMEMSD_RAM1                      ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPMEMSD_RAM1_POS)) /**< LPMEMSD_RAM1 Mask */

#define MXC_F_PWRSEQ_LPMEMSD_RAM2_POS                  2 /**< LPMEMSD_RAM2 Position */
#define MXC_F_PWRSEQ_LPMEMSD_RAM2                      ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPMEMSD_RAM2_POS)) /**< LPMEMSD_RAM2 Mask */

#define MXC_F_PWRSEQ_LPMEMSD_RAM3_POS                  3 /**< LPMEMSD_RAM3 Position */
#define MXC_F_PWRSEQ_LPMEMSD_RAM3                      ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPMEMSD_RAM3_POS)) /**< LPMEMSD_RAM3 Mask */

#define MXC_F_PWRSEQ_LPMEMSD_ICC_POS                   7 /**< LPMEMSD_ICC Position */
#define MXC_F_PWRSEQ_LPMEMSD_ICC                       ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPMEMSD_ICC_POS)) /**< LPMEMSD_ICC Mask */

#define MXC_F_PWRSEQ_LPMEMSD_ICCXIP_POS                8 /**< LPMEMSD_ICCXIP Position */
#define MXC_F_PWRSEQ_LPMEMSD_ICCXIP                    ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPMEMSD_ICCXIP_POS)) /**< LPMEMSD_ICCXIP Mask */

#define MXC_F_PWRSEQ_LPMEMSD_EMCC_POS                  9 /**< LPMEMSD_EMCC Position */
#define MXC_F_PWRSEQ_LPMEMSD_EMCC                      ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPMEMSD_EMCC_POS)) /**< LPMEMSD_EMCC Mask */

#define MXC_F_PWRSEQ_LPMEMSD_MEU_POS                   11 /**< LPMEMSD_MEU Position */
#define MXC_F_PWRSEQ_LPMEMSD_MEU                       ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPMEMSD_MEU_POS)) /**< LPMEMSD_MEU Mask */

#define MXC_F_PWRSEQ_LPMEMSD_USBFIFO_POS               12 /**< LPMEMSD_USBFIFO Position */
#define MXC_F_PWRSEQ_LPMEMSD_USBFIFO                   ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPMEMSD_USBFIFO_POS)) /**< LPMEMSD_USBFIFO Mask */

#define MXC_F_PWRSEQ_LPMEMSD_ROM_POS                   13 /**< LPMEMSD_ROM Position */
#define MXC_F_PWRSEQ_LPMEMSD_ROM                       ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPMEMSD_ROM_POS)) /**< LPMEMSD_ROM Mask */

#define MXC_F_PWRSEQ_LPMEMSD_MAA_POS                   15 /**< LPMEMSD_MAA Position */
#define MXC_F_PWRSEQ_LPMEMSD_MAA                       ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPMEMSD_MAA_POS)) /**< LPMEMSD_MAA Mask */

/**@} end of group PWRSEQ_LPMEMSD_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32570_INCLUDE_PWRSEQ_REGS_H_
