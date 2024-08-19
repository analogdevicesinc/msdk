/**
 * @file    pwrseq_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the PWRSEQ Peripheral Module.
 * @note    This file is @generated.
 * @ingroup pwrseq_registers
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32650_INCLUDE_PWRSEQ_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32650_INCLUDE_PWRSEQ_REGS_H_

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
    __IO uint32_t ctrl;                 /**< <tt>\b 0x00:</tt> PWRSEQ CTRL Register */
    __IO uint32_t gpio0_wk_fl;          /**< <tt>\b 0x04:</tt> PWRSEQ GPIO0_WK_FL Register */
    __IO uint32_t gpio0_wk_en;          /**< <tt>\b 0x08:</tt> PWRSEQ GPIO0_WK_EN Register */
    __IO uint32_t gpio1_wk_fl;          /**< <tt>\b 0x0C:</tt> PWRSEQ GPIO1_WK_FL Register */
    __IO uint32_t gpio1_wk_en;          /**< <tt>\b 0x10:</tt> PWRSEQ GPIO1_WK_EN Register */
    __IO uint32_t gpio2_wk_fl;          /**< <tt>\b 0x14:</tt> PWRSEQ GPIO2_WK_FL Register */
    __IO uint32_t gpio2_wk_en;          /**< <tt>\b 0x18:</tt> PWRSEQ GPIO2_WK_EN Register */
    __IO uint32_t gpio3_wk_fl;          /**< <tt>\b 0x1C:</tt> PWRSEQ GPIO3_WK_FL Register */
    __IO uint32_t gpio3_wk_en;          /**< <tt>\b 0x20:</tt> PWRSEQ GPIO3_WK_EN Register */
    __R  uint32_t rsv_0x24_0x2f[3];
    __IO uint32_t usb_wk_fl;            /**< <tt>\b 0x30:</tt> PWRSEQ USB_WK_FL Register */
    __IO uint32_t usb_wk_en;            /**< <tt>\b 0x34:</tt> PWRSEQ USB_WK_EN Register */
    __R  uint32_t rsv_0x38_0x3f[2];
    __IO uint32_t mem_pwr;              /**< <tt>\b 0x40:</tt> PWRSEQ MEM_PWR Register */
} mxc_pwrseq_regs_t;

/* Register offsets for module PWRSEQ */
/**
 * @ingroup    pwrseq_registers
 * @defgroup   PWRSEQ_Register_Offsets Register Offsets
 * @brief      PWRSEQ Peripheral Register Offsets from the PWRSEQ Base Peripheral Address.
 * @{
 */
#define MXC_R_PWRSEQ_CTRL                  ((uint32_t)0x00000000UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0000</tt> */
#define MXC_R_PWRSEQ_GPIO0_WK_FL           ((uint32_t)0x00000004UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0004</tt> */
#define MXC_R_PWRSEQ_GPIO0_WK_EN           ((uint32_t)0x00000008UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0008</tt> */
#define MXC_R_PWRSEQ_GPIO1_WK_FL           ((uint32_t)0x0000000CUL) /**< Offset from PWRSEQ Base Address: <tt> 0x000C</tt> */
#define MXC_R_PWRSEQ_GPIO1_WK_EN           ((uint32_t)0x00000010UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0010</tt> */
#define MXC_R_PWRSEQ_GPIO2_WK_FL           ((uint32_t)0x00000014UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0014</tt> */
#define MXC_R_PWRSEQ_GPIO2_WK_EN           ((uint32_t)0x00000018UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0018</tt> */
#define MXC_R_PWRSEQ_GPIO3_WK_FL           ((uint32_t)0x0000001CUL) /**< Offset from PWRSEQ Base Address: <tt> 0x001C</tt> */
#define MXC_R_PWRSEQ_GPIO3_WK_EN           ((uint32_t)0x00000020UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0020</tt> */
#define MXC_R_PWRSEQ_USB_WK_FL             ((uint32_t)0x00000030UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0030</tt> */
#define MXC_R_PWRSEQ_USB_WK_EN             ((uint32_t)0x00000034UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0034</tt> */
#define MXC_R_PWRSEQ_MEM_PWR               ((uint32_t)0x00000040UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0040</tt> */
/**@} end of group pwrseq_registers */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_CTRL PWRSEQ_CTRL
 * @brief    Low Power Control Register.
 * @{
 */
#define MXC_F_PWRSEQ_CTRL_RAMRET_POS                   0 /**< CTRL_RAMRET Position */
#define MXC_F_PWRSEQ_CTRL_RAMRET                       ((uint32_t)(0x3UL << MXC_F_PWRSEQ_CTRL_RAMRET_POS)) /**< CTRL_RAMRET Mask */
#define MXC_V_PWRSEQ_CTRL_RAMRET_DIS                   ((uint32_t)0x0UL) /**< CTRL_RAMRET_DIS Value */
#define MXC_S_PWRSEQ_CTRL_RAMRET_DIS                   (MXC_V_PWRSEQ_CTRL_RAMRET_DIS << MXC_F_PWRSEQ_CTRL_RAMRET_POS) /**< CTRL_RAMRET_DIS Setting */
#define MXC_V_PWRSEQ_CTRL_RAMRET_EN1                   ((uint32_t)0x1UL) /**< CTRL_RAMRET_EN1 Value */
#define MXC_S_PWRSEQ_CTRL_RAMRET_EN1                   (MXC_V_PWRSEQ_CTRL_RAMRET_EN1 << MXC_F_PWRSEQ_CTRL_RAMRET_POS) /**< CTRL_RAMRET_EN1 Setting */
#define MXC_V_PWRSEQ_CTRL_RAMRET_EN2                   ((uint32_t)0x2UL) /**< CTRL_RAMRET_EN2 Value */
#define MXC_S_PWRSEQ_CTRL_RAMRET_EN2                   (MXC_V_PWRSEQ_CTRL_RAMRET_EN2 << MXC_F_PWRSEQ_CTRL_RAMRET_POS) /**< CTRL_RAMRET_EN2 Setting */
#define MXC_V_PWRSEQ_CTRL_RAMRET_EN3                   ((uint32_t)0x3UL) /**< CTRL_RAMRET_EN3 Value */
#define MXC_S_PWRSEQ_CTRL_RAMRET_EN3                   (MXC_V_PWRSEQ_CTRL_RAMRET_EN3 << MXC_F_PWRSEQ_CTRL_RAMRET_POS) /**< CTRL_RAMRET_EN3 Setting */

#define MXC_F_PWRSEQ_CTRL_RREGEN_POS                   8 /**< CTRL_RREGEN Position */
#define MXC_F_PWRSEQ_CTRL_RREGEN                       ((uint32_t)(0x1UL << MXC_F_PWRSEQ_CTRL_RREGEN_POS)) /**< CTRL_RREGEN Mask */
#define MXC_V_PWRSEQ_CTRL_RREGEN_DIS                   ((uint32_t)0x0UL) /**< CTRL_RREGEN_DIS Value */
#define MXC_S_PWRSEQ_CTRL_RREGEN_DIS                   (MXC_V_PWRSEQ_CTRL_RREGEN_DIS << MXC_F_PWRSEQ_CTRL_RREGEN_POS) /**< CTRL_RREGEN_DIS Setting */
#define MXC_V_PWRSEQ_CTRL_RREGEN_EN                    ((uint32_t)0x1UL) /**< CTRL_RREGEN_EN Value */
#define MXC_S_PWRSEQ_CTRL_RREGEN_EN                    (MXC_V_PWRSEQ_CTRL_RREGEN_EN << MXC_F_PWRSEQ_CTRL_RREGEN_POS) /**< CTRL_RREGEN_EN Setting */

#define MXC_F_PWRSEQ_CTRL_BKGRND_POS                   9 /**< CTRL_BKGRND Position */
#define MXC_F_PWRSEQ_CTRL_BKGRND                       ((uint32_t)(0x1UL << MXC_F_PWRSEQ_CTRL_BKGRND_POS)) /**< CTRL_BKGRND Mask */
#define MXC_V_PWRSEQ_CTRL_BKGRND_DIS                   ((uint32_t)0x0UL) /**< CTRL_BKGRND_DIS Value */
#define MXC_S_PWRSEQ_CTRL_BKGRND_DIS                   (MXC_V_PWRSEQ_CTRL_BKGRND_DIS << MXC_F_PWRSEQ_CTRL_BKGRND_POS) /**< CTRL_BKGRND_DIS Setting */
#define MXC_V_PWRSEQ_CTRL_BKGRND_EN                    ((uint32_t)0x1UL) /**< CTRL_BKGRND_EN Value */
#define MXC_S_PWRSEQ_CTRL_BKGRND_EN                    (MXC_V_PWRSEQ_CTRL_BKGRND_EN << MXC_F_PWRSEQ_CTRL_BKGRND_POS) /**< CTRL_BKGRND_EN Setting */

#define MXC_F_PWRSEQ_CTRL_FWKM_POS                     10 /**< CTRL_FWKM Position */
#define MXC_F_PWRSEQ_CTRL_FWKM                         ((uint32_t)(0x1UL << MXC_F_PWRSEQ_CTRL_FWKM_POS)) /**< CTRL_FWKM Mask */
#define MXC_V_PWRSEQ_CTRL_FWKM_DIS                     ((uint32_t)0x0UL) /**< CTRL_FWKM_DIS Value */
#define MXC_S_PWRSEQ_CTRL_FWKM_DIS                     (MXC_V_PWRSEQ_CTRL_FWKM_DIS << MXC_F_PWRSEQ_CTRL_FWKM_POS) /**< CTRL_FWKM_DIS Setting */
#define MXC_V_PWRSEQ_CTRL_FWKM_EN                      ((uint32_t)0x1UL) /**< CTRL_FWKM_EN Value */
#define MXC_S_PWRSEQ_CTRL_FWKM_EN                      (MXC_V_PWRSEQ_CTRL_FWKM_EN << MXC_F_PWRSEQ_CTRL_FWKM_POS) /**< CTRL_FWKM_EN Setting */

#define MXC_F_PWRSEQ_CTRL_BGOFF_POS                    11 /**< CTRL_BGOFF Position */
#define MXC_F_PWRSEQ_CTRL_BGOFF                        ((uint32_t)(0x1UL << MXC_F_PWRSEQ_CTRL_BGOFF_POS)) /**< CTRL_BGOFF Mask */
#define MXC_V_PWRSEQ_CTRL_BGOFF_ON                     ((uint32_t)0x0UL) /**< CTRL_BGOFF_ON Value */
#define MXC_S_PWRSEQ_CTRL_BGOFF_ON                     (MXC_V_PWRSEQ_CTRL_BGOFF_ON << MXC_F_PWRSEQ_CTRL_BGOFF_POS) /**< CTRL_BGOFF_ON Setting */
#define MXC_V_PWRSEQ_CTRL_BGOFF_OFF                    ((uint32_t)0x1UL) /**< CTRL_BGOFF_OFF Value */
#define MXC_S_PWRSEQ_CTRL_BGOFF_OFF                    (MXC_V_PWRSEQ_CTRL_BGOFF_OFF << MXC_F_PWRSEQ_CTRL_BGOFF_POS) /**< CTRL_BGOFF_OFF Setting */

#define MXC_F_PWRSEQ_CTRL_PORVCOREMD_POS               12 /**< CTRL_PORVCOREMD Position */
#define MXC_F_PWRSEQ_CTRL_PORVCOREMD                   ((uint32_t)(0x1UL << MXC_F_PWRSEQ_CTRL_PORVCOREMD_POS)) /**< CTRL_PORVCOREMD Mask */
#define MXC_V_PWRSEQ_CTRL_PORVCOREMD_EN                ((uint32_t)0x0UL) /**< CTRL_PORVCOREMD_EN Value */
#define MXC_S_PWRSEQ_CTRL_PORVCOREMD_EN                (MXC_V_PWRSEQ_CTRL_PORVCOREMD_EN << MXC_F_PWRSEQ_CTRL_PORVCOREMD_POS) /**< CTRL_PORVCOREMD_EN Setting */
#define MXC_V_PWRSEQ_CTRL_PORVCOREMD_DIS               ((uint32_t)0x1UL) /**< CTRL_PORVCOREMD_DIS Value */
#define MXC_S_PWRSEQ_CTRL_PORVCOREMD_DIS               (MXC_V_PWRSEQ_CTRL_PORVCOREMD_DIS << MXC_F_PWRSEQ_CTRL_PORVCOREMD_POS) /**< CTRL_PORVCOREMD_DIS Setting */

#define MXC_F_PWRSEQ_CTRL_VCOREMD_POS                  20 /**< CTRL_VCOREMD Position */
#define MXC_F_PWRSEQ_CTRL_VCOREMD                      ((uint32_t)(0x1UL << MXC_F_PWRSEQ_CTRL_VCOREMD_POS)) /**< CTRL_VCOREMD Mask */
#define MXC_V_PWRSEQ_CTRL_VCOREMD_EN                   ((uint32_t)0x0UL) /**< CTRL_VCOREMD_EN Value */
#define MXC_S_PWRSEQ_CTRL_VCOREMD_EN                   (MXC_V_PWRSEQ_CTRL_VCOREMD_EN << MXC_F_PWRSEQ_CTRL_VCOREMD_POS) /**< CTRL_VCOREMD_EN Setting */
#define MXC_V_PWRSEQ_CTRL_VCOREMD_DIS                  ((uint32_t)0x1UL) /**< CTRL_VCOREMD_DIS Value */
#define MXC_S_PWRSEQ_CTRL_VCOREMD_DIS                  (MXC_V_PWRSEQ_CTRL_VCOREMD_DIS << MXC_F_PWRSEQ_CTRL_VCOREMD_POS) /**< CTRL_VCOREMD_DIS Setting */

#define MXC_F_PWRSEQ_CTRL_VRTCMD_POS                   21 /**< CTRL_VRTCMD Position */
#define MXC_F_PWRSEQ_CTRL_VRTCMD                       ((uint32_t)(0x1UL << MXC_F_PWRSEQ_CTRL_VRTCMD_POS)) /**< CTRL_VRTCMD Mask */
#define MXC_V_PWRSEQ_CTRL_VRTCMD_EN                    ((uint32_t)0x0UL) /**< CTRL_VRTCMD_EN Value */
#define MXC_S_PWRSEQ_CTRL_VRTCMD_EN                    (MXC_V_PWRSEQ_CTRL_VRTCMD_EN << MXC_F_PWRSEQ_CTRL_VRTCMD_POS) /**< CTRL_VRTCMD_EN Setting */
#define MXC_V_PWRSEQ_CTRL_VRTCMD_DIS                   ((uint32_t)0x1UL) /**< CTRL_VRTCMD_DIS Value */
#define MXC_S_PWRSEQ_CTRL_VRTCMD_DIS                   (MXC_V_PWRSEQ_CTRL_VRTCMD_DIS << MXC_F_PWRSEQ_CTRL_VRTCMD_POS) /**< CTRL_VRTCMD_DIS Setting */

#define MXC_F_PWRSEQ_CTRL_VDDAMD_POS                   22 /**< CTRL_VDDAMD Position */
#define MXC_F_PWRSEQ_CTRL_VDDAMD                       ((uint32_t)(0x1UL << MXC_F_PWRSEQ_CTRL_VDDAMD_POS)) /**< CTRL_VDDAMD Mask */
#define MXC_V_PWRSEQ_CTRL_VDDAMD_EN                    ((uint32_t)0x0UL) /**< CTRL_VDDAMD_EN Value */
#define MXC_S_PWRSEQ_CTRL_VDDAMD_EN                    (MXC_V_PWRSEQ_CTRL_VDDAMD_EN << MXC_F_PWRSEQ_CTRL_VDDAMD_POS) /**< CTRL_VDDAMD_EN Setting */
#define MXC_V_PWRSEQ_CTRL_VDDAMD_DIS                   ((uint32_t)0x1UL) /**< CTRL_VDDAMD_DIS Value */
#define MXC_S_PWRSEQ_CTRL_VDDAMD_DIS                   (MXC_V_PWRSEQ_CTRL_VDDAMD_DIS << MXC_F_PWRSEQ_CTRL_VDDAMD_POS) /**< CTRL_VDDAMD_DIS Setting */

#define MXC_F_PWRSEQ_CTRL_VDDIOMD_POS                  23 /**< CTRL_VDDIOMD Position */
#define MXC_F_PWRSEQ_CTRL_VDDIOMD                      ((uint32_t)(0x1UL << MXC_F_PWRSEQ_CTRL_VDDIOMD_POS)) /**< CTRL_VDDIOMD Mask */
#define MXC_V_PWRSEQ_CTRL_VDDIOMD_EN                   ((uint32_t)0x0UL) /**< CTRL_VDDIOMD_EN Value */
#define MXC_S_PWRSEQ_CTRL_VDDIOMD_EN                   (MXC_V_PWRSEQ_CTRL_VDDIOMD_EN << MXC_F_PWRSEQ_CTRL_VDDIOMD_POS) /**< CTRL_VDDIOMD_EN Setting */
#define MXC_V_PWRSEQ_CTRL_VDDIOMD_DIS                  ((uint32_t)0x1UL) /**< CTRL_VDDIOMD_DIS Value */
#define MXC_S_PWRSEQ_CTRL_VDDIOMD_DIS                  (MXC_V_PWRSEQ_CTRL_VDDIOMD_DIS << MXC_F_PWRSEQ_CTRL_VDDIOMD_POS) /**< CTRL_VDDIOMD_DIS Setting */

#define MXC_F_PWRSEQ_CTRL_VDDIOHMD_POS                 24 /**< CTRL_VDDIOHMD Position */
#define MXC_F_PWRSEQ_CTRL_VDDIOHMD                     ((uint32_t)(0x1UL << MXC_F_PWRSEQ_CTRL_VDDIOHMD_POS)) /**< CTRL_VDDIOHMD Mask */
#define MXC_V_PWRSEQ_CTRL_VDDIOHMD_EN                  ((uint32_t)0x0UL) /**< CTRL_VDDIOHMD_EN Value */
#define MXC_S_PWRSEQ_CTRL_VDDIOHMD_EN                  (MXC_V_PWRSEQ_CTRL_VDDIOHMD_EN << MXC_F_PWRSEQ_CTRL_VDDIOHMD_POS) /**< CTRL_VDDIOHMD_EN Setting */
#define MXC_V_PWRSEQ_CTRL_VDDIOHMD_DIS                 ((uint32_t)0x1UL) /**< CTRL_VDDIOHMD_DIS Value */
#define MXC_S_PWRSEQ_CTRL_VDDIOHMD_DIS                 (MXC_V_PWRSEQ_CTRL_VDDIOHMD_DIS << MXC_F_PWRSEQ_CTRL_VDDIOHMD_POS) /**< CTRL_VDDIOHMD_DIS Setting */

#define MXC_F_PWRSEQ_CTRL_PORVDDIOMD_POS               25 /**< CTRL_PORVDDIOMD Position */
#define MXC_F_PWRSEQ_CTRL_PORVDDIOMD                   ((uint32_t)(0x1UL << MXC_F_PWRSEQ_CTRL_PORVDDIOMD_POS)) /**< CTRL_PORVDDIOMD Mask */
#define MXC_V_PWRSEQ_CTRL_PORVDDIOMD_EN                ((uint32_t)0x0UL) /**< CTRL_PORVDDIOMD_EN Value */
#define MXC_S_PWRSEQ_CTRL_PORVDDIOMD_EN                (MXC_V_PWRSEQ_CTRL_PORVDDIOMD_EN << MXC_F_PWRSEQ_CTRL_PORVDDIOMD_POS) /**< CTRL_PORVDDIOMD_EN Setting */
#define MXC_V_PWRSEQ_CTRL_PORVDDIOMD_DIS               ((uint32_t)0x1UL) /**< CTRL_PORVDDIOMD_DIS Value */
#define MXC_S_PWRSEQ_CTRL_PORVDDIOMD_DIS               (MXC_V_PWRSEQ_CTRL_PORVDDIOMD_DIS << MXC_F_PWRSEQ_CTRL_PORVDDIOMD_POS) /**< CTRL_PORVDDIOMD_DIS Setting */

#define MXC_F_PWRSEQ_CTRL_PORVDDIOHMD_POS              26 /**< CTRL_PORVDDIOHMD Position */
#define MXC_F_PWRSEQ_CTRL_PORVDDIOHMD                  ((uint32_t)(0x1UL << MXC_F_PWRSEQ_CTRL_PORVDDIOHMD_POS)) /**< CTRL_PORVDDIOHMD Mask */
#define MXC_V_PWRSEQ_CTRL_PORVDDIOHMD_EN               ((uint32_t)0x0UL) /**< CTRL_PORVDDIOHMD_EN Value */
#define MXC_S_PWRSEQ_CTRL_PORVDDIOHMD_EN               (MXC_V_PWRSEQ_CTRL_PORVDDIOHMD_EN << MXC_F_PWRSEQ_CTRL_PORVDDIOHMD_POS) /**< CTRL_PORVDDIOHMD_EN Setting */
#define MXC_V_PWRSEQ_CTRL_PORVDDIOHMD_DIS              ((uint32_t)0x1UL) /**< CTRL_PORVDDIOHMD_DIS Value */
#define MXC_S_PWRSEQ_CTRL_PORVDDIOHMD_DIS              (MXC_V_PWRSEQ_CTRL_PORVDDIOHMD_DIS << MXC_F_PWRSEQ_CTRL_PORVDDIOHMD_POS) /**< CTRL_PORVDDIOHMD_DIS Setting */

#define MXC_F_PWRSEQ_CTRL_VDDBMD_POS                   27 /**< CTRL_VDDBMD Position */
#define MXC_F_PWRSEQ_CTRL_VDDBMD                       ((uint32_t)(0x1UL << MXC_F_PWRSEQ_CTRL_VDDBMD_POS)) /**< CTRL_VDDBMD Mask */
#define MXC_V_PWRSEQ_CTRL_VDDBMD_EN                    ((uint32_t)0x0UL) /**< CTRL_VDDBMD_EN Value */
#define MXC_S_PWRSEQ_CTRL_VDDBMD_EN                    (MXC_V_PWRSEQ_CTRL_VDDBMD_EN << MXC_F_PWRSEQ_CTRL_VDDBMD_POS) /**< CTRL_VDDBMD_EN Setting */
#define MXC_V_PWRSEQ_CTRL_VDDBMD_DIS                   ((uint32_t)0x1UL) /**< CTRL_VDDBMD_DIS Value */
#define MXC_S_PWRSEQ_CTRL_VDDBMD_DIS                   (MXC_V_PWRSEQ_CTRL_VDDBMD_DIS << MXC_F_PWRSEQ_CTRL_VDDBMD_POS) /**< CTRL_VDDBMD_DIS Setting */

/**@} end of group PWRSEQ_CTRL_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_GPIO0_WK_FL PWRSEQ_GPIO0_WK_FL
 * @brief    Low Power I/O Wakeup Status Register 0. This register indicates the low power
 *           wakeup status for GPIO0.
 * @{
 */
#define MXC_F_PWRSEQ_GPIO0_WK_FL_WAKEST_POS            0 /**< GPIO0_WK_FL_WAKEST Position */
#define MXC_F_PWRSEQ_GPIO0_WK_FL_WAKEST                ((uint32_t)(0xFFFFFFFFUL << MXC_F_PWRSEQ_GPIO0_WK_FL_WAKEST_POS)) /**< GPIO0_WK_FL_WAKEST Mask */

/**@} end of group PWRSEQ_GPIO0_WK_FL_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_GPIO0_WK_EN PWRSEQ_GPIO0_WK_EN
 * @brief    Low Power I/O Wakeup Enable Register 0. This register enables low power wakeup
 *           functionality for GPIO0.
 * @{
 */
#define MXC_F_PWRSEQ_GPIO0_WK_EN_WAKEEN_POS            0 /**< GPIO0_WK_EN_WAKEEN Position */
#define MXC_F_PWRSEQ_GPIO0_WK_EN_WAKEEN                ((uint32_t)(0xFFFFFFFFUL << MXC_F_PWRSEQ_GPIO0_WK_EN_WAKEEN_POS)) /**< GPIO0_WK_EN_WAKEEN Mask */

/**@} end of group PWRSEQ_GPIO0_WK_EN_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_GPIO1_WK_FL PWRSEQ_GPIO1_WK_FL
 * @brief    Low Power I/O Wakeup Status Register 1. This register indicates the low power
 *           wakeup status for GPIO1.
 * @{
 */
#define MXC_F_PWRSEQ_GPIO1_WK_FL_WAKEST_POS            0 /**< GPIO1_WK_FL_WAKEST Position */
#define MXC_F_PWRSEQ_GPIO1_WK_FL_WAKEST                ((uint32_t)(0xFFFFFFFFUL << MXC_F_PWRSEQ_GPIO1_WK_FL_WAKEST_POS)) /**< GPIO1_WK_FL_WAKEST Mask */

/**@} end of group PWRSEQ_GPIO1_WK_FL_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_GPIO1_WK_EN PWRSEQ_GPIO1_WK_EN
 * @brief    Low Power I/O Wakeup Enable Register 1. This register enables low power wakeup
 *           functionality for GPIO1.
 * @{
 */
#define MXC_F_PWRSEQ_GPIO1_WK_EN_WAKEEN_POS            0 /**< GPIO1_WK_EN_WAKEEN Position */
#define MXC_F_PWRSEQ_GPIO1_WK_EN_WAKEEN                ((uint32_t)(0xFFFFFFFFUL << MXC_F_PWRSEQ_GPIO1_WK_EN_WAKEEN_POS)) /**< GPIO1_WK_EN_WAKEEN Mask */

/**@} end of group PWRSEQ_GPIO1_WK_EN_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_GPIO2_WK_FL PWRSEQ_GPIO2_WK_FL
 * @brief    Low Power I/O Wakeup Status Register 0. This register indicates the low power
 *           wakeup status for GPIO2.
 * @{
 */
#define MXC_F_PWRSEQ_GPIO2_WK_FL_WAKEST_POS            0 /**< GPIO2_WK_FL_WAKEST Position */
#define MXC_F_PWRSEQ_GPIO2_WK_FL_WAKEST                ((uint32_t)(0xFFFFFFFFUL << MXC_F_PWRSEQ_GPIO2_WK_FL_WAKEST_POS)) /**< GPIO2_WK_FL_WAKEST Mask */

/**@} end of group PWRSEQ_GPIO2_WK_FL_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_GPIO2_WK_EN PWRSEQ_GPIO2_WK_EN
 * @brief    Low Power I/O Wakeup Enable Register 0. This register enables low power wakeup
 *           functionality for GPIO0.
 * @{
 */
#define MXC_F_PWRSEQ_GPIO2_WK_EN_WAKEEN_POS            0 /**< GPIO2_WK_EN_WAKEEN Position */
#define MXC_F_PWRSEQ_GPIO2_WK_EN_WAKEEN                ((uint32_t)(0xFFFFFFFFUL << MXC_F_PWRSEQ_GPIO2_WK_EN_WAKEEN_POS)) /**< GPIO2_WK_EN_WAKEEN Mask */

/**@} end of group PWRSEQ_GPIO2_WK_EN_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_GPIO3_WK_FL PWRSEQ_GPIO3_WK_FL
 * @brief    Low Power I/O Wakeup Status Register 0. This register indicates the low power
 *           wakeup status for GPIO3.
 * @{
 */
#define MXC_F_PWRSEQ_GPIO3_WK_FL_WAKEST_POS            0 /**< GPIO3_WK_FL_WAKEST Position */
#define MXC_F_PWRSEQ_GPIO3_WK_FL_WAKEST                ((uint32_t)(0x3FFUL << MXC_F_PWRSEQ_GPIO3_WK_FL_WAKEST_POS)) /**< GPIO3_WK_FL_WAKEST Mask */

/**@} end of group PWRSEQ_GPIO3_WK_FL_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_GPIO3_WK_EN PWRSEQ_GPIO3_WK_EN
 * @brief    Low Power I/O Wakeup Enable Register 0. This register enables low power wakeup
 *           functionality for GPIO3.
 * @{
 */
#define MXC_F_PWRSEQ_GPIO3_WK_EN_WAKEEN_POS            0 /**< GPIO3_WK_EN_WAKEEN Position */
#define MXC_F_PWRSEQ_GPIO3_WK_EN_WAKEEN                ((uint32_t)(0x3FFUL << MXC_F_PWRSEQ_GPIO3_WK_EN_WAKEEN_POS)) /**< GPIO3_WK_EN_WAKEEN Mask */

/**@} end of group PWRSEQ_GPIO3_WK_EN_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_USB_WK_FL PWRSEQ_USB_WK_FL
 * @brief    Low Power Peripheral Wakeup Status Register.
 * @{
 */
#define MXC_F_PWRSEQ_USB_WK_FL_USBLSWKST_POS           0 /**< USB_WK_FL_USBLSWKST Position */
#define MXC_F_PWRSEQ_USB_WK_FL_USBLSWKST               ((uint32_t)(0x3UL << MXC_F_PWRSEQ_USB_WK_FL_USBLSWKST_POS)) /**< USB_WK_FL_USBLSWKST Mask */
#define MXC_V_PWRSEQ_USB_WK_FL_USBLSWKST_DPLUS         ((uint32_t)0x0UL) /**< USB_WK_FL_USBLSWKST_DPLUS Value */
#define MXC_S_PWRSEQ_USB_WK_FL_USBLSWKST_DPLUS         (MXC_V_PWRSEQ_USB_WK_FL_USBLSWKST_DPLUS << MXC_F_PWRSEQ_USB_WK_FL_USBLSWKST_POS) /**< USB_WK_FL_USBLSWKST_DPLUS Setting */
#define MXC_V_PWRSEQ_USB_WK_FL_USBLSWKST_DMINUS        ((uint32_t)0x1UL) /**< USB_WK_FL_USBLSWKST_DMINUS Value */
#define MXC_S_PWRSEQ_USB_WK_FL_USBLSWKST_DMINUS        (MXC_V_PWRSEQ_USB_WK_FL_USBLSWKST_DMINUS << MXC_F_PWRSEQ_USB_WK_FL_USBLSWKST_POS) /**< USB_WK_FL_USBLSWKST_DMINUS Setting */

#define MXC_F_PWRSEQ_USB_WK_FL_USBVBUSWKST_POS         2 /**< USB_WK_FL_USBVBUSWKST Position */
#define MXC_F_PWRSEQ_USB_WK_FL_USBVBUSWKST             ((uint32_t)(0x1UL << MXC_F_PWRSEQ_USB_WK_FL_USBVBUSWKST_POS)) /**< USB_WK_FL_USBVBUSWKST Mask */
#define MXC_V_PWRSEQ_USB_WK_FL_USBVBUSWKST_NORMAL      ((uint32_t)0x0UL) /**< USB_WK_FL_USBVBUSWKST_NORMAL Value */
#define MXC_S_PWRSEQ_USB_WK_FL_USBVBUSWKST_NORMAL      (MXC_V_PWRSEQ_USB_WK_FL_USBVBUSWKST_NORMAL << MXC_F_PWRSEQ_USB_WK_FL_USBVBUSWKST_POS) /**< USB_WK_FL_USBVBUSWKST_NORMAL Setting */
#define MXC_V_PWRSEQ_USB_WK_FL_USBVBUSWKST_STCHNG      ((uint32_t)0x1UL) /**< USB_WK_FL_USBVBUSWKST_STCHNG Value */
#define MXC_S_PWRSEQ_USB_WK_FL_USBVBUSWKST_STCHNG      (MXC_V_PWRSEQ_USB_WK_FL_USBVBUSWKST_STCHNG << MXC_F_PWRSEQ_USB_WK_FL_USBVBUSWKST_POS) /**< USB_WK_FL_USBVBUSWKST_STCHNG Setting */

/**@} end of group PWRSEQ_USB_WK_FL_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_USB_WK_EN PWRSEQ_USB_WK_EN
 * @brief    Low Power Peripheral Wakeup Enable Register.
 * @{
 */
#define MXC_F_PWRSEQ_USB_WK_EN_USBLSWKEN_POS           0 /**< USB_WK_EN_USBLSWKEN Position */
#define MXC_F_PWRSEQ_USB_WK_EN_USBLSWKEN               ((uint32_t)(0x3UL << MXC_F_PWRSEQ_USB_WK_EN_USBLSWKEN_POS)) /**< USB_WK_EN_USBLSWKEN Mask */
#define MXC_V_PWRSEQ_USB_WK_EN_USBLSWKEN_DIS           ((uint32_t)0x0UL) /**< USB_WK_EN_USBLSWKEN_DIS Value */
#define MXC_S_PWRSEQ_USB_WK_EN_USBLSWKEN_DIS           (MXC_V_PWRSEQ_USB_WK_EN_USBLSWKEN_DIS << MXC_F_PWRSEQ_USB_WK_EN_USBLSWKEN_POS) /**< USB_WK_EN_USBLSWKEN_DIS Setting */
#define MXC_V_PWRSEQ_USB_WK_EN_USBLSWKEN_EN            ((uint32_t)0x3UL) /**< USB_WK_EN_USBLSWKEN_EN Value */
#define MXC_S_PWRSEQ_USB_WK_EN_USBLSWKEN_EN            (MXC_V_PWRSEQ_USB_WK_EN_USBLSWKEN_EN << MXC_F_PWRSEQ_USB_WK_EN_USBLSWKEN_POS) /**< USB_WK_EN_USBLSWKEN_EN Setting */

#define MXC_F_PWRSEQ_USB_WK_EN_USBVBUSWKEN_POS         2 /**< USB_WK_EN_USBVBUSWKEN Position */
#define MXC_F_PWRSEQ_USB_WK_EN_USBVBUSWKEN             ((uint32_t)(0x1UL << MXC_F_PWRSEQ_USB_WK_EN_USBVBUSWKEN_POS)) /**< USB_WK_EN_USBVBUSWKEN Mask */
#define MXC_V_PWRSEQ_USB_WK_EN_USBVBUSWKEN_DIS         ((uint32_t)0x0UL) /**< USB_WK_EN_USBVBUSWKEN_DIS Value */
#define MXC_S_PWRSEQ_USB_WK_EN_USBVBUSWKEN_DIS         (MXC_V_PWRSEQ_USB_WK_EN_USBVBUSWKEN_DIS << MXC_F_PWRSEQ_USB_WK_EN_USBVBUSWKEN_POS) /**< USB_WK_EN_USBVBUSWKEN_DIS Setting */
#define MXC_V_PWRSEQ_USB_WK_EN_USBVBUSWKEN_EN          ((uint32_t)0x1UL) /**< USB_WK_EN_USBVBUSWKEN_EN Value */
#define MXC_S_PWRSEQ_USB_WK_EN_USBVBUSWKEN_EN          (MXC_V_PWRSEQ_USB_WK_EN_USBVBUSWKEN_EN << MXC_F_PWRSEQ_USB_WK_EN_USBVBUSWKEN_POS) /**< USB_WK_EN_USBVBUSWKEN_EN Setting */

/**@} end of group PWRSEQ_USB_WK_EN_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_MEM_PWR PWRSEQ_MEM_PWR
 * @brief    Low Power Memory Shutdown Control.
 * @{
 */
#define MXC_F_PWRSEQ_MEM_PWR_SRAM0SD_POS               0 /**< MEM_PWR_SRAM0SD Position */
#define MXC_F_PWRSEQ_MEM_PWR_SRAM0SD                   ((uint32_t)(0x1UL << MXC_F_PWRSEQ_MEM_PWR_SRAM0SD_POS)) /**< MEM_PWR_SRAM0SD Mask */
#define MXC_V_PWRSEQ_MEM_PWR_SRAM0SD_NORMAL            ((uint32_t)0x0UL) /**< MEM_PWR_SRAM0SD_NORMAL Value */
#define MXC_S_PWRSEQ_MEM_PWR_SRAM0SD_NORMAL            (MXC_V_PWRSEQ_MEM_PWR_SRAM0SD_NORMAL << MXC_F_PWRSEQ_MEM_PWR_SRAM0SD_POS) /**< MEM_PWR_SRAM0SD_NORMAL Setting */
#define MXC_V_PWRSEQ_MEM_PWR_SRAM0SD_SHUTDOWN          ((uint32_t)0x1UL) /**< MEM_PWR_SRAM0SD_SHUTDOWN Value */
#define MXC_S_PWRSEQ_MEM_PWR_SRAM0SD_SHUTDOWN          (MXC_V_PWRSEQ_MEM_PWR_SRAM0SD_SHUTDOWN << MXC_F_PWRSEQ_MEM_PWR_SRAM0SD_POS) /**< MEM_PWR_SRAM0SD_SHUTDOWN Setting */

#define MXC_F_PWRSEQ_MEM_PWR_SRAM1SD_POS               1 /**< MEM_PWR_SRAM1SD Position */
#define MXC_F_PWRSEQ_MEM_PWR_SRAM1SD                   ((uint32_t)(0x1UL << MXC_F_PWRSEQ_MEM_PWR_SRAM1SD_POS)) /**< MEM_PWR_SRAM1SD Mask */
#define MXC_V_PWRSEQ_MEM_PWR_SRAM1SD_NORMAL            ((uint32_t)0x0UL) /**< MEM_PWR_SRAM1SD_NORMAL Value */
#define MXC_S_PWRSEQ_MEM_PWR_SRAM1SD_NORMAL            (MXC_V_PWRSEQ_MEM_PWR_SRAM1SD_NORMAL << MXC_F_PWRSEQ_MEM_PWR_SRAM1SD_POS) /**< MEM_PWR_SRAM1SD_NORMAL Setting */
#define MXC_V_PWRSEQ_MEM_PWR_SRAM1SD_SHUTDOWN          ((uint32_t)0x1UL) /**< MEM_PWR_SRAM1SD_SHUTDOWN Value */
#define MXC_S_PWRSEQ_MEM_PWR_SRAM1SD_SHUTDOWN          (MXC_V_PWRSEQ_MEM_PWR_SRAM1SD_SHUTDOWN << MXC_F_PWRSEQ_MEM_PWR_SRAM1SD_POS) /**< MEM_PWR_SRAM1SD_SHUTDOWN Setting */

#define MXC_F_PWRSEQ_MEM_PWR_SRAM2SD_POS               2 /**< MEM_PWR_SRAM2SD Position */
#define MXC_F_PWRSEQ_MEM_PWR_SRAM2SD                   ((uint32_t)(0x1UL << MXC_F_PWRSEQ_MEM_PWR_SRAM2SD_POS)) /**< MEM_PWR_SRAM2SD Mask */
#define MXC_V_PWRSEQ_MEM_PWR_SRAM2SD_NORMAL            ((uint32_t)0x0UL) /**< MEM_PWR_SRAM2SD_NORMAL Value */
#define MXC_S_PWRSEQ_MEM_PWR_SRAM2SD_NORMAL            (MXC_V_PWRSEQ_MEM_PWR_SRAM2SD_NORMAL << MXC_F_PWRSEQ_MEM_PWR_SRAM2SD_POS) /**< MEM_PWR_SRAM2SD_NORMAL Setting */
#define MXC_V_PWRSEQ_MEM_PWR_SRAM2SD_SHUTDOWN          ((uint32_t)0x1UL) /**< MEM_PWR_SRAM2SD_SHUTDOWN Value */
#define MXC_S_PWRSEQ_MEM_PWR_SRAM2SD_SHUTDOWN          (MXC_V_PWRSEQ_MEM_PWR_SRAM2SD_SHUTDOWN << MXC_F_PWRSEQ_MEM_PWR_SRAM2SD_POS) /**< MEM_PWR_SRAM2SD_SHUTDOWN Setting */

#define MXC_F_PWRSEQ_MEM_PWR_SRAM3SD_POS               3 /**< MEM_PWR_SRAM3SD Position */
#define MXC_F_PWRSEQ_MEM_PWR_SRAM3SD                   ((uint32_t)(0x1UL << MXC_F_PWRSEQ_MEM_PWR_SRAM3SD_POS)) /**< MEM_PWR_SRAM3SD Mask */
#define MXC_V_PWRSEQ_MEM_PWR_SRAM3SD_NORMAL            ((uint32_t)0x0UL) /**< MEM_PWR_SRAM3SD_NORMAL Value */
#define MXC_S_PWRSEQ_MEM_PWR_SRAM3SD_NORMAL            (MXC_V_PWRSEQ_MEM_PWR_SRAM3SD_NORMAL << MXC_F_PWRSEQ_MEM_PWR_SRAM3SD_POS) /**< MEM_PWR_SRAM3SD_NORMAL Setting */
#define MXC_V_PWRSEQ_MEM_PWR_SRAM3SD_SHUTDOWN          ((uint32_t)0x1UL) /**< MEM_PWR_SRAM3SD_SHUTDOWN Value */
#define MXC_S_PWRSEQ_MEM_PWR_SRAM3SD_SHUTDOWN          (MXC_V_PWRSEQ_MEM_PWR_SRAM3SD_SHUTDOWN << MXC_F_PWRSEQ_MEM_PWR_SRAM3SD_POS) /**< MEM_PWR_SRAM3SD_SHUTDOWN Setting */

#define MXC_F_PWRSEQ_MEM_PWR_SRAM4SD_POS               4 /**< MEM_PWR_SRAM4SD Position */
#define MXC_F_PWRSEQ_MEM_PWR_SRAM4SD                   ((uint32_t)(0x1UL << MXC_F_PWRSEQ_MEM_PWR_SRAM4SD_POS)) /**< MEM_PWR_SRAM4SD Mask */
#define MXC_V_PWRSEQ_MEM_PWR_SRAM4SD_NORMAL            ((uint32_t)0x0UL) /**< MEM_PWR_SRAM4SD_NORMAL Value */
#define MXC_S_PWRSEQ_MEM_PWR_SRAM4SD_NORMAL            (MXC_V_PWRSEQ_MEM_PWR_SRAM4SD_NORMAL << MXC_F_PWRSEQ_MEM_PWR_SRAM4SD_POS) /**< MEM_PWR_SRAM4SD_NORMAL Setting */
#define MXC_V_PWRSEQ_MEM_PWR_SRAM4SD_SHUTDOWN          ((uint32_t)0x1UL) /**< MEM_PWR_SRAM4SD_SHUTDOWN Value */
#define MXC_S_PWRSEQ_MEM_PWR_SRAM4SD_SHUTDOWN          (MXC_V_PWRSEQ_MEM_PWR_SRAM4SD_SHUTDOWN << MXC_F_PWRSEQ_MEM_PWR_SRAM4SD_POS) /**< MEM_PWR_SRAM4SD_SHUTDOWN Setting */

#define MXC_F_PWRSEQ_MEM_PWR_SRAM5SD_POS               5 /**< MEM_PWR_SRAM5SD Position */
#define MXC_F_PWRSEQ_MEM_PWR_SRAM5SD                   ((uint32_t)(0x1UL << MXC_F_PWRSEQ_MEM_PWR_SRAM5SD_POS)) /**< MEM_PWR_SRAM5SD Mask */
#define MXC_V_PWRSEQ_MEM_PWR_SRAM5SD_NORMAL            ((uint32_t)0x0UL) /**< MEM_PWR_SRAM5SD_NORMAL Value */
#define MXC_S_PWRSEQ_MEM_PWR_SRAM5SD_NORMAL            (MXC_V_PWRSEQ_MEM_PWR_SRAM5SD_NORMAL << MXC_F_PWRSEQ_MEM_PWR_SRAM5SD_POS) /**< MEM_PWR_SRAM5SD_NORMAL Setting */
#define MXC_V_PWRSEQ_MEM_PWR_SRAM5SD_SHUTDOWN          ((uint32_t)0x1UL) /**< MEM_PWR_SRAM5SD_SHUTDOWN Value */
#define MXC_S_PWRSEQ_MEM_PWR_SRAM5SD_SHUTDOWN          (MXC_V_PWRSEQ_MEM_PWR_SRAM5SD_SHUTDOWN << MXC_F_PWRSEQ_MEM_PWR_SRAM5SD_POS) /**< MEM_PWR_SRAM5SD_SHUTDOWN Setting */

#define MXC_F_PWRSEQ_MEM_PWR_SRAM6SD_POS               6 /**< MEM_PWR_SRAM6SD Position */
#define MXC_F_PWRSEQ_MEM_PWR_SRAM6SD                   ((uint32_t)(0x1UL << MXC_F_PWRSEQ_MEM_PWR_SRAM6SD_POS)) /**< MEM_PWR_SRAM6SD Mask */
#define MXC_V_PWRSEQ_MEM_PWR_SRAM6SD_NORMAL            ((uint32_t)0x0UL) /**< MEM_PWR_SRAM6SD_NORMAL Value */
#define MXC_S_PWRSEQ_MEM_PWR_SRAM6SD_NORMAL            (MXC_V_PWRSEQ_MEM_PWR_SRAM6SD_NORMAL << MXC_F_PWRSEQ_MEM_PWR_SRAM6SD_POS) /**< MEM_PWR_SRAM6SD_NORMAL Setting */
#define MXC_V_PWRSEQ_MEM_PWR_SRAM6SD_SHUTDOWN          ((uint32_t)0x1UL) /**< MEM_PWR_SRAM6SD_SHUTDOWN Value */
#define MXC_S_PWRSEQ_MEM_PWR_SRAM6SD_SHUTDOWN          (MXC_V_PWRSEQ_MEM_PWR_SRAM6SD_SHUTDOWN << MXC_F_PWRSEQ_MEM_PWR_SRAM6SD_POS) /**< MEM_PWR_SRAM6SD_SHUTDOWN Setting */

#define MXC_F_PWRSEQ_MEM_PWR_ICACHESD_POS              7 /**< MEM_PWR_ICACHESD Position */
#define MXC_F_PWRSEQ_MEM_PWR_ICACHESD                  ((uint32_t)(0x1UL << MXC_F_PWRSEQ_MEM_PWR_ICACHESD_POS)) /**< MEM_PWR_ICACHESD Mask */
#define MXC_V_PWRSEQ_MEM_PWR_ICACHESD_NORMAL           ((uint32_t)0x0UL) /**< MEM_PWR_ICACHESD_NORMAL Value */
#define MXC_S_PWRSEQ_MEM_PWR_ICACHESD_NORMAL           (MXC_V_PWRSEQ_MEM_PWR_ICACHESD_NORMAL << MXC_F_PWRSEQ_MEM_PWR_ICACHESD_POS) /**< MEM_PWR_ICACHESD_NORMAL Setting */
#define MXC_V_PWRSEQ_MEM_PWR_ICACHESD_SHUTDOWN         ((uint32_t)0x1UL) /**< MEM_PWR_ICACHESD_SHUTDOWN Value */
#define MXC_S_PWRSEQ_MEM_PWR_ICACHESD_SHUTDOWN         (MXC_V_PWRSEQ_MEM_PWR_ICACHESD_SHUTDOWN << MXC_F_PWRSEQ_MEM_PWR_ICACHESD_POS) /**< MEM_PWR_ICACHESD_SHUTDOWN Setting */

#define MXC_F_PWRSEQ_MEM_PWR_ICACHEXIPSD_POS           8 /**< MEM_PWR_ICACHEXIPSD Position */
#define MXC_F_PWRSEQ_MEM_PWR_ICACHEXIPSD               ((uint32_t)(0x1UL << MXC_F_PWRSEQ_MEM_PWR_ICACHEXIPSD_POS)) /**< MEM_PWR_ICACHEXIPSD Mask */
#define MXC_V_PWRSEQ_MEM_PWR_ICACHEXIPSD_NORMAL        ((uint32_t)0x0UL) /**< MEM_PWR_ICACHEXIPSD_NORMAL Value */
#define MXC_S_PWRSEQ_MEM_PWR_ICACHEXIPSD_NORMAL        (MXC_V_PWRSEQ_MEM_PWR_ICACHEXIPSD_NORMAL << MXC_F_PWRSEQ_MEM_PWR_ICACHEXIPSD_POS) /**< MEM_PWR_ICACHEXIPSD_NORMAL Setting */
#define MXC_V_PWRSEQ_MEM_PWR_ICACHEXIPSD_SHUTDOWN      ((uint32_t)0x1UL) /**< MEM_PWR_ICACHEXIPSD_SHUTDOWN Value */
#define MXC_S_PWRSEQ_MEM_PWR_ICACHEXIPSD_SHUTDOWN      (MXC_V_PWRSEQ_MEM_PWR_ICACHEXIPSD_SHUTDOWN << MXC_F_PWRSEQ_MEM_PWR_ICACHEXIPSD_POS) /**< MEM_PWR_ICACHEXIPSD_SHUTDOWN Setting */

#define MXC_F_PWRSEQ_MEM_PWR_SCACHESD_POS              9 /**< MEM_PWR_SCACHESD Position */
#define MXC_F_PWRSEQ_MEM_PWR_SCACHESD                  ((uint32_t)(0x1UL << MXC_F_PWRSEQ_MEM_PWR_SCACHESD_POS)) /**< MEM_PWR_SCACHESD Mask */
#define MXC_V_PWRSEQ_MEM_PWR_SCACHESD_NORMAL           ((uint32_t)0x0UL) /**< MEM_PWR_SCACHESD_NORMAL Value */
#define MXC_S_PWRSEQ_MEM_PWR_SCACHESD_NORMAL           (MXC_V_PWRSEQ_MEM_PWR_SCACHESD_NORMAL << MXC_F_PWRSEQ_MEM_PWR_SCACHESD_POS) /**< MEM_PWR_SCACHESD_NORMAL Setting */
#define MXC_V_PWRSEQ_MEM_PWR_SCACHESD_SHUTDOWN         ((uint32_t)0x1UL) /**< MEM_PWR_SCACHESD_SHUTDOWN Value */
#define MXC_S_PWRSEQ_MEM_PWR_SCACHESD_SHUTDOWN         (MXC_V_PWRSEQ_MEM_PWR_SCACHESD_SHUTDOWN << MXC_F_PWRSEQ_MEM_PWR_SCACHESD_POS) /**< MEM_PWR_SCACHESD_SHUTDOWN Setting */

#define MXC_F_PWRSEQ_MEM_PWR_CRYPTOSD_POS              10 /**< MEM_PWR_CRYPTOSD Position */
#define MXC_F_PWRSEQ_MEM_PWR_CRYPTOSD                  ((uint32_t)(0x1UL << MXC_F_PWRSEQ_MEM_PWR_CRYPTOSD_POS)) /**< MEM_PWR_CRYPTOSD Mask */
#define MXC_V_PWRSEQ_MEM_PWR_CRYPTOSD_NORMAL           ((uint32_t)0x0UL) /**< MEM_PWR_CRYPTOSD_NORMAL Value */
#define MXC_S_PWRSEQ_MEM_PWR_CRYPTOSD_NORMAL           (MXC_V_PWRSEQ_MEM_PWR_CRYPTOSD_NORMAL << MXC_F_PWRSEQ_MEM_PWR_CRYPTOSD_POS) /**< MEM_PWR_CRYPTOSD_NORMAL Setting */
#define MXC_V_PWRSEQ_MEM_PWR_CRYPTOSD_SHUTDOWN         ((uint32_t)0x1UL) /**< MEM_PWR_CRYPTOSD_SHUTDOWN Value */
#define MXC_S_PWRSEQ_MEM_PWR_CRYPTOSD_SHUTDOWN         (MXC_V_PWRSEQ_MEM_PWR_CRYPTOSD_SHUTDOWN << MXC_F_PWRSEQ_MEM_PWR_CRYPTOSD_POS) /**< MEM_PWR_CRYPTOSD_SHUTDOWN Setting */

#define MXC_F_PWRSEQ_MEM_PWR_USBFIFOSD_POS             11 /**< MEM_PWR_USBFIFOSD Position */
#define MXC_F_PWRSEQ_MEM_PWR_USBFIFOSD                 ((uint32_t)(0x1UL << MXC_F_PWRSEQ_MEM_PWR_USBFIFOSD_POS)) /**< MEM_PWR_USBFIFOSD Mask */
#define MXC_V_PWRSEQ_MEM_PWR_USBFIFOSD_NORMAL          ((uint32_t)0x0UL) /**< MEM_PWR_USBFIFOSD_NORMAL Value */
#define MXC_S_PWRSEQ_MEM_PWR_USBFIFOSD_NORMAL          (MXC_V_PWRSEQ_MEM_PWR_USBFIFOSD_NORMAL << MXC_F_PWRSEQ_MEM_PWR_USBFIFOSD_POS) /**< MEM_PWR_USBFIFOSD_NORMAL Setting */
#define MXC_V_PWRSEQ_MEM_PWR_USBFIFOSD_SHUTDOWN        ((uint32_t)0x1UL) /**< MEM_PWR_USBFIFOSD_SHUTDOWN Value */
#define MXC_S_PWRSEQ_MEM_PWR_USBFIFOSD_SHUTDOWN        (MXC_V_PWRSEQ_MEM_PWR_USBFIFOSD_SHUTDOWN << MXC_F_PWRSEQ_MEM_PWR_USBFIFOSD_POS) /**< MEM_PWR_USBFIFOSD_SHUTDOWN Setting */

#define MXC_F_PWRSEQ_MEM_PWR_ROMSD_POS                 12 /**< MEM_PWR_ROMSD Position */
#define MXC_F_PWRSEQ_MEM_PWR_ROMSD                     ((uint32_t)(0x1UL << MXC_F_PWRSEQ_MEM_PWR_ROMSD_POS)) /**< MEM_PWR_ROMSD Mask */
#define MXC_V_PWRSEQ_MEM_PWR_ROMSD_NORMAL              ((uint32_t)0x0UL) /**< MEM_PWR_ROMSD_NORMAL Value */
#define MXC_S_PWRSEQ_MEM_PWR_ROMSD_NORMAL              (MXC_V_PWRSEQ_MEM_PWR_ROMSD_NORMAL << MXC_F_PWRSEQ_MEM_PWR_ROMSD_POS) /**< MEM_PWR_ROMSD_NORMAL Setting */
#define MXC_V_PWRSEQ_MEM_PWR_ROMSD_SHUTDOWN            ((uint32_t)0x1UL) /**< MEM_PWR_ROMSD_SHUTDOWN Value */
#define MXC_S_PWRSEQ_MEM_PWR_ROMSD_SHUTDOWN            (MXC_V_PWRSEQ_MEM_PWR_ROMSD_SHUTDOWN << MXC_F_PWRSEQ_MEM_PWR_ROMSD_POS) /**< MEM_PWR_ROMSD_SHUTDOWN Setting */

/**@} end of group PWRSEQ_MEM_PWR_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32650_INCLUDE_PWRSEQ_REGS_H_
