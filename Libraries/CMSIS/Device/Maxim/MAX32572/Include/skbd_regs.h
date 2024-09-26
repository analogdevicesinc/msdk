/**
 * @file    skbd_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the SKBD Peripheral Module.
 * @note    This file is @generated.
 * @ingroup skbd_registers
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32572_INCLUDE_SKBD_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32572_INCLUDE_SKBD_REGS_H_

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
 * @ingroup     skbd
 * @defgroup    skbd_registers SKBD_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the SKBD Peripheral Module.
 * @details     Secure Keyboard
 */

/**
 * @ingroup skbd_registers
 * Structure type to access the SKBD Registers.
 */
typedef struct {
    __IO uint32_t ctrl0;                /**< <tt>\b 0x00:</tt> SKBD CTRL0 Register */
    __IO uint32_t ctrl1;                /**< <tt>\b 0x04:</tt> SKBD CTRL1 Register */
    __I  uint32_t status;               /**< <tt>\b 0x08:</tt> SKBD STATUS Register */
    __IO uint32_t inten;                /**< <tt>\b 0x0C:</tt> SKBD INTEN Register */
    __IO uint32_t intfl;                /**< <tt>\b 0x10:</tt> SKBD INTFL Register */
    __I  uint32_t evt[4];               /**< <tt>\b 0x14:</tt> SKBD EVT Register */
    __IO uint32_t gpio0;                /**< <tt>\b 0x24:</tt> SKBD GPIO0 Register */
    __IO uint32_t gpio1;                /**< <tt>\b 0x28:</tt> SKBD GPIO1 Register */
} mxc_skbd_regs_t;

/* Register offsets for module SKBD */
/**
 * @ingroup    skbd_registers
 * @defgroup   SKBD_Register_Offsets Register Offsets
 * @brief      SKBD Peripheral Register Offsets from the SKBD Base Peripheral Address.
 * @{
 */
#define MXC_R_SKBD_CTRL0                   ((uint32_t)0x00000000UL) /**< Offset from SKBD Base Address: <tt> 0x0000</tt> */
#define MXC_R_SKBD_CTRL1                   ((uint32_t)0x00000004UL) /**< Offset from SKBD Base Address: <tt> 0x0004</tt> */
#define MXC_R_SKBD_STATUS                  ((uint32_t)0x00000008UL) /**< Offset from SKBD Base Address: <tt> 0x0008</tt> */
#define MXC_R_SKBD_INTEN                   ((uint32_t)0x0000000CUL) /**< Offset from SKBD Base Address: <tt> 0x000C</tt> */
#define MXC_R_SKBD_INTFL                   ((uint32_t)0x00000010UL) /**< Offset from SKBD Base Address: <tt> 0x0010</tt> */
#define MXC_R_SKBD_EVT                     ((uint32_t)0x00000014UL) /**< Offset from SKBD Base Address: <tt> 0x0014</tt> */
#define MXC_R_SKBD_GPIO0                   ((uint32_t)0x00000024UL) /**< Offset from SKBD Base Address: <tt> 0x0024</tt> */
#define MXC_R_SKBD_GPIO1                   ((uint32_t)0x00000028UL) /**< Offset from SKBD Base Address: <tt> 0x0028</tt> */
/**@} end of group skbd_registers */

/**
 * @ingroup  skbd_registers
 * @defgroup SKBD_CTRL0 SKBD_CTRL0
 * @brief    Input Output Select Bits.  Each bit of IOSEL selects the pin direction for the
 *           corresponding KBDIO pin.  If IOSEL[0] = 1, KBDIO0 is an output.
 * @{
 */
#define MXC_F_SKBD_CTRL0_KBDIO0_POS                    0 /**< CTRL0_KBDIO0 Position */
#define MXC_F_SKBD_CTRL0_KBDIO0                        ((uint32_t)(0x1UL << MXC_F_SKBD_CTRL0_KBDIO0_POS)) /**< CTRL0_KBDIO0 Mask */

#define MXC_F_SKBD_CTRL0_KBDIO1_POS                    1 /**< CTRL0_KBDIO1 Position */
#define MXC_F_SKBD_CTRL0_KBDIO1                        ((uint32_t)(0x1UL << MXC_F_SKBD_CTRL0_KBDIO1_POS)) /**< CTRL0_KBDIO1 Mask */

#define MXC_F_SKBD_CTRL0_KBDIO2_POS                    2 /**< CTRL0_KBDIO2 Position */
#define MXC_F_SKBD_CTRL0_KBDIO2                        ((uint32_t)(0x1UL << MXC_F_SKBD_CTRL0_KBDIO2_POS)) /**< CTRL0_KBDIO2 Mask */

#define MXC_F_SKBD_CTRL0_KBDIO3_POS                    3 /**< CTRL0_KBDIO3 Position */
#define MXC_F_SKBD_CTRL0_KBDIO3                        ((uint32_t)(0x1UL << MXC_F_SKBD_CTRL0_KBDIO3_POS)) /**< CTRL0_KBDIO3 Mask */

#define MXC_F_SKBD_CTRL0_KBDIO4_POS                    4 /**< CTRL0_KBDIO4 Position */
#define MXC_F_SKBD_CTRL0_KBDIO4                        ((uint32_t)(0x1UL << MXC_F_SKBD_CTRL0_KBDIO4_POS)) /**< CTRL0_KBDIO4 Mask */

#define MXC_F_SKBD_CTRL0_KBDIO5_POS                    5 /**< CTRL0_KBDIO5 Position */
#define MXC_F_SKBD_CTRL0_KBDIO5                        ((uint32_t)(0x1UL << MXC_F_SKBD_CTRL0_KBDIO5_POS)) /**< CTRL0_KBDIO5 Mask */

#define MXC_F_SKBD_CTRL0_KBDIO6_POS                    6 /**< CTRL0_KBDIO6 Position */
#define MXC_F_SKBD_CTRL0_KBDIO6                        ((uint32_t)(0x1UL << MXC_F_SKBD_CTRL0_KBDIO6_POS)) /**< CTRL0_KBDIO6 Mask */

#define MXC_F_SKBD_CTRL0_KBDIO7_POS                    7 /**< CTRL0_KBDIO7 Position */
#define MXC_F_SKBD_CTRL0_KBDIO7                        ((uint32_t)(0x1UL << MXC_F_SKBD_CTRL0_KBDIO7_POS)) /**< CTRL0_KBDIO7 Mask */

#define MXC_F_SKBD_CTRL0_KBDIO8_POS                    8 /**< CTRL0_KBDIO8 Position */
#define MXC_F_SKBD_CTRL0_KBDIO8                        ((uint32_t)(0x1UL << MXC_F_SKBD_CTRL0_KBDIO8_POS)) /**< CTRL0_KBDIO8 Mask */

#define MXC_F_SKBD_CTRL0_KBDIO9_POS                    9 /**< CTRL0_KBDIO9 Position */
#define MXC_F_SKBD_CTRL0_KBDIO9                        ((uint32_t)(0x1UL << MXC_F_SKBD_CTRL0_KBDIO9_POS)) /**< CTRL0_KBDIO9 Mask */

/**@} end of group SKBD_CTRL0_Register */

/**
 * @ingroup  skbd_registers
 * @defgroup SKBD_CTRL1 SKBD_CTRL1
 * @brief    Control Register 1
 * @{
 */
#define MXC_F_SKBD_CTRL1_AUTOSCAN_EN_POS               0 /**< CTRL1_AUTOSCAN_EN Position */
#define MXC_F_SKBD_CTRL1_AUTOSCAN_EN                   ((uint32_t)(0x1UL << MXC_F_SKBD_CTRL1_AUTOSCAN_EN_POS)) /**< CTRL1_AUTOSCAN_EN Mask */

#define MXC_F_SKBD_CTRL1_AUTOCLEAR_POS                 1 /**< CTRL1_AUTOCLEAR Position */
#define MXC_F_SKBD_CTRL1_AUTOCLEAR                     ((uint32_t)(0x1UL << MXC_F_SKBD_CTRL1_AUTOCLEAR_POS)) /**< CTRL1_AUTOCLEAR Mask */

#define MXC_F_SKBD_CTRL1_OUTNUM_POS                    8 /**< CTRL1_OUTNUM Position */
#define MXC_F_SKBD_CTRL1_OUTNUM                        ((uint32_t)(0xFUL << MXC_F_SKBD_CTRL1_OUTNUM_POS)) /**< CTRL1_OUTNUM Mask */

#define MXC_F_SKBD_CTRL1_DBTM_POS                      13 /**< CTRL1_DBTM Position */
#define MXC_F_SKBD_CTRL1_DBTM                          ((uint32_t)(0x7UL << MXC_F_SKBD_CTRL1_DBTM_POS)) /**< CTRL1_DBTM Mask */
#define MXC_V_SKBD_CTRL1_DBTM_TIME4MS                  ((uint32_t)0x0UL) /**< CTRL1_DBTM_TIME4MS Value */
#define MXC_S_SKBD_CTRL1_DBTM_TIME4MS                  (MXC_V_SKBD_CTRL1_DBTM_TIME4MS << MXC_F_SKBD_CTRL1_DBTM_POS) /**< CTRL1_DBTM_TIME4MS Setting */
#define MXC_V_SKBD_CTRL1_DBTM_TIME5MS                  ((uint32_t)0x1UL) /**< CTRL1_DBTM_TIME5MS Value */
#define MXC_S_SKBD_CTRL1_DBTM_TIME5MS                  (MXC_V_SKBD_CTRL1_DBTM_TIME5MS << MXC_F_SKBD_CTRL1_DBTM_POS) /**< CTRL1_DBTM_TIME5MS Setting */
#define MXC_V_SKBD_CTRL1_DBTM_TIME6MS                  ((uint32_t)0x2UL) /**< CTRL1_DBTM_TIME6MS Value */
#define MXC_S_SKBD_CTRL1_DBTM_TIME6MS                  (MXC_V_SKBD_CTRL1_DBTM_TIME6MS << MXC_F_SKBD_CTRL1_DBTM_POS) /**< CTRL1_DBTM_TIME6MS Setting */
#define MXC_V_SKBD_CTRL1_DBTM_TIME7MS                  ((uint32_t)0x3UL) /**< CTRL1_DBTM_TIME7MS Value */
#define MXC_S_SKBD_CTRL1_DBTM_TIME7MS                  (MXC_V_SKBD_CTRL1_DBTM_TIME7MS << MXC_F_SKBD_CTRL1_DBTM_POS) /**< CTRL1_DBTM_TIME7MS Setting */
#define MXC_V_SKBD_CTRL1_DBTM_TIME8MS                  ((uint32_t)0x4UL) /**< CTRL1_DBTM_TIME8MS Value */
#define MXC_S_SKBD_CTRL1_DBTM_TIME8MS                  (MXC_V_SKBD_CTRL1_DBTM_TIME8MS << MXC_F_SKBD_CTRL1_DBTM_POS) /**< CTRL1_DBTM_TIME8MS Setting */
#define MXC_V_SKBD_CTRL1_DBTM_TIME10MS                 ((uint32_t)0x5UL) /**< CTRL1_DBTM_TIME10MS Value */
#define MXC_S_SKBD_CTRL1_DBTM_TIME10MS                 (MXC_V_SKBD_CTRL1_DBTM_TIME10MS << MXC_F_SKBD_CTRL1_DBTM_POS) /**< CTRL1_DBTM_TIME10MS Setting */
#define MXC_V_SKBD_CTRL1_DBTM_TIME11MS                 ((uint32_t)0x6UL) /**< CTRL1_DBTM_TIME11MS Value */
#define MXC_S_SKBD_CTRL1_DBTM_TIME11MS                 (MXC_V_SKBD_CTRL1_DBTM_TIME11MS << MXC_F_SKBD_CTRL1_DBTM_POS) /**< CTRL1_DBTM_TIME11MS Setting */
#define MXC_V_SKBD_CTRL1_DBTM_TIME12MS                 ((uint32_t)0x7UL) /**< CTRL1_DBTM_TIME12MS Value */
#define MXC_S_SKBD_CTRL1_DBTM_TIME12MS                 (MXC_V_SKBD_CTRL1_DBTM_TIME12MS << MXC_F_SKBD_CTRL1_DBTM_POS) /**< CTRL1_DBTM_TIME12MS Setting */

/**@} end of group SKBD_CTRL1_Register */

/**
 * @ingroup  skbd_registers
 * @defgroup SKBD_STATUS SKBD_STATUS
 * @brief    Status Register
 * @{
 */
#define MXC_F_SKBD_STATUS_BUSY_POS                     0 /**< STATUS_BUSY Position */
#define MXC_F_SKBD_STATUS_BUSY                         ((uint32_t)(0x1UL << MXC_F_SKBD_STATUS_BUSY_POS)) /**< STATUS_BUSY Mask */

/**@} end of group SKBD_STATUS_Register */

/**
 * @ingroup  skbd_registers
 * @defgroup SKBD_INTEN SKBD_INTEN
 * @brief    Interrupt Enable Register
 * @{
 */
#define MXC_F_SKBD_INTEN_PUSH_POS                      0 /**< INTEN_PUSH Position */
#define MXC_F_SKBD_INTEN_PUSH                          ((uint32_t)(0x1UL << MXC_F_SKBD_INTEN_PUSH_POS)) /**< INTEN_PUSH Mask */

#define MXC_F_SKBD_INTEN_RELEASE_POS                   1 /**< INTEN_RELEASE Position */
#define MXC_F_SKBD_INTEN_RELEASE                       ((uint32_t)(0x1UL << MXC_F_SKBD_INTEN_RELEASE_POS)) /**< INTEN_RELEASE Mask */

#define MXC_F_SKBD_INTEN_OVERRUN_POS                   2 /**< INTEN_OVERRUN Position */
#define MXC_F_SKBD_INTEN_OVERRUN                       ((uint32_t)(0x1UL << MXC_F_SKBD_INTEN_OVERRUN_POS)) /**< INTEN_OVERRUN Mask */

#define MXC_F_SKBD_INTEN_KBD_PINS_POS                  3 /**< INTEN_KBD_PINS Position */
#define MXC_F_SKBD_INTEN_KBD_PINS                      ((uint32_t)(0x1UL << MXC_F_SKBD_INTEN_KBD_PINS_POS)) /**< INTEN_KBD_PINS Mask */

/**@} end of group SKBD_INTEN_Register */

/**
 * @ingroup  skbd_registers
 * @defgroup SKBD_INTFL SKBD_INTFL
 * @brief    Interrupt Status Register
 * @{
 */
#define MXC_F_SKBD_INTFL_PUSH_POS                      0 /**< INTFL_PUSH Position */
#define MXC_F_SKBD_INTFL_PUSH                          ((uint32_t)(0x1UL << MXC_F_SKBD_INTFL_PUSH_POS)) /**< INTFL_PUSH Mask */

#define MXC_F_SKBD_INTFL_RELEASE_POS                   1 /**< INTFL_RELEASE Position */
#define MXC_F_SKBD_INTFL_RELEASE                       ((uint32_t)(0x1UL << MXC_F_SKBD_INTFL_RELEASE_POS)) /**< INTFL_RELEASE Mask */

#define MXC_F_SKBD_INTFL_OVERRUN_POS                   2 /**< INTFL_OVERRUN Position */
#define MXC_F_SKBD_INTFL_OVERRUN                       ((uint32_t)(0x1UL << MXC_F_SKBD_INTFL_OVERRUN_POS)) /**< INTFL_OVERRUN Mask */

/**@} end of group SKBD_INTFL_Register */

/**
 * @ingroup  skbd_registers
 * @defgroup SKBD_EVT SKBD_EVT
 * @brief    Key Register
 * @{
 */
#define MXC_F_SKBD_EVT_IOIN_POS                        0 /**< EVT_IOIN Position */
#define MXC_F_SKBD_EVT_IOIN                            ((uint32_t)(0x7UL << MXC_F_SKBD_EVT_IOIN_POS)) /**< EVT_IOIN Mask */

#define MXC_F_SKBD_EVT_IOOUT_POS                       5 /**< EVT_IOOUT Position */
#define MXC_F_SKBD_EVT_IOOUT                           ((uint32_t)(0x7UL << MXC_F_SKBD_EVT_IOOUT_POS)) /**< EVT_IOOUT Mask */

#define MXC_F_SKBD_EVT_PUSH_POS                        10 /**< EVT_PUSH Position */
#define MXC_F_SKBD_EVT_PUSH                            ((uint32_t)(0x1UL << MXC_F_SKBD_EVT_PUSH_POS)) /**< EVT_PUSH Mask */

#define MXC_F_SKBD_EVT_READ_POS                        11 /**< EVT_READ Position */
#define MXC_F_SKBD_EVT_READ                            ((uint32_t)(0x1UL << MXC_F_SKBD_EVT_READ_POS)) /**< EVT_READ Mask */

#define MXC_F_SKBD_EVT_NEXT_POS                        12 /**< EVT_NEXT Position */
#define MXC_F_SKBD_EVT_NEXT                            ((uint32_t)(0x1UL << MXC_F_SKBD_EVT_NEXT_POS)) /**< EVT_NEXT Mask */

/**@} end of group SKBD_EVT_Register */

/**
 * @ingroup  skbd_registers
 * @defgroup SKBD_GPIO0 SKBD_GPIO0
 * @brief    General Purpose Register 0.
 * @{
 */
#define MXC_F_SKBD_GPIO0_ALL_POS                       0 /**< GPIO0_ALL Position */
#define MXC_F_SKBD_GPIO0_ALL                           ((uint32_t)(0xFFFFFFFFUL << MXC_F_SKBD_GPIO0_ALL_POS)) /**< GPIO0_ALL Mask */

/**@} end of group SKBD_GPIO0_Register */

/**
 * @ingroup  skbd_registers
 * @defgroup SKBD_GPIO1 SKBD_GPIO1
 * @brief    General Purpose Register 1.
 * @{
 */
#define MXC_F_SKBD_GPIO1_ALL_POS                       0 /**< GPIO1_ALL Position */
#define MXC_F_SKBD_GPIO1_ALL                           ((uint32_t)(0xFFFFFFFFUL << MXC_F_SKBD_GPIO1_ALL_POS)) /**< GPIO1_ALL Mask */

/**@} end of group SKBD_GPIO1_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32572_INCLUDE_SKBD_REGS_H_
