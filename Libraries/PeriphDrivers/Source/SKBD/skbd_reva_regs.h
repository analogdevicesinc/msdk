/**
 * @file    skbd_reva_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the SKBD_REVA Peripheral Module.
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

#ifndef _SKBD_REVA_REGS_H_
#define _SKBD_REVA_REGS_H_

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
 * @ingroup     skbd_reva
 * @defgroup    skbd_reva_registers SKBD_REVA_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the SKBD_REVA Peripheral Module.
 * @details Secure Keyboard
 */

/**
 * @ingroup skbd_reva_registers
 * Structure type to access the SKBD_REVA Registers.
 */
typedef struct {
    __IO uint32_t ctrl0;                /**< <tt>\b 0x00:</tt> SKBD_REVA CTRL0 Register */
    __IO uint32_t ctrl1;                /**< <tt>\b 0x04:</tt> SKBD_REVA CTRL1 Register */
    __I  uint32_t sr;                   /**< <tt>\b 0x08:</tt> SKBD_REVA SR Register */
    __IO uint32_t ier;                  /**< <tt>\b 0x0C:</tt> SKBD_REVA IER Register */
    __IO uint32_t isr;                  /**< <tt>\b 0x10:</tt> SKBD_REVA ISR Register */
    __I  uint32_t evt[4];               /**< <tt>\b 0x14:</tt> SKBD_REVA EVT Register */
} mxc_skbd_reva_regs_t;

/* Register offsets for module SKBD_REVA */
/**
 * @ingroup    skbd_reva_registers
 * @defgroup   SKBD_REVA_Register_Offsets Register Offsets
 * @brief      SKBD_REVA Peripheral Register Offsets from the SKBD_REVA Base Peripheral Address. 
 * @{
 */
 #define MXC_R_SKBD_REVA_CTRL0              ((uint32_t)0x00000000UL) /**< Offset from SKBD_REVA Base Address: <tt> 0x0000</tt> */ 
 #define MXC_R_SKBD_REVA_CTRL1              ((uint32_t)0x00000004UL) /**< Offset from SKBD_REVA Base Address: <tt> 0x0004</tt> */ 
 #define MXC_R_SKBD_REVA_SR                 ((uint32_t)0x00000008UL) /**< Offset from SKBD_REVA Base Address: <tt> 0x0008</tt> */ 
 #define MXC_R_SKBD_REVA_IER                ((uint32_t)0x0000000CUL) /**< Offset from SKBD_REVA Base Address: <tt> 0x000C</tt> */ 
 #define MXC_R_SKBD_REVA_ISR                ((uint32_t)0x00000010UL) /**< Offset from SKBD_REVA Base Address: <tt> 0x0010</tt> */ 
 #define MXC_R_SKBD_REVA_EVT                ((uint32_t)0x00000014UL) /**< Offset from SKBD_REVA Base Address: <tt> 0x0014</tt> */ 
/**@} end of group skbd_reva_registers */

/**
 * @ingroup  skbd_reva_registers
 * @defgroup SKBD_REVA_CTRL0 SKBD_REVA_CTRL0
 * @brief    Input Output Select Bits.  Each bit of IOSEL selects the pin direction for the
 *           corresponding KBDIO pin.  If IOSEL[0] = 1, KBDIO0 is an output.
 * @{
 */
 #define MXC_F_SKBD_REVA_CTRL0_KBDIO_0_POS              0 /**< CTRL0_KBDIO_0 Position */
 #define MXC_F_SKBD_REVA_CTRL0_KBDIO_0                  ((uint32_t)(0x1UL << MXC_F_SKBD_REVA_CTRL0_KBDIO_0_POS)) /**< CTRL0_KBDIO_0 Mask */

 #define MXC_F_SKBD_REVA_CTRL0_KBDIO_1_POS              1 /**< CTRL0_KBDIO_1 Position */
 #define MXC_F_SKBD_REVA_CTRL0_KBDIO_1                  ((uint32_t)(0x1UL << MXC_F_SKBD_REVA_CTRL0_KBDIO_1_POS)) /**< CTRL0_KBDIO_1 Mask */

 #define MXC_F_SKBD_REVA_CTRL0_KBDIO_2_POS              2 /**< CTRL0_KBDIO_2 Position */
 #define MXC_F_SKBD_REVA_CTRL0_KBDIO_2                  ((uint32_t)(0x1UL << MXC_F_SKBD_REVA_CTRL0_KBDIO_2_POS)) /**< CTRL0_KBDIO_2 Mask */

 #define MXC_F_SKBD_REVA_CTRL0_KBDIO_3_POS              3 /**< CTRL0_KBDIO_3 Position */
 #define MXC_F_SKBD_REVA_CTRL0_KBDIO_3                  ((uint32_t)(0x1UL << MXC_F_SKBD_REVA_CTRL0_KBDIO_3_POS)) /**< CTRL0_KBDIO_3 Mask */

 #define MXC_F_SKBD_REVA_CTRL0_KBDIO_4_POS              4 /**< CTRL0_KBDIO_4 Position */
 #define MXC_F_SKBD_REVA_CTRL0_KBDIO_4                  ((uint32_t)(0x1UL << MXC_F_SKBD_REVA_CTRL0_KBDIO_4_POS)) /**< CTRL0_KBDIO_4 Mask */

 #define MXC_F_SKBD_REVA_CTRL0_KBDIO_5_POS              5 /**< CTRL0_KBDIO_5 Position */
 #define MXC_F_SKBD_REVA_CTRL0_KBDIO_5                  ((uint32_t)(0x1UL << MXC_F_SKBD_REVA_CTRL0_KBDIO_5_POS)) /**< CTRL0_KBDIO_5 Mask */

 #define MXC_F_SKBD_REVA_CTRL0_KBDIO_6_POS              6 /**< CTRL0_KBDIO_6 Position */
 #define MXC_F_SKBD_REVA_CTRL0_KBDIO_6                  ((uint32_t)(0x1UL << MXC_F_SKBD_REVA_CTRL0_KBDIO_6_POS)) /**< CTRL0_KBDIO_6 Mask */

 #define MXC_F_SKBD_REVA_CTRL0_KBDIO_7_POS              7 /**< CTRL0_KBDIO_7 Position */
 #define MXC_F_SKBD_REVA_CTRL0_KBDIO_7                  ((uint32_t)(0x1UL << MXC_F_SKBD_REVA_CTRL0_KBDIO_7_POS)) /**< CTRL0_KBDIO_7 Mask */

 #define MXC_F_SKBD_REVA_CTRL0_KBDIO_8_POS              8 /**< CTRL0_KBDIO_8 Position */
 #define MXC_F_SKBD_REVA_CTRL0_KBDIO_8                  ((uint32_t)(0x1UL << MXC_F_SKBD_REVA_CTRL0_KBDIO_8_POS)) /**< CTRL0_KBDIO_8 Mask */

 #define MXC_F_SKBD_REVA_CTRL0_KBDIO_9_POS              9 /**< CTRL0_KBDIO_9 Position */
 #define MXC_F_SKBD_REVA_CTRL0_KBDIO_9                  ((uint32_t)(0x1UL << MXC_F_SKBD_REVA_CTRL0_KBDIO_9_POS)) /**< CTRL0_KBDIO_9 Mask */

/**@} end of group SKBD_REVA_CTRL0_Register */

/**
 * @ingroup  skbd_reva_registers
 * @defgroup SKBD_REVA_CTRL1 SKBD_REVA_CTRL1
 * @brief    Control Register 1
 * @{
 */
 #define MXC_F_SKBD_REVA_CTRL1_AUTOEN_POS               0 /**< CTRL1_AUTOEN Position */
 #define MXC_F_SKBD_REVA_CTRL1_AUTOEN                   ((uint32_t)(0x1UL << MXC_F_SKBD_REVA_CTRL1_AUTOEN_POS)) /**< CTRL1_AUTOEN Mask */

 #define MXC_F_SKBD_REVA_CTRL1_CLEAR_POS                1 /**< CTRL1_CLEAR Position */
 #define MXC_F_SKBD_REVA_CTRL1_CLEAR                    ((uint32_t)(0x1UL << MXC_F_SKBD_REVA_CTRL1_CLEAR_POS)) /**< CTRL1_CLEAR Mask */

 #define MXC_F_SKBD_REVA_CTRL1_OUTNB_POS                8 /**< CTRL1_OUTNB Position */
 #define MXC_F_SKBD_REVA_CTRL1_OUTNB                    ((uint32_t)(0x7UL << MXC_F_SKBD_REVA_CTRL1_OUTNB_POS)) /**< CTRL1_OUTNB Mask */

 #define MXC_F_SKBD_REVA_CTRL1_DBTM_POS                 13 /**< CTRL1_DBTM Position */
 #define MXC_F_SKBD_REVA_CTRL1_DBTM                     ((uint32_t)(0x7UL << MXC_F_SKBD_REVA_CTRL1_DBTM_POS)) /**< CTRL1_DBTM Mask */
 #define MXC_V_SKBD_REVA_CTRL1_DBTM_TIME4MS             ((uint32_t)0x0UL) /**< CTRL1_DBTM_TIME4MS Value */
 #define MXC_S_SKBD_REVA_CTRL1_DBTM_TIME4MS             (MXC_V_SKBD_REVA_CTRL1_DBTM_TIME4MS << MXC_F_SKBD_REVA_CTRL1_DBTM_POS) /**< CTRL1_DBTM_TIME4MS Setting */
 #define MXC_V_SKBD_REVA_CTRL1_DBTM_TIME5MS             ((uint32_t)0x1UL) /**< CTRL1_DBTM_TIME5MS Value */
 #define MXC_S_SKBD_REVA_CTRL1_DBTM_TIME5MS             (MXC_V_SKBD_REVA_CTRL1_DBTM_TIME5MS << MXC_F_SKBD_REVA_CTRL1_DBTM_POS) /**< CTRL1_DBTM_TIME5MS Setting */
 #define MXC_V_SKBD_REVA_CTRL1_DBTM_TIME6MS             ((uint32_t)0x2UL) /**< CTRL1_DBTM_TIME6MS Value */
 #define MXC_S_SKBD_REVA_CTRL1_DBTM_TIME6MS             (MXC_V_SKBD_REVA_CTRL1_DBTM_TIME6MS << MXC_F_SKBD_REVA_CTRL1_DBTM_POS) /**< CTRL1_DBTM_TIME6MS Setting */
 #define MXC_V_SKBD_REVA_CTRL1_DBTM_TIME7MS             ((uint32_t)0x3UL) /**< CTRL1_DBTM_TIME7MS Value */
 #define MXC_S_SKBD_REVA_CTRL1_DBTM_TIME7MS             (MXC_V_SKBD_REVA_CTRL1_DBTM_TIME7MS << MXC_F_SKBD_REVA_CTRL1_DBTM_POS) /**< CTRL1_DBTM_TIME7MS Setting */
 #define MXC_V_SKBD_REVA_CTRL1_DBTM_TIME8MS             ((uint32_t)0x4UL) /**< CTRL1_DBTM_TIME8MS Value */
 #define MXC_S_SKBD_REVA_CTRL1_DBTM_TIME8MS             (MXC_V_SKBD_REVA_CTRL1_DBTM_TIME8MS << MXC_F_SKBD_REVA_CTRL1_DBTM_POS) /**< CTRL1_DBTM_TIME8MS Setting */
 #define MXC_V_SKBD_REVA_CTRL1_DBTM_TIME10MS            ((uint32_t)0x5UL) /**< CTRL1_DBTM_TIME10MS Value */
 #define MXC_S_SKBD_REVA_CTRL1_DBTM_TIME10MS            (MXC_V_SKBD_REVA_CTRL1_DBTM_TIME10MS << MXC_F_SKBD_REVA_CTRL1_DBTM_POS) /**< CTRL1_DBTM_TIME10MS Setting */
 #define MXC_V_SKBD_REVA_CTRL1_DBTM_TIME11MS            ((uint32_t)0x6UL) /**< CTRL1_DBTM_TIME11MS Value */
 #define MXC_S_SKBD_REVA_CTRL1_DBTM_TIME11MS            (MXC_V_SKBD_REVA_CTRL1_DBTM_TIME11MS << MXC_F_SKBD_REVA_CTRL1_DBTM_POS) /**< CTRL1_DBTM_TIME11MS Setting */
 #define MXC_V_SKBD_REVA_CTRL1_DBTM_TIME12MS            ((uint32_t)0x7UL) /**< CTRL1_DBTM_TIME12MS Value */
 #define MXC_S_SKBD_REVA_CTRL1_DBTM_TIME12MS            (MXC_V_SKBD_REVA_CTRL1_DBTM_TIME12MS << MXC_F_SKBD_REVA_CTRL1_DBTM_POS) /**< CTRL1_DBTM_TIME12MS Setting */

/**@} end of group SKBD_REVA_CTRL1_Register */

/**
 * @ingroup  skbd_reva_registers
 * @defgroup SKBD_REVA_SR SKBD_REVA_SR
 * @brief    Status Register
 * @{
 */
 #define MXC_F_SKBD_REVA_SR_BUSY_POS                    0 /**< SR_BUSY Position */
 #define MXC_F_SKBD_REVA_SR_BUSY                        ((uint32_t)(0x1UL << MXC_F_SKBD_REVA_SR_BUSY_POS)) /**< SR_BUSY Mask */

/**@} end of group SKBD_REVA_SR_Register */

/**
 * @ingroup  skbd_reva_registers
 * @defgroup SKBD_REVA_IER SKBD_REVA_IER
 * @brief    Interrupt Enable Register
 * @{
 */
 #define MXC_F_SKBD_REVA_IER_PUSHIE_POS                 0 /**< IER_PUSHIE Position */
 #define MXC_F_SKBD_REVA_IER_PUSHIE                     ((uint32_t)(0x1UL << MXC_F_SKBD_REVA_IER_PUSHIE_POS)) /**< IER_PUSHIE Mask */

 #define MXC_F_SKBD_REVA_IER_RELEASEIE_POS              1 /**< IER_RELEASEIE Position */
 #define MXC_F_SKBD_REVA_IER_RELEASEIE                  ((uint32_t)(0x1UL << MXC_F_SKBD_REVA_IER_RELEASEIE_POS)) /**< IER_RELEASEIE Mask */

 #define MXC_F_SKBD_REVA_IER_OVERIE_POS                 2 /**< IER_OVERIE Position */
 #define MXC_F_SKBD_REVA_IER_OVERIE                     ((uint32_t)(0x1UL << MXC_F_SKBD_REVA_IER_OVERIE_POS)) /**< IER_OVERIE Mask */

/**@} end of group SKBD_REVA_IER_Register */

/**
 * @ingroup  skbd_reva_registers
 * @defgroup SKBD_REVA_ISR SKBD_REVA_ISR
 * @brief    Interrupt Status Register
 * @{
 */
 #define MXC_F_SKBD_REVA_ISR_PUSHIS_POS                 0 /**< ISR_PUSHIS Position */
 #define MXC_F_SKBD_REVA_ISR_PUSHIS                     ((uint32_t)(0x1UL << MXC_F_SKBD_REVA_ISR_PUSHIS_POS)) /**< ISR_PUSHIS Mask */

 #define MXC_F_SKBD_REVA_ISR_RELEASEIS_POS              1 /**< ISR_RELEASEIS Position */
 #define MXC_F_SKBD_REVA_ISR_RELEASEIS                  ((uint32_t)(0x1UL << MXC_F_SKBD_REVA_ISR_RELEASEIS_POS)) /**< ISR_RELEASEIS Mask */

 #define MXC_F_SKBD_REVA_ISR_OVERIS_POS                 2 /**< ISR_OVERIS Position */
 #define MXC_F_SKBD_REVA_ISR_OVERIS                     ((uint32_t)(0x1UL << MXC_F_SKBD_REVA_ISR_OVERIS_POS)) /**< ISR_OVERIS Mask */

/**@} end of group SKBD_REVA_ISR_Register */

/**
 * @ingroup  skbd_reva_registers
 * @defgroup SKBD_REVA_EVT SKBD_REVA_EVT
 * @brief    Key Register
 * @{
 */
 #define MXC_F_SKBD_REVA_EVT_IOIN_POS                   0 /**< EVT_IOIN Position */
 #define MXC_F_SKBD_REVA_EVT_IOIN                       ((uint32_t)(0x7UL << MXC_F_SKBD_REVA_EVT_IOIN_POS)) /**< EVT_IOIN Mask */

 #define MXC_F_SKBD_REVA_EVT_IOOUT_POS                  5 /**< EVT_IOOUT Position */
 #define MXC_F_SKBD_REVA_EVT_IOOUT                      ((uint32_t)(0x7UL << MXC_F_SKBD_REVA_EVT_IOOUT_POS)) /**< EVT_IOOUT Mask */

 #define MXC_F_SKBD_REVA_EVT_PUSH_POS                   10 /**< EVT_PUSH Position */
 #define MXC_F_SKBD_REVA_EVT_PUSH                       ((uint32_t)(0x1UL << MXC_F_SKBD_REVA_EVT_PUSH_POS)) /**< EVT_PUSH Mask */

 #define MXC_F_SKBD_REVA_EVT_READ_POS                   11 /**< EVT_READ Position */
 #define MXC_F_SKBD_REVA_EVT_READ                       ((uint32_t)(0x1UL << MXC_F_SKBD_REVA_EVT_READ_POS)) /**< EVT_READ Mask */

 #define MXC_F_SKBD_REVA_EVT_NEXT_POS                   12 /**< EVT_NEXT Position */
 #define MXC_F_SKBD_REVA_EVT_NEXT                       ((uint32_t)(0x1UL << MXC_F_SKBD_REVA_EVT_NEXT_POS)) /**< EVT_NEXT Mask */

/**@} end of group SKBD_REVA_EVT_Register */

#ifdef __cplusplus
}
#endif

#endif /* _SKBD_REVA_REGS_H_ */
