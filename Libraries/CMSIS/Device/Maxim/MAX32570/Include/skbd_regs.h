/**
 * @file    skbd_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the SKBD Peripheral Module.
 * @note    This file is @generated.
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32570_INCLUDE_SKBD_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32570_INCLUDE_SKBD_REGS_H_

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
    __IO uint32_t cr0;                  /**< <tt>\b 0x00:</tt> SKBD CR0 Register */
    __IO uint32_t cr1;                  /**< <tt>\b 0x04:</tt> SKBD CR1 Register */
    __I  uint32_t sr;                   /**< <tt>\b 0x08:</tt> SKBD SR Register */
    __IO uint32_t ier;                  /**< <tt>\b 0x0C:</tt> SKBD IER Register */
    __IO uint32_t isr;                  /**< <tt>\b 0x10:</tt> SKBD ISR Register */
    __I  uint32_t event[4];             /**< <tt>\b 0x14:</tt> SKBD EVENT Register */
} mxc_skbd_regs_t;

/* Register offsets for module SKBD */
/**
 * @ingroup    skbd_registers
 * @defgroup   SKBD_Register_Offsets Register Offsets
 * @brief      SKBD Peripheral Register Offsets from the SKBD Base Peripheral Address.
 * @{
 */
#define MXC_R_SKBD_CR0                     ((uint32_t)0x00000000UL) /**< Offset from SKBD Base Address: <tt> 0x0000</tt> */
#define MXC_R_SKBD_CR1                     ((uint32_t)0x00000004UL) /**< Offset from SKBD Base Address: <tt> 0x0004</tt> */
#define MXC_R_SKBD_SR                      ((uint32_t)0x00000008UL) /**< Offset from SKBD Base Address: <tt> 0x0008</tt> */
#define MXC_R_SKBD_IER                     ((uint32_t)0x0000000CUL) /**< Offset from SKBD Base Address: <tt> 0x000C</tt> */
#define MXC_R_SKBD_ISR                     ((uint32_t)0x00000010UL) /**< Offset from SKBD Base Address: <tt> 0x0010</tt> */
#define MXC_R_SKBD_EVENT                   ((uint32_t)0x00000014UL) /**< Offset from SKBD Base Address: <tt> 0x0014</tt> */
/**@} end of group skbd_registers */

/**
 * @ingroup  skbd_registers
 * @defgroup SKBD_CR0 SKBD_CR0
 * @brief    Input Output Select Bits.  Each bit of IOSEL selects the pin direction for the
 *           corresponding KBDIO pin.  If IOSEL[0] = 1, KBDIO0 is an output.
 * @{
 */
#define MXC_F_SKBD_CR0_KBDIO_0_POS                     0 /**< CR0_KBDIO_0 Position */
#define MXC_F_SKBD_CR0_KBDIO_0                         ((uint32_t)(0x3FFUL << MXC_F_SKBD_CR0_KBDIO_0_POS)) /**< CR0_KBDIO_0 Mask */
#define MXC_V_SKBD_CR0_KBDIO_0_INPUT                   ((uint32_t)0x0UL) /**< CR0_KBDIO_0_INPUT Value */
#define MXC_S_SKBD_CR0_KBDIO_0_INPUT                   (MXC_V_SKBD_CR0_KBDIO_0_INPUT << MXC_F_SKBD_CR0_KBDIO_0_POS) /**< CR0_KBDIO_0_INPUT Setting */
#define MXC_V_SKBD_CR0_KBDIO_0_OUTPUT                  ((uint32_t)0x1UL) /**< CR0_KBDIO_0_OUTPUT Value */
#define MXC_S_SKBD_CR0_KBDIO_0_OUTPUT                  (MXC_V_SKBD_CR0_KBDIO_0_OUTPUT << MXC_F_SKBD_CR0_KBDIO_0_POS) /**< CR0_KBDIO_0_OUTPUT Setting */

/**@} end of group SKBD_CR0_Register */

/**
 * @ingroup  skbd_registers
 * @defgroup SKBD_CR1 SKBD_CR1
 * @brief    Control Register 1
 * @{
 */
#define MXC_F_SKBD_CR1_AUTOEN_POS                      0 /**< CR1_AUTOEN Position */
#define MXC_F_SKBD_CR1_AUTOEN                          ((uint32_t)(0x1UL << MXC_F_SKBD_CR1_AUTOEN_POS)) /**< CR1_AUTOEN Mask */

#define MXC_F_SKBD_CR1_CLEAR_POS                       1 /**< CR1_CLEAR Position */
#define MXC_F_SKBD_CR1_CLEAR                           ((uint32_t)(0x1UL << MXC_F_SKBD_CR1_CLEAR_POS)) /**< CR1_CLEAR Mask */

#define MXC_F_SKBD_CR1_OUTNB_POS                       8 /**< CR1_OUTNB Position */
#define MXC_F_SKBD_CR1_OUTNB                           ((uint32_t)(0x7UL << MXC_F_SKBD_CR1_OUTNB_POS)) /**< CR1_OUTNB Mask */

#define MXC_F_SKBD_CR1_DBTM_POS                        13 /**< CR1_DBTM Position */
#define MXC_F_SKBD_CR1_DBTM                            ((uint32_t)(0x7UL << MXC_F_SKBD_CR1_DBTM_POS)) /**< CR1_DBTM Mask */
#define MXC_V_SKBD_CR1_DBTM_TIME4MS                    ((uint32_t)0x0UL) /**< CR1_DBTM_TIME4MS Value */
#define MXC_S_SKBD_CR1_DBTM_TIME4MS                    (MXC_V_SKBD_CR1_DBTM_TIME4MS << MXC_F_SKBD_CR1_DBTM_POS) /**< CR1_DBTM_TIME4MS Setting */
#define MXC_V_SKBD_CR1_DBTM_TIME5MS                    ((uint32_t)0x1UL) /**< CR1_DBTM_TIME5MS Value */
#define MXC_S_SKBD_CR1_DBTM_TIME5MS                    (MXC_V_SKBD_CR1_DBTM_TIME5MS << MXC_F_SKBD_CR1_DBTM_POS) /**< CR1_DBTM_TIME5MS Setting */
#define MXC_V_SKBD_CR1_DBTM_TIME6MS                    ((uint32_t)0x2UL) /**< CR1_DBTM_TIME6MS Value */
#define MXC_S_SKBD_CR1_DBTM_TIME6MS                    (MXC_V_SKBD_CR1_DBTM_TIME6MS << MXC_F_SKBD_CR1_DBTM_POS) /**< CR1_DBTM_TIME6MS Setting */
#define MXC_V_SKBD_CR1_DBTM_TIME7MS                    ((uint32_t)0x3UL) /**< CR1_DBTM_TIME7MS Value */
#define MXC_S_SKBD_CR1_DBTM_TIME7MS                    (MXC_V_SKBD_CR1_DBTM_TIME7MS << MXC_F_SKBD_CR1_DBTM_POS) /**< CR1_DBTM_TIME7MS Setting */
#define MXC_V_SKBD_CR1_DBTM_TIME8MS                    ((uint32_t)0x4UL) /**< CR1_DBTM_TIME8MS Value */
#define MXC_S_SKBD_CR1_DBTM_TIME8MS                    (MXC_V_SKBD_CR1_DBTM_TIME8MS << MXC_F_SKBD_CR1_DBTM_POS) /**< CR1_DBTM_TIME8MS Setting */
#define MXC_V_SKBD_CR1_DBTM_TIME10MS                   ((uint32_t)0x5UL) /**< CR1_DBTM_TIME10MS Value */
#define MXC_S_SKBD_CR1_DBTM_TIME10MS                   (MXC_V_SKBD_CR1_DBTM_TIME10MS << MXC_F_SKBD_CR1_DBTM_POS) /**< CR1_DBTM_TIME10MS Setting */
#define MXC_V_SKBD_CR1_DBTM_TIME11MS                   ((uint32_t)0x6UL) /**< CR1_DBTM_TIME11MS Value */
#define MXC_S_SKBD_CR1_DBTM_TIME11MS                   (MXC_V_SKBD_CR1_DBTM_TIME11MS << MXC_F_SKBD_CR1_DBTM_POS) /**< CR1_DBTM_TIME11MS Setting */
#define MXC_V_SKBD_CR1_DBTM_TIME12MS                   ((uint32_t)0x7UL) /**< CR1_DBTM_TIME12MS Value */
#define MXC_S_SKBD_CR1_DBTM_TIME12MS                   (MXC_V_SKBD_CR1_DBTM_TIME12MS << MXC_F_SKBD_CR1_DBTM_POS) /**< CR1_DBTM_TIME12MS Setting */

/**@} end of group SKBD_CR1_Register */

/**
 * @ingroup  skbd_registers
 * @defgroup SKBD_SR SKBD_SR
 * @brief    Status Register
 * @{
 */
#define MXC_F_SKBD_SR_BUSY_POS                         0 /**< SR_BUSY Position */
#define MXC_F_SKBD_SR_BUSY                             ((uint32_t)(0x1UL << MXC_F_SKBD_SR_BUSY_POS)) /**< SR_BUSY Mask */

/**@} end of group SKBD_SR_Register */

/**
 * @ingroup  skbd_registers
 * @defgroup SKBD_IER SKBD_IER
 * @brief    Interrupt Enable Register
 * @{
 */
#define MXC_F_SKBD_IER_PUSHIE_POS                      0 /**< IER_PUSHIE Position */
#define MXC_F_SKBD_IER_PUSHIE                          ((uint32_t)(0x1UL << MXC_F_SKBD_IER_PUSHIE_POS)) /**< IER_PUSHIE Mask */

#define MXC_F_SKBD_IER_RELEASEIE_POS                   1 /**< IER_RELEASEIE Position */
#define MXC_F_SKBD_IER_RELEASEIE                       ((uint32_t)(0x1UL << MXC_F_SKBD_IER_RELEASEIE_POS)) /**< IER_RELEASEIE Mask */

#define MXC_F_SKBD_IER_OVERIE_POS                      2 /**< IER_OVERIE Position */
#define MXC_F_SKBD_IER_OVERIE                          ((uint32_t)(0x1UL << MXC_F_SKBD_IER_OVERIE_POS)) /**< IER_OVERIE Mask */

/**@} end of group SKBD_IER_Register */

/**
 * @ingroup  skbd_registers
 * @defgroup SKBD_ISR SKBD_ISR
 * @brief    Interrupt Status Register
 * @{
 */
#define MXC_F_SKBD_ISR_PUSHIS_POS                      0 /**< ISR_PUSHIS Position */
#define MXC_F_SKBD_ISR_PUSHIS                          ((uint32_t)(0x1UL << MXC_F_SKBD_ISR_PUSHIS_POS)) /**< ISR_PUSHIS Mask */

#define MXC_F_SKBD_ISR_RELEASEIS_POS                   1 /**< ISR_RELEASEIS Position */
#define MXC_F_SKBD_ISR_RELEASEIS                       ((uint32_t)(0x1UL << MXC_F_SKBD_ISR_RELEASEIS_POS)) /**< ISR_RELEASEIS Mask */

#define MXC_F_SKBD_ISR_OVERIS_POS                      2 /**< ISR_OVERIS Position */
#define MXC_F_SKBD_ISR_OVERIS                          ((uint32_t)(0x1UL << MXC_F_SKBD_ISR_OVERIS_POS)) /**< ISR_OVERIS Mask */

/**@} end of group SKBD_ISR_Register */

/**
 * @ingroup  skbd_registers
 * @defgroup SKBD_EVENT SKBD_EVENT
 * @brief    Key Register
 * @{
 */
#define MXC_F_SKBD_EVENT_IOIN_POS                      0 /**< EVENT_IOIN Position */
#define MXC_F_SKBD_EVENT_IOIN                          ((uint32_t)(0x7UL << MXC_F_SKBD_EVENT_IOIN_POS)) /**< EVENT_IOIN Mask */

#define MXC_F_SKBD_EVENT_IOOUT_POS                     5 /**< EVENT_IOOUT Position */
#define MXC_F_SKBD_EVENT_IOOUT                         ((uint32_t)(0x7UL << MXC_F_SKBD_EVENT_IOOUT_POS)) /**< EVENT_IOOUT Mask */

#define MXC_F_SKBD_EVENT_PUSH_POS                      10 /**< EVENT_PUSH Position */
#define MXC_F_SKBD_EVENT_PUSH                          ((uint32_t)(0x1UL << MXC_F_SKBD_EVENT_PUSH_POS)) /**< EVENT_PUSH Mask */

#define MXC_F_SKBD_EVENT_READ_POS                      11 /**< EVENT_READ Position */
#define MXC_F_SKBD_EVENT_READ                          ((uint32_t)(0x1UL << MXC_F_SKBD_EVENT_READ_POS)) /**< EVENT_READ Mask */

#define MXC_F_SKBD_EVENT_NEXT_POS                      12 /**< EVENT_NEXT Position */
#define MXC_F_SKBD_EVENT_NEXT                          ((uint32_t)(0x1UL << MXC_F_SKBD_EVENT_NEXT_POS)) /**< EVENT_NEXT Mask */

/**@} end of group SKBD_EVENT_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32570_INCLUDE_SKBD_REGS_H_
