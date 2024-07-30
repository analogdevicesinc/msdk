/**
 * @file    sema_reva_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the SEMA_REVA Peripheral Module.
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

#ifndef _SEMA_REVA_REGS_H_
#define _SEMA_REVA_REGS_H_

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
 * @ingroup     sema_reva
 * @defgroup    sema_reva_registers SEMA_REVA_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the SEMA_REVA Peripheral Module.
 * @details The Semaphore peripheral allows multiple cores in a system to cooperate when accessing shred resources.
                                     The peripheral contains eight semaphores that can be atomically set and cleared. It is left to the discretion of the software
                                     architect to decide how and when the semaphores are used and how they are allocated. Existing hardware does not have to be

                                     modified for this type of cooperative sharing, and the use of semaphores is exclusively within the software domain.
 */

/**
 * @ingroup sema_reva_registers
 * Structure type to access the SEMA_REVA Registers.
 */
typedef struct {
    __IO uint32_t semaphores[8];        /**< <tt>\b 0x00:</tt> SEMA SEMAPHORES Register */
    __R  uint32_t rsv_0x20_0x3f[8];
    __IO uint32_t irq0;                 /**< <tt>\b 0x40:</tt> SEMA IRQ0 Register */
    __IO uint32_t mail0;                /**< <tt>\b 0x44:</tt> SEMA MAIL0 Register */
    __IO uint32_t irq1;                 /**< <tt>\b 0x48:</tt> SEMA IRQ1 Register */
    __IO uint32_t mail1;                /**< <tt>\b 0x4C:</tt> SEMA MAIL1 Register */
    __R  uint32_t rsv_0x50_0xff[44];
    __IO uint32_t status;               /**< <tt>\b 0x100:</tt> SEMA STATUS Register */
} mxc_sema_reva_regs_t;

/* Register offsets for module SEMA_REVA */
/**
 * @ingroup    sema_reva_registers
 * @defgroup   SEMA_REVA_Register_Offsets Register Offsets
 * @brief      SEMA_REVA Peripheral Register Offsets from the SEMA_REVA Base Peripheral Address.
 * @{
 */
 #define MXC_R_SEMA_REVA_SEMAPHORES              ((uint32_t)0x00000000UL) /**< Offset from SEMA Base Address: <tt> 0x0000</tt> */
 #define MXC_R_SEMA_REVA_IRQ0                    ((uint32_t)0x00000040UL) /**< Offset from SEMA Base Address: <tt> 0x0040</tt> */
 #define MXC_R_SEMA_REVA_MAIL0                   ((uint32_t)0x00000044UL) /**< Offset from SEMA Base Address: <tt> 0x0044</tt> */
 #define MXC_R_SEMA_REVA_IRQ1                    ((uint32_t)0x00000048UL) /**< Offset from SEMA Base Address: <tt> 0x0048</tt> */
 #define MXC_R_SEMA_REVA_MAIL1                   ((uint32_t)0x0000004CUL) /**< Offset from SEMA Base Address: <tt> 0x004C</tt> */
 #define MXC_R_SEMA_REVA_STATUS                  ((uint32_t)0x00000100UL) /**< Offset from SEMA Base Address: <tt> 0x0100</tt> */
/**@} end of group sema_registers */

/**
 * @ingroup  sema_reva_registers
 * @defgroup SEMA_REVA_SEMAPHORES SEMA_REVA_SEMAPHORES
 * @brief    Read to test and set, returns prior value. Write 0 to clear semaphore.
 * @{
 */
 #define MXC_F_SEMA_REVA_SEMAPHORES_SEMA_POS            0 /**< SEMAPHORES_SEMA Position */
 #define MXC_F_SEMA_REVA_SEMAPHORES_SEMA                ((uint32_t)(0x1UL << MXC_F_SEMA_REVA_SEMAPHORES_SEMA_POS)) /**< SEMAPHORES_SEMA Mask */

/**@} end of group SEMA_REVA_SEMAPHORES_Register */

/**
 * @ingroup  sema_registers
 * @defgroup SEMA_REVA_IRQ0 SEMA_REVA_IRQ0
 * @brief    Semaphore IRQ0 register.
 * @{
 */
 #define MXC_F_SEMA_REVA_IRQ0_EN_POS                         0 /**< IRQ0_EN Position */
 #define MXC_F_SEMA_REVA_IRQ0_EN                             ((uint32_t)(0x1UL << MXC_F_SEMA_REVA_IRQ0_EN_POS)) /**< IRQ0_EN Mask */

 #define MXC_F_SEMA_REVA_IRQ0_CM4_IRQ_POS                    16 /**< IRQ0_CM4_IRQ Position */
 #define MXC_F_SEMA_REVA_IRQ0_CM4_IRQ                        ((uint32_t)(0x1UL << MXC_F_SEMA_REVA_IRQ0_CM4_IRQ_POS)) /**< IRQ0_CM4_IRQ Mask */

/**@} end of group SEMA_REVA_IRQ0_Register */

/**
 * @ingroup  sema_registers
 * @defgroup SEMA_REVA_MAIL0 SEMA_REVA_MAIL0
 * @brief    Semaphore Mailbox 0 register.
 * @{
 */
 #define MXC_F_SEMA_REVA_MAIL0_DATA_POS                      0 /**< MAIL0_DATA Position */
 #define MXC_F_SEMA_REVA_MAIL0_DATA                          ((uint32_t)(0xFFFFFFFFUL << MXC_F_SEMA_REVA_MAIL0_DATA_POS)) /**< MAIL0_DATA Mask */

/**@} end of group SEMA_REVA_MAIL0_Register */

/**
 * @ingroup  sema_registers
 * @defgroup SEMA_REVA_IRQ1 SEMA_REVA_IRQ1
 * @brief    Semaphore IRQ1 register.
 * @{
 */
 #define MXC_F_SEMA_REVA_IRQ1_EN_POS                         0 /**< IRQ1_EN Position */
 #define MXC_F_SEMA_REVA_IRQ1_EN                             ((uint32_t)(0x1UL << MXC_F_SEMA_REVA_IRQ1_EN_POS)) /**< IRQ1_EN Mask */

 #define MXC_F_SEMA_REVA_IRQ1_RV32_IRQ_POS                   16 /**< IRQ1_RV32_IRQ Position */
 #define MXC_F_SEMA_REVA_IRQ1_RV32_IRQ                       ((uint32_t)(0x1UL << MXC_F_SEMA_REVA_IRQ1_RV32_IRQ_POS)) /**< IRQ1_RV32_IRQ Mask */

/**@} end of group SEMA_REVA_IRQ1_Register */

/**
 * @ingroup  sema_registers
 * @defgroup SEMA_REVA_MAIL1 SEMA_REVA_MAIL1
 * @brief    Semaphore Mailbox 1 register.
 * @{
 */
 #define MXC_F_SEMA_REVA_MAIL1_DATA_POS                      0 /**< MAIL1_DATA Position */
 #define MXC_F_SEMA_REVA_MAIL1_DATA                          ((uint32_t)(0xFFFFFFFFUL << MXC_F_SEMA_REVA_MAIL1_DATA_POS)) /**< MAIL1_DATA Mask */

/**@} end of group SEMA_REVA_MAIL1_Register */

/**
 * @ingroup  sema_registers
 * @defgroup SEMA_REVA_STATUS SEMA_REVA_STATUS
 * @brief    Semaphore status bits. 0 indicates the semaphore is free, 1 indicates taken.
 * @{
 */
 #define MXC_F_SEMA_REVA_STATUS_STATUS_POS              0 /**< STATUS_STATUS Position */
 #define MXC_F_SEMA_REVA_STATUS_STATUS                  ((uint32_t)(0xFFUL << MXC_F_SEMA_REVA_STATUS_STATUS_POS)) /**< STATUS_STATUS Mask */

/**@} end of group SEMA_REVA_STATUS_Register */

#ifdef __cplusplus
}
#endif

#endif /* _SEMA_REVA_REGS_H_ */
