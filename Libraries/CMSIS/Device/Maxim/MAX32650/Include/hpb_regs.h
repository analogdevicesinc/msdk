/**
 * @file    hpb_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the HPB Peripheral Module.
 * @note    This file is @generated.
 * @ingroup hpb_registers
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32650_INCLUDE_HPB_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32650_INCLUDE_HPB_REGS_H_

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
 * @ingroup     hpb
 * @defgroup    hpb_registers HPB_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the HPB Peripheral Module.
 * @details     HyperBus Memory Controller
 */

/**
 * @ingroup hpb_registers
 * Structure type to access the HPB Registers.
 */
typedef struct {
    __IO uint32_t status;               /**< <tt>\b 0x00:</tt> HPB STATUS Register */
    __IO uint32_t inten;                /**< <tt>\b 0x04:</tt> HPB INTEN Register */
    __IO uint32_t intfl;                /**< <tt>\b 0x08:</tt> HPB INTFL Register */
    __R  uint32_t rsv_0xc;
    __IO uint32_t mbr[2];               /**< <tt>\b 0x10:</tt> HPB MBR Register */
    __R  uint32_t rsv_0x18_0x1f[2];
    __IO uint32_t mcr[2];               /**< <tt>\b 0x20:</tt> HPB MCR Register */
    __R  uint32_t rsv_0x28_0x2f[2];
    __IO uint32_t mtr[2];               /**< <tt>\b 0x30:</tt> HPB MTR Register */
} mxc_hpb_regs_t;

/* Register offsets for module HPB */
/**
 * @ingroup    hpb_registers
 * @defgroup   HPB_Register_Offsets Register Offsets
 * @brief      HPB Peripheral Register Offsets from the HPB Base Peripheral Address.
 * @{
 */
#define MXC_R_HPB_STATUS                   ((uint32_t)0x00000000UL) /**< Offset from HPB Base Address: <tt> 0x0000</tt> */
#define MXC_R_HPB_INTEN                    ((uint32_t)0x00000004UL) /**< Offset from HPB Base Address: <tt> 0x0004</tt> */
#define MXC_R_HPB_INTFL                    ((uint32_t)0x00000008UL) /**< Offset from HPB Base Address: <tt> 0x0008</tt> */
#define MXC_R_HPB_MBR                      ((uint32_t)0x00000010UL) /**< Offset from HPB Base Address: <tt> 0x0010</tt> */
#define MXC_R_HPB_MCR                      ((uint32_t)0x00000020UL) /**< Offset from HPB Base Address: <tt> 0x0020</tt> */
#define MXC_R_HPB_MTR                      ((uint32_t)0x00000030UL) /**< Offset from HPB Base Address: <tt> 0x0030</tt> */
/**@} end of group hpb_registers */

/**
 * @ingroup  hpb_registers
 * @defgroup HPB_STATUS HPB_STATUS
 * @brief    HPB Status Register.
 * @{
 */
#define MXC_F_HPB_STATUS_RACT_POS                      0 /**< STATUS_RACT Position */
#define MXC_F_HPB_STATUS_RACT                          ((uint32_t)(0x1UL << MXC_F_HPB_STATUS_RACT_POS)) /**< STATUS_RACT Mask */
#define MXC_V_HPB_STATUS_RACT_NOREAD                   ((uint32_t)0x0UL) /**< STATUS_RACT_NOREAD Value */
#define MXC_S_HPB_STATUS_RACT_NOREAD                   (MXC_V_HPB_STATUS_RACT_NOREAD << MXC_F_HPB_STATUS_RACT_POS) /**< STATUS_RACT_NOREAD Setting */
#define MXC_V_HPB_STATUS_RACT_READ                     ((uint32_t)0x1UL) /**< STATUS_RACT_READ Value */
#define MXC_S_HPB_STATUS_RACT_READ                     (MXC_V_HPB_STATUS_RACT_READ << MXC_F_HPB_STATUS_RACT_POS) /**< STATUS_RACT_READ Setting */

#define MXC_F_HPB_STATUS_RDECERR_POS                   8 /**< STATUS_RDECERR Position */
#define MXC_F_HPB_STATUS_RDECERR                       ((uint32_t)(0x1UL << MXC_F_HPB_STATUS_RDECERR_POS)) /**< STATUS_RDECERR Mask */
#define MXC_V_HPB_STATUS_RDECERR_NOERR                 ((uint32_t)0x0UL) /**< STATUS_RDECERR_NOERR Value */
#define MXC_S_HPB_STATUS_RDECERR_NOERR                 (MXC_V_HPB_STATUS_RDECERR_NOERR << MXC_F_HPB_STATUS_RDECERR_POS) /**< STATUS_RDECERR_NOERR Setting */
#define MXC_V_HPB_STATUS_RDECERR_ERR                   ((uint32_t)0x1UL) /**< STATUS_RDECERR_ERR Value */
#define MXC_S_HPB_STATUS_RDECERR_ERR                   (MXC_V_HPB_STATUS_RDECERR_ERR << MXC_F_HPB_STATUS_RDECERR_POS) /**< STATUS_RDECERR_ERR Setting */

#define MXC_F_HPB_STATUS_RRSTOERR_POS                  10 /**< STATUS_RRSTOERR Position */
#define MXC_F_HPB_STATUS_RRSTOERR                      ((uint32_t)(0x1UL << MXC_F_HPB_STATUS_RRSTOERR_POS)) /**< STATUS_RRSTOERR Mask */
#define MXC_V_HPB_STATUS_RRSTOERR_NOERR                ((uint32_t)0x0UL) /**< STATUS_RRSTOERR_NOERR Value */
#define MXC_S_HPB_STATUS_RRSTOERR_NOERR                (MXC_V_HPB_STATUS_RRSTOERR_NOERR << MXC_F_HPB_STATUS_RRSTOERR_POS) /**< STATUS_RRSTOERR_NOERR Setting */
#define MXC_V_HPB_STATUS_RRSTOERR_ERR                  ((uint32_t)0x1UL) /**< STATUS_RRSTOERR_ERR Value */
#define MXC_S_HPB_STATUS_RRSTOERR_ERR                  (MXC_V_HPB_STATUS_RRSTOERR_ERR << MXC_F_HPB_STATUS_RRSTOERR_POS) /**< STATUS_RRSTOERR_ERR Setting */

#define MXC_F_HPB_STATUS_RDSSTALL_POS                  11 /**< STATUS_RDSSTALL Position */
#define MXC_F_HPB_STATUS_RDSSTALL                      ((uint32_t)(0x1UL << MXC_F_HPB_STATUS_RDSSTALL_POS)) /**< STATUS_RDSSTALL Mask */
#define MXC_V_HPB_STATUS_RDSSTALL_NORMALOP             ((uint32_t)0x0UL) /**< STATUS_RDSSTALL_NORMALOP Value */
#define MXC_S_HPB_STATUS_RDSSTALL_NORMALOP             (MXC_V_HPB_STATUS_RDSSTALL_NORMALOP << MXC_F_HPB_STATUS_RDSSTALL_POS) /**< STATUS_RDSSTALL_NORMALOP Setting */
#define MXC_V_HPB_STATUS_RDSSTALL_STALLED              ((uint32_t)0x1UL) /**< STATUS_RDSSTALL_STALLED Value */
#define MXC_S_HPB_STATUS_RDSSTALL_STALLED              (MXC_V_HPB_STATUS_RDSSTALL_STALLED << MXC_F_HPB_STATUS_RDSSTALL_POS) /**< STATUS_RDSSTALL_STALLED Setting */

#define MXC_F_HPB_STATUS_WACT_POS                      16 /**< STATUS_WACT Position */
#define MXC_F_HPB_STATUS_WACT                          ((uint32_t)(0x1UL << MXC_F_HPB_STATUS_WACT_POS)) /**< STATUS_WACT Mask */
#define MXC_V_HPB_STATUS_WACT_NOWRITE                  ((uint32_t)0x0UL) /**< STATUS_WACT_NOWRITE Value */
#define MXC_S_HPB_STATUS_WACT_NOWRITE                  (MXC_V_HPB_STATUS_WACT_NOWRITE << MXC_F_HPB_STATUS_WACT_POS) /**< STATUS_WACT_NOWRITE Setting */
#define MXC_V_HPB_STATUS_WACT_WRITE                    ((uint32_t)0x1UL) /**< STATUS_WACT_WRITE Value */
#define MXC_S_HPB_STATUS_WACT_WRITE                    (MXC_V_HPB_STATUS_WACT_WRITE << MXC_F_HPB_STATUS_WACT_POS) /**< STATUS_WACT_WRITE Setting */

#define MXC_F_HPB_STATUS_WDECERR_POS                   24 /**< STATUS_WDECERR Position */
#define MXC_F_HPB_STATUS_WDECERR                       ((uint32_t)(0x1UL << MXC_F_HPB_STATUS_WDECERR_POS)) /**< STATUS_WDECERR Mask */
#define MXC_V_HPB_STATUS_WDECERR_NOERR                 ((uint32_t)0x0UL) /**< STATUS_WDECERR_NOERR Value */
#define MXC_S_HPB_STATUS_WDECERR_NOERR                 (MXC_V_HPB_STATUS_WDECERR_NOERR << MXC_F_HPB_STATUS_WDECERR_POS) /**< STATUS_WDECERR_NOERR Setting */
#define MXC_V_HPB_STATUS_WDECERR_ERR                   ((uint32_t)0x1UL) /**< STATUS_WDECERR_ERR Value */
#define MXC_S_HPB_STATUS_WDECERR_ERR                   (MXC_V_HPB_STATUS_WDECERR_ERR << MXC_F_HPB_STATUS_WDECERR_POS) /**< STATUS_WDECERR_ERR Setting */

#define MXC_F_HPB_STATUS_WRSTOERR_POS                  26 /**< STATUS_WRSTOERR Position */
#define MXC_F_HPB_STATUS_WRSTOERR                      ((uint32_t)(0x1UL << MXC_F_HPB_STATUS_WRSTOERR_POS)) /**< STATUS_WRSTOERR Mask */
#define MXC_V_HPB_STATUS_WRSTOERR_NOERR                ((uint32_t)0x0UL) /**< STATUS_WRSTOERR_NOERR Value */
#define MXC_S_HPB_STATUS_WRSTOERR_NOERR                (MXC_V_HPB_STATUS_WRSTOERR_NOERR << MXC_F_HPB_STATUS_WRSTOERR_POS) /**< STATUS_WRSTOERR_NOERR Setting */
#define MXC_V_HPB_STATUS_WRSTOERR_ERR                  ((uint32_t)0x1UL) /**< STATUS_WRSTOERR_ERR Value */
#define MXC_S_HPB_STATUS_WRSTOERR_ERR                  (MXC_V_HPB_STATUS_WRSTOERR_ERR << MXC_F_HPB_STATUS_WRSTOERR_POS) /**< STATUS_WRSTOERR_ERR Setting */

/**@} end of group HPB_STATUS_Register */

/**
 * @ingroup  hpb_registers
 * @defgroup HPB_INTEN HPB_INTEN
 * @brief    HPB Interrupt Enable.
 * @{
 */
#define MXC_F_HPB_INTEN_ERRINTE_POS                    1 /**< INTEN_ERRINTE Position */
#define MXC_F_HPB_INTEN_ERRINTE                        ((uint32_t)(0x1UL << MXC_F_HPB_INTEN_ERRINTE_POS)) /**< INTEN_ERRINTE Mask */
#define MXC_V_HPB_INTEN_ERRINTE_DIS                    ((uint32_t)0x0UL) /**< INTEN_ERRINTE_DIS Value */
#define MXC_S_HPB_INTEN_ERRINTE_DIS                    (MXC_V_HPB_INTEN_ERRINTE_DIS << MXC_F_HPB_INTEN_ERRINTE_POS) /**< INTEN_ERRINTE_DIS Setting */
#define MXC_V_HPB_INTEN_ERRINTE_EN                     ((uint32_t)0x1UL) /**< INTEN_ERRINTE_EN Value */
#define MXC_S_HPB_INTEN_ERRINTE_EN                     (MXC_V_HPB_INTEN_ERRINTE_EN << MXC_F_HPB_INTEN_ERRINTE_POS) /**< INTEN_ERRINTE_EN Setting */

/**@} end of group HPB_INTEN_Register */

/**
 * @ingroup  hpb_registers
 * @defgroup HPB_INTFL HPB_INTFL
 * @brief    HPB Interrupt Status Flags.
 * @{
 */
#define MXC_F_HPB_INTFL_ERRINT_POS                     1 /**< INTFL_ERRINT Position */
#define MXC_F_HPB_INTFL_ERRINT                         ((uint32_t)(0x1UL << MXC_F_HPB_INTFL_ERRINT_POS)) /**< INTFL_ERRINT Mask */
#define MXC_V_HPB_INTFL_ERRINT_NOINT                   ((uint32_t)0x0UL) /**< INTFL_ERRINT_NOINT Value */
#define MXC_S_HPB_INTFL_ERRINT_NOINT                   (MXC_V_HPB_INTFL_ERRINT_NOINT << MXC_F_HPB_INTFL_ERRINT_POS) /**< INTFL_ERRINT_NOINT Setting */
#define MXC_V_HPB_INTFL_ERRINT_PENDING                 ((uint32_t)0x1UL) /**< INTFL_ERRINT_PENDING Value */
#define MXC_S_HPB_INTFL_ERRINT_PENDING                 (MXC_V_HPB_INTFL_ERRINT_PENDING << MXC_F_HPB_INTFL_ERRINT_POS) /**< INTFL_ERRINT_PENDING Setting */

/**@} end of group HPB_INTFL_Register */

/**
 * @ingroup  hpb_registers
 * @defgroup HPB_MBR HPB_MBR
 * @brief    HPB Memory Base Address.
 * @{
 */
#define MXC_F_HPB_MBR_ADDR_POS                         24 /**< MBR_ADDR Position */
#define MXC_F_HPB_MBR_ADDR                             ((uint32_t)(0xFFUL << MXC_F_HPB_MBR_ADDR_POS)) /**< MBR_ADDR Mask */

/**@} end of group HPB_MBR_Register */

/**
 * @ingroup  hpb_registers
 * @defgroup HPB_MCR HPB_MCR
 * @brief    HPB Memory Configuration Register.
 * @{
 */
#define MXC_F_HPB_MCR_DEV_TYPE_POS                     3 /**< MCR_DEV_TYPE Position */
#define MXC_F_HPB_MCR_DEV_TYPE                         ((uint32_t)(0x3UL << MXC_F_HPB_MCR_DEV_TYPE_POS)) /**< MCR_DEV_TYPE Mask */
#define MXC_V_HPB_MCR_DEV_TYPE_HYPERFLASH              ((uint32_t)0x0UL) /**< MCR_DEV_TYPE_HYPERFLASH Value */
#define MXC_S_HPB_MCR_DEV_TYPE_HYPERFLASH              (MXC_V_HPB_MCR_DEV_TYPE_HYPERFLASH << MXC_F_HPB_MCR_DEV_TYPE_POS) /**< MCR_DEV_TYPE_HYPERFLASH Setting */
#define MXC_V_HPB_MCR_DEV_TYPE_XCCELAPSRAM             ((uint32_t)0x1UL) /**< MCR_DEV_TYPE_XCCELAPSRAM Value */
#define MXC_S_HPB_MCR_DEV_TYPE_XCCELAPSRAM             (MXC_V_HPB_MCR_DEV_TYPE_XCCELAPSRAM << MXC_F_HPB_MCR_DEV_TYPE_POS) /**< MCR_DEV_TYPE_XCCELAPSRAM Setting */
#define MXC_V_HPB_MCR_DEV_TYPE_HYPERRAM                ((uint32_t)0x2UL) /**< MCR_DEV_TYPE_HYPERRAM Value */
#define MXC_S_HPB_MCR_DEV_TYPE_HYPERRAM                (MXC_V_HPB_MCR_DEV_TYPE_HYPERRAM << MXC_F_HPB_MCR_DEV_TYPE_POS) /**< MCR_DEV_TYPE_HYPERRAM Setting */

#define MXC_F_HPB_MCR_CRT_POS                          5 /**< MCR_CRT Position */
#define MXC_F_HPB_MCR_CRT                              ((uint32_t)(0x1UL << MXC_F_HPB_MCR_CRT_POS)) /**< MCR_CRT Mask */
#define MXC_V_HPB_MCR_CRT_MEM_SPACE                    ((uint32_t)0x0UL) /**< MCR_CRT_MEM_SPACE Value */
#define MXC_S_HPB_MCR_CRT_MEM_SPACE                    (MXC_V_HPB_MCR_CRT_MEM_SPACE << MXC_F_HPB_MCR_CRT_POS) /**< MCR_CRT_MEM_SPACE Setting */
#define MXC_V_HPB_MCR_CRT_CONFIG_REG_SPACE             ((uint32_t)0x1UL) /**< MCR_CRT_CONFIG_REG_SPACE Value */
#define MXC_S_HPB_MCR_CRT_CONFIG_REG_SPACE             (MXC_V_HPB_MCR_CRT_CONFIG_REG_SPACE << MXC_F_HPB_MCR_CRT_POS) /**< MCR_CRT_CONFIG_REG_SPACE Setting */

#define MXC_F_HPB_MCR_READ_LATENCY_POS                 6 /**< MCR_READ_LATENCY Position */
#define MXC_F_HPB_MCR_READ_LATENCY                     ((uint32_t)(0x1UL << MXC_F_HPB_MCR_READ_LATENCY_POS)) /**< MCR_READ_LATENCY Mask */
#define MXC_V_HPB_MCR_READ_LATENCY_VARIABLE            ((uint32_t)0x0UL) /**< MCR_READ_LATENCY_VARIABLE Value */
#define MXC_S_HPB_MCR_READ_LATENCY_VARIABLE            (MXC_V_HPB_MCR_READ_LATENCY_VARIABLE << MXC_F_HPB_MCR_READ_LATENCY_POS) /**< MCR_READ_LATENCY_VARIABLE Setting */
#define MXC_V_HPB_MCR_READ_LATENCY_FIXED               ((uint32_t)0x1UL) /**< MCR_READ_LATENCY_FIXED Value */
#define MXC_S_HPB_MCR_READ_LATENCY_FIXED               (MXC_V_HPB_MCR_READ_LATENCY_FIXED << MXC_F_HPB_MCR_READ_LATENCY_POS) /**< MCR_READ_LATENCY_FIXED Setting */

#define MXC_F_HPB_MCR_HSE_POS                          7 /**< MCR_HSE Position */
#define MXC_F_HPB_MCR_HSE                              ((uint32_t)(0x1UL << MXC_F_HPB_MCR_HSE_POS)) /**< MCR_HSE Mask */
#define MXC_V_HPB_MCR_HSE_DIS                          ((uint32_t)0x0UL) /**< MCR_HSE_DIS Value */
#define MXC_S_HPB_MCR_HSE_DIS                          (MXC_V_HPB_MCR_HSE_DIS << MXC_F_HPB_MCR_HSE_POS) /**< MCR_HSE_DIS Setting */
#define MXC_V_HPB_MCR_HSE_EN                           ((uint32_t)0x1UL) /**< MCR_HSE_EN Value */
#define MXC_S_HPB_MCR_HSE_EN                           (MXC_V_HPB_MCR_HSE_EN << MXC_F_HPB_MCR_HSE_POS) /**< MCR_HSE_EN Setting */

#define MXC_F_HPB_MCR_MAXLEN_POS                       18 /**< MCR_MAXLEN Position */
#define MXC_F_HPB_MCR_MAXLEN                           ((uint32_t)(0x1FFUL << MXC_F_HPB_MCR_MAXLEN_POS)) /**< MCR_MAXLEN Mask */

#define MXC_F_HPB_MCR_MAXLEN_EN_POS                    31 /**< MCR_MAXLEN_EN Position */
#define MXC_F_HPB_MCR_MAXLEN_EN                        ((uint32_t)(0x1UL << MXC_F_HPB_MCR_MAXLEN_EN_POS)) /**< MCR_MAXLEN_EN Mask */
#define MXC_V_HPB_MCR_MAXLEN_EN_DIS                    ((uint32_t)0x0UL) /**< MCR_MAXLEN_EN_DIS Value */
#define MXC_S_HPB_MCR_MAXLEN_EN_DIS                    (MXC_V_HPB_MCR_MAXLEN_EN_DIS << MXC_F_HPB_MCR_MAXLEN_EN_POS) /**< MCR_MAXLEN_EN_DIS Setting */
#define MXC_V_HPB_MCR_MAXLEN_EN_EN                     ((uint32_t)0x1UL) /**< MCR_MAXLEN_EN_EN Value */
#define MXC_S_HPB_MCR_MAXLEN_EN_EN                     (MXC_V_HPB_MCR_MAXLEN_EN_EN << MXC_F_HPB_MCR_MAXLEN_EN_POS) /**< MCR_MAXLEN_EN_EN Setting */

/**@} end of group HPB_MCR_Register */

/**
 * @ingroup  hpb_registers
 * @defgroup HPB_MTR HPB_MTR
 * @brief    HPB Memory Timing Register.
 * @{
 */
#define MXC_F_HPB_MTR_LATENCY_POS                      0 /**< MTR_LATENCY Position */
#define MXC_F_HPB_MTR_LATENCY                          ((uint32_t)(0xFUL << MXC_F_HPB_MTR_LATENCY_POS)) /**< MTR_LATENCY Mask */
#define MXC_V_HPB_MTR_LATENCY_5CLK                     ((uint32_t)0x0UL) /**< MTR_LATENCY_5CLK Value */
#define MXC_S_HPB_MTR_LATENCY_5CLK                     (MXC_V_HPB_MTR_LATENCY_5CLK << MXC_F_HPB_MTR_LATENCY_POS) /**< MTR_LATENCY_5CLK Setting */
#define MXC_V_HPB_MTR_LATENCY_6CLK                     ((uint32_t)0x1UL) /**< MTR_LATENCY_6CLK Value */
#define MXC_S_HPB_MTR_LATENCY_6CLK                     (MXC_V_HPB_MTR_LATENCY_6CLK << MXC_F_HPB_MTR_LATENCY_POS) /**< MTR_LATENCY_6CLK Setting */
#define MXC_V_HPB_MTR_LATENCY_3CLK                     ((uint32_t)0xEUL) /**< MTR_LATENCY_3CLK Value */
#define MXC_S_HPB_MTR_LATENCY_3CLK                     (MXC_V_HPB_MTR_LATENCY_3CLK << MXC_F_HPB_MTR_LATENCY_POS) /**< MTR_LATENCY_3CLK Setting */
#define MXC_V_HPB_MTR_LATENCY_4CLK                     ((uint32_t)0xFUL) /**< MTR_LATENCY_4CLK Value */
#define MXC_S_HPB_MTR_LATENCY_4CLK                     (MXC_V_HPB_MTR_LATENCY_4CLK << MXC_F_HPB_MTR_LATENCY_POS) /**< MTR_LATENCY_4CLK Setting */

#define MXC_F_HPB_MTR_WCSH_POS                         8 /**< MTR_WCSH Position */
#define MXC_F_HPB_MTR_WCSH                             ((uint32_t)(0xFUL << MXC_F_HPB_MTR_WCSH_POS)) /**< MTR_WCSH Mask */

#define MXC_F_HPB_MTR_RCSH_POS                         12 /**< MTR_RCSH Position */
#define MXC_F_HPB_MTR_RCSH                             ((uint32_t)(0xFUL << MXC_F_HPB_MTR_RCSH_POS)) /**< MTR_RCSH Mask */

#define MXC_F_HPB_MTR_WCSS_POS                         16 /**< MTR_WCSS Position */
#define MXC_F_HPB_MTR_WCSS                             ((uint32_t)(0xFUL << MXC_F_HPB_MTR_WCSS_POS)) /**< MTR_WCSS Mask */

#define MXC_F_HPB_MTR_RCSS_POS                         20 /**< MTR_RCSS Position */
#define MXC_F_HPB_MTR_RCSS                             ((uint32_t)(0xFUL << MXC_F_HPB_MTR_RCSS_POS)) /**< MTR_RCSS Mask */

#define MXC_F_HPB_MTR_WCSHI_POS                        24 /**< MTR_WCSHI Position */
#define MXC_F_HPB_MTR_WCSHI                            ((uint32_t)(0xFUL << MXC_F_HPB_MTR_WCSHI_POS)) /**< MTR_WCSHI Mask */

#define MXC_F_HPB_MTR_RCSHI_POS                        28 /**< MTR_RCSHI Position */
#define MXC_F_HPB_MTR_RCSHI                            ((uint32_t)(0xFUL << MXC_F_HPB_MTR_RCSHI_POS)) /**< MTR_RCSHI Mask */

/**@} end of group HPB_MTR_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32650_INCLUDE_HPB_REGS_H_
