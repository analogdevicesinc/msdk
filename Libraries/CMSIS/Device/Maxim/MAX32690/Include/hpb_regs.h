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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32690_INCLUDE_HPB_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32690_INCLUDE_HPB_REGS_H_

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
#ifdef __cplusplus
#define __I volatile
#else
#define __I volatile const
#endif
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
 * @details     HyperBus Memory Controller Registers
 */

/**
 * @ingroup hpb_registers
 * Structure type to access the HPB Registers.
 */
typedef struct {
    __IO uint32_t stat;                 /**< <tt>\b 0x0000:</tt> HPB STAT Register */
    __IO uint32_t inten;                /**< <tt>\b 0x0004:</tt> HPB INTEN Register */
    __IO uint32_t intfl;                /**< <tt>\b 0x0008:</tt> HPB INTFL Register */
    __R  uint32_t rsv_0xc;
    __IO uint32_t membaddr[2];          /**< <tt>\b 0x0010:</tt> HPB MEMBADDR Register */
    __R  uint32_t rsv_0x18_0x1f[2];
    __IO uint32_t memctrl[2];           /**< <tt>\b 0x0020:</tt> HPB MEMCTRL Register */
    __R  uint32_t rsv_0x28_0x2f[2];
    __IO uint32_t memtim[2];            /**< <tt>\b 0x0030:</tt> HPB MEMTIM Register */
} mxc_hpb_regs_t;

/* Register offsets for module HPB */
/**
 * @ingroup    hpb_registers
 * @defgroup   HPB_Register_Offsets Register Offsets
 * @brief      HPB Peripheral Register Offsets from the HPB Base Peripheral Address.
 * @{
 */
#define MXC_R_HPB_STAT                     ((uint32_t)0x00000000UL) /**< Offset from HPB Base Address: <tt> 0x0000</tt> */
#define MXC_R_HPB_INTEN                    ((uint32_t)0x00000004UL) /**< Offset from HPB Base Address: <tt> 0x0004</tt> */
#define MXC_R_HPB_INTFL                    ((uint32_t)0x00000008UL) /**< Offset from HPB Base Address: <tt> 0x0008</tt> */
#define MXC_R_HPB_MEMBADDR                 ((uint32_t)0x00000010UL) /**< Offset from HPB Base Address: <tt> 0x0010</tt> */
#define MXC_R_HPB_MEMCTRL                  ((uint32_t)0x00000020UL) /**< Offset from HPB Base Address: <tt> 0x0020</tt> */
#define MXC_R_HPB_MEMTIM                   ((uint32_t)0x00000030UL) /**< Offset from HPB Base Address: <tt> 0x0030</tt> */
/**@} end of group hpb_registers */

/**
 * @ingroup  hpb_registers
 * @defgroup HPB_STAT HPB_STAT
 * @brief    Hyperbus Status Register.
 * @{
 */
#define MXC_F_HPB_STAT_RDTXN_POS                       0 /**< STAT_RDTXN Position */
#define MXC_F_HPB_STAT_RDTXN                           ((uint32_t)(0x1UL << MXC_F_HPB_STAT_RDTXN_POS)) /**< STAT_RDTXN Mask */

#define MXC_F_HPB_STAT_RDADDRERR_POS                   8 /**< STAT_RDADDRERR Position */
#define MXC_F_HPB_STAT_RDADDRERR                       ((uint32_t)(0x1UL << MXC_F_HPB_STAT_RDADDRERR_POS)) /**< STAT_RDADDRERR Mask */

#define MXC_F_HPB_STAT_RDSLVST_POS                     9 /**< STAT_RDSLVST Position */
#define MXC_F_HPB_STAT_RDSLVST                         ((uint32_t)(0x1UL << MXC_F_HPB_STAT_RDSLVST_POS)) /**< STAT_RDSLVST Mask */

#define MXC_F_HPB_STAT_RDRSTERR_POS                    10 /**< STAT_RDRSTERR Position */
#define MXC_F_HPB_STAT_RDRSTERR                        ((uint32_t)(0x1UL << MXC_F_HPB_STAT_RDRSTERR_POS)) /**< STAT_RDRSTERR Mask */

#define MXC_F_HPB_STAT_RDSTALL_POS                     11 /**< STAT_RDSTALL Position */
#define MXC_F_HPB_STAT_RDSTALL                         ((uint32_t)(0x1UL << MXC_F_HPB_STAT_RDSTALL_POS)) /**< STAT_RDSTALL Mask */

#define MXC_F_HPB_STAT_WRTXN_POS                       16 /**< STAT_WRTXN Position */
#define MXC_F_HPB_STAT_WRTXN                           ((uint32_t)(0x1UL << MXC_F_HPB_STAT_WRTXN_POS)) /**< STAT_WRTXN Mask */

#define MXC_F_HPB_STAT_WRADDRERR_POS                   24 /**< STAT_WRADDRERR Position */
#define MXC_F_HPB_STAT_WRADDRERR                       ((uint32_t)(0x1UL << MXC_F_HPB_STAT_WRADDRERR_POS)) /**< STAT_WRADDRERR Mask */

#define MXC_F_HPB_STAT_WRRSTERR_POS                    26 /**< STAT_WRRSTERR Position */
#define MXC_F_HPB_STAT_WRRSTERR                        ((uint32_t)(0x1UL << MXC_F_HPB_STAT_WRRSTERR_POS)) /**< STAT_WRRSTERR Mask */

/**@} end of group HPB_STAT_Register */

/**
 * @ingroup  hpb_registers
 * @defgroup HPB_INTEN HPB_INTEN
 * @brief    Hyperbus Interrupt Enable Register.
 * @{
 */
#define MXC_F_HPB_INTEN_MEM_POS                        0 /**< INTEN_MEM Position */
#define MXC_F_HPB_INTEN_MEM                            ((uint32_t)(0x1UL << MXC_F_HPB_INTEN_MEM_POS)) /**< INTEN_MEM Mask */

#define MXC_F_HPB_INTEN_ERR_POS                        1 /**< INTEN_ERR Position */
#define MXC_F_HPB_INTEN_ERR                            ((uint32_t)(0x1UL << MXC_F_HPB_INTEN_ERR_POS)) /**< INTEN_ERR Mask */

/**@} end of group HPB_INTEN_Register */

/**
 * @ingroup  hpb_registers
 * @defgroup HPB_INTFL HPB_INTFL
 * @brief    Hyperbus Interrupt Flag Register.
 * @{
 */
#define MXC_F_HPB_INTFL_MEM_POS                        0 /**< INTFL_MEM Position */
#define MXC_F_HPB_INTFL_MEM                            ((uint32_t)(0x1UL << MXC_F_HPB_INTFL_MEM_POS)) /**< INTFL_MEM Mask */

#define MXC_F_HPB_INTFL_ERR_POS                        1 /**< INTFL_ERR Position */
#define MXC_F_HPB_INTFL_ERR                            ((uint32_t)(0x1UL << MXC_F_HPB_INTFL_ERR_POS)) /**< INTFL_ERR Mask */

/**@} end of group HPB_INTFL_Register */

/**
 * @ingroup  hpb_registers
 * @defgroup HPB_MEMBADDR HPB_MEMBADDR
 * @brief    Hyperbus Memory Base Address Register.
 * @{
 */
#define MXC_F_HPB_MEMBADDR_ADDR_POS                    0 /**< MEMBADDR_ADDR Position */
#define MXC_F_HPB_MEMBADDR_ADDR                        ((uint32_t)(0xFFFFFFFFUL << MXC_F_HPB_MEMBADDR_ADDR_POS)) /**< MEMBADDR_ADDR Mask */

/**@} end of group HPB_MEMBADDR_Register */

/**
 * @ingroup  hpb_registers
 * @defgroup HPB_MEMCTRL HPB_MEMCTRL
 * @brief    Hyperbus Memory Control Register.
 * @{
 */
#define MXC_F_HPB_MEMCTRL_WRAPSIZE_POS                 0 /**< MEMCTRL_WRAPSIZE Position */
#define MXC_F_HPB_MEMCTRL_WRAPSIZE                     ((uint32_t)(0x3UL << MXC_F_HPB_MEMCTRL_WRAPSIZE_POS)) /**< MEMCTRL_WRAPSIZE Mask */
#define MXC_V_HPB_MEMCTRL_WRAPSIZE_64B                 ((uint32_t)0x1UL) /**< MEMCTRL_WRAPSIZE_64B Value */
#define MXC_S_HPB_MEMCTRL_WRAPSIZE_64B                 (MXC_V_HPB_MEMCTRL_WRAPSIZE_64B << MXC_F_HPB_MEMCTRL_WRAPSIZE_POS) /**< MEMCTRL_WRAPSIZE_64B Setting */
#define MXC_V_HPB_MEMCTRL_WRAPSIZE_16B                 ((uint32_t)0x2UL) /**< MEMCTRL_WRAPSIZE_16B Value */
#define MXC_S_HPB_MEMCTRL_WRAPSIZE_16B                 (MXC_V_HPB_MEMCTRL_WRAPSIZE_16B << MXC_F_HPB_MEMCTRL_WRAPSIZE_POS) /**< MEMCTRL_WRAPSIZE_16B Setting */
#define MXC_V_HPB_MEMCTRL_WRAPSIZE_32B                 ((uint32_t)0x3UL) /**< MEMCTRL_WRAPSIZE_32B Value */
#define MXC_S_HPB_MEMCTRL_WRAPSIZE_32B                 (MXC_V_HPB_MEMCTRL_WRAPSIZE_32B << MXC_F_HPB_MEMCTRL_WRAPSIZE_POS) /**< MEMCTRL_WRAPSIZE_32B Setting */

#define MXC_F_HPB_MEMCTRL_DEVTYPE_POS                  3 /**< MEMCTRL_DEVTYPE Position */
#define MXC_F_HPB_MEMCTRL_DEVTYPE                      ((uint32_t)(0x3UL << MXC_F_HPB_MEMCTRL_DEVTYPE_POS)) /**< MEMCTRL_DEVTYPE Mask */
#define MXC_V_HPB_MEMCTRL_DEVTYPE_HYPERFLASH           ((uint32_t)0x0UL) /**< MEMCTRL_DEVTYPE_HYPERFLASH Value */
#define MXC_S_HPB_MEMCTRL_DEVTYPE_HYPERFLASH           (MXC_V_HPB_MEMCTRL_DEVTYPE_HYPERFLASH << MXC_F_HPB_MEMCTRL_DEVTYPE_POS) /**< MEMCTRL_DEVTYPE_HYPERFLASH Setting */
#define MXC_V_HPB_MEMCTRL_DEVTYPE_XCCELA_PSRAM         ((uint32_t)0x1UL) /**< MEMCTRL_DEVTYPE_XCCELA_PSRAM Value */
#define MXC_S_HPB_MEMCTRL_DEVTYPE_XCCELA_PSRAM         (MXC_V_HPB_MEMCTRL_DEVTYPE_XCCELA_PSRAM << MXC_F_HPB_MEMCTRL_DEVTYPE_POS) /**< MEMCTRL_DEVTYPE_XCCELA_PSRAM Setting */
#define MXC_V_HPB_MEMCTRL_DEVTYPE_HYPERRAM             ((uint32_t)0x2UL) /**< MEMCTRL_DEVTYPE_HYPERRAM Value */
#define MXC_S_HPB_MEMCTRL_DEVTYPE_HYPERRAM             (MXC_V_HPB_MEMCTRL_DEVTYPE_HYPERRAM << MXC_F_HPB_MEMCTRL_DEVTYPE_POS) /**< MEMCTRL_DEVTYPE_HYPERRAM Setting */

#define MXC_F_HPB_MEMCTRL_CRT_POS                      5 /**< MEMCTRL_CRT Position */
#define MXC_F_HPB_MEMCTRL_CRT                          ((uint32_t)(0x1UL << MXC_F_HPB_MEMCTRL_CRT_POS)) /**< MEMCTRL_CRT Mask */

#define MXC_F_HPB_MEMCTRL_RDLAT_EN_POS                 6 /**< MEMCTRL_RDLAT_EN Position */
#define MXC_F_HPB_MEMCTRL_RDLAT_EN                     ((uint32_t)(0x1UL << MXC_F_HPB_MEMCTRL_RDLAT_EN_POS)) /**< MEMCTRL_RDLAT_EN Mask */

#define MXC_F_HPB_MEMCTRL_HSE_POS                      7 /**< MEMCTRL_HSE Position */
#define MXC_F_HPB_MEMCTRL_HSE                          ((uint32_t)(0x1UL << MXC_F_HPB_MEMCTRL_HSE_POS)) /**< MEMCTRL_HSE Mask */

#define MXC_F_HPB_MEMCTRL_MAXLEN_POS                   18 /**< MEMCTRL_MAXLEN Position */
#define MXC_F_HPB_MEMCTRL_MAXLEN                       ((uint32_t)(0x1FFUL << MXC_F_HPB_MEMCTRL_MAXLEN_POS)) /**< MEMCTRL_MAXLEN Mask */

#define MXC_F_HPB_MEMCTRL_MAX_EN_POS                   31 /**< MEMCTRL_MAX_EN Position */
#define MXC_F_HPB_MEMCTRL_MAX_EN                       ((uint32_t)(0x1UL << MXC_F_HPB_MEMCTRL_MAX_EN_POS)) /**< MEMCTRL_MAX_EN Mask */

/**@} end of group HPB_MEMCTRL_Register */

/**
 * @ingroup  hpb_registers
 * @defgroup HPB_MEMTIM HPB_MEMTIM
 * @brief    Hyperbus Memory Timing Register.
 * @{
 */
#define MXC_F_HPB_MEMTIM_LAT_POS                       0 /**< MEMTIM_LAT Position */
#define MXC_F_HPB_MEMTIM_LAT                           ((uint32_t)(0xFUL << MXC_F_HPB_MEMTIM_LAT_POS)) /**< MEMTIM_LAT Mask */
#define MXC_V_HPB_MEMTIM_LAT_5CLK                      ((uint32_t)0x0UL) /**< MEMTIM_LAT_5CLK Value */
#define MXC_S_HPB_MEMTIM_LAT_5CLK                      (MXC_V_HPB_MEMTIM_LAT_5CLK << MXC_F_HPB_MEMTIM_LAT_POS) /**< MEMTIM_LAT_5CLK Setting */
#define MXC_V_HPB_MEMTIM_LAT_6CLK                      ((uint32_t)0x1UL) /**< MEMTIM_LAT_6CLK Value */
#define MXC_S_HPB_MEMTIM_LAT_6CLK                      (MXC_V_HPB_MEMTIM_LAT_6CLK << MXC_F_HPB_MEMTIM_LAT_POS) /**< MEMTIM_LAT_6CLK Setting */
#define MXC_V_HPB_MEMTIM_LAT_3CLK                      ((uint32_t)0xEUL) /**< MEMTIM_LAT_3CLK Value */
#define MXC_S_HPB_MEMTIM_LAT_3CLK                      (MXC_V_HPB_MEMTIM_LAT_3CLK << MXC_F_HPB_MEMTIM_LAT_POS) /**< MEMTIM_LAT_3CLK Setting */
#define MXC_V_HPB_MEMTIM_LAT_4CLK                      ((uint32_t)0xFUL) /**< MEMTIM_LAT_4CLK Value */
#define MXC_S_HPB_MEMTIM_LAT_4CLK                      (MXC_V_HPB_MEMTIM_LAT_4CLK << MXC_F_HPB_MEMTIM_LAT_POS) /**< MEMTIM_LAT_4CLK Setting */

#define MXC_F_HPB_MEMTIM_WRCSHD_POS                    8 /**< MEMTIM_WRCSHD Position */
#define MXC_F_HPB_MEMTIM_WRCSHD                        ((uint32_t)(0xFUL << MXC_F_HPB_MEMTIM_WRCSHD_POS)) /**< MEMTIM_WRCSHD Mask */

#define MXC_F_HPB_MEMTIM_RDCSHD_POS                    12 /**< MEMTIM_RDCSHD Position */
#define MXC_F_HPB_MEMTIM_RDCSHD                        ((uint32_t)(0xFUL << MXC_F_HPB_MEMTIM_RDCSHD_POS)) /**< MEMTIM_RDCSHD Mask */

#define MXC_F_HPB_MEMTIM_WRCSST_POS                    16 /**< MEMTIM_WRCSST Position */
#define MXC_F_HPB_MEMTIM_WRCSST                        ((uint32_t)(0xFUL << MXC_F_HPB_MEMTIM_WRCSST_POS)) /**< MEMTIM_WRCSST Mask */

#define MXC_F_HPB_MEMTIM_RDCSST_POS                    20 /**< MEMTIM_RDCSST Position */
#define MXC_F_HPB_MEMTIM_RDCSST                        ((uint32_t)(0xFUL << MXC_F_HPB_MEMTIM_RDCSST_POS)) /**< MEMTIM_RDCSST Mask */

#define MXC_F_HPB_MEMTIM_WRCSHI_POS                    24 /**< MEMTIM_WRCSHI Position */
#define MXC_F_HPB_MEMTIM_WRCSHI                        ((uint32_t)(0xFUL << MXC_F_HPB_MEMTIM_WRCSHI_POS)) /**< MEMTIM_WRCSHI Mask */

#define MXC_F_HPB_MEMTIM_RDCSHI_POS                    28 /**< MEMTIM_RDCSHI Position */
#define MXC_F_HPB_MEMTIM_RDCSHI                        ((uint32_t)(0xFUL << MXC_F_HPB_MEMTIM_RDCSHI_POS)) /**< MEMTIM_RDCSHI Mask */

/**@} end of group HPB_MEMTIM_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32690_INCLUDE_HPB_REGS_H_
