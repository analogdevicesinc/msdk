/**
 * @file    gcr_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the GCR Peripheral Module.
 * @note    This file is @generated.
 * @ingroup gcr_registers
 */

/******************************************************************************
 *
 * Copyright (C) 2024-2025 Analog Devices, Inc.
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_GCR_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_GCR_REGS_H_

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
 * @ingroup     gcr
 * @defgroup    gcr_registers GCR_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the GCR Peripheral Module.
 * @details     Global Control Registers.
 */

/**
 * @ingroup gcr_registers
 * Structure type to access the GCR Registers.
 */
typedef struct {
    __IO uint32_t sysctrl;              /**< <tt>\b 0x00:</tt> GCR SYSCTRL Register */
    __IO uint32_t rst0;                 /**< <tt>\b 0x04:</tt> GCR RST0 Register */
    __IO uint32_t clkctrl;              /**< <tt>\b 0x08:</tt> GCR CLKCTRL Register */
    __IO uint32_t pm;                   /**< <tt>\b 0x0C:</tt> GCR PM Register */
    __R  uint32_t rsv_0x10_0x17[2];
    __IO uint32_t pclkdiv;              /**< <tt>\b 0x18:</tt> GCR PCLKDIV Register */
    __R  uint32_t rsv_0x1c_0x23[2];
    __IO uint32_t pclkdis0;             /**< <tt>\b 0x24:</tt> GCR PCLKDIS0 Register */
    __IO uint32_t memctrl;              /**< <tt>\b 0x28:</tt> GCR MEMCTRL Register */
    __IO uint32_t memz;                 /**< <tt>\b 0x2C:</tt> GCR MEMZ Register */
    __R  uint32_t rsv_0x30_0x3f[4];
    __IO uint32_t sysst;                /**< <tt>\b 0x40:</tt> GCR SYSST Register */
    __IO uint32_t rst1;                 /**< <tt>\b 0x44:</tt> GCR RST1 Register */
    __IO uint32_t pclkdis1;             /**< <tt>\b 0x48:</tt> GCR PCLKDIS1 Register */
    __IO uint32_t eventen;              /**< <tt>\b 0x4C:</tt> GCR EVENTEN Register */
    __I  uint32_t revision;             /**< <tt>\b 0x50:</tt> GCR REVISION Register */
    __IO uint32_t sysinten;             /**< <tt>\b 0x54:</tt> GCR SYSINTEN Register */
    __R  uint32_t rsv_0x58_0x63[3];
    __IO uint32_t eccerr;               /**< <tt>\b 0x64:</tt> GCR ECCERR Register */
    __IO uint32_t eccced;               /**< <tt>\b 0x68:</tt> GCR ECCCED Register */
    __IO uint32_t eccinten;             /**< <tt>\b 0x6C:</tt> GCR ECCINTEN Register */
    __IO uint32_t eccaddr;              /**< <tt>\b 0x70:</tt> GCR ECCADDR Register */
    __IO uint32_t btleldoctrl;          /**< <tt>\b 0x74:</tt> GCR BTLELDOCTRL Register */
    __IO uint32_t btleldodly;           /**< <tt>\b 0x78:</tt> GCR BTLELDODLY Register */
    __R  uint32_t rsv_0x7c;
    __IO uint32_t gpr;                  /**< <tt>\b 0x80:</tt> GCR GPR Register */
} mxc_gcr_regs_t;

/* Register offsets for module GCR */
/**
 * @ingroup    gcr_registers
 * @defgroup   GCR_Register_Offsets Register Offsets
 * @brief      GCR Peripheral Register Offsets from the GCR Base Peripheral Address.
 * @{
 */
#define MXC_R_GCR_SYSCTRL                  ((uint32_t)0x00000000UL) /**< Offset from GCR Base Address: <tt> 0x0000</tt> */
#define MXC_R_GCR_RST0                     ((uint32_t)0x00000004UL) /**< Offset from GCR Base Address: <tt> 0x0004</tt> */
#define MXC_R_GCR_CLKCTRL                  ((uint32_t)0x00000008UL) /**< Offset from GCR Base Address: <tt> 0x0008</tt> */
#define MXC_R_GCR_PM                       ((uint32_t)0x0000000CUL) /**< Offset from GCR Base Address: <tt> 0x000C</tt> */
#define MXC_R_GCR_PCLKDIV                  ((uint32_t)0x00000018UL) /**< Offset from GCR Base Address: <tt> 0x0018</tt> */
#define MXC_R_GCR_PCLKDIS0                 ((uint32_t)0x00000024UL) /**< Offset from GCR Base Address: <tt> 0x0024</tt> */
#define MXC_R_GCR_MEMCTRL                  ((uint32_t)0x00000028UL) /**< Offset from GCR Base Address: <tt> 0x0028</tt> */
#define MXC_R_GCR_MEMZ                     ((uint32_t)0x0000002CUL) /**< Offset from GCR Base Address: <tt> 0x002C</tt> */
#define MXC_R_GCR_SYSST                    ((uint32_t)0x00000040UL) /**< Offset from GCR Base Address: <tt> 0x0040</tt> */
#define MXC_R_GCR_RST1                     ((uint32_t)0x00000044UL) /**< Offset from GCR Base Address: <tt> 0x0044</tt> */
#define MXC_R_GCR_PCLKDIS1                 ((uint32_t)0x00000048UL) /**< Offset from GCR Base Address: <tt> 0x0048</tt> */
#define MXC_R_GCR_EVENTEN                  ((uint32_t)0x0000004CUL) /**< Offset from GCR Base Address: <tt> 0x004C</tt> */
#define MXC_R_GCR_REVISION                 ((uint32_t)0x00000050UL) /**< Offset from GCR Base Address: <tt> 0x0050</tt> */
#define MXC_R_GCR_SYSINTEN                 ((uint32_t)0x00000054UL) /**< Offset from GCR Base Address: <tt> 0x0054</tt> */
#define MXC_R_GCR_ECCERR                   ((uint32_t)0x00000064UL) /**< Offset from GCR Base Address: <tt> 0x0064</tt> */
#define MXC_R_GCR_ECCCED                   ((uint32_t)0x00000068UL) /**< Offset from GCR Base Address: <tt> 0x0068</tt> */
#define MXC_R_GCR_ECCINTEN                 ((uint32_t)0x0000006CUL) /**< Offset from GCR Base Address: <tt> 0x006C</tt> */
#define MXC_R_GCR_ECCADDR                  ((uint32_t)0x00000070UL) /**< Offset from GCR Base Address: <tt> 0x0070</tt> */
#define MXC_R_GCR_BTLELDOCTRL              ((uint32_t)0x00000074UL) /**< Offset from GCR Base Address: <tt> 0x0074</tt> */
#define MXC_R_GCR_BTLELDODLY               ((uint32_t)0x00000078UL) /**< Offset from GCR Base Address: <tt> 0x0078</tt> */
#define MXC_R_GCR_GPR                      ((uint32_t)0x00000080UL) /**< Offset from GCR Base Address: <tt> 0x0080</tt> */
/**@} end of group gcr_registers */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_SYSCTRL GCR_SYSCTRL
 * @brief    System Control.
 * @{
 */
#define MXC_F_GCR_SYSCTRL_ICC_FLUSH_POS                6 /**< SYSCTRL_ICC_FLUSH Position */
#define MXC_F_GCR_SYSCTRL_ICC_FLUSH                    ((uint32_t)(0x1UL << MXC_F_GCR_SYSCTRL_ICC_FLUSH_POS)) /**< SYSCTRL_ICC_FLUSH Mask */

#define MXC_F_GCR_SYSCTRL_CCHK_POS                     13 /**< SYSCTRL_CCHK Position */
#define MXC_F_GCR_SYSCTRL_CCHK                         ((uint32_t)(0x1UL << MXC_F_GCR_SYSCTRL_CCHK_POS)) /**< SYSCTRL_CCHK Mask */

#define MXC_F_GCR_SYSCTRL_CHKRES_POS                   15 /**< SYSCTRL_CHKRES Position */
#define MXC_F_GCR_SYSCTRL_CHKRES                       ((uint32_t)(0x1UL << MXC_F_GCR_SYSCTRL_CHKRES_POS)) /**< SYSCTRL_CHKRES Mask */

#define MXC_F_GCR_SYSCTRL_OVR_POS                      16 /**< SYSCTRL_OVR Position */
#define MXC_F_GCR_SYSCTRL_OVR                          ((uint32_t)(0x3UL << MXC_F_GCR_SYSCTRL_OVR_POS)) /**< SYSCTRL_OVR Mask */

/**@} end of group GCR_SYSCTRL_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_RST0 GCR_RST0
 * @brief    Reset.
 * @{
 */
#define MXC_F_GCR_RST0_DMA0_POS                        0 /**< RST0_DMA0 Position */
#define MXC_F_GCR_RST0_DMA0                            ((uint32_t)(0x1UL << MXC_F_GCR_RST0_DMA0_POS)) /**< RST0_DMA0 Mask */

#define MXC_F_GCR_RST0_WDT_POS                         1 /**< RST0_WDT Position */
#define MXC_F_GCR_RST0_WDT                             ((uint32_t)(0x1UL << MXC_F_GCR_RST0_WDT_POS)) /**< RST0_WDT Mask */

#define MXC_F_GCR_RST0_GPIO0_POS                       2 /**< RST0_GPIO0 Position */
#define MXC_F_GCR_RST0_GPIO0                           ((uint32_t)(0x1UL << MXC_F_GCR_RST0_GPIO0_POS)) /**< RST0_GPIO0 Mask */

#define MXC_F_GCR_RST0_TMR0_POS                        5 /**< RST0_TMR0 Position */
#define MXC_F_GCR_RST0_TMR0                            ((uint32_t)(0x1UL << MXC_F_GCR_RST0_TMR0_POS)) /**< RST0_TMR0 Mask */

#define MXC_F_GCR_RST0_TMR1_POS                        6 /**< RST0_TMR1 Position */
#define MXC_F_GCR_RST0_TMR1                            ((uint32_t)(0x1UL << MXC_F_GCR_RST0_TMR1_POS)) /**< RST0_TMR1 Mask */

#define MXC_F_GCR_RST0_TMR2_POS                        7 /**< RST0_TMR2 Position */
#define MXC_F_GCR_RST0_TMR2                            ((uint32_t)(0x1UL << MXC_F_GCR_RST0_TMR2_POS)) /**< RST0_TMR2 Mask */

#define MXC_F_GCR_RST0_TMR3_POS                        8 /**< RST0_TMR3 Position */
#define MXC_F_GCR_RST0_TMR3                            ((uint32_t)(0x1UL << MXC_F_GCR_RST0_TMR3_POS)) /**< RST0_TMR3 Mask */

#define MXC_F_GCR_RST0_TMR4_POS                        9 /**< RST0_TMR4 Position */
#define MXC_F_GCR_RST0_TMR4                            ((uint32_t)(0x1UL << MXC_F_GCR_RST0_TMR4_POS)) /**< RST0_TMR4 Mask */

#define MXC_F_GCR_RST0_TMR5_POS                        10 /**< RST0_TMR5 Position */
#define MXC_F_GCR_RST0_TMR5                            ((uint32_t)(0x1UL << MXC_F_GCR_RST0_TMR5_POS)) /**< RST0_TMR5 Mask */

#define MXC_F_GCR_RST0_UART_POS                        11 /**< RST0_UART Position */
#define MXC_F_GCR_RST0_UART                            ((uint32_t)(0x1UL << MXC_F_GCR_RST0_UART_POS)) /**< RST0_UART Mask */

#define MXC_F_GCR_RST0_SPI_POS                         13 /**< RST0_SPI Position */
#define MXC_F_GCR_RST0_SPI                             ((uint32_t)(0x1UL << MXC_F_GCR_RST0_SPI_POS)) /**< RST0_SPI Mask */

#define MXC_F_GCR_RST0_I3C_POS                         16 /**< RST0_I3C Position */
#define MXC_F_GCR_RST0_I3C                             ((uint32_t)(0x1UL << MXC_F_GCR_RST0_I3C_POS)) /**< RST0_I3C Mask */

#define MXC_F_GCR_RST0_RTC_POS                         17 /**< RST0_RTC Position */
#define MXC_F_GCR_RST0_RTC                             ((uint32_t)(0x1UL << MXC_F_GCR_RST0_RTC_POS)) /**< RST0_RTC Mask */

#define MXC_F_GCR_RST0_TRNG_POS                        24 /**< RST0_TRNG Position */
#define MXC_F_GCR_RST0_TRNG                            ((uint32_t)(0x1UL << MXC_F_GCR_RST0_TRNG_POS)) /**< RST0_TRNG Mask */

#define MXC_F_GCR_RST0_DMA1_POS                        27 /**< RST0_DMA1 Position */
#define MXC_F_GCR_RST0_DMA1                            ((uint32_t)(0x1UL << MXC_F_GCR_RST0_DMA1_POS)) /**< RST0_DMA1 Mask */

#define MXC_F_GCR_RST0_SOFT_POS                        29 /**< RST0_SOFT Position */
#define MXC_F_GCR_RST0_SOFT                            ((uint32_t)(0x1UL << MXC_F_GCR_RST0_SOFT_POS)) /**< RST0_SOFT Mask */

#define MXC_F_GCR_RST0_PERIPH_POS                      30 /**< RST0_PERIPH Position */
#define MXC_F_GCR_RST0_PERIPH                          ((uint32_t)(0x1UL << MXC_F_GCR_RST0_PERIPH_POS)) /**< RST0_PERIPH Mask */

#define MXC_F_GCR_RST0_SYS_POS                         31 /**< RST0_SYS Position */
#define MXC_F_GCR_RST0_SYS                             ((uint32_t)(0x1UL << MXC_F_GCR_RST0_SYS_POS)) /**< RST0_SYS Mask */

/**@} end of group GCR_RST0_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_CLKCTRL GCR_CLKCTRL
 * @brief    Clock Control.
 * @{
 */
#define MXC_F_GCR_CLKCTRL_SYSCLK_DIV_POS               6 /**< CLKCTRL_SYSCLK_DIV Position */
#define MXC_F_GCR_CLKCTRL_SYSCLK_DIV                   ((uint32_t)(0x7UL << MXC_F_GCR_CLKCTRL_SYSCLK_DIV_POS)) /**< CLKCTRL_SYSCLK_DIV Mask */
#define MXC_V_GCR_CLKCTRL_SYSCLK_DIV_DIV1              ((uint32_t)0x0UL) /**< CLKCTRL_SYSCLK_DIV_DIV1 Value */
#define MXC_S_GCR_CLKCTRL_SYSCLK_DIV_DIV1              (MXC_V_GCR_CLKCTRL_SYSCLK_DIV_DIV1 << MXC_F_GCR_CLKCTRL_SYSCLK_DIV_POS) /**< CLKCTRL_SYSCLK_DIV_DIV1 Setting */
#define MXC_V_GCR_CLKCTRL_SYSCLK_DIV_DIV2              ((uint32_t)0x1UL) /**< CLKCTRL_SYSCLK_DIV_DIV2 Value */
#define MXC_S_GCR_CLKCTRL_SYSCLK_DIV_DIV2              (MXC_V_GCR_CLKCTRL_SYSCLK_DIV_DIV2 << MXC_F_GCR_CLKCTRL_SYSCLK_DIV_POS) /**< CLKCTRL_SYSCLK_DIV_DIV2 Setting */
#define MXC_V_GCR_CLKCTRL_SYSCLK_DIV_DIV4              ((uint32_t)0x2UL) /**< CLKCTRL_SYSCLK_DIV_DIV4 Value */
#define MXC_S_GCR_CLKCTRL_SYSCLK_DIV_DIV4              (MXC_V_GCR_CLKCTRL_SYSCLK_DIV_DIV4 << MXC_F_GCR_CLKCTRL_SYSCLK_DIV_POS) /**< CLKCTRL_SYSCLK_DIV_DIV4 Setting */
#define MXC_V_GCR_CLKCTRL_SYSCLK_DIV_DIV8              ((uint32_t)0x3UL) /**< CLKCTRL_SYSCLK_DIV_DIV8 Value */
#define MXC_S_GCR_CLKCTRL_SYSCLK_DIV_DIV8              (MXC_V_GCR_CLKCTRL_SYSCLK_DIV_DIV8 << MXC_F_GCR_CLKCTRL_SYSCLK_DIV_POS) /**< CLKCTRL_SYSCLK_DIV_DIV8 Setting */
#define MXC_V_GCR_CLKCTRL_SYSCLK_DIV_DIV16             ((uint32_t)0x4UL) /**< CLKCTRL_SYSCLK_DIV_DIV16 Value */
#define MXC_S_GCR_CLKCTRL_SYSCLK_DIV_DIV16             (MXC_V_GCR_CLKCTRL_SYSCLK_DIV_DIV16 << MXC_F_GCR_CLKCTRL_SYSCLK_DIV_POS) /**< CLKCTRL_SYSCLK_DIV_DIV16 Setting */
#define MXC_V_GCR_CLKCTRL_SYSCLK_DIV_DIV32             ((uint32_t)0x5UL) /**< CLKCTRL_SYSCLK_DIV_DIV32 Value */
#define MXC_S_GCR_CLKCTRL_SYSCLK_DIV_DIV32             (MXC_V_GCR_CLKCTRL_SYSCLK_DIV_DIV32 << MXC_F_GCR_CLKCTRL_SYSCLK_DIV_POS) /**< CLKCTRL_SYSCLK_DIV_DIV32 Setting */
#define MXC_V_GCR_CLKCTRL_SYSCLK_DIV_DIV64             ((uint32_t)0x6UL) /**< CLKCTRL_SYSCLK_DIV_DIV64 Value */
#define MXC_S_GCR_CLKCTRL_SYSCLK_DIV_DIV64             (MXC_V_GCR_CLKCTRL_SYSCLK_DIV_DIV64 << MXC_F_GCR_CLKCTRL_SYSCLK_DIV_POS) /**< CLKCTRL_SYSCLK_DIV_DIV64 Setting */
#define MXC_V_GCR_CLKCTRL_SYSCLK_DIV_DIV128            ((uint32_t)0x7UL) /**< CLKCTRL_SYSCLK_DIV_DIV128 Value */
#define MXC_S_GCR_CLKCTRL_SYSCLK_DIV_DIV128            (MXC_V_GCR_CLKCTRL_SYSCLK_DIV_DIV128 << MXC_F_GCR_CLKCTRL_SYSCLK_DIV_POS) /**< CLKCTRL_SYSCLK_DIV_DIV128 Setting */

#define MXC_F_GCR_CLKCTRL_SYSCLK_SEL_POS               9 /**< CLKCTRL_SYSCLK_SEL Position */
#define MXC_F_GCR_CLKCTRL_SYSCLK_SEL                   ((uint32_t)(0x7UL << MXC_F_GCR_CLKCTRL_SYSCLK_SEL_POS)) /**< CLKCTRL_SYSCLK_SEL Mask */
#define MXC_V_GCR_CLKCTRL_SYSCLK_SEL_IPO               ((uint32_t)0x0UL) /**< CLKCTRL_SYSCLK_SEL_IPO Value */
#define MXC_S_GCR_CLKCTRL_SYSCLK_SEL_IPO               (MXC_V_GCR_CLKCTRL_SYSCLK_SEL_IPO << MXC_F_GCR_CLKCTRL_SYSCLK_SEL_POS) /**< CLKCTRL_SYSCLK_SEL_IPO Setting */
#define MXC_V_GCR_CLKCTRL_SYSCLK_SEL_ERFO              ((uint32_t)0x2UL) /**< CLKCTRL_SYSCLK_SEL_ERFO Value */
#define MXC_S_GCR_CLKCTRL_SYSCLK_SEL_ERFO              (MXC_V_GCR_CLKCTRL_SYSCLK_SEL_ERFO << MXC_F_GCR_CLKCTRL_SYSCLK_SEL_POS) /**< CLKCTRL_SYSCLK_SEL_ERFO Setting */
#define MXC_V_GCR_CLKCTRL_SYSCLK_SEL_INRO              ((uint32_t)0x3UL) /**< CLKCTRL_SYSCLK_SEL_INRO Value */
#define MXC_S_GCR_CLKCTRL_SYSCLK_SEL_INRO              (MXC_V_GCR_CLKCTRL_SYSCLK_SEL_INRO << MXC_F_GCR_CLKCTRL_SYSCLK_SEL_POS) /**< CLKCTRL_SYSCLK_SEL_INRO Setting */
#define MXC_V_GCR_CLKCTRL_SYSCLK_SEL_IBRO              ((uint32_t)0x5UL) /**< CLKCTRL_SYSCLK_SEL_IBRO Value */
#define MXC_S_GCR_CLKCTRL_SYSCLK_SEL_IBRO              (MXC_V_GCR_CLKCTRL_SYSCLK_SEL_IBRO << MXC_F_GCR_CLKCTRL_SYSCLK_SEL_POS) /**< CLKCTRL_SYSCLK_SEL_IBRO Setting */
#define MXC_V_GCR_CLKCTRL_SYSCLK_SEL_ERTCO             ((uint32_t)0x6UL) /**< CLKCTRL_SYSCLK_SEL_ERTCO Value */
#define MXC_S_GCR_CLKCTRL_SYSCLK_SEL_ERTCO             (MXC_V_GCR_CLKCTRL_SYSCLK_SEL_ERTCO << MXC_F_GCR_CLKCTRL_SYSCLK_SEL_POS) /**< CLKCTRL_SYSCLK_SEL_ERTCO Setting */
#define MXC_V_GCR_CLKCTRL_SYSCLK_SEL_EXTCLK            ((uint32_t)0x7UL) /**< CLKCTRL_SYSCLK_SEL_EXTCLK Value */
#define MXC_S_GCR_CLKCTRL_SYSCLK_SEL_EXTCLK            (MXC_V_GCR_CLKCTRL_SYSCLK_SEL_EXTCLK << MXC_F_GCR_CLKCTRL_SYSCLK_SEL_POS) /**< CLKCTRL_SYSCLK_SEL_EXTCLK Setting */

#define MXC_F_GCR_CLKCTRL_SYSCLK_RDY_POS               13 /**< CLKCTRL_SYSCLK_RDY Position */
#define MXC_F_GCR_CLKCTRL_SYSCLK_RDY                   ((uint32_t)(0x1UL << MXC_F_GCR_CLKCTRL_SYSCLK_RDY_POS)) /**< CLKCTRL_SYSCLK_RDY Mask */

#define MXC_F_GCR_CLKCTRL_ERFO_EN_POS                  16 /**< CLKCTRL_ERFO_EN Position */
#define MXC_F_GCR_CLKCTRL_ERFO_EN                      ((uint32_t)(0x1UL << MXC_F_GCR_CLKCTRL_ERFO_EN_POS)) /**< CLKCTRL_ERFO_EN Mask */

#define MXC_F_GCR_CLKCTRL_ERTCO_EN_POS                 17 /**< CLKCTRL_ERTCO_EN Position */
#define MXC_F_GCR_CLKCTRL_ERTCO_EN                     ((uint32_t)(0x1UL << MXC_F_GCR_CLKCTRL_ERTCO_EN_POS)) /**< CLKCTRL_ERTCO_EN Mask */

#define MXC_F_GCR_CLKCTRL_IPO_EN_POS                   18 /**< CLKCTRL_IPO_EN Position */
#define MXC_F_GCR_CLKCTRL_IPO_EN                       ((uint32_t)(0x1UL << MXC_F_GCR_CLKCTRL_IPO_EN_POS)) /**< CLKCTRL_IPO_EN Mask */

#define MXC_F_GCR_CLKCTRL_IBRO_EN_POS                  20 /**< CLKCTRL_IBRO_EN Position */
#define MXC_F_GCR_CLKCTRL_IBRO_EN                      ((uint32_t)(0x1UL << MXC_F_GCR_CLKCTRL_IBRO_EN_POS)) /**< CLKCTRL_IBRO_EN Mask */

#define MXC_F_GCR_CLKCTRL_IBRO_VS_POS                  21 /**< CLKCTRL_IBRO_VS Position */
#define MXC_F_GCR_CLKCTRL_IBRO_VS                      ((uint32_t)(0x1UL << MXC_F_GCR_CLKCTRL_IBRO_VS_POS)) /**< CLKCTRL_IBRO_VS Mask */

#define MXC_F_GCR_CLKCTRL_ERFO_RDY_POS                 24 /**< CLKCTRL_ERFO_RDY Position */
#define MXC_F_GCR_CLKCTRL_ERFO_RDY                     ((uint32_t)(0x1UL << MXC_F_GCR_CLKCTRL_ERFO_RDY_POS)) /**< CLKCTRL_ERFO_RDY Mask */

#define MXC_F_GCR_CLKCTRL_ERTCO_RDY_POS                25 /**< CLKCTRL_ERTCO_RDY Position */
#define MXC_F_GCR_CLKCTRL_ERTCO_RDY                    ((uint32_t)(0x1UL << MXC_F_GCR_CLKCTRL_ERTCO_RDY_POS)) /**< CLKCTRL_ERTCO_RDY Mask */

#define MXC_F_GCR_CLKCTRL_IPO_RDY_POS                  26 /**< CLKCTRL_IPO_RDY Position */
#define MXC_F_GCR_CLKCTRL_IPO_RDY                      ((uint32_t)(0x1UL << MXC_F_GCR_CLKCTRL_IPO_RDY_POS)) /**< CLKCTRL_IPO_RDY Mask */

#define MXC_F_GCR_CLKCTRL_IBRO_RDY_POS                 28 /**< CLKCTRL_IBRO_RDY Position */
#define MXC_F_GCR_CLKCTRL_IBRO_RDY                     ((uint32_t)(0x1UL << MXC_F_GCR_CLKCTRL_IBRO_RDY_POS)) /**< CLKCTRL_IBRO_RDY Mask */

#define MXC_F_GCR_CLKCTRL_INRO_RDY_POS                 29 /**< CLKCTRL_INRO_RDY Position */
#define MXC_F_GCR_CLKCTRL_INRO_RDY                     ((uint32_t)(0x1UL << MXC_F_GCR_CLKCTRL_INRO_RDY_POS)) /**< CLKCTRL_INRO_RDY Mask */

/**@} end of group GCR_CLKCTRL_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_PM GCR_PM
 * @brief    Power Management.
 * @{
 */
#define MXC_F_GCR_PM_MODE_POS                          0 /**< PM_MODE Position */
#define MXC_F_GCR_PM_MODE                              ((uint32_t)(0xFUL << MXC_F_GCR_PM_MODE_POS)) /**< PM_MODE Mask */
#define MXC_V_GCR_PM_MODE_ACTIVE                       ((uint32_t)0x0UL) /**< PM_MODE_ACTIVE Value */
#define MXC_S_GCR_PM_MODE_ACTIVE                       (MXC_V_GCR_PM_MODE_ACTIVE << MXC_F_GCR_PM_MODE_POS) /**< PM_MODE_ACTIVE Setting */
#define MXC_V_GCR_PM_MODE_BACKUP                       ((uint32_t)0x4UL) /**< PM_MODE_BACKUP Value */
#define MXC_S_GCR_PM_MODE_BACKUP                       (MXC_V_GCR_PM_MODE_BACKUP << MXC_F_GCR_PM_MODE_POS) /**< PM_MODE_BACKUP Setting */
#define MXC_V_GCR_PM_MODE_PDM                          ((uint32_t)0xAUL) /**< PM_MODE_PDM Value */
#define MXC_S_GCR_PM_MODE_PDM                          (MXC_V_GCR_PM_MODE_PDM << MXC_F_GCR_PM_MODE_POS) /**< PM_MODE_PDM Setting */

#define MXC_F_GCR_PM_GPIO_WE_POS                       4 /**< PM_GPIO_WE Position */
#define MXC_F_GCR_PM_GPIO_WE                           ((uint32_t)(0x1UL << MXC_F_GCR_PM_GPIO_WE_POS)) /**< PM_GPIO_WE Mask */

#define MXC_F_GCR_PM_RTC_WE_POS                        5 /**< PM_RTC_WE Position */
#define MXC_F_GCR_PM_RTC_WE                            ((uint32_t)(0x1UL << MXC_F_GCR_PM_RTC_WE_POS)) /**< PM_RTC_WE Mask */

#define MXC_F_GCR_PM_WUT_WE_POS                        6 /**< PM_WUT_WE Position */
#define MXC_F_GCR_PM_WUT_WE                            ((uint32_t)(0x1UL << MXC_F_GCR_PM_WUT_WE_POS)) /**< PM_WUT_WE Mask */

#define MXC_F_GCR_PM_ERFO_BP_POS                       20 /**< PM_ERFO_BP Position */
#define MXC_F_GCR_PM_ERFO_BP                           ((uint32_t)(0x1UL << MXC_F_GCR_PM_ERFO_BP_POS)) /**< PM_ERFO_BP Mask */

/**@} end of group GCR_PM_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_PCLKDIV GCR_PCLKDIV
 * @brief    Peripheral Clock Divider.
 * @{
 */
#define MXC_F_GCR_PCLKDIV_CLKDIV_POS                   0 /**< PCLKDIV_CLKDIV Position */
#define MXC_F_GCR_PCLKDIV_CLKDIV                       ((uint32_t)(0xFFFFFFFFUL << MXC_F_GCR_PCLKDIV_CLKDIV_POS)) /**< PCLKDIV_CLKDIV Mask */

/**@} end of group GCR_PCLKDIV_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_PCLKDIS0 GCR_PCLKDIS0
 * @brief    Peripheral Clock Disable.
 * @{
 */
#define MXC_F_GCR_PCLKDIS0_GPIO0_POS                   0 /**< PCLKDIS0_GPIO0 Position */
#define MXC_F_GCR_PCLKDIS0_GPIO0                       ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS0_GPIO0_POS)) /**< PCLKDIS0_GPIO0 Mask */

#define MXC_F_GCR_PCLKDIS0_DMA0_POS                    5 /**< PCLKDIS0_DMA0 Position */
#define MXC_F_GCR_PCLKDIS0_DMA0                        ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS0_DMA0_POS)) /**< PCLKDIS0_DMA0 Mask */

#define MXC_F_GCR_PCLKDIS0_SPI_POS                     6 /**< PCLKDIS0_SPI Position */
#define MXC_F_GCR_PCLKDIS0_SPI                         ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS0_SPI_POS)) /**< PCLKDIS0_SPI Mask */

#define MXC_F_GCR_PCLKDIS0_UART_POS                    9 /**< PCLKDIS0_UART Position */
#define MXC_F_GCR_PCLKDIS0_UART                        ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS0_UART_POS)) /**< PCLKDIS0_UART Mask */

#define MXC_F_GCR_PCLKDIS0_I3C_POS                     13 /**< PCLKDIS0_I3C Position */
#define MXC_F_GCR_PCLKDIS0_I3C                         ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS0_I3C_POS)) /**< PCLKDIS0_I3C Mask */

#define MXC_F_GCR_PCLKDIS0_TMR0_POS                    15 /**< PCLKDIS0_TMR0 Position */
#define MXC_F_GCR_PCLKDIS0_TMR0                        ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS0_TMR0_POS)) /**< PCLKDIS0_TMR0 Mask */

#define MXC_F_GCR_PCLKDIS0_TMR1_POS                    16 /**< PCLKDIS0_TMR1 Position */
#define MXC_F_GCR_PCLKDIS0_TMR1                        ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS0_TMR1_POS)) /**< PCLKDIS0_TMR1 Mask */

#define MXC_F_GCR_PCLKDIS0_TMR2_POS                    17 /**< PCLKDIS0_TMR2 Position */
#define MXC_F_GCR_PCLKDIS0_TMR2                        ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS0_TMR2_POS)) /**< PCLKDIS0_TMR2 Mask */

#define MXC_F_GCR_PCLKDIS0_TMR3_POS                    18 /**< PCLKDIS0_TMR3 Position */
#define MXC_F_GCR_PCLKDIS0_TMR3                        ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS0_TMR3_POS)) /**< PCLKDIS0_TMR3 Mask */

#define MXC_F_GCR_PCLKDIS0_TMR4_POS                    19 /**< PCLKDIS0_TMR4 Position */
#define MXC_F_GCR_PCLKDIS0_TMR4                        ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS0_TMR4_POS)) /**< PCLKDIS0_TMR4 Mask */

#define MXC_F_GCR_PCLKDIS0_TMR5_POS                    20 /**< PCLKDIS0_TMR5 Position */
#define MXC_F_GCR_PCLKDIS0_TMR5                        ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS0_TMR5_POS)) /**< PCLKDIS0_TMR5 Mask */

/**@} end of group GCR_PCLKDIS0_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_MEMCTRL GCR_MEMCTRL
 * @brief    Memory Clock Control Register.
 * @{
 */
#define MXC_F_GCR_MEMCTRL_FWS_POS                      0 /**< MEMCTRL_FWS Position */
#define MXC_F_GCR_MEMCTRL_FWS                          ((uint32_t)(0x7UL << MXC_F_GCR_MEMCTRL_FWS_POS)) /**< MEMCTRL_FWS Mask */

/**@} end of group GCR_MEMCTRL_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_MEMZ GCR_MEMZ
 * @brief    Memory Zeroize Control.
 * @{
 */
#define MXC_F_GCR_MEMZ_SRAM0_POS                       0 /**< MEMZ_SRAM0 Position */
#define MXC_F_GCR_MEMZ_SRAM0                           ((uint32_t)(0x1UL << MXC_F_GCR_MEMZ_SRAM0_POS)) /**< MEMZ_SRAM0 Mask */

#define MXC_F_GCR_MEMZ_SRAM1_POS                       1 /**< MEMZ_SRAM1 Position */
#define MXC_F_GCR_MEMZ_SRAM1                           ((uint32_t)(0x1UL << MXC_F_GCR_MEMZ_SRAM1_POS)) /**< MEMZ_SRAM1 Mask */

#define MXC_F_GCR_MEMZ_SRAM2_POS                       2 /**< MEMZ_SRAM2 Position */
#define MXC_F_GCR_MEMZ_SRAM2                           ((uint32_t)(0x1UL << MXC_F_GCR_MEMZ_SRAM2_POS)) /**< MEMZ_SRAM2 Mask */

#define MXC_F_GCR_MEMZ_SRAM3_POS                       3 /**< MEMZ_SRAM3 Position */
#define MXC_F_GCR_MEMZ_SRAM3                           ((uint32_t)(0x1UL << MXC_F_GCR_MEMZ_SRAM3_POS)) /**< MEMZ_SRAM3 Mask */

#define MXC_F_GCR_MEMZ_SRAM4_POS                       4 /**< MEMZ_SRAM4 Position */
#define MXC_F_GCR_MEMZ_SRAM4                           ((uint32_t)(0x1UL << MXC_F_GCR_MEMZ_SRAM4_POS)) /**< MEMZ_SRAM4 Mask */

#define MXC_F_GCR_MEMZ_ICC_POS                         5 /**< MEMZ_ICC Position */
#define MXC_F_GCR_MEMZ_ICC                             ((uint32_t)(0x1UL << MXC_F_GCR_MEMZ_ICC_POS)) /**< MEMZ_ICC Mask */

/**@} end of group GCR_MEMZ_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_SYSST GCR_SYSST
 * @brief    System Status Register.
 * @{
 */
#define MXC_F_GCR_SYSST_ICELOCK_POS                    0 /**< SYSST_ICELOCK Position */
#define MXC_F_GCR_SYSST_ICELOCK                        ((uint32_t)(0x1UL << MXC_F_GCR_SYSST_ICELOCK_POS)) /**< SYSST_ICELOCK Mask */

/**@} end of group GCR_SYSST_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_RST1 GCR_RST1
 * @brief    Reset 1.
 * @{
 */
#define MXC_F_GCR_RST1_CRC_POS                         9 /**< RST1_CRC Position */
#define MXC_F_GCR_RST1_CRC                             ((uint32_t)(0x1UL << MXC_F_GCR_RST1_CRC_POS)) /**< RST1_CRC Mask */

#define MXC_F_GCR_RST1_AES_POS                         10 /**< RST1_AES Position */
#define MXC_F_GCR_RST1_AES                             ((uint32_t)(0x1UL << MXC_F_GCR_RST1_AES_POS)) /**< RST1_AES Mask */

#define MXC_F_GCR_RST1_AUTOCAL_POS                     12 /**< RST1_AUTOCAL Position */
#define MXC_F_GCR_RST1_AUTOCAL                         ((uint32_t)(0x1UL << MXC_F_GCR_RST1_AUTOCAL_POS)) /**< RST1_AUTOCAL Mask */

#define MXC_F_GCR_RST1_BTLE_POS                        18 /**< RST1_BTLE Position */
#define MXC_F_GCR_RST1_BTLE                            ((uint32_t)(0x1UL << MXC_F_GCR_RST1_BTLE_POS)) /**< RST1_BTLE Mask */

/**@} end of group GCR_RST1_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_PCLKDIS1 GCR_PCLKDIS1
 * @brief    Peripheral Clock Disable.
 * @{
 */
#define MXC_F_GCR_PCLKDIS1_TRNG_POS                    2 /**< PCLKDIS1_TRNG Position */
#define MXC_F_GCR_PCLKDIS1_TRNG                        ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS1_TRNG_POS)) /**< PCLKDIS1_TRNG Mask */

#define MXC_F_GCR_PCLKDIS1_CRC_POS                     14 /**< PCLKDIS1_CRC Position */
#define MXC_F_GCR_PCLKDIS1_CRC                         ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS1_CRC_POS)) /**< PCLKDIS1_CRC Mask */

#define MXC_F_GCR_PCLKDIS1_AES_POS                     15 /**< PCLKDIS1_AES Position */
#define MXC_F_GCR_PCLKDIS1_AES                         ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS1_AES_POS)) /**< PCLKDIS1_AES Mask */

#define MXC_F_GCR_PCLKDIS1_DMA1_POS                    21 /**< PCLKDIS1_DMA1 Position */
#define MXC_F_GCR_PCLKDIS1_DMA1                        ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS1_DMA1_POS)) /**< PCLKDIS1_DMA1 Mask */

#define MXC_F_GCR_PCLKDIS1_WDT_POS                     27 /**< PCLKDIS1_WDT Position */
#define MXC_F_GCR_PCLKDIS1_WDT                         ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS1_WDT_POS)) /**< PCLKDIS1_WDT Mask */

/**@} end of group GCR_PCLKDIS1_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_EVENTEN GCR_EVENTEN
 * @brief    Event Enable Register.
 * @{
 */
#define MXC_F_GCR_EVENTEN_DMA0_POS                     0 /**< EVENTEN_DMA0 Position */
#define MXC_F_GCR_EVENTEN_DMA0                         ((uint32_t)(0x1UL << MXC_F_GCR_EVENTEN_DMA0_POS)) /**< EVENTEN_DMA0 Mask */

#define MXC_F_GCR_EVENTEN_DMA1_POS                     1 /**< EVENTEN_DMA1 Position */
#define MXC_F_GCR_EVENTEN_DMA1                         ((uint32_t)(0x1UL << MXC_F_GCR_EVENTEN_DMA1_POS)) /**< EVENTEN_DMA1 Mask */

#define MXC_F_GCR_EVENTEN_TX_POS                       2 /**< EVENTEN_TX Position */
#define MXC_F_GCR_EVENTEN_TX                           ((uint32_t)(0x1UL << MXC_F_GCR_EVENTEN_TX_POS)) /**< EVENTEN_TX Mask */

/**@} end of group GCR_EVENTEN_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_REVISION GCR_REVISION
 * @brief    Revision Register.
 * @{
 */
#define MXC_F_GCR_REVISION_REVISION_POS                0 /**< REVISION_REVISION Position */
#define MXC_F_GCR_REVISION_REVISION                    ((uint32_t)(0xFFFFUL << MXC_F_GCR_REVISION_REVISION_POS)) /**< REVISION_REVISION Mask */

/**@} end of group GCR_REVISION_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_SYSINTEN GCR_SYSINTEN
 * @brief    System Status Interrupt Enable Register.
 * @{
 */
#define MXC_F_GCR_SYSINTEN_ICEUNLOCK_POS               0 /**< SYSINTEN_ICEUNLOCK Position */
#define MXC_F_GCR_SYSINTEN_ICEUNLOCK                   ((uint32_t)(0x1UL << MXC_F_GCR_SYSINTEN_ICEUNLOCK_POS)) /**< SYSINTEN_ICEUNLOCK Mask */

/**@} end of group GCR_SYSINTEN_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_ECCERR GCR_ECCERR
 * @brief    ECC Error Register
 * @{
 */
#define MXC_F_GCR_ECCERR_FLASH_POS                     0 /**< ECCERR_FLASH Position */
#define MXC_F_GCR_ECCERR_FLASH                         ((uint32_t)(0x1UL << MXC_F_GCR_ECCERR_FLASH_POS)) /**< ECCERR_FLASH Mask */

/**@} end of group GCR_ECCERR_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_ECCCED GCR_ECCCED
 * @brief    ECC Correctable Error Detect Register.
 * @{
 */
#define MXC_F_GCR_ECCCED_FLASH_POS                     0 /**< ECCCED_FLASH Position */
#define MXC_F_GCR_ECCCED_FLASH                         ((uint32_t)(0x1UL << MXC_F_GCR_ECCCED_FLASH_POS)) /**< ECCCED_FLASH Mask */

/**@} end of group GCR_ECCCED_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_ECCINTEN GCR_ECCINTEN
 * @brief    ECC Interrupt Enable Register
 * @{
 */
#define MXC_F_GCR_ECCINTEN_FLASH_POS                   0 /**< ECCINTEN_FLASH Position */
#define MXC_F_GCR_ECCINTEN_FLASH                       ((uint32_t)(0x1UL << MXC_F_GCR_ECCINTEN_FLASH_POS)) /**< ECCINTEN_FLASH Mask */

/**@} end of group GCR_ECCINTEN_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_ECCADDR GCR_ECCADDR
 * @brief    ECC Error Address Register
 * @{
 */
#define MXC_F_GCR_ECCADDR_DADDR_POS                    0 /**< ECCADDR_DADDR Position */
#define MXC_F_GCR_ECCADDR_DADDR                        ((uint32_t)(0x3FFFUL << MXC_F_GCR_ECCADDR_DADDR_POS)) /**< ECCADDR_DADDR Mask */

#define MXC_F_GCR_ECCADDR_DB_POS                       14 /**< ECCADDR_DB Position */
#define MXC_F_GCR_ECCADDR_DB                           ((uint32_t)(0x1UL << MXC_F_GCR_ECCADDR_DB_POS)) /**< ECCADDR_DB Mask */

#define MXC_F_GCR_ECCADDR_DE_POS                       15 /**< ECCADDR_DE Position */
#define MXC_F_GCR_ECCADDR_DE                           ((uint32_t)(0x1UL << MXC_F_GCR_ECCADDR_DE_POS)) /**< ECCADDR_DE Mask */

#define MXC_F_GCR_ECCADDR_TADDR_POS                    16 /**< ECCADDR_TADDR Position */
#define MXC_F_GCR_ECCADDR_TADDR                        ((uint32_t)(0x3FFFUL << MXC_F_GCR_ECCADDR_TADDR_POS)) /**< ECCADDR_TADDR Mask */

#define MXC_F_GCR_ECCADDR_TB_POS                       30 /**< ECCADDR_TB Position */
#define MXC_F_GCR_ECCADDR_TB                           ((uint32_t)(0x1UL << MXC_F_GCR_ECCADDR_TB_POS)) /**< ECCADDR_TB Mask */

#define MXC_F_GCR_ECCADDR_TE_POS                       31 /**< ECCADDR_TE Position */
#define MXC_F_GCR_ECCADDR_TE                           ((uint32_t)(0x1UL << MXC_F_GCR_ECCADDR_TE_POS)) /**< ECCADDR_TE Mask */

/**@} end of group GCR_ECCADDR_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_BTLELDOCTRL GCR_BTLELDOCTRL
 * @brief    BTLE LDO Control Register
 * @{
 */
#define MXC_F_GCR_BTLELDOCTRL_BB_EN_POS                0 /**< BTLELDOCTRL_BB_EN Position */
#define MXC_F_GCR_BTLELDOCTRL_BB_EN                    ((uint32_t)(0x1UL << MXC_F_GCR_BTLELDOCTRL_BB_EN_POS)) /**< BTLELDOCTRL_BB_EN Mask */

#define MXC_F_GCR_BTLELDOCTRL_BB_PD_EN_POS             1 /**< BTLELDOCTRL_BB_PD_EN Position */
#define MXC_F_GCR_BTLELDOCTRL_BB_PD_EN                 ((uint32_t)(0x1UL << MXC_F_GCR_BTLELDOCTRL_BB_PD_EN_POS)) /**< BTLELDOCTRL_BB_PD_EN Mask */

#define MXC_F_GCR_BTLELDOCTRL_BB_VSEL_POS              2 /**< BTLELDOCTRL_BB_VSEL Position */
#define MXC_F_GCR_BTLELDOCTRL_BB_VSEL                  ((uint32_t)(0x3UL << MXC_F_GCR_BTLELDOCTRL_BB_VSEL_POS)) /**< BTLELDOCTRL_BB_VSEL Mask */
#define MXC_V_GCR_BTLELDOCTRL_BB_VSEL_0_9              ((uint32_t)0x0UL) /**< BTLELDOCTRL_BB_VSEL_0_9 Value */
#define MXC_S_GCR_BTLELDOCTRL_BB_VSEL_0_9              (MXC_V_GCR_BTLELDOCTRL_BB_VSEL_0_9 << MXC_F_GCR_BTLELDOCTRL_BB_VSEL_POS) /**< BTLELDOCTRL_BB_VSEL_0_9 Setting */
#define MXC_V_GCR_BTLELDOCTRL_BB_VSEL_1_0              ((uint32_t)0x1UL) /**< BTLELDOCTRL_BB_VSEL_1_0 Value */
#define MXC_S_GCR_BTLELDOCTRL_BB_VSEL_1_0              (MXC_V_GCR_BTLELDOCTRL_BB_VSEL_1_0 << MXC_F_GCR_BTLELDOCTRL_BB_VSEL_POS) /**< BTLELDOCTRL_BB_VSEL_1_0 Setting */
#define MXC_V_GCR_BTLELDOCTRL_BB_VSEL_1_1              ((uint32_t)0x2UL) /**< BTLELDOCTRL_BB_VSEL_1_1 Value */
#define MXC_S_GCR_BTLELDOCTRL_BB_VSEL_1_1              (MXC_V_GCR_BTLELDOCTRL_BB_VSEL_1_1 << MXC_F_GCR_BTLELDOCTRL_BB_VSEL_POS) /**< BTLELDOCTRL_BB_VSEL_1_1 Setting */
#define MXC_V_GCR_BTLELDOCTRL_BB_VSEL_1_2              ((uint32_t)0x3UL) /**< BTLELDOCTRL_BB_VSEL_1_2 Value */
#define MXC_S_GCR_BTLELDOCTRL_BB_VSEL_1_2              (MXC_V_GCR_BTLELDOCTRL_BB_VSEL_1_2 << MXC_F_GCR_BTLELDOCTRL_BB_VSEL_POS) /**< BTLELDOCTRL_BB_VSEL_1_2 Setting */

#define MXC_F_GCR_BTLELDOCTRL_RF_EN_POS                4 /**< BTLELDOCTRL_RF_EN Position */
#define MXC_F_GCR_BTLELDOCTRL_RF_EN                    ((uint32_t)(0x1UL << MXC_F_GCR_BTLELDOCTRL_RF_EN_POS)) /**< BTLELDOCTRL_RF_EN Mask */

#define MXC_F_GCR_BTLELDOCTRL_RF_PD_EN_POS             5 /**< BTLELDOCTRL_RF_PD_EN Position */
#define MXC_F_GCR_BTLELDOCTRL_RF_PD_EN                 ((uint32_t)(0x1UL << MXC_F_GCR_BTLELDOCTRL_RF_PD_EN_POS)) /**< BTLELDOCTRL_RF_PD_EN Mask */

#define MXC_F_GCR_BTLELDOCTRL_RF_VSEL_POS              6 /**< BTLELDOCTRL_RF_VSEL Position */
#define MXC_F_GCR_BTLELDOCTRL_RF_VSEL                  ((uint32_t)(0x3UL << MXC_F_GCR_BTLELDOCTRL_RF_VSEL_POS)) /**< BTLELDOCTRL_RF_VSEL Mask */
#define MXC_V_GCR_BTLELDOCTRL_RF_VSEL_0_9              ((uint32_t)0x0UL) /**< BTLELDOCTRL_RF_VSEL_0_9 Value */
#define MXC_S_GCR_BTLELDOCTRL_RF_VSEL_0_9              (MXC_V_GCR_BTLELDOCTRL_RF_VSEL_0_9 << MXC_F_GCR_BTLELDOCTRL_RF_VSEL_POS) /**< BTLELDOCTRL_RF_VSEL_0_9 Setting */
#define MXC_V_GCR_BTLELDOCTRL_RF_VSEL_1_0              ((uint32_t)0x1UL) /**< BTLELDOCTRL_RF_VSEL_1_0 Value */
#define MXC_S_GCR_BTLELDOCTRL_RF_VSEL_1_0              (MXC_V_GCR_BTLELDOCTRL_RF_VSEL_1_0 << MXC_F_GCR_BTLELDOCTRL_RF_VSEL_POS) /**< BTLELDOCTRL_RF_VSEL_1_0 Setting */
#define MXC_V_GCR_BTLELDOCTRL_RF_VSEL_1_1              ((uint32_t)0x2UL) /**< BTLELDOCTRL_RF_VSEL_1_1 Value */
#define MXC_S_GCR_BTLELDOCTRL_RF_VSEL_1_1              (MXC_V_GCR_BTLELDOCTRL_RF_VSEL_1_1 << MXC_F_GCR_BTLELDOCTRL_RF_VSEL_POS) /**< BTLELDOCTRL_RF_VSEL_1_1 Setting */
#define MXC_V_GCR_BTLELDOCTRL_RF_VSEL_1_2              ((uint32_t)0x3UL) /**< BTLELDOCTRL_RF_VSEL_1_2 Value */
#define MXC_S_GCR_BTLELDOCTRL_RF_VSEL_1_2              (MXC_V_GCR_BTLELDOCTRL_RF_VSEL_1_2 << MXC_F_GCR_BTLELDOCTRL_RF_VSEL_POS) /**< BTLELDOCTRL_RF_VSEL_1_2 Setting */

#define MXC_F_GCR_BTLELDOCTRL_RF_BP_EN_POS             8 /**< BTLELDOCTRL_RF_BP_EN Position */
#define MXC_F_GCR_BTLELDOCTRL_RF_BP_EN                 ((uint32_t)(0x1UL << MXC_F_GCR_BTLELDOCTRL_RF_BP_EN_POS)) /**< BTLELDOCTRL_RF_BP_EN Mask */

#define MXC_F_GCR_BTLELDOCTRL_RF_DISCH_POS             9 /**< BTLELDOCTRL_RF_DISCH Position */
#define MXC_F_GCR_BTLELDOCTRL_RF_DISCH                 ((uint32_t)(0x1UL << MXC_F_GCR_BTLELDOCTRL_RF_DISCH_POS)) /**< BTLELDOCTRL_RF_DISCH Mask */

#define MXC_F_GCR_BTLELDOCTRL_BB_BP_EN_POS             10 /**< BTLELDOCTRL_BB_BP_EN Position */
#define MXC_F_GCR_BTLELDOCTRL_BB_BP_EN                 ((uint32_t)(0x1UL << MXC_F_GCR_BTLELDOCTRL_BB_BP_EN_POS)) /**< BTLELDOCTRL_BB_BP_EN Mask */

#define MXC_F_GCR_BTLELDOCTRL_BB_DISCH_POS             11 /**< BTLELDOCTRL_BB_DISCH Position */
#define MXC_F_GCR_BTLELDOCTRL_BB_DISCH                 ((uint32_t)(0x1UL << MXC_F_GCR_BTLELDOCTRL_BB_DISCH_POS)) /**< BTLELDOCTRL_BB_DISCH Mask */

#define MXC_F_GCR_BTLELDOCTRL_BB_EN_DLY_POS            12 /**< BTLELDOCTRL_BB_EN_DLY Position */
#define MXC_F_GCR_BTLELDOCTRL_BB_EN_DLY                ((uint32_t)(0x1UL << MXC_F_GCR_BTLELDOCTRL_BB_EN_DLY_POS)) /**< BTLELDOCTRL_BB_EN_DLY Mask */

#define MXC_F_GCR_BTLELDOCTRL_RF_EN_DLY_POS            13 /**< BTLELDOCTRL_RF_EN_DLY Position */
#define MXC_F_GCR_BTLELDOCTRL_RF_EN_DLY                ((uint32_t)(0x1UL << MXC_F_GCR_BTLELDOCTRL_RF_EN_DLY_POS)) /**< BTLELDOCTRL_RF_EN_DLY Mask */

#define MXC_F_GCR_BTLELDOCTRL_RF_BP_EN_DLY_POS         14 /**< BTLELDOCTRL_RF_BP_EN_DLY Position */
#define MXC_F_GCR_BTLELDOCTRL_RF_BP_EN_DLY             ((uint32_t)(0x1UL << MXC_F_GCR_BTLELDOCTRL_RF_BP_EN_DLY_POS)) /**< BTLELDOCTRL_RF_BP_EN_DLY Mask */

#define MXC_F_GCR_BTLELDOCTRL_BB_BP_EN_DLY_POS         15 /**< BTLELDOCTRL_BB_BP_EN_DLY Position */
#define MXC_F_GCR_BTLELDOCTRL_BB_BP_EN_DLY             ((uint32_t)(0x1UL << MXC_F_GCR_BTLELDOCTRL_BB_BP_EN_DLY_POS)) /**< BTLELDOCTRL_BB_BP_EN_DLY Mask */

/**@} end of group GCR_BTLELDOCTRL_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_BTLELDODLY GCR_BTLELDODLY
 * @brief    BTLE LDO Delay Register
 * @{
 */
#define MXC_F_GCR_BTLELDODLY_BP_CNT_POS                0 /**< BTLELDODLY_BP_CNT Position */
#define MXC_F_GCR_BTLELDODLY_BP_CNT                    ((uint32_t)(0xFFUL << MXC_F_GCR_BTLELDODLY_BP_CNT_POS)) /**< BTLELDODLY_BP_CNT Mask */

#define MXC_F_GCR_BTLELDODLY_BB_CNT_POS                8 /**< BTLELDODLY_BB_CNT Position */
#define MXC_F_GCR_BTLELDODLY_BB_CNT                    ((uint32_t)(0x1FFUL << MXC_F_GCR_BTLELDODLY_BB_CNT_POS)) /**< BTLELDODLY_BB_CNT Mask */

#define MXC_F_GCR_BTLELDODLY_RF_CNT_POS                20 /**< BTLELDODLY_RF_CNT Position */
#define MXC_F_GCR_BTLELDODLY_RF_CNT                    ((uint32_t)(0x1FFUL << MXC_F_GCR_BTLELDODLY_RF_CNT_POS)) /**< BTLELDODLY_RF_CNT Mask */

/**@} end of group GCR_BTLELDODLY_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_GCR_REGS_H_
