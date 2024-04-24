/**
 * @file    gcr_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the GCR Peripheral Module.
 * @note    This file is @generated.
 * @ingroup gcr_registers
 */

/******************************************************************************
 *
 * Analog Devices, Inc.),
 * Copyright (C) 2024 Analog Devices, Inc.
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
    __IO uint32_t sysie;                /**< <tt>\b 0x54:</tt> GCR SYSIE Register */
    __R  uint32_t rsv_0x58_0x63[3];
    __IO uint32_t eccerr;               /**< <tt>\b 0x64:</tt> GCR ECCERR Register */
    __IO uint32_t rfu;                  /**< <tt>\b 0x68:</tt> GCR RFU Register */
    __IO uint32_t eccie;                /**< <tt>\b 0x6C:</tt> GCR ECCIE Register */
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
#define MXC_R_GCR_SYSIE                    ((uint32_t)0x00000054UL) /**< Offset from GCR Base Address: <tt> 0x0054</tt> */
#define MXC_R_GCR_ECCERR                   ((uint32_t)0x00000064UL) /**< Offset from GCR Base Address: <tt> 0x0064</tt> */
#define MXC_R_GCR_ECCIE                    ((uint32_t)0x0000006CUL) /**< Offset from GCR Base Address: <tt> 0x006C</tt> */
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
#define MXC_F_GCR_SYSCTRL_ICC0_FLUSH_POS               6 /**< SYSCTRL_ICC0_FLUSH Position */
#define MXC_F_GCR_SYSCTRL_ICC0_FLUSH                   ((uint32_t)(0x1UL << MXC_F_GCR_SYSCTRL_ICC0_FLUSH_POS)) /**< SYSCTRL_ICC0_FLUSH Mask */

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

#define MXC_F_GCR_RST0_WDT0_POS                        1 /**< RST0_WDT0 Position */
#define MXC_F_GCR_RST0_WDT0                            ((uint32_t)(0x1UL << MXC_F_GCR_RST0_WDT0_POS)) /**< RST0_WDT0 Mask */

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

#define MXC_F_GCR_RST0_BTLE_POS                        18 /**< RST0_BTLE Position */
#define MXC_F_GCR_RST0_BTLE                            ((uint32_t)(0x1UL << MXC_F_GCR_RST0_BTLE_POS)) /**< RST0_BTLE Mask */

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
#define MXC_V_GCR_PM_MODE_SLEEP                        ((uint32_t)0x1UL) /**< PM_MODE_SLEEP Value */
#define MXC_S_GCR_PM_MODE_SLEEP                        (MXC_V_GCR_PM_MODE_SLEEP << MXC_F_GCR_PM_MODE_POS) /**< PM_MODE_SLEEP Setting */
#define MXC_V_GCR_PM_MODE_STANDBY                      ((uint32_t)0x2UL) /**< PM_MODE_STANDBY Value */
#define MXC_S_GCR_PM_MODE_STANDBY                      (MXC_V_GCR_PM_MODE_STANDBY << MXC_F_GCR_PM_MODE_POS) /**< PM_MODE_STANDBY Setting */
#define MXC_V_GCR_PM_MODE_BACKUP                       ((uint32_t)0x4UL) /**< PM_MODE_BACKUP Value */
#define MXC_S_GCR_PM_MODE_BACKUP                       (MXC_V_GCR_PM_MODE_BACKUP << MXC_F_GCR_PM_MODE_POS) /**< PM_MODE_BACKUP Setting */
#define MXC_V_GCR_PM_MODE_POWERDOWN                    ((uint32_t)0xAUL) /**< PM_MODE_POWERDOWN Value */
#define MXC_S_GCR_PM_MODE_POWERDOWN                    (MXC_V_GCR_PM_MODE_POWERDOWN << MXC_F_GCR_PM_MODE_POS) /**< PM_MODE_POWERDOWN Setting */

#define MXC_F_GCR_PM_GPIO_WE_POS                       4 /**< PM_GPIO_WE Position */
#define MXC_F_GCR_PM_GPIO_WE                           ((uint32_t)(0x1UL << MXC_F_GCR_PM_GPIO_WE_POS)) /**< PM_GPIO_WE Mask */

#define MXC_F_GCR_PM_RTC_WE_POS                        5 /**< PM_RTC_WE Position */
#define MXC_F_GCR_PM_RTC_WE                            ((uint32_t)(0x1UL << MXC_F_GCR_PM_RTC_WE_POS)) /**< PM_RTC_WE Mask */

#define MXC_F_GCR_PM_WUT_WE_POS                        7 /**< PM_WUT_WE Position */
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
#define MXC_F_GCR_MEMZ_RAM0_POS                        0 /**< MEMZ_RAM0 Position */
#define MXC_F_GCR_MEMZ_RAM0                            ((uint32_t)(0x1UL << MXC_F_GCR_MEMZ_RAM0_POS)) /**< MEMZ_RAM0 Mask */

#define MXC_F_GCR_MEMZ_RAM1_POS                        1 /**< MEMZ_RAM1 Position */
#define MXC_F_GCR_MEMZ_RAM1                            ((uint32_t)(0x1UL << MXC_F_GCR_MEMZ_RAM1_POS)) /**< MEMZ_RAM1 Mask */

#define MXC_F_GCR_MEMZ_RAM2_POS                        2 /**< MEMZ_RAM2 Position */
#define MXC_F_GCR_MEMZ_RAM2                            ((uint32_t)(0x1UL << MXC_F_GCR_MEMZ_RAM2_POS)) /**< MEMZ_RAM2 Mask */

#define MXC_F_GCR_MEMZ_RAM3_POS                        3 /**< MEMZ_RAM3 Position */
#define MXC_F_GCR_MEMZ_RAM3                            ((uint32_t)(0x1UL << MXC_F_GCR_MEMZ_RAM3_POS)) /**< MEMZ_RAM3 Mask */

#define MXC_F_GCR_MEMZ_SYSRAM0ECC_POS                  4 /**< MEMZ_SYSRAM0ECC Position */
#define MXC_F_GCR_MEMZ_SYSRAM0ECC                      ((uint32_t)(0x1UL << MXC_F_GCR_MEMZ_SYSRAM0ECC_POS)) /**< MEMZ_SYSRAM0ECC Mask */

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

#define MXC_F_GCR_PCLKDIS1_SPI_POS                     16 /**< PCLKDIS1_SPI Position */
#define MXC_F_GCR_PCLKDIS1_SPI                         ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS1_SPI_POS)) /**< PCLKDIS1_SPI Mask */

#define MXC_F_GCR_PCLKDIS1_WDT0_POS                    27 /**< PCLKDIS1_WDT0 Position */
#define MXC_F_GCR_PCLKDIS1_WDT0                        ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS1_WDT0_POS)) /**< PCLKDIS1_WDT0 Mask */

/**@} end of group GCR_PCLKDIS1_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_EVENTEN GCR_EVENTEN
 * @brief    Event Enable Register.
 * @{
 */

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
 * @defgroup GCR_SYSIE GCR_SYSIE
 * @brief    System Status Interrupt Enable Register.
 * @{
 */
#define MXC_F_GCR_SYSIE_ICEUNLOCK_POS                  0 /**< SYSIE_ICEUNLOCK Position */
#define MXC_F_GCR_SYSIE_ICEUNLOCK                      ((uint32_t)(0x1UL << MXC_F_GCR_SYSIE_ICEUNLOCK_POS)) /**< SYSIE_ICEUNLOCK Mask */

/**@} end of group GCR_SYSIE_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_ECCERR GCR_ECCERR
 * @brief    ECC Error Register
 * @{
 */
#define MXC_F_GCR_FLECCERR_POS                         0 /**< FLECCERR Position */
#define MXC_F_GCR_FLECCERR                             ((uint32_t)(0x1UL << MXC_F_GCR_FECCERR_POS)) /**< FLECCERR Mask */

/**@} end of group GCR_ECCERR_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_ECCIE GCR_ECCIE
 * @brief    ECC IRQ Enable Register
 * @{
 */
#define MXC_F_GCR_ECCIE_FLLIRQEN_POS                   0 /**< FLLIRQEN Position */
#define MXC_F_GCR_ECCIE_FLLIRQEN                       ((uint32_t)(0x1UL << MXC_F_GCR_ECCIE_FLLIRQEN_POS)) /**< FLLIRQEN Mask */

/**@} end of group GCR_ECCIE_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_ECCADDR GCR_ECCADDR
 * @brief    ECC Error Address Register
 * @{
 */
#define MXC_F_GCR_ECCADDR_ECCERRAD_POS                 0 /**< ECCADDR_ECCERRAD Position */
#define MXC_F_GCR_ECCADDR_ECCERRAD                     ((uint32_t)(0xFFFFFFFFUL << MXC_F_GCR_ECCADDR_ECCERRAD_POS)) /**< ECCADDR_ECCERRAD Mask */

/**@} end of group GCR_ECCADDR_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_BTLELDOCTRL GCR_BTLELDOCTRL
 * @brief    BTLE LDO Control Register
 * @{
 */
#define MXC_F_GCR_BTLELDOCTRL_LDORXEN_POS              0 /**< BTLELDOCTRL_LDORXEN Position */
#define MXC_F_GCR_BTLELDOCTRL_LDORXEN                  ((uint32_t)(0x1UL << MXC_F_GCR_BTLELDOCTRL_LDORXEN_POS)) /**< BTLELDOCTRL_LDORXEN Mask */

#define MXC_F_GCR_BTLELDOCTRL_LDORXPULLD_POS           1 /**< BTLELDOCTRL_LDORXPULLD Position */
#define MXC_F_GCR_BTLELDOCTRL_LDORXPULLD               ((uint32_t)(0x1UL << MXC_F_GCR_BTLELDOCTRL_LDORXPULLD_POS)) /**< BTLELDOCTRL_LDORXPULLD Mask */

#define MXC_F_GCR_BTLELDOCTRL_LDORXVSEL_POS            2 /**< BTLELDOCTRL_LDORXVSEL Position */
#define MXC_F_GCR_BTLELDOCTRL_LDORXVSEL                ((uint32_t)(0x3UL << MXC_F_GCR_BTLELDOCTRL_LDORXVSEL_POS)) /**< BTLELDOCTRL_LDORXVSEL Mask */
#define MXC_V_GCR_BTLELDOCTRL_LDORXVSEL_0_85           ((uint32_t)0x0UL) /**< BTLELDOCTRL_LDOTXVSEL_0_85 Value */
#define MXC_S_GCR_BTLELDOCTRL_LDORXVSEL_0_85           (MXC_V_GCR_BTLELDOCTRL_LDOTXVSEL_0_85 << MXC_F_GCR_BTLELDOCTRL_LDOTXVSEL_POS) /**< BTLELDOCTRL_LDORXVSEL_0_85 Setting */
#define MXC_V_GCR_BTLELDOCTRL_LDORXVSEL_0_9            ((uint32_t)0x1UL) /**< BTLELDOCTRL_LDOTXVSEL_0_9 Value */
#define MXC_S_GCR_BTLELDOCTRL_LDORXVSEL_0_9            (MXC_V_GCR_BTLELDOCTRL_LDOTXVSEL_0_9 << MXC_F_GCR_BTLELDOCTRL_LDOTXVSEL_POS) /**< BTLELDOCTRL_LDORXVSEL_0_9 Setting */
#define MXC_V_GCR_BTLELDOCTRL_LDORXVSEL_1_0            ((uint32_t)0x2UL) /**< BTLELDOCTRL_LDOTXVSEL_1_0 Value */
#define MXC_S_GCR_BTLELDOCTRL_LDORXVSEL_1_0            (MXC_V_GCR_BTLELDOCTRL_LDOTXVSEL_1_0 << MXC_F_GCR_BTLELDOCTRL_LDOTXVSEL_POS) /**< BTLELDOCTRL_LDOTXVSEL_1_0 Setting */
#define MXC_V_GCR_BTLELDOCTRL_LDORXVSEL_1_1            ((uint32_t)0x3UL) /**< BTLELDOCTRL_LDOTXVSEL_1_1 Value */
#define MXC_S_GCR_BTLELDOCTRL_LDORXVSEL_1_1            (MXC_V_GCR_BTLELDOCTRL_LDOTXVSEL_1_1 << MXC_F_GCR_BTLELDOCTRL_LDOTXVSEL_POS) /**< BTLELDOCTRL_LDOTXVSEL_1_1 Setting */

#define MXC_F_GCR_BTLELDOCTRL_LDORXVSEL0_POS           2 /**< BTLELDOCTRL_LDORXVSEL0 Position */
#define MXC_F_GCR_BTLELDOCTRL_LDORXVSEL0               ((uint32_t)(0x1UL << MXC_F_GCR_BTLELDOCTRL_LDORXVSEL0_POS)) /**< BTLELDOCTRL_LDORXVSEL0 Mask */

#define MXC_F_GCR_BTLELDOCTRL_LDORXVSEL1_POS           3 /**< BTLELDOCTRL_LDORXVSEL1 Position */
#define MXC_F_GCR_BTLELDOCTRL_LDORXVSEL1               ((uint32_t)(0x1UL << MXC_F_GCR_BTLELDOCTRL_LDORXVSEL1_POS)) /**< BTLELDOCTRL_LDORXVSEL1 Mask */

#define MXC_F_GCR_BTLELDOCTRL_LDOTXEN_POS              4 /**< BTLELDOCTRL_LDOTXEN Position */
#define MXC_F_GCR_BTLELDOCTRL_LDOTXEN                  ((uint32_t)(0x1UL << MXC_F_GCR_BTLELDOCTRL_LDOTXEN_POS)) /**< BTLELDOCTRL_LDOTXEN Mask */

#define MXC_F_GCR_BTLELDOCTRL_LDOTXPULLD_POS           5 /**< BTLELDOCTRL_LDOTXPULLD Position */
#define MXC_F_GCR_BTLELDOCTRL_LDOTXPULLD               ((uint32_t)(0x1UL << MXC_F_GCR_BTLELDOCTRL_LDOTXPULLD_POS)) /**< BTLELDOCTRL_LDOTXPULLD Mask */

#define MXC_F_GCR_BTLELDOCTRL_LDOTXVSEL_POS            6 /**< BTLELDOCTRL_LDOTXVSEL Position */
#define MXC_F_GCR_BTLELDOCTRL_LDOTXVSEL                ((uint32_t)(0x3UL << MXC_F_GCR_BTLELDOCTRL_LDOTXVSEL_POS)) /**< BTLELDOCTRL_LDOTXVSEL Mask */
#define MXC_V_GCR_BTLELDOCTRL_LDOTXVSEL_0_85           ((uint32_t)0x0UL) /**< BTLELDOCTRL_LDOTXVSEL_0_85 Value */
#define MXC_S_GCR_BTLELDOCTRL_LDOTXVSEL_0_85           (MXC_V_GCR_BTLELDOCTRL_LDOTXVSEL_0_85 << MXC_F_GCR_BTLELDOCTRL_LDOTXVSEL_POS) /**< BTLELDOCTRL_LDOTXVSEL_0_85 Setting */
#define MXC_V_GCR_BTLELDOCTRL_LDOTXVSEL_0_9            ((uint32_t)0x1UL) /**< BTLELDOCTRL_LDOTXVSEL_0_9 Value */
#define MXC_S_GCR_BTLELDOCTRL_LDOTXVSEL_0_9            (MXC_V_GCR_BTLELDOCTRL_LDOTXVSEL_0_9 << MXC_F_GCR_BTLELDOCTRL_LDOTXVSEL_POS) /**< BTLELDOCTRL_LDOTXVSEL_0_9 Setting */
#define MXC_V_GCR_BTLELDOCTRL_LDOTXVSEL_1_0            ((uint32_t)0x2UL) /**< BTLELDOCTRL_LDOTXVSEL_1_0 Value */
#define MXC_S_GCR_BTLELDOCTRL_LDOTXVSEL_1_0            (MXC_V_GCR_BTLELDOCTRL_LDOTXVSEL_1_0 << MXC_F_GCR_BTLELDOCTRL_LDOTXVSEL_POS) /**< BTLELDOCTRL_LDOTXVSEL_1_0 Setting */
#define MXC_V_GCR_BTLELDOCTRL_LDOTXVSEL_1_1            ((uint32_t)0x3UL) /**< BTLELDOCTRL_LDOTXVSEL_1_1 Value */
#define MXC_S_GCR_BTLELDOCTRL_LDOTXVSEL_1_1            (MXC_V_GCR_BTLELDOCTRL_LDOTXVSEL_1_1 << MXC_F_GCR_BTLELDOCTRL_LDOTXVSEL_POS) /**< BTLELDOCTRL_LDOTXVSEL_1_1 Setting */

#define MXC_F_GCR_BTLELDOCTRL_LDOTXVSEL0_POS           6 /**< BTLELDOCTRL_LDOTXVSEL0 Position */
#define MXC_F_GCR_BTLELDOCTRL_LDOTXVSEL0               ((uint32_t)(0x1UL << MXC_F_GCR_BTLELDOCTRL_LDOTXVSEL0_POS)) /**< BTLELDOCTRL_LDOTXVSEL0 Mask */

#define MXC_F_GCR_BTLELDOCTRL_LDOTXVSEL1_POS           7 /**< BTLELDOCTRL_LDOTXVSEL1 Position */
#define MXC_F_GCR_BTLELDOCTRL_LDOTXVSEL1               ((uint32_t)(0x1UL << MXC_F_GCR_BTLELDOCTRL_LDOTXVSEL1_POS)) /**< BTLELDOCTRL_LDOTXVSEL1 Mask */

#define MXC_F_GCR_BTLELDOCTRL_LDOTXBYP_POS             8 /**< BTLELDOCTRL_LDOTXBYP Position */
#define MXC_F_GCR_BTLELDOCTRL_LDOTXBYP                 ((uint32_t)(0x1UL << MXC_F_GCR_BTLELDOCTRL_LDOTXBYP_POS)) /**< BTLELDOCTRL_LDOTXBYP Mask */

#define MXC_F_GCR_BTLELDOCTRL_LDOTXDISCH_POS           9 /**< BTLELDOCTRL_LDOTXDISCH Position */
#define MXC_F_GCR_BTLELDOCTRL_LDOTXDISCH               ((uint32_t)(0x1UL << MXC_F_GCR_BTLELDOCTRL_LDOTXDISCH_POS)) /**< BTLELDOCTRL_LDOTXDISCH Mask */

#define MXC_F_GCR_BTLELDOCTRL_LDORXBYP_POS             10 /**< BTLELDOCTRL_LDORXBYP Position */
#define MXC_F_GCR_BTLELDOCTRL_LDORXBYP                 ((uint32_t)(0x1UL << MXC_F_GCR_BTLELDOCTRL_LDORXBYP_POS)) /**< BTLELDOCTRL_LDORXBYP Mask */

#define MXC_F_GCR_BTLELDOCTRL_LDORXDISCH_POS           11 /**< BTLELDOCTRL_LDORXDISCH Position */
#define MXC_F_GCR_BTLELDOCTRL_LDORXDISCH               ((uint32_t)(0x1UL << MXC_F_GCR_BTLELDOCTRL_LDORXDISCH_POS)) /**< BTLELDOCTRL_LDORXDISCH Mask */

#define MXC_F_GCR_BTLELDOCTRL_LDORXENDLY_POS           12 /**< BTLELDOCTRL_LDORXENDLY Position */
#define MXC_F_GCR_BTLELDOCTRL_LDORXENDLY               ((uint32_t)(0x1UL << MXC_F_GCR_BTLELDOCTRL_LDORXENDLY_POS)) /**< BTLELDOCTRL_LDORXENDLY Mask */

#define MXC_F_GCR_BTLELDOCTRL_LDOTXENDLY_POS           13 /**< BTLELDOCTRL_LDOTXENDLY Position */
#define MXC_F_GCR_BTLELDOCTRL_LDOTXENDLY               ((uint32_t)(0x1UL << MXC_F_GCR_BTLELDOCTRL_LDOTXENDLY_POS)) /**< BTLELDOCTRL_LDOTXENDLY Mask */

#define MXC_F_GCR_BTLELDOCTRL_LDOTXBYPENENDLY_POS      14 /**< BTLELDOCTRL_LDOTXBYPENENDLY Position */
#define MXC_F_GCR_BTLELDOCTRL_LDOTXBYPENENDLY          ((uint32_t)(0x1UL << MXC_F_GCR_BTLELDOCTRL_LDOTXBYPENENDLY_POS)) /**< BTLELDOCTRL_LDOTXBYPENENDLY Mask */

#define MXC_F_GCR_BTLELDOCTRL_LDORXBYPENENDLY_POS      15 /**< BTLELDOCTRL_LDORXBYPENENDLY Position */
#define MXC_F_GCR_BTLELDOCTRL_LDORXBYPENENDLY          ((uint32_t)(0x1UL << MXC_F_GCR_BTLELDOCTRL_LDORXBYPENENDLY_POS)) /**< BTLELDOCTRL_LDORXBYPENENDLY Mask */

/**@} end of group GCR_BTLELDOCTRL_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_BTLELDODLY GCR_BTLELDODLY
 * @brief    BTLE LDO Delay Register
 * @{
 */
#define MXC_F_GCR_BTLELDODLY_BYPDLYCNT_POS             0 /**< BTLELDODLY_BYPDLYCNT Position */
#define MXC_F_GCR_BTLELDODLY_BYPDLYCNT                 ((uint32_t)(0xFFUL << MXC_F_GCR_BTLELDODLY_BYPDLYCNT_POS)) /**< BTLELDODLY_BYPDLYCNT Mask */

#define MXC_F_GCR_BTLELDODLY_LDORXDLYCNT_POS           8 /**< BTLELDODLY_LDORXDLYCNT Position */
#define MXC_F_GCR_BTLELDODLY_LDORXDLYCNT               ((uint32_t)(0x1FFUL << MXC_F_GCR_BTLELDODLY_LDORXDLYCNT_POS)) /**< BTLELDODLY_LDORXDLYCNT Mask */

#define MXC_F_GCR_BTLELDODLY_LDOTXDLYCNT_POS           20 /**< BTLELDODLY_LDOTXDLYCNT Position */
#define MXC_F_GCR_BTLELDODLY_LDOTXDLYCNT               ((uint32_t)(0x1FFUL << MXC_F_GCR_BTLELDODLY_LDOTXDLYCNT_POS)) /**< BTLELDODLY_LDOTXDLYCNT Mask */

/**@} end of group GCR_BTLELDODLY_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_GPR GCR_GPR
 * @brief    General Purpose Register
 * @{
 */
#define MXC_F_GCR_GPR_POS                              0 /**< GPR Position */
#define MXC_F_GCR_GPR                                  ((uint32_t)(0xFFFFFFFFUL << MXC_F_GCR_GPR_POS)) /**< GPR Mask */

/**@} end of group GCR_GPR_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_GCR_REGS_H_
