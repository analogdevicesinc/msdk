/**
 * @file    gcr_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the GCR Peripheral Module.
 * @note    This file is @generated.
 * @ingroup gcr_registers
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32662_INCLUDE_GCR_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32662_INCLUDE_GCR_REGS_H_

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
    __IO uint32_t eccced;               /**< <tt>\b 0x68:</tt> GCR ECCCED Register */
    __IO uint32_t eccie;                /**< <tt>\b 0x6C:</tt> GCR ECCIE Register */
    __IO uint32_t eccaddr;              /**< <tt>\b 0x70:</tt> GCR ECCADDR Register */
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
#define MXC_R_GCR_ECCCED                   ((uint32_t)0x00000068UL) /**< Offset from GCR Base Address: <tt> 0x0068</tt> */
#define MXC_R_GCR_ECCIE                    ((uint32_t)0x0000006CUL) /**< Offset from GCR Base Address: <tt> 0x006C</tt> */
#define MXC_R_GCR_ECCADDR                  ((uint32_t)0x00000070UL) /**< Offset from GCR Base Address: <tt> 0x0070</tt> */
/**@} end of group gcr_registers */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_SYSCTRL GCR_SYSCTRL
 * @brief    System Control.
 * @{
 */
#define MXC_F_GCR_SYSCTRL_SBUSARB_POS                  1 /**< SYSCTRL_SBUSARB Position */
#define MXC_F_GCR_SYSCTRL_SBUSARB                      ((uint32_t)(0x3UL << MXC_F_GCR_SYSCTRL_SBUSARB_POS)) /**< SYSCTRL_SBUSARB Mask */

#define MXC_F_GCR_SYSCTRL_FPUS_DIS_POS                 5 /**< SYSCTRL_FPUS_DIS Position */
#define MXC_F_GCR_SYSCTRL_FPUS_DIS                     ((uint32_t)(0x1UL << MXC_F_GCR_SYSCTRL_FPUS_DIS_POS)) /**< SYSCTRL_FPUS_DIS Mask */

#define MXC_F_GCR_SYSCTRL_ICC0_FLUSH_POS               6 /**< SYSCTRL_ICC0_FLUSH Position */
#define MXC_F_GCR_SYSCTRL_ICC0_FLUSH                   ((uint32_t)(0x1UL << MXC_F_GCR_SYSCTRL_ICC0_FLUSH_POS)) /**< SYSCTRL_ICC0_FLUSH Mask */

#define MXC_F_GCR_SYSCTRL_CCHK_POS                     13 /**< SYSCTRL_CCHK Position */
#define MXC_F_GCR_SYSCTRL_CCHK                         ((uint32_t)(0x1UL << MXC_F_GCR_SYSCTRL_CCHK_POS)) /**< SYSCTRL_CCHK Mask */

#define MXC_F_GCR_SYSCTRL_SWD_DIS_POS                  14 /**< SYSCTRL_SWD_DIS Position */
#define MXC_F_GCR_SYSCTRL_SWD_DIS                      ((uint32_t)(0x1UL << MXC_F_GCR_SYSCTRL_SWD_DIS_POS)) /**< SYSCTRL_SWD_DIS Mask */

#define MXC_F_GCR_SYSCTRL_CHKRES_POS                   15 /**< SYSCTRL_CHKRES Position */
#define MXC_F_GCR_SYSCTRL_CHKRES                       ((uint32_t)(0x1UL << MXC_F_GCR_SYSCTRL_CHKRES_POS)) /**< SYSCTRL_CHKRES Mask */

/**@} end of group GCR_SYSCTRL_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_RST0 GCR_RST0
 * @brief    Reset.
 * @{
 */
#define MXC_F_GCR_RST0_DMA_POS                         0 /**< RST0_DMA Position */
#define MXC_F_GCR_RST0_DMA                             ((uint32_t)(0x1UL << MXC_F_GCR_RST0_DMA_POS)) /**< RST0_DMA Mask */

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

#define MXC_F_GCR_RST0_UART0_POS                       11 /**< RST0_UART0 Position */
#define MXC_F_GCR_RST0_UART0                           ((uint32_t)(0x1UL << MXC_F_GCR_RST0_UART0_POS)) /**< RST0_UART0 Mask */

#define MXC_F_GCR_RST0_UART1_POS                       12 /**< RST0_UART1 Position */
#define MXC_F_GCR_RST0_UART1                           ((uint32_t)(0x1UL << MXC_F_GCR_RST0_UART1_POS)) /**< RST0_UART1 Mask */

#define MXC_F_GCR_RST0_SPI0_POS                        13 /**< RST0_SPI0 Position */
#define MXC_F_GCR_RST0_SPI0                            ((uint32_t)(0x1UL << MXC_F_GCR_RST0_SPI0_POS)) /**< RST0_SPI0 Mask */

#define MXC_F_GCR_RST0_SPI1_POS                        14 /**< RST0_SPI1 Position */
#define MXC_F_GCR_RST0_SPI1                            ((uint32_t)(0x1UL << MXC_F_GCR_RST0_SPI1_POS)) /**< RST0_SPI1 Mask */

#define MXC_F_GCR_RST0_I2C0_POS                        16 /**< RST0_I2C0 Position */
#define MXC_F_GCR_RST0_I2C0                            ((uint32_t)(0x1UL << MXC_F_GCR_RST0_I2C0_POS)) /**< RST0_I2C0 Mask */

#define MXC_F_GCR_RST0_CAN_POS                         19 /**< RST0_CAN Position */
#define MXC_F_GCR_RST0_CAN                             ((uint32_t)(0x1UL << MXC_F_GCR_RST0_CAN_POS)) /**< RST0_CAN Mask */

#define MXC_F_GCR_RST0_TRNG_POS                        24 /**< RST0_TRNG Position */
#define MXC_F_GCR_RST0_TRNG                            ((uint32_t)(0x1UL << MXC_F_GCR_RST0_TRNG_POS)) /**< RST0_TRNG Mask */

#define MXC_F_GCR_RST0_ADC_POS                         26 /**< RST0_ADC Position */
#define MXC_F_GCR_RST0_ADC                             ((uint32_t)(0x1UL << MXC_F_GCR_RST0_ADC_POS)) /**< RST0_ADC Mask */

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
#define MXC_V_GCR_CLKCTRL_SYSCLK_SEL_ERFO              ((uint32_t)0x2UL) /**< CLKCTRL_SYSCLK_SEL_ERFO Value */
#define MXC_S_GCR_CLKCTRL_SYSCLK_SEL_ERFO              (MXC_V_GCR_CLKCTRL_SYSCLK_SEL_ERFO << MXC_F_GCR_CLKCTRL_SYSCLK_SEL_POS) /**< CLKCTRL_SYSCLK_SEL_ERFO Setting */
#define MXC_V_GCR_CLKCTRL_SYSCLK_SEL_INRO              ((uint32_t)0x3UL) /**< CLKCTRL_SYSCLK_SEL_INRO Value */
#define MXC_S_GCR_CLKCTRL_SYSCLK_SEL_INRO              (MXC_V_GCR_CLKCTRL_SYSCLK_SEL_INRO << MXC_F_GCR_CLKCTRL_SYSCLK_SEL_POS) /**< CLKCTRL_SYSCLK_SEL_INRO Setting */
#define MXC_V_GCR_CLKCTRL_SYSCLK_SEL_IPO               ((uint32_t)0x4UL) /**< CLKCTRL_SYSCLK_SEL_IPO Value */
#define MXC_S_GCR_CLKCTRL_SYSCLK_SEL_IPO               (MXC_V_GCR_CLKCTRL_SYSCLK_SEL_IPO << MXC_F_GCR_CLKCTRL_SYSCLK_SEL_POS) /**< CLKCTRL_SYSCLK_SEL_IPO Setting */
#define MXC_V_GCR_CLKCTRL_SYSCLK_SEL_IBRO              ((uint32_t)0x5UL) /**< CLKCTRL_SYSCLK_SEL_IBRO Value */
#define MXC_S_GCR_CLKCTRL_SYSCLK_SEL_IBRO              (MXC_V_GCR_CLKCTRL_SYSCLK_SEL_IBRO << MXC_F_GCR_CLKCTRL_SYSCLK_SEL_POS) /**< CLKCTRL_SYSCLK_SEL_IBRO Setting */
#define MXC_V_GCR_CLKCTRL_SYSCLK_SEL_ERTCO             ((uint32_t)0x6UL) /**< CLKCTRL_SYSCLK_SEL_ERTCO Value */
#define MXC_S_GCR_CLKCTRL_SYSCLK_SEL_ERTCO             (MXC_V_GCR_CLKCTRL_SYSCLK_SEL_ERTCO << MXC_F_GCR_CLKCTRL_SYSCLK_SEL_POS) /**< CLKCTRL_SYSCLK_SEL_ERTCO Setting */
#define MXC_V_GCR_CLKCTRL_SYSCLK_SEL_EXTCLK            ((uint32_t)0x7UL) /**< CLKCTRL_SYSCLK_SEL_EXTCLK Value */
#define MXC_S_GCR_CLKCTRL_SYSCLK_SEL_EXTCLK            (MXC_V_GCR_CLKCTRL_SYSCLK_SEL_EXTCLK << MXC_F_GCR_CLKCTRL_SYSCLK_SEL_POS) /**< CLKCTRL_SYSCLK_SEL_EXTCLK Setting */

#define MXC_F_GCR_CLKCTRL_SYSCLK_RDY_POS               13 /**< CLKCTRL_SYSCLK_RDY Position */
#define MXC_F_GCR_CLKCTRL_SYSCLK_RDY                   ((uint32_t)(0x1UL << MXC_F_GCR_CLKCTRL_SYSCLK_RDY_POS)) /**< CLKCTRL_SYSCLK_RDY Mask */

#define MXC_F_GCR_CLKCTRL_IPO_DIV_POS                  14 /**< CLKCTRL_IPO_DIV Position */
#define MXC_F_GCR_CLKCTRL_IPO_DIV                      ((uint32_t)(0x3UL << MXC_F_GCR_CLKCTRL_IPO_DIV_POS)) /**< CLKCTRL_IPO_DIV Mask */
#define MXC_V_GCR_CLKCTRL_IPO_DIV_DIV1                 ((uint32_t)0x0UL) /**< CLKCTRL_IPO_DIV_DIV1 Value */
#define MXC_S_GCR_CLKCTRL_IPO_DIV_DIV1                 (MXC_V_GCR_CLKCTRL_IPO_DIV_DIV1 << MXC_F_GCR_CLKCTRL_IPO_DIV_POS) /**< CLKCTRL_IPO_DIV_DIV1 Setting */
#define MXC_V_GCR_CLKCTRL_IPO_DIV_DIV2                 ((uint32_t)0x1UL) /**< CLKCTRL_IPO_DIV_DIV2 Value */
#define MXC_S_GCR_CLKCTRL_IPO_DIV_DIV2                 (MXC_V_GCR_CLKCTRL_IPO_DIV_DIV2 << MXC_F_GCR_CLKCTRL_IPO_DIV_POS) /**< CLKCTRL_IPO_DIV_DIV2 Setting */
#define MXC_V_GCR_CLKCTRL_IPO_DIV_DIV4                 ((uint32_t)0x2UL) /**< CLKCTRL_IPO_DIV_DIV4 Value */
#define MXC_S_GCR_CLKCTRL_IPO_DIV_DIV4                 (MXC_V_GCR_CLKCTRL_IPO_DIV_DIV4 << MXC_F_GCR_CLKCTRL_IPO_DIV_POS) /**< CLKCTRL_IPO_DIV_DIV4 Setting */
#define MXC_V_GCR_CLKCTRL_IPO_DIV_DIV8                 ((uint32_t)0x3UL) /**< CLKCTRL_IPO_DIV_DIV8 Value */
#define MXC_S_GCR_CLKCTRL_IPO_DIV_DIV8                 (MXC_V_GCR_CLKCTRL_IPO_DIV_DIV8 << MXC_F_GCR_CLKCTRL_IPO_DIV_POS) /**< CLKCTRL_IPO_DIV_DIV8 Setting */

#define MXC_F_GCR_CLKCTRL_ERFO_EN_POS                  16 /**< CLKCTRL_ERFO_EN Position */
#define MXC_F_GCR_CLKCTRL_ERFO_EN                      ((uint32_t)(0x1UL << MXC_F_GCR_CLKCTRL_ERFO_EN_POS)) /**< CLKCTRL_ERFO_EN Mask */

#define MXC_F_GCR_CLKCTRL_IPO_EN_POS                   19 /**< CLKCTRL_IPO_EN Position */
#define MXC_F_GCR_CLKCTRL_IPO_EN                       ((uint32_t)(0x1UL << MXC_F_GCR_CLKCTRL_IPO_EN_POS)) /**< CLKCTRL_IPO_EN Mask */

#define MXC_F_GCR_CLKCTRL_IBRO_EN_POS                  20 /**< CLKCTRL_IBRO_EN Position */
#define MXC_F_GCR_CLKCTRL_IBRO_EN                      ((uint32_t)(0x1UL << MXC_F_GCR_CLKCTRL_IBRO_EN_POS)) /**< CLKCTRL_IBRO_EN Mask */

#define MXC_F_GCR_CLKCTRL_IBRO_VS_POS                  21 /**< CLKCTRL_IBRO_VS Position */
#define MXC_F_GCR_CLKCTRL_IBRO_VS                      ((uint32_t)(0x1UL << MXC_F_GCR_CLKCTRL_IBRO_VS_POS)) /**< CLKCTRL_IBRO_VS Mask */

#define MXC_F_GCR_CLKCTRL_ERFO_RDY_POS                 24 /**< CLKCTRL_ERFO_RDY Position */
#define MXC_F_GCR_CLKCTRL_ERFO_RDY                     ((uint32_t)(0x1UL << MXC_F_GCR_CLKCTRL_ERFO_RDY_POS)) /**< CLKCTRL_ERFO_RDY Mask */

#define MXC_F_GCR_CLKCTRL_ERTCO_RDY_POS                25 /**< CLKCTRL_ERTCO_RDY Position */
#define MXC_F_GCR_CLKCTRL_ERTCO_RDY                    ((uint32_t)(0x1UL << MXC_F_GCR_CLKCTRL_ERTCO_RDY_POS)) /**< CLKCTRL_ERTCO_RDY Mask */

#define MXC_F_GCR_CLKCTRL_IPO_RDY_POS                  27 /**< CLKCTRL_IPO_RDY Position */
#define MXC_F_GCR_CLKCTRL_IPO_RDY                      ((uint32_t)(0x1UL << MXC_F_GCR_CLKCTRL_IPO_RDY_POS)) /**< CLKCTRL_IPO_RDY Mask */

#define MXC_F_GCR_CLKCTRL_IBRO_RDY_POS                 28 /**< CLKCTRL_IBRO_RDY Position */
#define MXC_F_GCR_CLKCTRL_IBRO_RDY                     ((uint32_t)(0x1UL << MXC_F_GCR_CLKCTRL_IBRO_RDY_POS)) /**< CLKCTRL_IBRO_RDY Mask */

#define MXC_F_GCR_CLKCTRL_INRO_RDY_POS                 29 /**< CLKCTRL_INRO_RDY Position */
#define MXC_F_GCR_CLKCTRL_INRO_RDY                     ((uint32_t)(0x1UL << MXC_F_GCR_CLKCTRL_INRO_RDY_POS)) /**< CLKCTRL_INRO_RDY Mask */

#define MXC_F_GCR_CLKCTRL_EXTCLK_RDY_POS               31 /**< CLKCTRL_EXTCLK_RDY Position */
#define MXC_F_GCR_CLKCTRL_EXTCLK_RDY                   ((uint32_t)(0x1UL << MXC_F_GCR_CLKCTRL_EXTCLK_RDY_POS)) /**< CLKCTRL_EXTCLK_RDY Mask */

/**@} end of group GCR_CLKCTRL_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_PM GCR_PM
 * @brief    Power Management.
 * @{
 */
#define MXC_F_GCR_PM_MODE_POS                          0 /**< PM_MODE Position */
#define MXC_F_GCR_PM_MODE                              ((uint32_t)(0x7UL << MXC_F_GCR_PM_MODE_POS)) /**< PM_MODE Mask */
#define MXC_V_GCR_PM_MODE_ACTIVE                       ((uint32_t)0x0UL) /**< PM_MODE_ACTIVE Value */
#define MXC_S_GCR_PM_MODE_ACTIVE                       (MXC_V_GCR_PM_MODE_ACTIVE << MXC_F_GCR_PM_MODE_POS) /**< PM_MODE_ACTIVE Setting */
#define MXC_V_GCR_PM_MODE_BACKUP                       ((uint32_t)0x4UL) /**< PM_MODE_BACKUP Value */
#define MXC_S_GCR_PM_MODE_BACKUP                       (MXC_V_GCR_PM_MODE_BACKUP << MXC_F_GCR_PM_MODE_POS) /**< PM_MODE_BACKUP Setting */
#define MXC_V_GCR_PM_MODE_SHUTDOWN                     ((uint32_t)0x7UL) /**< PM_MODE_SHUTDOWN Value */
#define MXC_S_GCR_PM_MODE_SHUTDOWN                     (MXC_V_GCR_PM_MODE_SHUTDOWN << MXC_F_GCR_PM_MODE_POS) /**< PM_MODE_SHUTDOWN Setting */

#define MXC_F_GCR_PM_GPIO_WE_POS                       4 /**< PM_GPIO_WE Position */
#define MXC_F_GCR_PM_GPIO_WE                           ((uint32_t)(0x1UL << MXC_F_GCR_PM_GPIO_WE_POS)) /**< PM_GPIO_WE Mask */

#define MXC_F_GCR_PM_RTC_WE_POS                        5 /**< PM_RTC_WE Position */
#define MXC_F_GCR_PM_RTC_WE                            ((uint32_t)(0x1UL << MXC_F_GCR_PM_RTC_WE_POS)) /**< PM_RTC_WE Mask */

#define MXC_F_GCR_PM_TMR3_WE_POS                       6 /**< PM_TMR3_WE Position */
#define MXC_F_GCR_PM_TMR3_WE                           ((uint32_t)(0x1UL << MXC_F_GCR_PM_TMR3_WE_POS)) /**< PM_TMR3_WE Mask */

#define MXC_F_GCR_PM_AINCOMP_WE_POS                    7 /**< PM_AINCOMP_WE Position */
#define MXC_F_GCR_PM_AINCOMP_WE                        ((uint32_t)(0x1UL << MXC_F_GCR_PM_AINCOMP_WE_POS)) /**< PM_AINCOMP_WE Mask */

#define MXC_F_GCR_PM_ERFO_BP_POS                       20 /**< PM_ERFO_BP Position */
#define MXC_F_GCR_PM_ERFO_BP                           ((uint32_t)(0x1UL << MXC_F_GCR_PM_ERFO_BP_POS)) /**< PM_ERFO_BP Mask */

/**@} end of group GCR_PM_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_PCLKDIV GCR_PCLKDIV
 * @brief    Peripheral Clock Divider.
 * @{
 */
#define MXC_F_GCR_PCLKDIV_AON_CLKDIV_POS               0 /**< PCLKDIV_AON_CLKDIV Position */
#define MXC_F_GCR_PCLKDIV_AON_CLKDIV                   ((uint32_t)(0x3UL << MXC_F_GCR_PCLKDIV_AON_CLKDIV_POS)) /**< PCLKDIV_AON_CLKDIV Mask */
#define MXC_V_GCR_PCLKDIV_AON_CLKDIV_DIV4              ((uint32_t)0x0UL) /**< PCLKDIV_AON_CLKDIV_DIV4 Value */
#define MXC_S_GCR_PCLKDIV_AON_CLKDIV_DIV4              (MXC_V_GCR_PCLKDIV_AON_CLKDIV_DIV4 << MXC_F_GCR_PCLKDIV_AON_CLKDIV_POS) /**< PCLKDIV_AON_CLKDIV_DIV4 Setting */
#define MXC_V_GCR_PCLKDIV_AON_CLKDIV_DIV8              ((uint32_t)0x1UL) /**< PCLKDIV_AON_CLKDIV_DIV8 Value */
#define MXC_S_GCR_PCLKDIV_AON_CLKDIV_DIV8              (MXC_V_GCR_PCLKDIV_AON_CLKDIV_DIV8 << MXC_F_GCR_PCLKDIV_AON_CLKDIV_POS) /**< PCLKDIV_AON_CLKDIV_DIV8 Setting */
#define MXC_V_GCR_PCLKDIV_AON_CLKDIV_DIV16             ((uint32_t)0x2UL) /**< PCLKDIV_AON_CLKDIV_DIV16 Value */
#define MXC_S_GCR_PCLKDIV_AON_CLKDIV_DIV16             (MXC_V_GCR_PCLKDIV_AON_CLKDIV_DIV16 << MXC_F_GCR_PCLKDIV_AON_CLKDIV_POS) /**< PCLKDIV_AON_CLKDIV_DIV16 Setting */
#define MXC_V_GCR_PCLKDIV_AON_CLKDIV_DIV32             ((uint32_t)0x3UL) /**< PCLKDIV_AON_CLKDIV_DIV32 Value */
#define MXC_S_GCR_PCLKDIV_AON_CLKDIV_DIV32             (MXC_V_GCR_PCLKDIV_AON_CLKDIV_DIV32 << MXC_F_GCR_PCLKDIV_AON_CLKDIV_POS) /**< PCLKDIV_AON_CLKDIV_DIV32 Setting */

/**@} end of group GCR_PCLKDIV_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_PCLKDIS0 GCR_PCLKDIS0
 * @brief    Peripheral Clock Disable.
 * @{
 */
#define MXC_F_GCR_PCLKDIS0_GPIO0_POS                   0 /**< PCLKDIS0_GPIO0 Position */
#define MXC_F_GCR_PCLKDIS0_GPIO0                       ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS0_GPIO0_POS)) /**< PCLKDIS0_GPIO0 Mask */

#define MXC_F_GCR_PCLKDIS0_DMA_POS                     5 /**< PCLKDIS0_DMA Position */
#define MXC_F_GCR_PCLKDIS0_DMA                         ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS0_DMA_POS)) /**< PCLKDIS0_DMA Mask */

#define MXC_F_GCR_PCLKDIS0_SPI0_POS                    6 /**< PCLKDIS0_SPI0 Position */
#define MXC_F_GCR_PCLKDIS0_SPI0                        ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS0_SPI0_POS)) /**< PCLKDIS0_SPI0 Mask */

#define MXC_F_GCR_PCLKDIS0_SPI1_POS                    7 /**< PCLKDIS0_SPI1 Position */
#define MXC_F_GCR_PCLKDIS0_SPI1                        ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS0_SPI1_POS)) /**< PCLKDIS0_SPI1 Mask */

#define MXC_F_GCR_PCLKDIS0_UART0_POS                   9 /**< PCLKDIS0_UART0 Position */
#define MXC_F_GCR_PCLKDIS0_UART0                       ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS0_UART0_POS)) /**< PCLKDIS0_UART0 Mask */

#define MXC_F_GCR_PCLKDIS0_UART1_POS                   10 /**< PCLKDIS0_UART1 Position */
#define MXC_F_GCR_PCLKDIS0_UART1                       ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS0_UART1_POS)) /**< PCLKDIS0_UART1 Mask */

#define MXC_F_GCR_PCLKDIS0_I2C0_POS                    13 /**< PCLKDIS0_I2C0 Position */
#define MXC_F_GCR_PCLKDIS0_I2C0                        ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS0_I2C0_POS)) /**< PCLKDIS0_I2C0 Mask */

#define MXC_F_GCR_PCLKDIS0_TMR0_POS                    15 /**< PCLKDIS0_TMR0 Position */
#define MXC_F_GCR_PCLKDIS0_TMR0                        ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS0_TMR0_POS)) /**< PCLKDIS0_TMR0 Mask */

#define MXC_F_GCR_PCLKDIS0_TMR1_POS                    16 /**< PCLKDIS0_TMR1 Position */
#define MXC_F_GCR_PCLKDIS0_TMR1                        ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS0_TMR1_POS)) /**< PCLKDIS0_TMR1 Mask */

#define MXC_F_GCR_PCLKDIS0_TMR2_POS                    17 /**< PCLKDIS0_TMR2 Position */
#define MXC_F_GCR_PCLKDIS0_TMR2                        ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS0_TMR2_POS)) /**< PCLKDIS0_TMR2 Mask */

#define MXC_F_GCR_PCLKDIS0_ADC_POS                     23 /**< PCLKDIS0_ADC Position */
#define MXC_F_GCR_PCLKDIS0_ADC                         ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS0_ADC_POS)) /**< PCLKDIS0_ADC Mask */

#define MXC_F_GCR_PCLKDIS0_I2C1_POS                    28 /**< PCLKDIS0_I2C1 Position */
#define MXC_F_GCR_PCLKDIS0_I2C1                        ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS0_I2C1_POS)) /**< PCLKDIS0_I2C1 Mask */

#define MXC_F_GCR_PCLKDIS0_PT_POS                      29 /**< PCLKDIS0_PT Position */
#define MXC_F_GCR_PCLKDIS0_PT                          ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS0_PT_POS)) /**< PCLKDIS0_PT Mask */

/**@} end of group GCR_PCLKDIS0_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_MEMCTRL GCR_MEMCTRL
 * @brief    Memory Clock Control Register.
 * @{
 */
#define MXC_F_GCR_MEMCTRL_FWS_POS                      0 /**< MEMCTRL_FWS Position */
#define MXC_F_GCR_MEMCTRL_FWS                          ((uint32_t)(0x7UL << MXC_F_GCR_MEMCTRL_FWS_POS)) /**< MEMCTRL_FWS Mask */

#define MXC_F_GCR_MEMCTRL_RAMWS_EN_POS                 4 /**< MEMCTRL_RAMWS_EN Position */
#define MXC_F_GCR_MEMCTRL_RAMWS_EN                     ((uint32_t)(0x1UL << MXC_F_GCR_MEMCTRL_RAMWS_EN_POS)) /**< MEMCTRL_RAMWS_EN Mask */

#define MXC_F_GCR_MEMCTRL_RAM0LS_EN_POS                8 /**< MEMCTRL_RAM0LS_EN Position */
#define MXC_F_GCR_MEMCTRL_RAM0LS_EN                    ((uint32_t)(0x1UL << MXC_F_GCR_MEMCTRL_RAM0LS_EN_POS)) /**< MEMCTRL_RAM0LS_EN Mask */

#define MXC_F_GCR_MEMCTRL_RAM1LS_EN_POS                9 /**< MEMCTRL_RAM1LS_EN Position */
#define MXC_F_GCR_MEMCTRL_RAM1LS_EN                    ((uint32_t)(0x1UL << MXC_F_GCR_MEMCTRL_RAM1LS_EN_POS)) /**< MEMCTRL_RAM1LS_EN Mask */

#define MXC_F_GCR_MEMCTRL_RAM2LS_EN_POS                10 /**< MEMCTRL_RAM2LS_EN Position */
#define MXC_F_GCR_MEMCTRL_RAM2LS_EN                    ((uint32_t)(0x1UL << MXC_F_GCR_MEMCTRL_RAM2LS_EN_POS)) /**< MEMCTRL_RAM2LS_EN Mask */

#define MXC_F_GCR_MEMCTRL_RAM3LS_EN_POS                11 /**< MEMCTRL_RAM3LS_EN Position */
#define MXC_F_GCR_MEMCTRL_RAM3LS_EN                    ((uint32_t)(0x1UL << MXC_F_GCR_MEMCTRL_RAM3LS_EN_POS)) /**< MEMCTRL_RAM3LS_EN Mask */

#define MXC_F_GCR_MEMCTRL_ICC0LS_EN_POS                12 /**< MEMCTRL_ICC0LS_EN Position */
#define MXC_F_GCR_MEMCTRL_ICC0LS_EN                    ((uint32_t)(0x1UL << MXC_F_GCR_MEMCTRL_ICC0LS_EN_POS)) /**< MEMCTRL_ICC0LS_EN Mask */

#define MXC_F_GCR_MEMCTRL_ROMLS_EN_POS                 13 /**< MEMCTRL_ROMLS_EN Position */
#define MXC_F_GCR_MEMCTRL_ROMLS_EN                     ((uint32_t)(0x1UL << MXC_F_GCR_MEMCTRL_ROMLS_EN_POS)) /**< MEMCTRL_ROMLS_EN Mask */

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

#define MXC_F_GCR_MEMZ_RAMCB_POS                       3 /**< MEMZ_RAMCB Position */
#define MXC_F_GCR_MEMZ_RAMCB                           ((uint32_t)(0x1UL << MXC_F_GCR_MEMZ_RAMCB_POS)) /**< MEMZ_RAMCB Mask */

#define MXC_F_GCR_MEMZ_ICC0_POS                        4 /**< MEMZ_ICC0 Position */
#define MXC_F_GCR_MEMZ_ICC0                            ((uint32_t)(0x1UL << MXC_F_GCR_MEMZ_ICC0_POS)) /**< MEMZ_ICC0 Mask */

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
#define MXC_F_GCR_RST1_I2C1_POS                        0 /**< RST1_I2C1 Position */
#define MXC_F_GCR_RST1_I2C1                            ((uint32_t)(0x1UL << MXC_F_GCR_RST1_I2C1_POS)) /**< RST1_I2C1 Mask */

#define MXC_F_GCR_RST1_PT_POS                          1 /**< RST1_PT Position */
#define MXC_F_GCR_RST1_PT                              ((uint32_t)(0x1UL << MXC_F_GCR_RST1_PT_POS)) /**< RST1_PT Mask */

#define MXC_F_GCR_RST1_AES_POS                         10 /**< RST1_AES Position */
#define MXC_F_GCR_RST1_AES                             ((uint32_t)(0x1UL << MXC_F_GCR_RST1_AES_POS)) /**< RST1_AES Mask */

#define MXC_F_GCR_RST1_AC_POS                          14 /**< RST1_AC Position */
#define MXC_F_GCR_RST1_AC                              ((uint32_t)(0x1UL << MXC_F_GCR_RST1_AC_POS)) /**< RST1_AC Mask */

#define MXC_F_GCR_RST1_I2S_POS                         23 /**< RST1_I2S Position */
#define MXC_F_GCR_RST1_I2S                             ((uint32_t)(0x1UL << MXC_F_GCR_RST1_I2S_POS)) /**< RST1_I2S Mask */

/**@} end of group GCR_RST1_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_PCLKDIS1 GCR_PCLKDIS1
 * @brief    Peripheral Clock Disable.
 * @{
 */
#define MXC_F_GCR_PCLKDIS1_TRNG_POS                    2 /**< PCLKDIS1_TRNG Position */
#define MXC_F_GCR_PCLKDIS1_TRNG                        ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS1_TRNG_POS)) /**< PCLKDIS1_TRNG Mask */

#define MXC_F_GCR_PCLKDIS1_WDT_POS                     4 /**< PCLKDIS1_WDT Position */
#define MXC_F_GCR_PCLKDIS1_WDT                         ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS1_WDT_POS)) /**< PCLKDIS1_WDT Mask */

#define MXC_F_GCR_PCLKDIS1_CAN_POS                     11 /**< PCLKDIS1_CAN Position */
#define MXC_F_GCR_PCLKDIS1_CAN                         ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS1_CAN_POS)) /**< PCLKDIS1_CAN Mask */

#define MXC_F_GCR_PCLKDIS1_AES_POS                     15 /**< PCLKDIS1_AES Position */
#define MXC_F_GCR_PCLKDIS1_AES                         ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS1_AES_POS)) /**< PCLKDIS1_AES Mask */

#define MXC_F_GCR_PCLKDIS1_AES_KEY_POS                 16 /**< PCLKDIS1_AES_KEY Position */
#define MXC_F_GCR_PCLKDIS1_AES_KEY                     ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS1_AES_KEY_POS)) /**< PCLKDIS1_AES_KEY Mask */

#define MXC_F_GCR_PCLKDIS1_I2S_POS                     23 /**< PCLKDIS1_I2S Position */
#define MXC_F_GCR_PCLKDIS1_I2S                         ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS1_I2S_POS)) /**< PCLKDIS1_I2S Mask */

/**@} end of group GCR_PCLKDIS1_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_EVENTEN GCR_EVENTEN
 * @brief    Event Enable Register.
 * @{
 */
#define MXC_F_GCR_EVENTEN_DMA_POS                      0 /**< EVENTEN_DMA Position */
#define MXC_F_GCR_EVENTEN_DMA                          ((uint32_t)(0x1UL << MXC_F_GCR_EVENTEN_DMA_POS)) /**< EVENTEN_DMA Mask */

#define MXC_F_GCR_EVENTEN_RX_POS                       1 /**< EVENTEN_RX Position */
#define MXC_F_GCR_EVENTEN_RX                           ((uint32_t)(0x1UL << MXC_F_GCR_EVENTEN_RX_POS)) /**< EVENTEN_RX Mask */

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
#define MXC_F_GCR_ECCERR_RAM0_1_POS                    0 /**< ECCERR_RAM0_1 Position */
#define MXC_F_GCR_ECCERR_RAM0_1                        ((uint32_t)(0x1UL << MXC_F_GCR_ECCERR_RAM0_1_POS)) /**< ECCERR_RAM0_1 Mask */

#define MXC_F_GCR_ECCERR_RAM2_POS                      1 /**< ECCERR_RAM2 Position */
#define MXC_F_GCR_ECCERR_RAM2                          ((uint32_t)(0x1UL << MXC_F_GCR_ECCERR_RAM2_POS)) /**< ECCERR_RAM2 Mask */

#define MXC_F_GCR_ECCERR_RAM3_POS                      2 /**< ECCERR_RAM3 Position */
#define MXC_F_GCR_ECCERR_RAM3                          ((uint32_t)(0x1UL << MXC_F_GCR_ECCERR_RAM3_POS)) /**< ECCERR_RAM3 Mask */

#define MXC_F_GCR_ECCERR_ICC0_POS                      3 /**< ECCERR_ICC0 Position */
#define MXC_F_GCR_ECCERR_ICC0                          ((uint32_t)(0x1UL << MXC_F_GCR_ECCERR_ICC0_POS)) /**< ECCERR_ICC0 Mask */

#define MXC_F_GCR_ECCERR_FLASH0_POS                    4 /**< ECCERR_FLASH0 Position */
#define MXC_F_GCR_ECCERR_FLASH0                        ((uint32_t)(0x1UL << MXC_F_GCR_ECCERR_FLASH0_POS)) /**< ECCERR_FLASH0 Mask */

#define MXC_F_GCR_ECCERR_FLASH1_POS                    5 /**< ECCERR_FLASH1 Position */
#define MXC_F_GCR_ECCERR_FLASH1                        ((uint32_t)(0x1UL << MXC_F_GCR_ECCERR_FLASH1_POS)) /**< ECCERR_FLASH1 Mask */

/**@} end of group GCR_ECCERR_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_ECCCED GCR_ECCCED
 * @brief    ECC Not Double Error Detect Register
 * @{
 */
#define MXC_F_GCR_ECCCED_RAM0_1_POS                    0 /**< ECCCED_RAM0_1 Position */
#define MXC_F_GCR_ECCCED_RAM0_1                        ((uint32_t)(0x1UL << MXC_F_GCR_ECCCED_RAM0_1_POS)) /**< ECCCED_RAM0_1 Mask */

#define MXC_F_GCR_ECCCED_RAM2_POS                      1 /**< ECCCED_RAM2 Position */
#define MXC_F_GCR_ECCCED_RAM2                          ((uint32_t)(0x1UL << MXC_F_GCR_ECCCED_RAM2_POS)) /**< ECCCED_RAM2 Mask */

#define MXC_F_GCR_ECCCED_RAM3_POS                      2 /**< ECCCED_RAM3 Position */
#define MXC_F_GCR_ECCCED_RAM3                          ((uint32_t)(0x1UL << MXC_F_GCR_ECCCED_RAM3_POS)) /**< ECCCED_RAM3 Mask */

#define MXC_F_GCR_ECCCED_ICC0_POS                      3 /**< ECCCED_ICC0 Position */
#define MXC_F_GCR_ECCCED_ICC0                          ((uint32_t)(0x1UL << MXC_F_GCR_ECCCED_ICC0_POS)) /**< ECCCED_ICC0 Mask */

#define MXC_F_GCR_ECCCED_FLASH0_POS                    4 /**< ECCCED_FLASH0 Position */
#define MXC_F_GCR_ECCCED_FLASH0                        ((uint32_t)(0x1UL << MXC_F_GCR_ECCCED_FLASH0_POS)) /**< ECCCED_FLASH0 Mask */

#define MXC_F_GCR_ECCCED_FLASH1_POS                    5 /**< ECCCED_FLASH1 Position */
#define MXC_F_GCR_ECCCED_FLASH1                        ((uint32_t)(0x1UL << MXC_F_GCR_ECCCED_FLASH1_POS)) /**< ECCCED_FLASH1 Mask */

/**@} end of group GCR_ECCCED_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_ECCIE GCR_ECCIE
 * @brief    ECC IRQ Enable Register
 * @{
 */
#define MXC_F_GCR_ECCIE_RAM0_1_POS                     0 /**< ECCIE_RAM0_1 Position */
#define MXC_F_GCR_ECCIE_RAM0_1                         ((uint32_t)(0x1UL << MXC_F_GCR_ECCIE_RAM0_1_POS)) /**< ECCIE_RAM0_1 Mask */

#define MXC_F_GCR_ECCIE_RAM2_POS                       1 /**< ECCIE_RAM2 Position */
#define MXC_F_GCR_ECCIE_RAM2                           ((uint32_t)(0x1UL << MXC_F_GCR_ECCIE_RAM2_POS)) /**< ECCIE_RAM2 Mask */

#define MXC_F_GCR_ECCIE_RAM3_POS                       2 /**< ECCIE_RAM3 Position */
#define MXC_F_GCR_ECCIE_RAM3                           ((uint32_t)(0x1UL << MXC_F_GCR_ECCIE_RAM3_POS)) /**< ECCIE_RAM3 Mask */

#define MXC_F_GCR_ECCIE_ICC0_POS                       3 /**< ECCIE_ICC0 Position */
#define MXC_F_GCR_ECCIE_ICC0                           ((uint32_t)(0x1UL << MXC_F_GCR_ECCIE_ICC0_POS)) /**< ECCIE_ICC0 Mask */

#define MXC_F_GCR_ECCIE_FLASH0_POS                     4 /**< ECCIE_FLASH0 Position */
#define MXC_F_GCR_ECCIE_FLASH0                         ((uint32_t)(0x1UL << MXC_F_GCR_ECCIE_FLASH0_POS)) /**< ECCIE_FLASH0 Mask */

#define MXC_F_GCR_ECCIE_FLASH1_POS                     5 /**< ECCIE_FLASH1 Position */
#define MXC_F_GCR_ECCIE_FLASH1                         ((uint32_t)(0x1UL << MXC_F_GCR_ECCIE_FLASH1_POS)) /**< ECCIE_FLASH1 Mask */

/**@} end of group GCR_ECCIE_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_ECCADDR GCR_ECCADDR
 * @brief    ECC Error Address Register
 * @{
 */
#define MXC_F_GCR_ECCADDR_ERRADDR_POS                  0 /**< ECCADDR_ERRADDR Position */
#define MXC_F_GCR_ECCADDR_ERRADDR                      ((uint32_t)(0xFFFFFFFFUL << MXC_F_GCR_ECCADDR_ERRADDR_POS)) /**< ECCADDR_ERRADDR Mask */

/**@} end of group GCR_ECCADDR_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32662_INCLUDE_GCR_REGS_H_
