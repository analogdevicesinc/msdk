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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32572_INCLUDE_GCR_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32572_INCLUDE_GCR_REGS_H_

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
    __R  uint32_t rsv_0x30;
    __IO uint32_t scclkctrl;            /**< <tt>\b 0x34:</tt> GCR SCCLKCTRL Register */
    __R  uint32_t rsv_0x38_0x3f[2];
    __IO uint32_t sysst;                /**< <tt>\b 0x40:</tt> GCR SYSST Register */
    __IO uint32_t rst1;                 /**< <tt>\b 0x44:</tt> GCR RST1 Register */
    __IO uint32_t pclkdis1;             /**< <tt>\b 0x48:</tt> GCR PCLKDIS1 Register */
    __IO uint32_t eventen;              /**< <tt>\b 0x4C:</tt> GCR EVENTEN Register */
    __I  uint32_t revision;             /**< <tt>\b 0x50:</tt> GCR REVISION Register */
    __IO uint32_t sysie;                /**< <tt>\b 0x54:</tt> GCR SYSIE Register */
    __IO uint32_t ipocnt;               /**< <tt>\b 0x58:</tt> GCR IPOCNT Register */
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
#define MXC_R_GCR_SCCLKCTRL                ((uint32_t)0x00000034UL) /**< Offset from GCR Base Address: <tt> 0x0034</tt> */
#define MXC_R_GCR_SYSST                    ((uint32_t)0x00000040UL) /**< Offset from GCR Base Address: <tt> 0x0040</tt> */
#define MXC_R_GCR_RST1                     ((uint32_t)0x00000044UL) /**< Offset from GCR Base Address: <tt> 0x0044</tt> */
#define MXC_R_GCR_PCLKDIS1                 ((uint32_t)0x00000048UL) /**< Offset from GCR Base Address: <tt> 0x0048</tt> */
#define MXC_R_GCR_EVENTEN                  ((uint32_t)0x0000004CUL) /**< Offset from GCR Base Address: <tt> 0x004C</tt> */
#define MXC_R_GCR_REVISION                 ((uint32_t)0x00000050UL) /**< Offset from GCR Base Address: <tt> 0x0050</tt> */
#define MXC_R_GCR_SYSIE                    ((uint32_t)0x00000054UL) /**< Offset from GCR Base Address: <tt> 0x0054</tt> */
#define MXC_R_GCR_IPOCNT                   ((uint32_t)0x00000058UL) /**< Offset from GCR Base Address: <tt> 0x0058</tt> */
/**@} end of group gcr_registers */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_SYSCTRL GCR_SYSCTRL
 * @brief    System Control.
 * @{
 */
#define MXC_F_GCR_SYSCTRL_BSTAPEN_POS                  0 /**< SYSCTRL_BSTAPEN Position */
#define MXC_F_GCR_SYSCTRL_BSTAPEN                      ((uint32_t)(0x1UL << MXC_F_GCR_SYSCTRL_BSTAPEN_POS)) /**< SYSCTRL_BSTAPEN Mask */

#define MXC_F_GCR_SYSCTRL_SBUSARB_POS                  1 /**< SYSCTRL_SBUSARB Position */
#define MXC_F_GCR_SYSCTRL_SBUSARB                      ((uint32_t)(0x3UL << MXC_F_GCR_SYSCTRL_SBUSARB_POS)) /**< SYSCTRL_SBUSARB Mask */
#define MXC_V_GCR_SYSCTRL_SBUSARB_FIX                  ((uint32_t)0x0UL) /**< SYSCTRL_SBUSARB_FIX Value */
#define MXC_S_GCR_SYSCTRL_SBUSARB_FIX                  (MXC_V_GCR_SYSCTRL_SBUSARB_FIX << MXC_F_GCR_SYSCTRL_SBUSARB_POS) /**< SYSCTRL_SBUSARB_FIX Setting */
#define MXC_V_GCR_SYSCTRL_SBUSARB_ROUND                ((uint32_t)0x1UL) /**< SYSCTRL_SBUSARB_ROUND Value */
#define MXC_S_GCR_SYSCTRL_SBUSARB_ROUND                (MXC_V_GCR_SYSCTRL_SBUSARB_ROUND << MXC_F_GCR_SYSCTRL_SBUSARB_POS) /**< SYSCTRL_SBUSARB_ROUND Setting */

#define MXC_F_GCR_SYSCTRL_FPU_DIS_POS                  5 /**< SYSCTRL_FPU_DIS Position */
#define MXC_F_GCR_SYSCTRL_FPU_DIS                      ((uint32_t)(0x1UL << MXC_F_GCR_SYSCTRL_FPU_DIS_POS)) /**< SYSCTRL_FPU_DIS Mask */

#define MXC_F_GCR_SYSCTRL_SFCC_FLUSH_POS               6 /**< SYSCTRL_SFCC_FLUSH Position */
#define MXC_F_GCR_SYSCTRL_SFCC_FLUSH                   ((uint32_t)(0x1UL << MXC_F_GCR_SYSCTRL_SFCC_FLUSH_POS)) /**< SYSCTRL_SFCC_FLUSH Mask */

#define MXC_F_GCR_SYSCTRL_CHKRES1_POS                  11 /**< SYSCTRL_CHKRES1 Position */
#define MXC_F_GCR_SYSCTRL_CHKRES1                      ((uint32_t)(0x1UL << MXC_F_GCR_SYSCTRL_CHKRES1_POS)) /**< SYSCTRL_CHKRES1 Mask */

#define MXC_F_GCR_SYSCTRL_CCHK1_POS                    12 /**< SYSCTRL_CCHK1 Position */
#define MXC_F_GCR_SYSCTRL_CCHK1                        ((uint32_t)(0x1UL << MXC_F_GCR_SYSCTRL_CCHK1_POS)) /**< SYSCTRL_CCHK1 Mask */

#define MXC_F_GCR_SYSCTRL_CCHK0_POS                    13 /**< SYSCTRL_CCHK0 Position */
#define MXC_F_GCR_SYSCTRL_CCHK0                        ((uint32_t)(0x1UL << MXC_F_GCR_SYSCTRL_CCHK0_POS)) /**< SYSCTRL_CCHK0 Mask */

#define MXC_F_GCR_SYSCTRL_SWD_DIS_POS                  14 /**< SYSCTRL_SWD_DIS Position */
#define MXC_F_GCR_SYSCTRL_SWD_DIS                      ((uint32_t)(0x1UL << MXC_F_GCR_SYSCTRL_SWD_DIS_POS)) /**< SYSCTRL_SWD_DIS Mask */

#define MXC_F_GCR_SYSCTRL_CHKRES0_POS                  15 /**< SYSCTRL_CHKRES0 Position */
#define MXC_F_GCR_SYSCTRL_CHKRES0                      ((uint32_t)(0x1UL << MXC_F_GCR_SYSCTRL_CHKRES0_POS)) /**< SYSCTRL_CHKRES0 Mask */

/**@} end of group GCR_SYSCTRL_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_RST0 GCR_RST0
 * @brief    Reset.
 * @{
 */
#define MXC_F_GCR_RST0_DMA_POS                         0 /**< RST0_DMA Position */
#define MXC_F_GCR_RST0_DMA                             ((uint32_t)(0x1UL << MXC_F_GCR_RST0_DMA_POS)) /**< RST0_DMA Mask */

#define MXC_F_GCR_RST0_WDT0_POS                        1 /**< RST0_WDT0 Position */
#define MXC_F_GCR_RST0_WDT0                            ((uint32_t)(0x1UL << MXC_F_GCR_RST0_WDT0_POS)) /**< RST0_WDT0 Mask */

#define MXC_F_GCR_RST0_GPIO0_POS                       2 /**< RST0_GPIO0 Position */
#define MXC_F_GCR_RST0_GPIO0                           ((uint32_t)(0x1UL << MXC_F_GCR_RST0_GPIO0_POS)) /**< RST0_GPIO0 Mask */

#define MXC_F_GCR_RST0_GPIO1_POS                       3 /**< RST0_GPIO1 Position */
#define MXC_F_GCR_RST0_GPIO1                           ((uint32_t)(0x1UL << MXC_F_GCR_RST0_GPIO1_POS)) /**< RST0_GPIO1 Mask */

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

#define MXC_F_GCR_RST0_CRYPTO_POS                      18 /**< RST0_CRYPTO Position */
#define MXC_F_GCR_RST0_CRYPTO                          ((uint32_t)(0x1UL << MXC_F_GCR_RST0_CRYPTO_POS)) /**< RST0_CRYPTO Mask */

#define MXC_F_GCR_RST0_USB_POS                         23 /**< RST0_USB Position */
#define MXC_F_GCR_RST0_USB                             ((uint32_t)(0x1UL << MXC_F_GCR_RST0_USB_POS)) /**< RST0_USB Mask */

#define MXC_F_GCR_RST0_TRNG_POS                        24 /**< RST0_TRNG Position */
#define MXC_F_GCR_RST0_TRNG                            ((uint32_t)(0x1UL << MXC_F_GCR_RST0_TRNG_POS)) /**< RST0_TRNG Mask */

#define MXC_F_GCR_RST0_ADC_POS                         26 /**< RST0_ADC Position */
#define MXC_F_GCR_RST0_ADC                             ((uint32_t)(0x1UL << MXC_F_GCR_RST0_ADC_POS)) /**< RST0_ADC Mask */

#define MXC_F_GCR_RST0_UART2_POS                       28 /**< RST0_UART2 Position */
#define MXC_F_GCR_RST0_UART2                           ((uint32_t)(0x1UL << MXC_F_GCR_RST0_UART2_POS)) /**< RST0_UART2 Mask */

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
#define MXC_F_GCR_CLKCTRL_PCLK_DIV_POS                 3 /**< CLKCTRL_PCLK_DIV Position */
#define MXC_F_GCR_CLKCTRL_PCLK_DIV                     ((uint32_t)(0x7UL << MXC_F_GCR_CLKCTRL_PCLK_DIV_POS)) /**< CLKCTRL_PCLK_DIV Mask */
#define MXC_V_GCR_CLKCTRL_PCLK_DIV_DIV1                ((uint32_t)0x0UL) /**< CLKCTRL_PCLK_DIV_DIV1 Value */
#define MXC_S_GCR_CLKCTRL_PCLK_DIV_DIV1                (MXC_V_GCR_CLKCTRL_PCLK_DIV_DIV1 << MXC_F_GCR_CLKCTRL_PCLK_DIV_POS) /**< CLKCTRL_PCLK_DIV_DIV1 Setting */
#define MXC_V_GCR_CLKCTRL_PCLK_DIV_DIV2                ((uint32_t)0x1UL) /**< CLKCTRL_PCLK_DIV_DIV2 Value */
#define MXC_S_GCR_CLKCTRL_PCLK_DIV_DIV2                (MXC_V_GCR_CLKCTRL_PCLK_DIV_DIV2 << MXC_F_GCR_CLKCTRL_PCLK_DIV_POS) /**< CLKCTRL_PCLK_DIV_DIV2 Setting */
#define MXC_V_GCR_CLKCTRL_PCLK_DIV_DIV4                ((uint32_t)0x2UL) /**< CLKCTRL_PCLK_DIV_DIV4 Value */
#define MXC_S_GCR_CLKCTRL_PCLK_DIV_DIV4                (MXC_V_GCR_CLKCTRL_PCLK_DIV_DIV4 << MXC_F_GCR_CLKCTRL_PCLK_DIV_POS) /**< CLKCTRL_PCLK_DIV_DIV4 Setting */
#define MXC_V_GCR_CLKCTRL_PCLK_DIV_DIV8                ((uint32_t)0x3UL) /**< CLKCTRL_PCLK_DIV_DIV8 Value */
#define MXC_S_GCR_CLKCTRL_PCLK_DIV_DIV8                (MXC_V_GCR_CLKCTRL_PCLK_DIV_DIV8 << MXC_F_GCR_CLKCTRL_PCLK_DIV_POS) /**< CLKCTRL_PCLK_DIV_DIV8 Setting */

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
#define MXC_V_GCR_CLKCTRL_SYSCLK_SEL_ISO               ((uint32_t)0x0UL) /**< CLKCTRL_SYSCLK_SEL_ISO Value */
#define MXC_S_GCR_CLKCTRL_SYSCLK_SEL_ISO               (MXC_V_GCR_CLKCTRL_SYSCLK_SEL_ISO << MXC_F_GCR_CLKCTRL_SYSCLK_SEL_POS) /**< CLKCTRL_SYSCLK_SEL_ISO Setting */
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

#define MXC_F_GCR_CLKCTRL_CRYPTOCLK_DIV_POS            12 /**< CLKCTRL_CRYPTOCLK_DIV Position */
#define MXC_F_GCR_CLKCTRL_CRYPTOCLK_DIV                ((uint32_t)(0x1UL << MXC_F_GCR_CLKCTRL_CRYPTOCLK_DIV_POS)) /**< CLKCTRL_CRYPTOCLK_DIV Mask */

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

#define MXC_F_GCR_CLKCTRL_ISO_EN_POS                   18 /**< CLKCTRL_ISO_EN Position */
#define MXC_F_GCR_CLKCTRL_ISO_EN                       ((uint32_t)(0x1UL << MXC_F_GCR_CLKCTRL_ISO_EN_POS)) /**< CLKCTRL_ISO_EN Mask */

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

#define MXC_F_GCR_CLKCTRL_ISO_RDY_POS                  26 /**< CLKCTRL_ISO_RDY Position */
#define MXC_F_GCR_CLKCTRL_ISO_RDY                      ((uint32_t)(0x1UL << MXC_F_GCR_CLKCTRL_ISO_RDY_POS)) /**< CLKCTRL_ISO_RDY Mask */

#define MXC_F_GCR_CLKCTRL_IPO_RDY_POS                  27 /**< CLKCTRL_IPO_RDY Position */
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
#define MXC_F_GCR_PM_MODE                              ((uint32_t)(0x7UL << MXC_F_GCR_PM_MODE_POS)) /**< PM_MODE Mask */
#define MXC_V_GCR_PM_MODE_ACTIVE                       ((uint32_t)0x0UL) /**< PM_MODE_ACTIVE Value */
#define MXC_S_GCR_PM_MODE_ACTIVE                       (MXC_V_GCR_PM_MODE_ACTIVE << MXC_F_GCR_PM_MODE_POS) /**< PM_MODE_ACTIVE Setting */
#define MXC_V_GCR_PM_MODE_SLEEP                        ((uint32_t)0x1UL) /**< PM_MODE_SLEEP Value */
#define MXC_S_GCR_PM_MODE_SLEEP                        (MXC_V_GCR_PM_MODE_SLEEP << MXC_F_GCR_PM_MODE_POS) /**< PM_MODE_SLEEP Setting */
#define MXC_V_GCR_PM_MODE_DEEPSLEEP                    ((uint32_t)0x2UL) /**< PM_MODE_DEEPSLEEP Value */
#define MXC_S_GCR_PM_MODE_DEEPSLEEP                    (MXC_V_GCR_PM_MODE_DEEPSLEEP << MXC_F_GCR_PM_MODE_POS) /**< PM_MODE_DEEPSLEEP Setting */
#define MXC_V_GCR_PM_MODE_SHUTDOWN                     ((uint32_t)0x3UL) /**< PM_MODE_SHUTDOWN Value */
#define MXC_S_GCR_PM_MODE_SHUTDOWN                     (MXC_V_GCR_PM_MODE_SHUTDOWN << MXC_F_GCR_PM_MODE_POS) /**< PM_MODE_SHUTDOWN Setting */
#define MXC_V_GCR_PM_MODE_BACKUP                       ((uint32_t)0x4UL) /**< PM_MODE_BACKUP Value */
#define MXC_S_GCR_PM_MODE_BACKUP                       (MXC_V_GCR_PM_MODE_BACKUP << MXC_F_GCR_PM_MODE_POS) /**< PM_MODE_BACKUP Setting */

#define MXC_F_GCR_PM_GPIO_WE_POS                       4 /**< PM_GPIO_WE Position */
#define MXC_F_GCR_PM_GPIO_WE                           ((uint32_t)(0x1UL << MXC_F_GCR_PM_GPIO_WE_POS)) /**< PM_GPIO_WE Mask */

#define MXC_F_GCR_PM_RTC_WE_POS                        5 /**< PM_RTC_WE Position */
#define MXC_F_GCR_PM_RTC_WE                            ((uint32_t)(0x1UL << MXC_F_GCR_PM_RTC_WE_POS)) /**< PM_RTC_WE Mask */

#define MXC_F_GCR_PM_USB_WE_POS                        6 /**< PM_USB_WE Position */
#define MXC_F_GCR_PM_USB_WE                            ((uint32_t)(0x1UL << MXC_F_GCR_PM_USB_WE_POS)) /**< PM_USB_WE Mask */

#define MXC_F_GCR_PM_ERFO_PD_POS                       12 /**< PM_ERFO_PD Position */
#define MXC_F_GCR_PM_ERFO_PD                           ((uint32_t)(0x1UL << MXC_F_GCR_PM_ERFO_PD_POS)) /**< PM_ERFO_PD Mask */

#define MXC_F_GCR_PM_ISO_PD_POS                        15 /**< PM_ISO_PD Position */
#define MXC_F_GCR_PM_ISO_PD                            ((uint32_t)(0x1UL << MXC_F_GCR_PM_ISO_PD_POS)) /**< PM_ISO_PD Mask */

#define MXC_F_GCR_PM_IPO_PD_POS                        16 /**< PM_IPO_PD Position */
#define MXC_F_GCR_PM_IPO_PD                            ((uint32_t)(0x1UL << MXC_F_GCR_PM_IPO_PD_POS)) /**< PM_IPO_PD Mask */

#define MXC_F_GCR_PM_IBRO_PD_POS                       17 /**< PM_IBRO_PD Position */
#define MXC_F_GCR_PM_IBRO_PD                           ((uint32_t)(0x1UL << MXC_F_GCR_PM_IBRO_PD_POS)) /**< PM_IBRO_PD Mask */

#define MXC_F_GCR_PM_ERFO_BP_POS                       20 /**< PM_ERFO_BP Position */
#define MXC_F_GCR_PM_ERFO_BP                           ((uint32_t)(0x1UL << MXC_F_GCR_PM_ERFO_BP_POS)) /**< PM_ERFO_BP Mask */

/**@} end of group GCR_PM_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_PCLKDIV GCR_PCLKDIV
 * @brief    Peripheral Clock Divider.
 * @{
 */
#define MXC_F_GCR_PCLKDIV_SKBDFRQ_POS                  0 /**< PCLKDIV_SKBDFRQ Position */
#define MXC_F_GCR_PCLKDIV_SKBDFRQ                      ((uint32_t)(0x7UL << MXC_F_GCR_PCLKDIV_SKBDFRQ_POS)) /**< PCLKDIV_SKBDFRQ Mask */

#define MXC_F_GCR_PCLKDIV_ADCFRQ_POS                   10 /**< PCLKDIV_ADCFRQ Position */
#define MXC_F_GCR_PCLKDIV_ADCFRQ                       ((uint32_t)(0xFUL << MXC_F_GCR_PCLKDIV_ADCFRQ_POS)) /**< PCLKDIV_ADCFRQ Mask */

#define MXC_F_GCR_PCLKDIV_AONCLKDIV_POS                14 /**< PCLKDIV_AONCLKDIV Position */
#define MXC_F_GCR_PCLKDIV_AONCLKDIV                    ((uint32_t)(0x3UL << MXC_F_GCR_PCLKDIV_AONCLKDIV_POS)) /**< PCLKDIV_AONCLKDIV Mask */
#define MXC_V_GCR_PCLKDIV_AONCLKDIV_DIV4               ((uint32_t)0x0UL) /**< PCLKDIV_AONCLKDIV_DIV4 Value */
#define MXC_S_GCR_PCLKDIV_AONCLKDIV_DIV4               (MXC_V_GCR_PCLKDIV_AONCLKDIV_DIV4 << MXC_F_GCR_PCLKDIV_AONCLKDIV_POS) /**< PCLKDIV_AONCLKDIV_DIV4 Setting */
#define MXC_V_GCR_PCLKDIV_AONCLKDIV_DIV8               ((uint32_t)0x1UL) /**< PCLKDIV_AONCLKDIV_DIV8 Value */
#define MXC_S_GCR_PCLKDIV_AONCLKDIV_DIV8               (MXC_V_GCR_PCLKDIV_AONCLKDIV_DIV8 << MXC_F_GCR_PCLKDIV_AONCLKDIV_POS) /**< PCLKDIV_AONCLKDIV_DIV8 Setting */
#define MXC_V_GCR_PCLKDIV_AONCLKDIV_DIV16              ((uint32_t)0x2UL) /**< PCLKDIV_AONCLKDIV_DIV16 Value */
#define MXC_S_GCR_PCLKDIV_AONCLKDIV_DIV16              (MXC_V_GCR_PCLKDIV_AONCLKDIV_DIV16 << MXC_F_GCR_PCLKDIV_AONCLKDIV_POS) /**< PCLKDIV_AONCLKDIV_DIV16 Setting */
#define MXC_V_GCR_PCLKDIV_AONCLKDIV_DIV32              ((uint32_t)0x3UL) /**< PCLKDIV_AONCLKDIV_DIV32 Value */
#define MXC_S_GCR_PCLKDIV_AONCLKDIV_DIV32              (MXC_V_GCR_PCLKDIV_AONCLKDIV_DIV32 << MXC_F_GCR_PCLKDIV_AONCLKDIV_POS) /**< PCLKDIV_AONCLKDIV_DIV32 Setting */

/**@} end of group GCR_PCLKDIV_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_PCLKDIS0 GCR_PCLKDIS0
 * @brief    Peripheral Clock Disable.
 * @{
 */
#define MXC_F_GCR_PCLKDIS0_GPIO0_POS                   0 /**< PCLKDIS0_GPIO0 Position */
#define MXC_F_GCR_PCLKDIS0_GPIO0                       ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS0_GPIO0_POS)) /**< PCLKDIS0_GPIO0 Mask */

#define MXC_F_GCR_PCLKDIS0_GPIO1_POS                   1 /**< PCLKDIS0_GPIO1 Position */
#define MXC_F_GCR_PCLKDIS0_GPIO1                       ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS0_GPIO1_POS)) /**< PCLKDIS0_GPIO1 Mask */

#define MXC_F_GCR_PCLKDIS0_USB_POS                     3 /**< PCLKDIS0_USB Position */
#define MXC_F_GCR_PCLKDIS0_USB                         ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS0_USB_POS)) /**< PCLKDIS0_USB Mask */

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

#define MXC_F_GCR_PCLKDIS0_CRYPTO_POS                  14 /**< PCLKDIS0_CRYPTO Position */
#define MXC_F_GCR_PCLKDIS0_CRYPTO                      ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS0_CRYPTO_POS)) /**< PCLKDIS0_CRYPTO Mask */

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

#define MXC_F_GCR_PCLKDIS0_SKBD_POS                    22 /**< PCLKDIS0_SKBD Position */
#define MXC_F_GCR_PCLKDIS0_SKBD                        ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS0_SKBD_POS)) /**< PCLKDIS0_SKBD Mask */

#define MXC_F_GCR_PCLKDIS0_ADC_POS                     23 /**< PCLKDIS0_ADC Position */
#define MXC_F_GCR_PCLKDIS0_ADC                         ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS0_ADC_POS)) /**< PCLKDIS0_ADC Mask */

#define MXC_F_GCR_PCLKDIS0_HTMR0_POS                   26 /**< PCLKDIS0_HTMR0 Position */
#define MXC_F_GCR_PCLKDIS0_HTMR0                       ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS0_HTMR0_POS)) /**< PCLKDIS0_HTMR0 Mask */

#define MXC_F_GCR_PCLKDIS0_HTMR1_POS                   27 /**< PCLKDIS0_HTMR1 Position */
#define MXC_F_GCR_PCLKDIS0_HTMR1                       ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS0_HTMR1_POS)) /**< PCLKDIS0_HTMR1 Mask */

#define MXC_F_GCR_PCLKDIS0_I2C1_POS                    28 /**< PCLKDIS0_I2C1 Position */
#define MXC_F_GCR_PCLKDIS0_I2C1                        ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS0_I2C1_POS)) /**< PCLKDIS0_I2C1 Mask */

#define MXC_F_GCR_PCLKDIS0_PT_POS                      29 /**< PCLKDIS0_PT Position */
#define MXC_F_GCR_PCLKDIS0_PT                          ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS0_PT_POS)) /**< PCLKDIS0_PT Mask */

#define MXC_F_GCR_PCLKDIS0_SPIXIP_POS                  30 /**< PCLKDIS0_SPIXIP Position */
#define MXC_F_GCR_PCLKDIS0_SPIXIP                      ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS0_SPIXIP_POS)) /**< PCLKDIS0_SPIXIP Mask */

#define MXC_F_GCR_PCLKDIS0_SPIXIPC_POS                 31 /**< PCLKDIS0_SPIXIPC Position */
#define MXC_F_GCR_PCLKDIS0_SPIXIPC                     ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS0_SPIXIPC_POS)) /**< PCLKDIS0_SPIXIPC Mask */

/**@} end of group GCR_PCLKDIS0_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_MEMCTRL GCR_MEMCTRL
 * @brief    Memory Clock Control Register.
 * @{
 */
#define MXC_F_GCR_MEMCTRL_FWS_POS                      0 /**< MEMCTRL_FWS Position */
#define MXC_F_GCR_MEMCTRL_FWS                          ((uint32_t)(0x7UL << MXC_F_GCR_MEMCTRL_FWS_POS)) /**< MEMCTRL_FWS Mask */

#define MXC_F_GCR_MEMCTRL_RAM4_WS_POS                  4 /**< MEMCTRL_RAM4_WS Position */
#define MXC_F_GCR_MEMCTRL_RAM4_WS                      ((uint32_t)(0x1UL << MXC_F_GCR_MEMCTRL_RAM4_WS_POS)) /**< MEMCTRL_RAM4_WS Mask */

#define MXC_F_GCR_MEMCTRL_RAM5_WS_POS                  5 /**< MEMCTRL_RAM5_WS Position */
#define MXC_F_GCR_MEMCTRL_RAM5_WS                      ((uint32_t)(0x1UL << MXC_F_GCR_MEMCTRL_RAM5_WS_POS)) /**< MEMCTRL_RAM5_WS Mask */

#define MXC_F_GCR_MEMCTRL_RAM6_WS_POS                  6 /**< MEMCTRL_RAM6_WS Position */
#define MXC_F_GCR_MEMCTRL_RAM6_WS                      ((uint32_t)(0x1UL << MXC_F_GCR_MEMCTRL_RAM6_WS_POS)) /**< MEMCTRL_RAM6_WS Mask */

#define MXC_F_GCR_MEMCTRL_ROM1_WS_POS                  7 /**< MEMCTRL_ROM1_WS Position */
#define MXC_F_GCR_MEMCTRL_ROM1_WS                      ((uint32_t)(0x1UL << MXC_F_GCR_MEMCTRL_ROM1_WS_POS)) /**< MEMCTRL_ROM1_WS Mask */

#define MXC_F_GCR_MEMCTRL_RAM0LS_EN_POS                16 /**< MEMCTRL_RAM0LS_EN Position */
#define MXC_F_GCR_MEMCTRL_RAM0LS_EN                    ((uint32_t)(0x1UL << MXC_F_GCR_MEMCTRL_RAM0LS_EN_POS)) /**< MEMCTRL_RAM0LS_EN Mask */

#define MXC_F_GCR_MEMCTRL_RAM1LS_EN_POS                17 /**< MEMCTRL_RAM1LS_EN Position */
#define MXC_F_GCR_MEMCTRL_RAM1LS_EN                    ((uint32_t)(0x1UL << MXC_F_GCR_MEMCTRL_RAM1LS_EN_POS)) /**< MEMCTRL_RAM1LS_EN Mask */

#define MXC_F_GCR_MEMCTRL_RAM2LS_EN_POS                18 /**< MEMCTRL_RAM2LS_EN Position */
#define MXC_F_GCR_MEMCTRL_RAM2LS_EN                    ((uint32_t)(0x1UL << MXC_F_GCR_MEMCTRL_RAM2LS_EN_POS)) /**< MEMCTRL_RAM2LS_EN Mask */

#define MXC_F_GCR_MEMCTRL_RAM3LS_EN_POS                19 /**< MEMCTRL_RAM3LS_EN Position */
#define MXC_F_GCR_MEMCTRL_RAM3LS_EN                    ((uint32_t)(0x1UL << MXC_F_GCR_MEMCTRL_RAM3LS_EN_POS)) /**< MEMCTRL_RAM3LS_EN Mask */

#define MXC_F_GCR_MEMCTRL_RAM4LS_EN_POS                20 /**< MEMCTRL_RAM4LS_EN Position */
#define MXC_F_GCR_MEMCTRL_RAM4LS_EN                    ((uint32_t)(0x1UL << MXC_F_GCR_MEMCTRL_RAM4LS_EN_POS)) /**< MEMCTRL_RAM4LS_EN Mask */

#define MXC_F_GCR_MEMCTRL_RAM5LS_EN_POS                21 /**< MEMCTRL_RAM5LS_EN Position */
#define MXC_F_GCR_MEMCTRL_RAM5LS_EN                    ((uint32_t)(0x1UL << MXC_F_GCR_MEMCTRL_RAM5LS_EN_POS)) /**< MEMCTRL_RAM5LS_EN Mask */

#define MXC_F_GCR_MEMCTRL_RAM6LS_EN_POS                22 /**< MEMCTRL_RAM6LS_EN Position */
#define MXC_F_GCR_MEMCTRL_RAM6LS_EN                    ((uint32_t)(0x1UL << MXC_F_GCR_MEMCTRL_RAM6LS_EN_POS)) /**< MEMCTRL_RAM6LS_EN Mask */

#define MXC_F_GCR_MEMCTRL_ICCXIPLS_EN_POS              25 /**< MEMCTRL_ICCXIPLS_EN Position */
#define MXC_F_GCR_MEMCTRL_ICCXIPLS_EN                  ((uint32_t)(0x1UL << MXC_F_GCR_MEMCTRL_ICCXIPLS_EN_POS)) /**< MEMCTRL_ICCXIPLS_EN Mask */

#define MXC_F_GCR_MEMCTRL_CRYPTOLS_EN_POS              27 /**< MEMCTRL_CRYPTOLS_EN Position */
#define MXC_F_GCR_MEMCTRL_CRYPTOLS_EN                  ((uint32_t)(0x1UL << MXC_F_GCR_MEMCTRL_CRYPTOLS_EN_POS)) /**< MEMCTRL_CRYPTOLS_EN Mask */

#define MXC_F_GCR_MEMCTRL_USBLS_EN_POS                 28 /**< MEMCTRL_USBLS_EN Position */
#define MXC_F_GCR_MEMCTRL_USBLS_EN                     ((uint32_t)(0x1UL << MXC_F_GCR_MEMCTRL_USBLS_EN_POS)) /**< MEMCTRL_USBLS_EN Mask */

#define MXC_F_GCR_MEMCTRL_ROM0LS_EN_POS                29 /**< MEMCTRL_ROM0LS_EN Position */
#define MXC_F_GCR_MEMCTRL_ROM0LS_EN                    ((uint32_t)(0x1UL << MXC_F_GCR_MEMCTRL_ROM0LS_EN_POS)) /**< MEMCTRL_ROM0LS_EN Mask */

#define MXC_F_GCR_MEMCTRL_ROM1LS_EN_POS                30 /**< MEMCTRL_ROM1LS_EN Position */
#define MXC_F_GCR_MEMCTRL_ROM1LS_EN                    ((uint32_t)(0x1UL << MXC_F_GCR_MEMCTRL_ROM1LS_EN_POS)) /**< MEMCTRL_ROM1LS_EN Mask */

#define MXC_F_GCR_MEMCTRL_MAALS_EN_POS                 31 /**< MEMCTRL_MAALS_EN Position */
#define MXC_F_GCR_MEMCTRL_MAALS_EN                     ((uint32_t)(0x1UL << MXC_F_GCR_MEMCTRL_MAALS_EN_POS)) /**< MEMCTRL_MAALS_EN Mask */

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

#define MXC_F_GCR_MEMZ_RAM4_POS                        4 /**< MEMZ_RAM4 Position */
#define MXC_F_GCR_MEMZ_RAM4                            ((uint32_t)(0x1UL << MXC_F_GCR_MEMZ_RAM4_POS)) /**< MEMZ_RAM4 Mask */

#define MXC_F_GCR_MEMZ_RAM5_POS                        5 /**< MEMZ_RAM5 Position */
#define MXC_F_GCR_MEMZ_RAM5                            ((uint32_t)(0x1UL << MXC_F_GCR_MEMZ_RAM5_POS)) /**< MEMZ_RAM5 Mask */

#define MXC_F_GCR_MEMZ_RAM6_POS                        6 /**< MEMZ_RAM6 Position */
#define MXC_F_GCR_MEMZ_RAM6                            ((uint32_t)(0x1UL << MXC_F_GCR_MEMZ_RAM6_POS)) /**< MEMZ_RAM6 Mask */

#define MXC_F_GCR_MEMZ_ICCXIP_POS                      9 /**< MEMZ_ICCXIP Position */
#define MXC_F_GCR_MEMZ_ICCXIP                          ((uint32_t)(0x1UL << MXC_F_GCR_MEMZ_ICCXIP_POS)) /**< MEMZ_ICCXIP Mask */

#define MXC_F_GCR_MEMZ_CRYPTO_POS                      12 /**< MEMZ_CRYPTO Position */
#define MXC_F_GCR_MEMZ_CRYPTO                          ((uint32_t)(0x1UL << MXC_F_GCR_MEMZ_CRYPTO_POS)) /**< MEMZ_CRYPTO Mask */

#define MXC_F_GCR_MEMZ_USBFIFO_POS                     13 /**< MEMZ_USBFIFO Position */
#define MXC_F_GCR_MEMZ_USBFIFO                         ((uint32_t)(0x1UL << MXC_F_GCR_MEMZ_USBFIFO_POS)) /**< MEMZ_USBFIFO Mask */

/**@} end of group GCR_MEMZ_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_SCCLKCTRL GCR_SCCLKCTRL
 * @brief    Smart Card Clock Control.
 * @{
 */
#define MXC_F_GCR_SCCLKCTRL_SC0CLK_DIV_POS             0 /**< SCCLKCTRL_SC0CLK_DIV Position */
#define MXC_F_GCR_SCCLKCTRL_SC0CLK_DIV                 ((uint32_t)(0x3FUL << MXC_F_GCR_SCCLKCTRL_SC0CLK_DIV_POS)) /**< SCCLKCTRL_SC0CLK_DIV Mask */

#define MXC_F_GCR_SCCLKCTRL_SC1CLK_DIV_POS             8 /**< SCCLKCTRL_SC1CLK_DIV Position */
#define MXC_F_GCR_SCCLKCTRL_SC1CLK_DIV                 ((uint32_t)(0x3FUL << MXC_F_GCR_SCCLKCTRL_SC1CLK_DIV_POS)) /**< SCCLKCTRL_SC1CLK_DIV Mask */

/**@} end of group GCR_SCCLKCTRL_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_SYSST GCR_SYSST
 * @brief    System Status Register.
 * @{
 */
#define MXC_F_GCR_SYSST_ICELOCK_POS                    0 /**< SYSST_ICELOCK Position */
#define MXC_F_GCR_SYSST_ICELOCK                        ((uint32_t)(0x1UL << MXC_F_GCR_SYSST_ICELOCK_POS)) /**< SYSST_ICELOCK Mask */

#define MXC_F_GCR_SYSST_CODEINTERR_POS                 1 /**< SYSST_CODEINTERR Position */
#define MXC_F_GCR_SYSST_CODEINTERR                     ((uint32_t)(0x1UL << MXC_F_GCR_SYSST_CODEINTERR_POS)) /**< SYSST_CODEINTERR Mask */

#define MXC_F_GCR_SYSST_SCMEMF_POS                     5 /**< SYSST_SCMEMF Position */
#define MXC_F_GCR_SYSST_SCMEMF                         ((uint32_t)(0x1UL << MXC_F_GCR_SYSST_SCMEMF_POS)) /**< SYSST_SCMEMF Mask */

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

#define MXC_F_GCR_RST1_SPIXIP_POS                      3 /**< RST1_SPIXIP Position */
#define MXC_F_GCR_RST1_SPIXIP                          ((uint32_t)(0x1UL << MXC_F_GCR_RST1_SPIXIP_POS)) /**< RST1_SPIXIP Mask */

#define MXC_F_GCR_RST1_SPIXIPM_POS                     4 /**< RST1_SPIXIPM Position */
#define MXC_F_GCR_RST1_SPIXIPM                         ((uint32_t)(0x1UL << MXC_F_GCR_RST1_SPIXIPM_POS)) /**< RST1_SPIXIPM Mask */

#define MXC_F_GCR_RST1_WDT1_POS                        8 /**< RST1_WDT1 Position */
#define MXC_F_GCR_RST1_WDT1                            ((uint32_t)(0x1UL << MXC_F_GCR_RST1_WDT1_POS)) /**< RST1_WDT1 Mask */

#define MXC_F_GCR_RST1_SPI3_POS                        9 /**< RST1_SPI3 Position */
#define MXC_F_GCR_RST1_SPI3                            ((uint32_t)(0x1UL << MXC_F_GCR_RST1_SPI3_POS)) /**< RST1_SPI3 Mask */

#define MXC_F_GCR_RST1_AC_POS                          14 /**< RST1_AC Position */
#define MXC_F_GCR_RST1_AC                              ((uint32_t)(0x1UL << MXC_F_GCR_RST1_AC_POS)) /**< RST1_AC Mask */

#define MXC_F_GCR_RST1_SEMA_POS                        16 /**< RST1_SEMA Position */
#define MXC_F_GCR_RST1_SEMA                            ((uint32_t)(0x1UL << MXC_F_GCR_RST1_SEMA_POS)) /**< RST1_SEMA Mask */

#define MXC_F_GCR_RST1_UART3_POS                       18 /**< RST1_UART3 Position */
#define MXC_F_GCR_RST1_UART3                           ((uint32_t)(0x1UL << MXC_F_GCR_RST1_UART3_POS)) /**< RST1_UART3 Mask */

#define MXC_F_GCR_RST1_SKBD_POS                        21 /**< RST1_SKBD Position */
#define MXC_F_GCR_RST1_SKBD                            ((uint32_t)(0x1UL << MXC_F_GCR_RST1_SKBD_POS)) /**< RST1_SKBD Mask */

#define MXC_F_GCR_RST1_MSRADC_POS                      22 /**< RST1_MSRADC Position */
#define MXC_F_GCR_RST1_MSRADC                          ((uint32_t)(0x1UL << MXC_F_GCR_RST1_MSRADC_POS)) /**< RST1_MSRADC Mask */

#define MXC_F_GCR_RST1_SC0_POS                         23 /**< RST1_SC0 Position */
#define MXC_F_GCR_RST1_SC0                             ((uint32_t)(0x1UL << MXC_F_GCR_RST1_SC0_POS)) /**< RST1_SC0 Mask */

#define MXC_F_GCR_RST1_SC1_POS                         24 /**< RST1_SC1 Position */
#define MXC_F_GCR_RST1_SC1                             ((uint32_t)(0x1UL << MXC_F_GCR_RST1_SC1_POS)) /**< RST1_SC1 Mask */

#define MXC_F_GCR_RST1_HTMR0_POS                       28 /**< RST1_HTMR0 Position */
#define MXC_F_GCR_RST1_HTMR0                           ((uint32_t)(0x1UL << MXC_F_GCR_RST1_HTMR0_POS)) /**< RST1_HTMR0 Mask */

#define MXC_F_GCR_RST1_HTMR1_POS                       29 /**< RST1_HTMR1 Position */
#define MXC_F_GCR_RST1_HTMR1                           ((uint32_t)(0x1UL << MXC_F_GCR_RST1_HTMR1_POS)) /**< RST1_HTMR1 Mask */

#define MXC_F_GCR_RST1_CPU1_POS                        31 /**< RST1_CPU1 Position */
#define MXC_F_GCR_RST1_CPU1                            ((uint32_t)(0x1UL << MXC_F_GCR_RST1_CPU1_POS)) /**< RST1_CPU1 Mask */

/**@} end of group GCR_RST1_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_PCLKDIS1 GCR_PCLKDIS1
 * @brief    Peripheral Clock Disable.
 * @{
 */
#define MXC_F_GCR_PCLKDIS1_UART2_POS                   1 /**< PCLKDIS1_UART2 Position */
#define MXC_F_GCR_PCLKDIS1_UART2                       ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS1_UART2_POS)) /**< PCLKDIS1_UART2 Mask */

#define MXC_F_GCR_PCLKDIS1_TRNG_POS                    2 /**< PCLKDIS1_TRNG Position */
#define MXC_F_GCR_PCLKDIS1_TRNG                        ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS1_TRNG_POS)) /**< PCLKDIS1_TRNG Mask */

#define MXC_F_GCR_PCLKDIS1_OTP_POS                     3 /**< PCLKDIS1_OTP Position */
#define MXC_F_GCR_PCLKDIS1_OTP                         ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS1_OTP_POS)) /**< PCLKDIS1_OTP Mask */

#define MXC_F_GCR_PCLKDIS1_WDT0_POS                    4 /**< PCLKDIS1_WDT0 Position */
#define MXC_F_GCR_PCLKDIS1_WDT0                        ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS1_WDT0_POS)) /**< PCLKDIS1_WDT0 Mask */

#define MXC_F_GCR_PCLKDIS1_WDT1_POS                    5 /**< PCLKDIS1_WDT1 Position */
#define MXC_F_GCR_PCLKDIS1_WDT1                        ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS1_WDT1_POS)) /**< PCLKDIS1_WDT1 Mask */

#define MXC_F_GCR_PCLKDIS1_SEMA_POS                    9 /**< PCLKDIS1_SEMA Position */
#define MXC_F_GCR_PCLKDIS1_SEMA                        ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS1_SEMA_POS)) /**< PCLKDIS1_SEMA Mask */

#define MXC_F_GCR_PCLKDIS1_SPI3_POS                    14 /**< PCLKDIS1_SPI3 Position */
#define MXC_F_GCR_PCLKDIS1_SPI3                        ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS1_SPI3_POS)) /**< PCLKDIS1_SPI3 Mask */

#define MXC_F_GCR_PCLKDIS1_UART3_POS                   22 /**< PCLKDIS1_UART3 Position */
#define MXC_F_GCR_PCLKDIS1_UART3                       ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS1_UART3_POS)) /**< PCLKDIS1_UART3 Mask */

#define MXC_F_GCR_PCLKDIS1_MSRADC_POS                  25 /**< PCLKDIS1_MSRADC Position */
#define MXC_F_GCR_PCLKDIS1_MSRADC                      ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS1_MSRADC_POS)) /**< PCLKDIS1_MSRADC Mask */

#define MXC_F_GCR_PCLKDIS1_SC0_POS                     26 /**< PCLKDIS1_SC0 Position */
#define MXC_F_GCR_PCLKDIS1_SC0                         ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS1_SC0_POS)) /**< PCLKDIS1_SC0 Mask */

#define MXC_F_GCR_PCLKDIS1_SC1_POS                     27 /**< PCLKDIS1_SC1 Position */
#define MXC_F_GCR_PCLKDIS1_SC1                         ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS1_SC1_POS)) /**< PCLKDIS1_SC1 Mask */

#define MXC_F_GCR_PCLKDIS1_CPU1_POS                    31 /**< PCLKDIS1_CPU1 Position */
#define MXC_F_GCR_PCLKDIS1_CPU1                        ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS1_CPU1_POS)) /**< PCLKDIS1_CPU1 Mask */

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

#define MXC_F_GCR_SYSIE_CIE_POS                        1 /**< SYSIE_CIE Position */
#define MXC_F_GCR_SYSIE_CIE                            ((uint32_t)(0x1UL << MXC_F_GCR_SYSIE_CIE_POS)) /**< SYSIE_CIE Mask */

#define MXC_F_GCR_SYSIE_SCMF_POS                       5 /**< SYSIE_SCMF Position */
#define MXC_F_GCR_SYSIE_SCMF                           ((uint32_t)(0x1UL << MXC_F_GCR_SYSIE_SCMF_POS)) /**< SYSIE_SCMF Mask */

/**@} end of group GCR_SYSIE_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_IPOCNT GCR_IPOCNT
 * @brief    IPO Warmup Count Register.
 * @{
 */
#define MXC_F_GCR_IPOCNT_WMUPCNT_POS                   0 /**< IPOCNT_WMUPCNT Position */
#define MXC_F_GCR_IPOCNT_WMUPCNT                       ((uint32_t)(0x3FFUL << MXC_F_GCR_IPOCNT_WMUPCNT_POS)) /**< IPOCNT_WMUPCNT Mask */

/**@} end of group GCR_IPOCNT_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32572_INCLUDE_GCR_REGS_H_
