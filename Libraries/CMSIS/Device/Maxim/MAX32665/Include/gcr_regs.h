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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32665_INCLUDE_GCR_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32665_INCLUDE_GCR_REGS_H_

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
    __IO uint32_t scon;                 /**< <tt>\b 0x00:</tt> GCR SCON Register */
    __IO uint32_t rstr0;                /**< <tt>\b 0x04:</tt> GCR RSTR0 Register */
    __IO uint32_t clkcn;                /**< <tt>\b 0x08:</tt> GCR CLKCN Register */
    __IO uint32_t pm;                   /**< <tt>\b 0x0C:</tt> GCR PM Register */
    __R  uint32_t rsv_0x10_0x17[2];
    __IO uint32_t pckdiv;               /**< <tt>\b 0x18:</tt> GCR PCKDIV Register */
    __R  uint32_t rsv_0x1c_0x23[2];
    __IO uint32_t perckcn0;             /**< <tt>\b 0x24:</tt> GCR PERCKCN0 Register */
    __IO uint32_t memckcn;              /**< <tt>\b 0x28:</tt> GCR MEMCKCN Register */
    __IO uint32_t memzcn;               /**< <tt>\b 0x2C:</tt> GCR MEMZCN Register */
    __R  uint32_t rsv_0x30_0x3f[4];
    __IO uint32_t sysst;                /**< <tt>\b 0x40:</tt> GCR SYSST Register */
    __IO uint32_t rstr1;                /**< <tt>\b 0x44:</tt> GCR RSTR1 Register */
    __IO uint32_t perckcn1;             /**< <tt>\b 0x48:</tt> GCR PERCKCN1 Register */
    __IO uint32_t event_en;             /**< <tt>\b 0x4C:</tt> GCR EVENT_EN Register */
    __I  uint32_t revision;             /**< <tt>\b 0x50:</tt> GCR REVISION Register */
    __IO uint32_t syssie;               /**< <tt>\b 0x54:</tt> GCR SYSSIE Register */
    __R  uint32_t rsv_0x58_0x63[3];
    __IO uint32_t ecc_er;               /**< <tt>\b 0x64:</tt> GCR ECC_ER Register */
    __IO uint32_t ecc_ced;              /**< <tt>\b 0x68:</tt> GCR ECC_CED Register */
    __IO uint32_t ecc_irqen;            /**< <tt>\b 0x6C:</tt> GCR ECC_IRQEN Register */
    __IO uint32_t ecc_errad;            /**< <tt>\b 0x70:</tt> GCR ECC_ERRAD Register */
    __IO uint32_t btle_ldocr;           /**< <tt>\b 0x74:</tt> GCR BTLE_LDOCR Register */
    __IO uint32_t btle_ldodcr;          /**< <tt>\b 0x78:</tt> GCR BTLE_LDODCR Register */
    __R  uint32_t rsv_0x7c;
    __IO uint32_t gp0;                  /**< <tt>\b 0x80:</tt> GCR GP0 Register */
    __IO uint32_t apb_async;            /**< <tt>\b 0x84:</tt> GCR APB_ASYNC Register */
} mxc_gcr_regs_t;

/* Register offsets for module GCR */
/**
 * @ingroup    gcr_registers
 * @defgroup   GCR_Register_Offsets Register Offsets
 * @brief      GCR Peripheral Register Offsets from the GCR Base Peripheral Address.
 * @{
 */
#define MXC_R_GCR_SCON                     ((uint32_t)0x00000000UL) /**< Offset from GCR Base Address: <tt> 0x0000</tt> */
#define MXC_R_GCR_RSTR0                    ((uint32_t)0x00000004UL) /**< Offset from GCR Base Address: <tt> 0x0004</tt> */
#define MXC_R_GCR_CLKCN                    ((uint32_t)0x00000008UL) /**< Offset from GCR Base Address: <tt> 0x0008</tt> */
#define MXC_R_GCR_PM                       ((uint32_t)0x0000000CUL) /**< Offset from GCR Base Address: <tt> 0x000C</tt> */
#define MXC_R_GCR_PCKDIV                   ((uint32_t)0x00000018UL) /**< Offset from GCR Base Address: <tt> 0x0018</tt> */
#define MXC_R_GCR_PERCKCN0                 ((uint32_t)0x00000024UL) /**< Offset from GCR Base Address: <tt> 0x0024</tt> */
#define MXC_R_GCR_MEMCKCN                  ((uint32_t)0x00000028UL) /**< Offset from GCR Base Address: <tt> 0x0028</tt> */
#define MXC_R_GCR_MEMZCN                   ((uint32_t)0x0000002CUL) /**< Offset from GCR Base Address: <tt> 0x002C</tt> */
#define MXC_R_GCR_SYSST                    ((uint32_t)0x00000040UL) /**< Offset from GCR Base Address: <tt> 0x0040</tt> */
#define MXC_R_GCR_RSTR1                    ((uint32_t)0x00000044UL) /**< Offset from GCR Base Address: <tt> 0x0044</tt> */
#define MXC_R_GCR_PERCKCN1                 ((uint32_t)0x00000048UL) /**< Offset from GCR Base Address: <tt> 0x0048</tt> */
#define MXC_R_GCR_EVENT_EN                 ((uint32_t)0x0000004CUL) /**< Offset from GCR Base Address: <tt> 0x004C</tt> */
#define MXC_R_GCR_REVISION                 ((uint32_t)0x00000050UL) /**< Offset from GCR Base Address: <tt> 0x0050</tt> */
#define MXC_R_GCR_SYSSIE                   ((uint32_t)0x00000054UL) /**< Offset from GCR Base Address: <tt> 0x0054</tt> */
#define MXC_R_GCR_ECC_ER                   ((uint32_t)0x00000064UL) /**< Offset from GCR Base Address: <tt> 0x0064</tt> */
#define MXC_R_GCR_ECC_CED                  ((uint32_t)0x00000068UL) /**< Offset from GCR Base Address: <tt> 0x0068</tt> */
#define MXC_R_GCR_ECC_IRQEN                ((uint32_t)0x0000006CUL) /**< Offset from GCR Base Address: <tt> 0x006C</tt> */
#define MXC_R_GCR_ECC_ERRAD                ((uint32_t)0x00000070UL) /**< Offset from GCR Base Address: <tt> 0x0070</tt> */
#define MXC_R_GCR_BTLE_LDOCR               ((uint32_t)0x00000074UL) /**< Offset from GCR Base Address: <tt> 0x0074</tt> */
#define MXC_R_GCR_BTLE_LDODCR              ((uint32_t)0x00000078UL) /**< Offset from GCR Base Address: <tt> 0x0078</tt> */
#define MXC_R_GCR_GP0                      ((uint32_t)0x00000080UL) /**< Offset from GCR Base Address: <tt> 0x0080</tt> */
#define MXC_R_GCR_APB_ASYNC                ((uint32_t)0x00000084UL) /**< Offset from GCR Base Address: <tt> 0x0084</tt> */
/**@} end of group gcr_registers */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_SCON GCR_SCON
 * @brief    System Control.
 * @{
 */
#define MXC_F_GCR_SCON_BSTAPEN_POS                     0 /**< SCON_BSTAPEN Position */
#define MXC_F_GCR_SCON_BSTAPEN                         ((uint32_t)(0x1UL << MXC_F_GCR_SCON_BSTAPEN_POS)) /**< SCON_BSTAPEN Mask */

#define MXC_F_GCR_SCON_SBUSARB_POS                     1 /**< SCON_SBUSARB Position */
#define MXC_F_GCR_SCON_SBUSARB                         ((uint32_t)(0x3UL << MXC_F_GCR_SCON_SBUSARB_POS)) /**< SCON_SBUSARB Mask */
#define MXC_V_GCR_SCON_SBUSARB_FIX                     ((uint32_t)0x0UL) /**< SCON_SBUSARB_FIX Value */
#define MXC_S_GCR_SCON_SBUSARB_FIX                     (MXC_V_GCR_SCON_SBUSARB_FIX << MXC_F_GCR_SCON_SBUSARB_POS) /**< SCON_SBUSARB_FIX Setting */
#define MXC_V_GCR_SCON_SBUSARB_ROUND                   ((uint32_t)0x1UL) /**< SCON_SBUSARB_ROUND Value */
#define MXC_S_GCR_SCON_SBUSARB_ROUND                   (MXC_V_GCR_SCON_SBUSARB_ROUND << MXC_F_GCR_SCON_SBUSARB_POS) /**< SCON_SBUSARB_ROUND Setting */

#define MXC_F_GCR_SCON_FLASH_PAGE_FLIP_POS             4 /**< SCON_FLASH_PAGE_FLIP Position */
#define MXC_F_GCR_SCON_FLASH_PAGE_FLIP                 ((uint32_t)(0x1UL << MXC_F_GCR_SCON_FLASH_PAGE_FLIP_POS)) /**< SCON_FLASH_PAGE_FLIP Mask */

#define MXC_F_GCR_SCON_CCACHE_FLUSH_POS                6 /**< SCON_CCACHE_FLUSH Position */
#define MXC_F_GCR_SCON_CCACHE_FLUSH                    ((uint32_t)(0x1UL << MXC_F_GCR_SCON_CCACHE_FLUSH_POS)) /**< SCON_CCACHE_FLUSH Mask */

#define MXC_F_GCR_SCON_DCACHE_FLUSH_POS                7 /**< SCON_DCACHE_FLUSH Position */
#define MXC_F_GCR_SCON_DCACHE_FLUSH                    ((uint32_t)(0x1UL << MXC_F_GCR_SCON_DCACHE_FLUSH_POS)) /**< SCON_DCACHE_FLUSH Mask */

#define MXC_F_GCR_SCON_SRCC_DIS_POS                    9 /**< SCON_SRCC_DIS Position */
#define MXC_F_GCR_SCON_SRCC_DIS                        ((uint32_t)(0x1UL << MXC_F_GCR_SCON_SRCC_DIS_POS)) /**< SCON_SRCC_DIS Mask */

#define MXC_F_GCR_SCON_CCHK_POS                        13 /**< SCON_CCHK Position */
#define MXC_F_GCR_SCON_CCHK                            ((uint32_t)(0x1UL << MXC_F_GCR_SCON_CCHK_POS)) /**< SCON_CCHK Mask */

#define MXC_F_GCR_SCON_CHKRES_POS                      15 /**< SCON_CHKRES Position */
#define MXC_F_GCR_SCON_CHKRES                          ((uint32_t)(0x1UL << MXC_F_GCR_SCON_CHKRES_POS)) /**< SCON_CHKRES Mask */

#define MXC_F_GCR_SCON_OVR_POS                         16 /**< SCON_OVR Position */
#define MXC_F_GCR_SCON_OVR                             ((uint32_t)(0x3UL << MXC_F_GCR_SCON_OVR_POS)) /**< SCON_OVR Mask */
#define MXC_V_GCR_SCON_OVR_0_9V                        ((uint32_t)0x0UL) /**< SCON_OVR_0_9V Value */
#define MXC_S_GCR_SCON_OVR_0_9V                        (MXC_V_GCR_SCON_OVR_0_9V << MXC_F_GCR_SCON_OVR_POS) /**< SCON_OVR_0_9V Setting */
#define MXC_V_GCR_SCON_OVR_1_0V                        ((uint32_t)0x1UL) /**< SCON_OVR_1_0V Value */
#define MXC_S_GCR_SCON_OVR_1_0V                        (MXC_V_GCR_SCON_OVR_1_0V << MXC_F_GCR_SCON_OVR_POS) /**< SCON_OVR_1_0V Setting */
#define MXC_V_GCR_SCON_OVR_1_1V                        ((uint32_t)0x2UL) /**< SCON_OVR_1_1V Value */
#define MXC_S_GCR_SCON_OVR_1_1V                        (MXC_V_GCR_SCON_OVR_1_1V << MXC_F_GCR_SCON_OVR_POS) /**< SCON_OVR_1_1V Setting */

/**@} end of group GCR_SCON_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_RSTR0 GCR_RSTR0
 * @brief    Reset.
 * @{
 */
#define MXC_F_GCR_RSTR0_DMA_POS                        0 /**< RSTR0_DMA Position */
#define MXC_F_GCR_RSTR0_DMA                            ((uint32_t)(0x1UL << MXC_F_GCR_RSTR0_DMA_POS)) /**< RSTR0_DMA Mask */

#define MXC_F_GCR_RSTR0_WDT0_POS                       1 /**< RSTR0_WDT0 Position */
#define MXC_F_GCR_RSTR0_WDT0                           ((uint32_t)(0x1UL << MXC_F_GCR_RSTR0_WDT0_POS)) /**< RSTR0_WDT0 Mask */

#define MXC_F_GCR_RSTR0_GPIO0_POS                      2 /**< RSTR0_GPIO0 Position */
#define MXC_F_GCR_RSTR0_GPIO0                          ((uint32_t)(0x1UL << MXC_F_GCR_RSTR0_GPIO0_POS)) /**< RSTR0_GPIO0 Mask */

#define MXC_F_GCR_RSTR0_GPIO1_POS                      3 /**< RSTR0_GPIO1 Position */
#define MXC_F_GCR_RSTR0_GPIO1                          ((uint32_t)(0x1UL << MXC_F_GCR_RSTR0_GPIO1_POS)) /**< RSTR0_GPIO1 Mask */

#define MXC_F_GCR_RSTR0_TIMER0_POS                     5 /**< RSTR0_TIMER0 Position */
#define MXC_F_GCR_RSTR0_TIMER0                         ((uint32_t)(0x1UL << MXC_F_GCR_RSTR0_TIMER0_POS)) /**< RSTR0_TIMER0 Mask */

#define MXC_F_GCR_RSTR0_TIMER1_POS                     6 /**< RSTR0_TIMER1 Position */
#define MXC_F_GCR_RSTR0_TIMER1                         ((uint32_t)(0x1UL << MXC_F_GCR_RSTR0_TIMER1_POS)) /**< RSTR0_TIMER1 Mask */

#define MXC_F_GCR_RSTR0_TIMER2_POS                     7 /**< RSTR0_TIMER2 Position */
#define MXC_F_GCR_RSTR0_TIMER2                         ((uint32_t)(0x1UL << MXC_F_GCR_RSTR0_TIMER2_POS)) /**< RSTR0_TIMER2 Mask */

#define MXC_F_GCR_RSTR0_TIMER3_POS                     8 /**< RSTR0_TIMER3 Position */
#define MXC_F_GCR_RSTR0_TIMER3                         ((uint32_t)(0x1UL << MXC_F_GCR_RSTR0_TIMER3_POS)) /**< RSTR0_TIMER3 Mask */

#define MXC_F_GCR_RSTR0_TIMER4_POS                     9 /**< RSTR0_TIMER4 Position */
#define MXC_F_GCR_RSTR0_TIMER4                         ((uint32_t)(0x1UL << MXC_F_GCR_RSTR0_TIMER4_POS)) /**< RSTR0_TIMER4 Mask */

#define MXC_F_GCR_RSTR0_TIMER5_POS                     10 /**< RSTR0_TIMER5 Position */
#define MXC_F_GCR_RSTR0_TIMER5                         ((uint32_t)(0x1UL << MXC_F_GCR_RSTR0_TIMER5_POS)) /**< RSTR0_TIMER5 Mask */

#define MXC_F_GCR_RSTR0_UART0_POS                      11 /**< RSTR0_UART0 Position */
#define MXC_F_GCR_RSTR0_UART0                          ((uint32_t)(0x1UL << MXC_F_GCR_RSTR0_UART0_POS)) /**< RSTR0_UART0 Mask */

#define MXC_F_GCR_RSTR0_UART1_POS                      12 /**< RSTR0_UART1 Position */
#define MXC_F_GCR_RSTR0_UART1                          ((uint32_t)(0x1UL << MXC_F_GCR_RSTR0_UART1_POS)) /**< RSTR0_UART1 Mask */

#define MXC_F_GCR_RSTR0_SPI1_POS                       13 /**< RSTR0_SPI1 Position */
#define MXC_F_GCR_RSTR0_SPI1                           ((uint32_t)(0x1UL << MXC_F_GCR_RSTR0_SPI1_POS)) /**< RSTR0_SPI1 Mask */

#define MXC_F_GCR_RSTR0_SPI2_POS                       14 /**< RSTR0_SPI2 Position */
#define MXC_F_GCR_RSTR0_SPI2                           ((uint32_t)(0x1UL << MXC_F_GCR_RSTR0_SPI2_POS)) /**< RSTR0_SPI2 Mask */

#define MXC_F_GCR_RSTR0_I2C0_POS                       16 /**< RSTR0_I2C0 Position */
#define MXC_F_GCR_RSTR0_I2C0                           ((uint32_t)(0x1UL << MXC_F_GCR_RSTR0_I2C0_POS)) /**< RSTR0_I2C0 Mask */

#define MXC_F_GCR_RSTR0_RTC_POS                        17 /**< RSTR0_RTC Position */
#define MXC_F_GCR_RSTR0_RTC                            ((uint32_t)(0x1UL << MXC_F_GCR_RSTR0_RTC_POS)) /**< RSTR0_RTC Mask */

#define MXC_F_GCR_RSTR0_CRYPTO_POS                     18 /**< RSTR0_CRYPTO Position */
#define MXC_F_GCR_RSTR0_CRYPTO                         ((uint32_t)(0x1UL << MXC_F_GCR_RSTR0_CRYPTO_POS)) /**< RSTR0_CRYPTO Mask */

#define MXC_F_GCR_RSTR0_SMPHR_POS                      22 /**< RSTR0_SMPHR Position */
#define MXC_F_GCR_RSTR0_SMPHR                          ((uint32_t)(0x1UL << MXC_F_GCR_RSTR0_SMPHR_POS)) /**< RSTR0_SMPHR Mask */

#define MXC_F_GCR_RSTR0_USB_POS                        23 /**< RSTR0_USB Position */
#define MXC_F_GCR_RSTR0_USB                            ((uint32_t)(0x1UL << MXC_F_GCR_RSTR0_USB_POS)) /**< RSTR0_USB Mask */

#define MXC_F_GCR_RSTR0_ADC_POS                        26 /**< RSTR0_ADC Position */
#define MXC_F_GCR_RSTR0_ADC                            ((uint32_t)(0x1UL << MXC_F_GCR_RSTR0_ADC_POS)) /**< RSTR0_ADC Mask */

#define MXC_F_GCR_RSTR0_DMA1_POS                       27 /**< RSTR0_DMA1 Position */
#define MXC_F_GCR_RSTR0_DMA1                           ((uint32_t)(0x1UL << MXC_F_GCR_RSTR0_DMA1_POS)) /**< RSTR0_DMA1 Mask */

#define MXC_F_GCR_RSTR0_UART2_POS                      28 /**< RSTR0_UART2 Position */
#define MXC_F_GCR_RSTR0_UART2                          ((uint32_t)(0x1UL << MXC_F_GCR_RSTR0_UART2_POS)) /**< RSTR0_UART2 Mask */

#define MXC_F_GCR_RSTR0_SRST_POS                       29 /**< RSTR0_SRST Position */
#define MXC_F_GCR_RSTR0_SRST                           ((uint32_t)(0x1UL << MXC_F_GCR_RSTR0_SRST_POS)) /**< RSTR0_SRST Mask */

#define MXC_F_GCR_RSTR0_PRST_POS                       30 /**< RSTR0_PRST Position */
#define MXC_F_GCR_RSTR0_PRST                           ((uint32_t)(0x1UL << MXC_F_GCR_RSTR0_PRST_POS)) /**< RSTR0_PRST Mask */

#define MXC_F_GCR_RSTR0_SYSTEM_POS                     31 /**< RSTR0_SYSTEM Position */
#define MXC_F_GCR_RSTR0_SYSTEM                         ((uint32_t)(0x1UL << MXC_F_GCR_RSTR0_SYSTEM_POS)) /**< RSTR0_SYSTEM Mask */

/**@} end of group GCR_RSTR0_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_CLKCN GCR_CLKCN
 * @brief    Clock Control.
 * @{
 */
#define MXC_F_GCR_CLKCN_PSC_POS                        6 /**< CLKCN_PSC Position */
#define MXC_F_GCR_CLKCN_PSC                            ((uint32_t)(0x7UL << MXC_F_GCR_CLKCN_PSC_POS)) /**< CLKCN_PSC Mask */
#define MXC_V_GCR_CLKCN_PSC_DIV1                       ((uint32_t)0x0UL) /**< CLKCN_PSC_DIV1 Value */
#define MXC_S_GCR_CLKCN_PSC_DIV1                       (MXC_V_GCR_CLKCN_PSC_DIV1 << MXC_F_GCR_CLKCN_PSC_POS) /**< CLKCN_PSC_DIV1 Setting */
#define MXC_V_GCR_CLKCN_PSC_DIV2                       ((uint32_t)0x1UL) /**< CLKCN_PSC_DIV2 Value */
#define MXC_S_GCR_CLKCN_PSC_DIV2                       (MXC_V_GCR_CLKCN_PSC_DIV2 << MXC_F_GCR_CLKCN_PSC_POS) /**< CLKCN_PSC_DIV2 Setting */
#define MXC_V_GCR_CLKCN_PSC_DIV4                       ((uint32_t)0x2UL) /**< CLKCN_PSC_DIV4 Value */
#define MXC_S_GCR_CLKCN_PSC_DIV4                       (MXC_V_GCR_CLKCN_PSC_DIV4 << MXC_F_GCR_CLKCN_PSC_POS) /**< CLKCN_PSC_DIV4 Setting */
#define MXC_V_GCR_CLKCN_PSC_DIV8                       ((uint32_t)0x3UL) /**< CLKCN_PSC_DIV8 Value */
#define MXC_S_GCR_CLKCN_PSC_DIV8                       (MXC_V_GCR_CLKCN_PSC_DIV8 << MXC_F_GCR_CLKCN_PSC_POS) /**< CLKCN_PSC_DIV8 Setting */
#define MXC_V_GCR_CLKCN_PSC_DIV16                      ((uint32_t)0x4UL) /**< CLKCN_PSC_DIV16 Value */
#define MXC_S_GCR_CLKCN_PSC_DIV16                      (MXC_V_GCR_CLKCN_PSC_DIV16 << MXC_F_GCR_CLKCN_PSC_POS) /**< CLKCN_PSC_DIV16 Setting */
#define MXC_V_GCR_CLKCN_PSC_DIV32                      ((uint32_t)0x5UL) /**< CLKCN_PSC_DIV32 Value */
#define MXC_S_GCR_CLKCN_PSC_DIV32                      (MXC_V_GCR_CLKCN_PSC_DIV32 << MXC_F_GCR_CLKCN_PSC_POS) /**< CLKCN_PSC_DIV32 Setting */
#define MXC_V_GCR_CLKCN_PSC_DIV64                      ((uint32_t)0x6UL) /**< CLKCN_PSC_DIV64 Value */
#define MXC_S_GCR_CLKCN_PSC_DIV64                      (MXC_V_GCR_CLKCN_PSC_DIV64 << MXC_F_GCR_CLKCN_PSC_POS) /**< CLKCN_PSC_DIV64 Setting */
#define MXC_V_GCR_CLKCN_PSC_DIV128                     ((uint32_t)0x7UL) /**< CLKCN_PSC_DIV128 Value */
#define MXC_S_GCR_CLKCN_PSC_DIV128                     (MXC_V_GCR_CLKCN_PSC_DIV128 << MXC_F_GCR_CLKCN_PSC_POS) /**< CLKCN_PSC_DIV128 Setting */

#define MXC_F_GCR_CLKCN_CLKSEL_POS                     9 /**< CLKCN_CLKSEL Position */
#define MXC_F_GCR_CLKCN_CLKSEL                         ((uint32_t)(0x7UL << MXC_F_GCR_CLKCN_CLKSEL_POS)) /**< CLKCN_CLKSEL Mask */
#define MXC_V_GCR_CLKCN_CLKSEL_HIRC                    ((uint32_t)0x0UL) /**< CLKCN_CLKSEL_HIRC Value */
#define MXC_S_GCR_CLKCN_CLKSEL_HIRC                    (MXC_V_GCR_CLKCN_CLKSEL_HIRC << MXC_F_GCR_CLKCN_CLKSEL_POS) /**< CLKCN_CLKSEL_HIRC Setting */
#define MXC_V_GCR_CLKCN_CLKSEL_XTAL32M                 ((uint32_t)0x2UL) /**< CLKCN_CLKSEL_XTAL32M Value */
#define MXC_S_GCR_CLKCN_CLKSEL_XTAL32M                 (MXC_V_GCR_CLKCN_CLKSEL_XTAL32M << MXC_F_GCR_CLKCN_CLKSEL_POS) /**< CLKCN_CLKSEL_XTAL32M Setting */
#define MXC_V_GCR_CLKCN_CLKSEL_LIRC8                   ((uint32_t)0x3UL) /**< CLKCN_CLKSEL_LIRC8 Value */
#define MXC_S_GCR_CLKCN_CLKSEL_LIRC8                   (MXC_V_GCR_CLKCN_CLKSEL_LIRC8 << MXC_F_GCR_CLKCN_CLKSEL_POS) /**< CLKCN_CLKSEL_LIRC8 Setting */
#define MXC_V_GCR_CLKCN_CLKSEL_HIRC96                  ((uint32_t)0x4UL) /**< CLKCN_CLKSEL_HIRC96 Value */
#define MXC_S_GCR_CLKCN_CLKSEL_HIRC96                  (MXC_V_GCR_CLKCN_CLKSEL_HIRC96 << MXC_F_GCR_CLKCN_CLKSEL_POS) /**< CLKCN_CLKSEL_HIRC96 Setting */
#define MXC_V_GCR_CLKCN_CLKSEL_HIRC8                   ((uint32_t)0x5UL) /**< CLKCN_CLKSEL_HIRC8 Value */
#define MXC_S_GCR_CLKCN_CLKSEL_HIRC8                   (MXC_V_GCR_CLKCN_CLKSEL_HIRC8 << MXC_F_GCR_CLKCN_CLKSEL_POS) /**< CLKCN_CLKSEL_HIRC8 Setting */
#define MXC_V_GCR_CLKCN_CLKSEL_XTAL32K                 ((uint32_t)0x6UL) /**< CLKCN_CLKSEL_XTAL32K Value */
#define MXC_S_GCR_CLKCN_CLKSEL_XTAL32K                 (MXC_V_GCR_CLKCN_CLKSEL_XTAL32K << MXC_F_GCR_CLKCN_CLKSEL_POS) /**< CLKCN_CLKSEL_XTAL32K Setting */

#define MXC_F_GCR_CLKCN_CKRDY_POS                      13 /**< CLKCN_CKRDY Position */
#define MXC_F_GCR_CLKCN_CKRDY                          ((uint32_t)(0x1UL << MXC_F_GCR_CLKCN_CKRDY_POS)) /**< CLKCN_CKRDY Mask */

#define MXC_F_GCR_CLKCN_CCD_POS                        15 /**< CLKCN_CCD Position */
#define MXC_F_GCR_CLKCN_CCD                            ((uint32_t)(0x1UL << MXC_F_GCR_CLKCN_CCD_POS)) /**< CLKCN_CCD Mask */

#define MXC_F_GCR_CLKCN_X32M_EN_POS                    16 /**< CLKCN_X32M_EN Position */
#define MXC_F_GCR_CLKCN_X32M_EN                        ((uint32_t)(0x1UL << MXC_F_GCR_CLKCN_X32M_EN_POS)) /**< CLKCN_X32M_EN Mask */

#define MXC_F_GCR_CLKCN_X32K_EN_POS                    17 /**< CLKCN_X32K_EN Position */
#define MXC_F_GCR_CLKCN_X32K_EN                        ((uint32_t)(0x1UL << MXC_F_GCR_CLKCN_X32K_EN_POS)) /**< CLKCN_X32K_EN Mask */

#define MXC_F_GCR_CLKCN_HIRC_EN_POS                    18 /**< CLKCN_HIRC_EN Position */
#define MXC_F_GCR_CLKCN_HIRC_EN                        ((uint32_t)(0x1UL << MXC_F_GCR_CLKCN_HIRC_EN_POS)) /**< CLKCN_HIRC_EN Mask */

#define MXC_F_GCR_CLKCN_HIRC96M_EN_POS                 19 /**< CLKCN_HIRC96M_EN Position */
#define MXC_F_GCR_CLKCN_HIRC96M_EN                     ((uint32_t)(0x1UL << MXC_F_GCR_CLKCN_HIRC96M_EN_POS)) /**< CLKCN_HIRC96M_EN Mask */

#define MXC_F_GCR_CLKCN_HIRC8M_EN_POS                  20 /**< CLKCN_HIRC8M_EN Position */
#define MXC_F_GCR_CLKCN_HIRC8M_EN                      ((uint32_t)(0x1UL << MXC_F_GCR_CLKCN_HIRC8M_EN_POS)) /**< CLKCN_HIRC8M_EN Mask */

#define MXC_F_GCR_CLKCN_HIRC8M_VS_POS                  21 /**< CLKCN_HIRC8M_VS Position */
#define MXC_F_GCR_CLKCN_HIRC8M_VS                      ((uint32_t)(0x1UL << MXC_F_GCR_CLKCN_HIRC8M_VS_POS)) /**< CLKCN_HIRC8M_VS Mask */

#define MXC_F_GCR_CLKCN_X32M_RDY_POS                   24 /**< CLKCN_X32M_RDY Position */
#define MXC_F_GCR_CLKCN_X32M_RDY                       ((uint32_t)(0x1UL << MXC_F_GCR_CLKCN_X32M_RDY_POS)) /**< CLKCN_X32M_RDY Mask */

#define MXC_F_GCR_CLKCN_X32K_RDY_POS                   25 /**< CLKCN_X32K_RDY Position */
#define MXC_F_GCR_CLKCN_X32K_RDY                       ((uint32_t)(0x1UL << MXC_F_GCR_CLKCN_X32K_RDY_POS)) /**< CLKCN_X32K_RDY Mask */

#define MXC_F_GCR_CLKCN_HIRC_RDY_POS                   26 /**< CLKCN_HIRC_RDY Position */
#define MXC_F_GCR_CLKCN_HIRC_RDY                       ((uint32_t)(0x1UL << MXC_F_GCR_CLKCN_HIRC_RDY_POS)) /**< CLKCN_HIRC_RDY Mask */

#define MXC_F_GCR_CLKCN_HIRC96M_RDY_POS                27 /**< CLKCN_HIRC96M_RDY Position */
#define MXC_F_GCR_CLKCN_HIRC96M_RDY                    ((uint32_t)(0x1UL << MXC_F_GCR_CLKCN_HIRC96M_RDY_POS)) /**< CLKCN_HIRC96M_RDY Mask */

#define MXC_F_GCR_CLKCN_HIRC8M_RDY_POS                 28 /**< CLKCN_HIRC8M_RDY Position */
#define MXC_F_GCR_CLKCN_HIRC8M_RDY                     ((uint32_t)(0x1UL << MXC_F_GCR_CLKCN_HIRC8M_RDY_POS)) /**< CLKCN_HIRC8M_RDY Mask */

/**@} end of group GCR_CLKCN_Register */

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
#define MXC_V_GCR_PM_MODE_DEEPSLEEP                    ((uint32_t)0x2UL) /**< PM_MODE_DEEPSLEEP Value */
#define MXC_S_GCR_PM_MODE_DEEPSLEEP                    (MXC_V_GCR_PM_MODE_DEEPSLEEP << MXC_F_GCR_PM_MODE_POS) /**< PM_MODE_DEEPSLEEP Setting */
#define MXC_V_GCR_PM_MODE_SHUTDOWN                     ((uint32_t)0x3UL) /**< PM_MODE_SHUTDOWN Value */
#define MXC_S_GCR_PM_MODE_SHUTDOWN                     (MXC_V_GCR_PM_MODE_SHUTDOWN << MXC_F_GCR_PM_MODE_POS) /**< PM_MODE_SHUTDOWN Setting */
#define MXC_V_GCR_PM_MODE_BACKUP                       ((uint32_t)0x4UL) /**< PM_MODE_BACKUP Value */
#define MXC_S_GCR_PM_MODE_BACKUP                       (MXC_V_GCR_PM_MODE_BACKUP << MXC_F_GCR_PM_MODE_POS) /**< PM_MODE_BACKUP Setting */

#define MXC_F_GCR_PM_GPIOWKEN_POS                      4 /**< PM_GPIOWKEN Position */
#define MXC_F_GCR_PM_GPIOWKEN                          ((uint32_t)(0x1UL << MXC_F_GCR_PM_GPIOWKEN_POS)) /**< PM_GPIOWKEN Mask */

#define MXC_F_GCR_PM_RTCWKEN_POS                       5 /**< PM_RTCWKEN Position */
#define MXC_F_GCR_PM_RTCWKEN                           ((uint32_t)(0x1UL << MXC_F_GCR_PM_RTCWKEN_POS)) /**< PM_RTCWKEN Mask */

#define MXC_F_GCR_PM_USBWKEN_POS                       6 /**< PM_USBWKEN Position */
#define MXC_F_GCR_PM_USBWKEN                           ((uint32_t)(0x1UL << MXC_F_GCR_PM_USBWKEN_POS)) /**< PM_USBWKEN Mask */

#define MXC_F_GCR_PM_WUTWKEN_POS                       7 /**< PM_WUTWKEN Position */
#define MXC_F_GCR_PM_WUTWKEN                           ((uint32_t)(0x1UL << MXC_F_GCR_PM_WUTWKEN_POS)) /**< PM_WUTWKEN Mask */

#define MXC_F_GCR_PM_COMPWKEN_POS                      8 /**< PM_COMPWKEN Position */
#define MXC_F_GCR_PM_COMPWKEN                          ((uint32_t)(0x1UL << MXC_F_GCR_PM_COMPWKEN_POS)) /**< PM_COMPWKEN Mask */

#define MXC_F_GCR_PM_HIRCPD_POS                        15 /**< PM_HIRCPD Position */
#define MXC_F_GCR_PM_HIRCPD                            ((uint32_t)(0x1UL << MXC_F_GCR_PM_HIRCPD_POS)) /**< PM_HIRCPD Mask */

#define MXC_F_GCR_PM_HIRC96MPD_POS                     16 /**< PM_HIRC96MPD Position */
#define MXC_F_GCR_PM_HIRC96MPD                         ((uint32_t)(0x1UL << MXC_F_GCR_PM_HIRC96MPD_POS)) /**< PM_HIRC96MPD Mask */

#define MXC_F_GCR_PM_HIRC8MPD_POS                      17 /**< PM_HIRC8MPD Position */
#define MXC_F_GCR_PM_HIRC8MPD                          ((uint32_t)(0x1UL << MXC_F_GCR_PM_HIRC8MPD_POS)) /**< PM_HIRC8MPD Mask */

#define MXC_F_GCR_PM_XTALPB_POS                        20 /**< PM_XTALPB Position */
#define MXC_F_GCR_PM_XTALPB                            ((uint32_t)(0x1UL << MXC_F_GCR_PM_XTALPB_POS)) /**< PM_XTALPB Mask */

/**@} end of group GCR_PM_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_PCKDIV GCR_PCKDIV
 * @brief    Peripheral Clock Divider.
 * @{
 */
#define MXC_F_GCR_PCKDIV_SDHCFRQ_POS                   7 /**< PCKDIV_SDHCFRQ Position */
#define MXC_F_GCR_PCKDIV_SDHCFRQ                       ((uint32_t)(0x1UL << MXC_F_GCR_PCKDIV_SDHCFRQ_POS)) /**< PCKDIV_SDHCFRQ Mask */

#define MXC_F_GCR_PCKDIV_ADCFRQ_POS                    10 /**< PCKDIV_ADCFRQ Position */
#define MXC_F_GCR_PCKDIV_ADCFRQ                        ((uint32_t)(0xFUL << MXC_F_GCR_PCKDIV_ADCFRQ_POS)) /**< PCKDIV_ADCFRQ Mask */

#define MXC_F_GCR_PCKDIV_AONCD_POS                     14 /**< PCKDIV_AONCD Position */
#define MXC_F_GCR_PCKDIV_AONCD                         ((uint32_t)(0x3UL << MXC_F_GCR_PCKDIV_AONCD_POS)) /**< PCKDIV_AONCD Mask */
#define MXC_V_GCR_PCKDIV_AONCD_DIV_4                   ((uint32_t)0x0UL) /**< PCKDIV_AONCD_DIV_4 Value */
#define MXC_S_GCR_PCKDIV_AONCD_DIV_4                   (MXC_V_GCR_PCKDIV_AONCD_DIV_4 << MXC_F_GCR_PCKDIV_AONCD_POS) /**< PCKDIV_AONCD_DIV_4 Setting */
#define MXC_V_GCR_PCKDIV_AONCD_DIV_8                   ((uint32_t)0x1UL) /**< PCKDIV_AONCD_DIV_8 Value */
#define MXC_S_GCR_PCKDIV_AONCD_DIV_8                   (MXC_V_GCR_PCKDIV_AONCD_DIV_8 << MXC_F_GCR_PCKDIV_AONCD_POS) /**< PCKDIV_AONCD_DIV_8 Setting */
#define MXC_V_GCR_PCKDIV_AONCD_DIV_16                  ((uint32_t)0x2UL) /**< PCKDIV_AONCD_DIV_16 Value */
#define MXC_S_GCR_PCKDIV_AONCD_DIV_16                  (MXC_V_GCR_PCKDIV_AONCD_DIV_16 << MXC_F_GCR_PCKDIV_AONCD_POS) /**< PCKDIV_AONCD_DIV_16 Setting */
#define MXC_V_GCR_PCKDIV_AONCD_DIV_32                  ((uint32_t)0x3UL) /**< PCKDIV_AONCD_DIV_32 Value */
#define MXC_S_GCR_PCKDIV_AONCD_DIV_32                  (MXC_V_GCR_PCKDIV_AONCD_DIV_32 << MXC_F_GCR_PCKDIV_AONCD_POS) /**< PCKDIV_AONCD_DIV_32 Setting */

/**@} end of group GCR_PCKDIV_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_PERCKCN0 GCR_PERCKCN0
 * @brief    Peripheral Clock Disable.
 * @{
 */
#define MXC_F_GCR_PERCKCN0_GPIO0D_POS                  0 /**< PERCKCN0_GPIO0D Position */
#define MXC_F_GCR_PERCKCN0_GPIO0D                      ((uint32_t)(0x1UL << MXC_F_GCR_PERCKCN0_GPIO0D_POS)) /**< PERCKCN0_GPIO0D Mask */

#define MXC_F_GCR_PERCKCN0_GPIO1D_POS                  1 /**< PERCKCN0_GPIO1D Position */
#define MXC_F_GCR_PERCKCN0_GPIO1D                      ((uint32_t)(0x1UL << MXC_F_GCR_PERCKCN0_GPIO1D_POS)) /**< PERCKCN0_GPIO1D Mask */

#define MXC_F_GCR_PERCKCN0_USBD_POS                    3 /**< PERCKCN0_USBD Position */
#define MXC_F_GCR_PERCKCN0_USBD                        ((uint32_t)(0x1UL << MXC_F_GCR_PERCKCN0_USBD_POS)) /**< PERCKCN0_USBD Mask */

#define MXC_F_GCR_PERCKCN0_DMAD_POS                    5 /**< PERCKCN0_DMAD Position */
#define MXC_F_GCR_PERCKCN0_DMAD                        ((uint32_t)(0x1UL << MXC_F_GCR_PERCKCN0_DMAD_POS)) /**< PERCKCN0_DMAD Mask */

#define MXC_F_GCR_PERCKCN0_SPI1D_POS                   6 /**< PERCKCN0_SPI1D Position */
#define MXC_F_GCR_PERCKCN0_SPI1D                       ((uint32_t)(0x1UL << MXC_F_GCR_PERCKCN0_SPI1D_POS)) /**< PERCKCN0_SPI1D Mask */

#define MXC_F_GCR_PERCKCN0_SPI2D_POS                   7 /**< PERCKCN0_SPI2D Position */
#define MXC_F_GCR_PERCKCN0_SPI2D                       ((uint32_t)(0x1UL << MXC_F_GCR_PERCKCN0_SPI2D_POS)) /**< PERCKCN0_SPI2D Mask */

#define MXC_F_GCR_PERCKCN0_UART0D_POS                  9 /**< PERCKCN0_UART0D Position */
#define MXC_F_GCR_PERCKCN0_UART0D                      ((uint32_t)(0x1UL << MXC_F_GCR_PERCKCN0_UART0D_POS)) /**< PERCKCN0_UART0D Mask */

#define MXC_F_GCR_PERCKCN0_UART1D_POS                  10 /**< PERCKCN0_UART1D Position */
#define MXC_F_GCR_PERCKCN0_UART1D                      ((uint32_t)(0x1UL << MXC_F_GCR_PERCKCN0_UART1D_POS)) /**< PERCKCN0_UART1D Mask */

#define MXC_F_GCR_PERCKCN0_I2C0D_POS                   13 /**< PERCKCN0_I2C0D Position */
#define MXC_F_GCR_PERCKCN0_I2C0D                       ((uint32_t)(0x1UL << MXC_F_GCR_PERCKCN0_I2C0D_POS)) /**< PERCKCN0_I2C0D Mask */

#define MXC_F_GCR_PERCKCN0_CRYPTOD_POS                 14 /**< PERCKCN0_CRYPTOD Position */
#define MXC_F_GCR_PERCKCN0_CRYPTOD                     ((uint32_t)(0x1UL << MXC_F_GCR_PERCKCN0_CRYPTOD_POS)) /**< PERCKCN0_CRYPTOD Mask */

#define MXC_F_GCR_PERCKCN0_TIMER0D_POS                 15 /**< PERCKCN0_TIMER0D Position */
#define MXC_F_GCR_PERCKCN0_TIMER0D                     ((uint32_t)(0x1UL << MXC_F_GCR_PERCKCN0_TIMER0D_POS)) /**< PERCKCN0_TIMER0D Mask */

#define MXC_F_GCR_PERCKCN0_TIMER1D_POS                 16 /**< PERCKCN0_TIMER1D Position */
#define MXC_F_GCR_PERCKCN0_TIMER1D                     ((uint32_t)(0x1UL << MXC_F_GCR_PERCKCN0_TIMER1D_POS)) /**< PERCKCN0_TIMER1D Mask */

#define MXC_F_GCR_PERCKCN0_TIMER2D_POS                 17 /**< PERCKCN0_TIMER2D Position */
#define MXC_F_GCR_PERCKCN0_TIMER2D                     ((uint32_t)(0x1UL << MXC_F_GCR_PERCKCN0_TIMER2D_POS)) /**< PERCKCN0_TIMER2D Mask */

#define MXC_F_GCR_PERCKCN0_TIMER3D_POS                 18 /**< PERCKCN0_TIMER3D Position */
#define MXC_F_GCR_PERCKCN0_TIMER3D                     ((uint32_t)(0x1UL << MXC_F_GCR_PERCKCN0_TIMER3D_POS)) /**< PERCKCN0_TIMER3D Mask */

#define MXC_F_GCR_PERCKCN0_TIMER4D_POS                 19 /**< PERCKCN0_TIMER4D Position */
#define MXC_F_GCR_PERCKCN0_TIMER4D                     ((uint32_t)(0x1UL << MXC_F_GCR_PERCKCN0_TIMER4D_POS)) /**< PERCKCN0_TIMER4D Mask */

#define MXC_F_GCR_PERCKCN0_TIMER5D_POS                 20 /**< PERCKCN0_TIMER5D Position */
#define MXC_F_GCR_PERCKCN0_TIMER5D                     ((uint32_t)(0x1UL << MXC_F_GCR_PERCKCN0_TIMER5D_POS)) /**< PERCKCN0_TIMER5D Mask */

#define MXC_F_GCR_PERCKCN0_ADCD_POS                    23 /**< PERCKCN0_ADCD Position */
#define MXC_F_GCR_PERCKCN0_ADCD                        ((uint32_t)(0x1UL << MXC_F_GCR_PERCKCN0_ADCD_POS)) /**< PERCKCN0_ADCD Mask */

#define MXC_F_GCR_PERCKCN0_I2C1D_POS                   28 /**< PERCKCN0_I2C1D Position */
#define MXC_F_GCR_PERCKCN0_I2C1D                       ((uint32_t)(0x1UL << MXC_F_GCR_PERCKCN0_I2C1D_POS)) /**< PERCKCN0_I2C1D Mask */

#define MXC_F_GCR_PERCKCN0_PTD_POS                     29 /**< PERCKCN0_PTD Position */
#define MXC_F_GCR_PERCKCN0_PTD                         ((uint32_t)(0x1UL << MXC_F_GCR_PERCKCN0_PTD_POS)) /**< PERCKCN0_PTD Mask */

#define MXC_F_GCR_PERCKCN0_SPIXIPD_POS                 30 /**< PERCKCN0_SPIXIPD Position */
#define MXC_F_GCR_PERCKCN0_SPIXIPD                     ((uint32_t)(0x1UL << MXC_F_GCR_PERCKCN0_SPIXIPD_POS)) /**< PERCKCN0_SPIXIPD Mask */

#define MXC_F_GCR_PERCKCN0_SPIMD_POS                   31 /**< PERCKCN0_SPIMD Position */
#define MXC_F_GCR_PERCKCN0_SPIMD                       ((uint32_t)(0x1UL << MXC_F_GCR_PERCKCN0_SPIMD_POS)) /**< PERCKCN0_SPIMD Mask */

/**@} end of group GCR_PERCKCN0_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_MEMCKCN GCR_MEMCKCN
 * @brief    Memory Clock Control Register.
 * @{
 */
#define MXC_F_GCR_MEMCKCN_FWS_POS                      0 /**< MEMCKCN_FWS Position */
#define MXC_F_GCR_MEMCKCN_FWS                          ((uint32_t)(0x7UL << MXC_F_GCR_MEMCKCN_FWS_POS)) /**< MEMCKCN_FWS Mask */

#define MXC_F_GCR_MEMCKCN_SYSRAM0LS_POS                16 /**< MEMCKCN_SYSRAM0LS Position */
#define MXC_F_GCR_MEMCKCN_SYSRAM0LS                    ((uint32_t)(0x1UL << MXC_F_GCR_MEMCKCN_SYSRAM0LS_POS)) /**< MEMCKCN_SYSRAM0LS Mask */

#define MXC_F_GCR_MEMCKCN_SYSRAM1LS_POS                17 /**< MEMCKCN_SYSRAM1LS Position */
#define MXC_F_GCR_MEMCKCN_SYSRAM1LS                    ((uint32_t)(0x1UL << MXC_F_GCR_MEMCKCN_SYSRAM1LS_POS)) /**< MEMCKCN_SYSRAM1LS Mask */

#define MXC_F_GCR_MEMCKCN_SYSRAM2LS_POS                18 /**< MEMCKCN_SYSRAM2LS Position */
#define MXC_F_GCR_MEMCKCN_SYSRAM2LS                    ((uint32_t)(0x1UL << MXC_F_GCR_MEMCKCN_SYSRAM2LS_POS)) /**< MEMCKCN_SYSRAM2LS Mask */

#define MXC_F_GCR_MEMCKCN_SYSRAM3LS_POS                19 /**< MEMCKCN_SYSRAM3LS Position */
#define MXC_F_GCR_MEMCKCN_SYSRAM3LS                    ((uint32_t)(0x1UL << MXC_F_GCR_MEMCKCN_SYSRAM3LS_POS)) /**< MEMCKCN_SYSRAM3LS Mask */

#define MXC_F_GCR_MEMCKCN_SYSRAM4LS_POS                20 /**< MEMCKCN_SYSRAM4LS Position */
#define MXC_F_GCR_MEMCKCN_SYSRAM4LS                    ((uint32_t)(0x1UL << MXC_F_GCR_MEMCKCN_SYSRAM4LS_POS)) /**< MEMCKCN_SYSRAM4LS Mask */

#define MXC_F_GCR_MEMCKCN_SYSRAM5LS_POS                21 /**< MEMCKCN_SYSRAM5LS Position */
#define MXC_F_GCR_MEMCKCN_SYSRAM5LS                    ((uint32_t)(0x1UL << MXC_F_GCR_MEMCKCN_SYSRAM5LS_POS)) /**< MEMCKCN_SYSRAM5LS Mask */

#define MXC_F_GCR_MEMCKCN_SYSRAM6LS_POS                22 /**< MEMCKCN_SYSRAM6LS Position */
#define MXC_F_GCR_MEMCKCN_SYSRAM6LS                    ((uint32_t)(0x1UL << MXC_F_GCR_MEMCKCN_SYSRAM6LS_POS)) /**< MEMCKCN_SYSRAM6LS Mask */

#define MXC_F_GCR_MEMCKCN_ICACHELS_POS                 24 /**< MEMCKCN_ICACHELS Position */
#define MXC_F_GCR_MEMCKCN_ICACHELS                     ((uint32_t)(0x1UL << MXC_F_GCR_MEMCKCN_ICACHELS_POS)) /**< MEMCKCN_ICACHELS Mask */

#define MXC_F_GCR_MEMCKCN_ICACHEXIPLS_POS              25 /**< MEMCKCN_ICACHEXIPLS Position */
#define MXC_F_GCR_MEMCKCN_ICACHEXIPLS                  ((uint32_t)(0x1UL << MXC_F_GCR_MEMCKCN_ICACHEXIPLS_POS)) /**< MEMCKCN_ICACHEXIPLS Mask */

#define MXC_F_GCR_MEMCKCN_SCACHELS_POS                 26 /**< MEMCKCN_SCACHELS Position */
#define MXC_F_GCR_MEMCKCN_SCACHELS                     ((uint32_t)(0x1UL << MXC_F_GCR_MEMCKCN_SCACHELS_POS)) /**< MEMCKCN_SCACHELS Mask */

#define MXC_F_GCR_MEMCKCN_CRYPTOLS_POS                 27 /**< MEMCKCN_CRYPTOLS Position */
#define MXC_F_GCR_MEMCKCN_CRYPTOLS                     ((uint32_t)(0x1UL << MXC_F_GCR_MEMCKCN_CRYPTOLS_POS)) /**< MEMCKCN_CRYPTOLS Mask */

#define MXC_F_GCR_MEMCKCN_USBLS_POS                    28 /**< MEMCKCN_USBLS Position */
#define MXC_F_GCR_MEMCKCN_USBLS                        ((uint32_t)(0x1UL << MXC_F_GCR_MEMCKCN_USBLS_POS)) /**< MEMCKCN_USBLS Mask */

#define MXC_F_GCR_MEMCKCN_ROM0LS_POS                   29 /**< MEMCKCN_ROM0LS Position */
#define MXC_F_GCR_MEMCKCN_ROM0LS                       ((uint32_t)(0x1UL << MXC_F_GCR_MEMCKCN_ROM0LS_POS)) /**< MEMCKCN_ROM0LS Mask */

#define MXC_F_GCR_MEMCKCN_ROM1LS_POS                   30 /**< MEMCKCN_ROM1LS Position */
#define MXC_F_GCR_MEMCKCN_ROM1LS                       ((uint32_t)(0x1UL << MXC_F_GCR_MEMCKCN_ROM1LS_POS)) /**< MEMCKCN_ROM1LS Mask */

#define MXC_F_GCR_MEMCKCN_ICACHE1LS_POS                31 /**< MEMCKCN_ICACHE1LS Position */
#define MXC_F_GCR_MEMCKCN_ICACHE1LS                    ((uint32_t)(0x1UL << MXC_F_GCR_MEMCKCN_ICACHE1LS_POS)) /**< MEMCKCN_ICACHE1LS Mask */

/**@} end of group GCR_MEMCKCN_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_MEMZCN GCR_MEMZCN
 * @brief    Memory Zeroize Control.
 * @{
 */
#define MXC_F_GCR_MEMZCN_SRAM0Z_POS                    0 /**< MEMZCN_SRAM0Z Position */
#define MXC_F_GCR_MEMZCN_SRAM0Z                        ((uint32_t)(0x1UL << MXC_F_GCR_MEMZCN_SRAM0Z_POS)) /**< MEMZCN_SRAM0Z Mask */

#define MXC_F_GCR_MEMZCN_SRAM1Z_POS                    1 /**< MEMZCN_SRAM1Z Position */
#define MXC_F_GCR_MEMZCN_SRAM1Z                        ((uint32_t)(0x1UL << MXC_F_GCR_MEMZCN_SRAM1Z_POS)) /**< MEMZCN_SRAM1Z Mask */

#define MXC_F_GCR_MEMZCN_SRAM2_POS                     2 /**< MEMZCN_SRAM2 Position */
#define MXC_F_GCR_MEMZCN_SRAM2                         ((uint32_t)(0x1UL << MXC_F_GCR_MEMZCN_SRAM2_POS)) /**< MEMZCN_SRAM2 Mask */

#define MXC_F_GCR_MEMZCN_SRAM3Z_POS                    3 /**< MEMZCN_SRAM3Z Position */
#define MXC_F_GCR_MEMZCN_SRAM3Z                        ((uint32_t)(0x1UL << MXC_F_GCR_MEMZCN_SRAM3Z_POS)) /**< MEMZCN_SRAM3Z Mask */

#define MXC_F_GCR_MEMZCN_SRAM4Z_POS                    4 /**< MEMZCN_SRAM4Z Position */
#define MXC_F_GCR_MEMZCN_SRAM4Z                        ((uint32_t)(0x1UL << MXC_F_GCR_MEMZCN_SRAM4Z_POS)) /**< MEMZCN_SRAM4Z Mask */

#define MXC_F_GCR_MEMZCN_SRAM5Z_POS                    5 /**< MEMZCN_SRAM5Z Position */
#define MXC_F_GCR_MEMZCN_SRAM5Z                        ((uint32_t)(0x1UL << MXC_F_GCR_MEMZCN_SRAM5Z_POS)) /**< MEMZCN_SRAM5Z Mask */

#define MXC_F_GCR_MEMZCN_SRAM6Z_POS                    6 /**< MEMZCN_SRAM6Z Position */
#define MXC_F_GCR_MEMZCN_SRAM6Z                        ((uint32_t)(0x1UL << MXC_F_GCR_MEMZCN_SRAM6Z_POS)) /**< MEMZCN_SRAM6Z Mask */

#define MXC_F_GCR_MEMZCN_ICACHEZ_POS                   8 /**< MEMZCN_ICACHEZ Position */
#define MXC_F_GCR_MEMZCN_ICACHEZ                       ((uint32_t)(0x1UL << MXC_F_GCR_MEMZCN_ICACHEZ_POS)) /**< MEMZCN_ICACHEZ Mask */

#define MXC_F_GCR_MEMZCN_ICACHEXIPZ_POS                9 /**< MEMZCN_ICACHEXIPZ Position */
#define MXC_F_GCR_MEMZCN_ICACHEXIPZ                    ((uint32_t)(0x1UL << MXC_F_GCR_MEMZCN_ICACHEXIPZ_POS)) /**< MEMZCN_ICACHEXIPZ Mask */

#define MXC_F_GCR_MEMZCN_SCACHEDATAZ_POS               10 /**< MEMZCN_SCACHEDATAZ Position */
#define MXC_F_GCR_MEMZCN_SCACHEDATAZ                   ((uint32_t)(0x1UL << MXC_F_GCR_MEMZCN_SCACHEDATAZ_POS)) /**< MEMZCN_SCACHEDATAZ Mask */

#define MXC_F_GCR_MEMZCN_SCACHETAGZ_POS                11 /**< MEMZCN_SCACHETAGZ Position */
#define MXC_F_GCR_MEMZCN_SCACHETAGZ                    ((uint32_t)(0x1UL << MXC_F_GCR_MEMZCN_SCACHETAGZ_POS)) /**< MEMZCN_SCACHETAGZ Mask */

#define MXC_F_GCR_MEMZCN_CRYPTOZ_POS                   12 /**< MEMZCN_CRYPTOZ Position */
#define MXC_F_GCR_MEMZCN_CRYPTOZ                       ((uint32_t)(0x1UL << MXC_F_GCR_MEMZCN_CRYPTOZ_POS)) /**< MEMZCN_CRYPTOZ Mask */

#define MXC_F_GCR_MEMZCN_USBFIFOZ_POS                  13 /**< MEMZCN_USBFIFOZ Position */
#define MXC_F_GCR_MEMZCN_USBFIFOZ                      ((uint32_t)(0x1UL << MXC_F_GCR_MEMZCN_USBFIFOZ_POS)) /**< MEMZCN_USBFIFOZ Mask */

#define MXC_F_GCR_MEMZCN_ICACHE1Z_POS                  14 /**< MEMZCN_ICACHE1Z Position */
#define MXC_F_GCR_MEMZCN_ICACHE1Z                      ((uint32_t)(0x1UL << MXC_F_GCR_MEMZCN_ICACHE1Z_POS)) /**< MEMZCN_ICACHE1Z Mask */

/**@} end of group GCR_MEMZCN_Register */

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
 * @defgroup GCR_RSTR1 GCR_RSTR1
 * @brief    Reset 1.
 * @{
 */
#define MXC_F_GCR_RSTR1_I2C1_POS                       0 /**< RSTR1_I2C1 Position */
#define MXC_F_GCR_RSTR1_I2C1                           ((uint32_t)(0x1UL << MXC_F_GCR_RSTR1_I2C1_POS)) /**< RSTR1_I2C1 Mask */

#define MXC_F_GCR_RSTR1_PT_POS                         1 /**< RSTR1_PT Position */
#define MXC_F_GCR_RSTR1_PT                             ((uint32_t)(0x1UL << MXC_F_GCR_RSTR1_PT_POS)) /**< RSTR1_PT Mask */

#define MXC_F_GCR_RSTR1_SPIXIP_POS                     3 /**< RSTR1_SPIXIP Position */
#define MXC_F_GCR_RSTR1_SPIXIP                         ((uint32_t)(0x1UL << MXC_F_GCR_RSTR1_SPIXIP_POS)) /**< RSTR1_SPIXIP Mask */

#define MXC_F_GCR_RSTR1_XSPIM_POS                      4 /**< RSTR1_XSPIM Position */
#define MXC_F_GCR_RSTR1_XSPIM                          ((uint32_t)(0x1UL << MXC_F_GCR_RSTR1_XSPIM_POS)) /**< RSTR1_XSPIM Mask */

#define MXC_F_GCR_RSTR1_SDHC_POS                       6 /**< RSTR1_SDHC Position */
#define MXC_F_GCR_RSTR1_SDHC                           ((uint32_t)(0x1UL << MXC_F_GCR_RSTR1_SDHC_POS)) /**< RSTR1_SDHC Mask */

#define MXC_F_GCR_RSTR1_OWIRE_POS                      7 /**< RSTR1_OWIRE Position */
#define MXC_F_GCR_RSTR1_OWIRE                          ((uint32_t)(0x1UL << MXC_F_GCR_RSTR1_OWIRE_POS)) /**< RSTR1_OWIRE Mask */

#define MXC_F_GCR_RSTR1_WDT1_POS                       8 /**< RSTR1_WDT1 Position */
#define MXC_F_GCR_RSTR1_WDT1                           ((uint32_t)(0x1UL << MXC_F_GCR_RSTR1_WDT1_POS)) /**< RSTR1_WDT1 Mask */

#define MXC_F_GCR_RSTR1_SPI0_POS                       9 /**< RSTR1_SPI0 Position */
#define MXC_F_GCR_RSTR1_SPI0                           ((uint32_t)(0x1UL << MXC_F_GCR_RSTR1_SPI0_POS)) /**< RSTR1_SPI0 Mask */

#define MXC_F_GCR_RSTR1_SPIXMEM_POS                    15 /**< RSTR1_SPIXMEM Position */
#define MXC_F_GCR_RSTR1_SPIXMEM                        ((uint32_t)(0x1UL << MXC_F_GCR_RSTR1_SPIXMEM_POS)) /**< RSTR1_SPIXMEM Mask */

#define MXC_F_GCR_RSTR1_SMPHR_POS                      16 /**< RSTR1_SMPHR Position */
#define MXC_F_GCR_RSTR1_SMPHR                          ((uint32_t)(0x1UL << MXC_F_GCR_RSTR1_SMPHR_POS)) /**< RSTR1_SMPHR Mask */

#define MXC_F_GCR_RSTR1_WDT2_POS                       17 /**< RSTR1_WDT2 Position */
#define MXC_F_GCR_RSTR1_WDT2                           ((uint32_t)(0x1UL << MXC_F_GCR_RSTR1_WDT2_POS)) /**< RSTR1_WDT2 Mask */

#define MXC_F_GCR_RSTR1_BTLE_POS                       18 /**< RSTR1_BTLE Position */
#define MXC_F_GCR_RSTR1_BTLE                           ((uint32_t)(0x1UL << MXC_F_GCR_RSTR1_BTLE_POS)) /**< RSTR1_BTLE Mask */

#define MXC_F_GCR_RSTR1_AUDIO_POS                      19 /**< RSTR1_AUDIO Position */
#define MXC_F_GCR_RSTR1_AUDIO                          ((uint32_t)(0x1UL << MXC_F_GCR_RSTR1_AUDIO_POS)) /**< RSTR1_AUDIO Mask */

#define MXC_F_GCR_RSTR1_I2C2_POS                       20 /**< RSTR1_I2C2 Position */
#define MXC_F_GCR_RSTR1_I2C2                           ((uint32_t)(0x1UL << MXC_F_GCR_RSTR1_I2C2_POS)) /**< RSTR1_I2C2 Mask */

#define MXC_F_GCR_RSTR1_RPU_POS                        21 /**< RSTR1_RPU Position */
#define MXC_F_GCR_RSTR1_RPU                            ((uint32_t)(0x1UL << MXC_F_GCR_RSTR1_RPU_POS)) /**< RSTR1_RPU Mask */

#define MXC_F_GCR_RSTR1_HTMR0_POS                      22 /**< RSTR1_HTMR0 Position */
#define MXC_F_GCR_RSTR1_HTMR0                          ((uint32_t)(0x1UL << MXC_F_GCR_RSTR1_HTMR0_POS)) /**< RSTR1_HTMR0 Mask */

#define MXC_F_GCR_RSTR1_HTMR1_POS                      23 /**< RSTR1_HTMR1 Position */
#define MXC_F_GCR_RSTR1_HTMR1                          ((uint32_t)(0x1UL << MXC_F_GCR_RSTR1_HTMR1_POS)) /**< RSTR1_HTMR1 Mask */

#define MXC_F_GCR_RSTR1_DVS_POS                        24 /**< RSTR1_DVS Position */
#define MXC_F_GCR_RSTR1_DVS                            ((uint32_t)(0x1UL << MXC_F_GCR_RSTR1_DVS_POS)) /**< RSTR1_DVS Mask */

#define MXC_F_GCR_RSTR1_SIMO_POS                       25 /**< RSTR1_SIMO Position */
#define MXC_F_GCR_RSTR1_SIMO                           ((uint32_t)(0x1UL << MXC_F_GCR_RSTR1_SIMO_POS)) /**< RSTR1_SIMO Mask */

/**@} end of group GCR_RSTR1_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_PERCKCN1 GCR_PERCKCN1
 * @brief    Peripheral Clock Disable.
 * @{
 */
#define MXC_F_GCR_PERCKCN1_BTLED_POS                   0 /**< PERCKCN1_BTLED Position */
#define MXC_F_GCR_PERCKCN1_BTLED                       ((uint32_t)(0x1UL << MXC_F_GCR_PERCKCN1_BTLED_POS)) /**< PERCKCN1_BTLED Mask */

#define MXC_F_GCR_PERCKCN1_UART2D_POS                  1 /**< PERCKCN1_UART2D Position */
#define MXC_F_GCR_PERCKCN1_UART2D                      ((uint32_t)(0x1UL << MXC_F_GCR_PERCKCN1_UART2D_POS)) /**< PERCKCN1_UART2D Mask */

#define MXC_F_GCR_PERCKCN1_TRNGD_POS                   2 /**< PERCKCN1_TRNGD Position */
#define MXC_F_GCR_PERCKCN1_TRNGD                       ((uint32_t)(0x1UL << MXC_F_GCR_PERCKCN1_TRNGD_POS)) /**< PERCKCN1_TRNGD Mask */

#define MXC_F_GCR_PERCKCN1_SCACHED_POS                 7 /**< PERCKCN1_SCACHED Position */
#define MXC_F_GCR_PERCKCN1_SCACHED                     ((uint32_t)(0x1UL << MXC_F_GCR_PERCKCN1_SCACHED_POS)) /**< PERCKCN1_SCACHED Mask */

#define MXC_F_GCR_PERCKCN1_SDMAD_POS                   8 /**< PERCKCN1_SDMAD Position */
#define MXC_F_GCR_PERCKCN1_SDMAD                       ((uint32_t)(0x1UL << MXC_F_GCR_PERCKCN1_SDMAD_POS)) /**< PERCKCN1_SDMAD Mask */

#define MXC_F_GCR_PERCKCN1_SMPHRD_POS                  9 /**< PERCKCN1_SMPHRD Position */
#define MXC_F_GCR_PERCKCN1_SMPHRD                      ((uint32_t)(0x1UL << MXC_F_GCR_PERCKCN1_SMPHRD_POS)) /**< PERCKCN1_SMPHRD Mask */

#define MXC_F_GCR_PERCKCN1_SDHCD_POS                   10 /**< PERCKCN1_SDHCD Position */
#define MXC_F_GCR_PERCKCN1_SDHCD                       ((uint32_t)(0x1UL << MXC_F_GCR_PERCKCN1_SDHCD_POS)) /**< PERCKCN1_SDHCD Mask */

#define MXC_F_GCR_PERCKCN1_ICACHEXIPD_POS              12 /**< PERCKCN1_ICACHEXIPD Position */
#define MXC_F_GCR_PERCKCN1_ICACHEXIPD                  ((uint32_t)(0x1UL << MXC_F_GCR_PERCKCN1_ICACHEXIPD_POS)) /**< PERCKCN1_ICACHEXIPD Mask */

#define MXC_F_GCR_PERCKCN1_OWIRED_POS                  13 /**< PERCKCN1_OWIRED Position */
#define MXC_F_GCR_PERCKCN1_OWIRED                      ((uint32_t)(0x1UL << MXC_F_GCR_PERCKCN1_OWIRED_POS)) /**< PERCKCN1_OWIRED Mask */

#define MXC_F_GCR_PERCKCN1_SPI0D_POS                   14 /**< PERCKCN1_SPI0D Position */
#define MXC_F_GCR_PERCKCN1_SPI0D                       ((uint32_t)(0x1UL << MXC_F_GCR_PERCKCN1_SPI0D_POS)) /**< PERCKCN1_SPI0D Mask */

#define MXC_F_GCR_PERCKCN1_SPIXIPDD_POS                20 /**< PERCKCN1_SPIXIPDD Position */
#define MXC_F_GCR_PERCKCN1_SPIXIPDD                    ((uint32_t)(0x1UL << MXC_F_GCR_PERCKCN1_SPIXIPDD_POS)) /**< PERCKCN1_SPIXIPDD Mask */

#define MXC_F_GCR_PERCKCN1_DMA1D_POS                   21 /**< PERCKCN1_DMA1D Position */
#define MXC_F_GCR_PERCKCN1_DMA1D                       ((uint32_t)(0x1UL << MXC_F_GCR_PERCKCN1_DMA1D_POS)) /**< PERCKCN1_DMA1D Mask */

#define MXC_F_GCR_PERCKCN1_AUDIOD_POS                  23 /**< PERCKCN1_AUDIOD Position */
#define MXC_F_GCR_PERCKCN1_AUDIOD                      ((uint32_t)(0x1UL << MXC_F_GCR_PERCKCN1_AUDIOD_POS)) /**< PERCKCN1_AUDIOD Mask */

#define MXC_F_GCR_PERCKCN1_I2C2D_POS                   24 /**< PERCKCN1_I2C2D Position */
#define MXC_F_GCR_PERCKCN1_I2C2D                       ((uint32_t)(0x1UL << MXC_F_GCR_PERCKCN1_I2C2D_POS)) /**< PERCKCN1_I2C2D Mask */

#define MXC_F_GCR_PERCKCN1_HTMR0D_POS                  25 /**< PERCKCN1_HTMR0D Position */
#define MXC_F_GCR_PERCKCN1_HTMR0D                      ((uint32_t)(0x1UL << MXC_F_GCR_PERCKCN1_HTMR0D_POS)) /**< PERCKCN1_HTMR0D Mask */

#define MXC_F_GCR_PERCKCN1_HTMR1D_POS                  26 /**< PERCKCN1_HTMR1D Position */
#define MXC_F_GCR_PERCKCN1_HTMR1D                      ((uint32_t)(0x1UL << MXC_F_GCR_PERCKCN1_HTMR1D_POS)) /**< PERCKCN1_HTMR1D Mask */

#define MXC_F_GCR_PERCKCN1_WDT0D_POS                   27 /**< PERCKCN1_WDT0D Position */
#define MXC_F_GCR_PERCKCN1_WDT0D                       ((uint32_t)(0x1UL << MXC_F_GCR_PERCKCN1_WDT0D_POS)) /**< PERCKCN1_WDT0D Mask */

#define MXC_F_GCR_PERCKCN1_WDT1D_POS                   28 /**< PERCKCN1_WDT1D Position */
#define MXC_F_GCR_PERCKCN1_WDT1D                       ((uint32_t)(0x1UL << MXC_F_GCR_PERCKCN1_WDT1D_POS)) /**< PERCKCN1_WDT1D Mask */

#define MXC_F_GCR_PERCKCN1_WDT2D_POS                   29 /**< PERCKCN1_WDT2D Position */
#define MXC_F_GCR_PERCKCN1_WDT2D                       ((uint32_t)(0x1UL << MXC_F_GCR_PERCKCN1_WDT2D_POS)) /**< PERCKCN1_WDT2D Mask */

#define MXC_F_GCR_PERCKCN1_CPU1D_POS                   31 /**< PERCKCN1_CPU1D Position */
#define MXC_F_GCR_PERCKCN1_CPU1D                       ((uint32_t)(0x1UL << MXC_F_GCR_PERCKCN1_CPU1D_POS)) /**< PERCKCN1_CPU1D Mask */

/**@} end of group GCR_PERCKCN1_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_EVENT_EN GCR_EVENT_EN
 * @brief    Event Enable Register.
 * @{
 */
#define MXC_F_GCR_EVENT_EN_CPU0DMAEVENT_POS            0 /**< EVENT_EN_CPU0DMAEVENT Position */
#define MXC_F_GCR_EVENT_EN_CPU0DMAEVENT                ((uint32_t)(0x1UL << MXC_F_GCR_EVENT_EN_CPU0DMAEVENT_POS)) /**< EVENT_EN_CPU0DMAEVENT Mask */

#define MXC_F_GCR_EVENT_EN_CPU0DMA1EVENT_POS           1 /**< EVENT_EN_CPU0DMA1EVENT Position */
#define MXC_F_GCR_EVENT_EN_CPU0DMA1EVENT               ((uint32_t)(0x1UL << MXC_F_GCR_EVENT_EN_CPU0DMA1EVENT_POS)) /**< EVENT_EN_CPU0DMA1EVENT Mask */

#define MXC_F_GCR_EVENT_EN_CPU0TXEVENT_POS             2 /**< EVENT_EN_CPU0TXEVENT Position */
#define MXC_F_GCR_EVENT_EN_CPU0TXEVENT                 ((uint32_t)(0x1UL << MXC_F_GCR_EVENT_EN_CPU0TXEVENT_POS)) /**< EVENT_EN_CPU0TXEVENT Mask */

#define MXC_F_GCR_EVENT_EN_CPU1DMAEVENT_POS            3 /**< EVENT_EN_CPU1DMAEVENT Position */
#define MXC_F_GCR_EVENT_EN_CPU1DMAEVENT                ((uint32_t)(0x1UL << MXC_F_GCR_EVENT_EN_CPU1DMAEVENT_POS)) /**< EVENT_EN_CPU1DMAEVENT Mask */

#define MXC_F_GCR_EVENT_EN_CPU1DMA1EVENT_POS           4 /**< EVENT_EN_CPU1DMA1EVENT Position */
#define MXC_F_GCR_EVENT_EN_CPU1DMA1EVENT               ((uint32_t)(0x1UL << MXC_F_GCR_EVENT_EN_CPU1DMA1EVENT_POS)) /**< EVENT_EN_CPU1DMA1EVENT Mask */

#define MXC_F_GCR_EVENT_EN_CPU1TXEVENT_POS             5 /**< EVENT_EN_CPU1TXEVENT Position */
#define MXC_F_GCR_EVENT_EN_CPU1TXEVENT                 ((uint32_t)(0x1UL << MXC_F_GCR_EVENT_EN_CPU1TXEVENT_POS)) /**< EVENT_EN_CPU1TXEVENT Mask */

/**@} end of group GCR_EVENT_EN_Register */

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
 * @defgroup GCR_SYSSIE GCR_SYSSIE
 * @brief    System Status Interrupt Enable Register.
 * @{
 */
#define MXC_F_GCR_SYSSIE_ICEULIE_POS                   0 /**< SYSSIE_ICEULIE Position */
#define MXC_F_GCR_SYSSIE_ICEULIE                       ((uint32_t)(0x1UL << MXC_F_GCR_SYSSIE_ICEULIE_POS)) /**< SYSSIE_ICEULIE Mask */

#define MXC_F_GCR_SYSSIE_CIEIE_POS                     1 /**< SYSSIE_CIEIE Position */
#define MXC_F_GCR_SYSSIE_CIEIE                         ((uint32_t)(0x1UL << MXC_F_GCR_SYSSIE_CIEIE_POS)) /**< SYSSIE_CIEIE Mask */

#define MXC_F_GCR_SYSSIE_SCMFIE_POS                    5 /**< SYSSIE_SCMFIE Position */
#define MXC_F_GCR_SYSSIE_SCMFIE                        ((uint32_t)(0x1UL << MXC_F_GCR_SYSSIE_SCMFIE_POS)) /**< SYSSIE_SCMFIE Mask */

/**@} end of group GCR_SYSSIE_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_ECC_ER GCR_ECC_ER
 * @brief    ECC Error Register
 * @{
 */
#define MXC_F_GCR_ECC_ER_SYSRAM0ECCERR_POS             0 /**< ECC_ER_SYSRAM0ECCERR Position */
#define MXC_F_GCR_ECC_ER_SYSRAM0ECCERR                 ((uint32_t)(0x1UL << MXC_F_GCR_ECC_ER_SYSRAM0ECCERR_POS)) /**< ECC_ER_SYSRAM0ECCERR Mask */

#define MXC_F_GCR_ECC_ER_SYSRAM1ECCERR_POS             1 /**< ECC_ER_SYSRAM1ECCERR Position */
#define MXC_F_GCR_ECC_ER_SYSRAM1ECCERR                 ((uint32_t)(0x1UL << MXC_F_GCR_ECC_ER_SYSRAM1ECCERR_POS)) /**< ECC_ER_SYSRAM1ECCERR Mask */

#define MXC_F_GCR_ECC_ER_SYSRAM2ECCERR_POS             2 /**< ECC_ER_SYSRAM2ECCERR Position */
#define MXC_F_GCR_ECC_ER_SYSRAM2ECCERR                 ((uint32_t)(0x1UL << MXC_F_GCR_ECC_ER_SYSRAM2ECCERR_POS)) /**< ECC_ER_SYSRAM2ECCERR Mask */

#define MXC_F_GCR_ECC_ER_SYSRAM3ECCERR_POS             3 /**< ECC_ER_SYSRAM3ECCERR Position */
#define MXC_F_GCR_ECC_ER_SYSRAM3ECCERR                 ((uint32_t)(0x1UL << MXC_F_GCR_ECC_ER_SYSRAM3ECCERR_POS)) /**< ECC_ER_SYSRAM3ECCERR Mask */

#define MXC_F_GCR_ECC_ER_SYSRAM4ECCERR_POS             4 /**< ECC_ER_SYSRAM4ECCERR Position */
#define MXC_F_GCR_ECC_ER_SYSRAM4ECCERR                 ((uint32_t)(0x1UL << MXC_F_GCR_ECC_ER_SYSRAM4ECCERR_POS)) /**< ECC_ER_SYSRAM4ECCERR Mask */

#define MXC_F_GCR_ECC_ER_SYSRAM5ECCERR_POS             5 /**< ECC_ER_SYSRAM5ECCERR Position */
#define MXC_F_GCR_ECC_ER_SYSRAM5ECCERR                 ((uint32_t)(0x1UL << MXC_F_GCR_ECC_ER_SYSRAM5ECCERR_POS)) /**< ECC_ER_SYSRAM5ECCERR Mask */

#define MXC_F_GCR_ECC_ER_SYSRAM6ECCERR_POS             6 /**< ECC_ER_SYSRAM6ECCERR Position */
#define MXC_F_GCR_ECC_ER_SYSRAM6ECCERR                 ((uint32_t)(0x1UL << MXC_F_GCR_ECC_ER_SYSRAM6ECCERR_POS)) /**< ECC_ER_SYSRAM6ECCERR Mask */

#define MXC_F_GCR_ECC_ER_IC0ECCERR_POS                 8 /**< ECC_ER_IC0ECCERR Position */
#define MXC_F_GCR_ECC_ER_IC0ECCERR                     ((uint32_t)(0x1UL << MXC_F_GCR_ECC_ER_IC0ECCERR_POS)) /**< ECC_ER_IC0ECCERR Mask */

#define MXC_F_GCR_ECC_ER_IC1ECCERR_POS                 9 /**< ECC_ER_IC1ECCERR Position */
#define MXC_F_GCR_ECC_ER_IC1ECCERR                     ((uint32_t)(0x1UL << MXC_F_GCR_ECC_ER_IC1ECCERR_POS)) /**< ECC_ER_IC1ECCERR Mask */

#define MXC_F_GCR_ECC_ER_ICXIPECCERR_POS               10 /**< ECC_ER_ICXIPECCERR Position */
#define MXC_F_GCR_ECC_ER_ICXIPECCERR                   ((uint32_t)(0x1UL << MXC_F_GCR_ECC_ER_ICXIPECCERR_POS)) /**< ECC_ER_ICXIPECCERR Mask */

#define MXC_F_GCR_ECC_ER_FL0ECCERR_POS                 11 /**< ECC_ER_FL0ECCERR Position */
#define MXC_F_GCR_ECC_ER_FL0ECCERR                     ((uint32_t)(0x1UL << MXC_F_GCR_ECC_ER_FL0ECCERR_POS)) /**< ECC_ER_FL0ECCERR Mask */

#define MXC_F_GCR_ECC_ER_FL1ECCERR_POS                 12 /**< ECC_ER_FL1ECCERR Position */
#define MXC_F_GCR_ECC_ER_FL1ECCERR                     ((uint32_t)(0x1UL << MXC_F_GCR_ECC_ER_FL1ECCERR_POS)) /**< ECC_ER_FL1ECCERR Mask */

/**@} end of group GCR_ECC_ER_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_ECC_CED GCR_ECC_CED
 * @brief    ECC Correctable Error Detected Register
 * @{
 */
#define MXC_F_GCR_ECC_CED_SYSRAM0ECCNDED_POS           0 /**< ECC_CED_SYSRAM0ECCNDED Position */
#define MXC_F_GCR_ECC_CED_SYSRAM0ECCNDED               ((uint32_t)(0x1UL << MXC_F_GCR_ECC_CED_SYSRAM0ECCNDED_POS)) /**< ECC_CED_SYSRAM0ECCNDED Mask */

#define MXC_F_GCR_ECC_CED_SYSRAM1ECCNDED_POS           1 /**< ECC_CED_SYSRAM1ECCNDED Position */
#define MXC_F_GCR_ECC_CED_SYSRAM1ECCNDED               ((uint32_t)(0x1UL << MXC_F_GCR_ECC_CED_SYSRAM1ECCNDED_POS)) /**< ECC_CED_SYSRAM1ECCNDED Mask */

#define MXC_F_GCR_ECC_CED_SYSRAM2ECCNDED_POS           2 /**< ECC_CED_SYSRAM2ECCNDED Position */
#define MXC_F_GCR_ECC_CED_SYSRAM2ECCNDED               ((uint32_t)(0x1UL << MXC_F_GCR_ECC_CED_SYSRAM2ECCNDED_POS)) /**< ECC_CED_SYSRAM2ECCNDED Mask */

#define MXC_F_GCR_ECC_CED_SYSRAM3ECCNDED_POS           3 /**< ECC_CED_SYSRAM3ECCNDED Position */
#define MXC_F_GCR_ECC_CED_SYSRAM3ECCNDED               ((uint32_t)(0x1UL << MXC_F_GCR_ECC_CED_SYSRAM3ECCNDED_POS)) /**< ECC_CED_SYSRAM3ECCNDED Mask */

#define MXC_F_GCR_ECC_CED_SYSRAM4ECCNDED_POS           4 /**< ECC_CED_SYSRAM4ECCNDED Position */
#define MXC_F_GCR_ECC_CED_SYSRAM4ECCNDED               ((uint32_t)(0x1UL << MXC_F_GCR_ECC_CED_SYSRAM4ECCNDED_POS)) /**< ECC_CED_SYSRAM4ECCNDED Mask */

#define MXC_F_GCR_ECC_CED_SYSRAM5ECCNDED_POS           5 /**< ECC_CED_SYSRAM5ECCNDED Position */
#define MXC_F_GCR_ECC_CED_SYSRAM5ECCNDED               ((uint32_t)(0x1UL << MXC_F_GCR_ECC_CED_SYSRAM5ECCNDED_POS)) /**< ECC_CED_SYSRAM5ECCNDED Mask */

#define MXC_F_GCR_ECC_CED_IC0ECCNDED_POS               8 /**< ECC_CED_IC0ECCNDED Position */
#define MXC_F_GCR_ECC_CED_IC0ECCNDED                   ((uint32_t)(0x1UL << MXC_F_GCR_ECC_CED_IC0ECCNDED_POS)) /**< ECC_CED_IC0ECCNDED Mask */

#define MXC_F_GCR_ECC_CED_IC1ECCNDED_POS               9 /**< ECC_CED_IC1ECCNDED Position */
#define MXC_F_GCR_ECC_CED_IC1ECCNDED                   ((uint32_t)(0x1UL << MXC_F_GCR_ECC_CED_IC1ECCNDED_POS)) /**< ECC_CED_IC1ECCNDED Mask */

#define MXC_F_GCR_ECC_CED_ICXIPECCNDED_POS             10 /**< ECC_CED_ICXIPECCNDED Position */
#define MXC_F_GCR_ECC_CED_ICXIPECCNDED                 ((uint32_t)(0x1UL << MXC_F_GCR_ECC_CED_ICXIPECCNDED_POS)) /**< ECC_CED_ICXIPECCNDED Mask */

#define MXC_F_GCR_ECC_CED_FL0ECCNDED_POS               11 /**< ECC_CED_FL0ECCNDED Position */
#define MXC_F_GCR_ECC_CED_FL0ECCNDED                   ((uint32_t)(0x1UL << MXC_F_GCR_ECC_CED_FL0ECCNDED_POS)) /**< ECC_CED_FL0ECCNDED Mask */

#define MXC_F_GCR_ECC_CED_FL1ECCNDED_POS               12 /**< ECC_CED_FL1ECCNDED Position */
#define MXC_F_GCR_ECC_CED_FL1ECCNDED                   ((uint32_t)(0x1UL << MXC_F_GCR_ECC_CED_FL1ECCNDED_POS)) /**< ECC_CED_FL1ECCNDED Mask */

/**@} end of group GCR_ECC_CED_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_ECC_IRQEN GCR_ECC_IRQEN
 * @brief    ECC IRQ Enable Register
 * @{
 */
#define MXC_F_GCR_ECC_IRQEN_SYSRAM0ECCEN_POS           0 /**< ECC_IRQEN_SYSRAM0ECCEN Position */
#define MXC_F_GCR_ECC_IRQEN_SYSRAM0ECCEN               ((uint32_t)(0x1UL << MXC_F_GCR_ECC_IRQEN_SYSRAM0ECCEN_POS)) /**< ECC_IRQEN_SYSRAM0ECCEN Mask */

#define MXC_F_GCR_ECC_IRQEN_SYSRAM1ECCEN_POS           1 /**< ECC_IRQEN_SYSRAM1ECCEN Position */
#define MXC_F_GCR_ECC_IRQEN_SYSRAM1ECCEN               ((uint32_t)(0x1UL << MXC_F_GCR_ECC_IRQEN_SYSRAM1ECCEN_POS)) /**< ECC_IRQEN_SYSRAM1ECCEN Mask */

#define MXC_F_GCR_ECC_IRQEN_SYSRAM2ECCEN_POS           2 /**< ECC_IRQEN_SYSRAM2ECCEN Position */
#define MXC_F_GCR_ECC_IRQEN_SYSRAM2ECCEN               ((uint32_t)(0x1UL << MXC_F_GCR_ECC_IRQEN_SYSRAM2ECCEN_POS)) /**< ECC_IRQEN_SYSRAM2ECCEN Mask */

#define MXC_F_GCR_ECC_IRQEN_SYSRAM3ECCEN_POS           3 /**< ECC_IRQEN_SYSRAM3ECCEN Position */
#define MXC_F_GCR_ECC_IRQEN_SYSRAM3ECCEN               ((uint32_t)(0x1UL << MXC_F_GCR_ECC_IRQEN_SYSRAM3ECCEN_POS)) /**< ECC_IRQEN_SYSRAM3ECCEN Mask */

#define MXC_F_GCR_ECC_IRQEN_SYSRAM4ECCEN_POS           4 /**< ECC_IRQEN_SYSRAM4ECCEN Position */
#define MXC_F_GCR_ECC_IRQEN_SYSRAM4ECCEN               ((uint32_t)(0x1UL << MXC_F_GCR_ECC_IRQEN_SYSRAM4ECCEN_POS)) /**< ECC_IRQEN_SYSRAM4ECCEN Mask */

#define MXC_F_GCR_ECC_IRQEN_SYSRAM5ECCEN_POS           5 /**< ECC_IRQEN_SYSRAM5ECCEN Position */
#define MXC_F_GCR_ECC_IRQEN_SYSRAM5ECCEN               ((uint32_t)(0x1UL << MXC_F_GCR_ECC_IRQEN_SYSRAM5ECCEN_POS)) /**< ECC_IRQEN_SYSRAM5ECCEN Mask */

#define MXC_F_GCR_ECC_IRQEN_IC0ECCEN_POS               8 /**< ECC_IRQEN_IC0ECCEN Position */
#define MXC_F_GCR_ECC_IRQEN_IC0ECCEN                   ((uint32_t)(0x1UL << MXC_F_GCR_ECC_IRQEN_IC0ECCEN_POS)) /**< ECC_IRQEN_IC0ECCEN Mask */

#define MXC_F_GCR_ECC_IRQEN_IC1ECCEN_POS               9 /**< ECC_IRQEN_IC1ECCEN Position */
#define MXC_F_GCR_ECC_IRQEN_IC1ECCEN                   ((uint32_t)(0x1UL << MXC_F_GCR_ECC_IRQEN_IC1ECCEN_POS)) /**< ECC_IRQEN_IC1ECCEN Mask */

#define MXC_F_GCR_ECC_IRQEN_ICXIPECCEN_POS             10 /**< ECC_IRQEN_ICXIPECCEN Position */
#define MXC_F_GCR_ECC_IRQEN_ICXIPECCEN                 ((uint32_t)(0x1UL << MXC_F_GCR_ECC_IRQEN_ICXIPECCEN_POS)) /**< ECC_IRQEN_ICXIPECCEN Mask */

#define MXC_F_GCR_ECC_IRQEN_FL0ECCEN_POS               11 /**< ECC_IRQEN_FL0ECCEN Position */
#define MXC_F_GCR_ECC_IRQEN_FL0ECCEN                   ((uint32_t)(0x1UL << MXC_F_GCR_ECC_IRQEN_FL0ECCEN_POS)) /**< ECC_IRQEN_FL0ECCEN Mask */

#define MXC_F_GCR_ECC_IRQEN_FL1ECCEN_POS               12 /**< ECC_IRQEN_FL1ECCEN Position */
#define MXC_F_GCR_ECC_IRQEN_FL1ECCEN                   ((uint32_t)(0x1UL << MXC_F_GCR_ECC_IRQEN_FL1ECCEN_POS)) /**< ECC_IRQEN_FL1ECCEN Mask */

/**@} end of group GCR_ECC_IRQEN_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_ECC_ERRAD GCR_ECC_ERRAD
 * @brief    ECC Error Address Register
 * @{
 */
#define MXC_F_GCR_ECC_ERRAD_DATARAMADDR_POS            0 /**< ECC_ERRAD_DATARAMADDR Position */
#define MXC_F_GCR_ECC_ERRAD_DATARAMADDR                ((uint32_t)(0x1FFFUL << MXC_F_GCR_ECC_ERRAD_DATARAMADDR_POS)) /**< ECC_ERRAD_DATARAMADDR Mask */

#define MXC_F_GCR_ECC_ERRAD_DATARAMBANK_POS            14 /**< ECC_ERRAD_DATARAMBANK Position */
#define MXC_F_GCR_ECC_ERRAD_DATARAMBANK                ((uint32_t)(0x1UL << MXC_F_GCR_ECC_ERRAD_DATARAMBANK_POS)) /**< ECC_ERRAD_DATARAMBANK Mask */

#define MXC_F_GCR_ECC_ERRAD_DATARAMERR_POS             15 /**< ECC_ERRAD_DATARAMERR Position */
#define MXC_F_GCR_ECC_ERRAD_DATARAMERR                 ((uint32_t)(0x1UL << MXC_F_GCR_ECC_ERRAD_DATARAMERR_POS)) /**< ECC_ERRAD_DATARAMERR Mask */

#define MXC_F_GCR_ECC_ERRAD_TAGRAMADDR_POS             16 /**< ECC_ERRAD_TAGRAMADDR Position */
#define MXC_F_GCR_ECC_ERRAD_TAGRAMADDR                 ((uint32_t)(0x1FFFUL << MXC_F_GCR_ECC_ERRAD_TAGRAMADDR_POS)) /**< ECC_ERRAD_TAGRAMADDR Mask */

#define MXC_F_GCR_ECC_ERRAD_TAGRAMBANK_POS             30 /**< ECC_ERRAD_TAGRAMBANK Position */
#define MXC_F_GCR_ECC_ERRAD_TAGRAMBANK                 ((uint32_t)(0x1UL << MXC_F_GCR_ECC_ERRAD_TAGRAMBANK_POS)) /**< ECC_ERRAD_TAGRAMBANK Mask */

#define MXC_F_GCR_ECC_ERRAD_TAGRAMERR_POS              31 /**< ECC_ERRAD_TAGRAMERR Position */
#define MXC_F_GCR_ECC_ERRAD_TAGRAMERR                  ((uint32_t)(0x1UL << MXC_F_GCR_ECC_ERRAD_TAGRAMERR_POS)) /**< ECC_ERRAD_TAGRAMERR Mask */

/**@} end of group GCR_ECC_ERRAD_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_BTLE_LDOCR GCR_BTLE_LDOCR
 * @brief    BTLE LDO Control Register
 * @{
 */
#define MXC_F_GCR_BTLE_LDOCR_LDOTXEN_POS               0 /**< BTLE_LDOCR_LDOTXEN Position */
#define MXC_F_GCR_BTLE_LDOCR_LDOTXEN                   ((uint32_t)(0x1UL << MXC_F_GCR_BTLE_LDOCR_LDOTXEN_POS)) /**< BTLE_LDOCR_LDOTXEN Mask */

#define MXC_F_GCR_BTLE_LDOCR_LDOTXOPULLD_POS           1 /**< BTLE_LDOCR_LDOTXOPULLD Position */
#define MXC_F_GCR_BTLE_LDOCR_LDOTXOPULLD               ((uint32_t)(0x1UL << MXC_F_GCR_BTLE_LDOCR_LDOTXOPULLD_POS)) /**< BTLE_LDOCR_LDOTXOPULLD Mask */

#define MXC_F_GCR_BTLE_LDOCR_LDOTXVSEL_POS             2 /**< BTLE_LDOCR_LDOTXVSEL Position */
#define MXC_F_GCR_BTLE_LDOCR_LDOTXVSEL                 ((uint32_t)(0x3UL << MXC_F_GCR_BTLE_LDOCR_LDOTXVSEL_POS)) /**< BTLE_LDOCR_LDOTXVSEL Mask */
#define MXC_V_GCR_BTLE_LDOCR_LDOTXVSEL_0_7             ((uint32_t)0x0UL) /**< BTLE_LDOCR_LDOTXVSEL_0_7 Value */
#define MXC_S_GCR_BTLE_LDOCR_LDOTXVSEL_0_7             (MXC_V_GCR_BTLE_LDOCR_LDOTXVSEL_0_7 << MXC_F_GCR_BTLE_LDOCR_LDOTXVSEL_POS) /**< BTLE_LDOCR_LDOTXVSEL_0_7 Setting */
#define MXC_V_GCR_BTLE_LDOCR_LDOTXVSEL_0_85            ((uint32_t)0x1UL) /**< BTLE_LDOCR_LDOTXVSEL_0_85 Value */
#define MXC_S_GCR_BTLE_LDOCR_LDOTXVSEL_0_85            (MXC_V_GCR_BTLE_LDOCR_LDOTXVSEL_0_85 << MXC_F_GCR_BTLE_LDOCR_LDOTXVSEL_POS) /**< BTLE_LDOCR_LDOTXVSEL_0_85 Setting */
#define MXC_V_GCR_BTLE_LDOCR_LDOTXVSEL_0_9             ((uint32_t)0x2UL) /**< BTLE_LDOCR_LDOTXVSEL_0_9 Value */
#define MXC_S_GCR_BTLE_LDOCR_LDOTXVSEL_0_9             (MXC_V_GCR_BTLE_LDOCR_LDOTXVSEL_0_9 << MXC_F_GCR_BTLE_LDOCR_LDOTXVSEL_POS) /**< BTLE_LDOCR_LDOTXVSEL_0_9 Setting */
#define MXC_V_GCR_BTLE_LDOCR_LDOTXVSEL_1_1             ((uint32_t)0x3UL) /**< BTLE_LDOCR_LDOTXVSEL_1_1 Value */
#define MXC_S_GCR_BTLE_LDOCR_LDOTXVSEL_1_1             (MXC_V_GCR_BTLE_LDOCR_LDOTXVSEL_1_1 << MXC_F_GCR_BTLE_LDOCR_LDOTXVSEL_POS) /**< BTLE_LDOCR_LDOTXVSEL_1_1 Setting */

#define MXC_F_GCR_BTLE_LDOCR_LDORXEN_POS               4 /**< BTLE_LDOCR_LDORXEN Position */
#define MXC_F_GCR_BTLE_LDOCR_LDORXEN                   ((uint32_t)(0x1UL << MXC_F_GCR_BTLE_LDOCR_LDORXEN_POS)) /**< BTLE_LDOCR_LDORXEN Mask */

#define MXC_F_GCR_BTLE_LDOCR_LDORXPULLD_POS            5 /**< BTLE_LDOCR_LDORXPULLD Position */
#define MXC_F_GCR_BTLE_LDOCR_LDORXPULLD                ((uint32_t)(0x1UL << MXC_F_GCR_BTLE_LDOCR_LDORXPULLD_POS)) /**< BTLE_LDOCR_LDORXPULLD Mask */

#define MXC_F_GCR_BTLE_LDOCR_LDORXVSEL_POS             6 /**< BTLE_LDOCR_LDORXVSEL Position */
#define MXC_F_GCR_BTLE_LDOCR_LDORXVSEL                 ((uint32_t)(0x3UL << MXC_F_GCR_BTLE_LDOCR_LDORXVSEL_POS)) /**< BTLE_LDOCR_LDORXVSEL Mask */
#define MXC_V_GCR_BTLE_LDOCR_LDORXVSEL_0_7             ((uint32_t)0x0UL) /**< BTLE_LDOCR_LDORXVSEL_0_7 Value */
#define MXC_S_GCR_BTLE_LDOCR_LDORXVSEL_0_7             (MXC_V_GCR_BTLE_LDOCR_LDORXVSEL_0_7 << MXC_F_GCR_BTLE_LDOCR_LDORXVSEL_POS) /**< BTLE_LDOCR_LDORXVSEL_0_7 Setting */
#define MXC_V_GCR_BTLE_LDOCR_LDORXVSEL_0_85            ((uint32_t)0x1UL) /**< BTLE_LDOCR_LDORXVSEL_0_85 Value */
#define MXC_S_GCR_BTLE_LDOCR_LDORXVSEL_0_85            (MXC_V_GCR_BTLE_LDOCR_LDORXVSEL_0_85 << MXC_F_GCR_BTLE_LDOCR_LDORXVSEL_POS) /**< BTLE_LDOCR_LDORXVSEL_0_85 Setting */
#define MXC_V_GCR_BTLE_LDOCR_LDORXVSEL_0_9             ((uint32_t)0x2UL) /**< BTLE_LDOCR_LDORXVSEL_0_9 Value */
#define MXC_S_GCR_BTLE_LDOCR_LDORXVSEL_0_9             (MXC_V_GCR_BTLE_LDOCR_LDORXVSEL_0_9 << MXC_F_GCR_BTLE_LDOCR_LDORXVSEL_POS) /**< BTLE_LDOCR_LDORXVSEL_0_9 Setting */
#define MXC_V_GCR_BTLE_LDOCR_LDORXVSEL_1_1             ((uint32_t)0x3UL) /**< BTLE_LDOCR_LDORXVSEL_1_1 Value */
#define MXC_S_GCR_BTLE_LDOCR_LDORXVSEL_1_1             (MXC_V_GCR_BTLE_LDOCR_LDORXVSEL_1_1 << MXC_F_GCR_BTLE_LDOCR_LDORXVSEL_POS) /**< BTLE_LDOCR_LDORXVSEL_1_1 Setting */

#define MXC_F_GCR_BTLE_LDOCR_LDORXBYP_POS              8 /**< BTLE_LDOCR_LDORXBYP Position */
#define MXC_F_GCR_BTLE_LDOCR_LDORXBYP                  ((uint32_t)(0x1UL << MXC_F_GCR_BTLE_LDOCR_LDORXBYP_POS)) /**< BTLE_LDOCR_LDORXBYP Mask */

#define MXC_F_GCR_BTLE_LDOCR_LDORXDISCH_POS            9 /**< BTLE_LDOCR_LDORXDISCH Position */
#define MXC_F_GCR_BTLE_LDOCR_LDORXDISCH                ((uint32_t)(0x1UL << MXC_F_GCR_BTLE_LDOCR_LDORXDISCH_POS)) /**< BTLE_LDOCR_LDORXDISCH Mask */

#define MXC_F_GCR_BTLE_LDOCR_LDOTXBYP_POS              10 /**< BTLE_LDOCR_LDOTXBYP Position */
#define MXC_F_GCR_BTLE_LDOCR_LDOTXBYP                  ((uint32_t)(0x1UL << MXC_F_GCR_BTLE_LDOCR_LDOTXBYP_POS)) /**< BTLE_LDOCR_LDOTXBYP Mask */

#define MXC_F_GCR_BTLE_LDOCR_LDOTXDISCH_POS            11 /**< BTLE_LDOCR_LDOTXDISCH Position */
#define MXC_F_GCR_BTLE_LDOCR_LDOTXDISCH                ((uint32_t)(0x1UL << MXC_F_GCR_BTLE_LDOCR_LDOTXDISCH_POS)) /**< BTLE_LDOCR_LDOTXDISCH Mask */

#define MXC_F_GCR_BTLE_LDOCR_LDOTXENDLY_POS            12 /**< BTLE_LDOCR_LDOTXENDLY Position */
#define MXC_F_GCR_BTLE_LDOCR_LDOTXENDLY                ((uint32_t)(0x1UL << MXC_F_GCR_BTLE_LDOCR_LDOTXENDLY_POS)) /**< BTLE_LDOCR_LDOTXENDLY Mask */

#define MXC_F_GCR_BTLE_LDOCR_LDORXENDLY_POS            13 /**< BTLE_LDOCR_LDORXENDLY Position */
#define MXC_F_GCR_BTLE_LDOCR_LDORXENDLY                ((uint32_t)(0x1UL << MXC_F_GCR_BTLE_LDOCR_LDORXENDLY_POS)) /**< BTLE_LDOCR_LDORXENDLY Mask */

#define MXC_F_GCR_BTLE_LDOCR_LDORXBYPENENDLY_POS       14 /**< BTLE_LDOCR_LDORXBYPENENDLY Position */
#define MXC_F_GCR_BTLE_LDOCR_LDORXBYPENENDLY           ((uint32_t)(0x1UL << MXC_F_GCR_BTLE_LDOCR_LDORXBYPENENDLY_POS)) /**< BTLE_LDOCR_LDORXBYPENENDLY Mask */

#define MXC_F_GCR_BTLE_LDOCR_LDOTXBYPENENDLY_POS       15 /**< BTLE_LDOCR_LDOTXBYPENENDLY Position */
#define MXC_F_GCR_BTLE_LDOCR_LDOTXBYPENENDLY           ((uint32_t)(0x1UL << MXC_F_GCR_BTLE_LDOCR_LDOTXBYPENENDLY_POS)) /**< BTLE_LDOCR_LDOTXBYPENENDLY Mask */

/**@} end of group GCR_BTLE_LDOCR_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_BTLE_LDODCR GCR_BTLE_LDODCR
 * @brief    BTLE LDO Delay Register
 * @{
 */
#define MXC_F_GCR_BTLE_LDODCR_BYPDLYCNT_POS            0 /**< BTLE_LDODCR_BYPDLYCNT Position */
#define MXC_F_GCR_BTLE_LDODCR_BYPDLYCNT                ((uint32_t)(0xFFUL << MXC_F_GCR_BTLE_LDODCR_BYPDLYCNT_POS)) /**< BTLE_LDODCR_BYPDLYCNT Mask */

#define MXC_F_GCR_BTLE_LDODCR_LDORXDLYCNT_POS          8 /**< BTLE_LDODCR_LDORXDLYCNT Position */
#define MXC_F_GCR_BTLE_LDODCR_LDORXDLYCNT              ((uint32_t)(0x1FFUL << MXC_F_GCR_BTLE_LDODCR_LDORXDLYCNT_POS)) /**< BTLE_LDODCR_LDORXDLYCNT Mask */

#define MXC_F_GCR_BTLE_LDODCR_LDOTXDLYCNT_POS          20 /**< BTLE_LDODCR_LDOTXDLYCNT Position */
#define MXC_F_GCR_BTLE_LDODCR_LDOTXDLYCNT              ((uint32_t)(0x1FFUL << MXC_F_GCR_BTLE_LDODCR_LDOTXDLYCNT_POS)) /**< BTLE_LDODCR_LDOTXDLYCNT Mask */

/**@} end of group GCR_BTLE_LDODCR_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_GP0 GCR_GP0
 * @brief    General Purpose Register 0
 * @{
 */
#define MXC_F_GCR_GP0_GPR0_POS                         0 /**< GP0_GPR0 Position */
#define MXC_F_GCR_GP0_GPR0                             ((uint32_t)(0xFFFFFFFFUL << MXC_F_GCR_GP0_GPR0_POS)) /**< GP0_GPR0 Mask */

/**@} end of group GCR_GP0_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_APB_ASYNC GCR_APB_ASYNC
 * @brief    APB Asynchronous Bridge Select Register
 * @{
 */
#define MXC_F_GCR_APB_ASYNC_APBASYNCI2C0_POS           0 /**< APB_ASYNC_APBASYNCI2C0 Position */
#define MXC_F_GCR_APB_ASYNC_APBASYNCI2C0               ((uint32_t)(0x1UL << MXC_F_GCR_APB_ASYNC_APBASYNCI2C0_POS)) /**< APB_ASYNC_APBASYNCI2C0 Mask */

#define MXC_F_GCR_APB_ASYNC_APBASYNCI2C1_POS           1 /**< APB_ASYNC_APBASYNCI2C1 Position */
#define MXC_F_GCR_APB_ASYNC_APBASYNCI2C1               ((uint32_t)(0x1UL << MXC_F_GCR_APB_ASYNC_APBASYNCI2C1_POS)) /**< APB_ASYNC_APBASYNCI2C1 Mask */

#define MXC_F_GCR_APB_ASYNC_APBASYNCI2C2_POS           2 /**< APB_ASYNC_APBASYNCI2C2 Position */
#define MXC_F_GCR_APB_ASYNC_APBASYNCI2C2               ((uint32_t)(0x1UL << MXC_F_GCR_APB_ASYNC_APBASYNCI2C2_POS)) /**< APB_ASYNC_APBASYNCI2C2 Mask */

#define MXC_F_GCR_APB_ASYNC_APBASYNCPT_POS             3 /**< APB_ASYNC_APBASYNCPT Position */
#define MXC_F_GCR_APB_ASYNC_APBASYNCPT                 ((uint32_t)(0x1UL << MXC_F_GCR_APB_ASYNC_APBASYNCPT_POS)) /**< APB_ASYNC_APBASYNCPT Mask */

/**@} end of group GCR_APB_ASYNC_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32665_INCLUDE_GCR_REGS_H_
