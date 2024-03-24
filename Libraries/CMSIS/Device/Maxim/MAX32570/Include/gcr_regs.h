/**
 * @file    gcr_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the GCR Peripheral Module.
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32570_INCLUDE_GCR_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32570_INCLUDE_GCR_REGS_H_

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
    __IO uint32_t scck;                 /**< <tt>\b 0x34:</tt> GCR SCCK Register */
    __R  uint32_t rsv_0x38_0x3f[2];
    __IO uint32_t sysst;                /**< <tt>\b 0x40:</tt> GCR SYSST Register */
    __IO uint32_t rst1;                 /**< <tt>\b 0x44:</tt> GCR RST1 Register */
    __IO uint32_t pclkdis1;             /**< <tt>\b 0x48:</tt> GCR PCLKDIS1 Register */
    __IO uint32_t eventen;              /**< <tt>\b 0x4C:</tt> GCR EVENTEN Register */
    __I  uint32_t revision;             /**< <tt>\b 0x50:</tt> GCR REVISION Register */
    __IO uint32_t sysie;                /**< <tt>\b 0x54:</tt> GCR SYSIE Register */
    __IO uint32_t ipocnt;               /**< <tt>\b 0x58:</tt> GCR IPOCNT Register */
    __R  uint32_t rsv_0x5c_0x63[2];
    __IO uint32_t eccerr;               /**< <tt>\b 0x64:</tt> GCR ECCERR Register */
    __IO uint32_t eccced;               /**< <tt>\b 0x68:</tt> GCR ECCCED Register */
    __IO uint32_t eccie;                /**< <tt>\b 0x6C:</tt> GCR ECCIE Register */
    __IO uint32_t eccaddr;              /**< <tt>\b 0x70:</tt> GCR ECCADDR Register */
    __IO uint32_t nfc_ldocr;            /**< <tt>\b 0x74:</tt> GCR NFC_LDOCR Register */
    __IO uint32_t nfcldo_dly;           /**< <tt>\b 0x78:</tt> GCR NFCLDO_DLY Register */
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
#define MXC_R_GCR_SCCK                     ((uint32_t)0x00000034UL) /**< Offset from GCR Base Address: <tt> 0x0034</tt> */
#define MXC_R_GCR_SYSST                    ((uint32_t)0x00000040UL) /**< Offset from GCR Base Address: <tt> 0x0040</tt> */
#define MXC_R_GCR_RST1                     ((uint32_t)0x00000044UL) /**< Offset from GCR Base Address: <tt> 0x0044</tt> */
#define MXC_R_GCR_PCLKDIS1                 ((uint32_t)0x00000048UL) /**< Offset from GCR Base Address: <tt> 0x0048</tt> */
#define MXC_R_GCR_EVENTEN                  ((uint32_t)0x0000004CUL) /**< Offset from GCR Base Address: <tt> 0x004C</tt> */
#define MXC_R_GCR_REVISION                 ((uint32_t)0x00000050UL) /**< Offset from GCR Base Address: <tt> 0x0050</tt> */
#define MXC_R_GCR_SYSIE                    ((uint32_t)0x00000054UL) /**< Offset from GCR Base Address: <tt> 0x0054</tt> */
#define MXC_R_GCR_IPOCNT                   ((uint32_t)0x00000058UL) /**< Offset from GCR Base Address: <tt> 0x0058</tt> */
#define MXC_R_GCR_ECCERR                   ((uint32_t)0x00000064UL) /**< Offset from GCR Base Address: <tt> 0x0064</tt> */
#define MXC_R_GCR_ECCCED                   ((uint32_t)0x00000068UL) /**< Offset from GCR Base Address: <tt> 0x0068</tt> */
#define MXC_R_GCR_ECCIE                    ((uint32_t)0x0000006CUL) /**< Offset from GCR Base Address: <tt> 0x006C</tt> */
#define MXC_R_GCR_ECCADDR                  ((uint32_t)0x00000070UL) /**< Offset from GCR Base Address: <tt> 0x0070</tt> */
#define MXC_R_GCR_NFC_LDOCR                ((uint32_t)0x00000074UL) /**< Offset from GCR Base Address: <tt> 0x0074</tt> */
#define MXC_R_GCR_NFCLDO_DLY               ((uint32_t)0x00000078UL) /**< Offset from GCR Base Address: <tt> 0x0078</tt> */
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

#define MXC_F_GCR_SYSCTRL_FLASH0_PAGE_FLIP_POS         4 /**< SYSCTRL_FLASH0_PAGE_FLIP Position */
#define MXC_F_GCR_SYSCTRL_FLASH0_PAGE_FLIP             ((uint32_t)(0x1UL << MXC_F_GCR_SYSCTRL_FLASH0_PAGE_FLIP_POS)) /**< SYSCTRL_FLASH0_PAGE_FLIP Mask */

#define MXC_F_GCR_SYSCTRL_FPU_DIS_POS                  5 /**< SYSCTRL_FPU_DIS Position */
#define MXC_F_GCR_SYSCTRL_FPU_DIS                      ((uint32_t)(0x1UL << MXC_F_GCR_SYSCTRL_FPU_DIS_POS)) /**< SYSCTRL_FPU_DIS Mask */

#define MXC_F_GCR_SYSCTRL_ICC0_FLUSH_POS               6 /**< SYSCTRL_ICC0_FLUSH Position */
#define MXC_F_GCR_SYSCTRL_ICC0_FLUSH                   ((uint32_t)(0x1UL << MXC_F_GCR_SYSCTRL_ICC0_FLUSH_POS)) /**< SYSCTRL_ICC0_FLUSH Mask */

#define MXC_F_GCR_SYSCTRL_SRCC_FLUSH_POS               7 /**< SYSCTRL_SRCC_FLUSH Position */
#define MXC_F_GCR_SYSCTRL_SRCC_FLUSH                   ((uint32_t)(0x1UL << MXC_F_GCR_SYSCTRL_SRCC_FLUSH_POS)) /**< SYSCTRL_SRCC_FLUSH Mask */

#define MXC_F_GCR_SYSCTRL_SRCC_DIS_POS                 9 /**< SYSCTRL_SRCC_DIS Position */
#define MXC_F_GCR_SYSCTRL_SRCC_DIS                     ((uint32_t)(0x1UL << MXC_F_GCR_SYSCTRL_SRCC_DIS_POS)) /**< SYSCTRL_SRCC_DIS Mask */

#define MXC_F_GCR_SYSCTRL_CCHK_POS                     13 /**< SYSCTRL_CCHK Position */
#define MXC_F_GCR_SYSCTRL_CCHK                         ((uint32_t)(0x1UL << MXC_F_GCR_SYSCTRL_CCHK_POS)) /**< SYSCTRL_CCHK Mask */

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

#define MXC_F_GCR_RST0_WDT0_POS                        1 /**< RST0_WDT0 Position */
#define MXC_F_GCR_RST0_WDT0                            ((uint32_t)(0x1UL << MXC_F_GCR_RST0_WDT0_POS)) /**< RST0_WDT0 Mask */

#define MXC_F_GCR_RST0_GPIO0_POS                       2 /**< RST0_GPIO0 Position */
#define MXC_F_GCR_RST0_GPIO0                           ((uint32_t)(0x1UL << MXC_F_GCR_RST0_GPIO0_POS)) /**< RST0_GPIO0 Mask */

#define MXC_F_GCR_RST0_GPIO1_POS                       3 /**< RST0_GPIO1 Position */
#define MXC_F_GCR_RST0_GPIO1                           ((uint32_t)(0x1UL << MXC_F_GCR_RST0_GPIO1_POS)) /**< RST0_GPIO1 Mask */

#define MXC_F_GCR_RST0_GPIO2_POS                       4 /**< RST0_GPIO2 Position */
#define MXC_F_GCR_RST0_GPIO2                           ((uint32_t)(0x1UL << MXC_F_GCR_RST0_GPIO2_POS)) /**< RST0_GPIO2 Mask */

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

#define MXC_F_GCR_RST0_SPI2_POS                        15 /**< RST0_SPI2 Position */
#define MXC_F_GCR_RST0_SPI2                            ((uint32_t)(0x1UL << MXC_F_GCR_RST0_SPI2_POS)) /**< RST0_SPI2 Mask */

#define MXC_F_GCR_RST0_I2C0_POS                        16 /**< RST0_I2C0 Position */
#define MXC_F_GCR_RST0_I2C0                            ((uint32_t)(0x1UL << MXC_F_GCR_RST0_I2C0_POS)) /**< RST0_I2C0 Mask */

#define MXC_F_GCR_RST0_RTC_POS                         17 /**< RST0_RTC Position */
#define MXC_F_GCR_RST0_RTC                             ((uint32_t)(0x1UL << MXC_F_GCR_RST0_RTC_POS)) /**< RST0_RTC Mask */

#define MXC_F_GCR_RST0_CRYPTO_POS                      18 /**< RST0_CRYPTO Position */
#define MXC_F_GCR_RST0_CRYPTO                          ((uint32_t)(0x1UL << MXC_F_GCR_RST0_CRYPTO_POS)) /**< RST0_CRYPTO Mask */

#define MXC_F_GCR_RST0_TMR6_POS                        20 /**< RST0_TMR6 Position */
#define MXC_F_GCR_RST0_TMR6                            ((uint32_t)(0x1UL << MXC_F_GCR_RST0_TMR6_POS)) /**< RST0_TMR6 Mask */

#define MXC_F_GCR_RST0_TMR7_POS                        21 /**< RST0_TMR7 Position */
#define MXC_F_GCR_RST0_TMR7                            ((uint32_t)(0x1UL << MXC_F_GCR_RST0_TMR7_POS)) /**< RST0_TMR7 Mask */

#define MXC_F_GCR_RST0_CLCD_POS                        22 /**< RST0_CLCD Position */
#define MXC_F_GCR_RST0_CLCD                            ((uint32_t)(0x1UL << MXC_F_GCR_RST0_CLCD_POS)) /**< RST0_CLCD Mask */

#define MXC_F_GCR_RST0_USB_POS                         23 /**< RST0_USB Position */
#define MXC_F_GCR_RST0_USB                             ((uint32_t)(0x1UL << MXC_F_GCR_RST0_USB_POS)) /**< RST0_USB Mask */

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

#define MXC_F_GCR_CLKCTRL_SYSCLK_RDY_POS               13 /**< CLKCTRL_SYSCLK_RDY Position */
#define MXC_F_GCR_CLKCTRL_SYSCLK_RDY                   ((uint32_t)(0x1UL << MXC_F_GCR_CLKCTRL_SYSCLK_RDY_POS)) /**< CLKCTRL_SYSCLK_RDY Mask */

#define MXC_F_GCR_CLKCTRL_CCD_POS                      15 /**< CLKCTRL_CCD Position */
#define MXC_F_GCR_CLKCTRL_CCD                          ((uint32_t)(0x1UL << MXC_F_GCR_CLKCTRL_CCD_POS)) /**< CLKCTRL_CCD Mask */

#define MXC_F_GCR_CLKCTRL_ERFO_EN_POS                  16 /**< CLKCTRL_ERFO_EN Position */
#define MXC_F_GCR_CLKCTRL_ERFO_EN                      ((uint32_t)(0x1UL << MXC_F_GCR_CLKCTRL_ERFO_EN_POS)) /**< CLKCTRL_ERFO_EN Mask */

#define MXC_F_GCR_CLKCTRL_ERTCO_EN_POS                 17 /**< CLKCTRL_ERTCO_EN Position */
#define MXC_F_GCR_CLKCTRL_ERTCO_EN                     ((uint32_t)(0x1UL << MXC_F_GCR_CLKCTRL_ERTCO_EN_POS)) /**< CLKCTRL_ERTCO_EN Mask */

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

#define MXC_F_GCR_PM_HA0_WE_POS                        7 /**< PM_HA0_WE Position */
#define MXC_F_GCR_PM_HA0_WE                            ((uint32_t)(0x1UL << MXC_F_GCR_PM_HA0_WE_POS)) /**< PM_HA0_WE Mask */

#define MXC_F_GCR_PM_HA1_WE_POS                        9 /**< PM_HA1_WE Position */
#define MXC_F_GCR_PM_HA1_WE                            ((uint32_t)(0x1UL << MXC_F_GCR_PM_HA1_WE_POS)) /**< PM_HA1_WE Mask */

#define MXC_F_GCR_PM_ERFO_PD_POS                       12 /**< PM_ERFO_PD Position */
#define MXC_F_GCR_PM_ERFO_PD                           ((uint32_t)(0x1UL << MXC_F_GCR_PM_ERFO_PD_POS)) /**< PM_ERFO_PD Mask */

#define MXC_F_GCR_PM_ISO_PD_POS                        15 /**< PM_ISO_PD Position */
#define MXC_F_GCR_PM_ISO_PD                            ((uint32_t)(0x1UL << MXC_F_GCR_PM_ISO_PD_POS)) /**< PM_ISO_PD Mask */

#define MXC_F_GCR_PM_IPO_PD_POS                        16 /**< PM_IPO_PD Position */
#define MXC_F_GCR_PM_IPO_PD                            ((uint32_t)(0x1UL << MXC_F_GCR_PM_IPO_PD_POS)) /**< PM_IPO_PD Mask */

#define MXC_F_GCR_PM_IBRO_PD_POS                       17 /**< PM_IBRO_PD Position */
#define MXC_F_GCR_PM_IBRO_PD                           ((uint32_t)(0x1UL << MXC_F_GCR_PM_IBRO_PD_POS)) /**< PM_IBRO_PD Mask */

#define MXC_F_GCR_PM_NFC_PD_POS                        18 /**< PM_NFC_PD Position */
#define MXC_F_GCR_PM_NFC_PD                            ((uint32_t)(0x1UL << MXC_F_GCR_PM_NFC_PD_POS)) /**< PM_NFC_PD Mask */

#define MXC_F_GCR_PM_XTALBP_POS                        20 /**< PM_XTALBP Position */
#define MXC_F_GCR_PM_XTALBP                            ((uint32_t)(0x1UL << MXC_F_GCR_PM_XTALBP_POS)) /**< PM_XTALBP Mask */

/**@} end of group GCR_PM_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_PCLKDIV GCR_PCLKDIV
 * @brief    Peripheral Clock Divider.
 * @{
 */
#define MXC_F_GCR_PCLKDIV_PCF_POS                      0 /**< PCLKDIV_PCF Position */
#define MXC_F_GCR_PCLKDIV_PCF                          ((uint32_t)(0x7UL << MXC_F_GCR_PCLKDIV_PCF_POS)) /**< PCLKDIV_PCF Mask */
#define MXC_V_GCR_PCLKDIV_PCF_96MHZ                    ((uint32_t)0x2UL) /**< PCLKDIV_PCF_96MHZ Value */
#define MXC_S_GCR_PCLKDIV_PCF_96MHZ                    (MXC_V_GCR_PCLKDIV_PCF_96MHZ << MXC_F_GCR_PCLKDIV_PCF_POS) /**< PCLKDIV_PCF_96MHZ Setting */
#define MXC_V_GCR_PCLKDIV_PCF_48MHZ                    ((uint32_t)0x3UL) /**< PCLKDIV_PCF_48MHZ Value */
#define MXC_S_GCR_PCLKDIV_PCF_48MHZ                    (MXC_V_GCR_PCLKDIV_PCF_48MHZ << MXC_F_GCR_PCLKDIV_PCF_POS) /**< PCLKDIV_PCF_48MHZ Setting */
#define MXC_V_GCR_PCLKDIV_PCF_24MHZ                    ((uint32_t)0x4UL) /**< PCLKDIV_PCF_24MHZ Value */
#define MXC_S_GCR_PCLKDIV_PCF_24MHZ                    (MXC_V_GCR_PCLKDIV_PCF_24MHZ << MXC_F_GCR_PCLKDIV_PCF_POS) /**< PCLKDIV_PCF_24MHZ Setting */
#define MXC_V_GCR_PCLKDIV_PCF_12MHZ                    ((uint32_t)0x5UL) /**< PCLKDIV_PCF_12MHZ Value */
#define MXC_S_GCR_PCLKDIV_PCF_12MHZ                    (MXC_V_GCR_PCLKDIV_PCF_12MHZ << MXC_F_GCR_PCLKDIV_PCF_POS) /**< PCLKDIV_PCF_12MHZ Setting */
#define MXC_V_GCR_PCLKDIV_PCF_6MHZ                     ((uint32_t)0x6UL) /**< PCLKDIV_PCF_6MHZ Value */
#define MXC_S_GCR_PCLKDIV_PCF_6MHZ                     (MXC_V_GCR_PCLKDIV_PCF_6MHZ << MXC_F_GCR_PCLKDIV_PCF_POS) /**< PCLKDIV_PCF_6MHZ Setting */
#define MXC_V_GCR_PCLKDIV_PCF_3MHZ                     ((uint32_t)0x7UL) /**< PCLKDIV_PCF_3MHZ Value */
#define MXC_S_GCR_PCLKDIV_PCF_3MHZ                     (MXC_V_GCR_PCLKDIV_PCF_3MHZ << MXC_F_GCR_PCLKDIV_PCF_POS) /**< PCLKDIV_PCF_3MHZ Setting */

#define MXC_F_GCR_PCLKDIV_PCFWEN_POS                   3 /**< PCLKDIV_PCFWEN Position */
#define MXC_F_GCR_PCLKDIV_PCFWEN                       ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIV_PCFWEN_POS)) /**< PCLKDIV_PCFWEN Mask */

#define MXC_F_GCR_PCLKDIV_SDHCFRQ_POS                  7 /**< PCLKDIV_SDHCFRQ Position */
#define MXC_F_GCR_PCLKDIV_SDHCFRQ                      ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIV_SDHCFRQ_POS)) /**< PCLKDIV_SDHCFRQ Mask */

#define MXC_F_GCR_PCLKDIV_ADCFRQ_POS                   10 /**< PCLKDIV_ADCFRQ Position */
#define MXC_F_GCR_PCLKDIV_ADCFRQ                       ((uint32_t)(0xFUL << MXC_F_GCR_PCLKDIV_ADCFRQ_POS)) /**< PCLKDIV_ADCFRQ Mask */

#define MXC_F_GCR_PCLKDIV_AON_CLKDIV_POS               14 /**< PCLKDIV_AON_CLKDIV Position */
#define MXC_F_GCR_PCLKDIV_AON_CLKDIV                   ((uint32_t)(0x3UL << MXC_F_GCR_PCLKDIV_AON_CLKDIV_POS)) /**< PCLKDIV_AON_CLKDIV Mask */
#define MXC_V_GCR_PCLKDIV_AON_CLKDIV_DIV_4             ((uint32_t)0x0UL) /**< PCLKDIV_AON_CLKDIV_DIV_4 Value */
#define MXC_S_GCR_PCLKDIV_AON_CLKDIV_DIV_4             (MXC_V_GCR_PCLKDIV_AON_CLKDIV_DIV_4 << MXC_F_GCR_PCLKDIV_AON_CLKDIV_POS) /**< PCLKDIV_AON_CLKDIV_DIV_4 Setting */
#define MXC_V_GCR_PCLKDIV_AON_CLKDIV_DIV_8             ((uint32_t)0x1UL) /**< PCLKDIV_AON_CLKDIV_DIV_8 Value */
#define MXC_S_GCR_PCLKDIV_AON_CLKDIV_DIV_8             (MXC_V_GCR_PCLKDIV_AON_CLKDIV_DIV_8 << MXC_F_GCR_PCLKDIV_AON_CLKDIV_POS) /**< PCLKDIV_AON_CLKDIV_DIV_8 Setting */
#define MXC_V_GCR_PCLKDIV_AON_CLKDIV_DIV_16            ((uint32_t)0x2UL) /**< PCLKDIV_AON_CLKDIV_DIV_16 Value */
#define MXC_S_GCR_PCLKDIV_AON_CLKDIV_DIV_16            (MXC_V_GCR_PCLKDIV_AON_CLKDIV_DIV_16 << MXC_F_GCR_PCLKDIV_AON_CLKDIV_POS) /**< PCLKDIV_AON_CLKDIV_DIV_16 Setting */
#define MXC_V_GCR_PCLKDIV_AON_CLKDIV_DIV_32            ((uint32_t)0x3UL) /**< PCLKDIV_AON_CLKDIV_DIV_32 Value */
#define MXC_S_GCR_PCLKDIV_AON_CLKDIV_DIV_32            (MXC_V_GCR_PCLKDIV_AON_CLKDIV_DIV_32 << MXC_F_GCR_PCLKDIV_AON_CLKDIV_POS) /**< PCLKDIV_AON_CLKDIV_DIV_32 Setting */

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

#define MXC_F_GCR_PCLKDIS0_GPIO2_POS                   2 /**< PCLKDIS0_GPIO2 Position */
#define MXC_F_GCR_PCLKDIS0_GPIO2                       ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS0_GPIO2_POS)) /**< PCLKDIS0_GPIO2 Mask */

#define MXC_F_GCR_PCLKDIS0_USB_POS                     3 /**< PCLKDIS0_USB Position */
#define MXC_F_GCR_PCLKDIS0_USB                         ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS0_USB_POS)) /**< PCLKDIS0_USB Mask */

#define MXC_F_GCR_PCLKDIS0_CLCD_POS                    4 /**< PCLKDIS0_CLCD Position */
#define MXC_F_GCR_PCLKDIS0_CLCD                        ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS0_CLCD_POS)) /**< PCLKDIS0_CLCD Mask */

#define MXC_F_GCR_PCLKDIS0_DMA_POS                     5 /**< PCLKDIS0_DMA Position */
#define MXC_F_GCR_PCLKDIS0_DMA                         ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS0_DMA_POS)) /**< PCLKDIS0_DMA Mask */

#define MXC_F_GCR_PCLKDIS0_SPI0_POS                    6 /**< PCLKDIS0_SPI0 Position */
#define MXC_F_GCR_PCLKDIS0_SPI0                        ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS0_SPI0_POS)) /**< PCLKDIS0_SPI0 Mask */

#define MXC_F_GCR_PCLKDIS0_SPI1_POS                    7 /**< PCLKDIS0_SPI1 Position */
#define MXC_F_GCR_PCLKDIS0_SPI1                        ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS0_SPI1_POS)) /**< PCLKDIS0_SPI1 Mask */

#define MXC_F_GCR_PCLKDIS0_SPI2_POS                    8 /**< PCLKDIS0_SPI2 Position */
#define MXC_F_GCR_PCLKDIS0_SPI2                        ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS0_SPI2_POS)) /**< PCLKDIS0_SPI2 Mask */

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

#define MXC_F_GCR_PCLKDIS0_KBD_POS                     22 /**< PCLKDIS0_KBD Position */
#define MXC_F_GCR_PCLKDIS0_KBD                         ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS0_KBD_POS)) /**< PCLKDIS0_KBD Mask */

#define MXC_F_GCR_PCLKDIS0_ADC_POS                     23 /**< PCLKDIS0_ADC Position */
#define MXC_F_GCR_PCLKDIS0_ADC                         ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS0_ADC_POS)) /**< PCLKDIS0_ADC Mask */

#define MXC_F_GCR_PCLKDIS0_TMR6_POS                    24 /**< PCLKDIS0_TMR6 Position */
#define MXC_F_GCR_PCLKDIS0_TMR6                        ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS0_TMR6_POS)) /**< PCLKDIS0_TMR6 Mask */

#define MXC_F_GCR_PCLKDIS0_TMR7_POS                    25 /**< PCLKDIS0_TMR7 Position */
#define MXC_F_GCR_PCLKDIS0_TMR7                        ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS0_TMR7_POS)) /**< PCLKDIS0_TMR7 Mask */

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

#define MXC_F_GCR_PCLKDIS0_SPIM_POS                    31 /**< PCLKDIS0_SPIM Position */
#define MXC_F_GCR_PCLKDIS0_SPIM                        ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS0_SPIM_POS)) /**< PCLKDIS0_SPIM Mask */

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

#define MXC_F_GCR_MEMCTRL_ICC0LS_EN_POS                24 /**< MEMCTRL_ICC0LS_EN Position */
#define MXC_F_GCR_MEMCTRL_ICC0LS_EN                    ((uint32_t)(0x1UL << MXC_F_GCR_MEMCTRL_ICC0LS_EN_POS)) /**< MEMCTRL_ICC0LS_EN Mask */

#define MXC_F_GCR_MEMCTRL_ICCXIPLS_EN_POS              25 /**< MEMCTRL_ICCXIPLS_EN Position */
#define MXC_F_GCR_MEMCTRL_ICCXIPLS_EN                  ((uint32_t)(0x1UL << MXC_F_GCR_MEMCTRL_ICCXIPLS_EN_POS)) /**< MEMCTRL_ICCXIPLS_EN Mask */

#define MXC_F_GCR_MEMCTRL_SRCCLS_EN_POS                26 /**< MEMCTRL_SRCCLS_EN Position */
#define MXC_F_GCR_MEMCTRL_SRCCLS_EN                    ((uint32_t)(0x1UL << MXC_F_GCR_MEMCTRL_SRCCLS_EN_POS)) /**< MEMCTRL_SRCCLS_EN Mask */

#define MXC_F_GCR_MEMCTRL_CRYPTOLS_EN_POS              27 /**< MEMCTRL_CRYPTOLS_EN Position */
#define MXC_F_GCR_MEMCTRL_CRYPTOLS_EN                  ((uint32_t)(0x1UL << MXC_F_GCR_MEMCTRL_CRYPTOLS_EN_POS)) /**< MEMCTRL_CRYPTOLS_EN Mask */

#define MXC_F_GCR_MEMCTRL_USBLS_EN_POS                 28 /**< MEMCTRL_USBLS_EN Position */
#define MXC_F_GCR_MEMCTRL_USBLS_EN                     ((uint32_t)(0x1UL << MXC_F_GCR_MEMCTRL_USBLS_EN_POS)) /**< MEMCTRL_USBLS_EN Mask */

#define MXC_F_GCR_MEMCTRL_ROMLS_EN_POS                 29 /**< MEMCTRL_ROMLS_EN Position */
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

#define MXC_F_GCR_MEMZ_RAM3_POS                        3 /**< MEMZ_RAM3 Position */
#define MXC_F_GCR_MEMZ_RAM3                            ((uint32_t)(0x1UL << MXC_F_GCR_MEMZ_RAM3_POS)) /**< MEMZ_RAM3 Mask */

#define MXC_F_GCR_MEMZ_RAM4_POS                        4 /**< MEMZ_RAM4 Position */
#define MXC_F_GCR_MEMZ_RAM4                            ((uint32_t)(0x1UL << MXC_F_GCR_MEMZ_RAM4_POS)) /**< MEMZ_RAM4 Mask */

#define MXC_F_GCR_MEMZ_RAM5_POS                        5 /**< MEMZ_RAM5 Position */
#define MXC_F_GCR_MEMZ_RAM5                            ((uint32_t)(0x1UL << MXC_F_GCR_MEMZ_RAM5_POS)) /**< MEMZ_RAM5 Mask */

#define MXC_F_GCR_MEMZ_RAM6_POS                        6 /**< MEMZ_RAM6 Position */
#define MXC_F_GCR_MEMZ_RAM6                            ((uint32_t)(0x1UL << MXC_F_GCR_MEMZ_RAM6_POS)) /**< MEMZ_RAM6 Mask */

#define MXC_F_GCR_MEMZ_ICC0_POS                        8 /**< MEMZ_ICC0 Position */
#define MXC_F_GCR_MEMZ_ICC0                            ((uint32_t)(0x1UL << MXC_F_GCR_MEMZ_ICC0_POS)) /**< MEMZ_ICC0 Mask */

#define MXC_F_GCR_MEMZ_ICCXIP_POS                      9 /**< MEMZ_ICCXIP Position */
#define MXC_F_GCR_MEMZ_ICCXIP                          ((uint32_t)(0x1UL << MXC_F_GCR_MEMZ_ICCXIP_POS)) /**< MEMZ_ICCXIP Mask */

#define MXC_F_GCR_MEMZ_SCACHEDATA_POS                  10 /**< MEMZ_SCACHEDATA Position */
#define MXC_F_GCR_MEMZ_SCACHEDATA                      ((uint32_t)(0x1UL << MXC_F_GCR_MEMZ_SCACHEDATA_POS)) /**< MEMZ_SCACHEDATA Mask */

#define MXC_F_GCR_MEMZ_SCACHETAG_POS                   11 /**< MEMZ_SCACHETAG Position */
#define MXC_F_GCR_MEMZ_SCACHETAG                       ((uint32_t)(0x1UL << MXC_F_GCR_MEMZ_SCACHETAG_POS)) /**< MEMZ_SCACHETAG Mask */

#define MXC_F_GCR_MEMZ_CRYPTO_POS                      12 /**< MEMZ_CRYPTO Position */
#define MXC_F_GCR_MEMZ_CRYPTO                          ((uint32_t)(0x1UL << MXC_F_GCR_MEMZ_CRYPTO_POS)) /**< MEMZ_CRYPTO Mask */

#define MXC_F_GCR_MEMZ_USBFIFO_POS                     13 /**< MEMZ_USBFIFO Position */
#define MXC_F_GCR_MEMZ_USBFIFO                         ((uint32_t)(0x1UL << MXC_F_GCR_MEMZ_USBFIFO_POS)) /**< MEMZ_USBFIFO Mask */

/**@} end of group GCR_MEMZ_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_SCCK GCR_SCCK
 * @brief    Smart Card Clock Control.
 * @{
 */
#define MXC_F_GCR_SCCK_SC0CD_POS                       0 /**< SCCK_SC0CD Position */
#define MXC_F_GCR_SCCK_SC0CD                           ((uint32_t)(0x3FUL << MXC_F_GCR_SCCK_SC0CD_POS)) /**< SCCK_SC0CD Mask */

#define MXC_F_GCR_SCCK_SC1CD_POS                       8 /**< SCCK_SC1CD Position */
#define MXC_F_GCR_SCCK_SC1CD                           ((uint32_t)(0x3FUL << MXC_F_GCR_SCCK_SC1CD_POS)) /**< SCCK_SC1CD Mask */

/**@} end of group GCR_SCCK_Register */

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

#define MXC_F_GCR_RST1_XSPIM_POS                       4 /**< RST1_XSPIM Position */
#define MXC_F_GCR_RST1_XSPIM                           ((uint32_t)(0x1UL << MXC_F_GCR_RST1_XSPIM_POS)) /**< RST1_XSPIM Mask */

#define MXC_F_GCR_RST1_GPIO3_POS                       5 /**< RST1_GPIO3 Position */
#define MXC_F_GCR_RST1_GPIO3                           ((uint32_t)(0x1UL << MXC_F_GCR_RST1_GPIO3_POS)) /**< RST1_GPIO3 Mask */

#define MXC_F_GCR_RST1_SDHC_POS                        6 /**< RST1_SDHC Position */
#define MXC_F_GCR_RST1_SDHC                            ((uint32_t)(0x1UL << MXC_F_GCR_RST1_SDHC_POS)) /**< RST1_SDHC Mask */

#define MXC_F_GCR_RST1_OWIRE_POS                       7 /**< RST1_OWIRE Position */
#define MXC_F_GCR_RST1_OWIRE                           ((uint32_t)(0x1UL << MXC_F_GCR_RST1_OWIRE_POS)) /**< RST1_OWIRE Mask */

#define MXC_F_GCR_RST1_WDT1_POS                        8 /**< RST1_WDT1 Position */
#define MXC_F_GCR_RST1_WDT1                            ((uint32_t)(0x1UL << MXC_F_GCR_RST1_WDT1_POS)) /**< RST1_WDT1 Mask */

#define MXC_F_GCR_RST1_SPI3_POS                        9 /**< RST1_SPI3 Position */
#define MXC_F_GCR_RST1_SPI3                            ((uint32_t)(0x1UL << MXC_F_GCR_RST1_SPI3_POS)) /**< RST1_SPI3 Mask */

#define MXC_F_GCR_RST1_AC_POS                          14 /**< RST1_AC Position */
#define MXC_F_GCR_RST1_AC                              ((uint32_t)(0x1UL << MXC_F_GCR_RST1_AC_POS)) /**< RST1_AC Mask */

#define MXC_F_GCR_RST1_SPIXMEM_POS                     15 /**< RST1_SPIXMEM Position */
#define MXC_F_GCR_RST1_SPIXMEM                         ((uint32_t)(0x1UL << MXC_F_GCR_RST1_SPIXMEM_POS)) /**< RST1_SPIXMEM Mask */

#define MXC_F_GCR_RST1_I2C2_POS                        17 /**< RST1_I2C2 Position */
#define MXC_F_GCR_RST1_I2C2                            ((uint32_t)(0x1UL << MXC_F_GCR_RST1_I2C2_POS)) /**< RST1_I2C2 Mask */

#define MXC_F_GCR_RST1_UART3_POS                       18 /**< RST1_UART3 Position */
#define MXC_F_GCR_RST1_UART3                           ((uint32_t)(0x1UL << MXC_F_GCR_RST1_UART3_POS)) /**< RST1_UART3 Mask */

#define MXC_F_GCR_RST1_UART4_POS                       19 /**< RST1_UART4 Position */
#define MXC_F_GCR_RST1_UART4                           ((uint32_t)(0x1UL << MXC_F_GCR_RST1_UART4_POS)) /**< RST1_UART4 Mask */

#define MXC_F_GCR_RST1_UART5_POS                       20 /**< RST1_UART5 Position */
#define MXC_F_GCR_RST1_UART5                           ((uint32_t)(0x1UL << MXC_F_GCR_RST1_UART5_POS)) /**< RST1_UART5 Mask */

#define MXC_F_GCR_RST1_KBD_POS                         21 /**< RST1_KBD Position */
#define MXC_F_GCR_RST1_KBD                             ((uint32_t)(0x1UL << MXC_F_GCR_RST1_KBD_POS)) /**< RST1_KBD Mask */

#define MXC_F_GCR_RST1_ADC9_POS                        22 /**< RST1_ADC9 Position */
#define MXC_F_GCR_RST1_ADC9                            ((uint32_t)(0x1UL << MXC_F_GCR_RST1_ADC9_POS)) /**< RST1_ADC9 Mask */

#define MXC_F_GCR_RST1_SC0_POS                         23 /**< RST1_SC0 Position */
#define MXC_F_GCR_RST1_SC0                             ((uint32_t)(0x1UL << MXC_F_GCR_RST1_SC0_POS)) /**< RST1_SC0 Mask */

#define MXC_F_GCR_RST1_SC1_POS                         24 /**< RST1_SC1 Position */
#define MXC_F_GCR_RST1_SC1                             ((uint32_t)(0x1UL << MXC_F_GCR_RST1_SC1_POS)) /**< RST1_SC1 Mask */

#define MXC_F_GCR_RST1_NFC_POS                         25 /**< RST1_NFC Position */
#define MXC_F_GCR_RST1_NFC                             ((uint32_t)(0x1UL << MXC_F_GCR_RST1_NFC_POS)) /**< RST1_NFC Mask */

#define MXC_F_GCR_RST1_EMAC_POS                        26 /**< RST1_EMAC Position */
#define MXC_F_GCR_RST1_EMAC                            ((uint32_t)(0x1UL << MXC_F_GCR_RST1_EMAC_POS)) /**< RST1_EMAC Mask */

#define MXC_F_GCR_RST1_PCIF_POS                        27 /**< RST1_PCIF Position */
#define MXC_F_GCR_RST1_PCIF                            ((uint32_t)(0x1UL << MXC_F_GCR_RST1_PCIF_POS)) /**< RST1_PCIF Mask */

#define MXC_F_GCR_RST1_HTMR0_POS                       28 /**< RST1_HTMR0 Position */
#define MXC_F_GCR_RST1_HTMR0                           ((uint32_t)(0x1UL << MXC_F_GCR_RST1_HTMR0_POS)) /**< RST1_HTMR0 Mask */

#define MXC_F_GCR_RST1_HTMR1_POS                       29 /**< RST1_HTMR1 Position */
#define MXC_F_GCR_RST1_HTMR1                           ((uint32_t)(0x1UL << MXC_F_GCR_RST1_HTMR1_POS)) /**< RST1_HTMR1 Mask */

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

#define MXC_F_GCR_PCLKDIS1_WDT0_POS                    4 /**< PCLKDIS1_WDT0 Position */
#define MXC_F_GCR_PCLKDIS1_WDT0                        ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS1_WDT0_POS)) /**< PCLKDIS1_WDT0 Mask */

#define MXC_F_GCR_PCLKDIS1_WDT1_POS                    5 /**< PCLKDIS1_WDT1 Position */
#define MXC_F_GCR_PCLKDIS1_WDT1                        ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS1_WDT1_POS)) /**< PCLKDIS1_WDT1 Mask */

#define MXC_F_GCR_PCLKDIS1_GPIO3_POS                   6 /**< PCLKDIS1_GPIO3 Position */
#define MXC_F_GCR_PCLKDIS1_GPIO3                       ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS1_GPIO3_POS)) /**< PCLKDIS1_GPIO3 Mask */

#define MXC_F_GCR_PCLKDIS1_SCACHE_POS                  7 /**< PCLKDIS1_SCACHE Position */
#define MXC_F_GCR_PCLKDIS1_SCACHE                      ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS1_SCACHE_POS)) /**< PCLKDIS1_SCACHE Mask */

#define MXC_F_GCR_PCLKDIS1_HA0_POS                     8 /**< PCLKDIS1_HA0 Position */
#define MXC_F_GCR_PCLKDIS1_HA0                         ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS1_HA0_POS)) /**< PCLKDIS1_HA0 Mask */

#define MXC_F_GCR_PCLKDIS1_SDHC_POS                    10 /**< PCLKDIS1_SDHC Position */
#define MXC_F_GCR_PCLKDIS1_SDHC                        ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS1_SDHC_POS)) /**< PCLKDIS1_SDHC Mask */

#define MXC_F_GCR_PCLKDIS1_ICC0_POS                    11 /**< PCLKDIS1_ICC0 Position */
#define MXC_F_GCR_PCLKDIS1_ICC0                        ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS1_ICC0_POS)) /**< PCLKDIS1_ICC0 Mask */

#define MXC_F_GCR_PCLKDIS1_ICCXIP_POS                  12 /**< PCLKDIS1_ICCXIP Position */
#define MXC_F_GCR_PCLKDIS1_ICCXIP                      ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS1_ICCXIP_POS)) /**< PCLKDIS1_ICCXIP Mask */

#define MXC_F_GCR_PCLKDIS1_OWIRE_POS                   13 /**< PCLKDIS1_OWIRE Position */
#define MXC_F_GCR_PCLKDIS1_OWIRE                       ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS1_OWIRE_POS)) /**< PCLKDIS1_OWIRE Mask */

#define MXC_F_GCR_PCLKDIS1_SPI3_POS                    14 /**< PCLKDIS1_SPI3 Position */
#define MXC_F_GCR_PCLKDIS1_SPI3                        ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS1_SPI3_POS)) /**< PCLKDIS1_SPI3 Mask */

#define MXC_F_GCR_PCLKDIS1_SPIXIP_POS                  20 /**< PCLKDIS1_SPIXIP Position */
#define MXC_F_GCR_PCLKDIS1_SPIXIP                      ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS1_SPIXIP_POS)) /**< PCLKDIS1_SPIXIP Mask */

#define MXC_F_GCR_PCLKDIS1_I2C2_POS                    21 /**< PCLKDIS1_I2C2 Position */
#define MXC_F_GCR_PCLKDIS1_I2C2                        ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS1_I2C2_POS)) /**< PCLKDIS1_I2C2 Mask */

#define MXC_F_GCR_PCLKDIS1_UART3_POS                   22 /**< PCLKDIS1_UART3 Position */
#define MXC_F_GCR_PCLKDIS1_UART3                       ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS1_UART3_POS)) /**< PCLKDIS1_UART3 Mask */

#define MXC_F_GCR_PCLKDIS1_UART4_POS                   23 /**< PCLKDIS1_UART4 Position */
#define MXC_F_GCR_PCLKDIS1_UART4                       ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS1_UART4_POS)) /**< PCLKDIS1_UART4 Mask */

#define MXC_F_GCR_PCLKDIS1_UART5_POS                   24 /**< PCLKDIS1_UART5 Position */
#define MXC_F_GCR_PCLKDIS1_UART5                       ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS1_UART5_POS)) /**< PCLKDIS1_UART5 Mask */

#define MXC_F_GCR_PCLKDIS1_ADC9_POS                    25 /**< PCLKDIS1_ADC9 Position */
#define MXC_F_GCR_PCLKDIS1_ADC9                        ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS1_ADC9_POS)) /**< PCLKDIS1_ADC9 Mask */

#define MXC_F_GCR_PCLKDIS1_SC0_POS                     26 /**< PCLKDIS1_SC0 Position */
#define MXC_F_GCR_PCLKDIS1_SC0                         ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS1_SC0_POS)) /**< PCLKDIS1_SC0 Mask */

#define MXC_F_GCR_PCLKDIS1_SC1_POS                     27 /**< PCLKDIS1_SC1 Position */
#define MXC_F_GCR_PCLKDIS1_SC1                         ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS1_SC1_POS)) /**< PCLKDIS1_SC1 Mask */

#define MXC_F_GCR_PCLKDIS1_NFC_POS                     28 /**< PCLKDIS1_NFC Position */
#define MXC_F_GCR_PCLKDIS1_NFC                         ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS1_NFC_POS)) /**< PCLKDIS1_NFC Mask */

#define MXC_F_GCR_PCLKDIS1_EMAC_POS                    29 /**< PCLKDIS1_EMAC Position */
#define MXC_F_GCR_PCLKDIS1_EMAC                        ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS1_EMAC_POS)) /**< PCLKDIS1_EMAC Mask */

#define MXC_F_GCR_PCLKDIS1_HA1_POS                     30 /**< PCLKDIS1_HA1 Position */
#define MXC_F_GCR_PCLKDIS1_HA1                         ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS1_HA1_POS)) /**< PCLKDIS1_HA1 Mask */

#define MXC_F_GCR_PCLKDIS1_PCIF_POS                    31 /**< PCLKDIS1_PCIF Position */
#define MXC_F_GCR_PCLKDIS1_PCIF                        ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS1_PCIF_POS)) /**< PCLKDIS1_PCIF Mask */

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

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_ECCERR GCR_ECCERR
 * @brief    ECC Error Register
 * @{
 */
#define MXC_F_GCR_ECCERR_RAM0_POS                      0 /**< ECCERR_RAM0 Position */
#define MXC_F_GCR_ECCERR_RAM0                          ((uint32_t)(0x1UL << MXC_F_GCR_ECCERR_RAM0_POS)) /**< ECCERR_RAM0 Mask */

#define MXC_F_GCR_ECCERR_RAM1_POS                      1 /**< ECCERR_RAM1 Position */
#define MXC_F_GCR_ECCERR_RAM1                          ((uint32_t)(0x1UL << MXC_F_GCR_ECCERR_RAM1_POS)) /**< ECCERR_RAM1 Mask */

#define MXC_F_GCR_ECCERR_RAM2_POS                      2 /**< ECCERR_RAM2 Position */
#define MXC_F_GCR_ECCERR_RAM2                          ((uint32_t)(0x1UL << MXC_F_GCR_ECCERR_RAM2_POS)) /**< ECCERR_RAM2 Mask */

#define MXC_F_GCR_ECCERR_RAM3_POS                      3 /**< ECCERR_RAM3 Position */
#define MXC_F_GCR_ECCERR_RAM3                          ((uint32_t)(0x1UL << MXC_F_GCR_ECCERR_RAM3_POS)) /**< ECCERR_RAM3 Mask */

#define MXC_F_GCR_ECCERR_RAM4_POS                      4 /**< ECCERR_RAM4 Position */
#define MXC_F_GCR_ECCERR_RAM4                          ((uint32_t)(0x1UL << MXC_F_GCR_ECCERR_RAM4_POS)) /**< ECCERR_RAM4 Mask */

#define MXC_F_GCR_ECCERR_RAM5_POS                      5 /**< ECCERR_RAM5 Position */
#define MXC_F_GCR_ECCERR_RAM5                          ((uint32_t)(0x1UL << MXC_F_GCR_ECCERR_RAM5_POS)) /**< ECCERR_RAM5 Mask */

#define MXC_F_GCR_ECCERR_ICC0_POS                      8 /**< ECCERR_ICC0 Position */
#define MXC_F_GCR_ECCERR_ICC0                          ((uint32_t)(0x1UL << MXC_F_GCR_ECCERR_ICC0_POS)) /**< ECCERR_ICC0 Mask */

#define MXC_F_GCR_ECCERR_ICSPIXF_POS                   10 /**< ECCERR_ICSPIXF Position */
#define MXC_F_GCR_ECCERR_ICSPIXF                       ((uint32_t)(0x1UL << MXC_F_GCR_ECCERR_ICSPIXF_POS)) /**< ECCERR_ICSPIXF Mask */

#define MXC_F_GCR_ECCERR_FLASH0_POS                    11 /**< ECCERR_FLASH0 Position */
#define MXC_F_GCR_ECCERR_FLASH0                        ((uint32_t)(0x1UL << MXC_F_GCR_ECCERR_FLASH0_POS)) /**< ECCERR_FLASH0 Mask */

#define MXC_F_GCR_ECCERR_FLASH1_POS                    12 /**< ECCERR_FLASH1 Position */
#define MXC_F_GCR_ECCERR_FLASH1                        ((uint32_t)(0x1UL << MXC_F_GCR_ECCERR_FLASH1_POS)) /**< ECCERR_FLASH1 Mask */

/**@} end of group GCR_ECCERR_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_ECCCED GCR_ECCCED
 * @brief    ECC Not Double Error Detect Register
 * @{
 */
#define MXC_F_GCR_ECCCED_RAM0_POS                      0 /**< ECCCED_RAM0 Position */
#define MXC_F_GCR_ECCCED_RAM0                          ((uint32_t)(0x1UL << MXC_F_GCR_ECCCED_RAM0_POS)) /**< ECCCED_RAM0 Mask */

#define MXC_F_GCR_ECCCED_RAM1_POS                      1 /**< ECCCED_RAM1 Position */
#define MXC_F_GCR_ECCCED_RAM1                          ((uint32_t)(0x1UL << MXC_F_GCR_ECCCED_RAM1_POS)) /**< ECCCED_RAM1 Mask */

#define MXC_F_GCR_ECCCED_RAM2_POS                      2 /**< ECCCED_RAM2 Position */
#define MXC_F_GCR_ECCCED_RAM2                          ((uint32_t)(0x1UL << MXC_F_GCR_ECCCED_RAM2_POS)) /**< ECCCED_RAM2 Mask */

#define MXC_F_GCR_ECCCED_RAM3_POS                      3 /**< ECCCED_RAM3 Position */
#define MXC_F_GCR_ECCCED_RAM3                          ((uint32_t)(0x1UL << MXC_F_GCR_ECCCED_RAM3_POS)) /**< ECCCED_RAM3 Mask */

#define MXC_F_GCR_ECCCED_RAM4_POS                      4 /**< ECCCED_RAM4 Position */
#define MXC_F_GCR_ECCCED_RAM4                          ((uint32_t)(0x1UL << MXC_F_GCR_ECCCED_RAM4_POS)) /**< ECCCED_RAM4 Mask */

#define MXC_F_GCR_ECCCED_RAM5_POS                      5 /**< ECCCED_RAM5 Position */
#define MXC_F_GCR_ECCCED_RAM5                          ((uint32_t)(0x1UL << MXC_F_GCR_ECCCED_RAM5_POS)) /**< ECCCED_RAM5 Mask */

#define MXC_F_GCR_ECCCED_ICC0_POS                      8 /**< ECCCED_ICC0 Position */
#define MXC_F_GCR_ECCCED_ICC0                          ((uint32_t)(0x1UL << MXC_F_GCR_ECCCED_ICC0_POS)) /**< ECCCED_ICC0 Mask */

#define MXC_F_GCR_ECCCED_ICSPIXF_POS                   10 /**< ECCCED_ICSPIXF Position */
#define MXC_F_GCR_ECCCED_ICSPIXF                       ((uint32_t)(0x1UL << MXC_F_GCR_ECCCED_ICSPIXF_POS)) /**< ECCCED_ICSPIXF Mask */

#define MXC_F_GCR_ECCCED_FLASH0_POS                    11 /**< ECCCED_FLASH0 Position */
#define MXC_F_GCR_ECCCED_FLASH0                        ((uint32_t)(0x1UL << MXC_F_GCR_ECCCED_FLASH0_POS)) /**< ECCCED_FLASH0 Mask */

#define MXC_F_GCR_ECCCED_FLASH1_POS                    12 /**< ECCCED_FLASH1 Position */
#define MXC_F_GCR_ECCCED_FLASH1                        ((uint32_t)(0x1UL << MXC_F_GCR_ECCCED_FLASH1_POS)) /**< ECCCED_FLASH1 Mask */

/**@} end of group GCR_ECCCED_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_ECCIE GCR_ECCIE
 * @brief    ECC IRQ Enable Register
 * @{
 */
#define MXC_F_GCR_ECCIE_RAM0_POS                       0 /**< ECCIE_RAM0 Position */
#define MXC_F_GCR_ECCIE_RAM0                           ((uint32_t)(0x1UL << MXC_F_GCR_ECCIE_RAM0_POS)) /**< ECCIE_RAM0 Mask */

#define MXC_F_GCR_ECCIE_RAM1_POS                       1 /**< ECCIE_RAM1 Position */
#define MXC_F_GCR_ECCIE_RAM1                           ((uint32_t)(0x1UL << MXC_F_GCR_ECCIE_RAM1_POS)) /**< ECCIE_RAM1 Mask */

#define MXC_F_GCR_ECCIE_RAM2_POS                       2 /**< ECCIE_RAM2 Position */
#define MXC_F_GCR_ECCIE_RAM2                           ((uint32_t)(0x1UL << MXC_F_GCR_ECCIE_RAM2_POS)) /**< ECCIE_RAM2 Mask */

#define MXC_F_GCR_ECCIE_RAM3_POS                       3 /**< ECCIE_RAM3 Position */
#define MXC_F_GCR_ECCIE_RAM3                           ((uint32_t)(0x1UL << MXC_F_GCR_ECCIE_RAM3_POS)) /**< ECCIE_RAM3 Mask */

#define MXC_F_GCR_ECCIE_RAM4_POS                       4 /**< ECCIE_RAM4 Position */
#define MXC_F_GCR_ECCIE_RAM4                           ((uint32_t)(0x1UL << MXC_F_GCR_ECCIE_RAM4_POS)) /**< ECCIE_RAM4 Mask */

#define MXC_F_GCR_ECCIE_RAM5_POS                       5 /**< ECCIE_RAM5 Position */
#define MXC_F_GCR_ECCIE_RAM5                           ((uint32_t)(0x1UL << MXC_F_GCR_ECCIE_RAM5_POS)) /**< ECCIE_RAM5 Mask */

#define MXC_F_GCR_ECCIE_ICC0_POS                       8 /**< ECCIE_ICC0 Position */
#define MXC_F_GCR_ECCIE_ICC0                           ((uint32_t)(0x1UL << MXC_F_GCR_ECCIE_ICC0_POS)) /**< ECCIE_ICC0 Mask */

#define MXC_F_GCR_ECCIE_ICSPIXF_POS                    10 /**< ECCIE_ICSPIXF Position */
#define MXC_F_GCR_ECCIE_ICSPIXF                        ((uint32_t)(0x1UL << MXC_F_GCR_ECCIE_ICSPIXF_POS)) /**< ECCIE_ICSPIXF Mask */

#define MXC_F_GCR_ECCIE_FLASH0_POS                     11 /**< ECCIE_FLASH0 Position */
#define MXC_F_GCR_ECCIE_FLASH0                         ((uint32_t)(0x1UL << MXC_F_GCR_ECCIE_FLASH0_POS)) /**< ECCIE_FLASH0 Mask */

#define MXC_F_GCR_ECCIE_FLASH1_POS                     12 /**< ECCIE_FLASH1 Position */
#define MXC_F_GCR_ECCIE_FLASH1                         ((uint32_t)(0x1UL << MXC_F_GCR_ECCIE_FLASH1_POS)) /**< ECCIE_FLASH1 Mask */

/**@} end of group GCR_ECCIE_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_ECCADDR GCR_ECCADDR
 * @brief    ECC Error Address Register
 * @{
 */
#define MXC_F_GCR_ECCADDR_DATARAMADDR_POS              0 /**< ECCADDR_DATARAMADDR Position */
#define MXC_F_GCR_ECCADDR_DATARAMADDR                  ((uint32_t)(0x3FFFUL << MXC_F_GCR_ECCADDR_DATARAMADDR_POS)) /**< ECCADDR_DATARAMADDR Mask */

#define MXC_F_GCR_ECCADDR_DATARAMBANK_POS              14 /**< ECCADDR_DATARAMBANK Position */
#define MXC_F_GCR_ECCADDR_DATARAMBANK                  ((uint32_t)(0x1UL << MXC_F_GCR_ECCADDR_DATARAMBANK_POS)) /**< ECCADDR_DATARAMBANK Mask */

#define MXC_F_GCR_ECCADDR_DATARAMERR_POS               15 /**< ECCADDR_DATARAMERR Position */
#define MXC_F_GCR_ECCADDR_DATARAMERR                   ((uint32_t)(0x1UL << MXC_F_GCR_ECCADDR_DATARAMERR_POS)) /**< ECCADDR_DATARAMERR Mask */

#define MXC_F_GCR_ECCADDR_TAGRAMADDR_POS               16 /**< ECCADDR_TAGRAMADDR Position */
#define MXC_F_GCR_ECCADDR_TAGRAMADDR                   ((uint32_t)(0x3FFFUL << MXC_F_GCR_ECCADDR_TAGRAMADDR_POS)) /**< ECCADDR_TAGRAMADDR Mask */

#define MXC_F_GCR_ECCADDR_TAGRAMBANK_POS               30 /**< ECCADDR_TAGRAMBANK Position */
#define MXC_F_GCR_ECCADDR_TAGRAMBANK                   ((uint32_t)(0x1UL << MXC_F_GCR_ECCADDR_TAGRAMBANK_POS)) /**< ECCADDR_TAGRAMBANK Mask */

#define MXC_F_GCR_ECCADDR_TAGRAMERR_POS                31 /**< ECCADDR_TAGRAMERR Position */
#define MXC_F_GCR_ECCADDR_TAGRAMERR                    ((uint32_t)(0x1UL << MXC_F_GCR_ECCADDR_TAGRAMERR_POS)) /**< ECCADDR_TAGRAMERR Mask */

/**@} end of group GCR_ECCADDR_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_NFC_LDOCR GCR_NFC_LDOCR
 * @brief    NFC LDO Control Register
 * @{
 */
#define MXC_F_GCR_NFC_LDOCR_EN_POS                     4 /**< NFC_LDOCR_EN Position */
#define MXC_F_GCR_NFC_LDOCR_EN                         ((uint32_t)(0x1UL << MXC_F_GCR_NFC_LDOCR_EN_POS)) /**< NFC_LDOCR_EN Mask */

#define MXC_F_GCR_NFC_LDOCR_PULLD_POS                  5 /**< NFC_LDOCR_PULLD Position */
#define MXC_F_GCR_NFC_LDOCR_PULLD                      ((uint32_t)(0x1UL << MXC_F_GCR_NFC_LDOCR_PULLD_POS)) /**< NFC_LDOCR_PULLD Mask */

#define MXC_F_GCR_NFC_LDOCR_VSEL_POS                   6 /**< NFC_LDOCR_VSEL Position */
#define MXC_F_GCR_NFC_LDOCR_VSEL                       ((uint32_t)(0x3UL << MXC_F_GCR_NFC_LDOCR_VSEL_POS)) /**< NFC_LDOCR_VSEL Mask */

#define MXC_F_GCR_NFC_LDOCR_BYPEN_POS                  8 /**< NFC_LDOCR_BYPEN Position */
#define MXC_F_GCR_NFC_LDOCR_BYPEN                      ((uint32_t)(0x1UL << MXC_F_GCR_NFC_LDOCR_BYPEN_POS)) /**< NFC_LDOCR_BYPEN Mask */

#define MXC_F_GCR_NFC_LDOCR_DISCH_POS                  9 /**< NFC_LDOCR_DISCH Position */
#define MXC_F_GCR_NFC_LDOCR_DISCH                      ((uint32_t)(0x1UL << MXC_F_GCR_NFC_LDOCR_DISCH_POS)) /**< NFC_LDOCR_DISCH Mask */

#define MXC_F_GCR_NFC_LDOCR_EN_DLY_POS                 15 /**< NFC_LDOCR_EN_DLY Position */
#define MXC_F_GCR_NFC_LDOCR_EN_DLY                     ((uint32_t)(0x1UL << MXC_F_GCR_NFC_LDOCR_EN_DLY_POS)) /**< NFC_LDOCR_EN_DLY Mask */

#define MXC_F_GCR_NFC_LDOCR_BYP_EN_DLY_POS             14 /**< NFC_LDOCR_BYP_EN_DLY Position */
#define MXC_F_GCR_NFC_LDOCR_BYP_EN_DLY                 ((uint32_t)(0x1UL << MXC_F_GCR_NFC_LDOCR_BYP_EN_DLY_POS)) /**< NFC_LDOCR_BYP_EN_DLY Mask */

/**@} end of group GCR_NFC_LDOCR_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_NFCLDO_DLY GCR_NFCLDO_DLY
 * @brief    NFC LDO Delay Register
 * @{
 */
#define MXC_F_GCR_NFCLDO_DLY_BYPCNT_POS                0 /**< NFCLDO_DLY_BYPCNT Position */
#define MXC_F_GCR_NFCLDO_DLY_BYPCNT                    ((uint32_t)(0xFFUL << MXC_F_GCR_NFCLDO_DLY_BYPCNT_POS)) /**< NFCLDO_DLY_BYPCNT Mask */

#define MXC_F_GCR_NFCLDO_DLY_ENCNT_POS                 8 /**< NFCLDO_DLY_ENCNT Position */
#define MXC_F_GCR_NFCLDO_DLY_ENCNT                     ((uint32_t)(0xFFUL << MXC_F_GCR_NFCLDO_DLY_ENCNT_POS)) /**< NFCLDO_DLY_ENCNT Mask */

/**@} end of group GCR_NFCLDO_DLY_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32570_INCLUDE_GCR_REGS_H_
