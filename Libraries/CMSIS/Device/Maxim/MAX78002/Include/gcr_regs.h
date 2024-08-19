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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX78002_INCLUDE_GCR_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX78002_INCLUDE_GCR_REGS_H_

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
    __IO uint32_t ipll_ctrl;            /**< <tt>\b 0x10:</tt> GCR IPLL_CTRL Register */
    __R  uint32_t rsv_0x14;
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
    __R  uint32_t rsv_0x74_0x7f[3];
    __IO uint32_t gpr0;                 /**< <tt>\b 0x80:</tt> GCR GPR0 Register */
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
#define MXC_R_GCR_IPLL_CTRL                ((uint32_t)0x00000010UL) /**< Offset from GCR Base Address: <tt> 0x0010</tt> */
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
#define MXC_R_GCR_GPR0                     ((uint32_t)0x00000080UL) /**< Offset from GCR Base Address: <tt> 0x0080</tt> */
/**@} end of group gcr_registers */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_SYSCTRL GCR_SYSCTRL
 * @brief    System Control.
 * @{
 */
#define MXC_F_GCR_SYSCTRL_BSTAPEN_POS                  0 /**< SYSCTRL_BSTAPEN Position */
#define MXC_F_GCR_SYSCTRL_BSTAPEN                      ((uint32_t)(0x1UL << MXC_F_GCR_SYSCTRL_BSTAPEN_POS)) /**< SYSCTRL_BSTAPEN Mask */

#define MXC_F_GCR_SYSCTRL_FLASH_PAGE_FLIP_POS          4 /**< SYSCTRL_FLASH_PAGE_FLIP Position */
#define MXC_F_GCR_SYSCTRL_FLASH_PAGE_FLIP              ((uint32_t)(0x1UL << MXC_F_GCR_SYSCTRL_FLASH_PAGE_FLIP_POS)) /**< SYSCTRL_FLASH_PAGE_FLIP Mask */

#define MXC_F_GCR_SYSCTRL_ICC0_FLUSH_POS               6 /**< SYSCTRL_ICC0_FLUSH Position */
#define MXC_F_GCR_SYSCTRL_ICC0_FLUSH                   ((uint32_t)(0x1UL << MXC_F_GCR_SYSCTRL_ICC0_FLUSH_POS)) /**< SYSCTRL_ICC0_FLUSH Mask */

#define MXC_F_GCR_SYSCTRL_ROMDONE_POS                  12 /**< SYSCTRL_ROMDONE Position */
#define MXC_F_GCR_SYSCTRL_ROMDONE                      ((uint32_t)(0x1UL << MXC_F_GCR_SYSCTRL_ROMDONE_POS)) /**< SYSCTRL_ROMDONE Mask */

#define MXC_F_GCR_SYSCTRL_CCHK_POS                     13 /**< SYSCTRL_CCHK Position */
#define MXC_F_GCR_SYSCTRL_CCHK                         ((uint32_t)(0x1UL << MXC_F_GCR_SYSCTRL_CCHK_POS)) /**< SYSCTRL_CCHK Mask */

#define MXC_F_GCR_SYSCTRL_SWD_DIS_POS                  14 /**< SYSCTRL_SWD_DIS Position */
#define MXC_F_GCR_SYSCTRL_SWD_DIS                      ((uint32_t)(0x1UL << MXC_F_GCR_SYSCTRL_SWD_DIS_POS)) /**< SYSCTRL_SWD_DIS Mask */

#define MXC_F_GCR_SYSCTRL_CHKRES_POS                   15 /**< SYSCTRL_CHKRES Position */
#define MXC_F_GCR_SYSCTRL_CHKRES                       ((uint32_t)(0x1UL << MXC_F_GCR_SYSCTRL_CHKRES_POS)) /**< SYSCTRL_CHKRES Mask */

#define MXC_F_GCR_SYSCTRL_OVR_POS                      16 /**< SYSCTRL_OVR Position */
#define MXC_F_GCR_SYSCTRL_OVR                          ((uint32_t)(0x3UL << MXC_F_GCR_SYSCTRL_OVR_POS)) /**< SYSCTRL_OVR Mask */
#define MXC_V_GCR_SYSCTRL_OVR_V0_9                     ((uint32_t)0x0UL) /**< SYSCTRL_OVR_V0_9 Value */
#define MXC_S_GCR_SYSCTRL_OVR_V0_9                     (MXC_V_GCR_SYSCTRL_OVR_V0_9 << MXC_F_GCR_SYSCTRL_OVR_POS) /**< SYSCTRL_OVR_V0_9 Setting */
#define MXC_V_GCR_SYSCTRL_OVR_V1_0                     ((uint32_t)0x1UL) /**< SYSCTRL_OVR_V1_0 Value */
#define MXC_S_GCR_SYSCTRL_OVR_V1_0                     (MXC_V_GCR_SYSCTRL_OVR_V1_0 << MXC_F_GCR_SYSCTRL_OVR_POS) /**< SYSCTRL_OVR_V1_0 Setting */
#define MXC_V_GCR_SYSCTRL_OVR_V1_1                     ((uint32_t)0x2UL) /**< SYSCTRL_OVR_V1_1 Value */
#define MXC_S_GCR_SYSCTRL_OVR_V1_1                     (MXC_V_GCR_SYSCTRL_OVR_V1_1 << MXC_F_GCR_SYSCTRL_OVR_POS) /**< SYSCTRL_OVR_V1_1 Setting */

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

#define MXC_F_GCR_RST0_UART0_POS                       11 /**< RST0_UART0 Position */
#define MXC_F_GCR_RST0_UART0                           ((uint32_t)(0x1UL << MXC_F_GCR_RST0_UART0_POS)) /**< RST0_UART0 Mask */

#define MXC_F_GCR_RST0_UART1_POS                       12 /**< RST0_UART1 Position */
#define MXC_F_GCR_RST0_UART1                           ((uint32_t)(0x1UL << MXC_F_GCR_RST0_UART1_POS)) /**< RST0_UART1 Mask */

#define MXC_F_GCR_RST0_SPI1_POS                        13 /**< RST0_SPI1 Position */
#define MXC_F_GCR_RST0_SPI1                            ((uint32_t)(0x1UL << MXC_F_GCR_RST0_SPI1_POS)) /**< RST0_SPI1 Mask */

#define MXC_F_GCR_RST0_I2C0_POS                        16 /**< RST0_I2C0 Position */
#define MXC_F_GCR_RST0_I2C0                            ((uint32_t)(0x1UL << MXC_F_GCR_RST0_I2C0_POS)) /**< RST0_I2C0 Mask */

#define MXC_F_GCR_RST0_RTC_POS                         17 /**< RST0_RTC Position */
#define MXC_F_GCR_RST0_RTC                             ((uint32_t)(0x1UL << MXC_F_GCR_RST0_RTC_POS)) /**< RST0_RTC Mask */

#define MXC_F_GCR_RST0_SMPHR_POS                       22 /**< RST0_SMPHR Position */
#define MXC_F_GCR_RST0_SMPHR                           ((uint32_t)(0x1UL << MXC_F_GCR_RST0_SMPHR_POS)) /**< RST0_SMPHR Mask */

#define MXC_F_GCR_RST0_USB_POS                         23 /**< RST0_USB Position */
#define MXC_F_GCR_RST0_USB                             ((uint32_t)(0x1UL << MXC_F_GCR_RST0_USB_POS)) /**< RST0_USB Mask */

#define MXC_F_GCR_RST0_TRNG_POS                        24 /**< RST0_TRNG Position */
#define MXC_F_GCR_RST0_TRNG                            ((uint32_t)(0x1UL << MXC_F_GCR_RST0_TRNG_POS)) /**< RST0_TRNG Mask */

#define MXC_F_GCR_RST0_CNN_POS                         25 /**< RST0_CNN Position */
#define MXC_F_GCR_RST0_CNN                             ((uint32_t)(0x1UL << MXC_F_GCR_RST0_CNN_POS)) /**< RST0_CNN Mask */

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
#define MXC_V_GCR_CLKCTRL_SYSCLK_SEL_IPLL              ((uint32_t)0x1UL) /**< CLKCTRL_SYSCLK_SEL_IPLL Value */
#define MXC_S_GCR_CLKCTRL_SYSCLK_SEL_IPLL              (MXC_V_GCR_CLKCTRL_SYSCLK_SEL_IPLL << MXC_F_GCR_CLKCTRL_SYSCLK_SEL_POS) /**< CLKCTRL_SYSCLK_SEL_IPLL Setting */
#define MXC_V_GCR_CLKCTRL_SYSCLK_SEL_EBO               ((uint32_t)0x2UL) /**< CLKCTRL_SYSCLK_SEL_EBO Value */
#define MXC_S_GCR_CLKCTRL_SYSCLK_SEL_EBO               (MXC_V_GCR_CLKCTRL_SYSCLK_SEL_EBO << MXC_F_GCR_CLKCTRL_SYSCLK_SEL_POS) /**< CLKCTRL_SYSCLK_SEL_EBO Setting */
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

#define MXC_F_GCR_CLKCTRL_EBO_EN_POS                   16 /**< CLKCTRL_EBO_EN Position */
#define MXC_F_GCR_CLKCTRL_EBO_EN                       ((uint32_t)(0x1UL << MXC_F_GCR_CLKCTRL_EBO_EN_POS)) /**< CLKCTRL_EBO_EN Mask */

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

#define MXC_F_GCR_CLKCTRL_EBO_RDY_POS                  24 /**< CLKCTRL_EBO_RDY Position */
#define MXC_F_GCR_CLKCTRL_EBO_RDY                      ((uint32_t)(0x1UL << MXC_F_GCR_CLKCTRL_EBO_RDY_POS)) /**< CLKCTRL_EBO_RDY Mask */

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
#define MXC_F_GCR_PM_MODE                              ((uint32_t)(0xFUL << MXC_F_GCR_PM_MODE_POS)) /**< PM_MODE Mask */
#define MXC_V_GCR_PM_MODE_ACTIVE                       ((uint32_t)0x0UL) /**< PM_MODE_ACTIVE Value */
#define MXC_S_GCR_PM_MODE_ACTIVE                       (MXC_V_GCR_PM_MODE_ACTIVE << MXC_F_GCR_PM_MODE_POS) /**< PM_MODE_ACTIVE Setting */
#define MXC_V_GCR_PM_MODE_SLEEP                        ((uint32_t)0x1UL) /**< PM_MODE_SLEEP Value */
#define MXC_S_GCR_PM_MODE_SLEEP                        (MXC_V_GCR_PM_MODE_SLEEP << MXC_F_GCR_PM_MODE_POS) /**< PM_MODE_SLEEP Setting */
#define MXC_V_GCR_PM_MODE_STANDBY                      ((uint32_t)0x2UL) /**< PM_MODE_STANDBY Value */
#define MXC_S_GCR_PM_MODE_STANDBY                      (MXC_V_GCR_PM_MODE_STANDBY << MXC_F_GCR_PM_MODE_POS) /**< PM_MODE_STANDBY Setting */
#define MXC_V_GCR_PM_MODE_BACKUP                       ((uint32_t)0x4UL) /**< PM_MODE_BACKUP Value */
#define MXC_S_GCR_PM_MODE_BACKUP                       (MXC_V_GCR_PM_MODE_BACKUP << MXC_F_GCR_PM_MODE_POS) /**< PM_MODE_BACKUP Setting */
#define MXC_V_GCR_PM_MODE_LPM                          ((uint32_t)0x8UL) /**< PM_MODE_LPM Value */
#define MXC_S_GCR_PM_MODE_LPM                          (MXC_V_GCR_PM_MODE_LPM << MXC_F_GCR_PM_MODE_POS) /**< PM_MODE_LPM Setting */
#define MXC_V_GCR_PM_MODE_UPM                          ((uint32_t)0x9UL) /**< PM_MODE_UPM Value */
#define MXC_S_GCR_PM_MODE_UPM                          (MXC_V_GCR_PM_MODE_UPM << MXC_F_GCR_PM_MODE_POS) /**< PM_MODE_UPM Setting */
#define MXC_V_GCR_PM_MODE_POWERDOWN                    ((uint32_t)0xAUL) /**< PM_MODE_POWERDOWN Value */
#define MXC_S_GCR_PM_MODE_POWERDOWN                    (MXC_V_GCR_PM_MODE_POWERDOWN << MXC_F_GCR_PM_MODE_POS) /**< PM_MODE_POWERDOWN Setting */

#define MXC_F_GCR_PM_GPIO_WE_POS                       4 /**< PM_GPIO_WE Position */
#define MXC_F_GCR_PM_GPIO_WE                           ((uint32_t)(0x1UL << MXC_F_GCR_PM_GPIO_WE_POS)) /**< PM_GPIO_WE Mask */

#define MXC_F_GCR_PM_RTC_WE_POS                        5 /**< PM_RTC_WE Position */
#define MXC_F_GCR_PM_RTC_WE                            ((uint32_t)(0x1UL << MXC_F_GCR_PM_RTC_WE_POS)) /**< PM_RTC_WE Mask */

#define MXC_F_GCR_PM_USB_WE_POS                        6 /**< PM_USB_WE Position */
#define MXC_F_GCR_PM_USB_WE                            ((uint32_t)(0x1UL << MXC_F_GCR_PM_USB_WE_POS)) /**< PM_USB_WE Mask */

#define MXC_F_GCR_PM_WUT_WE_POS                        7 /**< PM_WUT_WE Position */
#define MXC_F_GCR_PM_WUT_WE                            ((uint32_t)(0x1UL << MXC_F_GCR_PM_WUT_WE_POS)) /**< PM_WUT_WE Mask */

#define MXC_F_GCR_PM_AINCOMP_WE_POS                    9 /**< PM_AINCOMP_WE Position */
#define MXC_F_GCR_PM_AINCOMP_WE                        ((uint32_t)(0x1UL << MXC_F_GCR_PM_AINCOMP_WE_POS)) /**< PM_AINCOMP_WE Mask */

#define MXC_F_GCR_PM_ISO_PD_POS                        15 /**< PM_ISO_PD Position */
#define MXC_F_GCR_PM_ISO_PD                            ((uint32_t)(0x1UL << MXC_F_GCR_PM_ISO_PD_POS)) /**< PM_ISO_PD Mask */

#define MXC_F_GCR_PM_IPO_PD_POS                        16 /**< PM_IPO_PD Position */
#define MXC_F_GCR_PM_IPO_PD                            ((uint32_t)(0x1UL << MXC_F_GCR_PM_IPO_PD_POS)) /**< PM_IPO_PD Mask */

#define MXC_F_GCR_PM_IBRO_PD_POS                       17 /**< PM_IBRO_PD Position */
#define MXC_F_GCR_PM_IBRO_PD                           ((uint32_t)(0x1UL << MXC_F_GCR_PM_IBRO_PD_POS)) /**< PM_IBRO_PD Mask */

#define MXC_F_GCR_PM_EBO_BP_POS                        20 /**< PM_EBO_BP Position */
#define MXC_F_GCR_PM_EBO_BP                            ((uint32_t)(0x1UL << MXC_F_GCR_PM_EBO_BP_POS)) /**< PM_EBO_BP Mask */

/**@} end of group GCR_PM_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_IPLL_CTRL GCR_IPLL_CTRL
 * @brief    IPLL Control
 * @{
 */
#define MXC_F_GCR_IPLL_CTRL_EN_POS                     0 /**< IPLL_CTRL_EN Position */
#define MXC_F_GCR_IPLL_CTRL_EN                         ((uint32_t)(0x1UL << MXC_F_GCR_IPLL_CTRL_EN_POS)) /**< IPLL_CTRL_EN Mask */

#define MXC_F_GCR_IPLL_CTRL_RDY_POS                    1 /**< IPLL_CTRL_RDY Position */
#define MXC_F_GCR_IPLL_CTRL_RDY                        ((uint32_t)(0x1UL << MXC_F_GCR_IPLL_CTRL_RDY_POS)) /**< IPLL_CTRL_RDY Mask */

/**@} end of group GCR_IPLL_CTRL_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_PCLKDIV GCR_PCLKDIV
 * @brief    Peripheral Clock Divider.
 * @{
 */
#define MXC_F_GCR_PCLKDIV_SDIOCLKDIV_POS               7 /**< PCLKDIV_SDIOCLKDIV Position */
#define MXC_F_GCR_PCLKDIV_SDIOCLKDIV                   ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIV_SDIOCLKDIV_POS)) /**< PCLKDIV_SDIOCLKDIV Mask */

#define MXC_F_GCR_PCLKDIV_CNNCLKDIV_POS                14 /**< PCLKDIV_CNNCLKDIV Position */
#define MXC_F_GCR_PCLKDIV_CNNCLKDIV                    ((uint32_t)(0x7UL << MXC_F_GCR_PCLKDIV_CNNCLKDIV_POS)) /**< PCLKDIV_CNNCLKDIV Mask */
#define MXC_V_GCR_PCLKDIV_CNNCLKDIV_DIV2               ((uint32_t)0x0UL) /**< PCLKDIV_CNNCLKDIV_DIV2 Value */
#define MXC_S_GCR_PCLKDIV_CNNCLKDIV_DIV2               (MXC_V_GCR_PCLKDIV_CNNCLKDIV_DIV2 << MXC_F_GCR_PCLKDIV_CNNCLKDIV_POS) /**< PCLKDIV_CNNCLKDIV_DIV2 Setting */
#define MXC_V_GCR_PCLKDIV_CNNCLKDIV_DIV4               ((uint32_t)0x1UL) /**< PCLKDIV_CNNCLKDIV_DIV4 Value */
#define MXC_S_GCR_PCLKDIV_CNNCLKDIV_DIV4               (MXC_V_GCR_PCLKDIV_CNNCLKDIV_DIV4 << MXC_F_GCR_PCLKDIV_CNNCLKDIV_POS) /**< PCLKDIV_CNNCLKDIV_DIV4 Setting */
#define MXC_V_GCR_PCLKDIV_CNNCLKDIV_DIV8               ((uint32_t)0x2UL) /**< PCLKDIV_CNNCLKDIV_DIV8 Value */
#define MXC_S_GCR_PCLKDIV_CNNCLKDIV_DIV8               (MXC_V_GCR_PCLKDIV_CNNCLKDIV_DIV8 << MXC_F_GCR_PCLKDIV_CNNCLKDIV_POS) /**< PCLKDIV_CNNCLKDIV_DIV8 Setting */
#define MXC_V_GCR_PCLKDIV_CNNCLKDIV_DIV16              ((uint32_t)0x3UL) /**< PCLKDIV_CNNCLKDIV_DIV16 Value */
#define MXC_S_GCR_PCLKDIV_CNNCLKDIV_DIV16              (MXC_V_GCR_PCLKDIV_CNNCLKDIV_DIV16 << MXC_F_GCR_PCLKDIV_CNNCLKDIV_POS) /**< PCLKDIV_CNNCLKDIV_DIV16 Setting */
#define MXC_V_GCR_PCLKDIV_CNNCLKDIV_DIV1               ((uint32_t)0x4UL) /**< PCLKDIV_CNNCLKDIV_DIV1 Value */
#define MXC_S_GCR_PCLKDIV_CNNCLKDIV_DIV1               (MXC_V_GCR_PCLKDIV_CNNCLKDIV_DIV1 << MXC_F_GCR_PCLKDIV_CNNCLKDIV_POS) /**< PCLKDIV_CNNCLKDIV_DIV1 Setting */

#define MXC_F_GCR_PCLKDIV_CNNCLKSEL_POS                17 /**< PCLKDIV_CNNCLKSEL Position */
#define MXC_F_GCR_PCLKDIV_CNNCLKSEL                    ((uint32_t)(0x3UL << MXC_F_GCR_PCLKDIV_CNNCLKSEL_POS)) /**< PCLKDIV_CNNCLKSEL Mask */
#define MXC_V_GCR_PCLKDIV_CNNCLKSEL_PCLK               ((uint32_t)0x0UL) /**< PCLKDIV_CNNCLKSEL_PCLK Value */
#define MXC_S_GCR_PCLKDIV_CNNCLKSEL_PCLK               (MXC_V_GCR_PCLKDIV_CNNCLKSEL_PCLK << MXC_F_GCR_PCLKDIV_CNNCLKSEL_POS) /**< PCLKDIV_CNNCLKSEL_PCLK Setting */
#define MXC_V_GCR_PCLKDIV_CNNCLKSEL_ISO                ((uint32_t)0x1UL) /**< PCLKDIV_CNNCLKSEL_ISO Value */
#define MXC_S_GCR_PCLKDIV_CNNCLKSEL_ISO                (MXC_V_GCR_PCLKDIV_CNNCLKSEL_ISO << MXC_F_GCR_PCLKDIV_CNNCLKSEL_POS) /**< PCLKDIV_CNNCLKSEL_ISO Setting */
#define MXC_V_GCR_PCLKDIV_CNNCLKSEL_IPLL               ((uint32_t)0x3UL) /**< PCLKDIV_CNNCLKSEL_IPLL Value */
#define MXC_S_GCR_PCLKDIV_CNNCLKSEL_IPLL               (MXC_V_GCR_PCLKDIV_CNNCLKSEL_IPLL << MXC_F_GCR_PCLKDIV_CNNCLKSEL_POS) /**< PCLKDIV_CNNCLKSEL_IPLL Setting */

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

#define MXC_F_GCR_PCLKDIS0_SPI1_POS                    6 /**< PCLKDIS0_SPI1 Position */
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

#define MXC_F_GCR_PCLKDIS0_TMR3_POS                    18 /**< PCLKDIS0_TMR3 Position */
#define MXC_F_GCR_PCLKDIS0_TMR3                        ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS0_TMR3_POS)) /**< PCLKDIS0_TMR3 Mask */

#define MXC_F_GCR_PCLKDIS0_ADC_POS                     23 /**< PCLKDIS0_ADC Position */
#define MXC_F_GCR_PCLKDIS0_ADC                         ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS0_ADC_POS)) /**< PCLKDIS0_ADC Mask */

#define MXC_F_GCR_PCLKDIS0_CNN_POS                     25 /**< PCLKDIS0_CNN Position */
#define MXC_F_GCR_PCLKDIS0_CNN                         ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS0_CNN_POS)) /**< PCLKDIS0_CNN Mask */

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

#define MXC_F_GCR_MEMCTRL_SYSRAM0ECC_POS               16 /**< MEMCTRL_SYSRAM0ECC Position */
#define MXC_F_GCR_MEMCTRL_SYSRAM0ECC                   ((uint32_t)(0x1UL << MXC_F_GCR_MEMCTRL_SYSRAM0ECC_POS)) /**< MEMCTRL_SYSRAM0ECC Mask */

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

#define MXC_F_GCR_MEMZ_RAM7_POS                        7 /**< MEMZ_RAM7 Position */
#define MXC_F_GCR_MEMZ_RAM7                            ((uint32_t)(0x1UL << MXC_F_GCR_MEMZ_RAM7_POS)) /**< MEMZ_RAM7 Mask */

#define MXC_F_GCR_MEMZ_RAM0ECC_POS                     8 /**< MEMZ_RAM0ECC Position */
#define MXC_F_GCR_MEMZ_RAM0ECC                         ((uint32_t)(0x1UL << MXC_F_GCR_MEMZ_RAM0ECC_POS)) /**< MEMZ_RAM0ECC Mask */

#define MXC_F_GCR_MEMZ_ICC0_POS                        9 /**< MEMZ_ICC0 Position */
#define MXC_F_GCR_MEMZ_ICC0                            ((uint32_t)(0x1UL << MXC_F_GCR_MEMZ_ICC0_POS)) /**< MEMZ_ICC0 Mask */

#define MXC_F_GCR_MEMZ_ICC1_POS                        10 /**< MEMZ_ICC1 Position */
#define MXC_F_GCR_MEMZ_ICC1                            ((uint32_t)(0x1UL << MXC_F_GCR_MEMZ_ICC1_POS)) /**< MEMZ_ICC1 Mask */

#define MXC_F_GCR_MEMZ_USBFIFO_POS                     11 /**< MEMZ_USBFIFO Position */
#define MXC_F_GCR_MEMZ_USBFIFO                         ((uint32_t)(0x1UL << MXC_F_GCR_MEMZ_USBFIFO_POS)) /**< MEMZ_USBFIFO Mask */

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

#define MXC_F_GCR_RST1_SDHC_POS                        6 /**< RST1_SDHC Position */
#define MXC_F_GCR_RST1_SDHC                            ((uint32_t)(0x1UL << MXC_F_GCR_RST1_SDHC_POS)) /**< RST1_SDHC Mask */

#define MXC_F_GCR_RST1_OWM_POS                         7 /**< RST1_OWM Position */
#define MXC_F_GCR_RST1_OWM                             ((uint32_t)(0x1UL << MXC_F_GCR_RST1_OWM_POS)) /**< RST1_OWM Mask */

#define MXC_F_GCR_RST1_CRC_POS                         9 /**< RST1_CRC Position */
#define MXC_F_GCR_RST1_CRC                             ((uint32_t)(0x1UL << MXC_F_GCR_RST1_CRC_POS)) /**< RST1_CRC Mask */

#define MXC_F_GCR_RST1_AES_POS                         10 /**< RST1_AES Position */
#define MXC_F_GCR_RST1_AES                             ((uint32_t)(0x1UL << MXC_F_GCR_RST1_AES_POS)) /**< RST1_AES Mask */

#define MXC_F_GCR_RST1_SPI0_POS                        11 /**< RST1_SPI0 Position */
#define MXC_F_GCR_RST1_SPI0                            ((uint32_t)(0x1UL << MXC_F_GCR_RST1_SPI0_POS)) /**< RST1_SPI0 Mask */

#define MXC_F_GCR_RST1_CSI2PHY_POS                     14 /**< RST1_CSI2PHY Position */
#define MXC_F_GCR_RST1_CSI2PHY                         ((uint32_t)(0x1UL << MXC_F_GCR_RST1_CSI2PHY_POS)) /**< RST1_CSI2PHY Mask */

#define MXC_F_GCR_RST1_SMPHR_POS                       16 /**< RST1_SMPHR Position */
#define MXC_F_GCR_RST1_SMPHR                           ((uint32_t)(0x1UL << MXC_F_GCR_RST1_SMPHR_POS)) /**< RST1_SMPHR Mask */

#define MXC_F_GCR_RST1_I2S_POS                         19 /**< RST1_I2S Position */
#define MXC_F_GCR_RST1_I2S                             ((uint32_t)(0x1UL << MXC_F_GCR_RST1_I2S_POS)) /**< RST1_I2S Mask */

#define MXC_F_GCR_RST1_I2C2_POS                        20 /**< RST1_I2C2 Position */
#define MXC_F_GCR_RST1_I2C2                            ((uint32_t)(0x1UL << MXC_F_GCR_RST1_I2C2_POS)) /**< RST1_I2C2 Mask */

#define MXC_F_GCR_RST1_DVS_POS                         24 /**< RST1_DVS Position */
#define MXC_F_GCR_RST1_DVS                             ((uint32_t)(0x1UL << MXC_F_GCR_RST1_DVS_POS)) /**< RST1_DVS Mask */

#define MXC_F_GCR_RST1_SIMO_POS                        25 /**< RST1_SIMO Position */
#define MXC_F_GCR_RST1_SIMO                            ((uint32_t)(0x1UL << MXC_F_GCR_RST1_SIMO_POS)) /**< RST1_SIMO Mask */

#define MXC_F_GCR_RST1_PCIF_POS                        26 /**< RST1_PCIF Position */
#define MXC_F_GCR_RST1_PCIF                            ((uint32_t)(0x1UL << MXC_F_GCR_RST1_PCIF_POS)) /**< RST1_PCIF Mask */

#define MXC_F_GCR_RST1_CSI2_POS                        27 /**< RST1_CSI2 Position */
#define MXC_F_GCR_RST1_CSI2                            ((uint32_t)(0x1UL << MXC_F_GCR_RST1_CSI2_POS)) /**< RST1_CSI2 Mask */

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

#define MXC_F_GCR_PCLKDIS1_SMPHR_POS                   9 /**< PCLKDIS1_SMPHR Position */
#define MXC_F_GCR_PCLKDIS1_SMPHR                       ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS1_SMPHR_POS)) /**< PCLKDIS1_SMPHR Mask */

#define MXC_F_GCR_PCLKDIS1_SDHC_POS                    10 /**< PCLKDIS1_SDHC Position */
#define MXC_F_GCR_PCLKDIS1_SDHC                        ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS1_SDHC_POS)) /**< PCLKDIS1_SDHC Mask */

#define MXC_F_GCR_PCLKDIS1_OWM_POS                     13 /**< PCLKDIS1_OWM Position */
#define MXC_F_GCR_PCLKDIS1_OWM                         ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS1_OWM_POS)) /**< PCLKDIS1_OWM Mask */

#define MXC_F_GCR_PCLKDIS1_CRC_POS                     14 /**< PCLKDIS1_CRC Position */
#define MXC_F_GCR_PCLKDIS1_CRC                         ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS1_CRC_POS)) /**< PCLKDIS1_CRC Mask */

#define MXC_F_GCR_PCLKDIS1_AES_POS                     15 /**< PCLKDIS1_AES Position */
#define MXC_F_GCR_PCLKDIS1_AES                         ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS1_AES_POS)) /**< PCLKDIS1_AES Mask */

#define MXC_F_GCR_PCLKDIS1_SPI0_POS                    16 /**< PCLKDIS1_SPI0 Position */
#define MXC_F_GCR_PCLKDIS1_SPI0                        ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS1_SPI0_POS)) /**< PCLKDIS1_SPI0 Mask */

#define MXC_F_GCR_PCLKDIS1_PCIF_POS                    18 /**< PCLKDIS1_PCIF Position */
#define MXC_F_GCR_PCLKDIS1_PCIF                        ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS1_PCIF_POS)) /**< PCLKDIS1_PCIF Mask */

#define MXC_F_GCR_PCLKDIS1_I2S_POS                     23 /**< PCLKDIS1_I2S Position */
#define MXC_F_GCR_PCLKDIS1_I2S                         ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS1_I2S_POS)) /**< PCLKDIS1_I2S Mask */

#define MXC_F_GCR_PCLKDIS1_I2C2_POS                    24 /**< PCLKDIS1_I2C2 Position */
#define MXC_F_GCR_PCLKDIS1_I2C2                        ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS1_I2C2_POS)) /**< PCLKDIS1_I2C2 Mask */

#define MXC_F_GCR_PCLKDIS1_WDT0_POS                    27 /**< PCLKDIS1_WDT0 Position */
#define MXC_F_GCR_PCLKDIS1_WDT0                        ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS1_WDT0_POS)) /**< PCLKDIS1_WDT0 Mask */

#define MXC_F_GCR_PCLKDIS1_CSI2_POS                    30 /**< PCLKDIS1_CSI2 Position */
#define MXC_F_GCR_PCLKDIS1_CSI2                        ((uint32_t)(0x1UL << MXC_F_GCR_PCLKDIS1_CSI2_POS)) /**< PCLKDIS1_CSI2 Mask */

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
#define MXC_F_GCR_ECCERR_RAM_POS                       0 /**< ECCERR_RAM Position */
#define MXC_F_GCR_ECCERR_RAM                           ((uint32_t)(0x1UL << MXC_F_GCR_ECCERR_RAM_POS)) /**< ECCERR_RAM Mask */

/**@} end of group GCR_ECCERR_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_ECCCED GCR_ECCCED
 * @brief    ECC Not Double Error Detect Register
 * @{
 */
#define MXC_F_GCR_ECCCED_RAM_POS                       0 /**< ECCCED_RAM Position */
#define MXC_F_GCR_ECCCED_RAM                           ((uint32_t)(0x1UL << MXC_F_GCR_ECCCED_RAM_POS)) /**< ECCCED_RAM Mask */

/**@} end of group GCR_ECCCED_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_ECCIE GCR_ECCIE
 * @brief    ECC IRQ Enable Register
 * @{
 */
#define MXC_F_GCR_ECCIE_RAM_POS                        0 /**< ECCIE_RAM Position */
#define MXC_F_GCR_ECCIE_RAM                            ((uint32_t)(0x1UL << MXC_F_GCR_ECCIE_RAM_POS)) /**< ECCIE_RAM Mask */

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

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX78002_INCLUDE_GCR_REGS_H_
