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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32650_INCLUDE_GCR_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32650_INCLUDE_GCR_REGS_H_

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
    __IO uint32_t rst0;                 /**< <tt>\b 0x04:</tt> GCR RST0 Register */
    __IO uint32_t clk_ctrl;             /**< <tt>\b 0x08:</tt> GCR CLK_CTRL Register */
    __IO uint32_t pmr;                  /**< <tt>\b 0x0C:</tt> GCR PMR Register */
    __R  uint32_t rsv_0x10_0x17[2];
    __IO uint32_t pclk_div;             /**< <tt>\b 0x18:</tt> GCR PCLK_DIV Register */
    __R  uint32_t rsv_0x1c_0x23[2];
    __IO uint32_t pclk_dis0;            /**< <tt>\b 0x24:</tt> GCR PCLK_DIS0 Register */
    __IO uint32_t mem_clk;              /**< <tt>\b 0x28:</tt> GCR MEM_CLK Register */
    __IO uint32_t mem_zero;             /**< <tt>\b 0x2C:</tt> GCR MEM_ZERO Register */
    __R  uint32_t rsv_0x30_0x3f[4];
    __IO uint32_t sys_stat;             /**< <tt>\b 0x40:</tt> GCR SYS_STAT Register */
    __IO uint32_t rst1;                 /**< <tt>\b 0x44:</tt> GCR RST1 Register */
    __IO uint32_t pclk_dis1;            /**< <tt>\b 0x48:</tt> GCR PCLK_DIS1 Register */
    __IO uint32_t event_en;             /**< <tt>\b 0x4C:</tt> GCR EVENT_EN Register */
    __I  uint32_t rev;                  /**< <tt>\b 0x50:</tt> GCR REV Register */
    __IO uint32_t sys_stat_ie;          /**< <tt>\b 0x54:</tt> GCR SYS_STAT_IE Register */
} mxc_gcr_regs_t;

/* Register offsets for module GCR */
/**
 * @ingroup    gcr_registers
 * @defgroup   GCR_Register_Offsets Register Offsets
 * @brief      GCR Peripheral Register Offsets from the GCR Base Peripheral Address.
 * @{
 */
#define MXC_R_GCR_SCON                     ((uint32_t)0x00000000UL) /**< Offset from GCR Base Address: <tt> 0x0000</tt> */
#define MXC_R_GCR_RST0                     ((uint32_t)0x00000004UL) /**< Offset from GCR Base Address: <tt> 0x0004</tt> */
#define MXC_R_GCR_CLK_CTRL                 ((uint32_t)0x00000008UL) /**< Offset from GCR Base Address: <tt> 0x0008</tt> */
#define MXC_R_GCR_PMR                      ((uint32_t)0x0000000CUL) /**< Offset from GCR Base Address: <tt> 0x000C</tt> */
#define MXC_R_GCR_PCLK_DIV                 ((uint32_t)0x00000018UL) /**< Offset from GCR Base Address: <tt> 0x0018</tt> */
#define MXC_R_GCR_PCLK_DIS0                ((uint32_t)0x00000024UL) /**< Offset from GCR Base Address: <tt> 0x0024</tt> */
#define MXC_R_GCR_MEM_CLK                  ((uint32_t)0x00000028UL) /**< Offset from GCR Base Address: <tt> 0x0028</tt> */
#define MXC_R_GCR_MEM_ZERO                 ((uint32_t)0x0000002CUL) /**< Offset from GCR Base Address: <tt> 0x002C</tt> */
#define MXC_R_GCR_SYS_STAT                 ((uint32_t)0x00000040UL) /**< Offset from GCR Base Address: <tt> 0x0040</tt> */
#define MXC_R_GCR_RST1                     ((uint32_t)0x00000044UL) /**< Offset from GCR Base Address: <tt> 0x0044</tt> */
#define MXC_R_GCR_PCLK_DIS1                ((uint32_t)0x00000048UL) /**< Offset from GCR Base Address: <tt> 0x0048</tt> */
#define MXC_R_GCR_EVENT_EN                 ((uint32_t)0x0000004CUL) /**< Offset from GCR Base Address: <tt> 0x004C</tt> */
#define MXC_R_GCR_REV                      ((uint32_t)0x00000050UL) /**< Offset from GCR Base Address: <tt> 0x0050</tt> */
#define MXC_R_GCR_SYS_STAT_IE              ((uint32_t)0x00000054UL) /**< Offset from GCR Base Address: <tt> 0x0054</tt> */
/**@} end of group gcr_registers */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_SCON GCR_SCON
 * @brief    System Control.
 * @{
 */
#define MXC_F_GCR_SCON_BSTAPEN_POS                     0 /**< SCON_BSTAPEN Position */
#define MXC_F_GCR_SCON_BSTAPEN                         ((uint32_t)(0x1UL << MXC_F_GCR_SCON_BSTAPEN_POS)) /**< SCON_BSTAPEN Mask */
#define MXC_V_GCR_SCON_BSTAPEN_DIS                     ((uint32_t)0x0UL) /**< SCON_BSTAPEN_DIS Value */
#define MXC_S_GCR_SCON_BSTAPEN_DIS                     (MXC_V_GCR_SCON_BSTAPEN_DIS << MXC_F_GCR_SCON_BSTAPEN_POS) /**< SCON_BSTAPEN_DIS Setting */
#define MXC_V_GCR_SCON_BSTAPEN_EN                      ((uint32_t)0x1UL) /**< SCON_BSTAPEN_EN Value */
#define MXC_S_GCR_SCON_BSTAPEN_EN                      (MXC_V_GCR_SCON_BSTAPEN_EN << MXC_F_GCR_SCON_BSTAPEN_POS) /**< SCON_BSTAPEN_EN Setting */

#define MXC_F_GCR_SCON_FLASH_PAGE_FLIP_POS             4 /**< SCON_FLASH_PAGE_FLIP Position */
#define MXC_F_GCR_SCON_FLASH_PAGE_FLIP                 ((uint32_t)(0x1UL << MXC_F_GCR_SCON_FLASH_PAGE_FLIP_POS)) /**< SCON_FLASH_PAGE_FLIP Mask */
#define MXC_V_GCR_SCON_FLASH_PAGE_FLIP_NORMAL          ((uint32_t)0x0UL) /**< SCON_FLASH_PAGE_FLIP_NORMAL Value */
#define MXC_S_GCR_SCON_FLASH_PAGE_FLIP_NORMAL          (MXC_V_GCR_SCON_FLASH_PAGE_FLIP_NORMAL << MXC_F_GCR_SCON_FLASH_PAGE_FLIP_POS) /**< SCON_FLASH_PAGE_FLIP_NORMAL Setting */
#define MXC_V_GCR_SCON_FLASH_PAGE_FLIP_FLIPPED         ((uint32_t)0x1UL) /**< SCON_FLASH_PAGE_FLIP_FLIPPED Value */
#define MXC_S_GCR_SCON_FLASH_PAGE_FLIP_FLIPPED         (MXC_V_GCR_SCON_FLASH_PAGE_FLIP_FLIPPED << MXC_F_GCR_SCON_FLASH_PAGE_FLIP_POS) /**< SCON_FLASH_PAGE_FLIP_FLIPPED Setting */

#define MXC_F_GCR_SCON_CCACHE_FLUSH_POS                6 /**< SCON_CCACHE_FLUSH Position */
#define MXC_F_GCR_SCON_CCACHE_FLUSH                    ((uint32_t)(0x1UL << MXC_F_GCR_SCON_CCACHE_FLUSH_POS)) /**< SCON_CCACHE_FLUSH Mask */
#define MXC_V_GCR_SCON_CCACHE_FLUSH_NORMAL             ((uint32_t)0x0UL) /**< SCON_CCACHE_FLUSH_NORMAL Value */
#define MXC_S_GCR_SCON_CCACHE_FLUSH_NORMAL             (MXC_V_GCR_SCON_CCACHE_FLUSH_NORMAL << MXC_F_GCR_SCON_CCACHE_FLUSH_POS) /**< SCON_CCACHE_FLUSH_NORMAL Setting */
#define MXC_V_GCR_SCON_CCACHE_FLUSH_FLUSH              ((uint32_t)0x1UL) /**< SCON_CCACHE_FLUSH_FLUSH Value */
#define MXC_S_GCR_SCON_CCACHE_FLUSH_FLUSH              (MXC_V_GCR_SCON_CCACHE_FLUSH_FLUSH << MXC_F_GCR_SCON_CCACHE_FLUSH_POS) /**< SCON_CCACHE_FLUSH_FLUSH Setting */

#define MXC_F_GCR_SCON_DCACHE_FLUSH_POS                7 /**< SCON_DCACHE_FLUSH Position */
#define MXC_F_GCR_SCON_DCACHE_FLUSH                    ((uint32_t)(0x1UL << MXC_F_GCR_SCON_DCACHE_FLUSH_POS)) /**< SCON_DCACHE_FLUSH Mask */
#define MXC_V_GCR_SCON_DCACHE_FLUSH_NORMAL             ((uint32_t)0x0UL) /**< SCON_DCACHE_FLUSH_NORMAL Value */
#define MXC_S_GCR_SCON_DCACHE_FLUSH_NORMAL             (MXC_V_GCR_SCON_DCACHE_FLUSH_NORMAL << MXC_F_GCR_SCON_DCACHE_FLUSH_POS) /**< SCON_DCACHE_FLUSH_NORMAL Setting */
#define MXC_V_GCR_SCON_DCACHE_FLUSH_FLUSH              ((uint32_t)0x1UL) /**< SCON_DCACHE_FLUSH_FLUSH Value */
#define MXC_S_GCR_SCON_DCACHE_FLUSH_FLUSH              (MXC_V_GCR_SCON_DCACHE_FLUSH_FLUSH << MXC_F_GCR_SCON_DCACHE_FLUSH_POS) /**< SCON_DCACHE_FLUSH_FLUSH Setting */

#define MXC_F_GCR_SCON_DCACHE_DIS_POS                  9 /**< SCON_DCACHE_DIS Position */
#define MXC_F_GCR_SCON_DCACHE_DIS                      ((uint32_t)(0x1UL << MXC_F_GCR_SCON_DCACHE_DIS_POS)) /**< SCON_DCACHE_DIS Mask */
#define MXC_V_GCR_SCON_DCACHE_DIS_ENABLED              ((uint32_t)0x0UL) /**< SCON_DCACHE_DIS_ENABLED Value */
#define MXC_S_GCR_SCON_DCACHE_DIS_ENABLED              (MXC_V_GCR_SCON_DCACHE_DIS_ENABLED << MXC_F_GCR_SCON_DCACHE_DIS_POS) /**< SCON_DCACHE_DIS_ENABLED Setting */
#define MXC_V_GCR_SCON_DCACHE_DIS_DISABLED             ((uint32_t)0x1UL) /**< SCON_DCACHE_DIS_DISABLED Value */
#define MXC_S_GCR_SCON_DCACHE_DIS_DISABLED             (MXC_V_GCR_SCON_DCACHE_DIS_DISABLED << MXC_F_GCR_SCON_DCACHE_DIS_POS) /**< SCON_DCACHE_DIS_DISABLED Setting */

#define MXC_F_GCR_SCON_CCHK_POS                        13 /**< SCON_CCHK Position */
#define MXC_F_GCR_SCON_CCHK                            ((uint32_t)(0x1UL << MXC_F_GCR_SCON_CCHK_POS)) /**< SCON_CCHK Mask */
#define MXC_V_GCR_SCON_CCHK_COMPLETE                   ((uint32_t)0x0UL) /**< SCON_CCHK_COMPLETE Value */
#define MXC_S_GCR_SCON_CCHK_COMPLETE                   (MXC_V_GCR_SCON_CCHK_COMPLETE << MXC_F_GCR_SCON_CCHK_POS) /**< SCON_CCHK_COMPLETE Setting */
#define MXC_V_GCR_SCON_CCHK_START                      ((uint32_t)0x1UL) /**< SCON_CCHK_START Value */
#define MXC_S_GCR_SCON_CCHK_START                      (MXC_V_GCR_SCON_CCHK_START << MXC_F_GCR_SCON_CCHK_POS) /**< SCON_CCHK_START Setting */

#define MXC_F_GCR_SCON_CHKRES_POS                      15 /**< SCON_CHKRES Position */
#define MXC_F_GCR_SCON_CHKRES                          ((uint32_t)(0x1UL << MXC_F_GCR_SCON_CHKRES_POS)) /**< SCON_CHKRES Mask */
#define MXC_V_GCR_SCON_CHKRES_PASS                     ((uint32_t)0x0UL) /**< SCON_CHKRES_PASS Value */
#define MXC_S_GCR_SCON_CHKRES_PASS                     (MXC_V_GCR_SCON_CHKRES_PASS << MXC_F_GCR_SCON_CHKRES_POS) /**< SCON_CHKRES_PASS Setting */
#define MXC_V_GCR_SCON_CHKRES_FAIL                     ((uint32_t)0x1UL) /**< SCON_CHKRES_FAIL Value */
#define MXC_S_GCR_SCON_CHKRES_FAIL                     (MXC_V_GCR_SCON_CHKRES_FAIL << MXC_F_GCR_SCON_CHKRES_POS) /**< SCON_CHKRES_FAIL Setting */

#define MXC_F_GCR_SCON_OVR_POS                         16 /**< SCON_OVR Position */
#define MXC_F_GCR_SCON_OVR                             ((uint32_t)(0x3UL << MXC_F_GCR_SCON_OVR_POS)) /**< SCON_OVR Mask */
#define MXC_V_GCR_SCON_OVR_0V9                         ((uint32_t)0x0UL) /**< SCON_OVR_0V9 Value */
#define MXC_S_GCR_SCON_OVR_0V9                         (MXC_V_GCR_SCON_OVR_0V9 << MXC_F_GCR_SCON_OVR_POS) /**< SCON_OVR_0V9 Setting */
#define MXC_V_GCR_SCON_OVR_1V                          ((uint32_t)0x1UL) /**< SCON_OVR_1V Value */
#define MXC_S_GCR_SCON_OVR_1V                          (MXC_V_GCR_SCON_OVR_1V << MXC_F_GCR_SCON_OVR_POS) /**< SCON_OVR_1V Setting */
#define MXC_V_GCR_SCON_OVR_1V1                         ((uint32_t)0x2UL) /**< SCON_OVR_1V1 Value */
#define MXC_S_GCR_SCON_OVR_1V1                         (MXC_V_GCR_SCON_OVR_1V1 << MXC_F_GCR_SCON_OVR_POS) /**< SCON_OVR_1V1 Setting */

/**@} end of group GCR_SCON_Register */

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

#define MXC_F_GCR_RST0_TIMER0_POS                      5 /**< RST0_TIMER0 Position */
#define MXC_F_GCR_RST0_TIMER0                          ((uint32_t)(0x1UL << MXC_F_GCR_RST0_TIMER0_POS)) /**< RST0_TIMER0 Mask */

#define MXC_F_GCR_RST0_TIMER1_POS                      6 /**< RST0_TIMER1 Position */
#define MXC_F_GCR_RST0_TIMER1                          ((uint32_t)(0x1UL << MXC_F_GCR_RST0_TIMER1_POS)) /**< RST0_TIMER1 Mask */

#define MXC_F_GCR_RST0_TIMER2_POS                      7 /**< RST0_TIMER2 Position */
#define MXC_F_GCR_RST0_TIMER2                          ((uint32_t)(0x1UL << MXC_F_GCR_RST0_TIMER2_POS)) /**< RST0_TIMER2 Mask */

#define MXC_F_GCR_RST0_TIMER3_POS                      8 /**< RST0_TIMER3 Position */
#define MXC_F_GCR_RST0_TIMER3                          ((uint32_t)(0x1UL << MXC_F_GCR_RST0_TIMER3_POS)) /**< RST0_TIMER3 Mask */

#define MXC_F_GCR_RST0_TIMER4_POS                      9 /**< RST0_TIMER4 Position */
#define MXC_F_GCR_RST0_TIMER4                          ((uint32_t)(0x1UL << MXC_F_GCR_RST0_TIMER4_POS)) /**< RST0_TIMER4 Mask */

#define MXC_F_GCR_RST0_TIMER5_POS                      10 /**< RST0_TIMER5 Position */
#define MXC_F_GCR_RST0_TIMER5                          ((uint32_t)(0x1UL << MXC_F_GCR_RST0_TIMER5_POS)) /**< RST0_TIMER5 Mask */

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

#define MXC_F_GCR_RST0_TPU_POS                         18 /**< RST0_TPU Position */
#define MXC_F_GCR_RST0_TPU                             ((uint32_t)(0x1UL << MXC_F_GCR_RST0_TPU_POS)) /**< RST0_TPU Mask */

#define MXC_F_GCR_RST0_HBC_POS                         21 /**< RST0_HBC Position */
#define MXC_F_GCR_RST0_HBC                             ((uint32_t)(0x1UL << MXC_F_GCR_RST0_HBC_POS)) /**< RST0_HBC Mask */

#define MXC_F_GCR_RST0_TFT_POS                         22 /**< RST0_TFT Position */
#define MXC_F_GCR_RST0_TFT                             ((uint32_t)(0x1UL << MXC_F_GCR_RST0_TFT_POS)) /**< RST0_TFT Mask */

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
 * @defgroup GCR_CLK_CTRL GCR_CLK_CTRL
 * @brief    Clock Control.
 * @{
 */
#define MXC_F_GCR_CLK_CTRL_SYSCLK_PRESCALE_POS         6 /**< CLK_CTRL_SYSCLK_PRESCALE Position */
#define MXC_F_GCR_CLK_CTRL_SYSCLK_PRESCALE             ((uint32_t)(0x7UL << MXC_F_GCR_CLK_CTRL_SYSCLK_PRESCALE_POS)) /**< CLK_CTRL_SYSCLK_PRESCALE Mask */
#define MXC_V_GCR_CLK_CTRL_SYSCLK_PRESCALE_DIV1        ((uint32_t)0x0UL) /**< CLK_CTRL_SYSCLK_PRESCALE_DIV1 Value */
#define MXC_S_GCR_CLK_CTRL_SYSCLK_PRESCALE_DIV1        (MXC_V_GCR_CLK_CTRL_SYSCLK_PRESCALE_DIV1 << MXC_F_GCR_CLK_CTRL_SYSCLK_PRESCALE_POS) /**< CLK_CTRL_SYSCLK_PRESCALE_DIV1 Setting */
#define MXC_V_GCR_CLK_CTRL_SYSCLK_PRESCALE_DIV2        ((uint32_t)0x1UL) /**< CLK_CTRL_SYSCLK_PRESCALE_DIV2 Value */
#define MXC_S_GCR_CLK_CTRL_SYSCLK_PRESCALE_DIV2        (MXC_V_GCR_CLK_CTRL_SYSCLK_PRESCALE_DIV2 << MXC_F_GCR_CLK_CTRL_SYSCLK_PRESCALE_POS) /**< CLK_CTRL_SYSCLK_PRESCALE_DIV2 Setting */
#define MXC_V_GCR_CLK_CTRL_SYSCLK_PRESCALE_DIV4        ((uint32_t)0x2UL) /**< CLK_CTRL_SYSCLK_PRESCALE_DIV4 Value */
#define MXC_S_GCR_CLK_CTRL_SYSCLK_PRESCALE_DIV4        (MXC_V_GCR_CLK_CTRL_SYSCLK_PRESCALE_DIV4 << MXC_F_GCR_CLK_CTRL_SYSCLK_PRESCALE_POS) /**< CLK_CTRL_SYSCLK_PRESCALE_DIV4 Setting */
#define MXC_V_GCR_CLK_CTRL_SYSCLK_PRESCALE_DIV8        ((uint32_t)0x3UL) /**< CLK_CTRL_SYSCLK_PRESCALE_DIV8 Value */
#define MXC_S_GCR_CLK_CTRL_SYSCLK_PRESCALE_DIV8        (MXC_V_GCR_CLK_CTRL_SYSCLK_PRESCALE_DIV8 << MXC_F_GCR_CLK_CTRL_SYSCLK_PRESCALE_POS) /**< CLK_CTRL_SYSCLK_PRESCALE_DIV8 Setting */
#define MXC_V_GCR_CLK_CTRL_SYSCLK_PRESCALE_DIV16       ((uint32_t)0x4UL) /**< CLK_CTRL_SYSCLK_PRESCALE_DIV16 Value */
#define MXC_S_GCR_CLK_CTRL_SYSCLK_PRESCALE_DIV16       (MXC_V_GCR_CLK_CTRL_SYSCLK_PRESCALE_DIV16 << MXC_F_GCR_CLK_CTRL_SYSCLK_PRESCALE_POS) /**< CLK_CTRL_SYSCLK_PRESCALE_DIV16 Setting */
#define MXC_V_GCR_CLK_CTRL_SYSCLK_PRESCALE_DIV32       ((uint32_t)0x5UL) /**< CLK_CTRL_SYSCLK_PRESCALE_DIV32 Value */
#define MXC_S_GCR_CLK_CTRL_SYSCLK_PRESCALE_DIV32       (MXC_V_GCR_CLK_CTRL_SYSCLK_PRESCALE_DIV32 << MXC_F_GCR_CLK_CTRL_SYSCLK_PRESCALE_POS) /**< CLK_CTRL_SYSCLK_PRESCALE_DIV32 Setting */
#define MXC_V_GCR_CLK_CTRL_SYSCLK_PRESCALE_DIV64       ((uint32_t)0x6UL) /**< CLK_CTRL_SYSCLK_PRESCALE_DIV64 Value */
#define MXC_S_GCR_CLK_CTRL_SYSCLK_PRESCALE_DIV64       (MXC_V_GCR_CLK_CTRL_SYSCLK_PRESCALE_DIV64 << MXC_F_GCR_CLK_CTRL_SYSCLK_PRESCALE_POS) /**< CLK_CTRL_SYSCLK_PRESCALE_DIV64 Setting */
#define MXC_V_GCR_CLK_CTRL_SYSCLK_PRESCALE_DIV128      ((uint32_t)0x7UL) /**< CLK_CTRL_SYSCLK_PRESCALE_DIV128 Value */
#define MXC_S_GCR_CLK_CTRL_SYSCLK_PRESCALE_DIV128      (MXC_V_GCR_CLK_CTRL_SYSCLK_PRESCALE_DIV128 << MXC_F_GCR_CLK_CTRL_SYSCLK_PRESCALE_POS) /**< CLK_CTRL_SYSCLK_PRESCALE_DIV128 Setting */

#define MXC_F_GCR_CLK_CTRL_SYSOSC_SEL_POS              9 /**< CLK_CTRL_SYSOSC_SEL Position */
#define MXC_F_GCR_CLK_CTRL_SYSOSC_SEL                  ((uint32_t)(0x7UL << MXC_F_GCR_CLK_CTRL_SYSOSC_SEL_POS)) /**< CLK_CTRL_SYSOSC_SEL Mask */
#define MXC_V_GCR_CLK_CTRL_SYSOSC_SEL_CRYPTO           ((uint32_t)0x0UL) /**< CLK_CTRL_SYSOSC_SEL_CRYPTO Value */
#define MXC_S_GCR_CLK_CTRL_SYSOSC_SEL_CRYPTO           (MXC_V_GCR_CLK_CTRL_SYSOSC_SEL_CRYPTO << MXC_F_GCR_CLK_CTRL_SYSOSC_SEL_POS) /**< CLK_CTRL_SYSOSC_SEL_CRYPTO Setting */
#define MXC_V_GCR_CLK_CTRL_SYSOSC_SEL_HFXIN            ((uint32_t)0x2UL) /**< CLK_CTRL_SYSOSC_SEL_HFXIN Value */
#define MXC_S_GCR_CLK_CTRL_SYSOSC_SEL_HFXIN            (MXC_V_GCR_CLK_CTRL_SYSOSC_SEL_HFXIN << MXC_F_GCR_CLK_CTRL_SYSOSC_SEL_POS) /**< CLK_CTRL_SYSOSC_SEL_HFXIN Setting */
#define MXC_V_GCR_CLK_CTRL_SYSOSC_SEL_NANORING         ((uint32_t)0x3UL) /**< CLK_CTRL_SYSOSC_SEL_NANORING Value */
#define MXC_S_GCR_CLK_CTRL_SYSOSC_SEL_NANORING         (MXC_V_GCR_CLK_CTRL_SYSOSC_SEL_NANORING << MXC_F_GCR_CLK_CTRL_SYSOSC_SEL_POS) /**< CLK_CTRL_SYSOSC_SEL_NANORING Setting */
#define MXC_V_GCR_CLK_CTRL_SYSOSC_SEL_HIRC96           ((uint32_t)0x4UL) /**< CLK_CTRL_SYSOSC_SEL_HIRC96 Value */
#define MXC_S_GCR_CLK_CTRL_SYSOSC_SEL_HIRC96           (MXC_V_GCR_CLK_CTRL_SYSOSC_SEL_HIRC96 << MXC_F_GCR_CLK_CTRL_SYSOSC_SEL_POS) /**< CLK_CTRL_SYSOSC_SEL_HIRC96 Setting */
#define MXC_V_GCR_CLK_CTRL_SYSOSC_SEL_HIRC8            ((uint32_t)0x5UL) /**< CLK_CTRL_SYSOSC_SEL_HIRC8 Value */
#define MXC_S_GCR_CLK_CTRL_SYSOSC_SEL_HIRC8            (MXC_V_GCR_CLK_CTRL_SYSOSC_SEL_HIRC8 << MXC_F_GCR_CLK_CTRL_SYSOSC_SEL_POS) /**< CLK_CTRL_SYSOSC_SEL_HIRC8 Setting */
#define MXC_V_GCR_CLK_CTRL_SYSOSC_SEL_X32K             ((uint32_t)0x6UL) /**< CLK_CTRL_SYSOSC_SEL_X32K Value */
#define MXC_S_GCR_CLK_CTRL_SYSOSC_SEL_X32K             (MXC_V_GCR_CLK_CTRL_SYSOSC_SEL_X32K << MXC_F_GCR_CLK_CTRL_SYSOSC_SEL_POS) /**< CLK_CTRL_SYSOSC_SEL_X32K Setting */

#define MXC_F_GCR_CLK_CTRL_SYSOSC_RDY_POS              13 /**< CLK_CTRL_SYSOSC_RDY Position */
#define MXC_F_GCR_CLK_CTRL_SYSOSC_RDY                  ((uint32_t)(0x1UL << MXC_F_GCR_CLK_CTRL_SYSOSC_RDY_POS)) /**< CLK_CTRL_SYSOSC_RDY Mask */
#define MXC_V_GCR_CLK_CTRL_SYSOSC_RDY_BUSY             ((uint32_t)0x0UL) /**< CLK_CTRL_SYSOSC_RDY_BUSY Value */
#define MXC_S_GCR_CLK_CTRL_SYSOSC_RDY_BUSY             (MXC_V_GCR_CLK_CTRL_SYSOSC_RDY_BUSY << MXC_F_GCR_CLK_CTRL_SYSOSC_RDY_POS) /**< CLK_CTRL_SYSOSC_RDY_BUSY Setting */
#define MXC_V_GCR_CLK_CTRL_SYSOSC_RDY_READY            ((uint32_t)0x1UL) /**< CLK_CTRL_SYSOSC_RDY_READY Value */
#define MXC_S_GCR_CLK_CTRL_SYSOSC_RDY_READY            (MXC_V_GCR_CLK_CTRL_SYSOSC_RDY_READY << MXC_F_GCR_CLK_CTRL_SYSOSC_RDY_POS) /**< CLK_CTRL_SYSOSC_RDY_READY Setting */

#define MXC_F_GCR_CLK_CTRL_CCD_POS                     15 /**< CLK_CTRL_CCD Position */
#define MXC_F_GCR_CLK_CTRL_CCD                         ((uint32_t)(0x1UL << MXC_F_GCR_CLK_CTRL_CCD_POS)) /**< CLK_CTRL_CCD Mask */
#define MXC_V_GCR_CLK_CTRL_CCD_NON_DIV                 ((uint32_t)0x0UL) /**< CLK_CTRL_CCD_NON_DIV Value */
#define MXC_S_GCR_CLK_CTRL_CCD_NON_DIV                 (MXC_V_GCR_CLK_CTRL_CCD_NON_DIV << MXC_F_GCR_CLK_CTRL_CCD_POS) /**< CLK_CTRL_CCD_NON_DIV Setting */
#define MXC_V_GCR_CLK_CTRL_CCD_DIV                     ((uint32_t)0x1UL) /**< CLK_CTRL_CCD_DIV Value */
#define MXC_S_GCR_CLK_CTRL_CCD_DIV                     (MXC_V_GCR_CLK_CTRL_CCD_DIV << MXC_F_GCR_CLK_CTRL_CCD_POS) /**< CLK_CTRL_CCD_DIV Setting */

#define MXC_F_GCR_CLK_CTRL_X32K_EN_POS                 17 /**< CLK_CTRL_X32K_EN Position */
#define MXC_F_GCR_CLK_CTRL_X32K_EN                     ((uint32_t)(0x1UL << MXC_F_GCR_CLK_CTRL_X32K_EN_POS)) /**< CLK_CTRL_X32K_EN Mask */

#define MXC_F_GCR_CLK_CTRL_CRYPTO_EN_POS               18 /**< CLK_CTRL_CRYPTO_EN Position */
#define MXC_F_GCR_CLK_CTRL_CRYPTO_EN                   ((uint32_t)(0x1UL << MXC_F_GCR_CLK_CTRL_CRYPTO_EN_POS)) /**< CLK_CTRL_CRYPTO_EN Mask */

#define MXC_F_GCR_CLK_CTRL_HIRC96_EN_POS               19 /**< CLK_CTRL_HIRC96_EN Position */
#define MXC_F_GCR_CLK_CTRL_HIRC96_EN                   ((uint32_t)(0x1UL << MXC_F_GCR_CLK_CTRL_HIRC96_EN_POS)) /**< CLK_CTRL_HIRC96_EN Mask */

#define MXC_F_GCR_CLK_CTRL_HIRC8_EN_POS                20 /**< CLK_CTRL_HIRC8_EN Position */
#define MXC_F_GCR_CLK_CTRL_HIRC8_EN                    ((uint32_t)(0x1UL << MXC_F_GCR_CLK_CTRL_HIRC8_EN_POS)) /**< CLK_CTRL_HIRC8_EN Mask */

#define MXC_F_GCR_CLK_CTRL_HIRC8_VS_POS                21 /**< CLK_CTRL_HIRC8_VS Position */
#define MXC_F_GCR_CLK_CTRL_HIRC8_VS                    ((uint32_t)(0x1UL << MXC_F_GCR_CLK_CTRL_HIRC8_VS_POS)) /**< CLK_CTRL_HIRC8_VS Mask */

#define MXC_F_GCR_CLK_CTRL_X32K_RDY_POS                25 /**< CLK_CTRL_X32K_RDY Position */
#define MXC_F_GCR_CLK_CTRL_X32K_RDY                    ((uint32_t)(0x1UL << MXC_F_GCR_CLK_CTRL_X32K_RDY_POS)) /**< CLK_CTRL_X32K_RDY Mask */

#define MXC_F_GCR_CLK_CTRL_CRYPTO_RDY_POS              26 /**< CLK_CTRL_CRYPTO_RDY Position */
#define MXC_F_GCR_CLK_CTRL_CRYPTO_RDY                  ((uint32_t)(0x1UL << MXC_F_GCR_CLK_CTRL_CRYPTO_RDY_POS)) /**< CLK_CTRL_CRYPTO_RDY Mask */

#define MXC_F_GCR_CLK_CTRL_HIRC96_RDY_POS              27 /**< CLK_CTRL_HIRC96_RDY Position */
#define MXC_F_GCR_CLK_CTRL_HIRC96_RDY                  ((uint32_t)(0x1UL << MXC_F_GCR_CLK_CTRL_HIRC96_RDY_POS)) /**< CLK_CTRL_HIRC96_RDY Mask */

#define MXC_F_GCR_CLK_CTRL_HIRC8_RDY_POS               28 /**< CLK_CTRL_HIRC8_RDY Position */
#define MXC_F_GCR_CLK_CTRL_HIRC8_RDY                   ((uint32_t)(0x1UL << MXC_F_GCR_CLK_CTRL_HIRC8_RDY_POS)) /**< CLK_CTRL_HIRC8_RDY Mask */

#define MXC_F_GCR_CLK_CTRL_NANORING_RDY_POS            29 /**< CLK_CTRL_NANORING_RDY Position */
#define MXC_F_GCR_CLK_CTRL_NANORING_RDY                ((uint32_t)(0x1UL << MXC_F_GCR_CLK_CTRL_NANORING_RDY_POS)) /**< CLK_CTRL_NANORING_RDY Mask */

/**@} end of group GCR_CLK_CTRL_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_PMR GCR_PMR
 * @brief    Power Management.
 * @{
 */
#define MXC_F_GCR_PMR_MODE_POS                         0 /**< PMR_MODE Position */
#define MXC_F_GCR_PMR_MODE                             ((uint32_t)(0x7UL << MXC_F_GCR_PMR_MODE_POS)) /**< PMR_MODE Mask */
#define MXC_V_GCR_PMR_MODE_ACTIVE                      ((uint32_t)0x0UL) /**< PMR_MODE_ACTIVE Value */
#define MXC_S_GCR_PMR_MODE_ACTIVE                      (MXC_V_GCR_PMR_MODE_ACTIVE << MXC_F_GCR_PMR_MODE_POS) /**< PMR_MODE_ACTIVE Setting */
#define MXC_V_GCR_PMR_MODE_SHUTDOWN                    ((uint32_t)0x3UL) /**< PMR_MODE_SHUTDOWN Value */
#define MXC_S_GCR_PMR_MODE_SHUTDOWN                    (MXC_V_GCR_PMR_MODE_SHUTDOWN << MXC_F_GCR_PMR_MODE_POS) /**< PMR_MODE_SHUTDOWN Setting */
#define MXC_V_GCR_PMR_MODE_BACKUP                      ((uint32_t)0x4UL) /**< PMR_MODE_BACKUP Value */
#define MXC_S_GCR_PMR_MODE_BACKUP                      (MXC_V_GCR_PMR_MODE_BACKUP << MXC_F_GCR_PMR_MODE_POS) /**< PMR_MODE_BACKUP Setting */

#define MXC_F_GCR_PMR_GPIOWKEN_POS                     4 /**< PMR_GPIOWKEN Position */
#define MXC_F_GCR_PMR_GPIOWKEN                         ((uint32_t)(0x1UL << MXC_F_GCR_PMR_GPIOWKEN_POS)) /**< PMR_GPIOWKEN Mask */

#define MXC_F_GCR_PMR_RTCWKEN_POS                      5 /**< PMR_RTCWKEN Position */
#define MXC_F_GCR_PMR_RTCWKEN                          ((uint32_t)(0x1UL << MXC_F_GCR_PMR_RTCWKEN_POS)) /**< PMR_RTCWKEN Mask */

#define MXC_F_GCR_PMR_USBWKEN_POS                      6 /**< PMR_USBWKEN Position */
#define MXC_F_GCR_PMR_USBWKEN                          ((uint32_t)(0x1UL << MXC_F_GCR_PMR_USBWKEN_POS)) /**< PMR_USBWKEN Mask */

#define MXC_F_GCR_PMR_CRYPTOPD_POS                     15 /**< PMR_CRYPTOPD Position */
#define MXC_F_GCR_PMR_CRYPTOPD                         ((uint32_t)(0x1UL << MXC_F_GCR_PMR_CRYPTOPD_POS)) /**< PMR_CRYPTOPD Mask */
#define MXC_V_GCR_PMR_CRYPTOPD_ACTIVE                  ((uint32_t)0x0UL) /**< PMR_CRYPTOPD_ACTIVE Value */
#define MXC_S_GCR_PMR_CRYPTOPD_ACTIVE                  (MXC_V_GCR_PMR_CRYPTOPD_ACTIVE << MXC_F_GCR_PMR_CRYPTOPD_POS) /**< PMR_CRYPTOPD_ACTIVE Setting */
#define MXC_V_GCR_PMR_CRYPTOPD_DEEPSLEEP               ((uint32_t)0x1UL) /**< PMR_CRYPTOPD_DEEPSLEEP Value */
#define MXC_S_GCR_PMR_CRYPTOPD_DEEPSLEEP               (MXC_V_GCR_PMR_CRYPTOPD_DEEPSLEEP << MXC_F_GCR_PMR_CRYPTOPD_POS) /**< PMR_CRYPTOPD_DEEPSLEEP Setting */

#define MXC_F_GCR_PMR_HIRC96PD_POS                     16 /**< PMR_HIRC96PD Position */
#define MXC_F_GCR_PMR_HIRC96PD                         ((uint32_t)(0x1UL << MXC_F_GCR_PMR_HIRC96PD_POS)) /**< PMR_HIRC96PD Mask */
#define MXC_V_GCR_PMR_HIRC96PD_ACTIVE                  ((uint32_t)0x0UL) /**< PMR_HIRC96PD_ACTIVE Value */
#define MXC_S_GCR_PMR_HIRC96PD_ACTIVE                  (MXC_V_GCR_PMR_HIRC96PD_ACTIVE << MXC_F_GCR_PMR_HIRC96PD_POS) /**< PMR_HIRC96PD_ACTIVE Setting */
#define MXC_V_GCR_PMR_HIRC96PD_DEEPSLEEP               ((uint32_t)0x1UL) /**< PMR_HIRC96PD_DEEPSLEEP Value */
#define MXC_S_GCR_PMR_HIRC96PD_DEEPSLEEP               (MXC_V_GCR_PMR_HIRC96PD_DEEPSLEEP << MXC_F_GCR_PMR_HIRC96PD_POS) /**< PMR_HIRC96PD_DEEPSLEEP Setting */

#define MXC_F_GCR_PMR_HIRC8PD_POS                      17 /**< PMR_HIRC8PD Position */
#define MXC_F_GCR_PMR_HIRC8PD                          ((uint32_t)(0x1UL << MXC_F_GCR_PMR_HIRC8PD_POS)) /**< PMR_HIRC8PD Mask */
#define MXC_V_GCR_PMR_HIRC8PD_ACTIVE                   ((uint32_t)0x0UL) /**< PMR_HIRC8PD_ACTIVE Value */
#define MXC_S_GCR_PMR_HIRC8PD_ACTIVE                   (MXC_V_GCR_PMR_HIRC8PD_ACTIVE << MXC_F_GCR_PMR_HIRC8PD_POS) /**< PMR_HIRC8PD_ACTIVE Setting */
#define MXC_V_GCR_PMR_HIRC8PD_DEEPSLEEP                ((uint32_t)0x1UL) /**< PMR_HIRC8PD_DEEPSLEEP Value */
#define MXC_S_GCR_PMR_HIRC8PD_DEEPSLEEP                (MXC_V_GCR_PMR_HIRC8PD_DEEPSLEEP << MXC_F_GCR_PMR_HIRC8PD_POS) /**< PMR_HIRC8PD_DEEPSLEEP Setting */

/**@} end of group GCR_PMR_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_PCLK_DIV GCR_PCLK_DIV
 * @brief    Peripheral Clock Divider.
 * @{
 */
#define MXC_F_GCR_PCLK_DIV_SDHCFRQ_POS                 7 /**< PCLK_DIV_SDHCFRQ Position */
#define MXC_F_GCR_PCLK_DIV_SDHCFRQ                     ((uint32_t)(0x1UL << MXC_F_GCR_PCLK_DIV_SDHCFRQ_POS)) /**< PCLK_DIV_SDHCFRQ Mask */
#define MXC_V_GCR_PCLK_DIV_SDHCFRQ_60M                 ((uint32_t)0x0UL) /**< PCLK_DIV_SDHCFRQ_60M Value */
#define MXC_S_GCR_PCLK_DIV_SDHCFRQ_60M                 (MXC_V_GCR_PCLK_DIV_SDHCFRQ_60M << MXC_F_GCR_PCLK_DIV_SDHCFRQ_POS) /**< PCLK_DIV_SDHCFRQ_60M Setting */
#define MXC_V_GCR_PCLK_DIV_SDHCFRQ_50M                 ((uint32_t)0x1UL) /**< PCLK_DIV_SDHCFRQ_50M Value */
#define MXC_S_GCR_PCLK_DIV_SDHCFRQ_50M                 (MXC_V_GCR_PCLK_DIV_SDHCFRQ_50M << MXC_F_GCR_PCLK_DIV_SDHCFRQ_POS) /**< PCLK_DIV_SDHCFRQ_50M Setting */

#define MXC_F_GCR_PCLK_DIV_ADCFRQ_POS                  10 /**< PCLK_DIV_ADCFRQ Position */
#define MXC_F_GCR_PCLK_DIV_ADCFRQ                      ((uint32_t)(0xFUL << MXC_F_GCR_PCLK_DIV_ADCFRQ_POS)) /**< PCLK_DIV_ADCFRQ Mask */
#define MXC_V_GCR_PCLK_DIV_ADCFRQ_DIV2                 ((uint32_t)0x2UL) /**< PCLK_DIV_ADCFRQ_DIV2 Value */
#define MXC_S_GCR_PCLK_DIV_ADCFRQ_DIV2                 (MXC_V_GCR_PCLK_DIV_ADCFRQ_DIV2 << MXC_F_GCR_PCLK_DIV_ADCFRQ_POS) /**< PCLK_DIV_ADCFRQ_DIV2 Setting */
#define MXC_V_GCR_PCLK_DIV_ADCFRQ_DIV3                 ((uint32_t)0x3UL) /**< PCLK_DIV_ADCFRQ_DIV3 Value */
#define MXC_S_GCR_PCLK_DIV_ADCFRQ_DIV3                 (MXC_V_GCR_PCLK_DIV_ADCFRQ_DIV3 << MXC_F_GCR_PCLK_DIV_ADCFRQ_POS) /**< PCLK_DIV_ADCFRQ_DIV3 Setting */
#define MXC_V_GCR_PCLK_DIV_ADCFRQ_DIV4                 ((uint32_t)0x4UL) /**< PCLK_DIV_ADCFRQ_DIV4 Value */
#define MXC_S_GCR_PCLK_DIV_ADCFRQ_DIV4                 (MXC_V_GCR_PCLK_DIV_ADCFRQ_DIV4 << MXC_F_GCR_PCLK_DIV_ADCFRQ_POS) /**< PCLK_DIV_ADCFRQ_DIV4 Setting */
#define MXC_V_GCR_PCLK_DIV_ADCFRQ_DIV5                 ((uint32_t)0x5UL) /**< PCLK_DIV_ADCFRQ_DIV5 Value */
#define MXC_S_GCR_PCLK_DIV_ADCFRQ_DIV5                 (MXC_V_GCR_PCLK_DIV_ADCFRQ_DIV5 << MXC_F_GCR_PCLK_DIV_ADCFRQ_POS) /**< PCLK_DIV_ADCFRQ_DIV5 Setting */
#define MXC_V_GCR_PCLK_DIV_ADCFRQ_DIV6                 ((uint32_t)0x6UL) /**< PCLK_DIV_ADCFRQ_DIV6 Value */
#define MXC_S_GCR_PCLK_DIV_ADCFRQ_DIV6                 (MXC_V_GCR_PCLK_DIV_ADCFRQ_DIV6 << MXC_F_GCR_PCLK_DIV_ADCFRQ_POS) /**< PCLK_DIV_ADCFRQ_DIV6 Setting */
#define MXC_V_GCR_PCLK_DIV_ADCFRQ_DIV7                 ((uint32_t)0x7UL) /**< PCLK_DIV_ADCFRQ_DIV7 Value */
#define MXC_S_GCR_PCLK_DIV_ADCFRQ_DIV7                 (MXC_V_GCR_PCLK_DIV_ADCFRQ_DIV7 << MXC_F_GCR_PCLK_DIV_ADCFRQ_POS) /**< PCLK_DIV_ADCFRQ_DIV7 Setting */
#define MXC_V_GCR_PCLK_DIV_ADCFRQ_DIV8                 ((uint32_t)0x8UL) /**< PCLK_DIV_ADCFRQ_DIV8 Value */
#define MXC_S_GCR_PCLK_DIV_ADCFRQ_DIV8                 (MXC_V_GCR_PCLK_DIV_ADCFRQ_DIV8 << MXC_F_GCR_PCLK_DIV_ADCFRQ_POS) /**< PCLK_DIV_ADCFRQ_DIV8 Setting */
#define MXC_V_GCR_PCLK_DIV_ADCFRQ_DIV9                 ((uint32_t)0x9UL) /**< PCLK_DIV_ADCFRQ_DIV9 Value */
#define MXC_S_GCR_PCLK_DIV_ADCFRQ_DIV9                 (MXC_V_GCR_PCLK_DIV_ADCFRQ_DIV9 << MXC_F_GCR_PCLK_DIV_ADCFRQ_POS) /**< PCLK_DIV_ADCFRQ_DIV9 Setting */
#define MXC_V_GCR_PCLK_DIV_ADCFRQ_DIV10                ((uint32_t)0xAUL) /**< PCLK_DIV_ADCFRQ_DIV10 Value */
#define MXC_S_GCR_PCLK_DIV_ADCFRQ_DIV10                (MXC_V_GCR_PCLK_DIV_ADCFRQ_DIV10 << MXC_F_GCR_PCLK_DIV_ADCFRQ_POS) /**< PCLK_DIV_ADCFRQ_DIV10 Setting */
#define MXC_V_GCR_PCLK_DIV_ADCFRQ_DIV11                ((uint32_t)0xBUL) /**< PCLK_DIV_ADCFRQ_DIV11 Value */
#define MXC_S_GCR_PCLK_DIV_ADCFRQ_DIV11                (MXC_V_GCR_PCLK_DIV_ADCFRQ_DIV11 << MXC_F_GCR_PCLK_DIV_ADCFRQ_POS) /**< PCLK_DIV_ADCFRQ_DIV11 Setting */
#define MXC_V_GCR_PCLK_DIV_ADCFRQ_DIV12                ((uint32_t)0xCUL) /**< PCLK_DIV_ADCFRQ_DIV12 Value */
#define MXC_S_GCR_PCLK_DIV_ADCFRQ_DIV12                (MXC_V_GCR_PCLK_DIV_ADCFRQ_DIV12 << MXC_F_GCR_PCLK_DIV_ADCFRQ_POS) /**< PCLK_DIV_ADCFRQ_DIV12 Setting */
#define MXC_V_GCR_PCLK_DIV_ADCFRQ_DIV13                ((uint32_t)0xDUL) /**< PCLK_DIV_ADCFRQ_DIV13 Value */
#define MXC_S_GCR_PCLK_DIV_ADCFRQ_DIV13                (MXC_V_GCR_PCLK_DIV_ADCFRQ_DIV13 << MXC_F_GCR_PCLK_DIV_ADCFRQ_POS) /**< PCLK_DIV_ADCFRQ_DIV13 Setting */
#define MXC_V_GCR_PCLK_DIV_ADCFRQ_DIV14                ((uint32_t)0xEUL) /**< PCLK_DIV_ADCFRQ_DIV14 Value */
#define MXC_S_GCR_PCLK_DIV_ADCFRQ_DIV14                (MXC_V_GCR_PCLK_DIV_ADCFRQ_DIV14 << MXC_F_GCR_PCLK_DIV_ADCFRQ_POS) /**< PCLK_DIV_ADCFRQ_DIV14 Setting */
#define MXC_V_GCR_PCLK_DIV_ADCFRQ_DIV15                ((uint32_t)0xFUL) /**< PCLK_DIV_ADCFRQ_DIV15 Value */
#define MXC_S_GCR_PCLK_DIV_ADCFRQ_DIV15                (MXC_V_GCR_PCLK_DIV_ADCFRQ_DIV15 << MXC_F_GCR_PCLK_DIV_ADCFRQ_POS) /**< PCLK_DIV_ADCFRQ_DIV15 Setting */

#define MXC_F_GCR_PCLK_DIV_AONDIV_POS                  14 /**< PCLK_DIV_AONDIV Position */
#define MXC_F_GCR_PCLK_DIV_AONDIV                      ((uint32_t)(0x3UL << MXC_F_GCR_PCLK_DIV_AONDIV_POS)) /**< PCLK_DIV_AONDIV Mask */
#define MXC_V_GCR_PCLK_DIV_AONDIV_DIV4                 ((uint32_t)0x0UL) /**< PCLK_DIV_AONDIV_DIV4 Value */
#define MXC_S_GCR_PCLK_DIV_AONDIV_DIV4                 (MXC_V_GCR_PCLK_DIV_AONDIV_DIV4 << MXC_F_GCR_PCLK_DIV_AONDIV_POS) /**< PCLK_DIV_AONDIV_DIV4 Setting */
#define MXC_V_GCR_PCLK_DIV_AONDIV_DIV8                 ((uint32_t)0x1UL) /**< PCLK_DIV_AONDIV_DIV8 Value */
#define MXC_S_GCR_PCLK_DIV_AONDIV_DIV8                 (MXC_V_GCR_PCLK_DIV_AONDIV_DIV8 << MXC_F_GCR_PCLK_DIV_AONDIV_POS) /**< PCLK_DIV_AONDIV_DIV8 Setting */
#define MXC_V_GCR_PCLK_DIV_AONDIV_DIV16                ((uint32_t)0x2UL) /**< PCLK_DIV_AONDIV_DIV16 Value */
#define MXC_S_GCR_PCLK_DIV_AONDIV_DIV16                (MXC_V_GCR_PCLK_DIV_AONDIV_DIV16 << MXC_F_GCR_PCLK_DIV_AONDIV_POS) /**< PCLK_DIV_AONDIV_DIV16 Setting */
#define MXC_V_GCR_PCLK_DIV_AONDIV_DIV32                ((uint32_t)0x3UL) /**< PCLK_DIV_AONDIV_DIV32 Value */
#define MXC_S_GCR_PCLK_DIV_AONDIV_DIV32                (MXC_V_GCR_PCLK_DIV_AONDIV_DIV32 << MXC_F_GCR_PCLK_DIV_AONDIV_POS) /**< PCLK_DIV_AONDIV_DIV32 Setting */

/**@} end of group GCR_PCLK_DIV_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_PCLK_DIS0 GCR_PCLK_DIS0
 * @brief    Peripheral Clock Disable.
 * @{
 */
#define MXC_F_GCR_PCLK_DIS0_GPIO0_POS                  0 /**< PCLK_DIS0_GPIO0 Position */
#define MXC_F_GCR_PCLK_DIS0_GPIO0                      ((uint32_t)(0x1UL << MXC_F_GCR_PCLK_DIS0_GPIO0_POS)) /**< PCLK_DIS0_GPIO0 Mask */
#define MXC_V_GCR_PCLK_DIS0_GPIO0_EN                   ((uint32_t)0x0UL) /**< PCLK_DIS0_GPIO0_EN Value */
#define MXC_S_GCR_PCLK_DIS0_GPIO0_EN                   (MXC_V_GCR_PCLK_DIS0_GPIO0_EN << MXC_F_GCR_PCLK_DIS0_GPIO0_POS) /**< PCLK_DIS0_GPIO0_EN Setting */
#define MXC_V_GCR_PCLK_DIS0_GPIO0_DIS                  ((uint32_t)0x1UL) /**< PCLK_DIS0_GPIO0_DIS Value */
#define MXC_S_GCR_PCLK_DIS0_GPIO0_DIS                  (MXC_V_GCR_PCLK_DIS0_GPIO0_DIS << MXC_F_GCR_PCLK_DIS0_GPIO0_POS) /**< PCLK_DIS0_GPIO0_DIS Setting */

#define MXC_F_GCR_PCLK_DIS0_GPIO1_POS                  1 /**< PCLK_DIS0_GPIO1 Position */
#define MXC_F_GCR_PCLK_DIS0_GPIO1                      ((uint32_t)(0x1UL << MXC_F_GCR_PCLK_DIS0_GPIO1_POS)) /**< PCLK_DIS0_GPIO1 Mask */
#define MXC_V_GCR_PCLK_DIS0_GPIO1_EN                   ((uint32_t)0x0UL) /**< PCLK_DIS0_GPIO1_EN Value */
#define MXC_S_GCR_PCLK_DIS0_GPIO1_EN                   (MXC_V_GCR_PCLK_DIS0_GPIO1_EN << MXC_F_GCR_PCLK_DIS0_GPIO1_POS) /**< PCLK_DIS0_GPIO1_EN Setting */
#define MXC_V_GCR_PCLK_DIS0_GPIO1_DIS                  ((uint32_t)0x1UL) /**< PCLK_DIS0_GPIO1_DIS Value */
#define MXC_S_GCR_PCLK_DIS0_GPIO1_DIS                  (MXC_V_GCR_PCLK_DIS0_GPIO1_DIS << MXC_F_GCR_PCLK_DIS0_GPIO1_POS) /**< PCLK_DIS0_GPIO1_DIS Setting */

#define MXC_F_GCR_PCLK_DIS0_GPIO2_POS                  2 /**< PCLK_DIS0_GPIO2 Position */
#define MXC_F_GCR_PCLK_DIS0_GPIO2                      ((uint32_t)(0x1UL << MXC_F_GCR_PCLK_DIS0_GPIO2_POS)) /**< PCLK_DIS0_GPIO2 Mask */
#define MXC_V_GCR_PCLK_DIS0_GPIO2_EN                   ((uint32_t)0x0UL) /**< PCLK_DIS0_GPIO2_EN Value */
#define MXC_S_GCR_PCLK_DIS0_GPIO2_EN                   (MXC_V_GCR_PCLK_DIS0_GPIO2_EN << MXC_F_GCR_PCLK_DIS0_GPIO2_POS) /**< PCLK_DIS0_GPIO2_EN Setting */
#define MXC_V_GCR_PCLK_DIS0_GPIO2_DIS                  ((uint32_t)0x1UL) /**< PCLK_DIS0_GPIO2_DIS Value */
#define MXC_S_GCR_PCLK_DIS0_GPIO2_DIS                  (MXC_V_GCR_PCLK_DIS0_GPIO2_DIS << MXC_F_GCR_PCLK_DIS0_GPIO2_POS) /**< PCLK_DIS0_GPIO2_DIS Setting */

#define MXC_F_GCR_PCLK_DIS0_USB_POS                    3 /**< PCLK_DIS0_USB Position */
#define MXC_F_GCR_PCLK_DIS0_USB                        ((uint32_t)(0x1UL << MXC_F_GCR_PCLK_DIS0_USB_POS)) /**< PCLK_DIS0_USB Mask */
#define MXC_V_GCR_PCLK_DIS0_USB_EN                     ((uint32_t)0x0UL) /**< PCLK_DIS0_USB_EN Value */
#define MXC_S_GCR_PCLK_DIS0_USB_EN                     (MXC_V_GCR_PCLK_DIS0_USB_EN << MXC_F_GCR_PCLK_DIS0_USB_POS) /**< PCLK_DIS0_USB_EN Setting */
#define MXC_V_GCR_PCLK_DIS0_USB_DIS                    ((uint32_t)0x1UL) /**< PCLK_DIS0_USB_DIS Value */
#define MXC_S_GCR_PCLK_DIS0_USB_DIS                    (MXC_V_GCR_PCLK_DIS0_USB_DIS << MXC_F_GCR_PCLK_DIS0_USB_POS) /**< PCLK_DIS0_USB_DIS Setting */

#define MXC_F_GCR_PCLK_DIS0_TFT_POS                    4 /**< PCLK_DIS0_TFT Position */
#define MXC_F_GCR_PCLK_DIS0_TFT                        ((uint32_t)(0x1UL << MXC_F_GCR_PCLK_DIS0_TFT_POS)) /**< PCLK_DIS0_TFT Mask */
#define MXC_V_GCR_PCLK_DIS0_TFT_EN                     ((uint32_t)0x0UL) /**< PCLK_DIS0_TFT_EN Value */
#define MXC_S_GCR_PCLK_DIS0_TFT_EN                     (MXC_V_GCR_PCLK_DIS0_TFT_EN << MXC_F_GCR_PCLK_DIS0_TFT_POS) /**< PCLK_DIS0_TFT_EN Setting */
#define MXC_V_GCR_PCLK_DIS0_TFT_DIS                    ((uint32_t)0x1UL) /**< PCLK_DIS0_TFT_DIS Value */
#define MXC_S_GCR_PCLK_DIS0_TFT_DIS                    (MXC_V_GCR_PCLK_DIS0_TFT_DIS << MXC_F_GCR_PCLK_DIS0_TFT_POS) /**< PCLK_DIS0_TFT_DIS Setting */

#define MXC_F_GCR_PCLK_DIS0_DMA_POS                    5 /**< PCLK_DIS0_DMA Position */
#define MXC_F_GCR_PCLK_DIS0_DMA                        ((uint32_t)(0x1UL << MXC_F_GCR_PCLK_DIS0_DMA_POS)) /**< PCLK_DIS0_DMA Mask */
#define MXC_V_GCR_PCLK_DIS0_DMA_EN                     ((uint32_t)0x0UL) /**< PCLK_DIS0_DMA_EN Value */
#define MXC_S_GCR_PCLK_DIS0_DMA_EN                     (MXC_V_GCR_PCLK_DIS0_DMA_EN << MXC_F_GCR_PCLK_DIS0_DMA_POS) /**< PCLK_DIS0_DMA_EN Setting */
#define MXC_V_GCR_PCLK_DIS0_DMA_DIS                    ((uint32_t)0x1UL) /**< PCLK_DIS0_DMA_DIS Value */
#define MXC_S_GCR_PCLK_DIS0_DMA_DIS                    (MXC_V_GCR_PCLK_DIS0_DMA_DIS << MXC_F_GCR_PCLK_DIS0_DMA_POS) /**< PCLK_DIS0_DMA_DIS Setting */

#define MXC_F_GCR_PCLK_DIS0_SPI0_POS                   6 /**< PCLK_DIS0_SPI0 Position */
#define MXC_F_GCR_PCLK_DIS0_SPI0                       ((uint32_t)(0x1UL << MXC_F_GCR_PCLK_DIS0_SPI0_POS)) /**< PCLK_DIS0_SPI0 Mask */
#define MXC_V_GCR_PCLK_DIS0_SPI0_EN                    ((uint32_t)0x0UL) /**< PCLK_DIS0_SPI0_EN Value */
#define MXC_S_GCR_PCLK_DIS0_SPI0_EN                    (MXC_V_GCR_PCLK_DIS0_SPI0_EN << MXC_F_GCR_PCLK_DIS0_SPI0_POS) /**< PCLK_DIS0_SPI0_EN Setting */
#define MXC_V_GCR_PCLK_DIS0_SPI0_DIS                   ((uint32_t)0x1UL) /**< PCLK_DIS0_SPI0_DIS Value */
#define MXC_S_GCR_PCLK_DIS0_SPI0_DIS                   (MXC_V_GCR_PCLK_DIS0_SPI0_DIS << MXC_F_GCR_PCLK_DIS0_SPI0_POS) /**< PCLK_DIS0_SPI0_DIS Setting */

#define MXC_F_GCR_PCLK_DIS0_SPI1_POS                   7 /**< PCLK_DIS0_SPI1 Position */
#define MXC_F_GCR_PCLK_DIS0_SPI1                       ((uint32_t)(0x1UL << MXC_F_GCR_PCLK_DIS0_SPI1_POS)) /**< PCLK_DIS0_SPI1 Mask */
#define MXC_V_GCR_PCLK_DIS0_SPI1_EN                    ((uint32_t)0x0UL) /**< PCLK_DIS0_SPI1_EN Value */
#define MXC_S_GCR_PCLK_DIS0_SPI1_EN                    (MXC_V_GCR_PCLK_DIS0_SPI1_EN << MXC_F_GCR_PCLK_DIS0_SPI1_POS) /**< PCLK_DIS0_SPI1_EN Setting */
#define MXC_V_GCR_PCLK_DIS0_SPI1_DIS                   ((uint32_t)0x1UL) /**< PCLK_DIS0_SPI1_DIS Value */
#define MXC_S_GCR_PCLK_DIS0_SPI1_DIS                   (MXC_V_GCR_PCLK_DIS0_SPI1_DIS << MXC_F_GCR_PCLK_DIS0_SPI1_POS) /**< PCLK_DIS0_SPI1_DIS Setting */

#define MXC_F_GCR_PCLK_DIS0_SPI2_POS                   8 /**< PCLK_DIS0_SPI2 Position */
#define MXC_F_GCR_PCLK_DIS0_SPI2                       ((uint32_t)(0x1UL << MXC_F_GCR_PCLK_DIS0_SPI2_POS)) /**< PCLK_DIS0_SPI2 Mask */
#define MXC_V_GCR_PCLK_DIS0_SPI2_EN                    ((uint32_t)0x0UL) /**< PCLK_DIS0_SPI2_EN Value */
#define MXC_S_GCR_PCLK_DIS0_SPI2_EN                    (MXC_V_GCR_PCLK_DIS0_SPI2_EN << MXC_F_GCR_PCLK_DIS0_SPI2_POS) /**< PCLK_DIS0_SPI2_EN Setting */
#define MXC_V_GCR_PCLK_DIS0_SPI2_DIS                   ((uint32_t)0x1UL) /**< PCLK_DIS0_SPI2_DIS Value */
#define MXC_S_GCR_PCLK_DIS0_SPI2_DIS                   (MXC_V_GCR_PCLK_DIS0_SPI2_DIS << MXC_F_GCR_PCLK_DIS0_SPI2_POS) /**< PCLK_DIS0_SPI2_DIS Setting */

#define MXC_F_GCR_PCLK_DIS0_UART0_POS                  9 /**< PCLK_DIS0_UART0 Position */
#define MXC_F_GCR_PCLK_DIS0_UART0                      ((uint32_t)(0x1UL << MXC_F_GCR_PCLK_DIS0_UART0_POS)) /**< PCLK_DIS0_UART0 Mask */
#define MXC_V_GCR_PCLK_DIS0_UART0_EN                   ((uint32_t)0x0UL) /**< PCLK_DIS0_UART0_EN Value */
#define MXC_S_GCR_PCLK_DIS0_UART0_EN                   (MXC_V_GCR_PCLK_DIS0_UART0_EN << MXC_F_GCR_PCLK_DIS0_UART0_POS) /**< PCLK_DIS0_UART0_EN Setting */
#define MXC_V_GCR_PCLK_DIS0_UART0_DIS                  ((uint32_t)0x1UL) /**< PCLK_DIS0_UART0_DIS Value */
#define MXC_S_GCR_PCLK_DIS0_UART0_DIS                  (MXC_V_GCR_PCLK_DIS0_UART0_DIS << MXC_F_GCR_PCLK_DIS0_UART0_POS) /**< PCLK_DIS0_UART0_DIS Setting */

#define MXC_F_GCR_PCLK_DIS0_UART1_POS                  10 /**< PCLK_DIS0_UART1 Position */
#define MXC_F_GCR_PCLK_DIS0_UART1                      ((uint32_t)(0x1UL << MXC_F_GCR_PCLK_DIS0_UART1_POS)) /**< PCLK_DIS0_UART1 Mask */
#define MXC_V_GCR_PCLK_DIS0_UART1_EN                   ((uint32_t)0x0UL) /**< PCLK_DIS0_UART1_EN Value */
#define MXC_S_GCR_PCLK_DIS0_UART1_EN                   (MXC_V_GCR_PCLK_DIS0_UART1_EN << MXC_F_GCR_PCLK_DIS0_UART1_POS) /**< PCLK_DIS0_UART1_EN Setting */
#define MXC_V_GCR_PCLK_DIS0_UART1_DIS                  ((uint32_t)0x1UL) /**< PCLK_DIS0_UART1_DIS Value */
#define MXC_S_GCR_PCLK_DIS0_UART1_DIS                  (MXC_V_GCR_PCLK_DIS0_UART1_DIS << MXC_F_GCR_PCLK_DIS0_UART1_POS) /**< PCLK_DIS0_UART1_DIS Setting */

#define MXC_F_GCR_PCLK_DIS0_I2C0_POS                   13 /**< PCLK_DIS0_I2C0 Position */
#define MXC_F_GCR_PCLK_DIS0_I2C0                       ((uint32_t)(0x1UL << MXC_F_GCR_PCLK_DIS0_I2C0_POS)) /**< PCLK_DIS0_I2C0 Mask */
#define MXC_V_GCR_PCLK_DIS0_I2C0_EN                    ((uint32_t)0x0UL) /**< PCLK_DIS0_I2C0_EN Value */
#define MXC_S_GCR_PCLK_DIS0_I2C0_EN                    (MXC_V_GCR_PCLK_DIS0_I2C0_EN << MXC_F_GCR_PCLK_DIS0_I2C0_POS) /**< PCLK_DIS0_I2C0_EN Setting */
#define MXC_V_GCR_PCLK_DIS0_I2C0_DIS                   ((uint32_t)0x1UL) /**< PCLK_DIS0_I2C0_DIS Value */
#define MXC_S_GCR_PCLK_DIS0_I2C0_DIS                   (MXC_V_GCR_PCLK_DIS0_I2C0_DIS << MXC_F_GCR_PCLK_DIS0_I2C0_POS) /**< PCLK_DIS0_I2C0_DIS Setting */

#define MXC_F_GCR_PCLK_DIS0_TPU_POS                    14 /**< PCLK_DIS0_TPU Position */
#define MXC_F_GCR_PCLK_DIS0_TPU                        ((uint32_t)(0x1UL << MXC_F_GCR_PCLK_DIS0_TPU_POS)) /**< PCLK_DIS0_TPU Mask */
#define MXC_V_GCR_PCLK_DIS0_TPU_EN                     ((uint32_t)0x0UL) /**< PCLK_DIS0_TPU_EN Value */
#define MXC_S_GCR_PCLK_DIS0_TPU_EN                     (MXC_V_GCR_PCLK_DIS0_TPU_EN << MXC_F_GCR_PCLK_DIS0_TPU_POS) /**< PCLK_DIS0_TPU_EN Setting */
#define MXC_V_GCR_PCLK_DIS0_TPU_DIS                    ((uint32_t)0x1UL) /**< PCLK_DIS0_TPU_DIS Value */
#define MXC_S_GCR_PCLK_DIS0_TPU_DIS                    (MXC_V_GCR_PCLK_DIS0_TPU_DIS << MXC_F_GCR_PCLK_DIS0_TPU_POS) /**< PCLK_DIS0_TPU_DIS Setting */

#define MXC_F_GCR_PCLK_DIS0_TIMER0_POS                 15 /**< PCLK_DIS0_TIMER0 Position */
#define MXC_F_GCR_PCLK_DIS0_TIMER0                     ((uint32_t)(0x1UL << MXC_F_GCR_PCLK_DIS0_TIMER0_POS)) /**< PCLK_DIS0_TIMER0 Mask */
#define MXC_V_GCR_PCLK_DIS0_TIMER0_EN                  ((uint32_t)0x0UL) /**< PCLK_DIS0_TIMER0_EN Value */
#define MXC_S_GCR_PCLK_DIS0_TIMER0_EN                  (MXC_V_GCR_PCLK_DIS0_TIMER0_EN << MXC_F_GCR_PCLK_DIS0_TIMER0_POS) /**< PCLK_DIS0_TIMER0_EN Setting */
#define MXC_V_GCR_PCLK_DIS0_TIMER0_DIS                 ((uint32_t)0x1UL) /**< PCLK_DIS0_TIMER0_DIS Value */
#define MXC_S_GCR_PCLK_DIS0_TIMER0_DIS                 (MXC_V_GCR_PCLK_DIS0_TIMER0_DIS << MXC_F_GCR_PCLK_DIS0_TIMER0_POS) /**< PCLK_DIS0_TIMER0_DIS Setting */

#define MXC_F_GCR_PCLK_DIS0_TIMER1_POS                 16 /**< PCLK_DIS0_TIMER1 Position */
#define MXC_F_GCR_PCLK_DIS0_TIMER1                     ((uint32_t)(0x1UL << MXC_F_GCR_PCLK_DIS0_TIMER1_POS)) /**< PCLK_DIS0_TIMER1 Mask */
#define MXC_V_GCR_PCLK_DIS0_TIMER1_EN                  ((uint32_t)0x0UL) /**< PCLK_DIS0_TIMER1_EN Value */
#define MXC_S_GCR_PCLK_DIS0_TIMER1_EN                  (MXC_V_GCR_PCLK_DIS0_TIMER1_EN << MXC_F_GCR_PCLK_DIS0_TIMER1_POS) /**< PCLK_DIS0_TIMER1_EN Setting */
#define MXC_V_GCR_PCLK_DIS0_TIMER1_DIS                 ((uint32_t)0x1UL) /**< PCLK_DIS0_TIMER1_DIS Value */
#define MXC_S_GCR_PCLK_DIS0_TIMER1_DIS                 (MXC_V_GCR_PCLK_DIS0_TIMER1_DIS << MXC_F_GCR_PCLK_DIS0_TIMER1_POS) /**< PCLK_DIS0_TIMER1_DIS Setting */

#define MXC_F_GCR_PCLK_DIS0_TIMER2_POS                 17 /**< PCLK_DIS0_TIMER2 Position */
#define MXC_F_GCR_PCLK_DIS0_TIMER2                     ((uint32_t)(0x1UL << MXC_F_GCR_PCLK_DIS0_TIMER2_POS)) /**< PCLK_DIS0_TIMER2 Mask */
#define MXC_V_GCR_PCLK_DIS0_TIMER2_EN                  ((uint32_t)0x0UL) /**< PCLK_DIS0_TIMER2_EN Value */
#define MXC_S_GCR_PCLK_DIS0_TIMER2_EN                  (MXC_V_GCR_PCLK_DIS0_TIMER2_EN << MXC_F_GCR_PCLK_DIS0_TIMER2_POS) /**< PCLK_DIS0_TIMER2_EN Setting */
#define MXC_V_GCR_PCLK_DIS0_TIMER2_DIS                 ((uint32_t)0x1UL) /**< PCLK_DIS0_TIMER2_DIS Value */
#define MXC_S_GCR_PCLK_DIS0_TIMER2_DIS                 (MXC_V_GCR_PCLK_DIS0_TIMER2_DIS << MXC_F_GCR_PCLK_DIS0_TIMER2_POS) /**< PCLK_DIS0_TIMER2_DIS Setting */

#define MXC_F_GCR_PCLK_DIS0_TIMER3_POS                 18 /**< PCLK_DIS0_TIMER3 Position */
#define MXC_F_GCR_PCLK_DIS0_TIMER3                     ((uint32_t)(0x1UL << MXC_F_GCR_PCLK_DIS0_TIMER3_POS)) /**< PCLK_DIS0_TIMER3 Mask */
#define MXC_V_GCR_PCLK_DIS0_TIMER3_EN                  ((uint32_t)0x0UL) /**< PCLK_DIS0_TIMER3_EN Value */
#define MXC_S_GCR_PCLK_DIS0_TIMER3_EN                  (MXC_V_GCR_PCLK_DIS0_TIMER3_EN << MXC_F_GCR_PCLK_DIS0_TIMER3_POS) /**< PCLK_DIS0_TIMER3_EN Setting */
#define MXC_V_GCR_PCLK_DIS0_TIMER3_DIS                 ((uint32_t)0x1UL) /**< PCLK_DIS0_TIMER3_DIS Value */
#define MXC_S_GCR_PCLK_DIS0_TIMER3_DIS                 (MXC_V_GCR_PCLK_DIS0_TIMER3_DIS << MXC_F_GCR_PCLK_DIS0_TIMER3_POS) /**< PCLK_DIS0_TIMER3_DIS Setting */

#define MXC_F_GCR_PCLK_DIS0_TIMER4_POS                 19 /**< PCLK_DIS0_TIMER4 Position */
#define MXC_F_GCR_PCLK_DIS0_TIMER4                     ((uint32_t)(0x1UL << MXC_F_GCR_PCLK_DIS0_TIMER4_POS)) /**< PCLK_DIS0_TIMER4 Mask */
#define MXC_V_GCR_PCLK_DIS0_TIMER4_EN                  ((uint32_t)0x0UL) /**< PCLK_DIS0_TIMER4_EN Value */
#define MXC_S_GCR_PCLK_DIS0_TIMER4_EN                  (MXC_V_GCR_PCLK_DIS0_TIMER4_EN << MXC_F_GCR_PCLK_DIS0_TIMER4_POS) /**< PCLK_DIS0_TIMER4_EN Setting */
#define MXC_V_GCR_PCLK_DIS0_TIMER4_DIS                 ((uint32_t)0x1UL) /**< PCLK_DIS0_TIMER4_DIS Value */
#define MXC_S_GCR_PCLK_DIS0_TIMER4_DIS                 (MXC_V_GCR_PCLK_DIS0_TIMER4_DIS << MXC_F_GCR_PCLK_DIS0_TIMER4_POS) /**< PCLK_DIS0_TIMER4_DIS Setting */

#define MXC_F_GCR_PCLK_DIS0_TIMER5_POS                 20 /**< PCLK_DIS0_TIMER5 Position */
#define MXC_F_GCR_PCLK_DIS0_TIMER5                     ((uint32_t)(0x1UL << MXC_F_GCR_PCLK_DIS0_TIMER5_POS)) /**< PCLK_DIS0_TIMER5 Mask */
#define MXC_V_GCR_PCLK_DIS0_TIMER5_EN                  ((uint32_t)0x0UL) /**< PCLK_DIS0_TIMER5_EN Value */
#define MXC_S_GCR_PCLK_DIS0_TIMER5_EN                  (MXC_V_GCR_PCLK_DIS0_TIMER5_EN << MXC_F_GCR_PCLK_DIS0_TIMER5_POS) /**< PCLK_DIS0_TIMER5_EN Setting */
#define MXC_V_GCR_PCLK_DIS0_TIMER5_DIS                 ((uint32_t)0x1UL) /**< PCLK_DIS0_TIMER5_DIS Value */
#define MXC_S_GCR_PCLK_DIS0_TIMER5_DIS                 (MXC_V_GCR_PCLK_DIS0_TIMER5_DIS << MXC_F_GCR_PCLK_DIS0_TIMER5_POS) /**< PCLK_DIS0_TIMER5_DIS Setting */

#define MXC_F_GCR_PCLK_DIS0_ADC_POS                    23 /**< PCLK_DIS0_ADC Position */
#define MXC_F_GCR_PCLK_DIS0_ADC                        ((uint32_t)(0x1UL << MXC_F_GCR_PCLK_DIS0_ADC_POS)) /**< PCLK_DIS0_ADC Mask */
#define MXC_V_GCR_PCLK_DIS0_ADC_EN                     ((uint32_t)0x0UL) /**< PCLK_DIS0_ADC_EN Value */
#define MXC_S_GCR_PCLK_DIS0_ADC_EN                     (MXC_V_GCR_PCLK_DIS0_ADC_EN << MXC_F_GCR_PCLK_DIS0_ADC_POS) /**< PCLK_DIS0_ADC_EN Setting */
#define MXC_V_GCR_PCLK_DIS0_ADC_DIS                    ((uint32_t)0x1UL) /**< PCLK_DIS0_ADC_DIS Value */
#define MXC_S_GCR_PCLK_DIS0_ADC_DIS                    (MXC_V_GCR_PCLK_DIS0_ADC_DIS << MXC_F_GCR_PCLK_DIS0_ADC_POS) /**< PCLK_DIS0_ADC_DIS Setting */

#define MXC_F_GCR_PCLK_DIS0_I2C1_POS                   28 /**< PCLK_DIS0_I2C1 Position */
#define MXC_F_GCR_PCLK_DIS0_I2C1                       ((uint32_t)(0x1UL << MXC_F_GCR_PCLK_DIS0_I2C1_POS)) /**< PCLK_DIS0_I2C1 Mask */
#define MXC_V_GCR_PCLK_DIS0_I2C1_EN                    ((uint32_t)0x0UL) /**< PCLK_DIS0_I2C1_EN Value */
#define MXC_S_GCR_PCLK_DIS0_I2C1_EN                    (MXC_V_GCR_PCLK_DIS0_I2C1_EN << MXC_F_GCR_PCLK_DIS0_I2C1_POS) /**< PCLK_DIS0_I2C1_EN Setting */
#define MXC_V_GCR_PCLK_DIS0_I2C1_DIS                   ((uint32_t)0x1UL) /**< PCLK_DIS0_I2C1_DIS Value */
#define MXC_S_GCR_PCLK_DIS0_I2C1_DIS                   (MXC_V_GCR_PCLK_DIS0_I2C1_DIS << MXC_F_GCR_PCLK_DIS0_I2C1_POS) /**< PCLK_DIS0_I2C1_DIS Setting */

#define MXC_F_GCR_PCLK_DIS0_PT_POS                     29 /**< PCLK_DIS0_PT Position */
#define MXC_F_GCR_PCLK_DIS0_PT                         ((uint32_t)(0x1UL << MXC_F_GCR_PCLK_DIS0_PT_POS)) /**< PCLK_DIS0_PT Mask */
#define MXC_V_GCR_PCLK_DIS0_PT_EN                      ((uint32_t)0x0UL) /**< PCLK_DIS0_PT_EN Value */
#define MXC_S_GCR_PCLK_DIS0_PT_EN                      (MXC_V_GCR_PCLK_DIS0_PT_EN << MXC_F_GCR_PCLK_DIS0_PT_POS) /**< PCLK_DIS0_PT_EN Setting */
#define MXC_V_GCR_PCLK_DIS0_PT_DIS                     ((uint32_t)0x1UL) /**< PCLK_DIS0_PT_DIS Value */
#define MXC_S_GCR_PCLK_DIS0_PT_DIS                     (MXC_V_GCR_PCLK_DIS0_PT_DIS << MXC_F_GCR_PCLK_DIS0_PT_POS) /**< PCLK_DIS0_PT_DIS Setting */

#define MXC_F_GCR_PCLK_DIS0_SPIXIPF_POS                30 /**< PCLK_DIS0_SPIXIPF Position */
#define MXC_F_GCR_PCLK_DIS0_SPIXIPF                    ((uint32_t)(0x1UL << MXC_F_GCR_PCLK_DIS0_SPIXIPF_POS)) /**< PCLK_DIS0_SPIXIPF Mask */
#define MXC_V_GCR_PCLK_DIS0_SPIXIPF_EN                 ((uint32_t)0x0UL) /**< PCLK_DIS0_SPIXIPF_EN Value */
#define MXC_S_GCR_PCLK_DIS0_SPIXIPF_EN                 (MXC_V_GCR_PCLK_DIS0_SPIXIPF_EN << MXC_F_GCR_PCLK_DIS0_SPIXIPF_POS) /**< PCLK_DIS0_SPIXIPF_EN Setting */
#define MXC_V_GCR_PCLK_DIS0_SPIXIPF_DIS                ((uint32_t)0x1UL) /**< PCLK_DIS0_SPIXIPF_DIS Value */
#define MXC_S_GCR_PCLK_DIS0_SPIXIPF_DIS                (MXC_V_GCR_PCLK_DIS0_SPIXIPF_DIS << MXC_F_GCR_PCLK_DIS0_SPIXIPF_POS) /**< PCLK_DIS0_SPIXIPF_DIS Setting */

#define MXC_F_GCR_PCLK_DIS0_SPIXIPM_POS                31 /**< PCLK_DIS0_SPIXIPM Position */
#define MXC_F_GCR_PCLK_DIS0_SPIXIPM                    ((uint32_t)(0x1UL << MXC_F_GCR_PCLK_DIS0_SPIXIPM_POS)) /**< PCLK_DIS0_SPIXIPM Mask */
#define MXC_V_GCR_PCLK_DIS0_SPIXIPM_EN                 ((uint32_t)0x0UL) /**< PCLK_DIS0_SPIXIPM_EN Value */
#define MXC_S_GCR_PCLK_DIS0_SPIXIPM_EN                 (MXC_V_GCR_PCLK_DIS0_SPIXIPM_EN << MXC_F_GCR_PCLK_DIS0_SPIXIPM_POS) /**< PCLK_DIS0_SPIXIPM_EN Setting */
#define MXC_V_GCR_PCLK_DIS0_SPIXIPM_DIS                ((uint32_t)0x1UL) /**< PCLK_DIS0_SPIXIPM_DIS Value */
#define MXC_S_GCR_PCLK_DIS0_SPIXIPM_DIS                (MXC_V_GCR_PCLK_DIS0_SPIXIPM_DIS << MXC_F_GCR_PCLK_DIS0_SPIXIPM_POS) /**< PCLK_DIS0_SPIXIPM_DIS Setting */

/**@} end of group GCR_PCLK_DIS0_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_MEM_CLK GCR_MEM_CLK
 * @brief    Memory Clock Control Register.
 * @{
 */
#define MXC_F_GCR_MEM_CLK_FWS_POS                      0 /**< MEM_CLK_FWS Position */
#define MXC_F_GCR_MEM_CLK_FWS                          ((uint32_t)(0x7UL << MXC_F_GCR_MEM_CLK_FWS_POS)) /**< MEM_CLK_FWS Mask */

#define MXC_F_GCR_MEM_CLK_SYSRAM0LS_POS                16 /**< MEM_CLK_SYSRAM0LS Position */
#define MXC_F_GCR_MEM_CLK_SYSRAM0LS                    ((uint32_t)(0x1UL << MXC_F_GCR_MEM_CLK_SYSRAM0LS_POS)) /**< MEM_CLK_SYSRAM0LS Mask */
#define MXC_V_GCR_MEM_CLK_SYSRAM0LS_ACTIVE             ((uint32_t)0x0UL) /**< MEM_CLK_SYSRAM0LS_ACTIVE Value */
#define MXC_S_GCR_MEM_CLK_SYSRAM0LS_ACTIVE             (MXC_V_GCR_MEM_CLK_SYSRAM0LS_ACTIVE << MXC_F_GCR_MEM_CLK_SYSRAM0LS_POS) /**< MEM_CLK_SYSRAM0LS_ACTIVE Setting */
#define MXC_V_GCR_MEM_CLK_SYSRAM0LS_LIGHT_SLEEP        ((uint32_t)0x1UL) /**< MEM_CLK_SYSRAM0LS_LIGHT_SLEEP Value */
#define MXC_S_GCR_MEM_CLK_SYSRAM0LS_LIGHT_SLEEP        (MXC_V_GCR_MEM_CLK_SYSRAM0LS_LIGHT_SLEEP << MXC_F_GCR_MEM_CLK_SYSRAM0LS_POS) /**< MEM_CLK_SYSRAM0LS_LIGHT_SLEEP Setting */

#define MXC_F_GCR_MEM_CLK_SYSRAM1LS_POS                17 /**< MEM_CLK_SYSRAM1LS Position */
#define MXC_F_GCR_MEM_CLK_SYSRAM1LS                    ((uint32_t)(0x1UL << MXC_F_GCR_MEM_CLK_SYSRAM1LS_POS)) /**< MEM_CLK_SYSRAM1LS Mask */
#define MXC_V_GCR_MEM_CLK_SYSRAM1LS_ACTIVE             ((uint32_t)0x0UL) /**< MEM_CLK_SYSRAM1LS_ACTIVE Value */
#define MXC_S_GCR_MEM_CLK_SYSRAM1LS_ACTIVE             (MXC_V_GCR_MEM_CLK_SYSRAM1LS_ACTIVE << MXC_F_GCR_MEM_CLK_SYSRAM1LS_POS) /**< MEM_CLK_SYSRAM1LS_ACTIVE Setting */
#define MXC_V_GCR_MEM_CLK_SYSRAM1LS_LIGHT_SLEEP        ((uint32_t)0x1UL) /**< MEM_CLK_SYSRAM1LS_LIGHT_SLEEP Value */
#define MXC_S_GCR_MEM_CLK_SYSRAM1LS_LIGHT_SLEEP        (MXC_V_GCR_MEM_CLK_SYSRAM1LS_LIGHT_SLEEP << MXC_F_GCR_MEM_CLK_SYSRAM1LS_POS) /**< MEM_CLK_SYSRAM1LS_LIGHT_SLEEP Setting */

#define MXC_F_GCR_MEM_CLK_SYSRAM2LS_POS                18 /**< MEM_CLK_SYSRAM2LS Position */
#define MXC_F_GCR_MEM_CLK_SYSRAM2LS                    ((uint32_t)(0x1UL << MXC_F_GCR_MEM_CLK_SYSRAM2LS_POS)) /**< MEM_CLK_SYSRAM2LS Mask */
#define MXC_V_GCR_MEM_CLK_SYSRAM2LS_ACTIVE             ((uint32_t)0x0UL) /**< MEM_CLK_SYSRAM2LS_ACTIVE Value */
#define MXC_S_GCR_MEM_CLK_SYSRAM2LS_ACTIVE             (MXC_V_GCR_MEM_CLK_SYSRAM2LS_ACTIVE << MXC_F_GCR_MEM_CLK_SYSRAM2LS_POS) /**< MEM_CLK_SYSRAM2LS_ACTIVE Setting */
#define MXC_V_GCR_MEM_CLK_SYSRAM2LS_LIGHT_SLEEP        ((uint32_t)0x1UL) /**< MEM_CLK_SYSRAM2LS_LIGHT_SLEEP Value */
#define MXC_S_GCR_MEM_CLK_SYSRAM2LS_LIGHT_SLEEP        (MXC_V_GCR_MEM_CLK_SYSRAM2LS_LIGHT_SLEEP << MXC_F_GCR_MEM_CLK_SYSRAM2LS_POS) /**< MEM_CLK_SYSRAM2LS_LIGHT_SLEEP Setting */

#define MXC_F_GCR_MEM_CLK_SYSRAM3LS_POS                19 /**< MEM_CLK_SYSRAM3LS Position */
#define MXC_F_GCR_MEM_CLK_SYSRAM3LS                    ((uint32_t)(0x1UL << MXC_F_GCR_MEM_CLK_SYSRAM3LS_POS)) /**< MEM_CLK_SYSRAM3LS Mask */
#define MXC_V_GCR_MEM_CLK_SYSRAM3LS_ACTIVE             ((uint32_t)0x0UL) /**< MEM_CLK_SYSRAM3LS_ACTIVE Value */
#define MXC_S_GCR_MEM_CLK_SYSRAM3LS_ACTIVE             (MXC_V_GCR_MEM_CLK_SYSRAM3LS_ACTIVE << MXC_F_GCR_MEM_CLK_SYSRAM3LS_POS) /**< MEM_CLK_SYSRAM3LS_ACTIVE Setting */
#define MXC_V_GCR_MEM_CLK_SYSRAM3LS_LIGHT_SLEEP        ((uint32_t)0x1UL) /**< MEM_CLK_SYSRAM3LS_LIGHT_SLEEP Value */
#define MXC_S_GCR_MEM_CLK_SYSRAM3LS_LIGHT_SLEEP        (MXC_V_GCR_MEM_CLK_SYSRAM3LS_LIGHT_SLEEP << MXC_F_GCR_MEM_CLK_SYSRAM3LS_POS) /**< MEM_CLK_SYSRAM3LS_LIGHT_SLEEP Setting */

#define MXC_F_GCR_MEM_CLK_SYSRAM4LS_POS                20 /**< MEM_CLK_SYSRAM4LS Position */
#define MXC_F_GCR_MEM_CLK_SYSRAM4LS                    ((uint32_t)(0x1UL << MXC_F_GCR_MEM_CLK_SYSRAM4LS_POS)) /**< MEM_CLK_SYSRAM4LS Mask */
#define MXC_V_GCR_MEM_CLK_SYSRAM4LS_ACTIVE             ((uint32_t)0x0UL) /**< MEM_CLK_SYSRAM4LS_ACTIVE Value */
#define MXC_S_GCR_MEM_CLK_SYSRAM4LS_ACTIVE             (MXC_V_GCR_MEM_CLK_SYSRAM4LS_ACTIVE << MXC_F_GCR_MEM_CLK_SYSRAM4LS_POS) /**< MEM_CLK_SYSRAM4LS_ACTIVE Setting */
#define MXC_V_GCR_MEM_CLK_SYSRAM4LS_LIGHT_SLEEP        ((uint32_t)0x1UL) /**< MEM_CLK_SYSRAM4LS_LIGHT_SLEEP Value */
#define MXC_S_GCR_MEM_CLK_SYSRAM4LS_LIGHT_SLEEP        (MXC_V_GCR_MEM_CLK_SYSRAM4LS_LIGHT_SLEEP << MXC_F_GCR_MEM_CLK_SYSRAM4LS_POS) /**< MEM_CLK_SYSRAM4LS_LIGHT_SLEEP Setting */

#define MXC_F_GCR_MEM_CLK_SYSRAM5LS_POS                21 /**< MEM_CLK_SYSRAM5LS Position */
#define MXC_F_GCR_MEM_CLK_SYSRAM5LS                    ((uint32_t)(0x1UL << MXC_F_GCR_MEM_CLK_SYSRAM5LS_POS)) /**< MEM_CLK_SYSRAM5LS Mask */
#define MXC_V_GCR_MEM_CLK_SYSRAM5LS_ACTIVE             ((uint32_t)0x0UL) /**< MEM_CLK_SYSRAM5LS_ACTIVE Value */
#define MXC_S_GCR_MEM_CLK_SYSRAM5LS_ACTIVE             (MXC_V_GCR_MEM_CLK_SYSRAM5LS_ACTIVE << MXC_F_GCR_MEM_CLK_SYSRAM5LS_POS) /**< MEM_CLK_SYSRAM5LS_ACTIVE Setting */
#define MXC_V_GCR_MEM_CLK_SYSRAM5LS_LIGHT_SLEEP        ((uint32_t)0x1UL) /**< MEM_CLK_SYSRAM5LS_LIGHT_SLEEP Value */
#define MXC_S_GCR_MEM_CLK_SYSRAM5LS_LIGHT_SLEEP        (MXC_V_GCR_MEM_CLK_SYSRAM5LS_LIGHT_SLEEP << MXC_F_GCR_MEM_CLK_SYSRAM5LS_POS) /**< MEM_CLK_SYSRAM5LS_LIGHT_SLEEP Setting */

#define MXC_F_GCR_MEM_CLK_SYSRAM6LS_POS                22 /**< MEM_CLK_SYSRAM6LS Position */
#define MXC_F_GCR_MEM_CLK_SYSRAM6LS                    ((uint32_t)(0x1UL << MXC_F_GCR_MEM_CLK_SYSRAM6LS_POS)) /**< MEM_CLK_SYSRAM6LS Mask */
#define MXC_V_GCR_MEM_CLK_SYSRAM6LS_ACTIVE             ((uint32_t)0x0UL) /**< MEM_CLK_SYSRAM6LS_ACTIVE Value */
#define MXC_S_GCR_MEM_CLK_SYSRAM6LS_ACTIVE             (MXC_V_GCR_MEM_CLK_SYSRAM6LS_ACTIVE << MXC_F_GCR_MEM_CLK_SYSRAM6LS_POS) /**< MEM_CLK_SYSRAM6LS_ACTIVE Setting */
#define MXC_V_GCR_MEM_CLK_SYSRAM6LS_LIGHT_SLEEP        ((uint32_t)0x1UL) /**< MEM_CLK_SYSRAM6LS_LIGHT_SLEEP Value */
#define MXC_S_GCR_MEM_CLK_SYSRAM6LS_LIGHT_SLEEP        (MXC_V_GCR_MEM_CLK_SYSRAM6LS_LIGHT_SLEEP << MXC_F_GCR_MEM_CLK_SYSRAM6LS_POS) /**< MEM_CLK_SYSRAM6LS_LIGHT_SLEEP Setting */

#define MXC_F_GCR_MEM_CLK_ICACHELS_POS                 24 /**< MEM_CLK_ICACHELS Position */
#define MXC_F_GCR_MEM_CLK_ICACHELS                     ((uint32_t)(0x1UL << MXC_F_GCR_MEM_CLK_ICACHELS_POS)) /**< MEM_CLK_ICACHELS Mask */
#define MXC_V_GCR_MEM_CLK_ICACHELS_ACTIVE              ((uint32_t)0x0UL) /**< MEM_CLK_ICACHELS_ACTIVE Value */
#define MXC_S_GCR_MEM_CLK_ICACHELS_ACTIVE              (MXC_V_GCR_MEM_CLK_ICACHELS_ACTIVE << MXC_F_GCR_MEM_CLK_ICACHELS_POS) /**< MEM_CLK_ICACHELS_ACTIVE Setting */
#define MXC_V_GCR_MEM_CLK_ICACHELS_LIGHT_SLEEP         ((uint32_t)0x1UL) /**< MEM_CLK_ICACHELS_LIGHT_SLEEP Value */
#define MXC_S_GCR_MEM_CLK_ICACHELS_LIGHT_SLEEP         (MXC_V_GCR_MEM_CLK_ICACHELS_LIGHT_SLEEP << MXC_F_GCR_MEM_CLK_ICACHELS_POS) /**< MEM_CLK_ICACHELS_LIGHT_SLEEP Setting */

#define MXC_F_GCR_MEM_CLK_ICACHEXIPLS_POS              25 /**< MEM_CLK_ICACHEXIPLS Position */
#define MXC_F_GCR_MEM_CLK_ICACHEXIPLS                  ((uint32_t)(0x1UL << MXC_F_GCR_MEM_CLK_ICACHEXIPLS_POS)) /**< MEM_CLK_ICACHEXIPLS Mask */
#define MXC_V_GCR_MEM_CLK_ICACHEXIPLS_ACTIVE           ((uint32_t)0x0UL) /**< MEM_CLK_ICACHEXIPLS_ACTIVE Value */
#define MXC_S_GCR_MEM_CLK_ICACHEXIPLS_ACTIVE           (MXC_V_GCR_MEM_CLK_ICACHEXIPLS_ACTIVE << MXC_F_GCR_MEM_CLK_ICACHEXIPLS_POS) /**< MEM_CLK_ICACHEXIPLS_ACTIVE Setting */
#define MXC_V_GCR_MEM_CLK_ICACHEXIPLS_LIGHT_SLEEP      ((uint32_t)0x1UL) /**< MEM_CLK_ICACHEXIPLS_LIGHT_SLEEP Value */
#define MXC_S_GCR_MEM_CLK_ICACHEXIPLS_LIGHT_SLEEP      (MXC_V_GCR_MEM_CLK_ICACHEXIPLS_LIGHT_SLEEP << MXC_F_GCR_MEM_CLK_ICACHEXIPLS_POS) /**< MEM_CLK_ICACHEXIPLS_LIGHT_SLEEP Setting */

#define MXC_F_GCR_MEM_CLK_SCACHELS_POS                 26 /**< MEM_CLK_SCACHELS Position */
#define MXC_F_GCR_MEM_CLK_SCACHELS                     ((uint32_t)(0x1UL << MXC_F_GCR_MEM_CLK_SCACHELS_POS)) /**< MEM_CLK_SCACHELS Mask */
#define MXC_V_GCR_MEM_CLK_SCACHELS_ACTIVE              ((uint32_t)0x0UL) /**< MEM_CLK_SCACHELS_ACTIVE Value */
#define MXC_S_GCR_MEM_CLK_SCACHELS_ACTIVE              (MXC_V_GCR_MEM_CLK_SCACHELS_ACTIVE << MXC_F_GCR_MEM_CLK_SCACHELS_POS) /**< MEM_CLK_SCACHELS_ACTIVE Setting */
#define MXC_V_GCR_MEM_CLK_SCACHELS_LIGHT_SLEEP         ((uint32_t)0x1UL) /**< MEM_CLK_SCACHELS_LIGHT_SLEEP Value */
#define MXC_S_GCR_MEM_CLK_SCACHELS_LIGHT_SLEEP         (MXC_V_GCR_MEM_CLK_SCACHELS_LIGHT_SLEEP << MXC_F_GCR_MEM_CLK_SCACHELS_POS) /**< MEM_CLK_SCACHELS_LIGHT_SLEEP Setting */

#define MXC_F_GCR_MEM_CLK_CRYPTOLS_POS                 27 /**< MEM_CLK_CRYPTOLS Position */
#define MXC_F_GCR_MEM_CLK_CRYPTOLS                     ((uint32_t)(0x1UL << MXC_F_GCR_MEM_CLK_CRYPTOLS_POS)) /**< MEM_CLK_CRYPTOLS Mask */
#define MXC_V_GCR_MEM_CLK_CRYPTOLS_ACTIVE              ((uint32_t)0x0UL) /**< MEM_CLK_CRYPTOLS_ACTIVE Value */
#define MXC_S_GCR_MEM_CLK_CRYPTOLS_ACTIVE              (MXC_V_GCR_MEM_CLK_CRYPTOLS_ACTIVE << MXC_F_GCR_MEM_CLK_CRYPTOLS_POS) /**< MEM_CLK_CRYPTOLS_ACTIVE Setting */
#define MXC_V_GCR_MEM_CLK_CRYPTOLS_LIGHT_SLEEP         ((uint32_t)0x1UL) /**< MEM_CLK_CRYPTOLS_LIGHT_SLEEP Value */
#define MXC_S_GCR_MEM_CLK_CRYPTOLS_LIGHT_SLEEP         (MXC_V_GCR_MEM_CLK_CRYPTOLS_LIGHT_SLEEP << MXC_F_GCR_MEM_CLK_CRYPTOLS_POS) /**< MEM_CLK_CRYPTOLS_LIGHT_SLEEP Setting */

#define MXC_F_GCR_MEM_CLK_USBLS_POS                    28 /**< MEM_CLK_USBLS Position */
#define MXC_F_GCR_MEM_CLK_USBLS                        ((uint32_t)(0x1UL << MXC_F_GCR_MEM_CLK_USBLS_POS)) /**< MEM_CLK_USBLS Mask */
#define MXC_V_GCR_MEM_CLK_USBLS_ACTIVE                 ((uint32_t)0x0UL) /**< MEM_CLK_USBLS_ACTIVE Value */
#define MXC_S_GCR_MEM_CLK_USBLS_ACTIVE                 (MXC_V_GCR_MEM_CLK_USBLS_ACTIVE << MXC_F_GCR_MEM_CLK_USBLS_POS) /**< MEM_CLK_USBLS_ACTIVE Setting */
#define MXC_V_GCR_MEM_CLK_USBLS_LIGHT_SLEEP            ((uint32_t)0x1UL) /**< MEM_CLK_USBLS_LIGHT_SLEEP Value */
#define MXC_S_GCR_MEM_CLK_USBLS_LIGHT_SLEEP            (MXC_V_GCR_MEM_CLK_USBLS_LIGHT_SLEEP << MXC_F_GCR_MEM_CLK_USBLS_POS) /**< MEM_CLK_USBLS_LIGHT_SLEEP Setting */

#define MXC_F_GCR_MEM_CLK_ROMLS_POS                    29 /**< MEM_CLK_ROMLS Position */
#define MXC_F_GCR_MEM_CLK_ROMLS                        ((uint32_t)(0x1UL << MXC_F_GCR_MEM_CLK_ROMLS_POS)) /**< MEM_CLK_ROMLS Mask */
#define MXC_V_GCR_MEM_CLK_ROMLS_ACTIVE                 ((uint32_t)0x0UL) /**< MEM_CLK_ROMLS_ACTIVE Value */
#define MXC_S_GCR_MEM_CLK_ROMLS_ACTIVE                 (MXC_V_GCR_MEM_CLK_ROMLS_ACTIVE << MXC_F_GCR_MEM_CLK_ROMLS_POS) /**< MEM_CLK_ROMLS_ACTIVE Setting */
#define MXC_V_GCR_MEM_CLK_ROMLS_LIGHT_SLEEP            ((uint32_t)0x1UL) /**< MEM_CLK_ROMLS_LIGHT_SLEEP Value */
#define MXC_S_GCR_MEM_CLK_ROMLS_LIGHT_SLEEP            (MXC_V_GCR_MEM_CLK_ROMLS_LIGHT_SLEEP << MXC_F_GCR_MEM_CLK_ROMLS_POS) /**< MEM_CLK_ROMLS_LIGHT_SLEEP Setting */

/**@} end of group GCR_MEM_CLK_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_MEM_ZERO GCR_MEM_ZERO
 * @brief    Memory Zeroize Control.
 * @{
 */
#define MXC_F_GCR_MEM_ZERO_SRAM0Z_POS                  0 /**< MEM_ZERO_SRAM0Z Position */
#define MXC_F_GCR_MEM_ZERO_SRAM0Z                      ((uint32_t)(0x1UL << MXC_F_GCR_MEM_ZERO_SRAM0Z_POS)) /**< MEM_ZERO_SRAM0Z Mask */
#define MXC_V_GCR_MEM_ZERO_SRAM0Z_NOP                  ((uint32_t)0x0UL) /**< MEM_ZERO_SRAM0Z_NOP Value */
#define MXC_S_GCR_MEM_ZERO_SRAM0Z_NOP                  (MXC_V_GCR_MEM_ZERO_SRAM0Z_NOP << MXC_F_GCR_MEM_ZERO_SRAM0Z_POS) /**< MEM_ZERO_SRAM0Z_NOP Setting */
#define MXC_V_GCR_MEM_ZERO_SRAM0Z_START                ((uint32_t)0x1UL) /**< MEM_ZERO_SRAM0Z_START Value */
#define MXC_S_GCR_MEM_ZERO_SRAM0Z_START                (MXC_V_GCR_MEM_ZERO_SRAM0Z_START << MXC_F_GCR_MEM_ZERO_SRAM0Z_POS) /**< MEM_ZERO_SRAM0Z_START Setting */

#define MXC_F_GCR_MEM_ZERO_SRAM1Z_POS                  1 /**< MEM_ZERO_SRAM1Z Position */
#define MXC_F_GCR_MEM_ZERO_SRAM1Z                      ((uint32_t)(0x1UL << MXC_F_GCR_MEM_ZERO_SRAM1Z_POS)) /**< MEM_ZERO_SRAM1Z Mask */
#define MXC_V_GCR_MEM_ZERO_SRAM1Z_NOP                  ((uint32_t)0x0UL) /**< MEM_ZERO_SRAM1Z_NOP Value */
#define MXC_S_GCR_MEM_ZERO_SRAM1Z_NOP                  (MXC_V_GCR_MEM_ZERO_SRAM1Z_NOP << MXC_F_GCR_MEM_ZERO_SRAM1Z_POS) /**< MEM_ZERO_SRAM1Z_NOP Setting */
#define MXC_V_GCR_MEM_ZERO_SRAM1Z_START                ((uint32_t)0x1UL) /**< MEM_ZERO_SRAM1Z_START Value */
#define MXC_S_GCR_MEM_ZERO_SRAM1Z_START                (MXC_V_GCR_MEM_ZERO_SRAM1Z_START << MXC_F_GCR_MEM_ZERO_SRAM1Z_POS) /**< MEM_ZERO_SRAM1Z_START Setting */

#define MXC_F_GCR_MEM_ZERO_SRAM2Z_POS                  2 /**< MEM_ZERO_SRAM2Z Position */
#define MXC_F_GCR_MEM_ZERO_SRAM2Z                      ((uint32_t)(0x1UL << MXC_F_GCR_MEM_ZERO_SRAM2Z_POS)) /**< MEM_ZERO_SRAM2Z Mask */
#define MXC_V_GCR_MEM_ZERO_SRAM2Z_NOP                  ((uint32_t)0x0UL) /**< MEM_ZERO_SRAM2Z_NOP Value */
#define MXC_S_GCR_MEM_ZERO_SRAM2Z_NOP                  (MXC_V_GCR_MEM_ZERO_SRAM2Z_NOP << MXC_F_GCR_MEM_ZERO_SRAM2Z_POS) /**< MEM_ZERO_SRAM2Z_NOP Setting */
#define MXC_V_GCR_MEM_ZERO_SRAM2Z_START                ((uint32_t)0x1UL) /**< MEM_ZERO_SRAM2Z_START Value */
#define MXC_S_GCR_MEM_ZERO_SRAM2Z_START                (MXC_V_GCR_MEM_ZERO_SRAM2Z_START << MXC_F_GCR_MEM_ZERO_SRAM2Z_POS) /**< MEM_ZERO_SRAM2Z_START Setting */

#define MXC_F_GCR_MEM_ZERO_SRAM3Z_POS                  3 /**< MEM_ZERO_SRAM3Z Position */
#define MXC_F_GCR_MEM_ZERO_SRAM3Z                      ((uint32_t)(0x1UL << MXC_F_GCR_MEM_ZERO_SRAM3Z_POS)) /**< MEM_ZERO_SRAM3Z Mask */
#define MXC_V_GCR_MEM_ZERO_SRAM3Z_NOP                  ((uint32_t)0x0UL) /**< MEM_ZERO_SRAM3Z_NOP Value */
#define MXC_S_GCR_MEM_ZERO_SRAM3Z_NOP                  (MXC_V_GCR_MEM_ZERO_SRAM3Z_NOP << MXC_F_GCR_MEM_ZERO_SRAM3Z_POS) /**< MEM_ZERO_SRAM3Z_NOP Setting */
#define MXC_V_GCR_MEM_ZERO_SRAM3Z_START                ((uint32_t)0x1UL) /**< MEM_ZERO_SRAM3Z_START Value */
#define MXC_S_GCR_MEM_ZERO_SRAM3Z_START                (MXC_V_GCR_MEM_ZERO_SRAM3Z_START << MXC_F_GCR_MEM_ZERO_SRAM3Z_POS) /**< MEM_ZERO_SRAM3Z_START Setting */

#define MXC_F_GCR_MEM_ZERO_SRAM4Z_POS                  4 /**< MEM_ZERO_SRAM4Z Position */
#define MXC_F_GCR_MEM_ZERO_SRAM4Z                      ((uint32_t)(0x1UL << MXC_F_GCR_MEM_ZERO_SRAM4Z_POS)) /**< MEM_ZERO_SRAM4Z Mask */
#define MXC_V_GCR_MEM_ZERO_SRAM4Z_NOP                  ((uint32_t)0x0UL) /**< MEM_ZERO_SRAM4Z_NOP Value */
#define MXC_S_GCR_MEM_ZERO_SRAM4Z_NOP                  (MXC_V_GCR_MEM_ZERO_SRAM4Z_NOP << MXC_F_GCR_MEM_ZERO_SRAM4Z_POS) /**< MEM_ZERO_SRAM4Z_NOP Setting */
#define MXC_V_GCR_MEM_ZERO_SRAM4Z_START                ((uint32_t)0x1UL) /**< MEM_ZERO_SRAM4Z_START Value */
#define MXC_S_GCR_MEM_ZERO_SRAM4Z_START                (MXC_V_GCR_MEM_ZERO_SRAM4Z_START << MXC_F_GCR_MEM_ZERO_SRAM4Z_POS) /**< MEM_ZERO_SRAM4Z_START Setting */

#define MXC_F_GCR_MEM_ZERO_SRAM5Z_POS                  5 /**< MEM_ZERO_SRAM5Z Position */
#define MXC_F_GCR_MEM_ZERO_SRAM5Z                      ((uint32_t)(0x1UL << MXC_F_GCR_MEM_ZERO_SRAM5Z_POS)) /**< MEM_ZERO_SRAM5Z Mask */
#define MXC_V_GCR_MEM_ZERO_SRAM5Z_NOP                  ((uint32_t)0x0UL) /**< MEM_ZERO_SRAM5Z_NOP Value */
#define MXC_S_GCR_MEM_ZERO_SRAM5Z_NOP                  (MXC_V_GCR_MEM_ZERO_SRAM5Z_NOP << MXC_F_GCR_MEM_ZERO_SRAM5Z_POS) /**< MEM_ZERO_SRAM5Z_NOP Setting */
#define MXC_V_GCR_MEM_ZERO_SRAM5Z_START                ((uint32_t)0x1UL) /**< MEM_ZERO_SRAM5Z_START Value */
#define MXC_S_GCR_MEM_ZERO_SRAM5Z_START                (MXC_V_GCR_MEM_ZERO_SRAM5Z_START << MXC_F_GCR_MEM_ZERO_SRAM5Z_POS) /**< MEM_ZERO_SRAM5Z_START Setting */

#define MXC_F_GCR_MEM_ZERO_SRAM6Z_POS                  6 /**< MEM_ZERO_SRAM6Z Position */
#define MXC_F_GCR_MEM_ZERO_SRAM6Z                      ((uint32_t)(0x1UL << MXC_F_GCR_MEM_ZERO_SRAM6Z_POS)) /**< MEM_ZERO_SRAM6Z Mask */
#define MXC_V_GCR_MEM_ZERO_SRAM6Z_NOP                  ((uint32_t)0x0UL) /**< MEM_ZERO_SRAM6Z_NOP Value */
#define MXC_S_GCR_MEM_ZERO_SRAM6Z_NOP                  (MXC_V_GCR_MEM_ZERO_SRAM6Z_NOP << MXC_F_GCR_MEM_ZERO_SRAM6Z_POS) /**< MEM_ZERO_SRAM6Z_NOP Setting */
#define MXC_V_GCR_MEM_ZERO_SRAM6Z_START                ((uint32_t)0x1UL) /**< MEM_ZERO_SRAM6Z_START Value */
#define MXC_S_GCR_MEM_ZERO_SRAM6Z_START                (MXC_V_GCR_MEM_ZERO_SRAM6Z_START << MXC_F_GCR_MEM_ZERO_SRAM6Z_POS) /**< MEM_ZERO_SRAM6Z_START Setting */

#define MXC_F_GCR_MEM_ZERO_ICACHEZ_POS                 8 /**< MEM_ZERO_ICACHEZ Position */
#define MXC_F_GCR_MEM_ZERO_ICACHEZ                     ((uint32_t)(0x1UL << MXC_F_GCR_MEM_ZERO_ICACHEZ_POS)) /**< MEM_ZERO_ICACHEZ Mask */
#define MXC_V_GCR_MEM_ZERO_ICACHEZ_NOP                 ((uint32_t)0x0UL) /**< MEM_ZERO_ICACHEZ_NOP Value */
#define MXC_S_GCR_MEM_ZERO_ICACHEZ_NOP                 (MXC_V_GCR_MEM_ZERO_ICACHEZ_NOP << MXC_F_GCR_MEM_ZERO_ICACHEZ_POS) /**< MEM_ZERO_ICACHEZ_NOP Setting */
#define MXC_V_GCR_MEM_ZERO_ICACHEZ_START               ((uint32_t)0x1UL) /**< MEM_ZERO_ICACHEZ_START Value */
#define MXC_S_GCR_MEM_ZERO_ICACHEZ_START               (MXC_V_GCR_MEM_ZERO_ICACHEZ_START << MXC_F_GCR_MEM_ZERO_ICACHEZ_POS) /**< MEM_ZERO_ICACHEZ_START Setting */

#define MXC_F_GCR_MEM_ZERO_ICACHEXIPZ_POS              9 /**< MEM_ZERO_ICACHEXIPZ Position */
#define MXC_F_GCR_MEM_ZERO_ICACHEXIPZ                  ((uint32_t)(0x1UL << MXC_F_GCR_MEM_ZERO_ICACHEXIPZ_POS)) /**< MEM_ZERO_ICACHEXIPZ Mask */
#define MXC_V_GCR_MEM_ZERO_ICACHEXIPZ_NOP              ((uint32_t)0x0UL) /**< MEM_ZERO_ICACHEXIPZ_NOP Value */
#define MXC_S_GCR_MEM_ZERO_ICACHEXIPZ_NOP              (MXC_V_GCR_MEM_ZERO_ICACHEXIPZ_NOP << MXC_F_GCR_MEM_ZERO_ICACHEXIPZ_POS) /**< MEM_ZERO_ICACHEXIPZ_NOP Setting */
#define MXC_V_GCR_MEM_ZERO_ICACHEXIPZ_START            ((uint32_t)0x1UL) /**< MEM_ZERO_ICACHEXIPZ_START Value */
#define MXC_S_GCR_MEM_ZERO_ICACHEXIPZ_START            (MXC_V_GCR_MEM_ZERO_ICACHEXIPZ_START << MXC_F_GCR_MEM_ZERO_ICACHEXIPZ_POS) /**< MEM_ZERO_ICACHEXIPZ_START Setting */

#define MXC_F_GCR_MEM_ZERO_SCACHEDATAZ_POS             10 /**< MEM_ZERO_SCACHEDATAZ Position */
#define MXC_F_GCR_MEM_ZERO_SCACHEDATAZ                 ((uint32_t)(0x1UL << MXC_F_GCR_MEM_ZERO_SCACHEDATAZ_POS)) /**< MEM_ZERO_SCACHEDATAZ Mask */
#define MXC_V_GCR_MEM_ZERO_SCACHEDATAZ_NOP             ((uint32_t)0x0UL) /**< MEM_ZERO_SCACHEDATAZ_NOP Value */
#define MXC_S_GCR_MEM_ZERO_SCACHEDATAZ_NOP             (MXC_V_GCR_MEM_ZERO_SCACHEDATAZ_NOP << MXC_F_GCR_MEM_ZERO_SCACHEDATAZ_POS) /**< MEM_ZERO_SCACHEDATAZ_NOP Setting */
#define MXC_V_GCR_MEM_ZERO_SCACHEDATAZ_START           ((uint32_t)0x1UL) /**< MEM_ZERO_SCACHEDATAZ_START Value */
#define MXC_S_GCR_MEM_ZERO_SCACHEDATAZ_START           (MXC_V_GCR_MEM_ZERO_SCACHEDATAZ_START << MXC_F_GCR_MEM_ZERO_SCACHEDATAZ_POS) /**< MEM_ZERO_SCACHEDATAZ_START Setting */

#define MXC_F_GCR_MEM_ZERO_SCACHETAGZ_POS              11 /**< MEM_ZERO_SCACHETAGZ Position */
#define MXC_F_GCR_MEM_ZERO_SCACHETAGZ                  ((uint32_t)(0x1UL << MXC_F_GCR_MEM_ZERO_SCACHETAGZ_POS)) /**< MEM_ZERO_SCACHETAGZ Mask */
#define MXC_V_GCR_MEM_ZERO_SCACHETAGZ_NOP              ((uint32_t)0x0UL) /**< MEM_ZERO_SCACHETAGZ_NOP Value */
#define MXC_S_GCR_MEM_ZERO_SCACHETAGZ_NOP              (MXC_V_GCR_MEM_ZERO_SCACHETAGZ_NOP << MXC_F_GCR_MEM_ZERO_SCACHETAGZ_POS) /**< MEM_ZERO_SCACHETAGZ_NOP Setting */
#define MXC_V_GCR_MEM_ZERO_SCACHETAGZ_START            ((uint32_t)0x1UL) /**< MEM_ZERO_SCACHETAGZ_START Value */
#define MXC_S_GCR_MEM_ZERO_SCACHETAGZ_START            (MXC_V_GCR_MEM_ZERO_SCACHETAGZ_START << MXC_F_GCR_MEM_ZERO_SCACHETAGZ_POS) /**< MEM_ZERO_SCACHETAGZ_START Setting */

#define MXC_F_GCR_MEM_ZERO_CRYPTOZ_POS                 12 /**< MEM_ZERO_CRYPTOZ Position */
#define MXC_F_GCR_MEM_ZERO_CRYPTOZ                     ((uint32_t)(0x1UL << MXC_F_GCR_MEM_ZERO_CRYPTOZ_POS)) /**< MEM_ZERO_CRYPTOZ Mask */
#define MXC_V_GCR_MEM_ZERO_CRYPTOZ_NOP                 ((uint32_t)0x0UL) /**< MEM_ZERO_CRYPTOZ_NOP Value */
#define MXC_S_GCR_MEM_ZERO_CRYPTOZ_NOP                 (MXC_V_GCR_MEM_ZERO_CRYPTOZ_NOP << MXC_F_GCR_MEM_ZERO_CRYPTOZ_POS) /**< MEM_ZERO_CRYPTOZ_NOP Setting */
#define MXC_V_GCR_MEM_ZERO_CRYPTOZ_START               ((uint32_t)0x1UL) /**< MEM_ZERO_CRYPTOZ_START Value */
#define MXC_S_GCR_MEM_ZERO_CRYPTOZ_START               (MXC_V_GCR_MEM_ZERO_CRYPTOZ_START << MXC_F_GCR_MEM_ZERO_CRYPTOZ_POS) /**< MEM_ZERO_CRYPTOZ_START Setting */

#define MXC_F_GCR_MEM_ZERO_USBFIFOZ_POS                13 /**< MEM_ZERO_USBFIFOZ Position */
#define MXC_F_GCR_MEM_ZERO_USBFIFOZ                    ((uint32_t)(0x1UL << MXC_F_GCR_MEM_ZERO_USBFIFOZ_POS)) /**< MEM_ZERO_USBFIFOZ Mask */
#define MXC_V_GCR_MEM_ZERO_USBFIFOZ_NOP                ((uint32_t)0x0UL) /**< MEM_ZERO_USBFIFOZ_NOP Value */
#define MXC_S_GCR_MEM_ZERO_USBFIFOZ_NOP                (MXC_V_GCR_MEM_ZERO_USBFIFOZ_NOP << MXC_F_GCR_MEM_ZERO_USBFIFOZ_POS) /**< MEM_ZERO_USBFIFOZ_NOP Setting */
#define MXC_V_GCR_MEM_ZERO_USBFIFOZ_START              ((uint32_t)0x1UL) /**< MEM_ZERO_USBFIFOZ_START Value */
#define MXC_S_GCR_MEM_ZERO_USBFIFOZ_START              (MXC_V_GCR_MEM_ZERO_USBFIFOZ_START << MXC_F_GCR_MEM_ZERO_USBFIFOZ_POS) /**< MEM_ZERO_USBFIFOZ_START Setting */

/**@} end of group GCR_MEM_ZERO_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_SYS_STAT GCR_SYS_STAT
 * @brief    System Status Register.
 * @{
 */
#define MXC_F_GCR_SYS_STAT_ICELOCK_POS                 0 /**< SYS_STAT_ICELOCK Position */
#define MXC_F_GCR_SYS_STAT_ICELOCK                     ((uint32_t)(0x1UL << MXC_F_GCR_SYS_STAT_ICELOCK_POS)) /**< SYS_STAT_ICELOCK Mask */
#define MXC_V_GCR_SYS_STAT_ICELOCK_UNLOCKED            ((uint32_t)0x0UL) /**< SYS_STAT_ICELOCK_UNLOCKED Value */
#define MXC_S_GCR_SYS_STAT_ICELOCK_UNLOCKED            (MXC_V_GCR_SYS_STAT_ICELOCK_UNLOCKED << MXC_F_GCR_SYS_STAT_ICELOCK_POS) /**< SYS_STAT_ICELOCK_UNLOCKED Setting */
#define MXC_V_GCR_SYS_STAT_ICELOCK_LOCKED              ((uint32_t)0x1UL) /**< SYS_STAT_ICELOCK_LOCKED Value */
#define MXC_S_GCR_SYS_STAT_ICELOCK_LOCKED              (MXC_V_GCR_SYS_STAT_ICELOCK_LOCKED << MXC_F_GCR_SYS_STAT_ICELOCK_POS) /**< SYS_STAT_ICELOCK_LOCKED Setting */

#define MXC_F_GCR_SYS_STAT_CODEINTERR_POS              1 /**< SYS_STAT_CODEINTERR Position */
#define MXC_F_GCR_SYS_STAT_CODEINTERR                  ((uint32_t)(0x1UL << MXC_F_GCR_SYS_STAT_CODEINTERR_POS)) /**< SYS_STAT_CODEINTERR Mask */
#define MXC_V_GCR_SYS_STAT_CODEINTERR_NOERR            ((uint32_t)0x0UL) /**< SYS_STAT_CODEINTERR_NOERR Value */
#define MXC_S_GCR_SYS_STAT_CODEINTERR_NOERR            (MXC_V_GCR_SYS_STAT_CODEINTERR_NOERR << MXC_F_GCR_SYS_STAT_CODEINTERR_POS) /**< SYS_STAT_CODEINTERR_NOERR Setting */
#define MXC_V_GCR_SYS_STAT_CODEINTERR_ERR              ((uint32_t)0x1UL) /**< SYS_STAT_CODEINTERR_ERR Value */
#define MXC_S_GCR_SYS_STAT_CODEINTERR_ERR              (MXC_V_GCR_SYS_STAT_CODEINTERR_ERR << MXC_F_GCR_SYS_STAT_CODEINTERR_POS) /**< SYS_STAT_CODEINTERR_ERR Setting */

#define MXC_F_GCR_SYS_STAT_SCMEMF_POS                  5 /**< SYS_STAT_SCMEMF Position */
#define MXC_F_GCR_SYS_STAT_SCMEMF                      ((uint32_t)(0x1UL << MXC_F_GCR_SYS_STAT_SCMEMF_POS)) /**< SYS_STAT_SCMEMF Mask */
#define MXC_V_GCR_SYS_STAT_SCMEMF_NOERR                ((uint32_t)0x0UL) /**< SYS_STAT_SCMEMF_NOERR Value */
#define MXC_S_GCR_SYS_STAT_SCMEMF_NOERR                (MXC_V_GCR_SYS_STAT_SCMEMF_NOERR << MXC_F_GCR_SYS_STAT_SCMEMF_POS) /**< SYS_STAT_SCMEMF_NOERR Setting */
#define MXC_V_GCR_SYS_STAT_SCMEMF_MEMFAULT             ((uint32_t)0x1UL) /**< SYS_STAT_SCMEMF_MEMFAULT Value */
#define MXC_S_GCR_SYS_STAT_SCMEMF_MEMFAULT             (MXC_V_GCR_SYS_STAT_SCMEMF_MEMFAULT << MXC_F_GCR_SYS_STAT_SCMEMF_POS) /**< SYS_STAT_SCMEMF_MEMFAULT Setting */

/**@} end of group GCR_SYS_STAT_Register */

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

#define MXC_F_GCR_RST1_I2S_POS                         10 /**< RST1_I2S Position */
#define MXC_F_GCR_RST1_I2S                             ((uint32_t)(0x1UL << MXC_F_GCR_RST1_I2S_POS)) /**< RST1_I2S Mask */

#define MXC_F_GCR_RST1_XIPR_POS                        15 /**< RST1_XIPR Position */
#define MXC_F_GCR_RST1_XIPR                            ((uint32_t)(0x1UL << MXC_F_GCR_RST1_XIPR_POS)) /**< RST1_XIPR Mask */

#define MXC_F_GCR_RST1_SEMA_POS                        16 /**< RST1_SEMA Position */
#define MXC_F_GCR_RST1_SEMA                            ((uint32_t)(0x1UL << MXC_F_GCR_RST1_SEMA_POS)) /**< RST1_SEMA Mask */

/**@} end of group GCR_RST1_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_PCLK_DIS1 GCR_PCLK_DIS1
 * @brief    Peripheral Clock Disable.
 * @{
 */
#define MXC_F_GCR_PCLK_DIS1_UART2_POS                  1 /**< PCLK_DIS1_UART2 Position */
#define MXC_F_GCR_PCLK_DIS1_UART2                      ((uint32_t)(0x1UL << MXC_F_GCR_PCLK_DIS1_UART2_POS)) /**< PCLK_DIS1_UART2 Mask */
#define MXC_V_GCR_PCLK_DIS1_UART2_EN                   ((uint32_t)0x0UL) /**< PCLK_DIS1_UART2_EN Value */
#define MXC_S_GCR_PCLK_DIS1_UART2_EN                   (MXC_V_GCR_PCLK_DIS1_UART2_EN << MXC_F_GCR_PCLK_DIS1_UART2_POS) /**< PCLK_DIS1_UART2_EN Setting */
#define MXC_V_GCR_PCLK_DIS1_UART2_DIS                  ((uint32_t)0x1UL) /**< PCLK_DIS1_UART2_DIS Value */
#define MXC_S_GCR_PCLK_DIS1_UART2_DIS                  (MXC_V_GCR_PCLK_DIS1_UART2_DIS << MXC_F_GCR_PCLK_DIS1_UART2_POS) /**< PCLK_DIS1_UART2_DIS Setting */

#define MXC_F_GCR_PCLK_DIS1_TRNG_POS                   2 /**< PCLK_DIS1_TRNG Position */
#define MXC_F_GCR_PCLK_DIS1_TRNG                       ((uint32_t)(0x1UL << MXC_F_GCR_PCLK_DIS1_TRNG_POS)) /**< PCLK_DIS1_TRNG Mask */
#define MXC_V_GCR_PCLK_DIS1_TRNG_EN                    ((uint32_t)0x0UL) /**< PCLK_DIS1_TRNG_EN Value */
#define MXC_S_GCR_PCLK_DIS1_TRNG_EN                    (MXC_V_GCR_PCLK_DIS1_TRNG_EN << MXC_F_GCR_PCLK_DIS1_TRNG_POS) /**< PCLK_DIS1_TRNG_EN Setting */
#define MXC_V_GCR_PCLK_DIS1_TRNG_DIS                   ((uint32_t)0x1UL) /**< PCLK_DIS1_TRNG_DIS Value */
#define MXC_S_GCR_PCLK_DIS1_TRNG_DIS                   (MXC_V_GCR_PCLK_DIS1_TRNG_DIS << MXC_F_GCR_PCLK_DIS1_TRNG_POS) /**< PCLK_DIS1_TRNG_DIS Setting */

#define MXC_F_GCR_PCLK_DIS1_SFLC_POS                   3 /**< PCLK_DIS1_SFLC Position */
#define MXC_F_GCR_PCLK_DIS1_SFLC                       ((uint32_t)(0x1UL << MXC_F_GCR_PCLK_DIS1_SFLC_POS)) /**< PCLK_DIS1_SFLC Mask */
#define MXC_V_GCR_PCLK_DIS1_SFLC_EN                    ((uint32_t)0x0UL) /**< PCLK_DIS1_SFLC_EN Value */
#define MXC_S_GCR_PCLK_DIS1_SFLC_EN                    (MXC_V_GCR_PCLK_DIS1_SFLC_EN << MXC_F_GCR_PCLK_DIS1_SFLC_POS) /**< PCLK_DIS1_SFLC_EN Setting */
#define MXC_V_GCR_PCLK_DIS1_SFLC_DIS                   ((uint32_t)0x1UL) /**< PCLK_DIS1_SFLC_DIS Value */
#define MXC_S_GCR_PCLK_DIS1_SFLC_DIS                   (MXC_V_GCR_PCLK_DIS1_SFLC_DIS << MXC_F_GCR_PCLK_DIS1_SFLC_POS) /**< PCLK_DIS1_SFLC_DIS Setting */

#define MXC_F_GCR_PCLK_DIS1_HBC_POS                    4 /**< PCLK_DIS1_HBC Position */
#define MXC_F_GCR_PCLK_DIS1_HBC                        ((uint32_t)(0x1UL << MXC_F_GCR_PCLK_DIS1_HBC_POS)) /**< PCLK_DIS1_HBC Mask */
#define MXC_V_GCR_PCLK_DIS1_HBC_EN                     ((uint32_t)0x0UL) /**< PCLK_DIS1_HBC_EN Value */
#define MXC_S_GCR_PCLK_DIS1_HBC_EN                     (MXC_V_GCR_PCLK_DIS1_HBC_EN << MXC_F_GCR_PCLK_DIS1_HBC_POS) /**< PCLK_DIS1_HBC_EN Setting */
#define MXC_V_GCR_PCLK_DIS1_HBC_DIS                    ((uint32_t)0x1UL) /**< PCLK_DIS1_HBC_DIS Value */
#define MXC_S_GCR_PCLK_DIS1_HBC_DIS                    (MXC_V_GCR_PCLK_DIS1_HBC_DIS << MXC_F_GCR_PCLK_DIS1_HBC_POS) /**< PCLK_DIS1_HBC_DIS Setting */

#define MXC_F_GCR_PCLK_DIS1_GPIO3_POS                  6 /**< PCLK_DIS1_GPIO3 Position */
#define MXC_F_GCR_PCLK_DIS1_GPIO3                      ((uint32_t)(0x1UL << MXC_F_GCR_PCLK_DIS1_GPIO3_POS)) /**< PCLK_DIS1_GPIO3 Mask */
#define MXC_V_GCR_PCLK_DIS1_GPIO3_EN                   ((uint32_t)0x0UL) /**< PCLK_DIS1_GPIO3_EN Value */
#define MXC_S_GCR_PCLK_DIS1_GPIO3_EN                   (MXC_V_GCR_PCLK_DIS1_GPIO3_EN << MXC_F_GCR_PCLK_DIS1_GPIO3_POS) /**< PCLK_DIS1_GPIO3_EN Setting */
#define MXC_V_GCR_PCLK_DIS1_GPIO3_DIS                  ((uint32_t)0x1UL) /**< PCLK_DIS1_GPIO3_DIS Value */
#define MXC_S_GCR_PCLK_DIS1_GPIO3_DIS                  (MXC_V_GCR_PCLK_DIS1_GPIO3_DIS << MXC_F_GCR_PCLK_DIS1_GPIO3_POS) /**< PCLK_DIS1_GPIO3_DIS Setting */

#define MXC_F_GCR_PCLK_DIS1_SCACHE_POS                 7 /**< PCLK_DIS1_SCACHE Position */
#define MXC_F_GCR_PCLK_DIS1_SCACHE                     ((uint32_t)(0x1UL << MXC_F_GCR_PCLK_DIS1_SCACHE_POS)) /**< PCLK_DIS1_SCACHE Mask */
#define MXC_V_GCR_PCLK_DIS1_SCACHE_EN                  ((uint32_t)0x0UL) /**< PCLK_DIS1_SCACHE_EN Value */
#define MXC_S_GCR_PCLK_DIS1_SCACHE_EN                  (MXC_V_GCR_PCLK_DIS1_SCACHE_EN << MXC_F_GCR_PCLK_DIS1_SCACHE_POS) /**< PCLK_DIS1_SCACHE_EN Setting */
#define MXC_V_GCR_PCLK_DIS1_SCACHE_DIS                 ((uint32_t)0x1UL) /**< PCLK_DIS1_SCACHE_DIS Value */
#define MXC_S_GCR_PCLK_DIS1_SCACHE_DIS                 (MXC_V_GCR_PCLK_DIS1_SCACHE_DIS << MXC_F_GCR_PCLK_DIS1_SCACHE_POS) /**< PCLK_DIS1_SCACHE_DIS Setting */

#define MXC_F_GCR_PCLK_DIS1_SDMA_POS                   8 /**< PCLK_DIS1_SDMA Position */
#define MXC_F_GCR_PCLK_DIS1_SDMA                       ((uint32_t)(0x1UL << MXC_F_GCR_PCLK_DIS1_SDMA_POS)) /**< PCLK_DIS1_SDMA Mask */
#define MXC_V_GCR_PCLK_DIS1_SDMA_EN                    ((uint32_t)0x0UL) /**< PCLK_DIS1_SDMA_EN Value */
#define MXC_S_GCR_PCLK_DIS1_SDMA_EN                    (MXC_V_GCR_PCLK_DIS1_SDMA_EN << MXC_F_GCR_PCLK_DIS1_SDMA_POS) /**< PCLK_DIS1_SDMA_EN Setting */
#define MXC_V_GCR_PCLK_DIS1_SDMA_DIS                   ((uint32_t)0x1UL) /**< PCLK_DIS1_SDMA_DIS Value */
#define MXC_S_GCR_PCLK_DIS1_SDMA_DIS                   (MXC_V_GCR_PCLK_DIS1_SDMA_DIS << MXC_F_GCR_PCLK_DIS1_SDMA_POS) /**< PCLK_DIS1_SDMA_DIS Setting */

#define MXC_F_GCR_PCLK_DIS1_SEMA_POS                   9 /**< PCLK_DIS1_SEMA Position */
#define MXC_F_GCR_PCLK_DIS1_SEMA                       ((uint32_t)(0x1UL << MXC_F_GCR_PCLK_DIS1_SEMA_POS)) /**< PCLK_DIS1_SEMA Mask */
#define MXC_V_GCR_PCLK_DIS1_SEMA_EN                    ((uint32_t)0x0UL) /**< PCLK_DIS1_SEMA_EN Value */
#define MXC_S_GCR_PCLK_DIS1_SEMA_EN                    (MXC_V_GCR_PCLK_DIS1_SEMA_EN << MXC_F_GCR_PCLK_DIS1_SEMA_POS) /**< PCLK_DIS1_SEMA_EN Setting */
#define MXC_V_GCR_PCLK_DIS1_SEMA_DIS                   ((uint32_t)0x1UL) /**< PCLK_DIS1_SEMA_DIS Value */
#define MXC_S_GCR_PCLK_DIS1_SEMA_DIS                   (MXC_V_GCR_PCLK_DIS1_SEMA_DIS << MXC_F_GCR_PCLK_DIS1_SEMA_POS) /**< PCLK_DIS1_SEMA_DIS Setting */

#define MXC_F_GCR_PCLK_DIS1_SDHC_POS                   10 /**< PCLK_DIS1_SDHC Position */
#define MXC_F_GCR_PCLK_DIS1_SDHC                       ((uint32_t)(0x1UL << MXC_F_GCR_PCLK_DIS1_SDHC_POS)) /**< PCLK_DIS1_SDHC Mask */
#define MXC_V_GCR_PCLK_DIS1_SDHC_EN                    ((uint32_t)0x0UL) /**< PCLK_DIS1_SDHC_EN Value */
#define MXC_S_GCR_PCLK_DIS1_SDHC_EN                    (MXC_V_GCR_PCLK_DIS1_SDHC_EN << MXC_F_GCR_PCLK_DIS1_SDHC_POS) /**< PCLK_DIS1_SDHC_EN Setting */
#define MXC_V_GCR_PCLK_DIS1_SDHC_DIS                   ((uint32_t)0x1UL) /**< PCLK_DIS1_SDHC_DIS Value */
#define MXC_S_GCR_PCLK_DIS1_SDHC_DIS                   (MXC_V_GCR_PCLK_DIS1_SDHC_DIS << MXC_F_GCR_PCLK_DIS1_SDHC_POS) /**< PCLK_DIS1_SDHC_DIS Setting */

#define MXC_F_GCR_PCLK_DIS1_ICACHE_POS                 11 /**< PCLK_DIS1_ICACHE Position */
#define MXC_F_GCR_PCLK_DIS1_ICACHE                     ((uint32_t)(0x1UL << MXC_F_GCR_PCLK_DIS1_ICACHE_POS)) /**< PCLK_DIS1_ICACHE Mask */
#define MXC_V_GCR_PCLK_DIS1_ICACHE_EN                  ((uint32_t)0x0UL) /**< PCLK_DIS1_ICACHE_EN Value */
#define MXC_S_GCR_PCLK_DIS1_ICACHE_EN                  (MXC_V_GCR_PCLK_DIS1_ICACHE_EN << MXC_F_GCR_PCLK_DIS1_ICACHE_POS) /**< PCLK_DIS1_ICACHE_EN Setting */
#define MXC_V_GCR_PCLK_DIS1_ICACHE_DIS                 ((uint32_t)0x1UL) /**< PCLK_DIS1_ICACHE_DIS Value */
#define MXC_S_GCR_PCLK_DIS1_ICACHE_DIS                 (MXC_V_GCR_PCLK_DIS1_ICACHE_DIS << MXC_F_GCR_PCLK_DIS1_ICACHE_POS) /**< PCLK_DIS1_ICACHE_DIS Setting */

#define MXC_F_GCR_PCLK_DIS1_ICACHEXIPF_POS             12 /**< PCLK_DIS1_ICACHEXIPF Position */
#define MXC_F_GCR_PCLK_DIS1_ICACHEXIPF                 ((uint32_t)(0x1UL << MXC_F_GCR_PCLK_DIS1_ICACHEXIPF_POS)) /**< PCLK_DIS1_ICACHEXIPF Mask */
#define MXC_V_GCR_PCLK_DIS1_ICACHEXIPF_EN              ((uint32_t)0x0UL) /**< PCLK_DIS1_ICACHEXIPF_EN Value */
#define MXC_S_GCR_PCLK_DIS1_ICACHEXIPF_EN              (MXC_V_GCR_PCLK_DIS1_ICACHEXIPF_EN << MXC_F_GCR_PCLK_DIS1_ICACHEXIPF_POS) /**< PCLK_DIS1_ICACHEXIPF_EN Setting */
#define MXC_V_GCR_PCLK_DIS1_ICACHEXIPF_DIS             ((uint32_t)0x1UL) /**< PCLK_DIS1_ICACHEXIPF_DIS Value */
#define MXC_S_GCR_PCLK_DIS1_ICACHEXIPF_DIS             (MXC_V_GCR_PCLK_DIS1_ICACHEXIPF_DIS << MXC_F_GCR_PCLK_DIS1_ICACHEXIPF_POS) /**< PCLK_DIS1_ICACHEXIPF_DIS Setting */

#define MXC_F_GCR_PCLK_DIS1_OW_POS                     13 /**< PCLK_DIS1_OW Position */
#define MXC_F_GCR_PCLK_DIS1_OW                         ((uint32_t)(0x1UL << MXC_F_GCR_PCLK_DIS1_OW_POS)) /**< PCLK_DIS1_OW Mask */
#define MXC_V_GCR_PCLK_DIS1_OW_EN                      ((uint32_t)0x0UL) /**< PCLK_DIS1_OW_EN Value */
#define MXC_S_GCR_PCLK_DIS1_OW_EN                      (MXC_V_GCR_PCLK_DIS1_OW_EN << MXC_F_GCR_PCLK_DIS1_OW_POS) /**< PCLK_DIS1_OW_EN Setting */
#define MXC_V_GCR_PCLK_DIS1_OW_DIS                     ((uint32_t)0x1UL) /**< PCLK_DIS1_OW_DIS Value */
#define MXC_S_GCR_PCLK_DIS1_OW_DIS                     (MXC_V_GCR_PCLK_DIS1_OW_DIS << MXC_F_GCR_PCLK_DIS1_OW_POS) /**< PCLK_DIS1_OW_DIS Setting */

#define MXC_F_GCR_PCLK_DIS1_SPI3_POS                   14 /**< PCLK_DIS1_SPI3 Position */
#define MXC_F_GCR_PCLK_DIS1_SPI3                       ((uint32_t)(0x1UL << MXC_F_GCR_PCLK_DIS1_SPI3_POS)) /**< PCLK_DIS1_SPI3 Mask */
#define MXC_V_GCR_PCLK_DIS1_SPI3_EN                    ((uint32_t)0x0UL) /**< PCLK_DIS1_SPI3_EN Value */
#define MXC_S_GCR_PCLK_DIS1_SPI3_EN                    (MXC_V_GCR_PCLK_DIS1_SPI3_EN << MXC_F_GCR_PCLK_DIS1_SPI3_POS) /**< PCLK_DIS1_SPI3_EN Setting */
#define MXC_V_GCR_PCLK_DIS1_SPI3_DIS                   ((uint32_t)0x1UL) /**< PCLK_DIS1_SPI3_DIS Value */
#define MXC_S_GCR_PCLK_DIS1_SPI3_DIS                   (MXC_V_GCR_PCLK_DIS1_SPI3_DIS << MXC_F_GCR_PCLK_DIS1_SPI3_POS) /**< PCLK_DIS1_SPI3_DIS Setting */

#define MXC_F_GCR_PCLK_DIS1_I2S_POS                    15 /**< PCLK_DIS1_I2S Position */
#define MXC_F_GCR_PCLK_DIS1_I2S                        ((uint32_t)(0x1UL << MXC_F_GCR_PCLK_DIS1_I2S_POS)) /**< PCLK_DIS1_I2S Mask */
#define MXC_V_GCR_PCLK_DIS1_I2S_EN                     ((uint32_t)0x0UL) /**< PCLK_DIS1_I2S_EN Value */
#define MXC_S_GCR_PCLK_DIS1_I2S_EN                     (MXC_V_GCR_PCLK_DIS1_I2S_EN << MXC_F_GCR_PCLK_DIS1_I2S_POS) /**< PCLK_DIS1_I2S_EN Setting */
#define MXC_V_GCR_PCLK_DIS1_I2S_DIS                    ((uint32_t)0x1UL) /**< PCLK_DIS1_I2S_DIS Value */
#define MXC_S_GCR_PCLK_DIS1_I2S_DIS                    (MXC_V_GCR_PCLK_DIS1_I2S_DIS << MXC_F_GCR_PCLK_DIS1_I2S_POS) /**< PCLK_DIS1_I2S_DIS Setting */

#define MXC_F_GCR_PCLK_DIS1_SPIXIPR_POS                20 /**< PCLK_DIS1_SPIXIPR Position */
#define MXC_F_GCR_PCLK_DIS1_SPIXIPR                    ((uint32_t)(0x1UL << MXC_F_GCR_PCLK_DIS1_SPIXIPR_POS)) /**< PCLK_DIS1_SPIXIPR Mask */
#define MXC_V_GCR_PCLK_DIS1_SPIXIPR_EN                 ((uint32_t)0x0UL) /**< PCLK_DIS1_SPIXIPR_EN Value */
#define MXC_S_GCR_PCLK_DIS1_SPIXIPR_EN                 (MXC_V_GCR_PCLK_DIS1_SPIXIPR_EN << MXC_F_GCR_PCLK_DIS1_SPIXIPR_POS) /**< PCLK_DIS1_SPIXIPR_EN Setting */
#define MXC_V_GCR_PCLK_DIS1_SPIXIPR_DIS                ((uint32_t)0x1UL) /**< PCLK_DIS1_SPIXIPR_DIS Value */
#define MXC_S_GCR_PCLK_DIS1_SPIXIPR_DIS                (MXC_V_GCR_PCLK_DIS1_SPIXIPR_DIS << MXC_F_GCR_PCLK_DIS1_SPIXIPR_POS) /**< PCLK_DIS1_SPIXIPR_DIS Setting */

/**@} end of group GCR_PCLK_DIS1_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_EVENT_EN GCR_EVENT_EN
 * @brief    Event Enable Register.
 * @{
 */
#define MXC_F_GCR_EVENT_EN_DMAEVENT_POS                0 /**< EVENT_EN_DMAEVENT Position */
#define MXC_F_GCR_EVENT_EN_DMAEVENT                    ((uint32_t)(0x1UL << MXC_F_GCR_EVENT_EN_DMAEVENT_POS)) /**< EVENT_EN_DMAEVENT Mask */
#define MXC_V_GCR_EVENT_EN_DMAEVENT_DIS                ((uint32_t)0x0UL) /**< EVENT_EN_DMAEVENT_DIS Value */
#define MXC_S_GCR_EVENT_EN_DMAEVENT_DIS                (MXC_V_GCR_EVENT_EN_DMAEVENT_DIS << MXC_F_GCR_EVENT_EN_DMAEVENT_POS) /**< EVENT_EN_DMAEVENT_DIS Setting */
#define MXC_V_GCR_EVENT_EN_DMAEVENT_EN                 ((uint32_t)0x1UL) /**< EVENT_EN_DMAEVENT_EN Value */
#define MXC_S_GCR_EVENT_EN_DMAEVENT_EN                 (MXC_V_GCR_EVENT_EN_DMAEVENT_EN << MXC_F_GCR_EVENT_EN_DMAEVENT_POS) /**< EVENT_EN_DMAEVENT_EN Setting */

#define MXC_F_GCR_EVENT_EN_RXEVENT_POS                 1 /**< EVENT_EN_RXEVENT Position */
#define MXC_F_GCR_EVENT_EN_RXEVENT                     ((uint32_t)(0x1UL << MXC_F_GCR_EVENT_EN_RXEVENT_POS)) /**< EVENT_EN_RXEVENT Mask */
#define MXC_V_GCR_EVENT_EN_RXEVENT_DIS                 ((uint32_t)0x0UL) /**< EVENT_EN_RXEVENT_DIS Value */
#define MXC_S_GCR_EVENT_EN_RXEVENT_DIS                 (MXC_V_GCR_EVENT_EN_RXEVENT_DIS << MXC_F_GCR_EVENT_EN_RXEVENT_POS) /**< EVENT_EN_RXEVENT_DIS Setting */
#define MXC_V_GCR_EVENT_EN_RXEVENT_EN                  ((uint32_t)0x1UL) /**< EVENT_EN_RXEVENT_EN Value */
#define MXC_S_GCR_EVENT_EN_RXEVENT_EN                  (MXC_V_GCR_EVENT_EN_RXEVENT_EN << MXC_F_GCR_EVENT_EN_RXEVENT_POS) /**< EVENT_EN_RXEVENT_EN Setting */

#define MXC_F_GCR_EVENT_EN_TXEVENT_POS                 2 /**< EVENT_EN_TXEVENT Position */
#define MXC_F_GCR_EVENT_EN_TXEVENT                     ((uint32_t)(0x1UL << MXC_F_GCR_EVENT_EN_TXEVENT_POS)) /**< EVENT_EN_TXEVENT Mask */
#define MXC_V_GCR_EVENT_EN_TXEVENT_DIS                 ((uint32_t)0x0UL) /**< EVENT_EN_TXEVENT_DIS Value */
#define MXC_S_GCR_EVENT_EN_TXEVENT_DIS                 (MXC_V_GCR_EVENT_EN_TXEVENT_DIS << MXC_F_GCR_EVENT_EN_TXEVENT_POS) /**< EVENT_EN_TXEVENT_DIS Setting */
#define MXC_V_GCR_EVENT_EN_TXEVENT_EN                  ((uint32_t)0x1UL) /**< EVENT_EN_TXEVENT_EN Value */
#define MXC_S_GCR_EVENT_EN_TXEVENT_EN                  (MXC_V_GCR_EVENT_EN_TXEVENT_EN << MXC_F_GCR_EVENT_EN_TXEVENT_POS) /**< EVENT_EN_TXEVENT_EN Setting */

/**@} end of group GCR_EVENT_EN_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_REV GCR_REV
 * @brief    Revision Register.
 * @{
 */
#define MXC_F_GCR_REV_REVISION_POS                     0 /**< REV_REVISION Position */
#define MXC_F_GCR_REV_REVISION                         ((uint32_t)(0xFFFFUL << MXC_F_GCR_REV_REVISION_POS)) /**< REV_REVISION Mask */

/**@} end of group GCR_REV_Register */

/**
 * @ingroup  gcr_registers
 * @defgroup GCR_SYS_STAT_IE GCR_SYS_STAT_IE
 * @brief    System Status Interrupt Enable Register.
 * @{
 */
#define MXC_F_GCR_SYS_STAT_IE_ICEULIE_POS              0 /**< SYS_STAT_IE_ICEULIE Position */
#define MXC_F_GCR_SYS_STAT_IE_ICEULIE                  ((uint32_t)(0x1UL << MXC_F_GCR_SYS_STAT_IE_ICEULIE_POS)) /**< SYS_STAT_IE_ICEULIE Mask */
#define MXC_V_GCR_SYS_STAT_IE_ICEULIE_DIS              ((uint32_t)0x0UL) /**< SYS_STAT_IE_ICEULIE_DIS Value */
#define MXC_S_GCR_SYS_STAT_IE_ICEULIE_DIS              (MXC_V_GCR_SYS_STAT_IE_ICEULIE_DIS << MXC_F_GCR_SYS_STAT_IE_ICEULIE_POS) /**< SYS_STAT_IE_ICEULIE_DIS Setting */
#define MXC_V_GCR_SYS_STAT_IE_ICEULIE_EN               ((uint32_t)0x1UL) /**< SYS_STAT_IE_ICEULIE_EN Value */
#define MXC_S_GCR_SYS_STAT_IE_ICEULIE_EN               (MXC_V_GCR_SYS_STAT_IE_ICEULIE_EN << MXC_F_GCR_SYS_STAT_IE_ICEULIE_POS) /**< SYS_STAT_IE_ICEULIE_EN Setting */

#define MXC_F_GCR_SYS_STAT_IE_CIEIE_POS                1 /**< SYS_STAT_IE_CIEIE Position */
#define MXC_F_GCR_SYS_STAT_IE_CIEIE                    ((uint32_t)(0x1UL << MXC_F_GCR_SYS_STAT_IE_CIEIE_POS)) /**< SYS_STAT_IE_CIEIE Mask */
#define MXC_V_GCR_SYS_STAT_IE_CIEIE_DIS                ((uint32_t)0x0UL) /**< SYS_STAT_IE_CIEIE_DIS Value */
#define MXC_S_GCR_SYS_STAT_IE_CIEIE_DIS                (MXC_V_GCR_SYS_STAT_IE_CIEIE_DIS << MXC_F_GCR_SYS_STAT_IE_CIEIE_POS) /**< SYS_STAT_IE_CIEIE_DIS Setting */
#define MXC_V_GCR_SYS_STAT_IE_CIEIE_EN                 ((uint32_t)0x1UL) /**< SYS_STAT_IE_CIEIE_EN Value */
#define MXC_S_GCR_SYS_STAT_IE_CIEIE_EN                 (MXC_V_GCR_SYS_STAT_IE_CIEIE_EN << MXC_F_GCR_SYS_STAT_IE_CIEIE_POS) /**< SYS_STAT_IE_CIEIE_EN Setting */

#define MXC_F_GCR_SYS_STAT_IE_SCMFIE_POS               5 /**< SYS_STAT_IE_SCMFIE Position */
#define MXC_F_GCR_SYS_STAT_IE_SCMFIE                   ((uint32_t)(0x1UL << MXC_F_GCR_SYS_STAT_IE_SCMFIE_POS)) /**< SYS_STAT_IE_SCMFIE Mask */
#define MXC_V_GCR_SYS_STAT_IE_SCMFIE_DIS               ((uint32_t)0x0UL) /**< SYS_STAT_IE_SCMFIE_DIS Value */
#define MXC_S_GCR_SYS_STAT_IE_SCMFIE_DIS               (MXC_V_GCR_SYS_STAT_IE_SCMFIE_DIS << MXC_F_GCR_SYS_STAT_IE_SCMFIE_POS) /**< SYS_STAT_IE_SCMFIE_DIS Setting */
#define MXC_V_GCR_SYS_STAT_IE_SCMFIE_EN                ((uint32_t)0x1UL) /**< SYS_STAT_IE_SCMFIE_EN Value */
#define MXC_S_GCR_SYS_STAT_IE_SCMFIE_EN                (MXC_V_GCR_SYS_STAT_IE_SCMFIE_EN << MXC_F_GCR_SYS_STAT_IE_SCMFIE_POS) /**< SYS_STAT_IE_SCMFIE_EN Setting */

/**@} end of group GCR_SYS_STAT_IE_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32650_INCLUDE_GCR_REGS_H_
