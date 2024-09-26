/**
 * @file    rpu_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the RPU Peripheral Module.
 * @note    This file is @generated.
 * @ingroup rpu_registers
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32665_INCLUDE_RPU_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32665_INCLUDE_RPU_REGS_H_

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
 * @ingroup     rpu
 * @defgroup    rpu_registers RPU_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the RPU Peripheral Module.
 * @details     Resource Protection Unit
 */

/**
 * @ingroup rpu_registers
 * Structure type to access the RPU Registers.
 */
typedef struct {
    __IO uint32_t gcr;                  /**< <tt>\b 0x0000:</tt> RPU GCR Register */
    __IO uint32_t sir;                  /**< <tt>\b 0x0004:</tt> RPU SIR Register */
    __IO uint32_t fcr;                  /**< <tt>\b 0x0008:</tt> RPU FCR Register */
    __R  uint32_t rsv_0xc;
    __IO uint32_t tpu;                  /**< <tt>\b 0x0010:</tt> RPU TPU Register */
    __R  uint32_t rsv_0x14_0x1f[3];
    __IO uint32_t rpu;                  /**< <tt>\b 0x0020:</tt> RPU RPU Register */
    __R  uint32_t rsv_0x24_0x2f[3];
    __IO uint32_t wdt0;                 /**< <tt>\b 0x0030:</tt> RPU WDT0 Register */
    __IO uint32_t wdt1;                 /**< <tt>\b 0x0034:</tt> RPU WDT1 Register */
    __IO uint32_t wdt2;                 /**< <tt>\b 0x0038:</tt> RPU WDT2 Register */
    __R  uint32_t rsv_0x3c;
    __IO uint32_t smon;                 /**< <tt>\b 0x0040:</tt> RPU SMON Register */
    __IO uint32_t simo;                 /**< <tt>\b 0x0044:</tt> RPU SIMO Register */
    __IO uint32_t dvs;                  /**< <tt>\b 0x0048:</tt> RPU DVS Register */
    __R  uint32_t rsv_0x4c;
    __IO uint32_t aes;                  /**< <tt>\b 0x0050:</tt> RPU AES Register */
    __R  uint32_t rsv_0x54_0x5f[3];
    __IO uint32_t rtc;                  /**< <tt>\b 0x0060:</tt> RPU RTC Register */
    __IO uint32_t wut;                  /**< <tt>\b 0x0064:</tt> RPU WUT Register */
    __IO uint32_t pwrseq;               /**< <tt>\b 0x0068:</tt> RPU PWRSEQ Register */
    __IO uint32_t mcr;                  /**< <tt>\b 0x006C:</tt> RPU MCR Register */
    __R  uint32_t rsv_0x70_0x7f[4];
    __IO uint32_t gpio0;                /**< <tt>\b 0x0080:</tt> RPU GPIO0 Register */
    __R  uint32_t rsv_0x84_0x8f[3];
    __IO uint32_t gpio1;                /**< <tt>\b 0x0090:</tt> RPU GPIO1 Register */
    __R  uint32_t rsv_0x94_0xff[27];
    __IO uint32_t tmr0;                 /**< <tt>\b 0x0100:</tt> RPU TMR0 Register */
    __R  uint32_t rsv_0x104_0x10f[3];
    __IO uint32_t tmr1;                 /**< <tt>\b 0x0110:</tt> RPU TMR1 Register */
    __R  uint32_t rsv_0x114_0x11f[3];
    __IO uint32_t tmr2;                 /**< <tt>\b 0x0120:</tt> RPU TMR2 Register */
    __R  uint32_t rsv_0x124_0x12f[3];
    __IO uint32_t tmr3;                 /**< <tt>\b 0x0130:</tt> RPU TMR3 Register */
    __R  uint32_t rsv_0x134_0x13f[3];
    __IO uint32_t tmr4;                 /**< <tt>\b 0x0140:</tt> RPU TMR4 Register */
    __R  uint32_t rsv_0x144_0x14f[3];
    __IO uint32_t tmr5;                 /**< <tt>\b 0x0150:</tt> RPU TMR5 Register */
    __R  uint32_t rsv_0x154_0x1af[23];
    __IO uint32_t htimer0;              /**< <tt>\b 0x01B0:</tt> RPU HTIMER0 Register */
    __R  uint32_t rsv_0x1b4_0x1bf[3];
    __IO uint32_t htimer1;              /**< <tt>\b 0x01C0:</tt> RPU HTIMER1 Register */
    __R  uint32_t rsv_0x1c4_0x1cf[3];
    __IO uint32_t i2c0_bus0;            /**< <tt>\b 0x01D0:</tt> RPU I2C0_BUS0 Register */
    __R  uint32_t rsv_0x1d4_0x1df[3];
    __IO uint32_t i2c1_bus0;            /**< <tt>\b 0x01E0:</tt> RPU I2C1_BUS0 Register */
    __R  uint32_t rsv_0x1e4_0x1ef[3];
    __IO uint32_t i2c2_bus0;            /**< <tt>\b 0x01F0:</tt> RPU I2C2_BUS0 Register */
    __R  uint32_t rsv_0x1f4_0x25f[27];
    __IO uint32_t spixfm;               /**< <tt>\b 0x0260:</tt> RPU SPIXFM Register */
    __R  uint32_t rsv_0x264_0x26f[3];
    __IO uint32_t spixfc;               /**< <tt>\b 0x0270:</tt> RPU SPIXFC Register */
    __R  uint32_t rsv_0x274_0x27f[3];
    __IO uint32_t dma0;                 /**< <tt>\b 0x0280:</tt> RPU DMA0 Register */
    __R  uint32_t rsv_0x284_0x28f[3];
    __IO uint32_t flc0;                 /**< <tt>\b 0x0290:</tt> RPU FLC0 Register */
    __IO uint32_t flc1;                 /**< <tt>\b 0x0294:</tt> RPU FLC1 Register */
    __R  uint32_t rsv_0x298_0x29f[2];
    __IO uint32_t icc0;                 /**< <tt>\b 0x02A0:</tt> RPU ICC0 Register */
    __IO uint32_t icc1;                 /**< <tt>\b 0x02A4:</tt> RPU ICC1 Register */
    __R  uint32_t rsv_0x2a8_0x2ef[18];
    __IO uint32_t sfcc;                 /**< <tt>\b 0x02F0:</tt> RPU SFCC Register */
    __R  uint32_t rsv_0x2f4_0x32f[15];
    __IO uint32_t srcc;                 /**< <tt>\b 0x0330:</tt> RPU SRCC Register */
    __R  uint32_t rsv_0x334_0x33f[3];
    __IO uint32_t adc;                  /**< <tt>\b 0x0340:</tt> RPU ADC Register */
    __R  uint32_t rsv_0x344_0x34f[3];
    __IO uint32_t dma1;                 /**< <tt>\b 0x0350:</tt> RPU DMA1 Register */
    __R  uint32_t rsv_0x354_0x35f[3];
    __IO uint32_t sdma;                 /**< <tt>\b 0x0360:</tt> RPU SDMA Register */
    __R  uint32_t rsv_0x364_0x36f[3];
    __IO uint32_t sdhcctrl;             /**< <tt>\b 0x0370:</tt> RPU SDHCCTRL Register */
    __R  uint32_t rsv_0x374_0x39f[11];
    __IO uint32_t spixr;                /**< <tt>\b 0x03A0:</tt> RPU SPIXR Register */
    __R  uint32_t rsv_0x3a4_0x3bf[7];
    __IO uint32_t ptg_bus0;             /**< <tt>\b 0x03C0:</tt> RPU PTG_BUS0 Register */
    __R  uint32_t rsv_0x3c4_0x3cf[3];
    __IO uint32_t owm;                  /**< <tt>\b 0x03D0:</tt> RPU OWM Register */
    __R  uint32_t rsv_0x3d4_0x3df[3];
    __IO uint32_t sema;                 /**< <tt>\b 0x03E0:</tt> RPU SEMA Register */
    __R  uint32_t rsv_0x3e4_0x41f[15];
    __IO uint32_t uart0;                /**< <tt>\b 0x0420:</tt> RPU UART0 Register */
    __R  uint32_t rsv_0x424_0x42f[3];
    __IO uint32_t uart1;                /**< <tt>\b 0x0430:</tt> RPU UART1 Register */
    __R  uint32_t rsv_0x434_0x43f[3];
    __IO uint32_t uart2;                /**< <tt>\b 0x0440:</tt> RPU UART2 Register */
    __R  uint32_t rsv_0x444_0x45f[7];
    __IO uint32_t spi1;                 /**< <tt>\b 0x0460:</tt> RPU SPI1 Register */
    __R  uint32_t rsv_0x464_0x46f[3];
    __IO uint32_t spi2;                 /**< <tt>\b 0x0470:</tt> RPU SPI2 Register */
    __R  uint32_t rsv_0x474_0x4bf[19];
    __IO uint32_t audio;                /**< <tt>\b 0x04C0:</tt> RPU AUDIO Register */
    __R  uint32_t rsv_0x4c4_0x4cf[3];
    __IO uint32_t trng;                 /**< <tt>\b 0x04D0:</tt> RPU TRNG Register */
    __R  uint32_t rsv_0x4d4_0x4ff[11];
    __IO uint32_t btle;                 /**< <tt>\b 0x0500:</tt> RPU BTLE Register */
    __R  uint32_t rsv_0x504_0xb0f[387];
    __IO uint32_t usbhs;                /**< <tt>\b 0x0B10:</tt> RPU USBHS Register */
    __R  uint32_t rsv_0xb14_0xb5f[19];
    __IO uint32_t sdio;                 /**< <tt>\b 0x0B60:</tt> RPU SDIO Register */
    __R  uint32_t rsv_0xb64_0xbbf[23];
    __IO uint32_t spixfm_fifo;          /**< <tt>\b 0x0BC0:</tt> RPU SPIXFM_FIFO Register */
    __R  uint32_t rsv_0xbc4_0xbdf[7];
    __IO uint32_t spi0;                 /**< <tt>\b 0x0BE0:</tt> RPU SPI0 Register */
    __R  uint32_t rsv_0xbe4_0xeff[199];
    __IO uint32_t sysram0;              /**< <tt>\b 0x0F00:</tt> RPU SYSRAM0 Register */
    __R  uint32_t rsv_0xf04_0xf0f[3];
    __IO uint32_t sysram1;              /**< <tt>\b 0x0F10:</tt> RPU SYSRAM1 Register */
    __R  uint32_t rsv_0xf14_0xf1f[3];
    __IO uint32_t sysram2;              /**< <tt>\b 0x0F20:</tt> RPU SYSRAM2 Register */
    __R  uint32_t rsv_0xf24_0xf2f[3];
    __IO uint32_t sysram3;              /**< <tt>\b 0x0F30:</tt> RPU SYSRAM3 Register */
    __R  uint32_t rsv_0xf34_0xf3f[3];
    __IO uint32_t sysram4;              /**< <tt>\b 0x0F40:</tt> RPU SYSRAM4 Register */
    __R  uint32_t rsv_0xf44_0xf4f[3];
    __IO uint32_t sysram5;              /**< <tt>\b 0x0F50:</tt> RPU SYSRAM5 Register */
    __R  uint32_t rsv_0xf54_0xf5f[3];
    __IO uint32_t sysram6_11;           /**< <tt>\b 0x0F60:</tt> RPU SYSRAM6_11 Register */
    __R  uint32_t rsv_0xf64_0x11cf[155];
    __IO uint32_t i2c0_bus1;            /**< <tt>\b 0x11D0:</tt> RPU I2C0_BUS1 Register */
    __R  uint32_t rsv_0x11d4_0x11df[3];
    __IO uint32_t i2c1_bus1;            /**< <tt>\b 0x11E0:</tt> RPU I2C1_BUS1 Register */
    __R  uint32_t rsv_0x11e4_0x11ef[3];
    __IO uint32_t i2c2_bus1;            /**< <tt>\b 0x11F0:</tt> RPU I2C2_BUS1 Register */
    __R  uint32_t rsv_0x11f4_0x13bf[115];
    __IO uint32_t ptg_bus1;             /**< <tt>\b 0x13C0:</tt> RPU PTG_BUS1 Register */
} mxc_rpu_regs_t;

/* Register offsets for module RPU */
/**
 * @ingroup    rpu_registers
 * @defgroup   RPU_Register_Offsets Register Offsets
 * @brief      RPU Peripheral Register Offsets from the RPU Base Peripheral Address.
 * @{
 */
#define MXC_R_RPU_GCR                      ((uint32_t)0x00000000UL) /**< Offset from RPU Base Address: <tt> 0x0000</tt> */
#define MXC_R_RPU_SIR                      ((uint32_t)0x00000004UL) /**< Offset from RPU Base Address: <tt> 0x0004</tt> */
#define MXC_R_RPU_FCR                      ((uint32_t)0x00000008UL) /**< Offset from RPU Base Address: <tt> 0x0008</tt> */
#define MXC_R_RPU_TPU                      ((uint32_t)0x00000010UL) /**< Offset from RPU Base Address: <tt> 0x0010</tt> */
#define MXC_R_RPU_RPU                      ((uint32_t)0x00000020UL) /**< Offset from RPU Base Address: <tt> 0x0020</tt> */
#define MXC_R_RPU_WDT0                     ((uint32_t)0x00000030UL) /**< Offset from RPU Base Address: <tt> 0x0030</tt> */
#define MXC_R_RPU_WDT1                     ((uint32_t)0x00000034UL) /**< Offset from RPU Base Address: <tt> 0x0034</tt> */
#define MXC_R_RPU_WDT2                     ((uint32_t)0x00000038UL) /**< Offset from RPU Base Address: <tt> 0x0038</tt> */
#define MXC_R_RPU_SMON                     ((uint32_t)0x00000040UL) /**< Offset from RPU Base Address: <tt> 0x0040</tt> */
#define MXC_R_RPU_SIMO                     ((uint32_t)0x00000044UL) /**< Offset from RPU Base Address: <tt> 0x0044</tt> */
#define MXC_R_RPU_DVS                      ((uint32_t)0x00000048UL) /**< Offset from RPU Base Address: <tt> 0x0048</tt> */
#define MXC_R_RPU_AES                      ((uint32_t)0x00000050UL) /**< Offset from RPU Base Address: <tt> 0x0050</tt> */
#define MXC_R_RPU_RTC                      ((uint32_t)0x00000060UL) /**< Offset from RPU Base Address: <tt> 0x0060</tt> */
#define MXC_R_RPU_WUT                      ((uint32_t)0x00000064UL) /**< Offset from RPU Base Address: <tt> 0x0064</tt> */
#define MXC_R_RPU_PWRSEQ                   ((uint32_t)0x00000068UL) /**< Offset from RPU Base Address: <tt> 0x0068</tt> */
#define MXC_R_RPU_MCR                      ((uint32_t)0x0000006CUL) /**< Offset from RPU Base Address: <tt> 0x006C</tt> */
#define MXC_R_RPU_GPIO0                    ((uint32_t)0x00000080UL) /**< Offset from RPU Base Address: <tt> 0x0080</tt> */
#define MXC_R_RPU_GPIO1                    ((uint32_t)0x00000090UL) /**< Offset from RPU Base Address: <tt> 0x0090</tt> */
#define MXC_R_RPU_TMR0                     ((uint32_t)0x00000100UL) /**< Offset from RPU Base Address: <tt> 0x0100</tt> */
#define MXC_R_RPU_TMR1                     ((uint32_t)0x00000110UL) /**< Offset from RPU Base Address: <tt> 0x0110</tt> */
#define MXC_R_RPU_TMR2                     ((uint32_t)0x00000120UL) /**< Offset from RPU Base Address: <tt> 0x0120</tt> */
#define MXC_R_RPU_TMR3                     ((uint32_t)0x00000130UL) /**< Offset from RPU Base Address: <tt> 0x0130</tt> */
#define MXC_R_RPU_TMR4                     ((uint32_t)0x00000140UL) /**< Offset from RPU Base Address: <tt> 0x0140</tt> */
#define MXC_R_RPU_TMR5                     ((uint32_t)0x00000150UL) /**< Offset from RPU Base Address: <tt> 0x0150</tt> */
#define MXC_R_RPU_HTIMER0                  ((uint32_t)0x000001B0UL) /**< Offset from RPU Base Address: <tt> 0x01B0</tt> */
#define MXC_R_RPU_HTIMER1                  ((uint32_t)0x000001C0UL) /**< Offset from RPU Base Address: <tt> 0x01C0</tt> */
#define MXC_R_RPU_I2C0_BUS0                ((uint32_t)0x000001D0UL) /**< Offset from RPU Base Address: <tt> 0x01D0</tt> */
#define MXC_R_RPU_I2C1_BUS0                ((uint32_t)0x000001E0UL) /**< Offset from RPU Base Address: <tt> 0x01E0</tt> */
#define MXC_R_RPU_I2C2_BUS0                ((uint32_t)0x000001F0UL) /**< Offset from RPU Base Address: <tt> 0x01F0</tt> */
#define MXC_R_RPU_SPIXFM                   ((uint32_t)0x00000260UL) /**< Offset from RPU Base Address: <tt> 0x0260</tt> */
#define MXC_R_RPU_SPIXFC                   ((uint32_t)0x00000270UL) /**< Offset from RPU Base Address: <tt> 0x0270</tt> */
#define MXC_R_RPU_DMA0                     ((uint32_t)0x00000280UL) /**< Offset from RPU Base Address: <tt> 0x0280</tt> */
#define MXC_R_RPU_FLC0                     ((uint32_t)0x00000290UL) /**< Offset from RPU Base Address: <tt> 0x0290</tt> */
#define MXC_R_RPU_FLC1                     ((uint32_t)0x00000294UL) /**< Offset from RPU Base Address: <tt> 0x0294</tt> */
#define MXC_R_RPU_ICC0                     ((uint32_t)0x000002A0UL) /**< Offset from RPU Base Address: <tt> 0x02A0</tt> */
#define MXC_R_RPU_ICC1                     ((uint32_t)0x000002A4UL) /**< Offset from RPU Base Address: <tt> 0x02A4</tt> */
#define MXC_R_RPU_SFCC                     ((uint32_t)0x000002F0UL) /**< Offset from RPU Base Address: <tt> 0x02F0</tt> */
#define MXC_R_RPU_SRCC                     ((uint32_t)0x00000330UL) /**< Offset from RPU Base Address: <tt> 0x0330</tt> */
#define MXC_R_RPU_ADC                      ((uint32_t)0x00000340UL) /**< Offset from RPU Base Address: <tt> 0x0340</tt> */
#define MXC_R_RPU_DMA1                     ((uint32_t)0x00000350UL) /**< Offset from RPU Base Address: <tt> 0x0350</tt> */
#define MXC_R_RPU_SDMA                     ((uint32_t)0x00000360UL) /**< Offset from RPU Base Address: <tt> 0x0360</tt> */
#define MXC_R_RPU_SDHCCTRL                 ((uint32_t)0x00000370UL) /**< Offset from RPU Base Address: <tt> 0x0370</tt> */
#define MXC_R_RPU_SPIXR                    ((uint32_t)0x000003A0UL) /**< Offset from RPU Base Address: <tt> 0x03A0</tt> */
#define MXC_R_RPU_PTG_BUS0                 ((uint32_t)0x000003C0UL) /**< Offset from RPU Base Address: <tt> 0x03C0</tt> */
#define MXC_R_RPU_OWM                      ((uint32_t)0x000003D0UL) /**< Offset from RPU Base Address: <tt> 0x03D0</tt> */
#define MXC_R_RPU_SEMA                     ((uint32_t)0x000003E0UL) /**< Offset from RPU Base Address: <tt> 0x03E0</tt> */
#define MXC_R_RPU_UART0                    ((uint32_t)0x00000420UL) /**< Offset from RPU Base Address: <tt> 0x0420</tt> */
#define MXC_R_RPU_UART1                    ((uint32_t)0x00000430UL) /**< Offset from RPU Base Address: <tt> 0x0430</tt> */
#define MXC_R_RPU_UART2                    ((uint32_t)0x00000440UL) /**< Offset from RPU Base Address: <tt> 0x0440</tt> */
#define MXC_R_RPU_SPI1                     ((uint32_t)0x00000460UL) /**< Offset from RPU Base Address: <tt> 0x0460</tt> */
#define MXC_R_RPU_SPI2                     ((uint32_t)0x00000470UL) /**< Offset from RPU Base Address: <tt> 0x0470</tt> */
#define MXC_R_RPU_AUDIO                    ((uint32_t)0x000004C0UL) /**< Offset from RPU Base Address: <tt> 0x04C0</tt> */
#define MXC_R_RPU_TRNG                     ((uint32_t)0x000004D0UL) /**< Offset from RPU Base Address: <tt> 0x04D0</tt> */
#define MXC_R_RPU_BTLE                     ((uint32_t)0x00000500UL) /**< Offset from RPU Base Address: <tt> 0x0500</tt> */
#define MXC_R_RPU_USBHS                    ((uint32_t)0x00000B10UL) /**< Offset from RPU Base Address: <tt> 0x0B10</tt> */
#define MXC_R_RPU_SDIO                     ((uint32_t)0x00000B60UL) /**< Offset from RPU Base Address: <tt> 0x0B60</tt> */
#define MXC_R_RPU_SPIXFM_FIFO              ((uint32_t)0x00000BC0UL) /**< Offset from RPU Base Address: <tt> 0x0BC0</tt> */
#define MXC_R_RPU_SPI0                     ((uint32_t)0x00000BE0UL) /**< Offset from RPU Base Address: <tt> 0x0BE0</tt> */
#define MXC_R_RPU_SYSRAM0                  ((uint32_t)0x00000F00UL) /**< Offset from RPU Base Address: <tt> 0x0F00</tt> */
#define MXC_R_RPU_SYSRAM1                  ((uint32_t)0x00000F10UL) /**< Offset from RPU Base Address: <tt> 0x0F10</tt> */
#define MXC_R_RPU_SYSRAM2                  ((uint32_t)0x00000F20UL) /**< Offset from RPU Base Address: <tt> 0x0F20</tt> */
#define MXC_R_RPU_SYSRAM3                  ((uint32_t)0x00000F30UL) /**< Offset from RPU Base Address: <tt> 0x0F30</tt> */
#define MXC_R_RPU_SYSRAM4                  ((uint32_t)0x00000F40UL) /**< Offset from RPU Base Address: <tt> 0x0F40</tt> */
#define MXC_R_RPU_SYSRAM5                  ((uint32_t)0x00000F50UL) /**< Offset from RPU Base Address: <tt> 0x0F50</tt> */
#define MXC_R_RPU_SYSRAM6_11               ((uint32_t)0x00000F60UL) /**< Offset from RPU Base Address: <tt> 0x0F60</tt> */
#define MXC_R_RPU_I2C0_BUS1                ((uint32_t)0x000011D0UL) /**< Offset from RPU Base Address: <tt> 0x11D0</tt> */
#define MXC_R_RPU_I2C1_BUS1                ((uint32_t)0x000011E0UL) /**< Offset from RPU Base Address: <tt> 0x11E0</tt> */
#define MXC_R_RPU_I2C2_BUS1                ((uint32_t)0x000011F0UL) /**< Offset from RPU Base Address: <tt> 0x11F0</tt> */
#define MXC_R_RPU_PTG_BUS1                 ((uint32_t)0x000013C0UL) /**< Offset from RPU Base Address: <tt> 0x13C0</tt> */
/**@} end of group rpu_registers */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_GCR RPU_GCR
 * @brief    GCR RPU Register.
 * @{
 */
#define MXC_F_RPU_GCR_ACCESS_POS                       0 /**< GCR_ACCESS Position */
#define MXC_F_RPU_GCR_ACCESS                           ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_GCR_ACCESS_POS)) /**< GCR_ACCESS Mask */

/**@} end of group RPU_GCR_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_SIR RPU_SIR
 * @brief    SIR RPU Register.
 * @{
 */
#define MXC_F_RPU_SIR_ACCESS_POS                       0 /**< SIR_ACCESS Position */
#define MXC_F_RPU_SIR_ACCESS                           ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_SIR_ACCESS_POS)) /**< SIR_ACCESS Mask */

/**@} end of group RPU_SIR_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_FCR RPU_FCR
 * @brief    FCR RPU Register.
 * @{
 */
#define MXC_F_RPU_FCR_ACCESS_POS                       0 /**< FCR_ACCESS Position */
#define MXC_F_RPU_FCR_ACCESS                           ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_FCR_ACCESS_POS)) /**< FCR_ACCESS Mask */

/**@} end of group RPU_FCR_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_TPU RPU_TPU
 * @brief    TPU RPU Register.
 * @{
 */
#define MXC_F_RPU_TPU_ACCESS_POS                       0 /**< TPU_ACCESS Position */
#define MXC_F_RPU_TPU_ACCESS                           ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_TPU_ACCESS_POS)) /**< TPU_ACCESS Mask */

/**@} end of group RPU_TPU_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_RPU RPU_RPU
 * @brief    RPU Register.
 * @{
 */
#define MXC_F_RPU_RPU_ACCESS_POS                       0 /**< RPU_ACCESS Position */
#define MXC_F_RPU_RPU_ACCESS                           ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_RPU_ACCESS_POS)) /**< RPU_ACCESS Mask */

/**@} end of group RPU_RPU_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_WDT0 RPU_WDT0
 * @brief    WDT0 RPU Register.
 * @{
 */
#define MXC_F_RPU_WDT0_ACCESS_POS                      0 /**< WDT0_ACCESS Position */
#define MXC_F_RPU_WDT0_ACCESS                          ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_WDT0_ACCESS_POS)) /**< WDT0_ACCESS Mask */

/**@} end of group RPU_WDT0_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_WDT1 RPU_WDT1
 * @brief    WDT1 RPU Register.
 * @{
 */
#define MXC_F_RPU_WDT1_ACCESS_POS                      0 /**< WDT1_ACCESS Position */
#define MXC_F_RPU_WDT1_ACCESS                          ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_WDT1_ACCESS_POS)) /**< WDT1_ACCESS Mask */

/**@} end of group RPU_WDT1_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_WDT2 RPU_WDT2
 * @brief    WDT2 RPU Register.
 * @{
 */
#define MXC_F_RPU_WDT2_ACCESS_POS                      0 /**< WDT2_ACCESS Position */
#define MXC_F_RPU_WDT2_ACCESS                          ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_WDT2_ACCESS_POS)) /**< WDT2_ACCESS Mask */

/**@} end of group RPU_WDT2_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_SMON RPU_SMON
 * @brief    SMON RPU Register.
 * @{
 */
#define MXC_F_RPU_SMON_ACCESS_POS                      0 /**< SMON_ACCESS Position */
#define MXC_F_RPU_SMON_ACCESS                          ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_SMON_ACCESS_POS)) /**< SMON_ACCESS Mask */

/**@} end of group RPU_SMON_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_SIMO RPU_SIMO
 * @brief    SIMO RPU Register.
 * @{
 */
#define MXC_F_RPU_SIMO_ACCESS_POS                      0 /**< SIMO_ACCESS Position */
#define MXC_F_RPU_SIMO_ACCESS                          ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_SIMO_ACCESS_POS)) /**< SIMO_ACCESS Mask */

/**@} end of group RPU_SIMO_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_DVS RPU_DVS
 * @brief    DVS RPU Register.
 * @{
 */
#define MXC_F_RPU_DVS_ACCESS_POS                       0 /**< DVS_ACCESS Position */
#define MXC_F_RPU_DVS_ACCESS                           ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_DVS_ACCESS_POS)) /**< DVS_ACCESS Mask */

/**@} end of group RPU_DVS_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_AES RPU_AES
 * @brief    AES RPU Register.
 * @{
 */
#define MXC_F_RPU_AES_ACCESS_POS                       0 /**< AES_ACCESS Position */
#define MXC_F_RPU_AES_ACCESS                           ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_AES_ACCESS_POS)) /**< AES_ACCESS Mask */

/**@} end of group RPU_AES_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_RTC RPU_RTC
 * @brief    RTC RPU Register.
 * @{
 */
#define MXC_F_RPU_RTC_ACCESS_POS                       0 /**< RTC_ACCESS Position */
#define MXC_F_RPU_RTC_ACCESS                           ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_RTC_ACCESS_POS)) /**< RTC_ACCESS Mask */

/**@} end of group RPU_RTC_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_WUT RPU_WUT
 * @brief    WUT RPU Register.
 * @{
 */
#define MXC_F_RPU_WUT_ACCESS_POS                       0 /**< WUT_ACCESS Position */
#define MXC_F_RPU_WUT_ACCESS                           ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_WUT_ACCESS_POS)) /**< WUT_ACCESS Mask */

/**@} end of group RPU_WUT_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_PWRSEQ RPU_PWRSEQ
 * @brief    PWRSEQ RPU Register.
 * @{
 */
#define MXC_F_RPU_PWRSEQ_ACCESS_POS                    0 /**< PWRSEQ_ACCESS Position */
#define MXC_F_RPU_PWRSEQ_ACCESS                        ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_PWRSEQ_ACCESS_POS)) /**< PWRSEQ_ACCESS Mask */

/**@} end of group RPU_PWRSEQ_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_MCR RPU_MCR
 * @brief    MCR RPU Register.
 * @{
 */
#define MXC_F_RPU_MCR_ACCESS_POS                       0 /**< MCR_ACCESS Position */
#define MXC_F_RPU_MCR_ACCESS                           ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_MCR_ACCESS_POS)) /**< MCR_ACCESS Mask */

/**@} end of group RPU_MCR_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_GPIO0 RPU_GPIO0
 * @brief    GPIO0 RPU Register.
 * @{
 */
#define MXC_F_RPU_GPIO0_ACCESS_POS                     0 /**< GPIO0_ACCESS Position */
#define MXC_F_RPU_GPIO0_ACCESS                         ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_GPIO0_ACCESS_POS)) /**< GPIO0_ACCESS Mask */

/**@} end of group RPU_GPIO0_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_GPIO1 RPU_GPIO1
 * @brief    GPIO1 RPU Register.
 * @{
 */
#define MXC_F_RPU_GPIO1_ACCESS_POS                     0 /**< GPIO1_ACCESS Position */
#define MXC_F_RPU_GPIO1_ACCESS                         ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_GPIO1_ACCESS_POS)) /**< GPIO1_ACCESS Mask */

/**@} end of group RPU_GPIO1_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_TMR0 RPU_TMR0
 * @brief    TMR0 RPU Register.
 * @{
 */
#define MXC_F_RPU_TMR0_ACCESS_POS                      0 /**< TMR0_ACCESS Position */
#define MXC_F_RPU_TMR0_ACCESS                          ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_TMR0_ACCESS_POS)) /**< TMR0_ACCESS Mask */

/**@} end of group RPU_TMR0_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_TMR1 RPU_TMR1
 * @brief    TMR1 RPU Register.
 * @{
 */
#define MXC_F_RPU_TMR1_ACCESS_POS                      0 /**< TMR1_ACCESS Position */
#define MXC_F_RPU_TMR1_ACCESS                          ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_TMR1_ACCESS_POS)) /**< TMR1_ACCESS Mask */

/**@} end of group RPU_TMR1_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_TMR2 RPU_TMR2
 * @brief    TMR2 RPU Register.
 * @{
 */
#define MXC_F_RPU_TMR2_ACCESS_POS                      0 /**< TMR2_ACCESS Position */
#define MXC_F_RPU_TMR2_ACCESS                          ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_TMR2_ACCESS_POS)) /**< TMR2_ACCESS Mask */

/**@} end of group RPU_TMR2_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_TMR3 RPU_TMR3
 * @brief    TMR3 RPU Register.
 * @{
 */
#define MXC_F_RPU_TMR3_ACCESS_POS                      0 /**< TMR3_ACCESS Position */
#define MXC_F_RPU_TMR3_ACCESS                          ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_TMR3_ACCESS_POS)) /**< TMR3_ACCESS Mask */

/**@} end of group RPU_TMR3_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_TMR4 RPU_TMR4
 * @brief    TMR4 RPU Register.
 * @{
 */
#define MXC_F_RPU_TMR4_ACCESS_POS                      0 /**< TMR4_ACCESS Position */
#define MXC_F_RPU_TMR4_ACCESS                          ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_TMR4_ACCESS_POS)) /**< TMR4_ACCESS Mask */

/**@} end of group RPU_TMR4_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_TMR5 RPU_TMR5
 * @brief    TMR5 RPU Register.
 * @{
 */
#define MXC_F_RPU_TMR5_ACCESS_POS                      0 /**< TMR5_ACCESS Position */
#define MXC_F_RPU_TMR5_ACCESS                          ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_TMR5_ACCESS_POS)) /**< TMR5_ACCESS Mask */

/**@} end of group RPU_TMR5_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_HTIMER0 RPU_HTIMER0
 * @brief    HTIMER0 RPU Register.
 * @{
 */
#define MXC_F_RPU_HTIMER0_ACCESS_POS                   0 /**< HTIMER0_ACCESS Position */
#define MXC_F_RPU_HTIMER0_ACCESS                       ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_HTIMER0_ACCESS_POS)) /**< HTIMER0_ACCESS Mask */

/**@} end of group RPU_HTIMER0_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_HTIMER1 RPU_HTIMER1
 * @brief    HTIMER1 RPU Register.
 * @{
 */
#define MXC_F_RPU_HTIMER1_ACCESS_POS                   0 /**< HTIMER1_ACCESS Position */
#define MXC_F_RPU_HTIMER1_ACCESS                       ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_HTIMER1_ACCESS_POS)) /**< HTIMER1_ACCESS Mask */

/**@} end of group RPU_HTIMER1_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_I2C0_BUS0 RPU_I2C0_BUS0
 * @brief    I2C0_BUS0 RPU Register.
 * @{
 */
#define MXC_F_RPU_I2C0_BUS0_ACCESS_POS                 0 /**< I2C0_BUS0_ACCESS Position */
#define MXC_F_RPU_I2C0_BUS0_ACCESS                     ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_I2C0_BUS0_ACCESS_POS)) /**< I2C0_BUS0_ACCESS Mask */

/**@} end of group RPU_I2C0_BUS0_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_I2C1_BUS0 RPU_I2C1_BUS0
 * @brief    I2C1_BUS0 RPU Register.
 * @{
 */
#define MXC_F_RPU_I2C1_BUS0_ACCESS_POS                 0 /**< I2C1_BUS0_ACCESS Position */
#define MXC_F_RPU_I2C1_BUS0_ACCESS                     ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_I2C1_BUS0_ACCESS_POS)) /**< I2C1_BUS0_ACCESS Mask */

/**@} end of group RPU_I2C1_BUS0_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_I2C2_BUS0 RPU_I2C2_BUS0
 * @brief    I2C2_BUS0 RPU Register.
 * @{
 */
#define MXC_F_RPU_I2C2_BUS0_ACCESS_POS                 0 /**< I2C2_BUS0_ACCESS Position */
#define MXC_F_RPU_I2C2_BUS0_ACCESS                     ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_I2C2_BUS0_ACCESS_POS)) /**< I2C2_BUS0_ACCESS Mask */

/**@} end of group RPU_I2C2_BUS0_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_SPIXFM RPU_SPIXFM
 * @brief    SPIXFM RPU Register.
 * @{
 */
#define MXC_F_RPU_SPIXFM_ACCESS_POS                    0 /**< SPIXFM_ACCESS Position */
#define MXC_F_RPU_SPIXFM_ACCESS                        ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_SPIXFM_ACCESS_POS)) /**< SPIXFM_ACCESS Mask */

/**@} end of group RPU_SPIXFM_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_SPIXFC RPU_SPIXFC
 * @brief    SPIXFC RPU Register.
 * @{
 */
#define MXC_F_RPU_SPIXFC_ACCESS_POS                    0 /**< SPIXFC_ACCESS Position */
#define MXC_F_RPU_SPIXFC_ACCESS                        ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_SPIXFC_ACCESS_POS)) /**< SPIXFC_ACCESS Mask */

/**@} end of group RPU_SPIXFC_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_DMA0 RPU_DMA0
 * @brief    DMA0 RPU Register.
 * @{
 */
#define MXC_F_RPU_DMA0_ACCESS_POS                      0 /**< DMA0_ACCESS Position */
#define MXC_F_RPU_DMA0_ACCESS                          ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_DMA0_ACCESS_POS)) /**< DMA0_ACCESS Mask */

/**@} end of group RPU_DMA0_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_FLC0 RPU_FLC0
 * @brief    FLC0 RPU Register.
 * @{
 */
#define MXC_F_RPU_FLC0_ACCESS_POS                      0 /**< FLC0_ACCESS Position */
#define MXC_F_RPU_FLC0_ACCESS                          ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_FLC0_ACCESS_POS)) /**< FLC0_ACCESS Mask */

/**@} end of group RPU_FLC0_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_FLC1 RPU_FLC1
 * @brief    FLC1 RPU Register.
 * @{
 */
#define MXC_F_RPU_FLC1_ACCESS_POS                      0 /**< FLC1_ACCESS Position */
#define MXC_F_RPU_FLC1_ACCESS                          ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_FLC1_ACCESS_POS)) /**< FLC1_ACCESS Mask */

/**@} end of group RPU_FLC1_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_ICC0 RPU_ICC0
 * @brief    ICC0 RPU Register.
 * @{
 */
#define MXC_F_RPU_ICC0_ACCESS_POS                      0 /**< ICC0_ACCESS Position */
#define MXC_F_RPU_ICC0_ACCESS                          ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_ICC0_ACCESS_POS)) /**< ICC0_ACCESS Mask */

/**@} end of group RPU_ICC0_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_ICC1 RPU_ICC1
 * @brief    ICC1 RPU Register.
 * @{
 */
#define MXC_F_RPU_ICC1_ACCESS_POS                      0 /**< ICC1_ACCESS Position */
#define MXC_F_RPU_ICC1_ACCESS                          ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_ICC1_ACCESS_POS)) /**< ICC1_ACCESS Mask */

/**@} end of group RPU_ICC1_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_SFCC RPU_SFCC
 * @brief    SFCC RPU Register.
 * @{
 */
#define MXC_F_RPU_SFCC_ACCESS_POS                      0 /**< SFCC_ACCESS Position */
#define MXC_F_RPU_SFCC_ACCESS                          ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_SFCC_ACCESS_POS)) /**< SFCC_ACCESS Mask */

/**@} end of group RPU_SFCC_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_SRCC RPU_SRCC
 * @brief    SRCC RPU Register.
 * @{
 */
#define MXC_F_RPU_SRCC_ACCESS_POS                      0 /**< SRCC_ACCESS Position */
#define MXC_F_RPU_SRCC_ACCESS                          ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_SRCC_ACCESS_POS)) /**< SRCC_ACCESS Mask */

/**@} end of group RPU_SRCC_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_ADC RPU_ADC
 * @brief    ADC RPU Register.
 * @{
 */
#define MXC_F_RPU_ADC_ACCESS_POS                       0 /**< ADC_ACCESS Position */
#define MXC_F_RPU_ADC_ACCESS                           ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_ADC_ACCESS_POS)) /**< ADC_ACCESS Mask */

/**@} end of group RPU_ADC_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_DMA1 RPU_DMA1
 * @brief    DMA1 RPU Register.
 * @{
 */
#define MXC_F_RPU_DMA1_ACCESS_POS                      0 /**< DMA1_ACCESS Position */
#define MXC_F_RPU_DMA1_ACCESS                          ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_DMA1_ACCESS_POS)) /**< DMA1_ACCESS Mask */

/**@} end of group RPU_DMA1_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_SDMA RPU_SDMA
 * @brief    SDMA RPU Register.
 * @{
 */
#define MXC_F_RPU_SDMA_ACCESS_POS                      0 /**< SDMA_ACCESS Position */
#define MXC_F_RPU_SDMA_ACCESS                          ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_SDMA_ACCESS_POS)) /**< SDMA_ACCESS Mask */

/**@} end of group RPU_SDMA_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_SDHCCTRL RPU_SDHCCTRL
 * @brief    SD Host Controller (APB).
 * @{
 */
#define MXC_F_RPU_SDHCCTRL_ACCESS_POS                  0 /**< SDHCCTRL_ACCESS Position */
#define MXC_F_RPU_SDHCCTRL_ACCESS                      ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_SDHCCTRL_ACCESS_POS)) /**< SDHCCTRL_ACCESS Mask */

/**@} end of group RPU_SDHCCTRL_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_SPIXR RPU_SPIXR
 * @brief    SPIXR RPU Register.
 * @{
 */
#define MXC_F_RPU_SPIXR_ACCESS_POS                     0 /**< SPIXR_ACCESS Position */
#define MXC_F_RPU_SPIXR_ACCESS                         ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_SPIXR_ACCESS_POS)) /**< SPIXR_ACCESS Mask */

/**@} end of group RPU_SPIXR_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_PTG_BUS0 RPU_PTG_BUS0
 * @brief    PTG_BUS0 RPU Register.
 * @{
 */
#define MXC_F_RPU_PTG_BUS0_ACCESS_POS                  0 /**< PTG_BUS0_ACCESS Position */
#define MXC_F_RPU_PTG_BUS0_ACCESS                      ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_PTG_BUS0_ACCESS_POS)) /**< PTG_BUS0_ACCESS Mask */

/**@} end of group RPU_PTG_BUS0_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_OWM RPU_OWM
 * @brief    OWM RPU Register.
 * @{
 */
#define MXC_F_RPU_OWM_ACCESS_POS                       0 /**< OWM_ACCESS Position */
#define MXC_F_RPU_OWM_ACCESS                           ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_OWM_ACCESS_POS)) /**< OWM_ACCESS Mask */

/**@} end of group RPU_OWM_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_SEMA RPU_SEMA
 * @brief    SEMA RPU Register.
 * @{
 */
#define MXC_F_RPU_SEMA_ACCESS_POS                      0 /**< SEMA_ACCESS Position */
#define MXC_F_RPU_SEMA_ACCESS                          ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_SEMA_ACCESS_POS)) /**< SEMA_ACCESS Mask */

/**@} end of group RPU_SEMA_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_UART0 RPU_UART0
 * @brief    UART0 RPU Register.
 * @{
 */
#define MXC_F_RPU_UART0_ACCESS_POS                     0 /**< UART0_ACCESS Position */
#define MXC_F_RPU_UART0_ACCESS                         ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_UART0_ACCESS_POS)) /**< UART0_ACCESS Mask */

/**@} end of group RPU_UART0_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_UART1 RPU_UART1
 * @brief    UART1 RPU Register.
 * @{
 */
#define MXC_F_RPU_UART1_ACCESS_POS                     0 /**< UART1_ACCESS Position */
#define MXC_F_RPU_UART1_ACCESS                         ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_UART1_ACCESS_POS)) /**< UART1_ACCESS Mask */

/**@} end of group RPU_UART1_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_UART2 RPU_UART2
 * @brief    UART2 RPU Register.
 * @{
 */
#define MXC_F_RPU_UART2_ACCESS_POS                     0 /**< UART2_ACCESS Position */
#define MXC_F_RPU_UART2_ACCESS                         ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_UART2_ACCESS_POS)) /**< UART2_ACCESS Mask */

/**@} end of group RPU_UART2_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_SPI1 RPU_SPI1
 * @brief    SPI1 RPU Register.
 * @{
 */
#define MXC_F_RPU_SPI1_ACCESS_POS                      0 /**< SPI1_ACCESS Position */
#define MXC_F_RPU_SPI1_ACCESS                          ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_SPI1_ACCESS_POS)) /**< SPI1_ACCESS Mask */

/**@} end of group RPU_SPI1_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_SPI2 RPU_SPI2
 * @brief    SPI2 RPU Register.
 * @{
 */
#define MXC_F_RPU_SPI2_ACCESS_POS                      0 /**< SPI2_ACCESS Position */
#define MXC_F_RPU_SPI2_ACCESS                          ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_SPI2_ACCESS_POS)) /**< SPI2_ACCESS Mask */

/**@} end of group RPU_SPI2_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_AUDIO RPU_AUDIO
 * @brief    AUDIO RPU Register.
 * @{
 */
#define MXC_F_RPU_AUDIO_ACCESS_POS                     0 /**< AUDIO_ACCESS Position */
#define MXC_F_RPU_AUDIO_ACCESS                         ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_AUDIO_ACCESS_POS)) /**< AUDIO_ACCESS Mask */

/**@} end of group RPU_AUDIO_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_TRNG RPU_TRNG
 * @brief    TRNG RPU Register.
 * @{
 */
#define MXC_F_RPU_TRNG_ACCESS_POS                      0 /**< TRNG_ACCESS Position */
#define MXC_F_RPU_TRNG_ACCESS                          ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_TRNG_ACCESS_POS)) /**< TRNG_ACCESS Mask */

/**@} end of group RPU_TRNG_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_BTLE RPU_BTLE
 * @brief    BTLE RPU Register.
 * @{
 */
#define MXC_F_RPU_BTLE_ACCESS_POS                      0 /**< BTLE_ACCESS Position */
#define MXC_F_RPU_BTLE_ACCESS                          ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_BTLE_ACCESS_POS)) /**< BTLE_ACCESS Mask */

/**@} end of group RPU_BTLE_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_USBHS RPU_USBHS
 * @brief    USBHS RPU Register.
 * @{
 */
#define MXC_F_RPU_USBHS_ACCESS_POS                     0 /**< USBHS_ACCESS Position */
#define MXC_F_RPU_USBHS_ACCESS                         ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_USBHS_ACCESS_POS)) /**< USBHS_ACCESS Mask */

/**@} end of group RPU_USBHS_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_SDIO RPU_SDIO
 * @brief    SDIO/SDHC Target RPU Register.
 * @{
 */
#define MXC_F_RPU_SDIO_ACCESS_POS                      0 /**< SDIO_ACCESS Position */
#define MXC_F_RPU_SDIO_ACCESS                          ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_SDIO_ACCESS_POS)) /**< SDIO_ACCESS Mask */

/**@} end of group RPU_SDIO_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_SPIXFM_FIFO RPU_SPIXFM_FIFO
 * @brief    SPIXFM_FIFO RPU Register.
 * @{
 */
#define MXC_F_RPU_SPIXFM_FIFO_ACCESS_POS               0 /**< SPIXFM_FIFO_ACCESS Position */
#define MXC_F_RPU_SPIXFM_FIFO_ACCESS                   ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_SPIXFM_FIFO_ACCESS_POS)) /**< SPIXFM_FIFO_ACCESS Mask */

/**@} end of group RPU_SPIXFM_FIFO_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_SPI0 RPU_SPI0
 * @brief    SPI0 RPU Register.
 * @{
 */
#define MXC_F_RPU_SPI0_ACCESS_POS                      0 /**< SPI0_ACCESS Position */
#define MXC_F_RPU_SPI0_ACCESS                          ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_SPI0_ACCESS_POS)) /**< SPI0_ACCESS Mask */

/**@} end of group RPU_SPI0_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_SYSRAM0 RPU_SYSRAM0
 * @brief    SYSRAM0 RPU Register.
 * @{
 */
#define MXC_F_RPU_SYSRAM0_ACCESS_POS                   0 /**< SYSRAM0_ACCESS Position */
#define MXC_F_RPU_SYSRAM0_ACCESS                       ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_SYSRAM0_ACCESS_POS)) /**< SYSRAM0_ACCESS Mask */

/**@} end of group RPU_SYSRAM0_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_SYSRAM1 RPU_SYSRAM1
 * @brief    SYSRAM1 RPU Register.
 * @{
 */
#define MXC_F_RPU_SYSRAM1_ACCESS_POS                   0 /**< SYSRAM1_ACCESS Position */
#define MXC_F_RPU_SYSRAM1_ACCESS                       ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_SYSRAM1_ACCESS_POS)) /**< SYSRAM1_ACCESS Mask */

/**@} end of group RPU_SYSRAM1_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_SYSRAM2 RPU_SYSRAM2
 * @brief    SYSRAM2 RPU Register.
 * @{
 */
#define MXC_F_RPU_SYSRAM2_ACCESS_POS                   0 /**< SYSRAM2_ACCESS Position */
#define MXC_F_RPU_SYSRAM2_ACCESS                       ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_SYSRAM2_ACCESS_POS)) /**< SYSRAM2_ACCESS Mask */

/**@} end of group RPU_SYSRAM2_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_SYSRAM3 RPU_SYSRAM3
 * @brief    SYSRAM3 RPU Register.
 * @{
 */
#define MXC_F_RPU_SYSRAM3_ACCESS_POS                   0 /**< SYSRAM3_ACCESS Position */
#define MXC_F_RPU_SYSRAM3_ACCESS                       ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_SYSRAM3_ACCESS_POS)) /**< SYSRAM3_ACCESS Mask */

/**@} end of group RPU_SYSRAM3_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_SYSRAM4 RPU_SYSRAM4
 * @brief    SYSRAM4 RPU Register.
 * @{
 */
#define MXC_F_RPU_SYSRAM4_ACCESS_POS                   0 /**< SYSRAM4_ACCESS Position */
#define MXC_F_RPU_SYSRAM4_ACCESS                       ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_SYSRAM4_ACCESS_POS)) /**< SYSRAM4_ACCESS Mask */

/**@} end of group RPU_SYSRAM4_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_SYSRAM5 RPU_SYSRAM5
 * @brief    SYSRAM5 RPU Register.
 * @{
 */
#define MXC_F_RPU_SYSRAM5_ACCESS_POS                   0 /**< SYSRAM5_ACCESS Position */
#define MXC_F_RPU_SYSRAM5_ACCESS                       ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_SYSRAM5_ACCESS_POS)) /**< SYSRAM5_ACCESS Mask */

/**@} end of group RPU_SYSRAM5_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_SYSRAM6_11 RPU_SYSRAM6_11
 * @brief    SYSRAM6-11 RPU Register.
 * @{
 */
#define MXC_F_RPU_SYSRAM6_11_ACCESS_POS                0 /**< SYSRAM6_11_ACCESS Position */
#define MXC_F_RPU_SYSRAM6_11_ACCESS                    ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_SYSRAM6_11_ACCESS_POS)) /**< SYSRAM6_11_ACCESS Mask */

/**@} end of group RPU_SYSRAM6_11_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_I2C0_BUS1 RPU_I2C0_BUS1
 * @brief    I2C0_BUS1 RPU Register.
 * @{
 */
#define MXC_F_RPU_I2C0_BUS1_ACCESS_POS                 0 /**< I2C0_BUS1_ACCESS Position */
#define MXC_F_RPU_I2C0_BUS1_ACCESS                     ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_I2C0_BUS1_ACCESS_POS)) /**< I2C0_BUS1_ACCESS Mask */

/**@} end of group RPU_I2C0_BUS1_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_I2C1_BUS1 RPU_I2C1_BUS1
 * @brief    I2C1_BUS1 RPU Register.
 * @{
 */
#define MXC_F_RPU_I2C1_BUS1_ACCESS_POS                 0 /**< I2C1_BUS1_ACCESS Position */
#define MXC_F_RPU_I2C1_BUS1_ACCESS                     ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_I2C1_BUS1_ACCESS_POS)) /**< I2C1_BUS1_ACCESS Mask */

/**@} end of group RPU_I2C1_BUS1_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_I2C2_BUS1 RPU_I2C2_BUS1
 * @brief    I2C2_BU1 RPU Register.
 * @{
 */
#define MXC_F_RPU_I2C2_BUS1_ACCESS_POS                 0 /**< I2C2_BUS1_ACCESS Position */
#define MXC_F_RPU_I2C2_BUS1_ACCESS                     ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_I2C2_BUS1_ACCESS_POS)) /**< I2C2_BUS1_ACCESS Mask */

/**@} end of group RPU_I2C2_BUS1_Register */

/**
 * @ingroup  rpu_registers
 * @defgroup RPU_PTG_BUS1 RPU_PTG_BUS1
 * @brief    PTG_BUS1  RPU Register.
 * @{
 */
#define MXC_F_RPU_PTG_BUS1_ACCESS_POS                  0 /**< PTG_BUS1_ACCESS Position */
#define MXC_F_RPU_PTG_BUS1_ACCESS                      ((uint32_t)(0xFFFFFFFFUL << MXC_F_RPU_PTG_BUS1_ACCESS_POS)) /**< PTG_BUS1_ACCESS Mask */

/**@} end of group RPU_PTG_BUS1_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32665_INCLUDE_RPU_REGS_H_
