/**
 * @file    i3c_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the I3C Peripheral Module.
 * @note    This file is @generated.
 * @ingroup i3c_registers
 */

/******************************************************************************
 *
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_I3C_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_I3C_REGS_H_

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
 * @ingroup     i3c
 * @defgroup    i3c_registers I3C_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the I3C Peripheral Module.
 * @details     I3C Registers
 */

/**
 * @ingroup i3c_registers
 * Structure type to access the I3C Registers.
 */
typedef struct {
    __IO uint32_t mconfig;              /**< <tt>\b 0x000:</tt> I3C MCONFIG Register */
    __IO uint32_t config;               /**< <tt>\b 0x004:</tt> I3C CONFIG Register */
    __IO uint32_t status;               /**< <tt>\b 0x008:</tt> I3C STATUS Register */
    __IO uint32_t ctrl;                 /**< <tt>\b 0x00C:</tt> I3C CTRL Register */
    __IO uint32_t intset;               /**< <tt>\b 0x010:</tt> I3C INTSET Register */
    __O  uint32_t intclr;               /**< <tt>\b 0x014:</tt> I3C INTCLR Register */
    __I  uint32_t intmasked;            /**< <tt>\b 0x018:</tt> I3C INTMASKED Register */
    __IO uint32_t errwarn;              /**< <tt>\b 0x01C:</tt> I3C ERRWARN Register */
    __IO uint32_t dmactrl;              /**< <tt>\b 0x020:</tt> I3C DMACTRL Register */
    __IO uint32_t hdrbtcfg;             /**< <tt>\b 0x024:</tt> I3C HDRBTCFG Register */
    __I  uint32_t hdrbtlast;            /**< <tt>\b 0x028:</tt> I3C HDRBTLAST Register */
    __IO uint32_t datactrl;             /**< <tt>\b 0x02C:</tt> I3C DATACTRL Register */
    __O  uint32_t wdatab;               /**< <tt>\b 0x030:</tt> I3C WDATAB Register */
    __O  uint32_t wdatabe;              /**< <tt>\b 0x034:</tt> I3C WDATABE Register */
    __O  uint32_t wdatah;               /**< <tt>\b 0x038:</tt> I3C WDATAH Register */
    __O  uint32_t wdatahe;              /**< <tt>\b 0x03C:</tt> I3C WDATAHE Register */
    __I  uint32_t rdatab;               /**< <tt>\b 0x040:</tt> I3C RDATAB Register */
    __R  uint32_t rsv_0x44;
    __I  uint32_t rdatah;               /**< <tt>\b 0x048:</tt> I3C RDATAH Register */
    __R  uint32_t rsv_0x4c_0x53[2];
    __O  uint32_t wdatab1;              /**< <tt>\b 0x054:</tt> I3C WDATAB1 Register */
    __R  uint32_t rsv_0x58;
    __I  uint32_t capabilities2;        /**< <tt>\b 0x05C:</tt> I3C CAPABILITIES2 Register */
    __I  uint32_t capabilities;         /**< <tt>\b 0x060:</tt> I3C CAPABILITIES Register */
    __I  uint32_t dynaddr;              /**< <tt>\b 0x064:</tt> I3C DYNADDR Register */
    __IO uint32_t maxlimits;            /**< <tt>\b 0x068:</tt> I3C MAXLIMITS Register */
    __IO uint32_t partno;               /**< <tt>\b 0x06C:</tt> I3C PARTNO Register */
    __IO uint32_t idext;                /**< <tt>\b 0x070:</tt> I3C IDEXT Register */
    __IO uint32_t vendorid;             /**< <tt>\b 0x074:</tt> I3C VENDORID Register */
    __IO uint32_t tcclock;              /**< <tt>\b 0x078:</tt> I3C TCCLOCK Register */
    __I  uint32_t msglast;              /**< <tt>\b 0x07C:</tt> I3C MSGLAST Register */
    __R  uint32_t rsv_0x80;
    __IO uint32_t mctrl;                /**< <tt>\b 0x084:</tt> I3C MCTRL Register */
    __IO uint32_t mstatus;              /**< <tt>\b 0x088:</tt> I3C MSTATUS Register */
    __IO uint32_t ibirules;             /**< <tt>\b 0x08C:</tt> I3C IBIRULES Register */
    __IO uint32_t mintset;              /**< <tt>\b 0x090:</tt> I3C MINTSET Register */
    __O  uint32_t mintclr;              /**< <tt>\b 0x094:</tt> I3C MINTCLR Register */
    __I  uint32_t mintmasked;           /**< <tt>\b 0x098:</tt> I3C MINTMASKED Register */
    __IO uint32_t merrwarn;             /**< <tt>\b 0x09C:</tt> I3C MERRWARN Register */
    __IO uint32_t mdmactrl;             /**< <tt>\b 0x0A0:</tt> I3C MDMACTRL Register */
    __IO uint32_t mhdrbtcfg;            /**< <tt>\b 0x0A4:</tt> I3C MHDRBTCFG Register */
    __IO uint32_t mhdrbtlast;           /**< <tt>\b 0x0A8:</tt> I3C MHDRBTLAST Register */
    __IO uint32_t mdatactrl;            /**< <tt>\b 0x0AC:</tt> I3C MDATACTRL Register */
    __O  uint32_t mwdatab;              /**< <tt>\b 0x0B0:</tt> I3C MWDATAB Register */
    __O  uint32_t mwdatabe;             /**< <tt>\b 0x0B4:</tt> I3C MWDATABE Register */
    __O  uint32_t mwdatah;              /**< <tt>\b 0x0B8:</tt> I3C MWDATAH Register */
    __O  uint32_t mwdatahe;             /**< <tt>\b 0x0BC:</tt> I3C MWDATAHE Register */
    __I  uint32_t mrdatab;              /**< <tt>\b 0x0C0:</tt> I3C MRDATAB Register */
    __R  uint32_t rsv_0xc4;
    __I  uint32_t mrdatah;              /**< <tt>\b 0x0C8:</tt> I3C MRDATAH Register */
    __O  uint32_t mwdatab1;             /**< <tt>\b 0x0CC:</tt> I3C MWDATAB1 Register */
    __O  uint32_t mwmsg_sdr;            /**< <tt>\b 0x0D0:</tt> I3C MWMSG_SDR Register */
    __I  uint32_t mrmsg_sdr;            /**< <tt>\b 0x0D4:</tt> I3C MRMSG_SDR Register */
    __O  uint32_t mwmsg_ddr;            /**< <tt>\b 0x0D8:</tt> I3C MWMSG_DDR Register */
    __I  uint32_t mrmsg_ddr;            /**< <tt>\b 0x0DC:</tt> I3C MRMSG_DDR Register */
    __R  uint32_t rsv_0xe0;
    __IO uint32_t mdynaddr;             /**< <tt>\b 0x0E4:</tt> I3C MDYNADDR Register */
    __R  uint32_t rsv_0xe8_0xef[2];
    __O  uint32_t mwdataw;              /**< <tt>\b 0x0F0:</tt> I3C MWDATAW Register */
    __R  uint32_t rsv_0xf4;
    __I  uint32_t mrdataw;              /**< <tt>\b 0x0F8:</tt> I3C MRDATAW Register */
    __R  uint32_t rsv_0xfc;
    __IO uint32_t rstacttime;           /**< <tt>\b 0x100:</tt> I3C RSTACTTIME Register */
    __R  uint32_t rsv_0x104;
    __I  uint32_t hdrcmd;               /**< <tt>\b 0x108:</tt> I3C HDRCMD Register */
    __R  uint32_t rsv_0x10c_0x113[2];
    __I  uint32_t groupdef;             /**< <tt>\b 0x114:</tt> I3C GROUPDEF Register */
    __R  uint32_t rsv_0x118;
    __I  uint32_t mapctrl0;             /**< <tt>\b 0x11C:</tt> I3C MAPCTRL0 Register */
    __IO uint32_t mapctrln[8];          /**< <tt>\b 0x120:</tt> I3C MAPCTRLN Register */
    __IO uint32_t ibiext1;              /**< <tt>\b 0x140:</tt> I3C IBIEXT1 Register */
    __IO uint32_t ibiext2;              /**< <tt>\b 0x144:</tt> I3C IBIEXT2 Register */
    __R  uint32_t rsv_0x148_0x14f[2];
    __O  uint32_t wdataw;               /**< <tt>\b 0x150:</tt> I3C WDATAW Register */
    __R  uint32_t rsv_0x154;
    __I  uint32_t rdataw;               /**< <tt>\b 0x158:</tt> I3C RDATAW Register */
    __R  uint32_t rsv_0x15c_0xffb[936];
    __IO uint32_t id;                   /**< <tt>\b 0xFFC:</tt> I3C ID Register */
} mxc_i3c_regs_t;

/* Register offsets for module I3C */
/**
 * @ingroup    i3c_registers
 * @defgroup   I3C_Register_Offsets Register Offsets
 * @brief      I3C Peripheral Register Offsets from the I3C Base Peripheral Address.
 * @{
 */
#define MXC_R_I3C_MCONFIG                  ((uint32_t)0x00000000UL) /**< Offset from I3C Base Address: <tt> 0x0000</tt> */
#define MXC_R_I3C_CONFIG                   ((uint32_t)0x00000004UL) /**< Offset from I3C Base Address: <tt> 0x0004</tt> */
#define MXC_R_I3C_STATUS                   ((uint32_t)0x00000008UL) /**< Offset from I3C Base Address: <tt> 0x0008</tt> */
#define MXC_R_I3C_CTRL                     ((uint32_t)0x0000000CUL) /**< Offset from I3C Base Address: <tt> 0x000C</tt> */
#define MXC_R_I3C_INTSET                   ((uint32_t)0x00000010UL) /**< Offset from I3C Base Address: <tt> 0x0010</tt> */
#define MXC_R_I3C_INTCLR                   ((uint32_t)0x00000014UL) /**< Offset from I3C Base Address: <tt> 0x0014</tt> */
#define MXC_R_I3C_INTMASKED                ((uint32_t)0x00000018UL) /**< Offset from I3C Base Address: <tt> 0x0018</tt> */
#define MXC_R_I3C_ERRWARN                  ((uint32_t)0x0000001CUL) /**< Offset from I3C Base Address: <tt> 0x001C</tt> */
#define MXC_R_I3C_DMACTRL                  ((uint32_t)0x00000020UL) /**< Offset from I3C Base Address: <tt> 0x0020</tt> */
#define MXC_R_I3C_HDRBTCFG                 ((uint32_t)0x00000024UL) /**< Offset from I3C Base Address: <tt> 0x0024</tt> */
#define MXC_R_I3C_HDRBTLAST                ((uint32_t)0x00000028UL) /**< Offset from I3C Base Address: <tt> 0x0028</tt> */
#define MXC_R_I3C_DATACTRL                 ((uint32_t)0x0000002CUL) /**< Offset from I3C Base Address: <tt> 0x002C</tt> */
#define MXC_R_I3C_WDATAB                   ((uint32_t)0x00000030UL) /**< Offset from I3C Base Address: <tt> 0x0030</tt> */
#define MXC_R_I3C_WDATABE                  ((uint32_t)0x00000034UL) /**< Offset from I3C Base Address: <tt> 0x0034</tt> */
#define MXC_R_I3C_WDATAH                   ((uint32_t)0x00000038UL) /**< Offset from I3C Base Address: <tt> 0x0038</tt> */
#define MXC_R_I3C_WDATAHE                  ((uint32_t)0x0000003CUL) /**< Offset from I3C Base Address: <tt> 0x003C</tt> */
#define MXC_R_I3C_RDATAB                   ((uint32_t)0x00000040UL) /**< Offset from I3C Base Address: <tt> 0x0040</tt> */
#define MXC_R_I3C_RDATAH                   ((uint32_t)0x00000048UL) /**< Offset from I3C Base Address: <tt> 0x0048</tt> */
#define MXC_R_I3C_WDATAB1                  ((uint32_t)0x00000054UL) /**< Offset from I3C Base Address: <tt> 0x0054</tt> */
#define MXC_R_I3C_CAPABILITIES2            ((uint32_t)0x0000005CUL) /**< Offset from I3C Base Address: <tt> 0x005C</tt> */
#define MXC_R_I3C_CAPABILITIES             ((uint32_t)0x00000060UL) /**< Offset from I3C Base Address: <tt> 0x0060</tt> */
#define MXC_R_I3C_DYNADDR                  ((uint32_t)0x00000064UL) /**< Offset from I3C Base Address: <tt> 0x0064</tt> */
#define MXC_R_I3C_MAXLIMITS                ((uint32_t)0x00000068UL) /**< Offset from I3C Base Address: <tt> 0x0068</tt> */
#define MXC_R_I3C_PARTNO                   ((uint32_t)0x0000006CUL) /**< Offset from I3C Base Address: <tt> 0x006C</tt> */
#define MXC_R_I3C_IDEXT                    ((uint32_t)0x00000070UL) /**< Offset from I3C Base Address: <tt> 0x0070</tt> */
#define MXC_R_I3C_VENDORID                 ((uint32_t)0x00000074UL) /**< Offset from I3C Base Address: <tt> 0x0074</tt> */
#define MXC_R_I3C_TCCLOCK                  ((uint32_t)0x00000078UL) /**< Offset from I3C Base Address: <tt> 0x0078</tt> */
#define MXC_R_I3C_MSGLAST                  ((uint32_t)0x0000007CUL) /**< Offset from I3C Base Address: <tt> 0x007C</tt> */
#define MXC_R_I3C_MCTRL                    ((uint32_t)0x00000084UL) /**< Offset from I3C Base Address: <tt> 0x0084</tt> */
#define MXC_R_I3C_MSTATUS                  ((uint32_t)0x00000088UL) /**< Offset from I3C Base Address: <tt> 0x0088</tt> */
#define MXC_R_I3C_IBIRULES                 ((uint32_t)0x0000008CUL) /**< Offset from I3C Base Address: <tt> 0x008C</tt> */
#define MXC_R_I3C_MINTSET                  ((uint32_t)0x00000090UL) /**< Offset from I3C Base Address: <tt> 0x0090</tt> */
#define MXC_R_I3C_MINTCLR                  ((uint32_t)0x00000094UL) /**< Offset from I3C Base Address: <tt> 0x0094</tt> */
#define MXC_R_I3C_MINTMASKED               ((uint32_t)0x00000098UL) /**< Offset from I3C Base Address: <tt> 0x0098</tt> */
#define MXC_R_I3C_MERRWARN                 ((uint32_t)0x0000009CUL) /**< Offset from I3C Base Address: <tt> 0x009C</tt> */
#define MXC_R_I3C_MDMACTRL                 ((uint32_t)0x000000A0UL) /**< Offset from I3C Base Address: <tt> 0x00A0</tt> */
#define MXC_R_I3C_MHDRBTCFG                ((uint32_t)0x000000A4UL) /**< Offset from I3C Base Address: <tt> 0x00A4</tt> */
#define MXC_R_I3C_MHDRBTLAST               ((uint32_t)0x000000A8UL) /**< Offset from I3C Base Address: <tt> 0x00A8</tt> */
#define MXC_R_I3C_MDATACTRL                ((uint32_t)0x000000ACUL) /**< Offset from I3C Base Address: <tt> 0x00AC</tt> */
#define MXC_R_I3C_MWDATAB                  ((uint32_t)0x000000B0UL) /**< Offset from I3C Base Address: <tt> 0x00B0</tt> */
#define MXC_R_I3C_MWDATABE                 ((uint32_t)0x000000B4UL) /**< Offset from I3C Base Address: <tt> 0x00B4</tt> */
#define MXC_R_I3C_MWDATAH                  ((uint32_t)0x000000B8UL) /**< Offset from I3C Base Address: <tt> 0x00B8</tt> */
#define MXC_R_I3C_MWDATAHE                 ((uint32_t)0x000000BCUL) /**< Offset from I3C Base Address: <tt> 0x00BC</tt> */
#define MXC_R_I3C_MRDATAB                  ((uint32_t)0x000000C0UL) /**< Offset from I3C Base Address: <tt> 0x00C0</tt> */
#define MXC_R_I3C_MRDATAH                  ((uint32_t)0x000000C8UL) /**< Offset from I3C Base Address: <tt> 0x00C8</tt> */
#define MXC_R_I3C_MWDATAB1                 ((uint32_t)0x000000CCUL) /**< Offset from I3C Base Address: <tt> 0x00CC</tt> */
#define MXC_R_I3C_MWMSG_SDR                ((uint32_t)0x000000D0UL) /**< Offset from I3C Base Address: <tt> 0x00D0</tt> */
#define MXC_R_I3C_MRMSG_SDR                ((uint32_t)0x000000D4UL) /**< Offset from I3C Base Address: <tt> 0x00D4</tt> */
#define MXC_R_I3C_MWMSG_DDR                ((uint32_t)0x000000D8UL) /**< Offset from I3C Base Address: <tt> 0x00D8</tt> */
#define MXC_R_I3C_MRMSG_DDR                ((uint32_t)0x000000DCUL) /**< Offset from I3C Base Address: <tt> 0x00DC</tt> */
#define MXC_R_I3C_MDYNADDR                 ((uint32_t)0x000000E4UL) /**< Offset from I3C Base Address: <tt> 0x00E4</tt> */
#define MXC_R_I3C_MWDATAW                  ((uint32_t)0x000000F0UL) /**< Offset from I3C Base Address: <tt> 0x00F0</tt> */
#define MXC_R_I3C_MRDATAW                  ((uint32_t)0x000000F8UL) /**< Offset from I3C Base Address: <tt> 0x00F8</tt> */
#define MXC_R_I3C_RSTACTTIME               ((uint32_t)0x00000100UL) /**< Offset from I3C Base Address: <tt> 0x0100</tt> */
#define MXC_R_I3C_HDRCMD                   ((uint32_t)0x00000108UL) /**< Offset from I3C Base Address: <tt> 0x0108</tt> */
#define MXC_R_I3C_GROUPDEF                 ((uint32_t)0x00000114UL) /**< Offset from I3C Base Address: <tt> 0x0114</tt> */
#define MXC_R_I3C_MAPCTRL0                 ((uint32_t)0x0000011CUL) /**< Offset from I3C Base Address: <tt> 0x011C</tt> */
#define MXC_R_I3C_MAPCTRLN                 ((uint32_t)0x00000120UL) /**< Offset from I3C Base Address: <tt> 0x0120</tt> */
#define MXC_R_I3C_IBIEXT1                  ((uint32_t)0x00000140UL) /**< Offset from I3C Base Address: <tt> 0x0140</tt> */
#define MXC_R_I3C_IBIEXT2                  ((uint32_t)0x00000144UL) /**< Offset from I3C Base Address: <tt> 0x0144</tt> */
#define MXC_R_I3C_WDATAW                   ((uint32_t)0x00000150UL) /**< Offset from I3C Base Address: <tt> 0x0150</tt> */
#define MXC_R_I3C_RDATAW                   ((uint32_t)0x00000158UL) /**< Offset from I3C Base Address: <tt> 0x0158</tt> */
#define MXC_R_I3C_ID                       ((uint32_t)0x00000FFCUL) /**< Offset from I3C Base Address: <tt> 0x0FFC</tt> */
/**@} end of group i3c_registers */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_MCONFIG I3C_MCONFIG
 * @brief    Controller Configuration Register
 * @{
 */
#define MXC_F_I3C_MCONFIG_CTARENA_POS                  0 /**< MCONFIG_CTARENA Position */
#define MXC_F_I3C_MCONFIG_CTARENA                      ((uint32_t)(0x3UL << MXC_F_I3C_MCONFIG_CTARENA_POS)) /**< MCONFIG_CTARENA Mask */
#define MXC_V_I3C_MCONFIG_CTARENA_OFF                  ((uint32_t)0x0UL) /**< MCONFIG_CTARENA_OFF Value */
#define MXC_S_I3C_MCONFIG_CTARENA_OFF                  (MXC_V_I3C_MCONFIG_CTARENA_OFF << MXC_F_I3C_MCONFIG_CTARENA_POS) /**< MCONFIG_CTARENA_OFF Setting */
#define MXC_V_I3C_MCONFIG_CTARENA_ON                   ((uint32_t)0x1UL) /**< MCONFIG_CTARENA_ON Value */
#define MXC_S_I3C_MCONFIG_CTARENA_ON                   (MXC_V_I3C_MCONFIG_CTARENA_ON << MXC_F_I3C_MCONFIG_CTARENA_POS) /**< MCONFIG_CTARENA_ON Setting */
#define MXC_V_I3C_MCONFIG_CTARENA_CAPABLE              ((uint32_t)0x2UL) /**< MCONFIG_CTARENA_CAPABLE Value */
#define MXC_S_I3C_MCONFIG_CTARENA_CAPABLE              (MXC_V_I3C_MCONFIG_CTARENA_CAPABLE << MXC_F_I3C_MCONFIG_CTARENA_POS) /**< MCONFIG_CTARENA_CAPABLE Setting */

#define MXC_F_I3C_MCONFIG_DISTO_POS                    3 /**< MCONFIG_DISTO Position */
#define MXC_F_I3C_MCONFIG_DISTO                        ((uint32_t)(0x1UL << MXC_F_I3C_MCONFIG_DISTO_POS)) /**< MCONFIG_DISTO Mask */

#define MXC_F_I3C_MCONFIG_HKEEP_POS                    4 /**< MCONFIG_HKEEP Position */
#define MXC_F_I3C_MCONFIG_HKEEP                        ((uint32_t)(0x3UL << MXC_F_I3C_MCONFIG_HKEEP_POS)) /**< MCONFIG_HKEEP Mask */
#define MXC_V_I3C_MCONFIG_HKEEP_OFF                    ((uint32_t)0x0UL) /**< MCONFIG_HKEEP_OFF Value */
#define MXC_S_I3C_MCONFIG_HKEEP_OFF                    (MXC_V_I3C_MCONFIG_HKEEP_OFF << MXC_F_I3C_MCONFIG_HKEEP_POS) /**< MCONFIG_HKEEP_OFF Setting */
#define MXC_V_I3C_MCONFIG_HKEEP_ONCHIP_SCL_SDA         ((uint32_t)0x1UL) /**< MCONFIG_HKEEP_ONCHIP_SCL_SDA Value */
#define MXC_S_I3C_MCONFIG_HKEEP_ONCHIP_SCL_SDA         (MXC_V_I3C_MCONFIG_HKEEP_ONCHIP_SCL_SDA << MXC_F_I3C_MCONFIG_HKEEP_POS) /**< MCONFIG_HKEEP_ONCHIP_SCL_SDA Setting */
#define MXC_V_I3C_MCONFIG_HKEEP_EXTERNAL_SDA           ((uint32_t)0x2UL) /**< MCONFIG_HKEEP_EXTERNAL_SDA Value */
#define MXC_S_I3C_MCONFIG_HKEEP_EXTERNAL_SDA           (MXC_V_I3C_MCONFIG_HKEEP_EXTERNAL_SDA << MXC_F_I3C_MCONFIG_HKEEP_POS) /**< MCONFIG_HKEEP_EXTERNAL_SDA Setting */
#define MXC_V_I3C_MCONFIG_HKEEP_EXTERNAL_SCL_SDA       ((uint32_t)0x3UL) /**< MCONFIG_HKEEP_EXTERNAL_SCL_SDA Value */
#define MXC_S_I3C_MCONFIG_HKEEP_EXTERNAL_SCL_SDA       (MXC_V_I3C_MCONFIG_HKEEP_EXTERNAL_SCL_SDA << MXC_F_I3C_MCONFIG_HKEEP_POS) /**< MCONFIG_HKEEP_EXTERNAL_SCL_SDA Setting */

#define MXC_F_I3C_MCONFIG_ODSTOP_POS                   6 /**< MCONFIG_ODSTOP Position */
#define MXC_F_I3C_MCONFIG_ODSTOP                       ((uint32_t)(0x1UL << MXC_F_I3C_MCONFIG_ODSTOP_POS)) /**< MCONFIG_ODSTOP Mask */

#define MXC_F_I3C_MCONFIG_PPBAUD_POS                   8 /**< MCONFIG_PPBAUD Position */
#define MXC_F_I3C_MCONFIG_PPBAUD                       ((uint32_t)(0xFUL << MXC_F_I3C_MCONFIG_PPBAUD_POS)) /**< MCONFIG_PPBAUD Mask */
#define MXC_V_I3C_MCONFIG_PPBAUD_FCLK1                 ((uint32_t)0x0UL) /**< MCONFIG_PPBAUD_FCLK1 Value */
#define MXC_S_I3C_MCONFIG_PPBAUD_FCLK1                 (MXC_V_I3C_MCONFIG_PPBAUD_FCLK1 << MXC_F_I3C_MCONFIG_PPBAUD_POS) /**< MCONFIG_PPBAUD_FCLK1 Setting */
#define MXC_V_I3C_MCONFIG_PPBAUD_FCLK2                 ((uint32_t)0x1UL) /**< MCONFIG_PPBAUD_FCLK2 Value */
#define MXC_S_I3C_MCONFIG_PPBAUD_FCLK2                 (MXC_V_I3C_MCONFIG_PPBAUD_FCLK2 << MXC_F_I3C_MCONFIG_PPBAUD_POS) /**< MCONFIG_PPBAUD_FCLK2 Setting */
#define MXC_V_I3C_MCONFIG_PPBAUD_FCLK3                 ((uint32_t)0x2UL) /**< MCONFIG_PPBAUD_FCLK3 Value */
#define MXC_S_I3C_MCONFIG_PPBAUD_FCLK3                 (MXC_V_I3C_MCONFIG_PPBAUD_FCLK3 << MXC_F_I3C_MCONFIG_PPBAUD_POS) /**< MCONFIG_PPBAUD_FCLK3 Setting */
#define MXC_V_I3C_MCONFIG_PPBAUD_FCLK4                 ((uint32_t)0x3UL) /**< MCONFIG_PPBAUD_FCLK4 Value */
#define MXC_S_I3C_MCONFIG_PPBAUD_FCLK4                 (MXC_V_I3C_MCONFIG_PPBAUD_FCLK4 << MXC_F_I3C_MCONFIG_PPBAUD_POS) /**< MCONFIG_PPBAUD_FCLK4 Setting */
#define MXC_V_I3C_MCONFIG_PPBAUD_FCLK5                 ((uint32_t)0x4UL) /**< MCONFIG_PPBAUD_FCLK5 Value */
#define MXC_S_I3C_MCONFIG_PPBAUD_FCLK5                 (MXC_V_I3C_MCONFIG_PPBAUD_FCLK5 << MXC_F_I3C_MCONFIG_PPBAUD_POS) /**< MCONFIG_PPBAUD_FCLK5 Setting */
#define MXC_V_I3C_MCONFIG_PPBAUD_FCLK6                 ((uint32_t)0x5UL) /**< MCONFIG_PPBAUD_FCLK6 Value */
#define MXC_S_I3C_MCONFIG_PPBAUD_FCLK6                 (MXC_V_I3C_MCONFIG_PPBAUD_FCLK6 << MXC_F_I3C_MCONFIG_PPBAUD_POS) /**< MCONFIG_PPBAUD_FCLK6 Setting */
#define MXC_V_I3C_MCONFIG_PPBAUD_FCLK7                 ((uint32_t)0x6UL) /**< MCONFIG_PPBAUD_FCLK7 Value */
#define MXC_S_I3C_MCONFIG_PPBAUD_FCLK7                 (MXC_V_I3C_MCONFIG_PPBAUD_FCLK7 << MXC_F_I3C_MCONFIG_PPBAUD_POS) /**< MCONFIG_PPBAUD_FCLK7 Setting */
#define MXC_V_I3C_MCONFIG_PPBAUD_FCLK8                 ((uint32_t)0x7UL) /**< MCONFIG_PPBAUD_FCLK8 Value */
#define MXC_S_I3C_MCONFIG_PPBAUD_FCLK8                 (MXC_V_I3C_MCONFIG_PPBAUD_FCLK8 << MXC_F_I3C_MCONFIG_PPBAUD_POS) /**< MCONFIG_PPBAUD_FCLK8 Setting */
#define MXC_V_I3C_MCONFIG_PPBAUD_FCLK9                 ((uint32_t)0x8UL) /**< MCONFIG_PPBAUD_FCLK9 Value */
#define MXC_S_I3C_MCONFIG_PPBAUD_FCLK9                 (MXC_V_I3C_MCONFIG_PPBAUD_FCLK9 << MXC_F_I3C_MCONFIG_PPBAUD_POS) /**< MCONFIG_PPBAUD_FCLK9 Setting */
#define MXC_V_I3C_MCONFIG_PPBAUD_FCLK10                ((uint32_t)0x9UL) /**< MCONFIG_PPBAUD_FCLK10 Value */
#define MXC_S_I3C_MCONFIG_PPBAUD_FCLK10                (MXC_V_I3C_MCONFIG_PPBAUD_FCLK10 << MXC_F_I3C_MCONFIG_PPBAUD_POS) /**< MCONFIG_PPBAUD_FCLK10 Setting */
#define MXC_V_I3C_MCONFIG_PPBAUD_FCLK11                ((uint32_t)0xAUL) /**< MCONFIG_PPBAUD_FCLK11 Value */
#define MXC_S_I3C_MCONFIG_PPBAUD_FCLK11                (MXC_V_I3C_MCONFIG_PPBAUD_FCLK11 << MXC_F_I3C_MCONFIG_PPBAUD_POS) /**< MCONFIG_PPBAUD_FCLK11 Setting */
#define MXC_V_I3C_MCONFIG_PPBAUD_FCLK12                ((uint32_t)0xBUL) /**< MCONFIG_PPBAUD_FCLK12 Value */
#define MXC_S_I3C_MCONFIG_PPBAUD_FCLK12                (MXC_V_I3C_MCONFIG_PPBAUD_FCLK12 << MXC_F_I3C_MCONFIG_PPBAUD_POS) /**< MCONFIG_PPBAUD_FCLK12 Setting */
#define MXC_V_I3C_MCONFIG_PPBAUD_FCLK13                ((uint32_t)0xCUL) /**< MCONFIG_PPBAUD_FCLK13 Value */
#define MXC_S_I3C_MCONFIG_PPBAUD_FCLK13                (MXC_V_I3C_MCONFIG_PPBAUD_FCLK13 << MXC_F_I3C_MCONFIG_PPBAUD_POS) /**< MCONFIG_PPBAUD_FCLK13 Setting */
#define MXC_V_I3C_MCONFIG_PPBAUD_FCLK14                ((uint32_t)0xDUL) /**< MCONFIG_PPBAUD_FCLK14 Value */
#define MXC_S_I3C_MCONFIG_PPBAUD_FCLK14                (MXC_V_I3C_MCONFIG_PPBAUD_FCLK14 << MXC_F_I3C_MCONFIG_PPBAUD_POS) /**< MCONFIG_PPBAUD_FCLK14 Setting */
#define MXC_V_I3C_MCONFIG_PPBAUD_FCLK15                ((uint32_t)0xEUL) /**< MCONFIG_PPBAUD_FCLK15 Value */
#define MXC_S_I3C_MCONFIG_PPBAUD_FCLK15                (MXC_V_I3C_MCONFIG_PPBAUD_FCLK15 << MXC_F_I3C_MCONFIG_PPBAUD_POS) /**< MCONFIG_PPBAUD_FCLK15 Setting */
#define MXC_V_I3C_MCONFIG_PPBAUD_FCLK16                ((uint32_t)0xFUL) /**< MCONFIG_PPBAUD_FCLK16 Value */
#define MXC_S_I3C_MCONFIG_PPBAUD_FCLK16                (MXC_V_I3C_MCONFIG_PPBAUD_FCLK16 << MXC_F_I3C_MCONFIG_PPBAUD_POS) /**< MCONFIG_PPBAUD_FCLK16 Setting */

#define MXC_F_I3C_MCONFIG_PPLOW_POS                    12 /**< MCONFIG_PPLOW Position */
#define MXC_F_I3C_MCONFIG_PPLOW                        ((uint32_t)(0xFUL << MXC_F_I3C_MCONFIG_PPLOW_POS)) /**< MCONFIG_PPLOW Mask */
#define MXC_V_I3C_MCONFIG_PPLOW_FCLK0                  ((uint32_t)0x0UL) /**< MCONFIG_PPLOW_FCLK0 Value */
#define MXC_S_I3C_MCONFIG_PPLOW_FCLK0                  (MXC_V_I3C_MCONFIG_PPLOW_FCLK0 << MXC_F_I3C_MCONFIG_PPLOW_POS) /**< MCONFIG_PPLOW_FCLK0 Setting */
#define MXC_V_I3C_MCONFIG_PPLOW_FCLK1                  ((uint32_t)0x1UL) /**< MCONFIG_PPLOW_FCLK1 Value */
#define MXC_S_I3C_MCONFIG_PPLOW_FCLK1                  (MXC_V_I3C_MCONFIG_PPLOW_FCLK1 << MXC_F_I3C_MCONFIG_PPLOW_POS) /**< MCONFIG_PPLOW_FCLK1 Setting */
#define MXC_V_I3C_MCONFIG_PPLOW_FCLK2                  ((uint32_t)0x2UL) /**< MCONFIG_PPLOW_FCLK2 Value */
#define MXC_S_I3C_MCONFIG_PPLOW_FCLK2                  (MXC_V_I3C_MCONFIG_PPLOW_FCLK2 << MXC_F_I3C_MCONFIG_PPLOW_POS) /**< MCONFIG_PPLOW_FCLK2 Setting */
#define MXC_V_I3C_MCONFIG_PPLOW_FCLK3                  ((uint32_t)0x3UL) /**< MCONFIG_PPLOW_FCLK3 Value */
#define MXC_S_I3C_MCONFIG_PPLOW_FCLK3                  (MXC_V_I3C_MCONFIG_PPLOW_FCLK3 << MXC_F_I3C_MCONFIG_PPLOW_POS) /**< MCONFIG_PPLOW_FCLK3 Setting */
#define MXC_V_I3C_MCONFIG_PPLOW_FCLK4                  ((uint32_t)0x4UL) /**< MCONFIG_PPLOW_FCLK4 Value */
#define MXC_S_I3C_MCONFIG_PPLOW_FCLK4                  (MXC_V_I3C_MCONFIG_PPLOW_FCLK4 << MXC_F_I3C_MCONFIG_PPLOW_POS) /**< MCONFIG_PPLOW_FCLK4 Setting */
#define MXC_V_I3C_MCONFIG_PPLOW_FCLK5                  ((uint32_t)0x5UL) /**< MCONFIG_PPLOW_FCLK5 Value */
#define MXC_S_I3C_MCONFIG_PPLOW_FCLK5                  (MXC_V_I3C_MCONFIG_PPLOW_FCLK5 << MXC_F_I3C_MCONFIG_PPLOW_POS) /**< MCONFIG_PPLOW_FCLK5 Setting */
#define MXC_V_I3C_MCONFIG_PPLOW_FCLK6                  ((uint32_t)0x6UL) /**< MCONFIG_PPLOW_FCLK6 Value */
#define MXC_S_I3C_MCONFIG_PPLOW_FCLK6                  (MXC_V_I3C_MCONFIG_PPLOW_FCLK6 << MXC_F_I3C_MCONFIG_PPLOW_POS) /**< MCONFIG_PPLOW_FCLK6 Setting */
#define MXC_V_I3C_MCONFIG_PPLOW_FCLK7                  ((uint32_t)0x7UL) /**< MCONFIG_PPLOW_FCLK7 Value */
#define MXC_S_I3C_MCONFIG_PPLOW_FCLK7                  (MXC_V_I3C_MCONFIG_PPLOW_FCLK7 << MXC_F_I3C_MCONFIG_PPLOW_POS) /**< MCONFIG_PPLOW_FCLK7 Setting */
#define MXC_V_I3C_MCONFIG_PPLOW_FCLK8                  ((uint32_t)0x8UL) /**< MCONFIG_PPLOW_FCLK8 Value */
#define MXC_S_I3C_MCONFIG_PPLOW_FCLK8                  (MXC_V_I3C_MCONFIG_PPLOW_FCLK8 << MXC_F_I3C_MCONFIG_PPLOW_POS) /**< MCONFIG_PPLOW_FCLK8 Setting */
#define MXC_V_I3C_MCONFIG_PPLOW_FCLK9                  ((uint32_t)0x9UL) /**< MCONFIG_PPLOW_FCLK9 Value */
#define MXC_S_I3C_MCONFIG_PPLOW_FCLK9                  (MXC_V_I3C_MCONFIG_PPLOW_FCLK9 << MXC_F_I3C_MCONFIG_PPLOW_POS) /**< MCONFIG_PPLOW_FCLK9 Setting */
#define MXC_V_I3C_MCONFIG_PPLOW_FCLK10                 ((uint32_t)0xAUL) /**< MCONFIG_PPLOW_FCLK10 Value */
#define MXC_S_I3C_MCONFIG_PPLOW_FCLK10                 (MXC_V_I3C_MCONFIG_PPLOW_FCLK10 << MXC_F_I3C_MCONFIG_PPLOW_POS) /**< MCONFIG_PPLOW_FCLK10 Setting */
#define MXC_V_I3C_MCONFIG_PPLOW_FCLK11                 ((uint32_t)0xBUL) /**< MCONFIG_PPLOW_FCLK11 Value */
#define MXC_S_I3C_MCONFIG_PPLOW_FCLK11                 (MXC_V_I3C_MCONFIG_PPLOW_FCLK11 << MXC_F_I3C_MCONFIG_PPLOW_POS) /**< MCONFIG_PPLOW_FCLK11 Setting */
#define MXC_V_I3C_MCONFIG_PPLOW_FCLK12                 ((uint32_t)0xCUL) /**< MCONFIG_PPLOW_FCLK12 Value */
#define MXC_S_I3C_MCONFIG_PPLOW_FCLK12                 (MXC_V_I3C_MCONFIG_PPLOW_FCLK12 << MXC_F_I3C_MCONFIG_PPLOW_POS) /**< MCONFIG_PPLOW_FCLK12 Setting */
#define MXC_V_I3C_MCONFIG_PPLOW_FCLK13                 ((uint32_t)0xDUL) /**< MCONFIG_PPLOW_FCLK13 Value */
#define MXC_S_I3C_MCONFIG_PPLOW_FCLK13                 (MXC_V_I3C_MCONFIG_PPLOW_FCLK13 << MXC_F_I3C_MCONFIG_PPLOW_POS) /**< MCONFIG_PPLOW_FCLK13 Setting */
#define MXC_V_I3C_MCONFIG_PPLOW_FCLK14                 ((uint32_t)0xEUL) /**< MCONFIG_PPLOW_FCLK14 Value */
#define MXC_S_I3C_MCONFIG_PPLOW_FCLK14                 (MXC_V_I3C_MCONFIG_PPLOW_FCLK14 << MXC_F_I3C_MCONFIG_PPLOW_POS) /**< MCONFIG_PPLOW_FCLK14 Setting */
#define MXC_V_I3C_MCONFIG_PPLOW_FCLK15                 ((uint32_t)0xFUL) /**< MCONFIG_PPLOW_FCLK15 Value */
#define MXC_S_I3C_MCONFIG_PPLOW_FCLK15                 (MXC_V_I3C_MCONFIG_PPLOW_FCLK15 << MXC_F_I3C_MCONFIG_PPLOW_POS) /**< MCONFIG_PPLOW_FCLK15 Setting */

#define MXC_F_I3C_MCONFIG_ODBAUD_POS                   16 /**< MCONFIG_ODBAUD Position */
#define MXC_F_I3C_MCONFIG_ODBAUD                       ((uint32_t)(0xFFUL << MXC_F_I3C_MCONFIG_ODBAUD_POS)) /**< MCONFIG_ODBAUD Mask */

#define MXC_F_I3C_MCONFIG_ODHPP_POS                    24 /**< MCONFIG_ODHPP Position */
#define MXC_F_I3C_MCONFIG_ODHPP                        ((uint32_t)(0x1UL << MXC_F_I3C_MCONFIG_ODHPP_POS)) /**< MCONFIG_ODHPP Mask */

#define MXC_F_I3C_MCONFIG_SKEW_POS                     25 /**< MCONFIG_SKEW Position */
#define MXC_F_I3C_MCONFIG_SKEW                         ((uint32_t)(0x7UL << MXC_F_I3C_MCONFIG_SKEW_POS)) /**< MCONFIG_SKEW Mask */
#define MXC_V_I3C_MCONFIG_SKEW_FCLK0                   ((uint32_t)0x0UL) /**< MCONFIG_SKEW_FCLK0 Value */
#define MXC_S_I3C_MCONFIG_SKEW_FCLK0                   (MXC_V_I3C_MCONFIG_SKEW_FCLK0 << MXC_F_I3C_MCONFIG_SKEW_POS) /**< MCONFIG_SKEW_FCLK0 Setting */
#define MXC_V_I3C_MCONFIG_SKEW_FCLK1                   ((uint32_t)0x1UL) /**< MCONFIG_SKEW_FCLK1 Value */
#define MXC_S_I3C_MCONFIG_SKEW_FCLK1                   (MXC_V_I3C_MCONFIG_SKEW_FCLK1 << MXC_F_I3C_MCONFIG_SKEW_POS) /**< MCONFIG_SKEW_FCLK1 Setting */
#define MXC_V_I3C_MCONFIG_SKEW_FCLK2                   ((uint32_t)0x2UL) /**< MCONFIG_SKEW_FCLK2 Value */
#define MXC_S_I3C_MCONFIG_SKEW_FCLK2                   (MXC_V_I3C_MCONFIG_SKEW_FCLK2 << MXC_F_I3C_MCONFIG_SKEW_POS) /**< MCONFIG_SKEW_FCLK2 Setting */
#define MXC_V_I3C_MCONFIG_SKEW_FCLK3                   ((uint32_t)0x3UL) /**< MCONFIG_SKEW_FCLK3 Value */
#define MXC_S_I3C_MCONFIG_SKEW_FCLK3                   (MXC_V_I3C_MCONFIG_SKEW_FCLK3 << MXC_F_I3C_MCONFIG_SKEW_POS) /**< MCONFIG_SKEW_FCLK3 Setting */
#define MXC_V_I3C_MCONFIG_SKEW_FCLK4                   ((uint32_t)0x4UL) /**< MCONFIG_SKEW_FCLK4 Value */
#define MXC_S_I3C_MCONFIG_SKEW_FCLK4                   (MXC_V_I3C_MCONFIG_SKEW_FCLK4 << MXC_F_I3C_MCONFIG_SKEW_POS) /**< MCONFIG_SKEW_FCLK4 Setting */
#define MXC_V_I3C_MCONFIG_SKEW_FCLK5                   ((uint32_t)0x5UL) /**< MCONFIG_SKEW_FCLK5 Value */
#define MXC_S_I3C_MCONFIG_SKEW_FCLK5                   (MXC_V_I3C_MCONFIG_SKEW_FCLK5 << MXC_F_I3C_MCONFIG_SKEW_POS) /**< MCONFIG_SKEW_FCLK5 Setting */
#define MXC_V_I3C_MCONFIG_SKEW_FCLK6                   ((uint32_t)0x6UL) /**< MCONFIG_SKEW_FCLK6 Value */
#define MXC_S_I3C_MCONFIG_SKEW_FCLK6                   (MXC_V_I3C_MCONFIG_SKEW_FCLK6 << MXC_F_I3C_MCONFIG_SKEW_POS) /**< MCONFIG_SKEW_FCLK6 Setting */
#define MXC_V_I3C_MCONFIG_SKEW_FCLK7                   ((uint32_t)0x7UL) /**< MCONFIG_SKEW_FCLK7 Value */
#define MXC_S_I3C_MCONFIG_SKEW_FCLK7                   (MXC_V_I3C_MCONFIG_SKEW_FCLK7 << MXC_F_I3C_MCONFIG_SKEW_POS) /**< MCONFIG_SKEW_FCLK7 Setting */

#define MXC_F_I3C_MCONFIG_I2CBAUD_POS                  28 /**< MCONFIG_I2CBAUD Position */
#define MXC_F_I3C_MCONFIG_I2CBAUD                      ((uint32_t)(0xFUL << MXC_F_I3C_MCONFIG_I2CBAUD_POS)) /**< MCONFIG_I2CBAUD Mask */

/**@} end of group I3C_MCONFIG_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_CONFIG I3C_CONFIG
 * @brief    Configuration Register
 * @{
 */
#define MXC_F_I3C_CONFIG_TGTENA_POS                    0 /**< CONFIG_TGTENA Position */
#define MXC_F_I3C_CONFIG_TGTENA                        ((uint32_t)(0x1UL << MXC_F_I3C_CONFIG_TGTENA_POS)) /**< CONFIG_TGTENA Mask */

#define MXC_F_I3C_CONFIG_MATCHSS_POS                   2 /**< CONFIG_MATCHSS Position */
#define MXC_F_I3C_CONFIG_MATCHSS                       ((uint32_t)(0x1UL << MXC_F_I3C_CONFIG_MATCHSS_POS)) /**< CONFIG_MATCHSS Mask */

#define MXC_F_I3C_CONFIG_S0IGNORE_POS                  3 /**< CONFIG_S0IGNORE Position */
#define MXC_F_I3C_CONFIG_S0IGNORE                      ((uint32_t)(0x1UL << MXC_F_I3C_CONFIG_S0IGNORE_POS)) /**< CONFIG_S0IGNORE Mask */

#define MXC_F_I3C_CONFIG_BTML_POS                      5 /**< CONFIG_BTML Position */
#define MXC_F_I3C_CONFIG_BTML                          ((uint32_t)(0x3UL << MXC_F_I3C_CONFIG_BTML_POS)) /**< CONFIG_BTML Mask */
#define MXC_V_I3C_CONFIG_BTML_S_LANE                   ((uint32_t)0x0UL) /**< CONFIG_BTML_S_LANE Value */
#define MXC_S_I3C_CONFIG_BTML_S_LANE                   (MXC_V_I3C_CONFIG_BTML_S_LANE << MXC_F_I3C_CONFIG_BTML_POS) /**< CONFIG_BTML_S_LANE Setting */
#define MXC_V_I3C_CONFIG_BTML_SD_LANE                  ((uint32_t)0x1UL) /**< CONFIG_BTML_SD_LANE Value */
#define MXC_S_I3C_CONFIG_BTML_SD_LANE                  (MXC_V_I3C_CONFIG_BTML_SD_LANE << MXC_F_I3C_CONFIG_BTML_POS) /**< CONFIG_BTML_SD_LANE Setting */
#define MXC_V_I3C_CONFIG_BTML_SDQ_LANE                 ((uint32_t)0x3UL) /**< CONFIG_BTML_SDQ_LANE Value */
#define MXC_S_I3C_CONFIG_BTML_SDQ_LANE                 (MXC_V_I3C_CONFIG_BTML_SDQ_LANE << MXC_F_I3C_CONFIG_BTML_POS) /**< CONFIG_BTML_SDQ_LANE Setting */

#define MXC_F_I3C_CONFIG_IDRAND_POS                    8 /**< CONFIG_IDRAND Position */
#define MXC_F_I3C_CONFIG_IDRAND                        ((uint32_t)(0x1UL << MXC_F_I3C_CONFIG_IDRAND_POS)) /**< CONFIG_IDRAND Mask */

#define MXC_F_I3C_CONFIG_OFFLINE_POS                   9 /**< CONFIG_OFFLINE Position */
#define MXC_F_I3C_CONFIG_OFFLINE                       ((uint32_t)(0x1UL << MXC_F_I3C_CONFIG_OFFLINE_POS)) /**< CONFIG_OFFLINE Mask */

#define MXC_F_I3C_CONFIG_HDRCMD_POS                    10 /**< CONFIG_HDRCMD Position */
#define MXC_F_I3C_CONFIG_HDRCMD                        ((uint32_t)(0x3UL << MXC_F_I3C_CONFIG_HDRCMD_POS)) /**< CONFIG_HDRCMD Mask */
#define MXC_V_I3C_CONFIG_HDRCMD_TO_FIFO                ((uint32_t)0x0UL) /**< CONFIG_HDRCMD_TO_FIFO Value */
#define MXC_S_I3C_CONFIG_HDRCMD_TO_FIFO                (MXC_V_I3C_CONFIG_HDRCMD_TO_FIFO << MXC_F_I3C_CONFIG_HDRCMD_POS) /**< CONFIG_HDRCMD_TO_FIFO Setting */
#define MXC_V_I3C_CONFIG_HDRCMD_TO_HDRCMD              ((uint32_t)0x1UL) /**< CONFIG_HDRCMD_TO_HDRCMD Value */
#define MXC_S_I3C_CONFIG_HDRCMD_TO_HDRCMD              (MXC_V_I3C_CONFIG_HDRCMD_TO_HDRCMD << MXC_F_I3C_CONFIG_HDRCMD_POS) /**< CONFIG_HDRCMD_TO_HDRCMD Setting */
#define MXC_V_I3C_CONFIG_HDRCMD_TO_HDRCMD_FIFO         ((uint32_t)0x2UL) /**< CONFIG_HDRCMD_TO_HDRCMD_FIFO Value */
#define MXC_S_I3C_CONFIG_HDRCMD_TO_HDRCMD_FIFO         (MXC_V_I3C_CONFIG_HDRCMD_TO_HDRCMD_FIFO << MXC_F_I3C_CONFIG_HDRCMD_POS) /**< CONFIG_HDRCMD_TO_HDRCMD_FIFO Setting */

#define MXC_F_I3C_CONFIG_BAMATCH_POS                   16 /**< CONFIG_BAMATCH Position */
#define MXC_F_I3C_CONFIG_BAMATCH                       ((uint32_t)(0xFFUL << MXC_F_I3C_CONFIG_BAMATCH_POS)) /**< CONFIG_BAMATCH Mask */

#define MXC_F_I3C_CONFIG_SADDR_POS                     25 /**< CONFIG_SADDR Position */
#define MXC_F_I3C_CONFIG_SADDR                         ((uint32_t)(0x7FUL << MXC_F_I3C_CONFIG_SADDR_POS)) /**< CONFIG_SADDR Mask */

/**@} end of group I3C_CONFIG_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_STATUS I3C_STATUS
 * @brief    Status Register
 * @{
 */
#define MXC_F_I3C_STATUS_STNOTSTOP_POS                 0 /**< STATUS_STNOTSTOP Position */
#define MXC_F_I3C_STATUS_STNOTSTOP                     ((uint32_t)(0x1UL << MXC_F_I3C_STATUS_STNOTSTOP_POS)) /**< STATUS_STNOTSTOP Mask */

#define MXC_F_I3C_STATUS_STMSG_POS                     1 /**< STATUS_STMSG Position */
#define MXC_F_I3C_STATUS_STMSG                         ((uint32_t)(0x1UL << MXC_F_I3C_STATUS_STMSG_POS)) /**< STATUS_STMSG Mask */

#define MXC_F_I3C_STATUS_STCCCH_POS                    2 /**< STATUS_STCCCH Position */
#define MXC_F_I3C_STATUS_STCCCH                        ((uint32_t)(0x1UL << MXC_F_I3C_STATUS_STCCCH_POS)) /**< STATUS_STCCCH Mask */

#define MXC_F_I3C_STATUS_STREQRD_POS                   3 /**< STATUS_STREQRD Position */
#define MXC_F_I3C_STATUS_STREQRD                       ((uint32_t)(0x1UL << MXC_F_I3C_STATUS_STREQRD_POS)) /**< STATUS_STREQRD Mask */

#define MXC_F_I3C_STATUS_STREQWR_POS                   4 /**< STATUS_STREQWR Position */
#define MXC_F_I3C_STATUS_STREQWR                       ((uint32_t)(0x1UL << MXC_F_I3C_STATUS_STREQWR_POS)) /**< STATUS_STREQWR Mask */

#define MXC_F_I3C_STATUS_STDAA_POS                     5 /**< STATUS_STDAA Position */
#define MXC_F_I3C_STATUS_STDAA                         ((uint32_t)(0x1UL << MXC_F_I3C_STATUS_STDAA_POS)) /**< STATUS_STDAA Mask */

#define MXC_F_I3C_STATUS_STHDR_POS                     6 /**< STATUS_STHDR Position */
#define MXC_F_I3C_STATUS_STHDR                         ((uint32_t)(0x1UL << MXC_F_I3C_STATUS_STHDR_POS)) /**< STATUS_STHDR Mask */

#define MXC_F_I3C_STATUS_START_POS                     8 /**< STATUS_START Position */
#define MXC_F_I3C_STATUS_START                         ((uint32_t)(0x1UL << MXC_F_I3C_STATUS_START_POS)) /**< STATUS_START Mask */

#define MXC_F_I3C_STATUS_MATCHED_POS                   9 /**< STATUS_MATCHED Position */
#define MXC_F_I3C_STATUS_MATCHED                       ((uint32_t)(0x1UL << MXC_F_I3C_STATUS_MATCHED_POS)) /**< STATUS_MATCHED Mask */

#define MXC_F_I3C_STATUS_STOP_POS                      10 /**< STATUS_STOP Position */
#define MXC_F_I3C_STATUS_STOP                          ((uint32_t)(0x1UL << MXC_F_I3C_STATUS_STOP_POS)) /**< STATUS_STOP Mask */

#define MXC_F_I3C_STATUS_RXPEND_POS                    11 /**< STATUS_RXPEND Position */
#define MXC_F_I3C_STATUS_RXPEND                        ((uint32_t)(0x1UL << MXC_F_I3C_STATUS_RXPEND_POS)) /**< STATUS_RXPEND Mask */

#define MXC_F_I3C_STATUS_TXNOTFULL_POS                 12 /**< STATUS_TXNOTFULL Position */
#define MXC_F_I3C_STATUS_TXNOTFULL                     ((uint32_t)(0x1UL << MXC_F_I3C_STATUS_TXNOTFULL_POS)) /**< STATUS_TXNOTFULL Mask */

#define MXC_F_I3C_STATUS_DACHG_POS                     13 /**< STATUS_DACHG Position */
#define MXC_F_I3C_STATUS_DACHG                         ((uint32_t)(0x1UL << MXC_F_I3C_STATUS_DACHG_POS)) /**< STATUS_DACHG Mask */

#define MXC_F_I3C_STATUS_CCC_POS                       14 /**< STATUS_CCC Position */
#define MXC_F_I3C_STATUS_CCC                           ((uint32_t)(0x1UL << MXC_F_I3C_STATUS_CCC_POS)) /**< STATUS_CCC Mask */

#define MXC_F_I3C_STATUS_ERRWARN_POS                   15 /**< STATUS_ERRWARN Position */
#define MXC_F_I3C_STATUS_ERRWARN                       ((uint32_t)(0x1UL << MXC_F_I3C_STATUS_ERRWARN_POS)) /**< STATUS_ERRWARN Mask */

#define MXC_F_I3C_STATUS_HDRMATCHED_POS                16 /**< STATUS_HDRMATCHED Position */
#define MXC_F_I3C_STATUS_HDRMATCHED                    ((uint32_t)(0x1UL << MXC_F_I3C_STATUS_HDRMATCHED_POS)) /**< STATUS_HDRMATCHED Mask */

#define MXC_F_I3C_STATUS_CHANDLED_POS                  17 /**< STATUS_CHANDLED Position */
#define MXC_F_I3C_STATUS_CHANDLED                      ((uint32_t)(0x1UL << MXC_F_I3C_STATUS_CHANDLED_POS)) /**< STATUS_CHANDLED Mask */

#define MXC_F_I3C_STATUS_EVENT_POS                     18 /**< STATUS_EVENT Position */
#define MXC_F_I3C_STATUS_EVENT                         ((uint32_t)(0x1UL << MXC_F_I3C_STATUS_EVENT_POS)) /**< STATUS_EVENT Mask */

#define MXC_F_I3C_STATUS_TGTRST_POS                    19 /**< STATUS_TGTRST Position */
#define MXC_F_I3C_STATUS_TGTRST                        ((uint32_t)(0x1UL << MXC_F_I3C_STATUS_TGTRST_POS)) /**< STATUS_TGTRST Mask */

#define MXC_F_I3C_STATUS_EVDET_POS                     20 /**< STATUS_EVDET Position */
#define MXC_F_I3C_STATUS_EVDET                         ((uint32_t)(0x3UL << MXC_F_I3C_STATUS_EVDET_POS)) /**< STATUS_EVDET Mask */
#define MXC_V_I3C_STATUS_EVDET_NONE                    ((uint32_t)0x0UL) /**< STATUS_EVDET_NONE Value */
#define MXC_S_I3C_STATUS_EVDET_NONE                    (MXC_V_I3C_STATUS_EVDET_NONE << MXC_F_I3C_STATUS_EVDET_POS) /**< STATUS_EVDET_NONE Setting */
#define MXC_V_I3C_STATUS_EVDET_REQ_NOT_SENT            ((uint32_t)0x1UL) /**< STATUS_EVDET_REQ_NOT_SENT Value */
#define MXC_S_I3C_STATUS_EVDET_REQ_NOT_SENT            (MXC_V_I3C_STATUS_EVDET_REQ_NOT_SENT << MXC_F_I3C_STATUS_EVDET_POS) /**< STATUS_EVDET_REQ_NOT_SENT Setting */
#define MXC_V_I3C_STATUS_EVDET_REQ_NACKED              ((uint32_t)0x2UL) /**< STATUS_EVDET_REQ_NACKED Value */
#define MXC_S_I3C_STATUS_EVDET_REQ_NACKED              (MXC_V_I3C_STATUS_EVDET_REQ_NACKED << MXC_F_I3C_STATUS_EVDET_POS) /**< STATUS_EVDET_REQ_NACKED Setting */
#define MXC_V_I3C_STATUS_EVDET_REQ_ACKED               ((uint32_t)0x3UL) /**< STATUS_EVDET_REQ_ACKED Value */
#define MXC_S_I3C_STATUS_EVDET_REQ_ACKED               (MXC_V_I3C_STATUS_EVDET_REQ_ACKED << MXC_F_I3C_STATUS_EVDET_POS) /**< STATUS_EVDET_REQ_ACKED Setting */

#define MXC_F_I3C_STATUS_IBIDIS_POS                    24 /**< STATUS_IBIDIS Position */
#define MXC_F_I3C_STATUS_IBIDIS                        ((uint32_t)(0x1UL << MXC_F_I3C_STATUS_IBIDIS_POS)) /**< STATUS_IBIDIS Mask */

#define MXC_F_I3C_STATUS_MRDIS_POS                     25 /**< STATUS_MRDIS Position */
#define MXC_F_I3C_STATUS_MRDIS                         ((uint32_t)(0x1UL << MXC_F_I3C_STATUS_MRDIS_POS)) /**< STATUS_MRDIS Mask */

#define MXC_F_I3C_STATUS_HJDIS_POS                     27 /**< STATUS_HJDIS Position */
#define MXC_F_I3C_STATUS_HJDIS                         ((uint32_t)(0x1UL << MXC_F_I3C_STATUS_HJDIS_POS)) /**< STATUS_HJDIS Mask */

#define MXC_F_I3C_STATUS_ACTSTATE_POS                  28 /**< STATUS_ACTSTATE Position */
#define MXC_F_I3C_STATUS_ACTSTATE                      ((uint32_t)(0x3UL << MXC_F_I3C_STATUS_ACTSTATE_POS)) /**< STATUS_ACTSTATE Mask */
#define MXC_V_I3C_STATUS_ACTSTATE_LATENCY_NONE         ((uint32_t)0x0UL) /**< STATUS_ACTSTATE_LATENCY_NONE Value */
#define MXC_S_I3C_STATUS_ACTSTATE_LATENCY_NONE         (MXC_V_I3C_STATUS_ACTSTATE_LATENCY_NONE << MXC_F_I3C_STATUS_ACTSTATE_POS) /**< STATUS_ACTSTATE_LATENCY_NONE Setting */
#define MXC_V_I3C_STATUS_ACTSTATE_LATENCY_1MS          ((uint32_t)0x1UL) /**< STATUS_ACTSTATE_LATENCY_1MS Value */
#define MXC_S_I3C_STATUS_ACTSTATE_LATENCY_1MS          (MXC_V_I3C_STATUS_ACTSTATE_LATENCY_1MS << MXC_F_I3C_STATUS_ACTSTATE_POS) /**< STATUS_ACTSTATE_LATENCY_1MS Setting */
#define MXC_V_I3C_STATUS_ACTSTATE_LATENCY_100MS        ((uint32_t)0x2UL) /**< STATUS_ACTSTATE_LATENCY_100MS Value */
#define MXC_S_I3C_STATUS_ACTSTATE_LATENCY_100MS        (MXC_V_I3C_STATUS_ACTSTATE_LATENCY_100MS << MXC_F_I3C_STATUS_ACTSTATE_POS) /**< STATUS_ACTSTATE_LATENCY_100MS Setting */
#define MXC_V_I3C_STATUS_ACTSTATE_LATENCY_10S          ((uint32_t)0x3UL) /**< STATUS_ACTSTATE_LATENCY_10S Value */
#define MXC_S_I3C_STATUS_ACTSTATE_LATENCY_10S          (MXC_V_I3C_STATUS_ACTSTATE_LATENCY_10S << MXC_F_I3C_STATUS_ACTSTATE_POS) /**< STATUS_ACTSTATE_LATENCY_10S Setting */

#define MXC_F_I3C_STATUS_TIMECTRL_POS                  30 /**< STATUS_TIMECTRL Position */
#define MXC_F_I3C_STATUS_TIMECTRL                      ((uint32_t)(0x3UL << MXC_F_I3C_STATUS_TIMECTRL_POS)) /**< STATUS_TIMECTRL Mask */
#define MXC_V_I3C_STATUS_TIMECTRL_DISABLED             ((uint32_t)0x0UL) /**< STATUS_TIMECTRL_DISABLED Value */
#define MXC_S_I3C_STATUS_TIMECTRL_DISABLED             (MXC_V_I3C_STATUS_TIMECTRL_DISABLED << MXC_F_I3C_STATUS_TIMECTRL_POS) /**< STATUS_TIMECTRL_DISABLED Setting */
#define MXC_V_I3C_STATUS_TIMECTRL_SYNC                 ((uint32_t)0x1UL) /**< STATUS_TIMECTRL_SYNC Value */
#define MXC_S_I3C_STATUS_TIMECTRL_SYNC                 (MXC_V_I3C_STATUS_TIMECTRL_SYNC << MXC_F_I3C_STATUS_TIMECTRL_POS) /**< STATUS_TIMECTRL_SYNC Setting */
#define MXC_V_I3C_STATUS_TIMECTRL_ASYNCHRONOUS         ((uint32_t)0x2UL) /**< STATUS_TIMECTRL_ASYNCHRONOUS Value */
#define MXC_S_I3C_STATUS_TIMECTRL_ASYNCHRONOUS         (MXC_V_I3C_STATUS_TIMECTRL_ASYNCHRONOUS << MXC_F_I3C_STATUS_TIMECTRL_POS) /**< STATUS_TIMECTRL_ASYNCHRONOUS Setting */
#define MXC_V_I3C_STATUS_TIMECTRL_SYNC_ASYNC           ((uint32_t)0x3UL) /**< STATUS_TIMECTRL_SYNC_ASYNC Value */
#define MXC_S_I3C_STATUS_TIMECTRL_SYNC_ASYNC           (MXC_V_I3C_STATUS_TIMECTRL_SYNC_ASYNC << MXC_F_I3C_STATUS_TIMECTRL_POS) /**< STATUS_TIMECTRL_SYNC_ASYNC Setting */

/**@} end of group I3C_STATUS_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_CTRL I3C_CTRL
 * @brief    I3C Control Register
 * @{
 */
#define MXC_F_I3C_CTRL_EVENT_POS                       0 /**< CTRL_EVENT Position */
#define MXC_F_I3C_CTRL_EVENT                           ((uint32_t)(0x3UL << MXC_F_I3C_CTRL_EVENT_POS)) /**< CTRL_EVENT Mask */
#define MXC_V_I3C_CTRL_EVENT_NORMAL                    ((uint32_t)0x0UL) /**< CTRL_EVENT_NORMAL Value */
#define MXC_S_I3C_CTRL_EVENT_NORMAL                    (MXC_V_I3C_CTRL_EVENT_NORMAL << MXC_F_I3C_CTRL_EVENT_POS) /**< CTRL_EVENT_NORMAL Setting */
#define MXC_V_I3C_CTRL_EVENT_IBI                       ((uint32_t)0x1UL) /**< CTRL_EVENT_IBI Value */
#define MXC_S_I3C_CTRL_EVENT_IBI                       (MXC_V_I3C_CTRL_EVENT_IBI << MXC_F_I3C_CTRL_EVENT_POS) /**< CTRL_EVENT_IBI Setting */
#define MXC_V_I3C_CTRL_EVENT_CTLR_REQ                  ((uint32_t)0x2UL) /**< CTRL_EVENT_CTLR_REQ Value */
#define MXC_S_I3C_CTRL_EVENT_CTLR_REQ                  (MXC_V_I3C_CTRL_EVENT_CTLR_REQ << MXC_F_I3C_CTRL_EVENT_POS) /**< CTRL_EVENT_CTLR_REQ Setting */
#define MXC_V_I3C_CTRL_EVENT_HOTJOIN                   ((uint32_t)0x3UL) /**< CTRL_EVENT_HOTJOIN Value */
#define MXC_S_I3C_CTRL_EVENT_HOTJOIN                   (MXC_V_I3C_CTRL_EVENT_HOTJOIN << MXC_F_I3C_CTRL_EVENT_POS) /**< CTRL_EVENT_HOTJOIN Setting */

#define MXC_F_I3C_CTRL_EXTDATA_POS                     3 /**< CTRL_EXTDATA Position */
#define MXC_F_I3C_CTRL_EXTDATA                         ((uint32_t)(0x1UL << MXC_F_I3C_CTRL_EXTDATA_POS)) /**< CTRL_EXTDATA Mask */

#define MXC_F_I3C_CTRL_MAPIDX_POS                      4 /**< CTRL_MAPIDX Position */
#define MXC_F_I3C_CTRL_MAPIDX                          ((uint32_t)(0xFUL << MXC_F_I3C_CTRL_MAPIDX_POS)) /**< CTRL_MAPIDX Mask */

#define MXC_F_I3C_CTRL_IBIDATA_POS                     8 /**< CTRL_IBIDATA Position */
#define MXC_F_I3C_CTRL_IBIDATA                         ((uint32_t)(0xFFUL << MXC_F_I3C_CTRL_IBIDATA_POS)) /**< CTRL_IBIDATA Mask */

#define MXC_F_I3C_CTRL_PENDINT_POS                     16 /**< CTRL_PENDINT Position */
#define MXC_F_I3C_CTRL_PENDINT                         ((uint32_t)(0xFUL << MXC_F_I3C_CTRL_PENDINT_POS)) /**< CTRL_PENDINT Mask */

#define MXC_F_I3C_CTRL_ACTSTATE_POS                    20 /**< CTRL_ACTSTATE Position */
#define MXC_F_I3C_CTRL_ACTSTATE                        ((uint32_t)(0x3UL << MXC_F_I3C_CTRL_ACTSTATE_POS)) /**< CTRL_ACTSTATE Mask */

#define MXC_F_I3C_CTRL_VENDINFO_POS                    24 /**< CTRL_VENDINFO Position */
#define MXC_F_I3C_CTRL_VENDINFO                        ((uint32_t)(0xFFUL << MXC_F_I3C_CTRL_VENDINFO_POS)) /**< CTRL_VENDINFO Mask */

/**@} end of group I3C_CTRL_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_INTSET I3C_INTSET
 * @brief    Interrupt Enable Set Register
 * @{
 */
#define MXC_F_I3C_INTSET_START_POS                     8 /**< INTSET_START Position */
#define MXC_F_I3C_INTSET_START                         ((uint32_t)(0x1UL << MXC_F_I3C_INTSET_START_POS)) /**< INTSET_START Mask */

#define MXC_F_I3C_INTSET_MATCHED_POS                   9 /**< INTSET_MATCHED Position */
#define MXC_F_I3C_INTSET_MATCHED                       ((uint32_t)(0x1UL << MXC_F_I3C_INTSET_MATCHED_POS)) /**< INTSET_MATCHED Mask */

#define MXC_F_I3C_INTSET_STOP_POS                      10 /**< INTSET_STOP Position */
#define MXC_F_I3C_INTSET_STOP                          ((uint32_t)(0x1UL << MXC_F_I3C_INTSET_STOP_POS)) /**< INTSET_STOP Mask */

#define MXC_F_I3C_INTSET_RXPEND_POS                    11 /**< INTSET_RXPEND Position */
#define MXC_F_I3C_INTSET_RXPEND                        ((uint32_t)(0x1UL << MXC_F_I3C_INTSET_RXPEND_POS)) /**< INTSET_RXPEND Mask */

#define MXC_F_I3C_INTSET_TXNOTFULL_POS                 12 /**< INTSET_TXNOTFULL Position */
#define MXC_F_I3C_INTSET_TXNOTFULL                     ((uint32_t)(0x1UL << MXC_F_I3C_INTSET_TXNOTFULL_POS)) /**< INTSET_TXNOTFULL Mask */

#define MXC_F_I3C_INTSET_DACHG_POS                     13 /**< INTSET_DACHG Position */
#define MXC_F_I3C_INTSET_DACHG                         ((uint32_t)(0x1UL << MXC_F_I3C_INTSET_DACHG_POS)) /**< INTSET_DACHG Mask */

#define MXC_F_I3C_INTSET_CCC_POS                       14 /**< INTSET_CCC Position */
#define MXC_F_I3C_INTSET_CCC                           ((uint32_t)(0x1UL << MXC_F_I3C_INTSET_CCC_POS)) /**< INTSET_CCC Mask */

#define MXC_F_I3C_INTSET_ERRWARN_POS                   15 /**< INTSET_ERRWARN Position */
#define MXC_F_I3C_INTSET_ERRWARN                       ((uint32_t)(0x1UL << MXC_F_I3C_INTSET_ERRWARN_POS)) /**< INTSET_ERRWARN Mask */

#define MXC_F_I3C_INTSET_HDRMATCHED_POS                16 /**< INTSET_HDRMATCHED Position */
#define MXC_F_I3C_INTSET_HDRMATCHED                    ((uint32_t)(0x1UL << MXC_F_I3C_INTSET_HDRMATCHED_POS)) /**< INTSET_HDRMATCHED Mask */

#define MXC_F_I3C_INTSET_CHANDLED_POS                  17 /**< INTSET_CHANDLED Position */
#define MXC_F_I3C_INTSET_CHANDLED                      ((uint32_t)(0x1UL << MXC_F_I3C_INTSET_CHANDLED_POS)) /**< INTSET_CHANDLED Mask */

#define MXC_F_I3C_INTSET_EVENT_POS                     18 /**< INTSET_EVENT Position */
#define MXC_F_I3C_INTSET_EVENT                         ((uint32_t)(0x1UL << MXC_F_I3C_INTSET_EVENT_POS)) /**< INTSET_EVENT Mask */

#define MXC_F_I3C_INTSET_TGRST_POS                     19 /**< INTSET_TGRST Position */
#define MXC_F_I3C_INTSET_TGRST                         ((uint32_t)(0x1UL << MXC_F_I3C_INTSET_TGRST_POS)) /**< INTSET_TGRST Mask */

/**@} end of group I3C_INTSET_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_INTCLR I3C_INTCLR
 * @brief    Interrupt Enable Clear Register
 * @{
 */
#define MXC_F_I3C_INTCLR_START_POS                     8 /**< INTCLR_START Position */
#define MXC_F_I3C_INTCLR_START                         ((uint32_t)(0x1UL << MXC_F_I3C_INTCLR_START_POS)) /**< INTCLR_START Mask */

#define MXC_F_I3C_INTCLR_MATCHED_POS                   9 /**< INTCLR_MATCHED Position */
#define MXC_F_I3C_INTCLR_MATCHED                       ((uint32_t)(0x1UL << MXC_F_I3C_INTCLR_MATCHED_POS)) /**< INTCLR_MATCHED Mask */

#define MXC_F_I3C_INTCLR_STOP_POS                      10 /**< INTCLR_STOP Position */
#define MXC_F_I3C_INTCLR_STOP                          ((uint32_t)(0x1UL << MXC_F_I3C_INTCLR_STOP_POS)) /**< INTCLR_STOP Mask */

#define MXC_F_I3C_INTCLR_RXPEND_POS                    11 /**< INTCLR_RXPEND Position */
#define MXC_F_I3C_INTCLR_RXPEND                        ((uint32_t)(0x1UL << MXC_F_I3C_INTCLR_RXPEND_POS)) /**< INTCLR_RXPEND Mask */

#define MXC_F_I3C_INTCLR_TXNOTFULL_POS                 12 /**< INTCLR_TXNOTFULL Position */
#define MXC_F_I3C_INTCLR_TXNOTFULL                     ((uint32_t)(0x1UL << MXC_F_I3C_INTCLR_TXNOTFULL_POS)) /**< INTCLR_TXNOTFULL Mask */

#define MXC_F_I3C_INTCLR_DACHG_POS                     13 /**< INTCLR_DACHG Position */
#define MXC_F_I3C_INTCLR_DACHG                         ((uint32_t)(0x1UL << MXC_F_I3C_INTCLR_DACHG_POS)) /**< INTCLR_DACHG Mask */

#define MXC_F_I3C_INTCLR_CCC_POS                       14 /**< INTCLR_CCC Position */
#define MXC_F_I3C_INTCLR_CCC                           ((uint32_t)(0x1UL << MXC_F_I3C_INTCLR_CCC_POS)) /**< INTCLR_CCC Mask */

#define MXC_F_I3C_INTCLR_ERRWARN_POS                   15 /**< INTCLR_ERRWARN Position */
#define MXC_F_I3C_INTCLR_ERRWARN                       ((uint32_t)(0x1UL << MXC_F_I3C_INTCLR_ERRWARN_POS)) /**< INTCLR_ERRWARN Mask */

#define MXC_F_I3C_INTCLR_HDRMATCHED_POS                16 /**< INTCLR_HDRMATCHED Position */
#define MXC_F_I3C_INTCLR_HDRMATCHED                    ((uint32_t)(0x1UL << MXC_F_I3C_INTCLR_HDRMATCHED_POS)) /**< INTCLR_HDRMATCHED Mask */

#define MXC_F_I3C_INTCLR_CHANDLED_POS                  17 /**< INTCLR_CHANDLED Position */
#define MXC_F_I3C_INTCLR_CHANDLED                      ((uint32_t)(0x1UL << MXC_F_I3C_INTCLR_CHANDLED_POS)) /**< INTCLR_CHANDLED Mask */

#define MXC_F_I3C_INTCLR_EVENT_POS                     18 /**< INTCLR_EVENT Position */
#define MXC_F_I3C_INTCLR_EVENT                         ((uint32_t)(0x1UL << MXC_F_I3C_INTCLR_EVENT_POS)) /**< INTCLR_EVENT Mask */

#define MXC_F_I3C_INTCLR_TGRST_POS                     19 /**< INTCLR_TGRST Position */
#define MXC_F_I3C_INTCLR_TGRST                         ((uint32_t)(0x1UL << MXC_F_I3C_INTCLR_TGRST_POS)) /**< INTCLR_TGRST Mask */

/**@} end of group I3C_INTCLR_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_INTMASKED I3C_INTMASKED
 * @brief    Interrupt Masked Register
 * @{
 */
#define MXC_F_I3C_INTMASKED_START_POS                  8 /**< INTMASKED_START Position */
#define MXC_F_I3C_INTMASKED_START                      ((uint32_t)(0x1UL << MXC_F_I3C_INTMASKED_START_POS)) /**< INTMASKED_START Mask */

#define MXC_F_I3C_INTMASKED_MATCHED_POS                9 /**< INTMASKED_MATCHED Position */
#define MXC_F_I3C_INTMASKED_MATCHED                    ((uint32_t)(0x1UL << MXC_F_I3C_INTMASKED_MATCHED_POS)) /**< INTMASKED_MATCHED Mask */

#define MXC_F_I3C_INTMASKED_STOP_POS                   10 /**< INTMASKED_STOP Position */
#define MXC_F_I3C_INTMASKED_STOP                       ((uint32_t)(0x1UL << MXC_F_I3C_INTMASKED_STOP_POS)) /**< INTMASKED_STOP Mask */

#define MXC_F_I3C_INTMASKED_RXPEND_POS                 11 /**< INTMASKED_RXPEND Position */
#define MXC_F_I3C_INTMASKED_RXPEND                     ((uint32_t)(0x1UL << MXC_F_I3C_INTMASKED_RXPEND_POS)) /**< INTMASKED_RXPEND Mask */

#define MXC_F_I3C_INTMASKED_TXNOTFULL_POS              12 /**< INTMASKED_TXNOTFULL Position */
#define MXC_F_I3C_INTMASKED_TXNOTFULL                  ((uint32_t)(0x1UL << MXC_F_I3C_INTMASKED_TXNOTFULL_POS)) /**< INTMASKED_TXNOTFULL Mask */

#define MXC_F_I3C_INTMASKED_DACHG_POS                  13 /**< INTMASKED_DACHG Position */
#define MXC_F_I3C_INTMASKED_DACHG                      ((uint32_t)(0x1UL << MXC_F_I3C_INTMASKED_DACHG_POS)) /**< INTMASKED_DACHG Mask */

#define MXC_F_I3C_INTMASKED_CCC_POS                    14 /**< INTMASKED_CCC Position */
#define MXC_F_I3C_INTMASKED_CCC                        ((uint32_t)(0x1UL << MXC_F_I3C_INTMASKED_CCC_POS)) /**< INTMASKED_CCC Mask */

#define MXC_F_I3C_INTMASKED_ERRWARN_POS                15 /**< INTMASKED_ERRWARN Position */
#define MXC_F_I3C_INTMASKED_ERRWARN                    ((uint32_t)(0x1UL << MXC_F_I3C_INTMASKED_ERRWARN_POS)) /**< INTMASKED_ERRWARN Mask */

#define MXC_F_I3C_INTMASKED_HDRMATCHED_POS             16 /**< INTMASKED_HDRMATCHED Position */
#define MXC_F_I3C_INTMASKED_HDRMATCHED                 ((uint32_t)(0x1UL << MXC_F_I3C_INTMASKED_HDRMATCHED_POS)) /**< INTMASKED_HDRMATCHED Mask */

#define MXC_F_I3C_INTMASKED_CHANDLED_POS               17 /**< INTMASKED_CHANDLED Position */
#define MXC_F_I3C_INTMASKED_CHANDLED                   ((uint32_t)(0x1UL << MXC_F_I3C_INTMASKED_CHANDLED_POS)) /**< INTMASKED_CHANDLED Mask */

#define MXC_F_I3C_INTMASKED_EVENT_POS                  18 /**< INTMASKED_EVENT Position */
#define MXC_F_I3C_INTMASKED_EVENT                      ((uint32_t)(0x1UL << MXC_F_I3C_INTMASKED_EVENT_POS)) /**< INTMASKED_EVENT Mask */

#define MXC_F_I3C_INTMASKED_TGRST_POS                  19 /**< INTMASKED_TGRST Position */
#define MXC_F_I3C_INTMASKED_TGRST                      ((uint32_t)(0x1UL << MXC_F_I3C_INTMASKED_TGRST_POS)) /**< INTMASKED_TGRST Mask */

/**@} end of group I3C_INTMASKED_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_ERRWARN I3C_ERRWARN
 * @brief    Error and Warning Register
 * @{
 */
#define MXC_F_I3C_ERRWARN_ORUN_POS                     0 /**< ERRWARN_ORUN Position */
#define MXC_F_I3C_ERRWARN_ORUN                         ((uint32_t)(0x1UL << MXC_F_I3C_ERRWARN_ORUN_POS)) /**< ERRWARN_ORUN Mask */

#define MXC_F_I3C_ERRWARN_URUN_POS                     1 /**< ERRWARN_URUN Position */
#define MXC_F_I3C_ERRWARN_URUN                         ((uint32_t)(0x1UL << MXC_F_I3C_ERRWARN_URUN_POS)) /**< ERRWARN_URUN Mask */

#define MXC_F_I3C_ERRWARN_URUNNACK_POS                 2 /**< ERRWARN_URUNNACK Position */
#define MXC_F_I3C_ERRWARN_URUNNACK                     ((uint32_t)(0x1UL << MXC_F_I3C_ERRWARN_URUNNACK_POS)) /**< ERRWARN_URUNNACK Mask */

#define MXC_F_I3C_ERRWARN_TERM_POS                     3 /**< ERRWARN_TERM Position */
#define MXC_F_I3C_ERRWARN_TERM                         ((uint32_t)(0x1UL << MXC_F_I3C_ERRWARN_TERM_POS)) /**< ERRWARN_TERM Mask */

#define MXC_F_I3C_ERRWARN_INVSTART_POS                 4 /**< ERRWARN_INVSTART Position */
#define MXC_F_I3C_ERRWARN_INVSTART                     ((uint32_t)(0x1UL << MXC_F_I3C_ERRWARN_INVSTART_POS)) /**< ERRWARN_INVSTART Mask */

#define MXC_F_I3C_ERRWARN_SPAR_POS                     8 /**< ERRWARN_SPAR Position */
#define MXC_F_I3C_ERRWARN_SPAR                         ((uint32_t)(0x1UL << MXC_F_I3C_ERRWARN_SPAR_POS)) /**< ERRWARN_SPAR Mask */

#define MXC_F_I3C_ERRWARN_HPAR_POS                     9 /**< ERRWARN_HPAR Position */
#define MXC_F_I3C_ERRWARN_HPAR                         ((uint32_t)(0x1UL << MXC_F_I3C_ERRWARN_HPAR_POS)) /**< ERRWARN_HPAR Mask */

#define MXC_F_I3C_ERRWARN_HCRC_POS                     10 /**< ERRWARN_HCRC Position */
#define MXC_F_I3C_ERRWARN_HCRC                         ((uint32_t)(0x1UL << MXC_F_I3C_ERRWARN_HCRC_POS)) /**< ERRWARN_HCRC Mask */

#define MXC_F_I3C_ERRWARN_S0S1_POS                     11 /**< ERRWARN_S0S1 Position */
#define MXC_F_I3C_ERRWARN_S0S1                         ((uint32_t)(0x1UL << MXC_F_I3C_ERRWARN_S0S1_POS)) /**< ERRWARN_S0S1 Mask */

#define MXC_F_I3C_ERRWARN_HNOVERIFY_POS                12 /**< ERRWARN_HNOVERIFY Position */
#define MXC_F_I3C_ERRWARN_HNOVERIFY                    ((uint32_t)(0x1UL << MXC_F_I3C_ERRWARN_HNOVERIFY_POS)) /**< ERRWARN_HNOVERIFY Mask */

#define MXC_F_I3C_ERRWARN_HINVREQ_POS                  13 /**< ERRWARN_HINVREQ Position */
#define MXC_F_I3C_ERRWARN_HINVREQ                      ((uint32_t)(0x1UL << MXC_F_I3C_ERRWARN_HINVREQ_POS)) /**< ERRWARN_HINVREQ Mask */

#define MXC_F_I3C_ERRWARN_OREAD_POS                    16 /**< ERRWARN_OREAD Position */
#define MXC_F_I3C_ERRWARN_OREAD                        ((uint32_t)(0x1UL << MXC_F_I3C_ERRWARN_OREAD_POS)) /**< ERRWARN_OREAD Mask */

#define MXC_F_I3C_ERRWARN_OWRITE_POS                   17 /**< ERRWARN_OWRITE Position */
#define MXC_F_I3C_ERRWARN_OWRITE                       ((uint32_t)(0x1UL << MXC_F_I3C_ERRWARN_OWRITE_POS)) /**< ERRWARN_OWRITE Mask */

#define MXC_F_I3C_ERRWARN_WRONGSIZE_POS                21 /**< ERRWARN_WRONGSIZE Position */
#define MXC_F_I3C_ERRWARN_WRONGSIZE                    ((uint32_t)(0x1UL << MXC_F_I3C_ERRWARN_WRONGSIZE_POS)) /**< ERRWARN_WRONGSIZE Mask */

/**@} end of group I3C_ERRWARN_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_DMACTRL I3C_DMACTRL
 * @brief    DMA Control Register
 * @{
 */
#define MXC_F_I3C_DMACTRL_DMAFB_POS                    0 /**< DMACTRL_DMAFB Position */
#define MXC_F_I3C_DMACTRL_DMAFB                        ((uint32_t)(0x3UL << MXC_F_I3C_DMACTRL_DMAFB_POS)) /**< DMACTRL_DMAFB Mask */
#define MXC_V_I3C_DMACTRL_DMAFB_DISABLE                ((uint32_t)0x0UL) /**< DMACTRL_DMAFB_DISABLE Value */
#define MXC_S_I3C_DMACTRL_DMAFB_DISABLE                (MXC_V_I3C_DMACTRL_DMAFB_DISABLE << MXC_F_I3C_DMACTRL_DMAFB_POS) /**< DMACTRL_DMAFB_DISABLE Setting */
#define MXC_V_I3C_DMACTRL_DMAFB_EN_ONEFRAME_SDR        ((uint32_t)0x1UL) /**< DMACTRL_DMAFB_EN_ONEFRAME_SDR Value */
#define MXC_S_I3C_DMACTRL_DMAFB_EN_ONEFRAME_SDR        (MXC_V_I3C_DMACTRL_DMAFB_EN_ONEFRAME_SDR << MXC_F_I3C_DMACTRL_DMAFB_POS) /**< DMACTRL_DMAFB_EN_ONEFRAME_SDR Setting */
#define MXC_V_I3C_DMACTRL_DMAFB_ENABLE                 ((uint32_t)0x2UL) /**< DMACTRL_DMAFB_ENABLE Value */
#define MXC_S_I3C_DMACTRL_DMAFB_ENABLE                 (MXC_V_I3C_DMACTRL_DMAFB_ENABLE << MXC_F_I3C_DMACTRL_DMAFB_POS) /**< DMACTRL_DMAFB_ENABLE Setting */

#define MXC_F_I3C_DMACTRL_DMATB_POS                    2 /**< DMACTRL_DMATB Position */
#define MXC_F_I3C_DMACTRL_DMATB                        ((uint32_t)(0x3UL << MXC_F_I3C_DMACTRL_DMATB_POS)) /**< DMACTRL_DMATB Mask */
#define MXC_V_I3C_DMACTRL_DMATB_DISABLE                ((uint32_t)0x0UL) /**< DMACTRL_DMATB_DISABLE Value */
#define MXC_S_I3C_DMACTRL_DMATB_DISABLE                (MXC_V_I3C_DMACTRL_DMATB_DISABLE << MXC_F_I3C_DMACTRL_DMATB_POS) /**< DMACTRL_DMATB_DISABLE Setting */
#define MXC_V_I3C_DMACTRL_DMATB_EN_ONEFRAME_SDR        ((uint32_t)0x1UL) /**< DMACTRL_DMATB_EN_ONEFRAME_SDR Value */
#define MXC_S_I3C_DMACTRL_DMATB_EN_ONEFRAME_SDR        (MXC_V_I3C_DMACTRL_DMATB_EN_ONEFRAME_SDR << MXC_F_I3C_DMACTRL_DMATB_POS) /**< DMACTRL_DMATB_EN_ONEFRAME_SDR Setting */
#define MXC_V_I3C_DMACTRL_DMATB_ENABLE                 ((uint32_t)0x2UL) /**< DMACTRL_DMATB_ENABLE Value */
#define MXC_S_I3C_DMACTRL_DMATB_ENABLE                 (MXC_V_I3C_DMACTRL_DMATB_ENABLE << MXC_F_I3C_DMACTRL_DMATB_POS) /**< DMACTRL_DMATB_ENABLE Setting */

#define MXC_F_I3C_DMACTRL_DMAWIDTH_POS                 4 /**< DMACTRL_DMAWIDTH Position */
#define MXC_F_I3C_DMACTRL_DMAWIDTH                     ((uint32_t)(0x3UL << MXC_F_I3C_DMACTRL_DMAWIDTH_POS)) /**< DMACTRL_DMAWIDTH Mask */
#define MXC_V_I3C_DMACTRL_DMAWIDTH_BYTE                ((uint32_t)0x0UL) /**< DMACTRL_DMAWIDTH_BYTE Value */
#define MXC_S_I3C_DMACTRL_DMAWIDTH_BYTE                (MXC_V_I3C_DMACTRL_DMAWIDTH_BYTE << MXC_F_I3C_DMACTRL_DMAWIDTH_POS) /**< DMACTRL_DMAWIDTH_BYTE Setting */
#define MXC_V_I3C_DMACTRL_DMAWIDTH_HALFWORD            ((uint32_t)0x2UL) /**< DMACTRL_DMAWIDTH_HALFWORD Value */
#define MXC_S_I3C_DMACTRL_DMAWIDTH_HALFWORD            (MXC_V_I3C_DMACTRL_DMAWIDTH_HALFWORD << MXC_F_I3C_DMACTRL_DMAWIDTH_POS) /**< DMACTRL_DMAWIDTH_HALFWORD Setting */

/**@} end of group I3C_DMACTRL_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_HDRBTCFG I3C_HDRBTCFG
 * @brief    HDR-BT Configuration Register
 * @{
 */
#define MXC_F_I3C_HDRBTCFG_CRC32_POS                   2 /**< HDRBTCFG_CRC32 Position */
#define MXC_F_I3C_HDRBTCFG_CRC32                       ((uint32_t)(0x1UL << MXC_F_I3C_HDRBTCFG_CRC32_POS)) /**< HDRBTCFG_CRC32 Mask */

#define MXC_F_I3C_HDRBTCFG_WDATAMAX_POS                4 /**< HDRBTCFG_WDATAMAX Position */
#define MXC_F_I3C_HDRBTCFG_WDATAMAX                    ((uint32_t)(0xFFFUL << MXC_F_I3C_HDRBTCFG_WDATAMAX_POS)) /**< HDRBTCFG_WDATAMAX Mask */

#define MXC_F_I3C_HDRBTCFG_RDATALEN_POS                16 /**< HDRBTCFG_RDATALEN Position */
#define MXC_F_I3C_HDRBTCFG_RDATALEN                    ((uint32_t)(0xFFFFUL << MXC_F_I3C_HDRBTCFG_RDATALEN_POS)) /**< HDRBTCFG_RDATALEN Mask */

/**@} end of group I3C_HDRBTCFG_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_HDRBTLAST I3C_HDRBTLAST
 * @brief    HDR-BT Last Data Length Register
 * @{
 */
#define MXC_F_I3C_HDRBTLAST_DATALEN_POS                16 /**< HDRBTLAST_DATALEN Position */
#define MXC_F_I3C_HDRBTLAST_DATALEN                    ((uint32_t)(0xFFFFUL << MXC_F_I3C_HDRBTLAST_DATALEN_POS)) /**< HDRBTLAST_DATALEN Mask */

/**@} end of group I3C_HDRBTLAST_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_DATACTRL I3C_DATACTRL
 * @brief    Data Control Register
 * @{
 */
#define MXC_F_I3C_DATACTRL_FLUSHTB_POS                 0 /**< DATACTRL_FLUSHTB Position */
#define MXC_F_I3C_DATACTRL_FLUSHTB                     ((uint32_t)(0x1UL << MXC_F_I3C_DATACTRL_FLUSHTB_POS)) /**< DATACTRL_FLUSHTB Mask */

#define MXC_F_I3C_DATACTRL_FLUSHFB_POS                 1 /**< DATACTRL_FLUSHFB Position */
#define MXC_F_I3C_DATACTRL_FLUSHFB                     ((uint32_t)(0x1UL << MXC_F_I3C_DATACTRL_FLUSHFB_POS)) /**< DATACTRL_FLUSHFB Mask */

#define MXC_F_I3C_DATACTRL_UNLOCK_POS                  3 /**< DATACTRL_UNLOCK Position */
#define MXC_F_I3C_DATACTRL_UNLOCK                      ((uint32_t)(0x1UL << MXC_F_I3C_DATACTRL_UNLOCK_POS)) /**< DATACTRL_UNLOCK Mask */

#define MXC_F_I3C_DATACTRL_TXTRIG_POS                  4 /**< DATACTRL_TXTRIG Position */
#define MXC_F_I3C_DATACTRL_TXTRIG                      ((uint32_t)(0x3UL << MXC_F_I3C_DATACTRL_TXTRIG_POS)) /**< DATACTRL_TXTRIG Mask */
#define MXC_V_I3C_DATACTRL_TXTRIG_EMPTY                ((uint32_t)0x0UL) /**< DATACTRL_TXTRIG_EMPTY Value */
#define MXC_S_I3C_DATACTRL_TXTRIG_EMPTY                (MXC_V_I3C_DATACTRL_TXTRIG_EMPTY << MXC_F_I3C_DATACTRL_TXTRIG_POS) /**< DATACTRL_TXTRIG_EMPTY Setting */
#define MXC_V_I3C_DATACTRL_TXTRIG_QUARTER_FULL         ((uint32_t)0x1UL) /**< DATACTRL_TXTRIG_QUARTER_FULL Value */
#define MXC_S_I3C_DATACTRL_TXTRIG_QUARTER_FULL         (MXC_V_I3C_DATACTRL_TXTRIG_QUARTER_FULL << MXC_F_I3C_DATACTRL_TXTRIG_POS) /**< DATACTRL_TXTRIG_QUARTER_FULL Setting */
#define MXC_V_I3C_DATACTRL_TXTRIG_HALF_FULL            ((uint32_t)0x2UL) /**< DATACTRL_TXTRIG_HALF_FULL Value */
#define MXC_S_I3C_DATACTRL_TXTRIG_HALF_FULL            (MXC_V_I3C_DATACTRL_TXTRIG_HALF_FULL << MXC_F_I3C_DATACTRL_TXTRIG_POS) /**< DATACTRL_TXTRIG_HALF_FULL Setting */
#define MXC_V_I3C_DATACTRL_TXTRIG_ALMOST_FULL          ((uint32_t)0x3UL) /**< DATACTRL_TXTRIG_ALMOST_FULL Value */
#define MXC_S_I3C_DATACTRL_TXTRIG_ALMOST_FULL          (MXC_V_I3C_DATACTRL_TXTRIG_ALMOST_FULL << MXC_F_I3C_DATACTRL_TXTRIG_POS) /**< DATACTRL_TXTRIG_ALMOST_FULL Setting */

#define MXC_F_I3C_DATACTRL_RXTRIG_POS                  6 /**< DATACTRL_RXTRIG Position */
#define MXC_F_I3C_DATACTRL_RXTRIG                      ((uint32_t)(0x3UL << MXC_F_I3C_DATACTRL_RXTRIG_POS)) /**< DATACTRL_RXTRIG Mask */
#define MXC_V_I3C_DATACTRL_RXTRIG_NOT_EMPTY            ((uint32_t)0x0UL) /**< DATACTRL_RXTRIG_NOT_EMPTY Value */
#define MXC_S_I3C_DATACTRL_RXTRIG_NOT_EMPTY            (MXC_V_I3C_DATACTRL_RXTRIG_NOT_EMPTY << MXC_F_I3C_DATACTRL_RXTRIG_POS) /**< DATACTRL_RXTRIG_NOT_EMPTY Setting */
#define MXC_V_I3C_DATACTRL_RXTRIG_QUARTER_FULL         ((uint32_t)0x1UL) /**< DATACTRL_RXTRIG_QUARTER_FULL Value */
#define MXC_S_I3C_DATACTRL_RXTRIG_QUARTER_FULL         (MXC_V_I3C_DATACTRL_RXTRIG_QUARTER_FULL << MXC_F_I3C_DATACTRL_RXTRIG_POS) /**< DATACTRL_RXTRIG_QUARTER_FULL Setting */
#define MXC_V_I3C_DATACTRL_RXTRIG_HALF_FULL            ((uint32_t)0x2UL) /**< DATACTRL_RXTRIG_HALF_FULL Value */
#define MXC_S_I3C_DATACTRL_RXTRIG_HALF_FULL            (MXC_V_I3C_DATACTRL_RXTRIG_HALF_FULL << MXC_F_I3C_DATACTRL_RXTRIG_POS) /**< DATACTRL_RXTRIG_HALF_FULL Setting */
#define MXC_V_I3C_DATACTRL_RXTRIG_3_4_FULL             ((uint32_t)0x3UL) /**< DATACTRL_RXTRIG_3_4_FULL Value */
#define MXC_S_I3C_DATACTRL_RXTRIG_3_4_FULL             (MXC_V_I3C_DATACTRL_RXTRIG_3_4_FULL << MXC_F_I3C_DATACTRL_RXTRIG_POS) /**< DATACTRL_RXTRIG_3_4_FULL Setting */

#define MXC_F_I3C_DATACTRL_TXCOUNT_POS                 16 /**< DATACTRL_TXCOUNT Position */
#define MXC_F_I3C_DATACTRL_TXCOUNT                     ((uint32_t)(0x3FUL << MXC_F_I3C_DATACTRL_TXCOUNT_POS)) /**< DATACTRL_TXCOUNT Mask */

#define MXC_F_I3C_DATACTRL_RXCOUNT_POS                 24 /**< DATACTRL_RXCOUNT Position */
#define MXC_F_I3C_DATACTRL_RXCOUNT                     ((uint32_t)(0x3FUL << MXC_F_I3C_DATACTRL_RXCOUNT_POS)) /**< DATACTRL_RXCOUNT Mask */

#define MXC_F_I3C_DATACTRL_TXFULL_POS                  30 /**< DATACTRL_TXFULL Position */
#define MXC_F_I3C_DATACTRL_TXFULL                      ((uint32_t)(0x1UL << MXC_F_I3C_DATACTRL_TXFULL_POS)) /**< DATACTRL_TXFULL Mask */

#define MXC_F_I3C_DATACTRL_RXEMPTY_POS                 31 /**< DATACTRL_RXEMPTY Position */
#define MXC_F_I3C_DATACTRL_RXEMPTY                     ((uint32_t)(0x1UL << MXC_F_I3C_DATACTRL_RXEMPTY_POS)) /**< DATACTRL_RXEMPTY Mask */

/**@} end of group I3C_DATACTRL_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_WDATAB I3C_WDATAB
 * @brief    Write Byte Data Register
 * @{
 */
#define MXC_F_I3C_WDATAB_DATA_POS                      0 /**< WDATAB_DATA Position */
#define MXC_F_I3C_WDATAB_DATA                          ((uint32_t)(0xFFUL << MXC_F_I3C_WDATAB_DATA_POS)) /**< WDATAB_DATA Mask */

#define MXC_F_I3C_WDATAB_END_POS                       8 /**< WDATAB_END Position */
#define MXC_F_I3C_WDATAB_END                           ((uint32_t)(0x1UL << MXC_F_I3C_WDATAB_END_POS)) /**< WDATAB_END Mask */

#define MXC_F_I3C_WDATAB_END2_POS                      16 /**< WDATAB_END2 Position */
#define MXC_F_I3C_WDATAB_END2                          ((uint32_t)(0x1UL << MXC_F_I3C_WDATAB_END2_POS)) /**< WDATAB_END2 Mask */

/**@} end of group I3C_WDATAB_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_WDATABE I3C_WDATABE
 * @brief    Write Byte Data as End Register
 * @{
 */
#define MXC_F_I3C_WDATABE_DATA_POS                     0 /**< WDATABE_DATA Position */
#define MXC_F_I3C_WDATABE_DATA                         ((uint32_t)(0xFFUL << MXC_F_I3C_WDATABE_DATA_POS)) /**< WDATABE_DATA Mask */

/**@} end of group I3C_WDATABE_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_WDATAH I3C_WDATAH
 * @brief    Write Half-Word Data Register
 * @{
 */
#define MXC_F_I3C_WDATAH_DATA0_POS                     0 /**< WDATAH_DATA0 Position */
#define MXC_F_I3C_WDATAH_DATA0                         ((uint32_t)(0xFFUL << MXC_F_I3C_WDATAH_DATA0_POS)) /**< WDATAH_DATA0 Mask */

#define MXC_F_I3C_WDATAH_DATA1_POS                     8 /**< WDATAH_DATA1 Position */
#define MXC_F_I3C_WDATAH_DATA1                         ((uint32_t)(0xFFUL << MXC_F_I3C_WDATAH_DATA1_POS)) /**< WDATAH_DATA1 Mask */

#define MXC_F_I3C_WDATAH_END_POS                       16 /**< WDATAH_END Position */
#define MXC_F_I3C_WDATAH_END                           ((uint32_t)(0x1UL << MXC_F_I3C_WDATAH_END_POS)) /**< WDATAH_END Mask */

/**@} end of group I3C_WDATAH_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_WDATAHE I3C_WDATAHE
 * @brief    Write Half-Word Data as End Register
 * @{
 */
#define MXC_F_I3C_WDATAHE_DATA0_POS                    0 /**< WDATAHE_DATA0 Position */
#define MXC_F_I3C_WDATAHE_DATA0                        ((uint32_t)(0xFFUL << MXC_F_I3C_WDATAHE_DATA0_POS)) /**< WDATAHE_DATA0 Mask */

#define MXC_F_I3C_WDATAHE_DATA1_POS                    8 /**< WDATAHE_DATA1 Position */
#define MXC_F_I3C_WDATAHE_DATA1                        ((uint32_t)(0xFFUL << MXC_F_I3C_WDATAHE_DATA1_POS)) /**< WDATAHE_DATA1 Mask */

/**@} end of group I3C_WDATAHE_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_RDATAB I3C_RDATAB
 * @brief    Read Byte Data Register
 * @{
 */
#define MXC_F_I3C_RDATAB_DATA_POS                      0 /**< RDATAB_DATA Position */
#define MXC_F_I3C_RDATAB_DATA                          ((uint32_t)(0xFFUL << MXC_F_I3C_RDATAB_DATA_POS)) /**< RDATAB_DATA Mask */

/**@} end of group I3C_RDATAB_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_RDATAH I3C_RDATAH
 * @brief    Read Half-Word Data Register
 * @{
 */
#define MXC_F_I3C_RDATAH_DATA0_POS                     0 /**< RDATAH_DATA0 Position */
#define MXC_F_I3C_RDATAH_DATA0                         ((uint32_t)(0xFFUL << MXC_F_I3C_RDATAH_DATA0_POS)) /**< RDATAH_DATA0 Mask */

#define MXC_F_I3C_RDATAH_DATA1_POS                     8 /**< RDATAH_DATA1 Position */
#define MXC_F_I3C_RDATAH_DATA1                         ((uint32_t)(0xFFUL << MXC_F_I3C_RDATAH_DATA1_POS)) /**< RDATAH_DATA1 Mask */

/**@} end of group I3C_RDATAH_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_WDATAB1 I3C_WDATAB1
 * @brief    Byte-Only Write Byte Data Register
 * @{
 */
#define MXC_F_I3C_WDATAB1_DATA_POS                     0 /**< WDATAB1_DATA Position */
#define MXC_F_I3C_WDATAB1_DATA                         ((uint32_t)(0xFFUL << MXC_F_I3C_WDATAB1_DATA_POS)) /**< WDATAB1_DATA Mask */

/**@} end of group I3C_WDATAB1_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_CAPABILITIES2 I3C_CAPABILITIES2
 * @brief    Capabilities 2 Register
 * @{
 */
#define MXC_F_I3C_CAPABILITIES2_MAPCNT_POS             0 /**< CAPABILITIES2_MAPCNT Position */
#define MXC_F_I3C_CAPABILITIES2_MAPCNT                 ((uint32_t)(0xFUL << MXC_F_I3C_CAPABILITIES2_MAPCNT_POS)) /**< CAPABILITIES2_MAPCNT Mask */

#define MXC_F_I3C_CAPABILITIES2_I2C10B_POS             4 /**< CAPABILITIES2_I2C10B Position */
#define MXC_F_I3C_CAPABILITIES2_I2C10B                 ((uint32_t)(0x1UL << MXC_F_I3C_CAPABILITIES2_I2C10B_POS)) /**< CAPABILITIES2_I2C10B Mask */

#define MXC_F_I3C_CAPABILITIES2_I2CRST_POS             5 /**< CAPABILITIES2_I2CRST Position */
#define MXC_F_I3C_CAPABILITIES2_I2CRST                 ((uint32_t)(0x1UL << MXC_F_I3C_CAPABILITIES2_I2CRST_POS)) /**< CAPABILITIES2_I2CRST Mask */

#define MXC_F_I3C_CAPABILITIES2_I2CDEVID_POS           6 /**< CAPABILITIES2_I2CDEVID Position */
#define MXC_F_I3C_CAPABILITIES2_I2CDEVID               ((uint32_t)(0x1UL << MXC_F_I3C_CAPABILITIES2_I2CDEVID_POS)) /**< CAPABILITIES2_I2CDEVID Mask */

#define MXC_F_I3C_CAPABILITIES2_DATA32_POS             7 /**< CAPABILITIES2_DATA32 Position */
#define MXC_F_I3C_CAPABILITIES2_DATA32                 ((uint32_t)(0x1UL << MXC_F_I3C_CAPABILITIES2_DATA32_POS)) /**< CAPABILITIES2_DATA32 Mask */

#define MXC_F_I3C_CAPABILITIES2_IBIEXT_POS             8 /**< CAPABILITIES2_IBIEXT Position */
#define MXC_F_I3C_CAPABILITIES2_IBIEXT                 ((uint32_t)(0x1UL << MXC_F_I3C_CAPABILITIES2_IBIEXT_POS)) /**< CAPABILITIES2_IBIEXT Mask */

#define MXC_F_I3C_CAPABILITIES2_IBIXREG_POS            9 /**< CAPABILITIES2_IBIXREG Position */
#define MXC_F_I3C_CAPABILITIES2_IBIXREG                ((uint32_t)(0x1UL << MXC_F_I3C_CAPABILITIES2_IBIXREG_POS)) /**< CAPABILITIES2_IBIXREG Mask */

#define MXC_F_I3C_CAPABILITIES2_SMLANE_POS             12 /**< CAPABILITIES2_SMLANE Position */
#define MXC_F_I3C_CAPABILITIES2_SMLANE                 ((uint32_t)(0x3UL << MXC_F_I3C_CAPABILITIES2_SMLANE_POS)) /**< CAPABILITIES2_SMLANE Mask */
#define MXC_V_I3C_CAPABILITIES2_SMLANE_S_LANE          ((uint32_t)0x0UL) /**< CAPABILITIES2_SMLANE_S_LANE Value */
#define MXC_S_I3C_CAPABILITIES2_SMLANE_S_LANE          (MXC_V_I3C_CAPABILITIES2_SMLANE_S_LANE << MXC_F_I3C_CAPABILITIES2_SMLANE_POS) /**< CAPABILITIES2_SMLANE_S_LANE Setting */
#define MXC_V_I3C_CAPABILITIES2_SMLANE_SD_LANE         ((uint32_t)0x1UL) /**< CAPABILITIES2_SMLANE_SD_LANE Value */
#define MXC_S_I3C_CAPABILITIES2_SMLANE_SD_LANE         (MXC_V_I3C_CAPABILITIES2_SMLANE_SD_LANE << MXC_F_I3C_CAPABILITIES2_SMLANE_POS) /**< CAPABILITIES2_SMLANE_SD_LANE Setting */
#define MXC_V_I3C_CAPABILITIES2_SMLANE_SDQ_LANE        ((uint32_t)0x3UL) /**< CAPABILITIES2_SMLANE_SDQ_LANE Value */
#define MXC_S_I3C_CAPABILITIES2_SMLANE_SDQ_LANE        (MXC_V_I3C_CAPABILITIES2_SMLANE_SDQ_LANE << MXC_F_I3C_CAPABILITIES2_SMLANE_POS) /**< CAPABILITIES2_SMLANE_SDQ_LANE Setting */

#define MXC_F_I3C_CAPABILITIES2_V1_1_POS               16 /**< CAPABILITIES2_V1_1 Position */
#define MXC_F_I3C_CAPABILITIES2_V1_1                   ((uint32_t)(0x1UL << MXC_F_I3C_CAPABILITIES2_V1_1_POS)) /**< CAPABILITIES2_V1_1 Mask */

#define MXC_F_I3C_CAPABILITIES2_TGRST_POS              17 /**< CAPABILITIES2_TGRST Position */
#define MXC_F_I3C_CAPABILITIES2_TGRST                  ((uint32_t)(0x1UL << MXC_F_I3C_CAPABILITIES2_TGRST_POS)) /**< CAPABILITIES2_TGRST Mask */

#define MXC_F_I3C_CAPABILITIES2_GROUP_POS              18 /**< CAPABILITIES2_GROUP Position */
#define MXC_F_I3C_CAPABILITIES2_GROUP                  ((uint32_t)(0x3UL << MXC_F_I3C_CAPABILITIES2_GROUP_POS)) /**< CAPABILITIES2_GROUP Mask */
#define MXC_V_I3C_CAPABILITIES2_GROUP_NONE             ((uint32_t)0x0UL) /**< CAPABILITIES2_GROUP_NONE Value */
#define MXC_S_I3C_CAPABILITIES2_GROUP_NONE             (MXC_V_I3C_CAPABILITIES2_GROUP_NONE << MXC_F_I3C_CAPABILITIES2_GROUP_POS) /**< CAPABILITIES2_GROUP_NONE Setting */
#define MXC_V_I3C_CAPABILITIES2_GROUP_ONE              ((uint32_t)0x1UL) /**< CAPABILITIES2_GROUP_ONE Value */
#define MXC_S_I3C_CAPABILITIES2_GROUP_ONE              (MXC_V_I3C_CAPABILITIES2_GROUP_ONE << MXC_F_I3C_CAPABILITIES2_GROUP_POS) /**< CAPABILITIES2_GROUP_ONE Setting */
#define MXC_V_I3C_CAPABILITIES2_GROUP_TWO              ((uint32_t)0x2UL) /**< CAPABILITIES2_GROUP_TWO Value */
#define MXC_S_I3C_CAPABILITIES2_GROUP_TWO              (MXC_V_I3C_CAPABILITIES2_GROUP_TWO << MXC_F_I3C_CAPABILITIES2_GROUP_POS) /**< CAPABILITIES2_GROUP_TWO Setting */
#define MXC_V_I3C_CAPABILITIES2_GROUP_THREE            ((uint32_t)0x3UL) /**< CAPABILITIES2_GROUP_THREE Value */
#define MXC_S_I3C_CAPABILITIES2_GROUP_THREE            (MXC_V_I3C_CAPABILITIES2_GROUP_THREE << MXC_F_I3C_CAPABILITIES2_GROUP_POS) /**< CAPABILITIES2_GROUP_THREE Setting */

#define MXC_F_I3C_CAPABILITIES2_AASA_POS               21 /**< CAPABILITIES2_AASA Position */
#define MXC_F_I3C_CAPABILITIES2_AASA                   ((uint32_t)(0x1UL << MXC_F_I3C_CAPABILITIES2_AASA_POS)) /**< CAPABILITIES2_AASA Mask */

#define MXC_F_I3C_CAPABILITIES2_SSTSUB_POS             22 /**< CAPABILITIES2_SSTSUB Position */
#define MXC_F_I3C_CAPABILITIES2_SSTSUB                 ((uint32_t)(0x1UL << MXC_F_I3C_CAPABILITIES2_SSTSUB_POS)) /**< CAPABILITIES2_SSTSUB Mask */

#define MXC_F_I3C_CAPABILITIES2_SSTWR_POS              23 /**< CAPABILITIES2_SSTWR Position */
#define MXC_F_I3C_CAPABILITIES2_SSTWR                  ((uint32_t)(0x1UL << MXC_F_I3C_CAPABILITIES2_SSTWR_POS)) /**< CAPABILITIES2_SSTWR Mask */

/**@} end of group I3C_CAPABILITIES2_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_CAPABILITIES I3C_CAPABILITIES
 * @brief    Capabilities Register
 * @{
 */
#define MXC_F_I3C_CAPABILITIES_IDENA_POS               0 /**< CAPABILITIES_IDENA Position */
#define MXC_F_I3C_CAPABILITIES_IDENA                   ((uint32_t)(0x3UL << MXC_F_I3C_CAPABILITIES_IDENA_POS)) /**< CAPABILITIES_IDENA Mask */
#define MXC_V_I3C_CAPABILITIES_IDENA_PROV_PROG         ((uint32_t)0x0UL) /**< CAPABILITIES_IDENA_PROV_PROG Value */
#define MXC_S_I3C_CAPABILITIES_IDENA_PROV_PROG         (MXC_V_I3C_CAPABILITIES_IDENA_PROV_PROG << MXC_F_I3C_CAPABILITIES_IDENA_POS) /**< CAPABILITIES_IDENA_PROV_PROG Setting */
#define MXC_V_I3C_CAPABILITIES_IDENA_PROV_SET          ((uint32_t)0x1UL) /**< CAPABILITIES_IDENA_PROV_SET Value */
#define MXC_S_I3C_CAPABILITIES_IDENA_PROV_SET          (MXC_V_I3C_CAPABILITIES_IDENA_PROV_SET << MXC_F_I3C_CAPABILITIES_IDENA_POS) /**< CAPABILITIES_IDENA_PROV_SET Setting */
#define MXC_V_I3C_CAPABILITIES_IDENA_INST_PROG         ((uint32_t)0x2UL) /**< CAPABILITIES_IDENA_INST_PROG Value */
#define MXC_S_I3C_CAPABILITIES_IDENA_INST_PROG         (MXC_V_I3C_CAPABILITIES_IDENA_INST_PROG << MXC_F_I3C_CAPABILITIES_IDENA_POS) /**< CAPABILITIES_IDENA_INST_PROG Setting */
#define MXC_V_I3C_CAPABILITIES_IDENA_PART_PROG         ((uint32_t)0x3UL) /**< CAPABILITIES_IDENA_PART_PROG Value */
#define MXC_S_I3C_CAPABILITIES_IDENA_PART_PROG         (MXC_V_I3C_CAPABILITIES_IDENA_PART_PROG << MXC_F_I3C_CAPABILITIES_IDENA_POS) /**< CAPABILITIES_IDENA_PART_PROG Setting */

#define MXC_F_I3C_CAPABILITIES_IDREG_POS               2 /**< CAPABILITIES_IDREG Position */
#define MXC_F_I3C_CAPABILITIES_IDREG                   ((uint32_t)(0xFUL << MXC_F_I3C_CAPABILITIES_IDREG_POS)) /**< CAPABILITIES_IDREG Mask */

#define MXC_F_I3C_CAPABILITIES_HDRSUPP_POS             6 /**< CAPABILITIES_HDRSUPP Position */
#define MXC_F_I3C_CAPABILITIES_HDRSUPP                 ((uint32_t)(0x7UL << MXC_F_I3C_CAPABILITIES_HDRSUPP_POS)) /**< CAPABILITIES_HDRSUPP Mask */

#define MXC_F_I3C_CAPABILITIES_CNTLR_POS               9 /**< CAPABILITIES_CNTLR Position */
#define MXC_F_I3C_CAPABILITIES_CNTLR                   ((uint32_t)(0x1UL << MXC_F_I3C_CAPABILITIES_CNTLR_POS)) /**< CAPABILITIES_CNTLR Mask */

#define MXC_F_I3C_CAPABILITIES_SADDR_POS               10 /**< CAPABILITIES_SADDR Position */
#define MXC_F_I3C_CAPABILITIES_SADDR                   ((uint32_t)(0x3UL << MXC_F_I3C_CAPABILITIES_SADDR_POS)) /**< CAPABILITIES_SADDR Mask */
#define MXC_V_I3C_CAPABILITIES_SADDR_NONE              ((uint32_t)0x0UL) /**< CAPABILITIES_SADDR_NONE Value */
#define MXC_S_I3C_CAPABILITIES_SADDR_NONE              (MXC_V_I3C_CAPABILITIES_SADDR_NONE << MXC_F_I3C_CAPABILITIES_SADDR_POS) /**< CAPABILITIES_SADDR_NONE Setting */
#define MXC_V_I3C_CAPABILITIES_SADDR_SADDR             ((uint32_t)0x1UL) /**< CAPABILITIES_SADDR_SADDR Value */
#define MXC_S_I3C_CAPABILITIES_SADDR_SADDR             (MXC_V_I3C_CAPABILITIES_SADDR_SADDR << MXC_F_I3C_CAPABILITIES_SADDR_POS) /**< CAPABILITIES_SADDR_SADDR Setting */
#define MXC_V_I3C_CAPABILITIES_SADDR_EXT_TGTSA         ((uint32_t)0x2UL) /**< CAPABILITIES_SADDR_EXT_TGTSA Value */
#define MXC_S_I3C_CAPABILITIES_SADDR_EXT_TGTSA         (MXC_V_I3C_CAPABILITIES_SADDR_EXT_TGTSA << MXC_F_I3C_CAPABILITIES_SADDR_POS) /**< CAPABILITIES_SADDR_EXT_TGTSA Setting */
#define MXC_V_I3C_CAPABILITIES_SADDR_CONFIG            ((uint32_t)0x3UL) /**< CAPABILITIES_SADDR_CONFIG Value */
#define MXC_S_I3C_CAPABILITIES_SADDR_CONFIG            (MXC_V_I3C_CAPABILITIES_SADDR_CONFIG << MXC_F_I3C_CAPABILITIES_SADDR_POS) /**< CAPABILITIES_SADDR_CONFIG Setting */

#define MXC_F_I3C_CAPABILITIES_CCCHANDLE_POS           12 /**< CAPABILITIES_CCCHANDLE Position */
#define MXC_F_I3C_CAPABILITIES_CCCHANDLE               ((uint32_t)(0xFUL << MXC_F_I3C_CAPABILITIES_CCCHANDLE_POS)) /**< CAPABILITIES_CCCHANDLE Mask */

#define MXC_F_I3C_CAPABILITIES_IBI_MR_HJ_POS           16 /**< CAPABILITIES_IBI_MR_HJ Position */
#define MXC_F_I3C_CAPABILITIES_IBI_MR_HJ               ((uint32_t)(0x1FUL << MXC_F_I3C_CAPABILITIES_IBI_MR_HJ_POS)) /**< CAPABILITIES_IBI_MR_HJ Mask */

#define MXC_F_I3C_CAPABILITIES_TIMECTRL_POS            21 /**< CAPABILITIES_TIMECTRL Position */
#define MXC_F_I3C_CAPABILITIES_TIMECTRL                ((uint32_t)(0x1UL << MXC_F_I3C_CAPABILITIES_TIMECTRL_POS)) /**< CAPABILITIES_TIMECTRL Mask */

#define MXC_F_I3C_CAPABILITIES_EXTFIFO_POS             23 /**< CAPABILITIES_EXTFIFO Position */
#define MXC_F_I3C_CAPABILITIES_EXTFIFO                 ((uint32_t)(0x7UL << MXC_F_I3C_CAPABILITIES_EXTFIFO_POS)) /**< CAPABILITIES_EXTFIFO Mask */

#define MXC_F_I3C_CAPABILITIES_FIFOTX_POS              26 /**< CAPABILITIES_FIFOTX Position */
#define MXC_F_I3C_CAPABILITIES_FIFOTX                  ((uint32_t)(0x3UL << MXC_F_I3C_CAPABILITIES_FIFOTX_POS)) /**< CAPABILITIES_FIFOTX Mask */
#define MXC_V_I3C_CAPABILITIES_FIFOTX_NONE             ((uint32_t)0x0UL) /**< CAPABILITIES_FIFOTX_NONE Value */
#define MXC_S_I3C_CAPABILITIES_FIFOTX_NONE             (MXC_V_I3C_CAPABILITIES_FIFOTX_NONE << MXC_F_I3C_CAPABILITIES_FIFOTX_POS) /**< CAPABILITIES_FIFOTX_NONE Setting */
#define MXC_V_I3C_CAPABILITIES_FIFOTX_4BYTE            ((uint32_t)0x1UL) /**< CAPABILITIES_FIFOTX_4BYTE Value */
#define MXC_S_I3C_CAPABILITIES_FIFOTX_4BYTE            (MXC_V_I3C_CAPABILITIES_FIFOTX_4BYTE << MXC_F_I3C_CAPABILITIES_FIFOTX_POS) /**< CAPABILITIES_FIFOTX_4BYTE Setting */
#define MXC_V_I3C_CAPABILITIES_FIFOTX_8BYTE            ((uint32_t)0x2UL) /**< CAPABILITIES_FIFOTX_8BYTE Value */
#define MXC_S_I3C_CAPABILITIES_FIFOTX_8BYTE            (MXC_V_I3C_CAPABILITIES_FIFOTX_8BYTE << MXC_F_I3C_CAPABILITIES_FIFOTX_POS) /**< CAPABILITIES_FIFOTX_8BYTE Setting */
#define MXC_V_I3C_CAPABILITIES_FIFOTX_16BYTE           ((uint32_t)0x3UL) /**< CAPABILITIES_FIFOTX_16BYTE Value */
#define MXC_S_I3C_CAPABILITIES_FIFOTX_16BYTE           (MXC_V_I3C_CAPABILITIES_FIFOTX_16BYTE << MXC_F_I3C_CAPABILITIES_FIFOTX_POS) /**< CAPABILITIES_FIFOTX_16BYTE Setting */

#define MXC_F_I3C_CAPABILITIES_FIFORX_POS              28 /**< CAPABILITIES_FIFORX Position */
#define MXC_F_I3C_CAPABILITIES_FIFORX                  ((uint32_t)(0x3UL << MXC_F_I3C_CAPABILITIES_FIFORX_POS)) /**< CAPABILITIES_FIFORX Mask */
#define MXC_V_I3C_CAPABILITIES_FIFORX_NONE             ((uint32_t)0x0UL) /**< CAPABILITIES_FIFORX_NONE Value */
#define MXC_S_I3C_CAPABILITIES_FIFORX_NONE             (MXC_V_I3C_CAPABILITIES_FIFORX_NONE << MXC_F_I3C_CAPABILITIES_FIFORX_POS) /**< CAPABILITIES_FIFORX_NONE Setting */
#define MXC_V_I3C_CAPABILITIES_FIFORX_4BYTE            ((uint32_t)0x1UL) /**< CAPABILITIES_FIFORX_4BYTE Value */
#define MXC_S_I3C_CAPABILITIES_FIFORX_4BYTE            (MXC_V_I3C_CAPABILITIES_FIFORX_4BYTE << MXC_F_I3C_CAPABILITIES_FIFORX_POS) /**< CAPABILITIES_FIFORX_4BYTE Setting */
#define MXC_V_I3C_CAPABILITIES_FIFORX_8BYTE            ((uint32_t)0x2UL) /**< CAPABILITIES_FIFORX_8BYTE Value */
#define MXC_S_I3C_CAPABILITIES_FIFORX_8BYTE            (MXC_V_I3C_CAPABILITIES_FIFORX_8BYTE << MXC_F_I3C_CAPABILITIES_FIFORX_POS) /**< CAPABILITIES_FIFORX_8BYTE Setting */
#define MXC_V_I3C_CAPABILITIES_FIFORX_16BYTE           ((uint32_t)0x3UL) /**< CAPABILITIES_FIFORX_16BYTE Value */
#define MXC_S_I3C_CAPABILITIES_FIFORX_16BYTE           (MXC_V_I3C_CAPABILITIES_FIFORX_16BYTE << MXC_F_I3C_CAPABILITIES_FIFORX_POS) /**< CAPABILITIES_FIFORX_16BYTE Setting */

#define MXC_F_I3C_CAPABILITIES_INT_POS                 30 /**< CAPABILITIES_INT Position */
#define MXC_F_I3C_CAPABILITIES_INT                     ((uint32_t)(0x1UL << MXC_F_I3C_CAPABILITIES_INT_POS)) /**< CAPABILITIES_INT Mask */

#define MXC_F_I3C_CAPABILITIES_DMA_POS                 31 /**< CAPABILITIES_DMA Position */
#define MXC_F_I3C_CAPABILITIES_DMA                     ((uint32_t)(0x1UL << MXC_F_I3C_CAPABILITIES_DMA_POS)) /**< CAPABILITIES_DMA Mask */

/**@} end of group I3C_CAPABILITIES_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_DYNADDR I3C_DYNADDR
 * @brief    Dynamic Address Register
 * @{
 */
#define MXC_F_I3C_DYNADDR_DAVALID_POS                  0 /**< DYNADDR_DAVALID Position */
#define MXC_F_I3C_DYNADDR_DAVALID                      ((uint32_t)(0x1UL << MXC_F_I3C_DYNADDR_DAVALID_POS)) /**< DYNADDR_DAVALID Mask */

#define MXC_F_I3C_DYNADDR_DADDR_POS                    1 /**< DYNADDR_DADDR Position */
#define MXC_F_I3C_DYNADDR_DADDR                        ((uint32_t)(0x7FUL << MXC_F_I3C_DYNADDR_DADDR_POS)) /**< DYNADDR_DADDR Mask */

#define MXC_F_I3C_DYNADDR_CAUSE_POS                    8 /**< DYNADDR_CAUSE Position */
#define MXC_F_I3C_DYNADDR_CAUSE                        ((uint32_t)(0x7UL << MXC_F_I3C_DYNADDR_CAUSE_POS)) /**< DYNADDR_CAUSE Mask */
#define MXC_V_I3C_DYNADDR_CAUSE_NO_CHANGE              ((uint32_t)0x0UL) /**< DYNADDR_CAUSE_NO_CHANGE Value */
#define MXC_S_I3C_DYNADDR_CAUSE_NO_CHANGE              (MXC_V_I3C_DYNADDR_CAUSE_NO_CHANGE << MXC_F_I3C_DYNADDR_CAUSE_POS) /**< DYNADDR_CAUSE_NO_CHANGE Setting */
#define MXC_V_I3C_DYNADDR_CAUSE_ENTDAA                 ((uint32_t)0x1UL) /**< DYNADDR_CAUSE_ENTDAA Value */
#define MXC_S_I3C_DYNADDR_CAUSE_ENTDAA                 (MXC_V_I3C_DYNADDR_CAUSE_ENTDAA << MXC_F_I3C_DYNADDR_CAUSE_POS) /**< DYNADDR_CAUSE_ENTDAA Setting */
#define MXC_V_I3C_DYNADDR_CAUSE_SETDASA_AASA_NEWDA     ((uint32_t)0x2UL) /**< DYNADDR_CAUSE_SETDASA_AASA_NEWDA Value */
#define MXC_S_I3C_DYNADDR_CAUSE_SETDASA_AASA_NEWDA     (MXC_V_I3C_DYNADDR_CAUSE_SETDASA_AASA_NEWDA << MXC_F_I3C_DYNADDR_CAUSE_POS) /**< DYNADDR_CAUSE_SETDASA_AASA_NEWDA Setting */
#define MXC_V_I3C_DYNADDR_CAUSE_RSTDAA                 ((uint32_t)0x3UL) /**< DYNADDR_CAUSE_RSTDAA Value */
#define MXC_S_I3C_DYNADDR_CAUSE_RSTDAA                 (MXC_V_I3C_DYNADDR_CAUSE_RSTDAA << MXC_F_I3C_DYNADDR_CAUSE_POS) /**< DYNADDR_CAUSE_RSTDAA Setting */
#define MXC_V_I3C_DYNADDR_CAUSE_MAPPED_ADDR_OP         ((uint32_t)0x4UL) /**< DYNADDR_CAUSE_MAPPED_ADDR_OP Value */
#define MXC_S_I3C_DYNADDR_CAUSE_MAPPED_ADDR_OP         (MXC_V_I3C_DYNADDR_CAUSE_MAPPED_ADDR_OP << MXC_F_I3C_DYNADDR_CAUSE_POS) /**< DYNADDR_CAUSE_MAPPED_ADDR_OP Setting */

/**@} end of group I3C_DYNADDR_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_MAXLIMITS I3C_MAXLIMITS
 * @brief    Maximum Limits Register
 * @{
 */
#define MXC_F_I3C_MAXLIMITS_MAXRD_POS                  0 /**< MAXLIMITS_MAXRD Position */
#define MXC_F_I3C_MAXLIMITS_MAXRD                      ((uint32_t)(0xFFFUL << MXC_F_I3C_MAXLIMITS_MAXRD_POS)) /**< MAXLIMITS_MAXRD Mask */

#define MXC_F_I3C_MAXLIMITS_MAXWR_POS                  16 /**< MAXLIMITS_MAXWR Position */
#define MXC_F_I3C_MAXLIMITS_MAXWR                      ((uint32_t)(0xFFFUL << MXC_F_I3C_MAXLIMITS_MAXWR_POS)) /**< MAXLIMITS_MAXWR Mask */

/**@} end of group I3C_MAXLIMITS_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_PARTNO I3C_PARTNO
 * @brief    Part Number Register
 * @{
 */
#define MXC_F_I3C_PARTNO_VENDDEF_POS                   0 /**< PARTNO_VENDDEF Position */
#define MXC_F_I3C_PARTNO_VENDDEF                       ((uint32_t)(0xFFFUL << MXC_F_I3C_PARTNO_VENDDEF_POS)) /**< PARTNO_VENDDEF Mask */

#define MXC_F_I3C_PARTNO_INSTID_POS                    12 /**< PARTNO_INSTID Position */
#define MXC_F_I3C_PARTNO_INSTID                        ((uint32_t)(0xFUL << MXC_F_I3C_PARTNO_INSTID_POS)) /**< PARTNO_INSTID Mask */

#define MXC_F_I3C_PARTNO_PARTID_POS                    16 /**< PARTNO_PARTID Position */
#define MXC_F_I3C_PARTNO_PARTID                        ((uint32_t)(0xFFFFUL << MXC_F_I3C_PARTNO_PARTID_POS)) /**< PARTNO_PARTID Mask */

/**@} end of group I3C_PARTNO_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_IDEXT I3C_IDEXT
 * @brief    ID Extension Register
 * @{
 */
#define MXC_F_I3C_IDEXT_INSTANCE_POS                   0 /**< IDEXT_INSTANCE Position */
#define MXC_F_I3C_IDEXT_INSTANCE                       ((uint32_t)(0xFUL << MXC_F_I3C_IDEXT_INSTANCE_POS)) /**< IDEXT_INSTANCE Mask */

#define MXC_F_I3C_IDEXT_DCR_POS                        8 /**< IDEXT_DCR Position */
#define MXC_F_I3C_IDEXT_DCR                            ((uint32_t)(0xFFUL << MXC_F_I3C_IDEXT_DCR_POS)) /**< IDEXT_DCR Mask */

#define MXC_F_I3C_IDEXT_BCR_POS                        16 /**< IDEXT_BCR Position */
#define MXC_F_I3C_IDEXT_BCR                            ((uint32_t)(0xFFUL << MXC_F_I3C_IDEXT_BCR_POS)) /**< IDEXT_BCR Mask */

/**@} end of group I3C_IDEXT_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_VENDORID I3C_VENDORID
 * @brief    Vendor ID Register
 * @{
 */
#define MXC_F_I3C_VENDORID_VID_POS                     0 /**< VENDORID_VID Position */
#define MXC_F_I3C_VENDORID_VID                         ((uint32_t)(0x7FFFUL << MXC_F_I3C_VENDORID_VID_POS)) /**< VENDORID_VID Mask */

/**@} end of group I3C_VENDORID_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_TCCLOCK I3C_TCCLOCK
 * @brief    Timing Control Clock Register
 * @{
 */
#define MXC_F_I3C_TCCLOCK_ACCURACY_POS                 0 /**< TCCLOCK_ACCURACY Position */
#define MXC_F_I3C_TCCLOCK_ACCURACY                     ((uint32_t)(0xFFUL << MXC_F_I3C_TCCLOCK_ACCURACY_POS)) /**< TCCLOCK_ACCURACY Mask */

#define MXC_F_I3C_TCCLOCK_FREQ_POS                     8 /**< TCCLOCK_FREQ Position */
#define MXC_F_I3C_TCCLOCK_FREQ                         ((uint32_t)(0xFFUL << MXC_F_I3C_TCCLOCK_FREQ_POS)) /**< TCCLOCK_FREQ Mask */

/**@} end of group I3C_TCCLOCK_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_MSGLAST I3C_MSGLAST
 * @brief    Matching Address Index Register
 * @{
 */
#define MXC_F_I3C_MSGLAST_MAPLAST_POS                  0 /**< MSGLAST_MAPLAST Position */
#define MXC_F_I3C_MSGLAST_MAPLAST                      ((uint32_t)(0xFUL << MXC_F_I3C_MSGLAST_MAPLAST_POS)) /**< MSGLAST_MAPLAST Mask */

#define MXC_F_I3C_MSGLAST_LASTSTATIC_POS               4 /**< MSGLAST_LASTSTATIC Position */
#define MXC_F_I3C_MSGLAST_LASTSTATIC                   ((uint32_t)(0x1UL << MXC_F_I3C_MSGLAST_LASTSTATIC_POS)) /**< MSGLAST_LASTSTATIC Mask */

#define MXC_F_I3C_MSGLAST_LASTGROUP_POS                5 /**< MSGLAST_LASTGROUP Position */
#define MXC_F_I3C_MSGLAST_LASTGROUP                    ((uint32_t)(0x1UL << MXC_F_I3C_MSGLAST_LASTGROUP_POS)) /**< MSGLAST_LASTGROUP Mask */

#define MXC_F_I3C_MSGLAST_LASTMODE_POS                 6 /**< MSGLAST_LASTMODE Position */
#define MXC_F_I3C_MSGLAST_LASTMODE                     ((uint32_t)(0x3UL << MXC_F_I3C_MSGLAST_LASTMODE_POS)) /**< MSGLAST_LASTMODE Mask */
#define MXC_V_I3C_MSGLAST_LASTMODE_I3C_SDR_OR_I2C      ((uint32_t)0x0UL) /**< MSGLAST_LASTMODE_I3C_SDR_OR_I2C Value */
#define MXC_S_I3C_MSGLAST_LASTMODE_I3C_SDR_OR_I2C      (MXC_V_I3C_MSGLAST_LASTMODE_I3C_SDR_OR_I2C << MXC_F_I3C_MSGLAST_LASTMODE_POS) /**< MSGLAST_LASTMODE_I3C_SDR_OR_I2C Setting */
#define MXC_V_I3C_MSGLAST_LASTMODE_HDRDDR              ((uint32_t)0x1UL) /**< MSGLAST_LASTMODE_HDRDDR Value */
#define MXC_S_I3C_MSGLAST_LASTMODE_HDRDDR              (MXC_V_I3C_MSGLAST_LASTMODE_HDRDDR << MXC_F_I3C_MSGLAST_LASTMODE_POS) /**< MSGLAST_LASTMODE_HDRDDR Setting */
#define MXC_V_I3C_MSGLAST_LASTMODE_HDRBT               ((uint32_t)0x2UL) /**< MSGLAST_LASTMODE_HDRBT Value */
#define MXC_S_I3C_MSGLAST_LASTMODE_HDRBT               (MXC_V_I3C_MSGLAST_LASTMODE_HDRBT << MXC_F_I3C_MSGLAST_LASTMODE_POS) /**< MSGLAST_LASTMODE_HDRBT Setting */

#define MXC_F_I3C_MSGLAST_MAPLASTM1_POS                8 /**< MSGLAST_MAPLASTM1 Position */
#define MXC_F_I3C_MSGLAST_MAPLASTM1                    ((uint32_t)(0xFUL << MXC_F_I3C_MSGLAST_MAPLASTM1_POS)) /**< MSGLAST_MAPLASTM1 Mask */

#define MXC_F_I3C_MSGLAST_LASTGROUPM1_POS              13 /**< MSGLAST_LASTGROUPM1 Position */
#define MXC_F_I3C_MSGLAST_LASTGROUPM1                  ((uint32_t)(0x1UL << MXC_F_I3C_MSGLAST_LASTGROUPM1_POS)) /**< MSGLAST_LASTGROUPM1 Mask */

#define MXC_F_I3C_MSGLAST_LASTMODE1_POS                14 /**< MSGLAST_LASTMODE1 Position */
#define MXC_F_I3C_MSGLAST_LASTMODE1                    ((uint32_t)(0x3UL << MXC_F_I3C_MSGLAST_LASTMODE1_POS)) /**< MSGLAST_LASTMODE1 Mask */
#define MXC_V_I3C_MSGLAST_LASTMODE1_I3C_SDR_OR_I2C     ((uint32_t)0x0UL) /**< MSGLAST_LASTMODE1_I3C_SDR_OR_I2C Value */
#define MXC_S_I3C_MSGLAST_LASTMODE1_I3C_SDR_OR_I2C     (MXC_V_I3C_MSGLAST_LASTMODE1_I3C_SDR_OR_I2C << MXC_F_I3C_MSGLAST_LASTMODE1_POS) /**< MSGLAST_LASTMODE1_I3C_SDR_OR_I2C Setting */
#define MXC_V_I3C_MSGLAST_LASTMODE1_HDRDDR             ((uint32_t)0x1UL) /**< MSGLAST_LASTMODE1_HDRDDR Value */
#define MXC_S_I3C_MSGLAST_LASTMODE1_HDRDDR             (MXC_V_I3C_MSGLAST_LASTMODE1_HDRDDR << MXC_F_I3C_MSGLAST_LASTMODE1_POS) /**< MSGLAST_LASTMODE1_HDRDDR Setting */
#define MXC_V_I3C_MSGLAST_LASTMODE1_HDRBT              ((uint32_t)0x2UL) /**< MSGLAST_LASTMODE1_HDRBT Value */
#define MXC_S_I3C_MSGLAST_LASTMODE1_HDRBT              (MXC_V_I3C_MSGLAST_LASTMODE1_HDRBT << MXC_F_I3C_MSGLAST_LASTMODE1_POS) /**< MSGLAST_LASTMODE1_HDRBT Setting */

#define MXC_F_I3C_MSGLAST_MAPLASTM2_POS                16 /**< MSGLAST_MAPLASTM2 Position */
#define MXC_F_I3C_MSGLAST_MAPLASTM2                    ((uint32_t)(0xFUL << MXC_F_I3C_MSGLAST_MAPLASTM2_POS)) /**< MSGLAST_MAPLASTM2 Mask */

#define MXC_F_I3C_MSGLAST_LASTGROUPM2_POS              21 /**< MSGLAST_LASTGROUPM2 Position */
#define MXC_F_I3C_MSGLAST_LASTGROUPM2                  ((uint32_t)(0x1UL << MXC_F_I3C_MSGLAST_LASTGROUPM2_POS)) /**< MSGLAST_LASTGROUPM2 Mask */

#define MXC_F_I3C_MSGLAST_LASTMODE2_POS                22 /**< MSGLAST_LASTMODE2 Position */
#define MXC_F_I3C_MSGLAST_LASTMODE2                    ((uint32_t)(0x3UL << MXC_F_I3C_MSGLAST_LASTMODE2_POS)) /**< MSGLAST_LASTMODE2 Mask */
#define MXC_V_I3C_MSGLAST_LASTMODE2_I3C_SDR_OR_I2C     ((uint32_t)0x0UL) /**< MSGLAST_LASTMODE2_I3C_SDR_OR_I2C Value */
#define MXC_S_I3C_MSGLAST_LASTMODE2_I3C_SDR_OR_I2C     (MXC_V_I3C_MSGLAST_LASTMODE2_I3C_SDR_OR_I2C << MXC_F_I3C_MSGLAST_LASTMODE2_POS) /**< MSGLAST_LASTMODE2_I3C_SDR_OR_I2C Setting */
#define MXC_V_I3C_MSGLAST_LASTMODE2_HDRDDR             ((uint32_t)0x1UL) /**< MSGLAST_LASTMODE2_HDRDDR Value */
#define MXC_S_I3C_MSGLAST_LASTMODE2_HDRDDR             (MXC_V_I3C_MSGLAST_LASTMODE2_HDRDDR << MXC_F_I3C_MSGLAST_LASTMODE2_POS) /**< MSGLAST_LASTMODE2_HDRDDR Setting */
#define MXC_V_I3C_MSGLAST_LASTMODE2_HDRBT              ((uint32_t)0x2UL) /**< MSGLAST_LASTMODE2_HDRBT Value */
#define MXC_S_I3C_MSGLAST_LASTMODE2_HDRBT              (MXC_V_I3C_MSGLAST_LASTMODE2_HDRBT << MXC_F_I3C_MSGLAST_LASTMODE2_POS) /**< MSGLAST_LASTMODE2_HDRBT Setting */

/**@} end of group I3C_MSGLAST_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_MCTRL I3C_MCTRL
 * @brief    Controller Control Register
 * @{
 */
#define MXC_F_I3C_MCTRL_REQUEST_POS                    0 /**< MCTRL_REQUEST Position */
#define MXC_F_I3C_MCTRL_REQUEST                        ((uint32_t)(0x7UL << MXC_F_I3C_MCTRL_REQUEST_POS)) /**< MCTRL_REQUEST Mask */
#define MXC_V_I3C_MCTRL_REQUEST_NONE                   ((uint32_t)0x0UL) /**< MCTRL_REQUEST_NONE Value */
#define MXC_S_I3C_MCTRL_REQUEST_NONE                   (MXC_V_I3C_MCTRL_REQUEST_NONE << MXC_F_I3C_MCTRL_REQUEST_POS) /**< MCTRL_REQUEST_NONE Setting */
#define MXC_V_I3C_MCTRL_REQUEST_EMIT_START_ADDR        ((uint32_t)0x1UL) /**< MCTRL_REQUEST_EMIT_START_ADDR Value */
#define MXC_S_I3C_MCTRL_REQUEST_EMIT_START_ADDR        (MXC_V_I3C_MCTRL_REQUEST_EMIT_START_ADDR << MXC_F_I3C_MCTRL_REQUEST_POS) /**< MCTRL_REQUEST_EMIT_START_ADDR Setting */
#define MXC_V_I3C_MCTRL_REQUEST_EMIT_STOP              ((uint32_t)0x2UL) /**< MCTRL_REQUEST_EMIT_STOP Value */
#define MXC_S_I3C_MCTRL_REQUEST_EMIT_STOP              (MXC_V_I3C_MCTRL_REQUEST_EMIT_STOP << MXC_F_I3C_MCTRL_REQUEST_POS) /**< MCTRL_REQUEST_EMIT_STOP Setting */
#define MXC_V_I3C_MCTRL_REQUEST_IBI_ACK_NACK           ((uint32_t)0x3UL) /**< MCTRL_REQUEST_IBI_ACK_NACK Value */
#define MXC_S_I3C_MCTRL_REQUEST_IBI_ACK_NACK           (MXC_V_I3C_MCTRL_REQUEST_IBI_ACK_NACK << MXC_F_I3C_MCTRL_REQUEST_POS) /**< MCTRL_REQUEST_IBI_ACK_NACK Setting */
#define MXC_V_I3C_MCTRL_REQUEST_PROCESS_DAA            ((uint32_t)0x4UL) /**< MCTRL_REQUEST_PROCESS_DAA Value */
#define MXC_S_I3C_MCTRL_REQUEST_PROCESS_DAA            (MXC_V_I3C_MCTRL_REQUEST_PROCESS_DAA << MXC_F_I3C_MCTRL_REQUEST_POS) /**< MCTRL_REQUEST_PROCESS_DAA Setting */
#define MXC_V_I3C_MCTRL_REQUEST_FORCE_EXIT_RESET       ((uint32_t)0x6UL) /**< MCTRL_REQUEST_FORCE_EXIT_RESET Value */
#define MXC_S_I3C_MCTRL_REQUEST_FORCE_EXIT_RESET       (MXC_V_I3C_MCTRL_REQUEST_FORCE_EXIT_RESET << MXC_F_I3C_MCTRL_REQUEST_POS) /**< MCTRL_REQUEST_FORCE_EXIT_RESET Setting */
#define MXC_V_I3C_MCTRL_REQUEST_AUTO_IBI               ((uint32_t)0x7UL) /**< MCTRL_REQUEST_AUTO_IBI Value */
#define MXC_S_I3C_MCTRL_REQUEST_AUTO_IBI               (MXC_V_I3C_MCTRL_REQUEST_AUTO_IBI << MXC_F_I3C_MCTRL_REQUEST_POS) /**< MCTRL_REQUEST_AUTO_IBI Setting */

#define MXC_F_I3C_MCTRL_TYPE_POS                       4 /**< MCTRL_TYPE Position */
#define MXC_F_I3C_MCTRL_TYPE                           ((uint32_t)(0x3UL << MXC_F_I3C_MCTRL_TYPE_POS)) /**< MCTRL_TYPE Mask */

#define MXC_F_I3C_MCTRL_IBIRESP_POS                    6 /**< MCTRL_IBIRESP Position */
#define MXC_F_I3C_MCTRL_IBIRESP                        ((uint32_t)(0x3UL << MXC_F_I3C_MCTRL_IBIRESP_POS)) /**< MCTRL_IBIRESP Mask */

#define MXC_F_I3C_MCTRL_DIR_POS                        8 /**< MCTRL_DIR Position */
#define MXC_F_I3C_MCTRL_DIR                            ((uint32_t)(0x1UL << MXC_F_I3C_MCTRL_DIR_POS)) /**< MCTRL_DIR Mask */

#define MXC_F_I3C_MCTRL_ADDR_POS                       9 /**< MCTRL_ADDR Position */
#define MXC_F_I3C_MCTRL_ADDR                           ((uint32_t)(0x7UL << MXC_F_I3C_MCTRL_ADDR_POS)) /**< MCTRL_ADDR Mask */

#define MXC_F_I3C_MCTRL_RDTERM_POS                     16 /**< MCTRL_RDTERM Position */
#define MXC_F_I3C_MCTRL_RDTERM                         ((uint32_t)(0xFUL << MXC_F_I3C_MCTRL_RDTERM_POS)) /**< MCTRL_RDTERM Mask */

/**@} end of group I3C_MCTRL_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_MSTATUS I3C_MSTATUS
 * @brief    Controller Status Register
 * @{
 */
#define MXC_F_I3C_MSTATUS_STATE_POS                    0 /**< MSTATUS_STATE Position */
#define MXC_F_I3C_MSTATUS_STATE                        ((uint32_t)(0x7UL << MXC_F_I3C_MSTATUS_STATE_POS)) /**< MSTATUS_STATE Mask */
#define MXC_V_I3C_MSTATUS_STATE_IDLE                   ((uint32_t)0x0UL) /**< MSTATUS_STATE_IDLE Value */
#define MXC_S_I3C_MSTATUS_STATE_IDLE                   (MXC_V_I3C_MSTATUS_STATE_IDLE << MXC_F_I3C_MSTATUS_STATE_POS) /**< MSTATUS_STATE_IDLE Setting */
#define MXC_V_I3C_MSTATUS_STATE_TGTREQ                 ((uint32_t)0x1UL) /**< MSTATUS_STATE_TGTREQ Value */
#define MXC_S_I3C_MSTATUS_STATE_TGTREQ                 (MXC_V_I3C_MSTATUS_STATE_TGTREQ << MXC_F_I3C_MSTATUS_STATE_POS) /**< MSTATUS_STATE_TGTREQ Setting */
#define MXC_V_I3C_MSTATUS_STATE_MSGSDR                 ((uint32_t)0x2UL) /**< MSTATUS_STATE_MSGSDR Value */
#define MXC_S_I3C_MSTATUS_STATE_MSGSDR                 (MXC_V_I3C_MSTATUS_STATE_MSGSDR << MXC_F_I3C_MSTATUS_STATE_POS) /**< MSTATUS_STATE_MSGSDR Setting */
#define MXC_V_I3C_MSTATUS_STATE_NORMACT                ((uint32_t)0x3UL) /**< MSTATUS_STATE_NORMACT Value */
#define MXC_S_I3C_MSTATUS_STATE_NORMACT                (MXC_V_I3C_MSTATUS_STATE_NORMACT << MXC_F_I3C_MSTATUS_STATE_POS) /**< MSTATUS_STATE_NORMACT Setting */
#define MXC_V_I3C_MSTATUS_STATE_DDR                    ((uint32_t)0x4UL) /**< MSTATUS_STATE_DDR Value */
#define MXC_S_I3C_MSTATUS_STATE_DDR                    (MXC_V_I3C_MSTATUS_STATE_DDR << MXC_F_I3C_MSTATUS_STATE_POS) /**< MSTATUS_STATE_DDR Setting */
#define MXC_V_I3C_MSTATUS_STATE_DAA                    ((uint32_t)0x5UL) /**< MSTATUS_STATE_DAA Value */
#define MXC_S_I3C_MSTATUS_STATE_DAA                    (MXC_V_I3C_MSTATUS_STATE_DAA << MXC_F_I3C_MSTATUS_STATE_POS) /**< MSTATUS_STATE_DAA Setting */
#define MXC_V_I3C_MSTATUS_STATE_IBIACK                 ((uint32_t)0x6UL) /**< MSTATUS_STATE_IBIACK Value */
#define MXC_S_I3C_MSTATUS_STATE_IBIACK                 (MXC_V_I3C_MSTATUS_STATE_IBIACK << MXC_F_I3C_MSTATUS_STATE_POS) /**< MSTATUS_STATE_IBIACK Setting */
#define MXC_V_I3C_MSTATUS_STATE_IBIRCV                 ((uint32_t)0x7UL) /**< MSTATUS_STATE_IBIRCV Value */
#define MXC_S_I3C_MSTATUS_STATE_IBIRCV                 (MXC_V_I3C_MSTATUS_STATE_IBIRCV << MXC_F_I3C_MSTATUS_STATE_POS) /**< MSTATUS_STATE_IBIRCV Setting */

#define MXC_F_I3C_MSTATUS_BETWEEN_POS                  4 /**< MSTATUS_BETWEEN Position */
#define MXC_F_I3C_MSTATUS_BETWEEN                      ((uint32_t)(0x1UL << MXC_F_I3C_MSTATUS_BETWEEN_POS)) /**< MSTATUS_BETWEEN Mask */

#define MXC_F_I3C_MSTATUS_NACKED_POS                   5 /**< MSTATUS_NACKED Position */
#define MXC_F_I3C_MSTATUS_NACKED                       ((uint32_t)(0x1UL << MXC_F_I3C_MSTATUS_NACKED_POS)) /**< MSTATUS_NACKED Mask */

#define MXC_F_I3C_MSTATUS_IBITYPE_POS                  6 /**< MSTATUS_IBITYPE Position */
#define MXC_F_I3C_MSTATUS_IBITYPE                      ((uint32_t)(0x3UL << MXC_F_I3C_MSTATUS_IBITYPE_POS)) /**< MSTATUS_IBITYPE Mask */
#define MXC_V_I3C_MSTATUS_IBITYPE_NONE                 ((uint32_t)0x0UL) /**< MSTATUS_IBITYPE_NONE Value */
#define MXC_S_I3C_MSTATUS_IBITYPE_NONE                 (MXC_V_I3C_MSTATUS_IBITYPE_NONE << MXC_F_I3C_MSTATUS_IBITYPE_POS) /**< MSTATUS_IBITYPE_NONE Setting */
#define MXC_V_I3C_MSTATUS_IBITYPE_IBI                  ((uint32_t)0x1UL) /**< MSTATUS_IBITYPE_IBI Value */
#define MXC_S_I3C_MSTATUS_IBITYPE_IBI                  (MXC_V_I3C_MSTATUS_IBITYPE_IBI << MXC_F_I3C_MSTATUS_IBITYPE_POS) /**< MSTATUS_IBITYPE_IBI Setting */
#define MXC_V_I3C_MSTATUS_IBITYPE_CONTROLLER_REQ       ((uint32_t)0x2UL) /**< MSTATUS_IBITYPE_CONTROLLER_REQ Value */
#define MXC_S_I3C_MSTATUS_IBITYPE_CONTROLLER_REQ       (MXC_V_I3C_MSTATUS_IBITYPE_CONTROLLER_REQ << MXC_F_I3C_MSTATUS_IBITYPE_POS) /**< MSTATUS_IBITYPE_CONTROLLER_REQ Setting */
#define MXC_V_I3C_MSTATUS_IBITYPE_HOTJOIN_REQ          ((uint32_t)0x3UL) /**< MSTATUS_IBITYPE_HOTJOIN_REQ Value */
#define MXC_S_I3C_MSTATUS_IBITYPE_HOTJOIN_REQ          (MXC_V_I3C_MSTATUS_IBITYPE_HOTJOIN_REQ << MXC_F_I3C_MSTATUS_IBITYPE_POS) /**< MSTATUS_IBITYPE_HOTJOIN_REQ Setting */

#define MXC_F_I3C_MSTATUS_TGTSTART_POS                 8 /**< MSTATUS_TGTSTART Position */
#define MXC_F_I3C_MSTATUS_TGTSTART                     ((uint32_t)(0x1UL << MXC_F_I3C_MSTATUS_TGTSTART_POS)) /**< MSTATUS_TGTSTART Mask */

#define MXC_F_I3C_MSTATUS_MCTRLDONE_POS                9 /**< MSTATUS_MCTRLDONE Position */
#define MXC_F_I3C_MSTATUS_MCTRLDONE                    ((uint32_t)(0x1UL << MXC_F_I3C_MSTATUS_MCTRLDONE_POS)) /**< MSTATUS_MCTRLDONE Mask */

#define MXC_F_I3C_MSTATUS_COMPLETE_POS                 10 /**< MSTATUS_COMPLETE Position */
#define MXC_F_I3C_MSTATUS_COMPLETE                     ((uint32_t)(0x1UL << MXC_F_I3C_MSTATUS_COMPLETE_POS)) /**< MSTATUS_COMPLETE Mask */

#define MXC_F_I3C_MSTATUS_RXPEND_POS                   11 /**< MSTATUS_RXPEND Position */
#define MXC_F_I3C_MSTATUS_RXPEND                       ((uint32_t)(0x1UL << MXC_F_I3C_MSTATUS_RXPEND_POS)) /**< MSTATUS_RXPEND Mask */

#define MXC_F_I3C_MSTATUS_TXNOTFULL_POS                12 /**< MSTATUS_TXNOTFULL Position */
#define MXC_F_I3C_MSTATUS_TXNOTFULL                    ((uint32_t)(0x1UL << MXC_F_I3C_MSTATUS_TXNOTFULL_POS)) /**< MSTATUS_TXNOTFULL Mask */

#define MXC_F_I3C_MSTATUS_IBIWON_POS                   13 /**< MSTATUS_IBIWON Position */
#define MXC_F_I3C_MSTATUS_IBIWON                       ((uint32_t)(0x1UL << MXC_F_I3C_MSTATUS_IBIWON_POS)) /**< MSTATUS_IBIWON Mask */

#define MXC_F_I3C_MSTATUS_ERRWARN_POS                  15 /**< MSTATUS_ERRWARN Position */
#define MXC_F_I3C_MSTATUS_ERRWARN                      ((uint32_t)(0x1UL << MXC_F_I3C_MSTATUS_ERRWARN_POS)) /**< MSTATUS_ERRWARN Mask */

#define MXC_F_I3C_MSTATUS_NOWCNTLR_POS                 19 /**< MSTATUS_NOWCNTLR Position */
#define MXC_F_I3C_MSTATUS_NOWCNTLR                     ((uint32_t)(0x1UL << MXC_F_I3C_MSTATUS_NOWCNTLR_POS)) /**< MSTATUS_NOWCNTLR Mask */

#define MXC_F_I3C_MSTATUS_IBIADDR_POS                  24 /**< MSTATUS_IBIADDR Position */
#define MXC_F_I3C_MSTATUS_IBIADDR                      ((uint32_t)(0x7FUL << MXC_F_I3C_MSTATUS_IBIADDR_POS)) /**< MSTATUS_IBIADDR Mask */

/**@} end of group I3C_MSTATUS_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_IBIRULES I3C_IBIRULES
 * @brief    IBI Registry and Rules Register
 * @{
 */
#define MXC_F_I3C_IBIRULES_ADDR0_POS                   0 /**< IBIRULES_ADDR0 Position */
#define MXC_F_I3C_IBIRULES_ADDR0                       ((uint32_t)(0x3FUL << MXC_F_I3C_IBIRULES_ADDR0_POS)) /**< IBIRULES_ADDR0 Mask */

#define MXC_F_I3C_IBIRULES_ADDR1_POS                   6 /**< IBIRULES_ADDR1 Position */
#define MXC_F_I3C_IBIRULES_ADDR1                       ((uint32_t)(0x3FUL << MXC_F_I3C_IBIRULES_ADDR1_POS)) /**< IBIRULES_ADDR1 Mask */

#define MXC_F_I3C_IBIRULES_ADDR2_POS                   12 /**< IBIRULES_ADDR2 Position */
#define MXC_F_I3C_IBIRULES_ADDR2                       ((uint32_t)(0x3FUL << MXC_F_I3C_IBIRULES_ADDR2_POS)) /**< IBIRULES_ADDR2 Mask */

#define MXC_F_I3C_IBIRULES_ADDR3_POS                   18 /**< IBIRULES_ADDR3 Position */
#define MXC_F_I3C_IBIRULES_ADDR3                       ((uint32_t)(0x3FUL << MXC_F_I3C_IBIRULES_ADDR3_POS)) /**< IBIRULES_ADDR3 Mask */

#define MXC_F_I3C_IBIRULES_ADDR4_POS                   24 /**< IBIRULES_ADDR4 Position */
#define MXC_F_I3C_IBIRULES_ADDR4                       ((uint32_t)(0x3FUL << MXC_F_I3C_IBIRULES_ADDR4_POS)) /**< IBIRULES_ADDR4 Mask */

#define MXC_F_I3C_IBIRULES_MSB0_POS                    30 /**< IBIRULES_MSB0 Position */
#define MXC_F_I3C_IBIRULES_MSB0                        ((uint32_t)(0x1UL << MXC_F_I3C_IBIRULES_MSB0_POS)) /**< IBIRULES_MSB0 Mask */

#define MXC_F_I3C_IBIRULES_NOBYTE_POS                  31 /**< IBIRULES_NOBYTE Position */
#define MXC_F_I3C_IBIRULES_NOBYTE                      ((uint32_t)(0x1UL << MXC_F_I3C_IBIRULES_NOBYTE_POS)) /**< IBIRULES_NOBYTE Mask */

/**@} end of group I3C_IBIRULES_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_MINTSET I3C_MINTSET
 * @brief    Controller Interrupt Enable Set Register
 * @{
 */
#define MXC_F_I3C_MINTSET_TGTSTART_POS                 8 /**< MINTSET_TGTSTART Position */
#define MXC_F_I3C_MINTSET_TGTSTART                     ((uint32_t)(0x1UL << MXC_F_I3C_MINTSET_TGTSTART_POS)) /**< MINTSET_TGTSTART Mask */

#define MXC_F_I3C_MINTSET_MCTRLDONE_POS                9 /**< MINTSET_MCTRLDONE Position */
#define MXC_F_I3C_MINTSET_MCTRLDONE                    ((uint32_t)(0x1UL << MXC_F_I3C_MINTSET_MCTRLDONE_POS)) /**< MINTSET_MCTRLDONE Mask */

#define MXC_F_I3C_MINTSET_COMPLETE_POS                 10 /**< MINTSET_COMPLETE Position */
#define MXC_F_I3C_MINTSET_COMPLETE                     ((uint32_t)(0x1UL << MXC_F_I3C_MINTSET_COMPLETE_POS)) /**< MINTSET_COMPLETE Mask */

#define MXC_F_I3C_MINTSET_RXPEND_POS                   11 /**< MINTSET_RXPEND Position */
#define MXC_F_I3C_MINTSET_RXPEND                       ((uint32_t)(0x1UL << MXC_F_I3C_MINTSET_RXPEND_POS)) /**< MINTSET_RXPEND Mask */

#define MXC_F_I3C_MINTSET_TXNOTFULL_POS                12 /**< MINTSET_TXNOTFULL Position */
#define MXC_F_I3C_MINTSET_TXNOTFULL                    ((uint32_t)(0x1UL << MXC_F_I3C_MINTSET_TXNOTFULL_POS)) /**< MINTSET_TXNOTFULL Mask */

#define MXC_F_I3C_MINTSET_IBIWON_POS                   13 /**< MINTSET_IBIWON Position */
#define MXC_F_I3C_MINTSET_IBIWON                       ((uint32_t)(0x1UL << MXC_F_I3C_MINTSET_IBIWON_POS)) /**< MINTSET_IBIWON Mask */

#define MXC_F_I3C_MINTSET_ERRWARN_POS                  15 /**< MINTSET_ERRWARN Position */
#define MXC_F_I3C_MINTSET_ERRWARN                      ((uint32_t)(0x1UL << MXC_F_I3C_MINTSET_ERRWARN_POS)) /**< MINTSET_ERRWARN Mask */

#define MXC_F_I3C_MINTSET_NOWCNTLR_POS                 19 /**< MINTSET_NOWCNTLR Position */
#define MXC_F_I3C_MINTSET_NOWCNTLR                     ((uint32_t)(0x1UL << MXC_F_I3C_MINTSET_NOWCNTLR_POS)) /**< MINTSET_NOWCNTLR Mask */

/**@} end of group I3C_MINTSET_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_MINTCLR I3C_MINTCLR
 * @brief    Controller Interrupt Enable Clear Register
 * @{
 */
#define MXC_F_I3C_MINTCLR_TGTSTART_POS                 8 /**< MINTCLR_TGTSTART Position */
#define MXC_F_I3C_MINTCLR_TGTSTART                     ((uint32_t)(0x1UL << MXC_F_I3C_MINTCLR_TGTSTART_POS)) /**< MINTCLR_TGTSTART Mask */

#define MXC_F_I3C_MINTCLR_MCTRLDONE_POS                9 /**< MINTCLR_MCTRLDONE Position */
#define MXC_F_I3C_MINTCLR_MCTRLDONE                    ((uint32_t)(0x1UL << MXC_F_I3C_MINTCLR_MCTRLDONE_POS)) /**< MINTCLR_MCTRLDONE Mask */

#define MXC_F_I3C_MINTCLR_COMPLETE_POS                 10 /**< MINTCLR_COMPLETE Position */
#define MXC_F_I3C_MINTCLR_COMPLETE                     ((uint32_t)(0x1UL << MXC_F_I3C_MINTCLR_COMPLETE_POS)) /**< MINTCLR_COMPLETE Mask */

#define MXC_F_I3C_MINTCLR_RXPEND_POS                   11 /**< MINTCLR_RXPEND Position */
#define MXC_F_I3C_MINTCLR_RXPEND                       ((uint32_t)(0x1UL << MXC_F_I3C_MINTCLR_RXPEND_POS)) /**< MINTCLR_RXPEND Mask */

#define MXC_F_I3C_MINTCLR_TXNOTFULL_POS                12 /**< MINTCLR_TXNOTFULL Position */
#define MXC_F_I3C_MINTCLR_TXNOTFULL                    ((uint32_t)(0x1UL << MXC_F_I3C_MINTCLR_TXNOTFULL_POS)) /**< MINTCLR_TXNOTFULL Mask */

#define MXC_F_I3C_MINTCLR_IBIWON_POS                   13 /**< MINTCLR_IBIWON Position */
#define MXC_F_I3C_MINTCLR_IBIWON                       ((uint32_t)(0x1UL << MXC_F_I3C_MINTCLR_IBIWON_POS)) /**< MINTCLR_IBIWON Mask */

#define MXC_F_I3C_MINTCLR_ERRWARN_POS                  15 /**< MINTCLR_ERRWARN Position */
#define MXC_F_I3C_MINTCLR_ERRWARN                      ((uint32_t)(0x1UL << MXC_F_I3C_MINTCLR_ERRWARN_POS)) /**< MINTCLR_ERRWARN Mask */

#define MXC_F_I3C_MINTCLR_NOWCNTLR_POS                 19 /**< MINTCLR_NOWCNTLR Position */
#define MXC_F_I3C_MINTCLR_NOWCNTLR                     ((uint32_t)(0x1UL << MXC_F_I3C_MINTCLR_NOWCNTLR_POS)) /**< MINTCLR_NOWCNTLR Mask */

/**@} end of group I3C_MINTCLR_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_MINTMASKED I3C_MINTMASKED
 * @brief    Controller Interrupt Masked Register
 * @{
 */
#define MXC_F_I3C_MINTMASKED_TGTSTART_POS              8 /**< MINTMASKED_TGTSTART Position */
#define MXC_F_I3C_MINTMASKED_TGTSTART                  ((uint32_t)(0x1UL << MXC_F_I3C_MINTMASKED_TGTSTART_POS)) /**< MINTMASKED_TGTSTART Mask */

#define MXC_F_I3C_MINTMASKED_MCTRLDONE_POS             9 /**< MINTMASKED_MCTRLDONE Position */
#define MXC_F_I3C_MINTMASKED_MCTRLDONE                 ((uint32_t)(0x1UL << MXC_F_I3C_MINTMASKED_MCTRLDONE_POS)) /**< MINTMASKED_MCTRLDONE Mask */

#define MXC_F_I3C_MINTMASKED_COMPLETE_POS              10 /**< MINTMASKED_COMPLETE Position */
#define MXC_F_I3C_MINTMASKED_COMPLETE                  ((uint32_t)(0x1UL << MXC_F_I3C_MINTMASKED_COMPLETE_POS)) /**< MINTMASKED_COMPLETE Mask */

#define MXC_F_I3C_MINTMASKED_RXPEND_POS                11 /**< MINTMASKED_RXPEND Position */
#define MXC_F_I3C_MINTMASKED_RXPEND                    ((uint32_t)(0x1UL << MXC_F_I3C_MINTMASKED_RXPEND_POS)) /**< MINTMASKED_RXPEND Mask */

#define MXC_F_I3C_MINTMASKED_TXNOTFULL_POS             12 /**< MINTMASKED_TXNOTFULL Position */
#define MXC_F_I3C_MINTMASKED_TXNOTFULL                 ((uint32_t)(0x1UL << MXC_F_I3C_MINTMASKED_TXNOTFULL_POS)) /**< MINTMASKED_TXNOTFULL Mask */

#define MXC_F_I3C_MINTMASKED_IBIWON_POS                13 /**< MINTMASKED_IBIWON Position */
#define MXC_F_I3C_MINTMASKED_IBIWON                    ((uint32_t)(0x1UL << MXC_F_I3C_MINTMASKED_IBIWON_POS)) /**< MINTMASKED_IBIWON Mask */

#define MXC_F_I3C_MINTMASKED_ERRWARN_POS               15 /**< MINTMASKED_ERRWARN Position */
#define MXC_F_I3C_MINTMASKED_ERRWARN                   ((uint32_t)(0x1UL << MXC_F_I3C_MINTMASKED_ERRWARN_POS)) /**< MINTMASKED_ERRWARN Mask */

#define MXC_F_I3C_MINTMASKED_NOWCNTLR_POS              19 /**< MINTMASKED_NOWCNTLR Position */
#define MXC_F_I3C_MINTMASKED_NOWCNTLR                  ((uint32_t)(0x1UL << MXC_F_I3C_MINTMASKED_NOWCNTLR_POS)) /**< MINTMASKED_NOWCNTLR Mask */

/**@} end of group I3C_MINTMASKED_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_MERRWARN I3C_MERRWARN
 * @brief    Controller Error and Warning Register
 * @{
 */
#define MXC_F_I3C_MERRWARN_URUN_POS                    1 /**< MERRWARN_URUN Position */
#define MXC_F_I3C_MERRWARN_URUN                        ((uint32_t)(0x1UL << MXC_F_I3C_MERRWARN_URUN_POS)) /**< MERRWARN_URUN Mask */

#define MXC_F_I3C_MERRWARN_NACK_POS                    2 /**< MERRWARN_NACK Position */
#define MXC_F_I3C_MERRWARN_NACK                        ((uint32_t)(0x1UL << MXC_F_I3C_MERRWARN_NACK_POS)) /**< MERRWARN_NACK Mask */

#define MXC_F_I3C_MERRWARN_WRABT_POS                   3 /**< MERRWARN_WRABT Position */
#define MXC_F_I3C_MERRWARN_WRABT                       ((uint32_t)(0x1UL << MXC_F_I3C_MERRWARN_WRABT_POS)) /**< MERRWARN_WRABT Mask */

#define MXC_F_I3C_MERRWARN_TERM_POS                    4 /**< MERRWARN_TERM Position */
#define MXC_F_I3C_MERRWARN_TERM                        ((uint32_t)(0x1UL << MXC_F_I3C_MERRWARN_TERM_POS)) /**< MERRWARN_TERM Mask */

#define MXC_F_I3C_MERRWARN_HPAR_POS                    9 /**< MERRWARN_HPAR Position */
#define MXC_F_I3C_MERRWARN_HPAR                        ((uint32_t)(0x1UL << MXC_F_I3C_MERRWARN_HPAR_POS)) /**< MERRWARN_HPAR Mask */

#define MXC_F_I3C_MERRWARN_HCRC_POS                    10 /**< MERRWARN_HCRC Position */
#define MXC_F_I3C_MERRWARN_HCRC                        ((uint32_t)(0x1UL << MXC_F_I3C_MERRWARN_HCRC_POS)) /**< MERRWARN_HCRC Mask */

#define MXC_F_I3C_MERRWARN_HNOVERIFY_POS               12 /**< MERRWARN_HNOVERIFY Position */
#define MXC_F_I3C_MERRWARN_HNOVERIFY                   ((uint32_t)(0x1UL << MXC_F_I3C_MERRWARN_HNOVERIFY_POS)) /**< MERRWARN_HNOVERIFY Mask */

#define MXC_F_I3C_MERRWARN_OREAD_POS                   16 /**< MERRWARN_OREAD Position */
#define MXC_F_I3C_MERRWARN_OREAD                       ((uint32_t)(0x1UL << MXC_F_I3C_MERRWARN_OREAD_POS)) /**< MERRWARN_OREAD Mask */

#define MXC_F_I3C_MERRWARN_OWRITE_POS                  17 /**< MERRWARN_OWRITE Position */
#define MXC_F_I3C_MERRWARN_OWRITE                      ((uint32_t)(0x1UL << MXC_F_I3C_MERRWARN_OWRITE_POS)) /**< MERRWARN_OWRITE Mask */

#define MXC_F_I3C_MERRWARN_MSGERR_POS                  18 /**< MERRWARN_MSGERR Position */
#define MXC_F_I3C_MERRWARN_MSGERR                      ((uint32_t)(0x1UL << MXC_F_I3C_MERRWARN_MSGERR_POS)) /**< MERRWARN_MSGERR Mask */

#define MXC_F_I3C_MERRWARN_INVREQ_POS                  19 /**< MERRWARN_INVREQ Position */
#define MXC_F_I3C_MERRWARN_INVREQ                      ((uint32_t)(0x1UL << MXC_F_I3C_MERRWARN_INVREQ_POS)) /**< MERRWARN_INVREQ Mask */

#define MXC_F_I3C_MERRWARN_TIMEOUT_POS                 20 /**< MERRWARN_TIMEOUT Position */
#define MXC_F_I3C_MERRWARN_TIMEOUT                     ((uint32_t)(0x1UL << MXC_F_I3C_MERRWARN_TIMEOUT_POS)) /**< MERRWARN_TIMEOUT Mask */

#define MXC_F_I3C_MERRWARN_WRONGSIZE_POS               21 /**< MERRWARN_WRONGSIZE Position */
#define MXC_F_I3C_MERRWARN_WRONGSIZE                   ((uint32_t)(0x1UL << MXC_F_I3C_MERRWARN_WRONGSIZE_POS)) /**< MERRWARN_WRONGSIZE Mask */

/**@} end of group I3C_MERRWARN_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_MDMACTRL I3C_MDMACTRL
 * @brief    Controller DMA Control Register
 * @{
 */
#define MXC_F_I3C_MDMACTRL_DMAFB_POS                   0 /**< MDMACTRL_DMAFB Position */
#define MXC_F_I3C_MDMACTRL_DMAFB                       ((uint32_t)(0x3UL << MXC_F_I3C_MDMACTRL_DMAFB_POS)) /**< MDMACTRL_DMAFB Mask */
#define MXC_V_I3C_MDMACTRL_DMAFB_DISABLE               ((uint32_t)0x0UL) /**< MDMACTRL_DMAFB_DISABLE Value */
#define MXC_S_I3C_MDMACTRL_DMAFB_DISABLE               (MXC_V_I3C_MDMACTRL_DMAFB_DISABLE << MXC_F_I3C_MDMACTRL_DMAFB_POS) /**< MDMACTRL_DMAFB_DISABLE Setting */
#define MXC_V_I3C_MDMACTRL_DMAFB_EN_ONEFRAME_SDR       ((uint32_t)0x1UL) /**< MDMACTRL_DMAFB_EN_ONEFRAME_SDR Value */
#define MXC_S_I3C_MDMACTRL_DMAFB_EN_ONEFRAME_SDR       (MXC_V_I3C_MDMACTRL_DMAFB_EN_ONEFRAME_SDR << MXC_F_I3C_MDMACTRL_DMAFB_POS) /**< MDMACTRL_DMAFB_EN_ONEFRAME_SDR Setting */
#define MXC_V_I3C_MDMACTRL_DMAFB_ENABLE                ((uint32_t)0x2UL) /**< MDMACTRL_DMAFB_ENABLE Value */
#define MXC_S_I3C_MDMACTRL_DMAFB_ENABLE                (MXC_V_I3C_MDMACTRL_DMAFB_ENABLE << MXC_F_I3C_MDMACTRL_DMAFB_POS) /**< MDMACTRL_DMAFB_ENABLE Setting */

#define MXC_F_I3C_MDMACTRL_DMATB_POS                   2 /**< MDMACTRL_DMATB Position */
#define MXC_F_I3C_MDMACTRL_DMATB                       ((uint32_t)(0x3UL << MXC_F_I3C_MDMACTRL_DMATB_POS)) /**< MDMACTRL_DMATB Mask */
#define MXC_V_I3C_MDMACTRL_DMATB_DISABLE               ((uint32_t)0x0UL) /**< MDMACTRL_DMATB_DISABLE Value */
#define MXC_S_I3C_MDMACTRL_DMATB_DISABLE               (MXC_V_I3C_MDMACTRL_DMATB_DISABLE << MXC_F_I3C_MDMACTRL_DMATB_POS) /**< MDMACTRL_DMATB_DISABLE Setting */
#define MXC_V_I3C_MDMACTRL_DMATB_EN_ONEFRAME_SDR       ((uint32_t)0x1UL) /**< MDMACTRL_DMATB_EN_ONEFRAME_SDR Value */
#define MXC_S_I3C_MDMACTRL_DMATB_EN_ONEFRAME_SDR       (MXC_V_I3C_MDMACTRL_DMATB_EN_ONEFRAME_SDR << MXC_F_I3C_MDMACTRL_DMATB_POS) /**< MDMACTRL_DMATB_EN_ONEFRAME_SDR Setting */
#define MXC_V_I3C_MDMACTRL_DMATB_ENABLE                ((uint32_t)0x2UL) /**< MDMACTRL_DMATB_ENABLE Value */
#define MXC_S_I3C_MDMACTRL_DMATB_ENABLE                (MXC_V_I3C_MDMACTRL_DMATB_ENABLE << MXC_F_I3C_MDMACTRL_DMATB_POS) /**< MDMACTRL_DMATB_ENABLE Setting */

#define MXC_F_I3C_MDMACTRL_DMAWIDTH_POS                4 /**< MDMACTRL_DMAWIDTH Position */
#define MXC_F_I3C_MDMACTRL_DMAWIDTH                    ((uint32_t)(0x3UL << MXC_F_I3C_MDMACTRL_DMAWIDTH_POS)) /**< MDMACTRL_DMAWIDTH Mask */
#define MXC_V_I3C_MDMACTRL_DMAWIDTH_BYTE               ((uint32_t)0x0UL) /**< MDMACTRL_DMAWIDTH_BYTE Value */
#define MXC_S_I3C_MDMACTRL_DMAWIDTH_BYTE               (MXC_V_I3C_MDMACTRL_DMAWIDTH_BYTE << MXC_F_I3C_MDMACTRL_DMAWIDTH_POS) /**< MDMACTRL_DMAWIDTH_BYTE Setting */
#define MXC_V_I3C_MDMACTRL_DMAWIDTH_HALFWORD           ((uint32_t)0x2UL) /**< MDMACTRL_DMAWIDTH_HALFWORD Value */
#define MXC_S_I3C_MDMACTRL_DMAWIDTH_HALFWORD           (MXC_V_I3C_MDMACTRL_DMAWIDTH_HALFWORD << MXC_F_I3C_MDMACTRL_DMAWIDTH_POS) /**< MDMACTRL_DMAWIDTH_HALFWORD Setting */

/**@} end of group I3C_MDMACTRL_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_MHDRBTCFG I3C_MHDRBTCFG
 * @brief    Controller HDR-BT Configuration Register
 * @{
 */
#define MXC_F_I3C_MHDRBTCFG_MLHDR_POS                  0 /**< MHDRBTCFG_MLHDR Position */
#define MXC_F_I3C_MHDRBTCFG_MLHDR                      ((uint32_t)(0x3UL << MXC_F_I3C_MHDRBTCFG_MLHDR_POS)) /**< MHDRBTCFG_MLHDR Mask */
#define MXC_V_I3C_MHDRBTCFG_MLHDR_SINGLE_LANE          ((uint32_t)0x0UL) /**< MHDRBTCFG_MLHDR_SINGLE_LANE Value */
#define MXC_S_I3C_MHDRBTCFG_MLHDR_SINGLE_LANE          (MXC_V_I3C_MHDRBTCFG_MLHDR_SINGLE_LANE << MXC_F_I3C_MHDRBTCFG_MLHDR_POS) /**< MHDRBTCFG_MLHDR_SINGLE_LANE Setting */
#define MXC_V_I3C_MHDRBTCFG_MLHDR_DUAL_LANE            ((uint32_t)0x1UL) /**< MHDRBTCFG_MLHDR_DUAL_LANE Value */
#define MXC_S_I3C_MHDRBTCFG_MLHDR_DUAL_LANE            (MXC_V_I3C_MHDRBTCFG_MLHDR_DUAL_LANE << MXC_F_I3C_MHDRBTCFG_MLHDR_POS) /**< MHDRBTCFG_MLHDR_DUAL_LANE Setting */
#define MXC_V_I3C_MHDRBTCFG_MLHDR_QUAD_LANE            ((uint32_t)0x3UL) /**< MHDRBTCFG_MLHDR_QUAD_LANE Value */
#define MXC_S_I3C_MHDRBTCFG_MLHDR_QUAD_LANE            (MXC_V_I3C_MHDRBTCFG_MLHDR_QUAD_LANE << MXC_F_I3C_MHDRBTCFG_MLHDR_POS) /**< MHDRBTCFG_MLHDR_QUAD_LANE Setting */

#define MXC_F_I3C_MHDRBTCFG_MLDAT_POS                  2 /**< MHDRBTCFG_MLDAT Position */
#define MXC_F_I3C_MHDRBTCFG_MLDAT                      ((uint32_t)(0x3UL << MXC_F_I3C_MHDRBTCFG_MLDAT_POS)) /**< MHDRBTCFG_MLDAT Mask */
#define MXC_V_I3C_MHDRBTCFG_MLDAT_SINGLE_LANE          ((uint32_t)0x0UL) /**< MHDRBTCFG_MLDAT_SINGLE_LANE Value */
#define MXC_S_I3C_MHDRBTCFG_MLDAT_SINGLE_LANE          (MXC_V_I3C_MHDRBTCFG_MLDAT_SINGLE_LANE << MXC_F_I3C_MHDRBTCFG_MLDAT_POS) /**< MHDRBTCFG_MLDAT_SINGLE_LANE Setting */
#define MXC_V_I3C_MHDRBTCFG_MLDAT_DUAL_LANE            ((uint32_t)0x1UL) /**< MHDRBTCFG_MLDAT_DUAL_LANE Value */
#define MXC_S_I3C_MHDRBTCFG_MLDAT_DUAL_LANE            (MXC_V_I3C_MHDRBTCFG_MLDAT_DUAL_LANE << MXC_F_I3C_MHDRBTCFG_MLDAT_POS) /**< MHDRBTCFG_MLDAT_DUAL_LANE Setting */
#define MXC_V_I3C_MHDRBTCFG_MLDAT_QUAD_LANE            ((uint32_t)0x3UL) /**< MHDRBTCFG_MLDAT_QUAD_LANE Value */
#define MXC_S_I3C_MHDRBTCFG_MLDAT_QUAD_LANE            (MXC_V_I3C_MHDRBTCFG_MLDAT_QUAD_LANE << MXC_F_I3C_MHDRBTCFG_MLDAT_POS) /**< MHDRBTCFG_MLDAT_QUAD_LANE Setting */

#define MXC_F_I3C_MHDRBTCFG_CRC32_POS                  4 /**< MHDRBTCFG_CRC32 Position */
#define MXC_F_I3C_MHDRBTCFG_CRC32                      ((uint32_t)(0x1UL << MXC_F_I3C_MHDRBTCFG_CRC32_POS)) /**< MHDRBTCFG_CRC32 Mask */

#define MXC_F_I3C_MHDRBTCFG_DATALEN_POS                16 /**< MHDRBTCFG_DATALEN Position */
#define MXC_F_I3C_MHDRBTCFG_DATALEN                    ((uint32_t)(0xFFFFUL << MXC_F_I3C_MHDRBTCFG_DATALEN_POS)) /**< MHDRBTCFG_DATALEN Mask */

/**@} end of group I3C_MHDRBTCFG_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_MHDRBTLAST I3C_MHDRBTLAST
 * @brief    Controller HDR-BT Last Data Length Register
 * @{
 */
#define MXC_F_I3C_MHDRBTLAST_DATALEN_POS               16 /**< MHDRBTLAST_DATALEN Position */
#define MXC_F_I3C_MHDRBTLAST_DATALEN                   ((uint32_t)(0xFFFFUL << MXC_F_I3C_MHDRBTLAST_DATALEN_POS)) /**< MHDRBTLAST_DATALEN Mask */

/**@} end of group I3C_MHDRBTLAST_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_MDATACTRL I3C_MDATACTRL
 * @brief    Controller Data Control Register
 * @{
 */
#define MXC_F_I3C_MDATACTRL_FLUSHTB_POS                0 /**< MDATACTRL_FLUSHTB Position */
#define MXC_F_I3C_MDATACTRL_FLUSHTB                    ((uint32_t)(0x1UL << MXC_F_I3C_MDATACTRL_FLUSHTB_POS)) /**< MDATACTRL_FLUSHTB Mask */

#define MXC_F_I3C_MDATACTRL_FLUSHFB_POS                1 /**< MDATACTRL_FLUSHFB Position */
#define MXC_F_I3C_MDATACTRL_FLUSHFB                    ((uint32_t)(0x1UL << MXC_F_I3C_MDATACTRL_FLUSHFB_POS)) /**< MDATACTRL_FLUSHFB Mask */

#define MXC_F_I3C_MDATACTRL_UNLOCK_POS                 3 /**< MDATACTRL_UNLOCK Position */
#define MXC_F_I3C_MDATACTRL_UNLOCK                     ((uint32_t)(0x1UL << MXC_F_I3C_MDATACTRL_UNLOCK_POS)) /**< MDATACTRL_UNLOCK Mask */

#define MXC_F_I3C_MDATACTRL_TXTRIG_POS                 4 /**< MDATACTRL_TXTRIG Position */
#define MXC_F_I3C_MDATACTRL_TXTRIG                     ((uint32_t)(0x3UL << MXC_F_I3C_MDATACTRL_TXTRIG_POS)) /**< MDATACTRL_TXTRIG Mask */
#define MXC_V_I3C_MDATACTRL_TXTRIG_EMPTY               ((uint32_t)0x0UL) /**< MDATACTRL_TXTRIG_EMPTY Value */
#define MXC_S_I3C_MDATACTRL_TXTRIG_EMPTY               (MXC_V_I3C_MDATACTRL_TXTRIG_EMPTY << MXC_F_I3C_MDATACTRL_TXTRIG_POS) /**< MDATACTRL_TXTRIG_EMPTY Setting */
#define MXC_V_I3C_MDATACTRL_TXTRIG_QUARTER_FULL        ((uint32_t)0x1UL) /**< MDATACTRL_TXTRIG_QUARTER_FULL Value */
#define MXC_S_I3C_MDATACTRL_TXTRIG_QUARTER_FULL        (MXC_V_I3C_MDATACTRL_TXTRIG_QUARTER_FULL << MXC_F_I3C_MDATACTRL_TXTRIG_POS) /**< MDATACTRL_TXTRIG_QUARTER_FULL Setting */
#define MXC_V_I3C_MDATACTRL_TXTRIG_HALF_FULL           ((uint32_t)0x2UL) /**< MDATACTRL_TXTRIG_HALF_FULL Value */
#define MXC_S_I3C_MDATACTRL_TXTRIG_HALF_FULL           (MXC_V_I3C_MDATACTRL_TXTRIG_HALF_FULL << MXC_F_I3C_MDATACTRL_TXTRIG_POS) /**< MDATACTRL_TXTRIG_HALF_FULL Setting */
#define MXC_V_I3C_MDATACTRL_TXTRIG_ALMOST_FULL         ((uint32_t)0x3UL) /**< MDATACTRL_TXTRIG_ALMOST_FULL Value */
#define MXC_S_I3C_MDATACTRL_TXTRIG_ALMOST_FULL         (MXC_V_I3C_MDATACTRL_TXTRIG_ALMOST_FULL << MXC_F_I3C_MDATACTRL_TXTRIG_POS) /**< MDATACTRL_TXTRIG_ALMOST_FULL Setting */

#define MXC_F_I3C_MDATACTRL_RXTRIG_POS                 6 /**< MDATACTRL_RXTRIG Position */
#define MXC_F_I3C_MDATACTRL_RXTRIG                     ((uint32_t)(0x3UL << MXC_F_I3C_MDATACTRL_RXTRIG_POS)) /**< MDATACTRL_RXTRIG Mask */
#define MXC_V_I3C_MDATACTRL_RXTRIG_NOT_EMPTY           ((uint32_t)0x0UL) /**< MDATACTRL_RXTRIG_NOT_EMPTY Value */
#define MXC_S_I3C_MDATACTRL_RXTRIG_NOT_EMPTY           (MXC_V_I3C_MDATACTRL_RXTRIG_NOT_EMPTY << MXC_F_I3C_MDATACTRL_RXTRIG_POS) /**< MDATACTRL_RXTRIG_NOT_EMPTY Setting */
#define MXC_V_I3C_MDATACTRL_RXTRIG_QUARTER_FULL        ((uint32_t)0x1UL) /**< MDATACTRL_RXTRIG_QUARTER_FULL Value */
#define MXC_S_I3C_MDATACTRL_RXTRIG_QUARTER_FULL        (MXC_V_I3C_MDATACTRL_RXTRIG_QUARTER_FULL << MXC_F_I3C_MDATACTRL_RXTRIG_POS) /**< MDATACTRL_RXTRIG_QUARTER_FULL Setting */
#define MXC_V_I3C_MDATACTRL_RXTRIG_HALF_FULL           ((uint32_t)0x2UL) /**< MDATACTRL_RXTRIG_HALF_FULL Value */
#define MXC_S_I3C_MDATACTRL_RXTRIG_HALF_FULL           (MXC_V_I3C_MDATACTRL_RXTRIG_HALF_FULL << MXC_F_I3C_MDATACTRL_RXTRIG_POS) /**< MDATACTRL_RXTRIG_HALF_FULL Setting */
#define MXC_V_I3C_MDATACTRL_RXTRIG_3_4_FULL            ((uint32_t)0x3UL) /**< MDATACTRL_RXTRIG_3_4_FULL Value */
#define MXC_S_I3C_MDATACTRL_RXTRIG_3_4_FULL            (MXC_V_I3C_MDATACTRL_RXTRIG_3_4_FULL << MXC_F_I3C_MDATACTRL_RXTRIG_POS) /**< MDATACTRL_RXTRIG_3_4_FULL Setting */

#define MXC_F_I3C_MDATACTRL_TXCOUNT_POS                16 /**< MDATACTRL_TXCOUNT Position */
#define MXC_F_I3C_MDATACTRL_TXCOUNT                    ((uint32_t)(0x3FUL << MXC_F_I3C_MDATACTRL_TXCOUNT_POS)) /**< MDATACTRL_TXCOUNT Mask */

#define MXC_F_I3C_MDATACTRL_RXCOUNT_POS                24 /**< MDATACTRL_RXCOUNT Position */
#define MXC_F_I3C_MDATACTRL_RXCOUNT                    ((uint32_t)(0x3FUL << MXC_F_I3C_MDATACTRL_RXCOUNT_POS)) /**< MDATACTRL_RXCOUNT Mask */

#define MXC_F_I3C_MDATACTRL_TXFULL_POS                 30 /**< MDATACTRL_TXFULL Position */
#define MXC_F_I3C_MDATACTRL_TXFULL                     ((uint32_t)(0x1UL << MXC_F_I3C_MDATACTRL_TXFULL_POS)) /**< MDATACTRL_TXFULL Mask */

#define MXC_F_I3C_MDATACTRL_RXEMPTY_POS                31 /**< MDATACTRL_RXEMPTY Position */
#define MXC_F_I3C_MDATACTRL_RXEMPTY                    ((uint32_t)(0x1UL << MXC_F_I3C_MDATACTRL_RXEMPTY_POS)) /**< MDATACTRL_RXEMPTY Mask */

/**@} end of group I3C_MDATACTRL_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_MWDATAB I3C_MWDATAB
 * @brief    Controller Write Byte Data Register
 * @{
 */
#define MXC_F_I3C_MWDATAB_DATA_POS                     0 /**< MWDATAB_DATA Position */
#define MXC_F_I3C_MWDATAB_DATA                         ((uint32_t)(0xFFUL << MXC_F_I3C_MWDATAB_DATA_POS)) /**< MWDATAB_DATA Mask */

#define MXC_F_I3C_MWDATAB_END_POS                      8 /**< MWDATAB_END Position */
#define MXC_F_I3C_MWDATAB_END                          ((uint32_t)(0x1UL << MXC_F_I3C_MWDATAB_END_POS)) /**< MWDATAB_END Mask */

#define MXC_F_I3C_MWDATAB_END2_POS                     16 /**< MWDATAB_END2 Position */
#define MXC_F_I3C_MWDATAB_END2                         ((uint32_t)(0x1UL << MXC_F_I3C_MWDATAB_END2_POS)) /**< MWDATAB_END2 Mask */

/**@} end of group I3C_MWDATAB_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_MWDATABE I3C_MWDATABE
 * @brief    Controller Write Byte Data as End Register
 * @{
 */
#define MXC_F_I3C_MWDATABE_DATA_POS                    0 /**< MWDATABE_DATA Position */
#define MXC_F_I3C_MWDATABE_DATA                        ((uint32_t)(0xFFUL << MXC_F_I3C_MWDATABE_DATA_POS)) /**< MWDATABE_DATA Mask */

/**@} end of group I3C_MWDATABE_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_MWDATAH I3C_MWDATAH
 * @brief    Controller Write Half-Word Data Register
 * @{
 */
#define MXC_F_I3C_MWDATAH_DATA0_POS                    0 /**< MWDATAH_DATA0 Position */
#define MXC_F_I3C_MWDATAH_DATA0                        ((uint32_t)(0xFFUL << MXC_F_I3C_MWDATAH_DATA0_POS)) /**< MWDATAH_DATA0 Mask */

#define MXC_F_I3C_MWDATAH_DATA1_POS                    8 /**< MWDATAH_DATA1 Position */
#define MXC_F_I3C_MWDATAH_DATA1                        ((uint32_t)(0xFFUL << MXC_F_I3C_MWDATAH_DATA1_POS)) /**< MWDATAH_DATA1 Mask */

#define MXC_F_I3C_MWDATAH_END_POS                      16 /**< MWDATAH_END Position */
#define MXC_F_I3C_MWDATAH_END                          ((uint32_t)(0x1UL << MXC_F_I3C_MWDATAH_END_POS)) /**< MWDATAH_END Mask */

/**@} end of group I3C_MWDATAH_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_MWDATAHE I3C_MWDATAHE
 * @brief    Controller Write Half-Word Data as End Register
 * @{
 */
#define MXC_F_I3C_MWDATAHE_DATA0_POS                   0 /**< MWDATAHE_DATA0 Position */
#define MXC_F_I3C_MWDATAHE_DATA0                       ((uint32_t)(0xFFUL << MXC_F_I3C_MWDATAHE_DATA0_POS)) /**< MWDATAHE_DATA0 Mask */

#define MXC_F_I3C_MWDATAHE_DATA1_POS                   8 /**< MWDATAHE_DATA1 Position */
#define MXC_F_I3C_MWDATAHE_DATA1                       ((uint32_t)(0xFFUL << MXC_F_I3C_MWDATAHE_DATA1_POS)) /**< MWDATAHE_DATA1 Mask */

/**@} end of group I3C_MWDATAHE_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_MRDATAB I3C_MRDATAB
 * @brief    Controller Read Byte Data Register
 * @{
 */
#define MXC_F_I3C_MRDATAB_DATA_POS                     0 /**< MRDATAB_DATA Position */
#define MXC_F_I3C_MRDATAB_DATA                         ((uint32_t)(0xFFUL << MXC_F_I3C_MRDATAB_DATA_POS)) /**< MRDATAB_DATA Mask */

/**@} end of group I3C_MRDATAB_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_MRDATAH I3C_MRDATAH
 * @brief    Controller Read Half-Word Data Register
 * @{
 */
#define MXC_F_I3C_MRDATAH_DATA0_POS                    0 /**< MRDATAH_DATA0 Position */
#define MXC_F_I3C_MRDATAH_DATA0                        ((uint32_t)(0xFFUL << MXC_F_I3C_MRDATAH_DATA0_POS)) /**< MRDATAH_DATA0 Mask */

#define MXC_F_I3C_MRDATAH_DATA1_POS                    8 /**< MRDATAH_DATA1 Position */
#define MXC_F_I3C_MRDATAH_DATA1                        ((uint32_t)(0xFFUL << MXC_F_I3C_MRDATAH_DATA1_POS)) /**< MRDATAH_DATA1 Mask */

/**@} end of group I3C_MRDATAH_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_MWDATAB1 I3C_MWDATAB1
 * @brief    Controller Byte-Only Write Byte Data Register
 * @{
 */
#define MXC_F_I3C_MWDATAB1_DATA_POS                    0 /**< MWDATAB1_DATA Position */
#define MXC_F_I3C_MWDATAB1_DATA                        ((uint32_t)(0xFFUL << MXC_F_I3C_MWDATAB1_DATA_POS)) /**< MWDATAB1_DATA Mask */

/**@} end of group I3C_MWDATAB1_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_MWMSG_SDR I3C_MWMSG_SDR
 * @brief    Start or Continue SDR Message Register
 * @{
 */
#define MXC_F_I3C_MWMSG_SDR_DIR_POS                    0 /**< MWMSG_SDR_DIR Position */
#define MXC_F_I3C_MWMSG_SDR_DIR                        ((uint32_t)(0x1UL << MXC_F_I3C_MWMSG_SDR_DIR_POS)) /**< MWMSG_SDR_DIR Mask */

#define MXC_F_I3C_MWMSG_SDR_ADDR_POS                   1 /**< MWMSG_SDR_ADDR Position */
#define MXC_F_I3C_MWMSG_SDR_ADDR                       ((uint32_t)(0x7FUL << MXC_F_I3C_MWMSG_SDR_ADDR_POS)) /**< MWMSG_SDR_ADDR Mask */

#define MXC_F_I3C_MWMSG_SDR_END_POS                    8 /**< MWMSG_SDR_END Position */
#define MXC_F_I3C_MWMSG_SDR_END                        ((uint32_t)(0x1UL << MXC_F_I3C_MWMSG_SDR_END_POS)) /**< MWMSG_SDR_END Mask */

#define MXC_F_I3C_MWMSG_SDR_I2C_POS                    10 /**< MWMSG_SDR_I2C Position */
#define MXC_F_I3C_MWMSG_SDR_I2C                        ((uint32_t)(0x1UL << MXC_F_I3C_MWMSG_SDR_I2C_POS)) /**< MWMSG_SDR_I2C Mask */

#define MXC_F_I3C_MWMSG_SDR_LEN_POS                    11 /**< MWMSG_SDR_LEN Position */
#define MXC_F_I3C_MWMSG_SDR_LEN                        ((uint32_t)(0x1FUL << MXC_F_I3C_MWMSG_SDR_LEN_POS)) /**< MWMSG_SDR_LEN Mask */

/**@} end of group I3C_MWMSG_SDR_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_MRMSG_SDR I3C_MRMSG_SDR
 * @brief    Read SDR Message Data Register
 * @{
 */
#define MXC_F_I3C_MRMSG_SDR_DATA_POS                   0 /**< MRMSG_SDR_DATA Position */
#define MXC_F_I3C_MRMSG_SDR_DATA                       ((uint32_t)(0xFFFFUL << MXC_F_I3C_MRMSG_SDR_DATA_POS)) /**< MRMSG_SDR_DATA Mask */

/**@} end of group I3C_MRMSG_SDR_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_MWMSG_DDR I3C_MWMSG_DDR
 * @brief    Start or Continue DDR Message Register
 * @{
 */
#define MXC_F_I3C_MWMSG_DDR_CONTROL_ADDRCMD_DATA_POS   0 /**< MWMSG_DDR_CONTROL_ADDRCMD_DATA Position */
#define MXC_F_I3C_MWMSG_DDR_CONTROL_ADDRCMD_DATA       ((uint32_t)(0xFFFFUL << MXC_F_I3C_MWMSG_DDR_CONTROL_ADDRCMD_DATA_POS)) /**< MWMSG_DDR_CONTROL_ADDRCMD_DATA Mask */

/**@} end of group I3C_MWMSG_DDR_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_MRMSG_DDR I3C_MRMSG_DDR
 * @brief    Read DDR Message Data Register
 * @{
 */
#define MXC_F_I3C_MRMSG_DDR_DATA_POS                   0 /**< MRMSG_DDR_DATA Position */
#define MXC_F_I3C_MRMSG_DDR_DATA                       ((uint32_t)(0xFFFFUL << MXC_F_I3C_MRMSG_DDR_DATA_POS)) /**< MRMSG_DDR_DATA Mask */

/**@} end of group I3C_MRMSG_DDR_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_MDYNADDR I3C_MDYNADDR
 * @brief    Controller Dynamic Address Register
 * @{
 */
#define MXC_F_I3C_MDYNADDR_DAVALID_POS                 0 /**< MDYNADDR_DAVALID Position */
#define MXC_F_I3C_MDYNADDR_DAVALID                     ((uint32_t)(0x1UL << MXC_F_I3C_MDYNADDR_DAVALID_POS)) /**< MDYNADDR_DAVALID Mask */

#define MXC_F_I3C_MDYNADDR_DADDR_POS                   1 /**< MDYNADDR_DADDR Position */
#define MXC_F_I3C_MDYNADDR_DADDR                       ((uint32_t)(0x7FUL << MXC_F_I3C_MDYNADDR_DADDR_POS)) /**< MDYNADDR_DADDR Mask */

/**@} end of group I3C_MDYNADDR_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_MWDATAW I3C_MWDATAW
 * @brief    Controller Write Word Data Register
 * @{
 */
#define MXC_F_I3C_MWDATAW_DATA_POS                     0 /**< MWDATAW_DATA Position */
#define MXC_F_I3C_MWDATAW_DATA                         ((uint32_t)(0xFFFFFFFFUL << MXC_F_I3C_MWDATAW_DATA_POS)) /**< MWDATAW_DATA Mask */

/**@} end of group I3C_MWDATAW_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_MRDATAW I3C_MRDATAW
 * @brief    Controller Read Word Data Register
 * @{
 */
#define MXC_F_I3C_MRDATAW_DATA_POS                     0 /**< MRDATAW_DATA Position */
#define MXC_F_I3C_MRDATAW_DATA                         ((uint32_t)(0xFFFFFFFFUL << MXC_F_I3C_MRDATAW_DATA_POS)) /**< MRDATAW_DATA Mask */

/**@} end of group I3C_MRDATAW_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_RSTACTTIME I3C_RSTACTTIME
 * @brief    Target Reset Recovery Time Register
 * @{
 */
#define MXC_F_I3C_RSTACTTIME_PERRSTTIM_POS             0 /**< RSTACTTIME_PERRSTTIM Position */
#define MXC_F_I3C_RSTACTTIME_PERRSTTIM                 ((uint32_t)(0xFFUL << MXC_F_I3C_RSTACTTIME_PERRSTTIM_POS)) /**< RSTACTTIME_PERRSTTIM Mask */

#define MXC_F_I3C_RSTACTTIME_SYSRSTTIM_POS             8 /**< RSTACTTIME_SYSRSTTIM Position */
#define MXC_F_I3C_RSTACTTIME_SYSRSTTIM                 ((uint32_t)(0xFFUL << MXC_F_I3C_RSTACTTIME_SYSRSTTIM_POS)) /**< RSTACTTIME_SYSRSTTIM Mask */

/**@} end of group I3C_RSTACTTIME_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_HDRCMD I3C_HDRCMD
 * @brief    HDR Command Byte Register
 * @{
 */
#define MXC_F_I3C_HDRCMD_CMD0_POS                      0 /**< HDRCMD_CMD0 Position */
#define MXC_F_I3C_HDRCMD_CMD0                          ((uint32_t)(0xFFUL << MXC_F_I3C_HDRCMD_CMD0_POS)) /**< HDRCMD_CMD0 Mask */

#define MXC_F_I3C_HDRCMD_OVFLW_POS                     30 /**< HDRCMD_OVFLW Position */
#define MXC_F_I3C_HDRCMD_OVFLW                         ((uint32_t)(0x1UL << MXC_F_I3C_HDRCMD_OVFLW_POS)) /**< HDRCMD_OVFLW Mask */

#define MXC_F_I3C_HDRCMD_NEWCMD_POS                    31 /**< HDRCMD_NEWCMD Position */
#define MXC_F_I3C_HDRCMD_NEWCMD                        ((uint32_t)(0x1UL << MXC_F_I3C_HDRCMD_NEWCMD_POS)) /**< HDRCMD_NEWCMD Mask */

/**@} end of group I3C_HDRCMD_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_GROUPDEF I3C_GROUPDEF
 * @brief    Group Definition Register
 * @{
 */
#define MXC_F_I3C_GROUPDEF_GRP0ENA_POS                 0 /**< GROUPDEF_GRP0ENA Position */
#define MXC_F_I3C_GROUPDEF_GRP0ENA                     ((uint32_t)(0x1UL << MXC_F_I3C_GROUPDEF_GRP0ENA_POS)) /**< GROUPDEF_GRP0ENA Mask */

#define MXC_F_I3C_GROUPDEF_GRP0DA_POS                  1 /**< GROUPDEF_GRP0DA Position */
#define MXC_F_I3C_GROUPDEF_GRP0DA                      ((uint32_t)(0x7FUL << MXC_F_I3C_GROUPDEF_GRP0DA_POS)) /**< GROUPDEF_GRP0DA Mask */

#define MXC_F_I3C_GROUPDEF_GRP1ENA_POS                 8 /**< GROUPDEF_GRP1ENA Position */
#define MXC_F_I3C_GROUPDEF_GRP1ENA                     ((uint32_t)(0x1UL << MXC_F_I3C_GROUPDEF_GRP1ENA_POS)) /**< GROUPDEF_GRP1ENA Mask */

#define MXC_F_I3C_GROUPDEF_GRP1DA_POS                  9 /**< GROUPDEF_GRP1DA Position */
#define MXC_F_I3C_GROUPDEF_GRP1DA                      ((uint32_t)(0x7FUL << MXC_F_I3C_GROUPDEF_GRP1DA_POS)) /**< GROUPDEF_GRP1DA Mask */

#define MXC_F_I3C_GROUPDEF_GRP2ENA_POS                 16 /**< GROUPDEF_GRP2ENA Position */
#define MXC_F_I3C_GROUPDEF_GRP2ENA                     ((uint32_t)(0x1UL << MXC_F_I3C_GROUPDEF_GRP2ENA_POS)) /**< GROUPDEF_GRP2ENA Mask */

#define MXC_F_I3C_GROUPDEF_GRP2DA_POS                  17 /**< GROUPDEF_GRP2DA Position */
#define MXC_F_I3C_GROUPDEF_GRP2DA                      ((uint32_t)(0x7FUL << MXC_F_I3C_GROUPDEF_GRP2DA_POS)) /**< GROUPDEF_GRP2DA Mask */

/**@} end of group I3C_GROUPDEF_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_MAPCTRL0 I3C_MAPCTRL0
 * @brief    Map Control Register 0
 * @{
 */
#define MXC_F_I3C_MAPCTRL0_ENA_POS                     0 /**< MAPCTRL0_ENA Position */
#define MXC_F_I3C_MAPCTRL0_ENA                         ((uint32_t)(0x1UL << MXC_F_I3C_MAPCTRL0_ENA_POS)) /**< MAPCTRL0_ENA Mask */

#define MXC_F_I3C_MAPCTRL0_DA_POS                      1 /**< MAPCTRL0_DA Position */
#define MXC_F_I3C_MAPCTRL0_DA                          ((uint32_t)(0x7FUL << MXC_F_I3C_MAPCTRL0_DA_POS)) /**< MAPCTRL0_DA Mask */

#define MXC_F_I3C_MAPCTRL0_CAUSE_POS                   8 /**< MAPCTRL0_CAUSE Position */
#define MXC_F_I3C_MAPCTRL0_CAUSE                       ((uint32_t)(0x7UL << MXC_F_I3C_MAPCTRL0_CAUSE_POS)) /**< MAPCTRL0_CAUSE Mask */
#define MXC_V_I3C_MAPCTRL0_CAUSE_NO_CHANGE             ((uint32_t)0x0UL) /**< MAPCTRL0_CAUSE_NO_CHANGE Value */
#define MXC_S_I3C_MAPCTRL0_CAUSE_NO_CHANGE             (MXC_V_I3C_MAPCTRL0_CAUSE_NO_CHANGE << MXC_F_I3C_MAPCTRL0_CAUSE_POS) /**< MAPCTRL0_CAUSE_NO_CHANGE Setting */
#define MXC_V_I3C_MAPCTRL0_CAUSE_ENTDAA                ((uint32_t)0x1UL) /**< MAPCTRL0_CAUSE_ENTDAA Value */
#define MXC_S_I3C_MAPCTRL0_CAUSE_ENTDAA                (MXC_V_I3C_MAPCTRL0_CAUSE_ENTDAA << MXC_F_I3C_MAPCTRL0_CAUSE_POS) /**< MAPCTRL0_CAUSE_ENTDAA Setting */
#define MXC_V_I3C_MAPCTRL0_CAUSE_SETDASA_AASA_NEWDA    ((uint32_t)0x2UL) /**< MAPCTRL0_CAUSE_SETDASA_AASA_NEWDA Value */
#define MXC_S_I3C_MAPCTRL0_CAUSE_SETDASA_AASA_NEWDA    (MXC_V_I3C_MAPCTRL0_CAUSE_SETDASA_AASA_NEWDA << MXC_F_I3C_MAPCTRL0_CAUSE_POS) /**< MAPCTRL0_CAUSE_SETDASA_AASA_NEWDA Setting */
#define MXC_V_I3C_MAPCTRL0_CAUSE_RSTDAA                ((uint32_t)0x3UL) /**< MAPCTRL0_CAUSE_RSTDAA Value */
#define MXC_S_I3C_MAPCTRL0_CAUSE_RSTDAA                (MXC_V_I3C_MAPCTRL0_CAUSE_RSTDAA << MXC_F_I3C_MAPCTRL0_CAUSE_POS) /**< MAPCTRL0_CAUSE_RSTDAA Setting */
#define MXC_V_I3C_MAPCTRL0_CAUSE_MAPPED_ADDR_OP        ((uint32_t)0x4UL) /**< MAPCTRL0_CAUSE_MAPPED_ADDR_OP Value */
#define MXC_S_I3C_MAPCTRL0_CAUSE_MAPPED_ADDR_OP        (MXC_V_I3C_MAPCTRL0_CAUSE_MAPPED_ADDR_OP << MXC_F_I3C_MAPCTRL0_CAUSE_POS) /**< MAPCTRL0_CAUSE_MAPPED_ADDR_OP Setting */

/**@} end of group I3C_MAPCTRL0_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_MAPCTRLN I3C_MAPCTRLN
 * @brief    Map Control Register n
 * @{
 */
#define MXC_F_I3C_MAPCTRLN_ENA_POS                     0 /**< MAPCTRLN_ENA Position */
#define MXC_F_I3C_MAPCTRLN_ENA                         ((uint32_t)(0x1UL << MXC_F_I3C_MAPCTRLN_ENA_POS)) /**< MAPCTRLN_ENA Mask */

#define MXC_F_I3C_MAPCTRLN_ADDR_POS                    1 /**< MAPCTRLN_ADDR Position */
#define MXC_F_I3C_MAPCTRLN_ADDR                        ((uint32_t)(0x7FUL << MXC_F_I3C_MAPCTRLN_ADDR_POS)) /**< MAPCTRLN_ADDR Mask */

#define MXC_F_I3C_MAPCTRLN_MAPSA_POS                   8 /**< MAPCTRLN_MAPSA Position */
#define MXC_F_I3C_MAPCTRLN_MAPSA                       ((uint32_t)(0x1UL << MXC_F_I3C_MAPCTRLN_MAPSA_POS)) /**< MAPCTRLN_MAPSA Mask */

#define MXC_F_I3C_MAPCTRLN_SA10B_POS                   9 /**< MAPCTRLN_SA10B Position */
#define MXC_F_I3C_MAPCTRLN_SA10B                       ((uint32_t)(0x7UL << MXC_F_I3C_MAPCTRLN_SA10B_POS)) /**< MAPCTRLN_SA10B Mask */

#define MXC_F_I3C_MAPCTRLN_NACK_POS                    12 /**< MAPCTRLN_NACK Position */
#define MXC_F_I3C_MAPCTRLN_NACK                        ((uint32_t)(0x1UL << MXC_F_I3C_MAPCTRLN_NACK_POS)) /**< MAPCTRLN_NACK Mask */

#define MXC_F_I3C_MAPCTRLN_AUTO_POS                    13 /**< MAPCTRLN_AUTO Position */
#define MXC_F_I3C_MAPCTRLN_AUTO                        ((uint32_t)(0x1UL << MXC_F_I3C_MAPCTRLN_AUTO_POS)) /**< MAPCTRLN_AUTO Mask */

#define MXC_F_I3C_MAPCTRLN_PID_POS                     14 /**< MAPCTRLN_PID Position */
#define MXC_F_I3C_MAPCTRLN_PID                         ((uint32_t)(0x3FFUL << MXC_F_I3C_MAPCTRLN_PID_POS)) /**< MAPCTRLN_PID Mask */

#define MXC_F_I3C_MAPCTRLN_DCR_OR_PID_POS              24 /**< MAPCTRLN_DCR_OR_PID Position */
#define MXC_F_I3C_MAPCTRLN_DCR_OR_PID                  ((uint32_t)(0xFFUL << MXC_F_I3C_MAPCTRLN_DCR_OR_PID_POS)) /**< MAPCTRLN_DCR_OR_PID Mask */

/**@} end of group I3C_MAPCTRLN_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_IBIEXT1 I3C_IBIEXT1
 * @brief    Extended IBI Data Register 1
 * @{
 */
#define MXC_F_I3C_IBIEXT1_CNT_POS                      0 /**< IBIEXT1_CNT Position */
#define MXC_F_I3C_IBIEXT1_CNT                          ((uint32_t)(0x7UL << MXC_F_I3C_IBIEXT1_CNT_POS)) /**< IBIEXT1_CNT Mask */

#define MXC_F_I3C_IBIEXT1_MAX_POS                      4 /**< IBIEXT1_MAX Position */
#define MXC_F_I3C_IBIEXT1_MAX                          ((uint32_t)(0x7UL << MXC_F_I3C_IBIEXT1_MAX_POS)) /**< IBIEXT1_MAX Mask */

#define MXC_F_I3C_IBIEXT1_EXT1_POS                     8 /**< IBIEXT1_EXT1 Position */
#define MXC_F_I3C_IBIEXT1_EXT1                         ((uint32_t)(0xFFUL << MXC_F_I3C_IBIEXT1_EXT1_POS)) /**< IBIEXT1_EXT1 Mask */

#define MXC_F_I3C_IBIEXT1_EXT2_POS                     16 /**< IBIEXT1_EXT2 Position */
#define MXC_F_I3C_IBIEXT1_EXT2                         ((uint32_t)(0xFFUL << MXC_F_I3C_IBIEXT1_EXT2_POS)) /**< IBIEXT1_EXT2 Mask */

#define MXC_F_I3C_IBIEXT1_EXT3_POS                     24 /**< IBIEXT1_EXT3 Position */
#define MXC_F_I3C_IBIEXT1_EXT3                         ((uint32_t)(0xFFUL << MXC_F_I3C_IBIEXT1_EXT3_POS)) /**< IBIEXT1_EXT3 Mask */

/**@} end of group I3C_IBIEXT1_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_IBIEXT2 I3C_IBIEXT2
 * @brief    Extended IBI Data Register 2
 * @{
 */
#define MXC_F_I3C_IBIEXT2_EXT4_POS                     0 /**< IBIEXT2_EXT4 Position */
#define MXC_F_I3C_IBIEXT2_EXT4                         ((uint32_t)(0xFFUL << MXC_F_I3C_IBIEXT2_EXT4_POS)) /**< IBIEXT2_EXT4 Mask */

#define MXC_F_I3C_IBIEXT2_EXT5_POS                     8 /**< IBIEXT2_EXT5 Position */
#define MXC_F_I3C_IBIEXT2_EXT5                         ((uint32_t)(0xFFUL << MXC_F_I3C_IBIEXT2_EXT5_POS)) /**< IBIEXT2_EXT5 Mask */

#define MXC_F_I3C_IBIEXT2_EXT6_POS                     16 /**< IBIEXT2_EXT6 Position */
#define MXC_F_I3C_IBIEXT2_EXT6                         ((uint32_t)(0xFFUL << MXC_F_I3C_IBIEXT2_EXT6_POS)) /**< IBIEXT2_EXT6 Mask */

#define MXC_F_I3C_IBIEXT2_EXT7_POS                     24 /**< IBIEXT2_EXT7 Position */
#define MXC_F_I3C_IBIEXT2_EXT7                         ((uint32_t)(0xFFUL << MXC_F_I3C_IBIEXT2_EXT7_POS)) /**< IBIEXT2_EXT7 Mask */

/**@} end of group I3C_IBIEXT2_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_WDATAW I3C_WDATAW
 * @brief    Write Word Data Register
 * @{
 */
#define MXC_F_I3C_WDATAW_DATA_POS                      0 /**< WDATAW_DATA Position */
#define MXC_F_I3C_WDATAW_DATA                          ((uint32_t)(0xFFFFFFFFUL << MXC_F_I3C_WDATAW_DATA_POS)) /**< WDATAW_DATA Mask */

/**@} end of group I3C_WDATAW_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_RDATAW I3C_RDATAW
 * @brief    Read Word Data Register
 * @{
 */
#define MXC_F_I3C_RDATAW_DATA_POS                      0 /**< RDATAW_DATA Position */
#define MXC_F_I3C_RDATAW_DATA                          ((uint32_t)(0xFFFFFFFFUL << MXC_F_I3C_RDATAW_DATA_POS)) /**< RDATAW_DATA Mask */

/**@} end of group I3C_RDATAW_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_ID I3C_ID
 * @brief    Block ID Register
 * @{
 */
#define MXC_F_I3C_ID_ID_POS                            0 /**< ID_ID Position */
#define MXC_F_I3C_ID_ID                                ((uint32_t)(0xFFFFFFFFUL << MXC_F_I3C_ID_ID_POS)) /**< ID_ID Mask */

/**@} end of group I3C_ID_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_I3C_REGS_H_
