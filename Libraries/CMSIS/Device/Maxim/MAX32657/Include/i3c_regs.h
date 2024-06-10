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
 * @details     Improved Inter-Integrated Circuit.
 */

/**
 * @ingroup i3c_registers
 * Structure type to access the I3C Registers.
 */
typedef struct {
    __IO uint32_t cont_ctrl0;           /**< <tt>\b 0x000:</tt> I3C CONT_CTRL0 Register */
    __IO uint32_t targ_ctrl0;           /**< <tt>\b 0x004:</tt> I3C TARG_CTRL0 Register */
    __IO uint32_t targ_status;          /**< <tt>\b 0x008:</tt> I3C TARG_STATUS Register */
    __IO uint32_t targ_ctrl1;           /**< <tt>\b 0x00C:</tt> I3C TARG_CTRL1 Register */
    __IO uint32_t targ_inten;           /**< <tt>\b 0x010:</tt> I3C TARG_INTEN Register */
    __O  uint32_t targ_intclr;          /**< <tt>\b 0x014:</tt> I3C TARG_INTCLR Register */
    __I  uint32_t targ_intfl;           /**< <tt>\b 0x018:</tt> I3C TARG_INTFL Register */
    __IO uint32_t targ_errwarn;         /**< <tt>\b 0x01C:</tt> I3C TARG_ERRWARN Register */
    __IO uint32_t targ_dmactrl;         /**< <tt>\b 0x020:</tt> I3C TARG_DMACTRL Register */
    __R  uint32_t rsv_0x24_0x2b[2];
    __IO uint32_t targ_fifoctrl;        /**< <tt>\b 0x02C:</tt> I3C TARG_FIFOCTRL Register */
    __O  uint32_t targ_txfifo8;         /**< <tt>\b 0x030:</tt> I3C TARG_TXFIFO8 Register */
    __O  uint32_t targ_txfifo8e;        /**< <tt>\b 0x034:</tt> I3C TARG_TXFIFO8E Register */
    __O  uint32_t targ_txfifo16;        /**< <tt>\b 0x038:</tt> I3C TARG_TXFIFO16 Register */
    __O  uint32_t targ_txfifo16e;       /**< <tt>\b 0x03C:</tt> I3C TARG_TXFIFO16E Register */
    __I  uint32_t targ_rxfifo8;         /**< <tt>\b 0x040:</tt> I3C TARG_RXFIFO8 Register */
    __R  uint32_t rsv_0x44;
    __I  uint32_t targ_rxfifo16;        /**< <tt>\b 0x048:</tt> I3C TARG_RXFIFO16 Register */
    __R  uint32_t rsv_0x4c_0x53[2];
    __O  uint32_t targ_txfifo8o;        /**< <tt>\b 0x054:</tt> I3C TARG_TXFIFO8O Register */
    __R  uint32_t rsv_0x58;
    __I  uint32_t targ_cap0;            /**< <tt>\b 0x05C:</tt> I3C TARG_CAP0 Register */
    __I  uint32_t targ_cap1;            /**< <tt>\b 0x060:</tt> I3C TARG_CAP1 Register */
    __IO uint32_t targ_dynaddr;         /**< <tt>\b 0x064:</tt> I3C TARG_DYNADDR Register */
    __IO uint32_t targ_maxlimits;       /**< <tt>\b 0x068:</tt> I3C TARG_MAXLIMITS Register */
    __R  uint32_t rsv_0x6c;
    __IO uint32_t targ_idext;           /**< <tt>\b 0x070:</tt> I3C TARG_IDEXT Register */
    __R  uint32_t rsv_0x74_0x7b[2];
    __I  uint32_t targ_msglast;         /**< <tt>\b 0x07C:</tt> I3C TARG_MSGLAST Register */
    __R  uint32_t rsv_0x80;
    __IO uint32_t cont_ctrl1;           /**< <tt>\b 0x084:</tt> I3C CONT_CTRL1 Register */
    __IO uint32_t cont_status;          /**< <tt>\b 0x088:</tt> I3C CONT_STATUS Register */
    __IO uint32_t cont_ibirules;        /**< <tt>\b 0x08C:</tt> I3C CONT_IBIRULES Register */
    __IO uint32_t cont_inten;           /**< <tt>\b 0x090:</tt> I3C CONT_INTEN Register */
    __O  uint32_t cont_intclr;          /**< <tt>\b 0x094:</tt> I3C CONT_INTCLR Register */
    __I  uint32_t cont_intfl;           /**< <tt>\b 0x098:</tt> I3C CONT_INTFL Register */
    __IO uint32_t cont_errwarn;         /**< <tt>\b 0x09C:</tt> I3C CONT_ERRWARN Register */
    __IO uint32_t cont_dmactrl;         /**< <tt>\b 0x0A0:</tt> I3C CONT_DMACTRL Register */
    __R  uint32_t rsv_0xa4_0xab[2];
    __IO uint32_t cont_fifoctrl;        /**< <tt>\b 0x0AC:</tt> I3C CONT_FIFOCTRL Register */
    __O  uint32_t cont_txfifo8;         /**< <tt>\b 0x0B0:</tt> I3C CONT_TXFIFO8 Register */
    __O  uint32_t cont_txfifo8e;        /**< <tt>\b 0x0B4:</tt> I3C CONT_TXFIFO8E Register */
    __O  uint32_t cont_txfifo16;        /**< <tt>\b 0x0B8:</tt> I3C CONT_TXFIFO16 Register */
    __O  uint32_t cont_txfifo16e;       /**< <tt>\b 0x0BC:</tt> I3C CONT_TXFIFO16E Register */
    __I  uint32_t cont_rxfifo8;         /**< <tt>\b 0x0C0:</tt> I3C CONT_RXFIFO8 Register */
    __R  uint32_t rsv_0xc4;
    __I  uint32_t cont_rxfifo16;        /**< <tt>\b 0x0C8:</tt> I3C CONT_RXFIFO16 Register */
    __O  uint32_t cont_txfifo8o;        /**< <tt>\b 0x0CC:</tt> I3C CONT_TXFIFO8O Register */
    union {
        __IO uint32_t cont_txsdrmsg_ctrl; /**< <tt>\b 0x0D0:</tt> I3C CONT_TXSDRMSG_CTRL Register */
        __O  uint32_t cont_txsdrmsg_fifo; /**< <tt>\b 0x0D0:</tt> I3C CONT_TXSDRMSG_FIFO Register */
    };
    __I  uint32_t cont_rxsdrmsg;        /**< <tt>\b 0x0D4:</tt> I3C CONT_RXSDRMSG Register */
    __O  uint32_t cont_txddrmsg;        /**< <tt>\b 0x0D8:</tt> I3C CONT_TXDDRMSG Register */
    __I  uint32_t cont_rxddr16;         /**< <tt>\b 0x0DC:</tt> I3C CONT_RXDDR16 Register */
    __R  uint32_t rsv_0xe0;
    __IO uint32_t cont_dynaddr;         /**< <tt>\b 0x0E4:</tt> I3C CONT_DYNADDR Register */
    __R  uint32_t rsv_0xe8_0x113[11];
    __I  uint32_t targ_groupdef;        /**< <tt>\b 0x114:</tt> I3C TARG_GROUPDEF Register */
    __R  uint32_t rsv_0x118;
    __IO uint32_t targ_mapctrl0;        /**< <tt>\b 0x11C:</tt> I3C TARG_MAPCTRL0 Register */
    __IO uint32_t targ_mapctrl1;        /**< <tt>\b 0x120:</tt> I3C TARG_MAPCTRL1 Register */
    __IO uint32_t targ_mapctrl2;        /**< <tt>\b 0x124:</tt> I3C TARG_MAPCTRL2 Register */
} mxc_i3c_regs_t;

/* Register offsets for module I3C */
/**
 * @ingroup    i3c_registers
 * @defgroup   I3C_Register_Offsets Register Offsets
 * @brief      I3C Peripheral Register Offsets from the I3C Base Peripheral Address.
 * @{
 */
#define MXC_R_I3C_CONT_CTRL0               ((uint32_t)0x00000000UL) /**< Offset from I3C Base Address: <tt> 0x0000</tt> */
#define MXC_R_I3C_TARG_CTRL0               ((uint32_t)0x00000004UL) /**< Offset from I3C Base Address: <tt> 0x0004</tt> */
#define MXC_R_I3C_TARG_STATUS              ((uint32_t)0x00000008UL) /**< Offset from I3C Base Address: <tt> 0x0008</tt> */
#define MXC_R_I3C_TARG_CTRL1               ((uint32_t)0x0000000CUL) /**< Offset from I3C Base Address: <tt> 0x000C</tt> */
#define MXC_R_I3C_TARG_INTEN               ((uint32_t)0x00000010UL) /**< Offset from I3C Base Address: <tt> 0x0010</tt> */
#define MXC_R_I3C_TARG_INTCLR              ((uint32_t)0x00000014UL) /**< Offset from I3C Base Address: <tt> 0x0014</tt> */
#define MXC_R_I3C_TARG_INTFL               ((uint32_t)0x00000018UL) /**< Offset from I3C Base Address: <tt> 0x0018</tt> */
#define MXC_R_I3C_TARG_ERRWARN             ((uint32_t)0x0000001CUL) /**< Offset from I3C Base Address: <tt> 0x001C</tt> */
#define MXC_R_I3C_TARG_DMACTRL             ((uint32_t)0x00000020UL) /**< Offset from I3C Base Address: <tt> 0x0020</tt> */
#define MXC_R_I3C_TARG_FIFOCTRL            ((uint32_t)0x0000002CUL) /**< Offset from I3C Base Address: <tt> 0x002C</tt> */
#define MXC_R_I3C_TARG_TXFIFO8             ((uint32_t)0x00000030UL) /**< Offset from I3C Base Address: <tt> 0x0030</tt> */
#define MXC_R_I3C_TARG_TXFIFO8E            ((uint32_t)0x00000034UL) /**< Offset from I3C Base Address: <tt> 0x0034</tt> */
#define MXC_R_I3C_TARG_TXFIFO16            ((uint32_t)0x00000038UL) /**< Offset from I3C Base Address: <tt> 0x0038</tt> */
#define MXC_R_I3C_TARG_TXFIFO16E           ((uint32_t)0x0000003CUL) /**< Offset from I3C Base Address: <tt> 0x003C</tt> */
#define MXC_R_I3C_TARG_RXFIFO8             ((uint32_t)0x00000040UL) /**< Offset from I3C Base Address: <tt> 0x0040</tt> */
#define MXC_R_I3C_TARG_RXFIFO16            ((uint32_t)0x00000048UL) /**< Offset from I3C Base Address: <tt> 0x0048</tt> */
#define MXC_R_I3C_TARG_TXFIFO8O            ((uint32_t)0x00000054UL) /**< Offset from I3C Base Address: <tt> 0x0054</tt> */
#define MXC_R_I3C_TARG_CAP0                ((uint32_t)0x0000005CUL) /**< Offset from I3C Base Address: <tt> 0x005C</tt> */
#define MXC_R_I3C_TARG_CAP1                ((uint32_t)0x00000060UL) /**< Offset from I3C Base Address: <tt> 0x0060</tt> */
#define MXC_R_I3C_TARG_DYNADDR             ((uint32_t)0x00000064UL) /**< Offset from I3C Base Address: <tt> 0x0064</tt> */
#define MXC_R_I3C_TARG_MAXLIMITS           ((uint32_t)0x00000068UL) /**< Offset from I3C Base Address: <tt> 0x0068</tt> */
#define MXC_R_I3C_TARG_IDEXT               ((uint32_t)0x00000070UL) /**< Offset from I3C Base Address: <tt> 0x0070</tt> */
#define MXC_R_I3C_TARG_MSGLAST             ((uint32_t)0x0000007CUL) /**< Offset from I3C Base Address: <tt> 0x007C</tt> */
#define MXC_R_I3C_CONT_CTRL1               ((uint32_t)0x00000084UL) /**< Offset from I3C Base Address: <tt> 0x0084</tt> */
#define MXC_R_I3C_CONT_STATUS              ((uint32_t)0x00000088UL) /**< Offset from I3C Base Address: <tt> 0x0088</tt> */
#define MXC_R_I3C_CONT_IBIRULES            ((uint32_t)0x0000008CUL) /**< Offset from I3C Base Address: <tt> 0x008C</tt> */
#define MXC_R_I3C_CONT_INTEN               ((uint32_t)0x00000090UL) /**< Offset from I3C Base Address: <tt> 0x0090</tt> */
#define MXC_R_I3C_CONT_INTCLR              ((uint32_t)0x00000094UL) /**< Offset from I3C Base Address: <tt> 0x0094</tt> */
#define MXC_R_I3C_CONT_INTFL               ((uint32_t)0x00000098UL) /**< Offset from I3C Base Address: <tt> 0x0098</tt> */
#define MXC_R_I3C_CONT_ERRWARN             ((uint32_t)0x0000009CUL) /**< Offset from I3C Base Address: <tt> 0x009C</tt> */
#define MXC_R_I3C_CONT_DMACTRL             ((uint32_t)0x000000A0UL) /**< Offset from I3C Base Address: <tt> 0x00A0</tt> */
#define MXC_R_I3C_CONT_FIFOCTRL            ((uint32_t)0x000000ACUL) /**< Offset from I3C Base Address: <tt> 0x00AC</tt> */
#define MXC_R_I3C_CONT_TXFIFO8             ((uint32_t)0x000000B0UL) /**< Offset from I3C Base Address: <tt> 0x00B0</tt> */
#define MXC_R_I3C_CONT_TXFIFO8E            ((uint32_t)0x000000B4UL) /**< Offset from I3C Base Address: <tt> 0x00B4</tt> */
#define MXC_R_I3C_CONT_TXFIFO16            ((uint32_t)0x000000B8UL) /**< Offset from I3C Base Address: <tt> 0x00B8</tt> */
#define MXC_R_I3C_CONT_TXFIFO16E           ((uint32_t)0x000000BCUL) /**< Offset from I3C Base Address: <tt> 0x00BC</tt> */
#define MXC_R_I3C_CONT_RXFIFO8             ((uint32_t)0x000000C0UL) /**< Offset from I3C Base Address: <tt> 0x00C0</tt> */
#define MXC_R_I3C_CONT_RXFIFO16            ((uint32_t)0x000000C8UL) /**< Offset from I3C Base Address: <tt> 0x00C8</tt> */
#define MXC_R_I3C_CONT_TXFIFO8O            ((uint32_t)0x000000CCUL) /**< Offset from I3C Base Address: <tt> 0x00CC</tt> */
#define MXC_R_I3C_CONT_TXSDRMSG_CTRL       ((uint32_t)0x000000D0UL) /**< Offset from I3C Base Address: <tt> 0x00D0</tt> */
#define MXC_R_I3C_CONT_TXSDRMSG_FIFO       ((uint32_t)0x000000D0UL) /**< Offset from I3C Base Address: <tt> 0x00D0</tt> */
#define MXC_R_I3C_CONT_RXSDRMSG            ((uint32_t)0x000000D4UL) /**< Offset from I3C Base Address: <tt> 0x00D4</tt> */
#define MXC_R_I3C_CONT_TXDDRMSG            ((uint32_t)0x000000D8UL) /**< Offset from I3C Base Address: <tt> 0x00D8</tt> */
#define MXC_R_I3C_CONT_RXDDR16             ((uint32_t)0x000000DCUL) /**< Offset from I3C Base Address: <tt> 0x00DC</tt> */
#define MXC_R_I3C_CONT_DYNADDR             ((uint32_t)0x000000E4UL) /**< Offset from I3C Base Address: <tt> 0x00E4</tt> */
#define MXC_R_I3C_TARG_GROUPDEF            ((uint32_t)0x00000114UL) /**< Offset from I3C Base Address: <tt> 0x0114</tt> */
#define MXC_R_I3C_TARG_MAPCTRL0            ((uint32_t)0x0000011CUL) /**< Offset from I3C Base Address: <tt> 0x011C</tt> */
#define MXC_R_I3C_TARG_MAPCTRL1            ((uint32_t)0x00000120UL) /**< Offset from I3C Base Address: <tt> 0x0120</tt> */
#define MXC_R_I3C_TARG_MAPCTRL2            ((uint32_t)0x00000124UL) /**< Offset from I3C Base Address: <tt> 0x0124</tt> */
/**@} end of group i3c_registers */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_CONT_CTRL0 I3C_CONT_CTRL0
 * @brief    Controller Control 0 (Configuration) Register.
 * @{
 */
#define MXC_F_I3C_CONT_CTRL0_EN_POS                    0 /**< CONT_CTRL0_EN Position */
#define MXC_F_I3C_CONT_CTRL0_EN                        ((uint32_t)(0x3UL << MXC_F_I3C_CONT_CTRL0_EN_POS)) /**< CONT_CTRL0_EN Mask */
#define MXC_V_I3C_CONT_CTRL0_EN_OFF                    ((uint32_t)0x0UL) /**< CONT_CTRL0_EN_OFF Value */
#define MXC_S_I3C_CONT_CTRL0_EN_OFF                    (MXC_V_I3C_CONT_CTRL0_EN_OFF << MXC_F_I3C_CONT_CTRL0_EN_POS) /**< CONT_CTRL0_EN_OFF Setting */
#define MXC_V_I3C_CONT_CTRL0_EN_ON                     ((uint32_t)0x1UL) /**< CONT_CTRL0_EN_ON Value */
#define MXC_S_I3C_CONT_CTRL0_EN_ON                     (MXC_V_I3C_CONT_CTRL0_EN_ON << MXC_F_I3C_CONT_CTRL0_EN_POS) /**< CONT_CTRL0_EN_ON Setting */
#define MXC_V_I3C_CONT_CTRL0_EN_CAP                    ((uint32_t)0x2UL) /**< CONT_CTRL0_EN_CAP Value */
#define MXC_S_I3C_CONT_CTRL0_EN_CAP                    (MXC_V_I3C_CONT_CTRL0_EN_CAP << MXC_F_I3C_CONT_CTRL0_EN_POS) /**< CONT_CTRL0_EN_CAP Setting */

#define MXC_F_I3C_CONT_CTRL0_TO_DIS_POS                3 /**< CONT_CTRL0_TO_DIS Position */
#define MXC_F_I3C_CONT_CTRL0_TO_DIS                    ((uint32_t)(0x1UL << MXC_F_I3C_CONT_CTRL0_TO_DIS_POS)) /**< CONT_CTRL0_TO_DIS Mask */

#define MXC_F_I3C_CONT_CTRL0_HKEEP_POS                 4 /**< CONT_CTRL0_HKEEP Position */
#define MXC_F_I3C_CONT_CTRL0_HKEEP                     ((uint32_t)(0x3UL << MXC_F_I3C_CONT_CTRL0_HKEEP_POS)) /**< CONT_CTRL0_HKEEP Mask */
#define MXC_V_I3C_CONT_CTRL0_HKEEP_OFF                 ((uint32_t)0x0UL) /**< CONT_CTRL0_HKEEP_OFF Value */
#define MXC_S_I3C_CONT_CTRL0_HKEEP_OFF                 (MXC_V_I3C_CONT_CTRL0_HKEEP_OFF << MXC_F_I3C_CONT_CTRL0_HKEEP_POS) /**< CONT_CTRL0_HKEEP_OFF Setting */
#define MXC_V_I3C_CONT_CTRL0_HKEEP_ON_CHIP             ((uint32_t)0x1UL) /**< CONT_CTRL0_HKEEP_ON_CHIP Value */
#define MXC_S_I3C_CONT_CTRL0_HKEEP_ON_CHIP             (MXC_V_I3C_CONT_CTRL0_HKEEP_ON_CHIP << MXC_F_I3C_CONT_CTRL0_HKEEP_POS) /**< CONT_CTRL0_HKEEP_ON_CHIP Setting */
#define MXC_V_I3C_CONT_CTRL0_HKEEP_EXT_SDA             ((uint32_t)0x2UL) /**< CONT_CTRL0_HKEEP_EXT_SDA Value */
#define MXC_S_I3C_CONT_CTRL0_HKEEP_EXT_SDA             (MXC_V_I3C_CONT_CTRL0_HKEEP_EXT_SDA << MXC_F_I3C_CONT_CTRL0_HKEEP_POS) /**< CONT_CTRL0_HKEEP_EXT_SDA Setting */
#define MXC_V_I3C_CONT_CTRL0_HKEEP_EXT_SCL_SDA         ((uint32_t)0x3UL) /**< CONT_CTRL0_HKEEP_EXT_SCL_SDA Value */
#define MXC_S_I3C_CONT_CTRL0_HKEEP_EXT_SCL_SDA         (MXC_V_I3C_CONT_CTRL0_HKEEP_EXT_SCL_SDA << MXC_F_I3C_CONT_CTRL0_HKEEP_POS) /**< CONT_CTRL0_HKEEP_EXT_SCL_SDA Setting */

#define MXC_F_I3C_CONT_CTRL0_OD_STOP_POS               6 /**< CONT_CTRL0_OD_STOP Position */
#define MXC_F_I3C_CONT_CTRL0_OD_STOP                   ((uint32_t)(0x1UL << MXC_F_I3C_CONT_CTRL0_OD_STOP_POS)) /**< CONT_CTRL0_OD_STOP Mask */

#define MXC_F_I3C_CONT_CTRL0_PP_BAUD_POS               8 /**< CONT_CTRL0_PP_BAUD Position */
#define MXC_F_I3C_CONT_CTRL0_PP_BAUD                   ((uint32_t)(0xFUL << MXC_F_I3C_CONT_CTRL0_PP_BAUD_POS)) /**< CONT_CTRL0_PP_BAUD Mask */
#define MXC_V_I3C_CONT_CTRL0_PP_BAUD_1_FCLK            ((uint32_t)0x0UL) /**< CONT_CTRL0_PP_BAUD_1_FCLK Value */
#define MXC_S_I3C_CONT_CTRL0_PP_BAUD_1_FCLK            (MXC_V_I3C_CONT_CTRL0_PP_BAUD_1_FCLK << MXC_F_I3C_CONT_CTRL0_PP_BAUD_POS) /**< CONT_CTRL0_PP_BAUD_1_FCLK Setting */
#define MXC_V_I3C_CONT_CTRL0_PP_BAUD_2_FCLK            ((uint32_t)0x1UL) /**< CONT_CTRL0_PP_BAUD_2_FCLK Value */
#define MXC_S_I3C_CONT_CTRL0_PP_BAUD_2_FCLK            (MXC_V_I3C_CONT_CTRL0_PP_BAUD_2_FCLK << MXC_F_I3C_CONT_CTRL0_PP_BAUD_POS) /**< CONT_CTRL0_PP_BAUD_2_FCLK Setting */
#define MXC_V_I3C_CONT_CTRL0_PP_BAUD_3_FCLK            ((uint32_t)0x2UL) /**< CONT_CTRL0_PP_BAUD_3_FCLK Value */
#define MXC_S_I3C_CONT_CTRL0_PP_BAUD_3_FCLK            (MXC_V_I3C_CONT_CTRL0_PP_BAUD_3_FCLK << MXC_F_I3C_CONT_CTRL0_PP_BAUD_POS) /**< CONT_CTRL0_PP_BAUD_3_FCLK Setting */
#define MXC_V_I3C_CONT_CTRL0_PP_BAUD_4_FCLK            ((uint32_t)0x3UL) /**< CONT_CTRL0_PP_BAUD_4_FCLK Value */
#define MXC_S_I3C_CONT_CTRL0_PP_BAUD_4_FCLK            (MXC_V_I3C_CONT_CTRL0_PP_BAUD_4_FCLK << MXC_F_I3C_CONT_CTRL0_PP_BAUD_POS) /**< CONT_CTRL0_PP_BAUD_4_FCLK Setting */
#define MXC_V_I3C_CONT_CTRL0_PP_BAUD_5_FCLK            ((uint32_t)0x4UL) /**< CONT_CTRL0_PP_BAUD_5_FCLK Value */
#define MXC_S_I3C_CONT_CTRL0_PP_BAUD_5_FCLK            (MXC_V_I3C_CONT_CTRL0_PP_BAUD_5_FCLK << MXC_F_I3C_CONT_CTRL0_PP_BAUD_POS) /**< CONT_CTRL0_PP_BAUD_5_FCLK Setting */
#define MXC_V_I3C_CONT_CTRL0_PP_BAUD_6_FCLK            ((uint32_t)0x5UL) /**< CONT_CTRL0_PP_BAUD_6_FCLK Value */
#define MXC_S_I3C_CONT_CTRL0_PP_BAUD_6_FCLK            (MXC_V_I3C_CONT_CTRL0_PP_BAUD_6_FCLK << MXC_F_I3C_CONT_CTRL0_PP_BAUD_POS) /**< CONT_CTRL0_PP_BAUD_6_FCLK Setting */
#define MXC_V_I3C_CONT_CTRL0_PP_BAUD_7_FCLK            ((uint32_t)0x6UL) /**< CONT_CTRL0_PP_BAUD_7_FCLK Value */
#define MXC_S_I3C_CONT_CTRL0_PP_BAUD_7_FCLK            (MXC_V_I3C_CONT_CTRL0_PP_BAUD_7_FCLK << MXC_F_I3C_CONT_CTRL0_PP_BAUD_POS) /**< CONT_CTRL0_PP_BAUD_7_FCLK Setting */
#define MXC_V_I3C_CONT_CTRL0_PP_BAUD_8_FCLK            ((uint32_t)0x7UL) /**< CONT_CTRL0_PP_BAUD_8_FCLK Value */
#define MXC_S_I3C_CONT_CTRL0_PP_BAUD_8_FCLK            (MXC_V_I3C_CONT_CTRL0_PP_BAUD_8_FCLK << MXC_F_I3C_CONT_CTRL0_PP_BAUD_POS) /**< CONT_CTRL0_PP_BAUD_8_FCLK Setting */
#define MXC_V_I3C_CONT_CTRL0_PP_BAUD_9_FCLK            ((uint32_t)0x8UL) /**< CONT_CTRL0_PP_BAUD_9_FCLK Value */
#define MXC_S_I3C_CONT_CTRL0_PP_BAUD_9_FCLK            (MXC_V_I3C_CONT_CTRL0_PP_BAUD_9_FCLK << MXC_F_I3C_CONT_CTRL0_PP_BAUD_POS) /**< CONT_CTRL0_PP_BAUD_9_FCLK Setting */
#define MXC_V_I3C_CONT_CTRL0_PP_BAUD_10_FCLK           ((uint32_t)0x9UL) /**< CONT_CTRL0_PP_BAUD_10_FCLK Value */
#define MXC_S_I3C_CONT_CTRL0_PP_BAUD_10_FCLK           (MXC_V_I3C_CONT_CTRL0_PP_BAUD_10_FCLK << MXC_F_I3C_CONT_CTRL0_PP_BAUD_POS) /**< CONT_CTRL0_PP_BAUD_10_FCLK Setting */
#define MXC_V_I3C_CONT_CTRL0_PP_BAUD_11_FCLK           ((uint32_t)0xAUL) /**< CONT_CTRL0_PP_BAUD_11_FCLK Value */
#define MXC_S_I3C_CONT_CTRL0_PP_BAUD_11_FCLK           (MXC_V_I3C_CONT_CTRL0_PP_BAUD_11_FCLK << MXC_F_I3C_CONT_CTRL0_PP_BAUD_POS) /**< CONT_CTRL0_PP_BAUD_11_FCLK Setting */
#define MXC_V_I3C_CONT_CTRL0_PP_BAUD_12_FCLK           ((uint32_t)0xBUL) /**< CONT_CTRL0_PP_BAUD_12_FCLK Value */
#define MXC_S_I3C_CONT_CTRL0_PP_BAUD_12_FCLK           (MXC_V_I3C_CONT_CTRL0_PP_BAUD_12_FCLK << MXC_F_I3C_CONT_CTRL0_PP_BAUD_POS) /**< CONT_CTRL0_PP_BAUD_12_FCLK Setting */
#define MXC_V_I3C_CONT_CTRL0_PP_BAUD_13_FCLK           ((uint32_t)0xCUL) /**< CONT_CTRL0_PP_BAUD_13_FCLK Value */
#define MXC_S_I3C_CONT_CTRL0_PP_BAUD_13_FCLK           (MXC_V_I3C_CONT_CTRL0_PP_BAUD_13_FCLK << MXC_F_I3C_CONT_CTRL0_PP_BAUD_POS) /**< CONT_CTRL0_PP_BAUD_13_FCLK Setting */
#define MXC_V_I3C_CONT_CTRL0_PP_BAUD_14_FCLK           ((uint32_t)0xDUL) /**< CONT_CTRL0_PP_BAUD_14_FCLK Value */
#define MXC_S_I3C_CONT_CTRL0_PP_BAUD_14_FCLK           (MXC_V_I3C_CONT_CTRL0_PP_BAUD_14_FCLK << MXC_F_I3C_CONT_CTRL0_PP_BAUD_POS) /**< CONT_CTRL0_PP_BAUD_14_FCLK Setting */
#define MXC_V_I3C_CONT_CTRL0_PP_BAUD_15_FCLK           ((uint32_t)0xEUL) /**< CONT_CTRL0_PP_BAUD_15_FCLK Value */
#define MXC_S_I3C_CONT_CTRL0_PP_BAUD_15_FCLK           (MXC_V_I3C_CONT_CTRL0_PP_BAUD_15_FCLK << MXC_F_I3C_CONT_CTRL0_PP_BAUD_POS) /**< CONT_CTRL0_PP_BAUD_15_FCLK Setting */
#define MXC_V_I3C_CONT_CTRL0_PP_BAUD_16_FCLK           ((uint32_t)0xFUL) /**< CONT_CTRL0_PP_BAUD_16_FCLK Value */
#define MXC_S_I3C_CONT_CTRL0_PP_BAUD_16_FCLK           (MXC_V_I3C_CONT_CTRL0_PP_BAUD_16_FCLK << MXC_F_I3C_CONT_CTRL0_PP_BAUD_POS) /**< CONT_CTRL0_PP_BAUD_16_FCLK Setting */

#define MXC_F_I3C_CONT_CTRL0_PP_ADD_LBAUD_POS          12 /**< CONT_CTRL0_PP_ADD_LBAUD Position */
#define MXC_F_I3C_CONT_CTRL0_PP_ADD_LBAUD              ((uint32_t)(0xFUL << MXC_F_I3C_CONT_CTRL0_PP_ADD_LBAUD_POS)) /**< CONT_CTRL0_PP_ADD_LBAUD Mask */
#define MXC_V_I3C_CONT_CTRL0_PP_ADD_LBAUD_0_FCLK       ((uint32_t)0x0UL) /**< CONT_CTRL0_PP_ADD_LBAUD_0_FCLK Value */
#define MXC_S_I3C_CONT_CTRL0_PP_ADD_LBAUD_0_FCLK       (MXC_V_I3C_CONT_CTRL0_PP_ADD_LBAUD_0_FCLK << MXC_F_I3C_CONT_CTRL0_PP_ADD_LBAUD_POS) /**< CONT_CTRL0_PP_ADD_LBAUD_0_FCLK Setting */
#define MXC_V_I3C_CONT_CTRL0_PP_ADD_LBAUD_1_FCLK       ((uint32_t)0x1UL) /**< CONT_CTRL0_PP_ADD_LBAUD_1_FCLK Value */
#define MXC_S_I3C_CONT_CTRL0_PP_ADD_LBAUD_1_FCLK       (MXC_V_I3C_CONT_CTRL0_PP_ADD_LBAUD_1_FCLK << MXC_F_I3C_CONT_CTRL0_PP_ADD_LBAUD_POS) /**< CONT_CTRL0_PP_ADD_LBAUD_1_FCLK Setting */
#define MXC_V_I3C_CONT_CTRL0_PP_ADD_LBAUD_2_FCLK       ((uint32_t)0x2UL) /**< CONT_CTRL0_PP_ADD_LBAUD_2_FCLK Value */
#define MXC_S_I3C_CONT_CTRL0_PP_ADD_LBAUD_2_FCLK       (MXC_V_I3C_CONT_CTRL0_PP_ADD_LBAUD_2_FCLK << MXC_F_I3C_CONT_CTRL0_PP_ADD_LBAUD_POS) /**< CONT_CTRL0_PP_ADD_LBAUD_2_FCLK Setting */
#define MXC_V_I3C_CONT_CTRL0_PP_ADD_LBAUD_3_FCLK       ((uint32_t)0x3UL) /**< CONT_CTRL0_PP_ADD_LBAUD_3_FCLK Value */
#define MXC_S_I3C_CONT_CTRL0_PP_ADD_LBAUD_3_FCLK       (MXC_V_I3C_CONT_CTRL0_PP_ADD_LBAUD_3_FCLK << MXC_F_I3C_CONT_CTRL0_PP_ADD_LBAUD_POS) /**< CONT_CTRL0_PP_ADD_LBAUD_3_FCLK Setting */
#define MXC_V_I3C_CONT_CTRL0_PP_ADD_LBAUD_4_FCLK       ((uint32_t)0x4UL) /**< CONT_CTRL0_PP_ADD_LBAUD_4_FCLK Value */
#define MXC_S_I3C_CONT_CTRL0_PP_ADD_LBAUD_4_FCLK       (MXC_V_I3C_CONT_CTRL0_PP_ADD_LBAUD_4_FCLK << MXC_F_I3C_CONT_CTRL0_PP_ADD_LBAUD_POS) /**< CONT_CTRL0_PP_ADD_LBAUD_4_FCLK Setting */
#define MXC_V_I3C_CONT_CTRL0_PP_ADD_LBAUD_5_FCLK       ((uint32_t)0x5UL) /**< CONT_CTRL0_PP_ADD_LBAUD_5_FCLK Value */
#define MXC_S_I3C_CONT_CTRL0_PP_ADD_LBAUD_5_FCLK       (MXC_V_I3C_CONT_CTRL0_PP_ADD_LBAUD_5_FCLK << MXC_F_I3C_CONT_CTRL0_PP_ADD_LBAUD_POS) /**< CONT_CTRL0_PP_ADD_LBAUD_5_FCLK Setting */
#define MXC_V_I3C_CONT_CTRL0_PP_ADD_LBAUD_6_FCLK       ((uint32_t)0x6UL) /**< CONT_CTRL0_PP_ADD_LBAUD_6_FCLK Value */
#define MXC_S_I3C_CONT_CTRL0_PP_ADD_LBAUD_6_FCLK       (MXC_V_I3C_CONT_CTRL0_PP_ADD_LBAUD_6_FCLK << MXC_F_I3C_CONT_CTRL0_PP_ADD_LBAUD_POS) /**< CONT_CTRL0_PP_ADD_LBAUD_6_FCLK Setting */
#define MXC_V_I3C_CONT_CTRL0_PP_ADD_LBAUD_7_FCLK       ((uint32_t)0x7UL) /**< CONT_CTRL0_PP_ADD_LBAUD_7_FCLK Value */
#define MXC_S_I3C_CONT_CTRL0_PP_ADD_LBAUD_7_FCLK       (MXC_V_I3C_CONT_CTRL0_PP_ADD_LBAUD_7_FCLK << MXC_F_I3C_CONT_CTRL0_PP_ADD_LBAUD_POS) /**< CONT_CTRL0_PP_ADD_LBAUD_7_FCLK Setting */
#define MXC_V_I3C_CONT_CTRL0_PP_ADD_LBAUD_8_FCLK       ((uint32_t)0x8UL) /**< CONT_CTRL0_PP_ADD_LBAUD_8_FCLK Value */
#define MXC_S_I3C_CONT_CTRL0_PP_ADD_LBAUD_8_FCLK       (MXC_V_I3C_CONT_CTRL0_PP_ADD_LBAUD_8_FCLK << MXC_F_I3C_CONT_CTRL0_PP_ADD_LBAUD_POS) /**< CONT_CTRL0_PP_ADD_LBAUD_8_FCLK Setting */
#define MXC_V_I3C_CONT_CTRL0_PP_ADD_LBAUD_9_FCLK       ((uint32_t)0x9UL) /**< CONT_CTRL0_PP_ADD_LBAUD_9_FCLK Value */
#define MXC_S_I3C_CONT_CTRL0_PP_ADD_LBAUD_9_FCLK       (MXC_V_I3C_CONT_CTRL0_PP_ADD_LBAUD_9_FCLK << MXC_F_I3C_CONT_CTRL0_PP_ADD_LBAUD_POS) /**< CONT_CTRL0_PP_ADD_LBAUD_9_FCLK Setting */
#define MXC_V_I3C_CONT_CTRL0_PP_ADD_LBAUD_10_FCLK      ((uint32_t)0xAUL) /**< CONT_CTRL0_PP_ADD_LBAUD_10_FCLK Value */
#define MXC_S_I3C_CONT_CTRL0_PP_ADD_LBAUD_10_FCLK      (MXC_V_I3C_CONT_CTRL0_PP_ADD_LBAUD_10_FCLK << MXC_F_I3C_CONT_CTRL0_PP_ADD_LBAUD_POS) /**< CONT_CTRL0_PP_ADD_LBAUD_10_FCLK Setting */
#define MXC_V_I3C_CONT_CTRL0_PP_ADD_LBAUD_11_FCLK      ((uint32_t)0xBUL) /**< CONT_CTRL0_PP_ADD_LBAUD_11_FCLK Value */
#define MXC_S_I3C_CONT_CTRL0_PP_ADD_LBAUD_11_FCLK      (MXC_V_I3C_CONT_CTRL0_PP_ADD_LBAUD_11_FCLK << MXC_F_I3C_CONT_CTRL0_PP_ADD_LBAUD_POS) /**< CONT_CTRL0_PP_ADD_LBAUD_11_FCLK Setting */
#define MXC_V_I3C_CONT_CTRL0_PP_ADD_LBAUD_12_FCLK      ((uint32_t)0xCUL) /**< CONT_CTRL0_PP_ADD_LBAUD_12_FCLK Value */
#define MXC_S_I3C_CONT_CTRL0_PP_ADD_LBAUD_12_FCLK      (MXC_V_I3C_CONT_CTRL0_PP_ADD_LBAUD_12_FCLK << MXC_F_I3C_CONT_CTRL0_PP_ADD_LBAUD_POS) /**< CONT_CTRL0_PP_ADD_LBAUD_12_FCLK Setting */
#define MXC_V_I3C_CONT_CTRL0_PP_ADD_LBAUD_13_FCLK      ((uint32_t)0xDUL) /**< CONT_CTRL0_PP_ADD_LBAUD_13_FCLK Value */
#define MXC_S_I3C_CONT_CTRL0_PP_ADD_LBAUD_13_FCLK      (MXC_V_I3C_CONT_CTRL0_PP_ADD_LBAUD_13_FCLK << MXC_F_I3C_CONT_CTRL0_PP_ADD_LBAUD_POS) /**< CONT_CTRL0_PP_ADD_LBAUD_13_FCLK Setting */
#define MXC_V_I3C_CONT_CTRL0_PP_ADD_LBAUD_14_FCLK      ((uint32_t)0xEUL) /**< CONT_CTRL0_PP_ADD_LBAUD_14_FCLK Value */
#define MXC_S_I3C_CONT_CTRL0_PP_ADD_LBAUD_14_FCLK      (MXC_V_I3C_CONT_CTRL0_PP_ADD_LBAUD_14_FCLK << MXC_F_I3C_CONT_CTRL0_PP_ADD_LBAUD_POS) /**< CONT_CTRL0_PP_ADD_LBAUD_14_FCLK Setting */
#define MXC_V_I3C_CONT_CTRL0_PP_ADD_LBAUD_15_FCLK      ((uint32_t)0xFUL) /**< CONT_CTRL0_PP_ADD_LBAUD_15_FCLK Value */
#define MXC_S_I3C_CONT_CTRL0_PP_ADD_LBAUD_15_FCLK      (MXC_V_I3C_CONT_CTRL0_PP_ADD_LBAUD_15_FCLK << MXC_F_I3C_CONT_CTRL0_PP_ADD_LBAUD_POS) /**< CONT_CTRL0_PP_ADD_LBAUD_15_FCLK Setting */

#define MXC_F_I3C_CONT_CTRL0_OD_LBAUD_POS              16 /**< CONT_CTRL0_OD_LBAUD Position */
#define MXC_F_I3C_CONT_CTRL0_OD_LBAUD                  ((uint32_t)(0xFFUL << MXC_F_I3C_CONT_CTRL0_OD_LBAUD_POS)) /**< CONT_CTRL0_OD_LBAUD Mask */

#define MXC_F_I3C_CONT_CTRL0_OD_HP_POS                 24 /**< CONT_CTRL0_OD_HP Position */
#define MXC_F_I3C_CONT_CTRL0_OD_HP                     ((uint32_t)(0x1UL << MXC_F_I3C_CONT_CTRL0_OD_HP_POS)) /**< CONT_CTRL0_OD_HP Mask */

#define MXC_F_I3C_CONT_CTRL0_PP_SKEW_POS               25 /**< CONT_CTRL0_PP_SKEW Position */
#define MXC_F_I3C_CONT_CTRL0_PP_SKEW                   ((uint32_t)(0x7UL << MXC_F_I3C_CONT_CTRL0_PP_SKEW_POS)) /**< CONT_CTRL0_PP_SKEW Mask */

#define MXC_F_I3C_CONT_CTRL0_I2C_BAUD_POS              28 /**< CONT_CTRL0_I2C_BAUD Position */
#define MXC_F_I3C_CONT_CTRL0_I2C_BAUD                  ((uint32_t)(0xFUL << MXC_F_I3C_CONT_CTRL0_I2C_BAUD_POS)) /**< CONT_CTRL0_I2C_BAUD Mask */

/**@} end of group I3C_CONT_CTRL0_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_TARG_CTRL0 I3C_TARG_CTRL0
 * @brief    Target Control 0 (Configuration) Register.
 * @{
 */
#define MXC_F_I3C_TARG_CTRL0_EN_POS                    0 /**< TARG_CTRL0_EN Position */
#define MXC_F_I3C_TARG_CTRL0_EN                        ((uint32_t)(0x1UL << MXC_F_I3C_TARG_CTRL0_EN_POS)) /**< TARG_CTRL0_EN Mask */

#define MXC_F_I3C_TARG_CTRL0_MATCHSS_POS               2 /**< TARG_CTRL0_MATCHSS Position */
#define MXC_F_I3C_TARG_CTRL0_MATCHSS                   ((uint32_t)(0x1UL << MXC_F_I3C_TARG_CTRL0_MATCHSS_POS)) /**< TARG_CTRL0_MATCHSS Mask */

#define MXC_F_I3C_TARG_CTRL0_TO_IGN_POS                3 /**< TARG_CTRL0_TO_IGN Position */
#define MXC_F_I3C_TARG_CTRL0_TO_IGN                    ((uint32_t)(0x1UL << MXC_F_I3C_TARG_CTRL0_TO_IGN_POS)) /**< TARG_CTRL0_TO_IGN Mask */

#define MXC_F_I3C_TARG_CTRL0_OFFLINE_POS               9 /**< TARG_CTRL0_OFFLINE Position */
#define MXC_F_I3C_TARG_CTRL0_OFFLINE                   ((uint32_t)(0x1UL << MXC_F_I3C_TARG_CTRL0_OFFLINE_POS)) /**< TARG_CTRL0_OFFLINE Mask */

/**@} end of group I3C_TARG_CTRL0_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_TARG_STATUS I3C_TARG_STATUS
 * @brief    Target Status Register.
 * @{
 */
#define MXC_F_I3C_TARG_STATUS_BUSY_POS                 0 /**< TARG_STATUS_BUSY Position */
#define MXC_F_I3C_TARG_STATUS_BUSY                     ((uint32_t)(0x1UL << MXC_F_I3C_TARG_STATUS_BUSY_POS)) /**< TARG_STATUS_BUSY Mask */

#define MXC_F_I3C_TARG_STATUS_LIST_RESP_POS            1 /**< TARG_STATUS_LIST_RESP Position */
#define MXC_F_I3C_TARG_STATUS_LIST_RESP                ((uint32_t)(0x1UL << MXC_F_I3C_TARG_STATUS_LIST_RESP_POS)) /**< TARG_STATUS_LIST_RESP Mask */

#define MXC_F_I3C_TARG_STATUS_CCCH_POS                 2 /**< TARG_STATUS_CCCH Position */
#define MXC_F_I3C_TARG_STATUS_CCCH                     ((uint32_t)(0x1UL << MXC_F_I3C_TARG_STATUS_CCCH_POS)) /**< TARG_STATUS_CCCH Mask */

#define MXC_F_I3C_TARG_STATUS_RX_SDR_POS               3 /**< TARG_STATUS_RX_SDR Position */
#define MXC_F_I3C_TARG_STATUS_RX_SDR                   ((uint32_t)(0x1UL << MXC_F_I3C_TARG_STATUS_RX_SDR_POS)) /**< TARG_STATUS_RX_SDR Mask */

#define MXC_F_I3C_TARG_STATUS_TX_SDR_POS               4 /**< TARG_STATUS_TX_SDR Position */
#define MXC_F_I3C_TARG_STATUS_TX_SDR                   ((uint32_t)(0x1UL << MXC_F_I3C_TARG_STATUS_TX_SDR_POS)) /**< TARG_STATUS_TX_SDR Mask */

#define MXC_F_I3C_TARG_STATUS_DAA_POS                  5 /**< TARG_STATUS_DAA Position */
#define MXC_F_I3C_TARG_STATUS_DAA                      ((uint32_t)(0x1UL << MXC_F_I3C_TARG_STATUS_DAA_POS)) /**< TARG_STATUS_DAA Mask */

#define MXC_F_I3C_TARG_STATUS_HDR_POS                  6 /**< TARG_STATUS_HDR Position */
#define MXC_F_I3C_TARG_STATUS_HDR                      ((uint32_t)(0x1UL << MXC_F_I3C_TARG_STATUS_HDR_POS)) /**< TARG_STATUS_HDR Mask */

#define MXC_F_I3C_TARG_STATUS_START_POS                8 /**< TARG_STATUS_START Position */
#define MXC_F_I3C_TARG_STATUS_START                    ((uint32_t)(0x1UL << MXC_F_I3C_TARG_STATUS_START_POS)) /**< TARG_STATUS_START Mask */

#define MXC_F_I3C_TARG_STATUS_ADDRMATCH_POS            9 /**< TARG_STATUS_ADDRMATCH Position */
#define MXC_F_I3C_TARG_STATUS_ADDRMATCH                ((uint32_t)(0x1UL << MXC_F_I3C_TARG_STATUS_ADDRMATCH_POS)) /**< TARG_STATUS_ADDRMATCH Mask */

#define MXC_F_I3C_TARG_STATUS_STOP_POS                 10 /**< TARG_STATUS_STOP Position */
#define MXC_F_I3C_TARG_STATUS_STOP                     ((uint32_t)(0x1UL << MXC_F_I3C_TARG_STATUS_STOP_POS)) /**< TARG_STATUS_STOP Mask */

#define MXC_F_I3C_TARG_STATUS_RX_RDY_POS               11 /**< TARG_STATUS_RX_RDY Position */
#define MXC_F_I3C_TARG_STATUS_RX_RDY                   ((uint32_t)(0x1UL << MXC_F_I3C_TARG_STATUS_RX_RDY_POS)) /**< TARG_STATUS_RX_RDY Mask */

#define MXC_F_I3C_TARG_STATUS_TX_NFULL_POS             12 /**< TARG_STATUS_TX_NFULL Position */
#define MXC_F_I3C_TARG_STATUS_TX_NFULL                 ((uint32_t)(0x1UL << MXC_F_I3C_TARG_STATUS_TX_NFULL_POS)) /**< TARG_STATUS_TX_NFULL Mask */

#define MXC_F_I3C_TARG_STATUS_DYNADDR_CHG_POS          13 /**< TARG_STATUS_DYNADDR_CHG Position */
#define MXC_F_I3C_TARG_STATUS_DYNADDR_CHG              ((uint32_t)(0x1UL << MXC_F_I3C_TARG_STATUS_DYNADDR_CHG_POS)) /**< TARG_STATUS_DYNADDR_CHG Mask */

#define MXC_F_I3C_TARG_STATUS_CCC_POS                  14 /**< TARG_STATUS_CCC Position */
#define MXC_F_I3C_TARG_STATUS_CCC                      ((uint32_t)(0x1UL << MXC_F_I3C_TARG_STATUS_CCC_POS)) /**< TARG_STATUS_CCC Mask */

#define MXC_F_I3C_TARG_STATUS_ERRWARN_POS              15 /**< TARG_STATUS_ERRWARN Position */
#define MXC_F_I3C_TARG_STATUS_ERRWARN                  ((uint32_t)(0x1UL << MXC_F_I3C_TARG_STATUS_ERRWARN_POS)) /**< TARG_STATUS_ERRWARN Mask */

#define MXC_F_I3C_TARG_STATUS_CCCH_DONE_POS            17 /**< TARG_STATUS_CCCH_DONE Position */
#define MXC_F_I3C_TARG_STATUS_CCCH_DONE                ((uint32_t)(0x1UL << MXC_F_I3C_TARG_STATUS_CCCH_DONE_POS)) /**< TARG_STATUS_CCCH_DONE Mask */

#define MXC_F_I3C_TARG_STATUS_EVENT_REQ_POS            18 /**< TARG_STATUS_EVENT_REQ Position */
#define MXC_F_I3C_TARG_STATUS_EVENT_REQ                ((uint32_t)(0x1UL << MXC_F_I3C_TARG_STATUS_EVENT_REQ_POS)) /**< TARG_STATUS_EVENT_REQ Mask */

#define MXC_F_I3C_TARG_STATUS_TARG_RST_POS             19 /**< TARG_STATUS_TARG_RST Position */
#define MXC_F_I3C_TARG_STATUS_TARG_RST                 ((uint32_t)(0x1UL << MXC_F_I3C_TARG_STATUS_TARG_RST_POS)) /**< TARG_STATUS_TARG_RST Mask */

#define MXC_F_I3C_TARG_STATUS_EVENT_POS                20 /**< TARG_STATUS_EVENT Position */
#define MXC_F_I3C_TARG_STATUS_EVENT                    ((uint32_t)(0x3UL << MXC_F_I3C_TARG_STATUS_EVENT_POS)) /**< TARG_STATUS_EVENT Mask */
#define MXC_V_I3C_TARG_STATUS_EVENT_NONE               ((uint32_t)0x0UL) /**< TARG_STATUS_EVENT_NONE Value */
#define MXC_S_I3C_TARG_STATUS_EVENT_NONE               (MXC_V_I3C_TARG_STATUS_EVENT_NONE << MXC_F_I3C_TARG_STATUS_EVENT_POS) /**< TARG_STATUS_EVENT_NONE Setting */
#define MXC_V_I3C_TARG_STATUS_EVENT_REQ_PEND           ((uint32_t)0x1UL) /**< TARG_STATUS_EVENT_REQ_PEND Value */
#define MXC_S_I3C_TARG_STATUS_EVENT_REQ_PEND           (MXC_V_I3C_TARG_STATUS_EVENT_REQ_PEND << MXC_F_I3C_TARG_STATUS_EVENT_POS) /**< TARG_STATUS_EVENT_REQ_PEND Setting */
#define MXC_V_I3C_TARG_STATUS_EVENT_REQ_NACK           ((uint32_t)0x2UL) /**< TARG_STATUS_EVENT_REQ_NACK Value */
#define MXC_S_I3C_TARG_STATUS_EVENT_REQ_NACK           (MXC_V_I3C_TARG_STATUS_EVENT_REQ_NACK << MXC_F_I3C_TARG_STATUS_EVENT_POS) /**< TARG_STATUS_EVENT_REQ_NACK Setting */
#define MXC_V_I3C_TARG_STATUS_EVENT_REQ_ACK            ((uint32_t)0x3UL) /**< TARG_STATUS_EVENT_REQ_ACK Value */
#define MXC_S_I3C_TARG_STATUS_EVENT_REQ_ACK            (MXC_V_I3C_TARG_STATUS_EVENT_REQ_ACK << MXC_F_I3C_TARG_STATUS_EVENT_POS) /**< TARG_STATUS_EVENT_REQ_ACK Setting */

#define MXC_F_I3C_TARG_STATUS_IBI_DIS_POS              24 /**< TARG_STATUS_IBI_DIS Position */
#define MXC_F_I3C_TARG_STATUS_IBI_DIS                  ((uint32_t)(0x1UL << MXC_F_I3C_TARG_STATUS_IBI_DIS_POS)) /**< TARG_STATUS_IBI_DIS Mask */

#define MXC_F_I3C_TARG_STATUS_CONTREQ_DIS_POS          25 /**< TARG_STATUS_CONTREQ_DIS Position */
#define MXC_F_I3C_TARG_STATUS_CONTREQ_DIS              ((uint32_t)(0x1UL << MXC_F_I3C_TARG_STATUS_CONTREQ_DIS_POS)) /**< TARG_STATUS_CONTREQ_DIS Mask */

#define MXC_F_I3C_TARG_STATUS_HJ_DIS_POS               27 /**< TARG_STATUS_HJ_DIS Position */
#define MXC_F_I3C_TARG_STATUS_HJ_DIS                   ((uint32_t)(0x1UL << MXC_F_I3C_TARG_STATUS_HJ_DIS_POS)) /**< TARG_STATUS_HJ_DIS Mask */

#define MXC_F_I3C_TARG_STATUS_ACTSTATE_POS             28 /**< TARG_STATUS_ACTSTATE Position */
#define MXC_F_I3C_TARG_STATUS_ACTSTATE                 ((uint32_t)(0x3UL << MXC_F_I3C_TARG_STATUS_ACTSTATE_POS)) /**< TARG_STATUS_ACTSTATE Mask */
#define MXC_V_I3C_TARG_STATUS_ACTSTATE_NORMAL          ((uint32_t)0x0UL) /**< TARG_STATUS_ACTSTATE_NORMAL Value */
#define MXC_S_I3C_TARG_STATUS_ACTSTATE_NORMAL          (MXC_V_I3C_TARG_STATUS_ACTSTATE_NORMAL << MXC_F_I3C_TARG_STATUS_ACTSTATE_POS) /**< TARG_STATUS_ACTSTATE_NORMAL Setting */
#define MXC_V_I3C_TARG_STATUS_ACTSTATE_1MS_LAT         ((uint32_t)0x1UL) /**< TARG_STATUS_ACTSTATE_1MS_LAT Value */
#define MXC_S_I3C_TARG_STATUS_ACTSTATE_1MS_LAT         (MXC_V_I3C_TARG_STATUS_ACTSTATE_1MS_LAT << MXC_F_I3C_TARG_STATUS_ACTSTATE_POS) /**< TARG_STATUS_ACTSTATE_1MS_LAT Setting */
#define MXC_V_I3C_TARG_STATUS_ACTSTATE_100MS_LAT       ((uint32_t)0x2UL) /**< TARG_STATUS_ACTSTATE_100MS_LAT Value */
#define MXC_S_I3C_TARG_STATUS_ACTSTATE_100MS_LAT       (MXC_V_I3C_TARG_STATUS_ACTSTATE_100MS_LAT << MXC_F_I3C_TARG_STATUS_ACTSTATE_POS) /**< TARG_STATUS_ACTSTATE_100MS_LAT Setting */
#define MXC_V_I3C_TARG_STATUS_ACTSTATE_10S_LAT         ((uint32_t)0x3UL) /**< TARG_STATUS_ACTSTATE_10S_LAT Value */
#define MXC_S_I3C_TARG_STATUS_ACTSTATE_10S_LAT         (MXC_V_I3C_TARG_STATUS_ACTSTATE_10S_LAT << MXC_F_I3C_TARG_STATUS_ACTSTATE_POS) /**< TARG_STATUS_ACTSTATE_10S_LAT Setting */

#define MXC_F_I3C_TARG_STATUS_TIMECTRL_POS             30 /**< TARG_STATUS_TIMECTRL Position */
#define MXC_F_I3C_TARG_STATUS_TIMECTRL                 ((uint32_t)(0x3UL << MXC_F_I3C_TARG_STATUS_TIMECTRL_POS)) /**< TARG_STATUS_TIMECTRL Mask */
#define MXC_V_I3C_TARG_STATUS_TIMECTRL_DIS             ((uint32_t)0x0UL) /**< TARG_STATUS_TIMECTRL_DIS Value */
#define MXC_S_I3C_TARG_STATUS_TIMECTRL_DIS             (MXC_V_I3C_TARG_STATUS_TIMECTRL_DIS << MXC_F_I3C_TARG_STATUS_TIMECTRL_POS) /**< TARG_STATUS_TIMECTRL_DIS Setting */
#define MXC_V_I3C_TARG_STATUS_TIMECTRL_SYNC            ((uint32_t)0x1UL) /**< TARG_STATUS_TIMECTRL_SYNC Value */
#define MXC_S_I3C_TARG_STATUS_TIMECTRL_SYNC            (MXC_V_I3C_TARG_STATUS_TIMECTRL_SYNC << MXC_F_I3C_TARG_STATUS_TIMECTRL_POS) /**< TARG_STATUS_TIMECTRL_SYNC Setting */
#define MXC_V_I3C_TARG_STATUS_TIMECTRL_ASYNC           ((uint32_t)0x2UL) /**< TARG_STATUS_TIMECTRL_ASYNC Value */
#define MXC_S_I3C_TARG_STATUS_TIMECTRL_ASYNC           (MXC_V_I3C_TARG_STATUS_TIMECTRL_ASYNC << MXC_F_I3C_TARG_STATUS_TIMECTRL_POS) /**< TARG_STATUS_TIMECTRL_ASYNC Setting */
#define MXC_V_I3C_TARG_STATUS_TIMECTRL_BOTH            ((uint32_t)0x3UL) /**< TARG_STATUS_TIMECTRL_BOTH Value */
#define MXC_S_I3C_TARG_STATUS_TIMECTRL_BOTH            (MXC_V_I3C_TARG_STATUS_TIMECTRL_BOTH << MXC_F_I3C_TARG_STATUS_TIMECTRL_POS) /**< TARG_STATUS_TIMECTRL_BOTH Setting */

/**@} end of group I3C_TARG_STATUS_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_TARG_CTRL1 I3C_TARG_CTRL1
 * @brief    Target Control 1 Register.
 * @{
 */
#define MXC_F_I3C_TARG_CTRL1_EVENT_POS                 0 /**< TARG_CTRL1_EVENT Position */
#define MXC_F_I3C_TARG_CTRL1_EVENT                     ((uint32_t)(0x3UL << MXC_F_I3C_TARG_CTRL1_EVENT_POS)) /**< TARG_CTRL1_EVENT Mask */
#define MXC_V_I3C_TARG_CTRL1_EVENT_NORMAL              ((uint32_t)0x0UL) /**< TARG_CTRL1_EVENT_NORMAL Value */
#define MXC_S_I3C_TARG_CTRL1_EVENT_NORMAL              (MXC_V_I3C_TARG_CTRL1_EVENT_NORMAL << MXC_F_I3C_TARG_CTRL1_EVENT_POS) /**< TARG_CTRL1_EVENT_NORMAL Setting */
#define MXC_V_I3C_TARG_CTRL1_EVENT_IBI                 ((uint32_t)0x1UL) /**< TARG_CTRL1_EVENT_IBI Value */
#define MXC_S_I3C_TARG_CTRL1_EVENT_IBI                 (MXC_V_I3C_TARG_CTRL1_EVENT_IBI << MXC_F_I3C_TARG_CTRL1_EVENT_POS) /**< TARG_CTRL1_EVENT_IBI Setting */
#define MXC_V_I3C_TARG_CTRL1_EVENT_CONTREQ             ((uint32_t)0x2UL) /**< TARG_CTRL1_EVENT_CONTREQ Value */
#define MXC_S_I3C_TARG_CTRL1_EVENT_CONTREQ             (MXC_V_I3C_TARG_CTRL1_EVENT_CONTREQ << MXC_F_I3C_TARG_CTRL1_EVENT_POS) /**< TARG_CTRL1_EVENT_CONTREQ Setting */
#define MXC_V_I3C_TARG_CTRL1_EVENT_HJ                  ((uint32_t)0x3UL) /**< TARG_CTRL1_EVENT_HJ Value */
#define MXC_S_I3C_TARG_CTRL1_EVENT_HJ                  (MXC_V_I3C_TARG_CTRL1_EVENT_HJ << MXC_F_I3C_TARG_CTRL1_EVENT_POS) /**< TARG_CTRL1_EVENT_HJ Setting */

#define MXC_F_I3C_TARG_CTRL1_EXTIBI_POS                3 /**< TARG_CTRL1_EXTIBI Position */
#define MXC_F_I3C_TARG_CTRL1_EXTIBI                    ((uint32_t)(0x1UL << MXC_F_I3C_TARG_CTRL1_EXTIBI_POS)) /**< TARG_CTRL1_EXTIBI Mask */

#define MXC_F_I3C_TARG_CTRL1_DYNADDR_IDX_POS           4 /**< TARG_CTRL1_DYNADDR_IDX Position */
#define MXC_F_I3C_TARG_CTRL1_DYNADDR_IDX               ((uint32_t)(0xFUL << MXC_F_I3C_TARG_CTRL1_DYNADDR_IDX_POS)) /**< TARG_CTRL1_DYNADDR_IDX Mask */

#define MXC_F_I3C_TARG_CTRL1_IBIDATA_POS               8 /**< TARG_CTRL1_IBIDATA Position */
#define MXC_F_I3C_TARG_CTRL1_IBIDATA                   ((uint32_t)(0xFFUL << MXC_F_I3C_TARG_CTRL1_IBIDATA_POS)) /**< TARG_CTRL1_IBIDATA Mask */

/**@} end of group I3C_TARG_CTRL1_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_TARG_INTEN I3C_TARG_INTEN
 * @brief    Target Interrupt Enable Register.
 * @{
 */
#define MXC_F_I3C_TARG_INTEN_START_POS                 8 /**< TARG_INTEN_START Position */
#define MXC_F_I3C_TARG_INTEN_START                     ((uint32_t)(0x1UL << MXC_F_I3C_TARG_INTEN_START_POS)) /**< TARG_INTEN_START Mask */

#define MXC_F_I3C_TARG_INTEN_ADDRMATCH_POS             9 /**< TARG_INTEN_ADDRMATCH Position */
#define MXC_F_I3C_TARG_INTEN_ADDRMATCH                 ((uint32_t)(0x1UL << MXC_F_I3C_TARG_INTEN_ADDRMATCH_POS)) /**< TARG_INTEN_ADDRMATCH Mask */

#define MXC_F_I3C_TARG_INTEN_STOP_POS                  10 /**< TARG_INTEN_STOP Position */
#define MXC_F_I3C_TARG_INTEN_STOP                      ((uint32_t)(0x1UL << MXC_F_I3C_TARG_INTEN_STOP_POS)) /**< TARG_INTEN_STOP Mask */

#define MXC_F_I3C_TARG_INTEN_RX_RDY_POS                11 /**< TARG_INTEN_RX_RDY Position */
#define MXC_F_I3C_TARG_INTEN_RX_RDY                    ((uint32_t)(0x1UL << MXC_F_I3C_TARG_INTEN_RX_RDY_POS)) /**< TARG_INTEN_RX_RDY Mask */

#define MXC_F_I3C_TARG_INTEN_TX_NFULL_POS              12 /**< TARG_INTEN_TX_NFULL Position */
#define MXC_F_I3C_TARG_INTEN_TX_NFULL                  ((uint32_t)(0x1UL << MXC_F_I3C_TARG_INTEN_TX_NFULL_POS)) /**< TARG_INTEN_TX_NFULL Mask */

#define MXC_F_I3C_TARG_INTEN_DYNADDR_CHG_POS           13 /**< TARG_INTEN_DYNADDR_CHG Position */
#define MXC_F_I3C_TARG_INTEN_DYNADDR_CHG               ((uint32_t)(0x1UL << MXC_F_I3C_TARG_INTEN_DYNADDR_CHG_POS)) /**< TARG_INTEN_DYNADDR_CHG Mask */

#define MXC_F_I3C_TARG_INTEN_CCC_POS                   14 /**< TARG_INTEN_CCC Position */
#define MXC_F_I3C_TARG_INTEN_CCC                       ((uint32_t)(0x1UL << MXC_F_I3C_TARG_INTEN_CCC_POS)) /**< TARG_INTEN_CCC Mask */

#define MXC_F_I3C_TARG_INTEN_ERRWARN_POS               15 /**< TARG_INTEN_ERRWARN Position */
#define MXC_F_I3C_TARG_INTEN_ERRWARN                   ((uint32_t)(0x1UL << MXC_F_I3C_TARG_INTEN_ERRWARN_POS)) /**< TARG_INTEN_ERRWARN Mask */

#define MXC_F_I3C_TARG_INTEN_CCCH_DONE_POS             17 /**< TARG_INTEN_CCCH_DONE Position */
#define MXC_F_I3C_TARG_INTEN_CCCH_DONE                 ((uint32_t)(0x1UL << MXC_F_I3C_TARG_INTEN_CCCH_DONE_POS)) /**< TARG_INTEN_CCCH_DONE Mask */

#define MXC_F_I3C_TARG_INTEN_EVENT_REQ_POS             18 /**< TARG_INTEN_EVENT_REQ Position */
#define MXC_F_I3C_TARG_INTEN_EVENT_REQ                 ((uint32_t)(0x1UL << MXC_F_I3C_TARG_INTEN_EVENT_REQ_POS)) /**< TARG_INTEN_EVENT_REQ Mask */

#define MXC_F_I3C_TARG_INTEN_TARG_RST_POS              19 /**< TARG_INTEN_TARG_RST Position */
#define MXC_F_I3C_TARG_INTEN_TARG_RST                  ((uint32_t)(0x1UL << MXC_F_I3C_TARG_INTEN_TARG_RST_POS)) /**< TARG_INTEN_TARG_RST Mask */

/**@} end of group I3C_TARG_INTEN_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_TARG_INTCLR I3C_TARG_INTCLR
 * @brief    Target Interrupt Clear Register.
 * @{
 */
#define MXC_F_I3C_TARG_INTCLR_START_POS                8 /**< TARG_INTCLR_START Position */
#define MXC_F_I3C_TARG_INTCLR_START                    ((uint32_t)(0x1UL << MXC_F_I3C_TARG_INTCLR_START_POS)) /**< TARG_INTCLR_START Mask */

#define MXC_F_I3C_TARG_INTCLR_ADDRMATCH_POS            9 /**< TARG_INTCLR_ADDRMATCH Position */
#define MXC_F_I3C_TARG_INTCLR_ADDRMATCH                ((uint32_t)(0x1UL << MXC_F_I3C_TARG_INTCLR_ADDRMATCH_POS)) /**< TARG_INTCLR_ADDRMATCH Mask */

#define MXC_F_I3C_TARG_INTCLR_STOP_POS                 10 /**< TARG_INTCLR_STOP Position */
#define MXC_F_I3C_TARG_INTCLR_STOP                     ((uint32_t)(0x1UL << MXC_F_I3C_TARG_INTCLR_STOP_POS)) /**< TARG_INTCLR_STOP Mask */

#define MXC_F_I3C_TARG_INTCLR_RX_RDY_POS               11 /**< TARG_INTCLR_RX_RDY Position */
#define MXC_F_I3C_TARG_INTCLR_RX_RDY                   ((uint32_t)(0x1UL << MXC_F_I3C_TARG_INTCLR_RX_RDY_POS)) /**< TARG_INTCLR_RX_RDY Mask */

#define MXC_F_I3C_TARG_INTCLR_TX_NFULL_POS             12 /**< TARG_INTCLR_TX_NFULL Position */
#define MXC_F_I3C_TARG_INTCLR_TX_NFULL                 ((uint32_t)(0x1UL << MXC_F_I3C_TARG_INTCLR_TX_NFULL_POS)) /**< TARG_INTCLR_TX_NFULL Mask */

#define MXC_F_I3C_TARG_INTCLR_DYNADDR_CHG_POS          13 /**< TARG_INTCLR_DYNADDR_CHG Position */
#define MXC_F_I3C_TARG_INTCLR_DYNADDR_CHG              ((uint32_t)(0x1UL << MXC_F_I3C_TARG_INTCLR_DYNADDR_CHG_POS)) /**< TARG_INTCLR_DYNADDR_CHG Mask */

#define MXC_F_I3C_TARG_INTCLR_CCC_POS                  14 /**< TARG_INTCLR_CCC Position */
#define MXC_F_I3C_TARG_INTCLR_CCC                      ((uint32_t)(0x1UL << MXC_F_I3C_TARG_INTCLR_CCC_POS)) /**< TARG_INTCLR_CCC Mask */

#define MXC_F_I3C_TARG_INTCLR_ERRWARN_POS              15 /**< TARG_INTCLR_ERRWARN Position */
#define MXC_F_I3C_TARG_INTCLR_ERRWARN                  ((uint32_t)(0x1UL << MXC_F_I3C_TARG_INTCLR_ERRWARN_POS)) /**< TARG_INTCLR_ERRWARN Mask */

#define MXC_F_I3C_TARG_INTCLR_CCCH_DONE_POS            17 /**< TARG_INTCLR_CCCH_DONE Position */
#define MXC_F_I3C_TARG_INTCLR_CCCH_DONE                ((uint32_t)(0x1UL << MXC_F_I3C_TARG_INTCLR_CCCH_DONE_POS)) /**< TARG_INTCLR_CCCH_DONE Mask */

#define MXC_F_I3C_TARG_INTCLR_EVENT_REQ_POS            18 /**< TARG_INTCLR_EVENT_REQ Position */
#define MXC_F_I3C_TARG_INTCLR_EVENT_REQ                ((uint32_t)(0x1UL << MXC_F_I3C_TARG_INTCLR_EVENT_REQ_POS)) /**< TARG_INTCLR_EVENT_REQ Mask */

#define MXC_F_I3C_TARG_INTCLR_TARG_RST_POS             19 /**< TARG_INTCLR_TARG_RST Position */
#define MXC_F_I3C_TARG_INTCLR_TARG_RST                 ((uint32_t)(0x1UL << MXC_F_I3C_TARG_INTCLR_TARG_RST_POS)) /**< TARG_INTCLR_TARG_RST Mask */

/**@} end of group I3C_TARG_INTCLR_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_TARG_INTFL I3C_TARG_INTFL
 * @brief    Target Interrupt Flag Register.
 * @{
 */
#define MXC_F_I3C_TARG_INTFL_START_POS                 8 /**< TARG_INTFL_START Position */
#define MXC_F_I3C_TARG_INTFL_START                     ((uint32_t)(0x1UL << MXC_F_I3C_TARG_INTFL_START_POS)) /**< TARG_INTFL_START Mask */

#define MXC_F_I3C_TARG_INTFL_ADDRMATCH_POS             9 /**< TARG_INTFL_ADDRMATCH Position */
#define MXC_F_I3C_TARG_INTFL_ADDRMATCH                 ((uint32_t)(0x1UL << MXC_F_I3C_TARG_INTFL_ADDRMATCH_POS)) /**< TARG_INTFL_ADDRMATCH Mask */

#define MXC_F_I3C_TARG_INTFL_STOP_POS                  10 /**< TARG_INTFL_STOP Position */
#define MXC_F_I3C_TARG_INTFL_STOP                      ((uint32_t)(0x1UL << MXC_F_I3C_TARG_INTFL_STOP_POS)) /**< TARG_INTFL_STOP Mask */

#define MXC_F_I3C_TARG_INTFL_RX_RDY_POS                11 /**< TARG_INTFL_RX_RDY Position */
#define MXC_F_I3C_TARG_INTFL_RX_RDY                    ((uint32_t)(0x1UL << MXC_F_I3C_TARG_INTFL_RX_RDY_POS)) /**< TARG_INTFL_RX_RDY Mask */

#define MXC_F_I3C_TARG_INTFL_TX_NFULL_POS              12 /**< TARG_INTFL_TX_NFULL Position */
#define MXC_F_I3C_TARG_INTFL_TX_NFULL                  ((uint32_t)(0x1UL << MXC_F_I3C_TARG_INTFL_TX_NFULL_POS)) /**< TARG_INTFL_TX_NFULL Mask */

#define MXC_F_I3C_TARG_INTFL_DYNADDR_CHG_POS           13 /**< TARG_INTFL_DYNADDR_CHG Position */
#define MXC_F_I3C_TARG_INTFL_DYNADDR_CHG               ((uint32_t)(0x1UL << MXC_F_I3C_TARG_INTFL_DYNADDR_CHG_POS)) /**< TARG_INTFL_DYNADDR_CHG Mask */

#define MXC_F_I3C_TARG_INTFL_CCC_POS                   14 /**< TARG_INTFL_CCC Position */
#define MXC_F_I3C_TARG_INTFL_CCC                       ((uint32_t)(0x1UL << MXC_F_I3C_TARG_INTFL_CCC_POS)) /**< TARG_INTFL_CCC Mask */

#define MXC_F_I3C_TARG_INTFL_ERRWARN_POS               15 /**< TARG_INTFL_ERRWARN Position */
#define MXC_F_I3C_TARG_INTFL_ERRWARN                   ((uint32_t)(0x1UL << MXC_F_I3C_TARG_INTFL_ERRWARN_POS)) /**< TARG_INTFL_ERRWARN Mask */

#define MXC_F_I3C_TARG_INTFL_CCCH_DONE_POS             17 /**< TARG_INTFL_CCCH_DONE Position */
#define MXC_F_I3C_TARG_INTFL_CCCH_DONE                 ((uint32_t)(0x1UL << MXC_F_I3C_TARG_INTFL_CCCH_DONE_POS)) /**< TARG_INTFL_CCCH_DONE Mask */

#define MXC_F_I3C_TARG_INTFL_EVENT_REQ_POS             18 /**< TARG_INTFL_EVENT_REQ Position */
#define MXC_F_I3C_TARG_INTFL_EVENT_REQ                 ((uint32_t)(0x1UL << MXC_F_I3C_TARG_INTFL_EVENT_REQ_POS)) /**< TARG_INTFL_EVENT_REQ Mask */

#define MXC_F_I3C_TARG_INTFL_TARG_RST_POS              19 /**< TARG_INTFL_TARG_RST Position */
#define MXC_F_I3C_TARG_INTFL_TARG_RST                  ((uint32_t)(0x1UL << MXC_F_I3C_TARG_INTFL_TARG_RST_POS)) /**< TARG_INTFL_TARG_RST Mask */

/**@} end of group I3C_TARG_INTFL_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_TARG_ERRWARN I3C_TARG_ERRWARN
 * @brief    Target Error and Warning Register.
 * @{
 */
#define MXC_F_I3C_TARG_ERRWARN_OVR_POS                 0 /**< TARG_ERRWARN_OVR Position */
#define MXC_F_I3C_TARG_ERRWARN_OVR                     ((uint32_t)(0x1UL << MXC_F_I3C_TARG_ERRWARN_OVR_POS)) /**< TARG_ERRWARN_OVR Mask */

#define MXC_F_I3C_TARG_ERRWARN_UNR_POS                 1 /**< TARG_ERRWARN_UNR Position */
#define MXC_F_I3C_TARG_ERRWARN_UNR                     ((uint32_t)(0x1UL << MXC_F_I3C_TARG_ERRWARN_UNR_POS)) /**< TARG_ERRWARN_UNR Mask */

#define MXC_F_I3C_TARG_ERRWARN_UNR_NACK_POS            2 /**< TARG_ERRWARN_UNR_NACK Position */
#define MXC_F_I3C_TARG_ERRWARN_UNR_NACK                ((uint32_t)(0x1UL << MXC_F_I3C_TARG_ERRWARN_UNR_NACK_POS)) /**< TARG_ERRWARN_UNR_NACK Mask */

#define MXC_F_I3C_TARG_ERRWARN_CONT_RX_TERM_POS        3 /**< TARG_ERRWARN_CONT_RX_TERM Position */
#define MXC_F_I3C_TARG_ERRWARN_CONT_RX_TERM            ((uint32_t)(0x1UL << MXC_F_I3C_TARG_ERRWARN_CONT_RX_TERM_POS)) /**< TARG_ERRWARN_CONT_RX_TERM Mask */

#define MXC_F_I3C_TARG_ERRWARN_INVSTART_POS            4 /**< TARG_ERRWARN_INVSTART Position */
#define MXC_F_I3C_TARG_ERRWARN_INVSTART                ((uint32_t)(0x1UL << MXC_F_I3C_TARG_ERRWARN_INVSTART_POS)) /**< TARG_ERRWARN_INVSTART Mask */

#define MXC_F_I3C_TARG_ERRWARN_SDR_PAR_POS             8 /**< TARG_ERRWARN_SDR_PAR Position */
#define MXC_F_I3C_TARG_ERRWARN_SDR_PAR                 ((uint32_t)(0x1UL << MXC_F_I3C_TARG_ERRWARN_SDR_PAR_POS)) /**< TARG_ERRWARN_SDR_PAR Mask */

#define MXC_F_I3C_TARG_ERRWARN_TO_POS                  11 /**< TARG_ERRWARN_TO Position */
#define MXC_F_I3C_TARG_ERRWARN_TO                      ((uint32_t)(0x1UL << MXC_F_I3C_TARG_ERRWARN_TO_POS)) /**< TARG_ERRWARN_TO Mask */

#define MXC_F_I3C_TARG_ERRWARN_RX_UNR_POS              16 /**< TARG_ERRWARN_RX_UNR Position */
#define MXC_F_I3C_TARG_ERRWARN_RX_UNR                  ((uint32_t)(0x1UL << MXC_F_I3C_TARG_ERRWARN_RX_UNR_POS)) /**< TARG_ERRWARN_RX_UNR Mask */

#define MXC_F_I3C_TARG_ERRWARN_TX_OVR_POS              17 /**< TARG_ERRWARN_TX_OVR Position */
#define MXC_F_I3C_TARG_ERRWARN_TX_OVR                  ((uint32_t)(0x1UL << MXC_F_I3C_TARG_ERRWARN_TX_OVR_POS)) /**< TARG_ERRWARN_TX_OVR Mask */

/**@} end of group I3C_TARG_ERRWARN_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_TARG_DMACTRL I3C_TARG_DMACTRL
 * @brief    Target DMA Control Register.
 * @{
 */
#define MXC_F_I3C_TARG_DMACTRL_RX_EN_POS               0 /**< TARG_DMACTRL_RX_EN Position */
#define MXC_F_I3C_TARG_DMACTRL_RX_EN                   ((uint32_t)(0x3UL << MXC_F_I3C_TARG_DMACTRL_RX_EN_POS)) /**< TARG_DMACTRL_RX_EN Mask */
#define MXC_V_I3C_TARG_DMACTRL_RX_EN_DIS               ((uint32_t)0x0UL) /**< TARG_DMACTRL_RX_EN_DIS Value */
#define MXC_S_I3C_TARG_DMACTRL_RX_EN_DIS               (MXC_V_I3C_TARG_DMACTRL_RX_EN_DIS << MXC_F_I3C_TARG_DMACTRL_RX_EN_POS) /**< TARG_DMACTRL_RX_EN_DIS Setting */
#define MXC_V_I3C_TARG_DMACTRL_RX_EN_ONE_FR            ((uint32_t)0x1UL) /**< TARG_DMACTRL_RX_EN_ONE_FR Value */
#define MXC_S_I3C_TARG_DMACTRL_RX_EN_ONE_FR            (MXC_V_I3C_TARG_DMACTRL_RX_EN_ONE_FR << MXC_F_I3C_TARG_DMACTRL_RX_EN_POS) /**< TARG_DMACTRL_RX_EN_ONE_FR Setting */
#define MXC_V_I3C_TARG_DMACTRL_RX_EN_EN                ((uint32_t)0x2UL) /**< TARG_DMACTRL_RX_EN_EN Value */
#define MXC_S_I3C_TARG_DMACTRL_RX_EN_EN                (MXC_V_I3C_TARG_DMACTRL_RX_EN_EN << MXC_F_I3C_TARG_DMACTRL_RX_EN_POS) /**< TARG_DMACTRL_RX_EN_EN Setting */

#define MXC_F_I3C_TARG_DMACTRL_TX_EN_POS               2 /**< TARG_DMACTRL_TX_EN Position */
#define MXC_F_I3C_TARG_DMACTRL_TX_EN                   ((uint32_t)(0x3UL << MXC_F_I3C_TARG_DMACTRL_TX_EN_POS)) /**< TARG_DMACTRL_TX_EN Mask */
#define MXC_V_I3C_TARG_DMACTRL_TX_EN_DIS               ((uint32_t)0x0UL) /**< TARG_DMACTRL_TX_EN_DIS Value */
#define MXC_S_I3C_TARG_DMACTRL_TX_EN_DIS               (MXC_V_I3C_TARG_DMACTRL_TX_EN_DIS << MXC_F_I3C_TARG_DMACTRL_TX_EN_POS) /**< TARG_DMACTRL_TX_EN_DIS Setting */
#define MXC_V_I3C_TARG_DMACTRL_TX_EN_ONE_FR            ((uint32_t)0x1UL) /**< TARG_DMACTRL_TX_EN_ONE_FR Value */
#define MXC_S_I3C_TARG_DMACTRL_TX_EN_ONE_FR            (MXC_V_I3C_TARG_DMACTRL_TX_EN_ONE_FR << MXC_F_I3C_TARG_DMACTRL_TX_EN_POS) /**< TARG_DMACTRL_TX_EN_ONE_FR Setting */
#define MXC_V_I3C_TARG_DMACTRL_TX_EN_EN                ((uint32_t)0x2UL) /**< TARG_DMACTRL_TX_EN_EN Value */
#define MXC_S_I3C_TARG_DMACTRL_TX_EN_EN                (MXC_V_I3C_TARG_DMACTRL_TX_EN_EN << MXC_F_I3C_TARG_DMACTRL_TX_EN_POS) /**< TARG_DMACTRL_TX_EN_EN Setting */

#define MXC_F_I3C_TARG_DMACTRL_WIDTH_POS               4 /**< TARG_DMACTRL_WIDTH Position */
#define MXC_F_I3C_TARG_DMACTRL_WIDTH                   ((uint32_t)(0x3UL << MXC_F_I3C_TARG_DMACTRL_WIDTH_POS)) /**< TARG_DMACTRL_WIDTH Mask */
#define MXC_V_I3C_TARG_DMACTRL_WIDTH_BYTE              ((uint32_t)0x0UL) /**< TARG_DMACTRL_WIDTH_BYTE Value */
#define MXC_S_I3C_TARG_DMACTRL_WIDTH_BYTE              (MXC_V_I3C_TARG_DMACTRL_WIDTH_BYTE << MXC_F_I3C_TARG_DMACTRL_WIDTH_POS) /**< TARG_DMACTRL_WIDTH_BYTE Setting */
#define MXC_V_I3C_TARG_DMACTRL_WIDTH_HALFWORD          ((uint32_t)0x2UL) /**< TARG_DMACTRL_WIDTH_HALFWORD Value */
#define MXC_S_I3C_TARG_DMACTRL_WIDTH_HALFWORD          (MXC_V_I3C_TARG_DMACTRL_WIDTH_HALFWORD << MXC_F_I3C_TARG_DMACTRL_WIDTH_POS) /**< TARG_DMACTRL_WIDTH_HALFWORD Setting */

/**@} end of group I3C_TARG_DMACTRL_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_TARG_FIFOCTRL I3C_TARG_FIFOCTRL
 * @brief    Target FIFO Control Register.
 * @{
 */
#define MXC_F_I3C_TARG_FIFOCTRL_TX_FLUSH_POS           0 /**< TARG_FIFOCTRL_TX_FLUSH Position */
#define MXC_F_I3C_TARG_FIFOCTRL_TX_FLUSH               ((uint32_t)(0x1UL << MXC_F_I3C_TARG_FIFOCTRL_TX_FLUSH_POS)) /**< TARG_FIFOCTRL_TX_FLUSH Mask */

#define MXC_F_I3C_TARG_FIFOCTRL_RX_FLUSH_POS           1 /**< TARG_FIFOCTRL_RX_FLUSH Position */
#define MXC_F_I3C_TARG_FIFOCTRL_RX_FLUSH               ((uint32_t)(0x1UL << MXC_F_I3C_TARG_FIFOCTRL_RX_FLUSH_POS)) /**< TARG_FIFOCTRL_RX_FLUSH Mask */

#define MXC_F_I3C_TARG_FIFOCTRL_UNLOCK_POS             3 /**< TARG_FIFOCTRL_UNLOCK Position */
#define MXC_F_I3C_TARG_FIFOCTRL_UNLOCK                 ((uint32_t)(0x1UL << MXC_F_I3C_TARG_FIFOCTRL_UNLOCK_POS)) /**< TARG_FIFOCTRL_UNLOCK Mask */

#define MXC_F_I3C_TARG_FIFOCTRL_TX_THD_LVL_POS         4 /**< TARG_FIFOCTRL_TX_THD_LVL Position */
#define MXC_F_I3C_TARG_FIFOCTRL_TX_THD_LVL             ((uint32_t)(0x3UL << MXC_F_I3C_TARG_FIFOCTRL_TX_THD_LVL_POS)) /**< TARG_FIFOCTRL_TX_THD_LVL Mask */
#define MXC_V_I3C_TARG_FIFOCTRL_TX_THD_LVL_EMPTY       ((uint32_t)0x0UL) /**< TARG_FIFOCTRL_TX_THD_LVL_EMPTY Value */
#define MXC_S_I3C_TARG_FIFOCTRL_TX_THD_LVL_EMPTY       (MXC_V_I3C_TARG_FIFOCTRL_TX_THD_LVL_EMPTY << MXC_F_I3C_TARG_FIFOCTRL_TX_THD_LVL_POS) /**< TARG_FIFOCTRL_TX_THD_LVL_EMPTY Setting */
#define MXC_V_I3C_TARG_FIFOCTRL_TX_THD_LVL_QUARTER_FULL ((uint32_t)0x1UL) /**< TARG_FIFOCTRL_TX_THD_LVL_QUARTER_FULL Value */
#define MXC_S_I3C_TARG_FIFOCTRL_TX_THD_LVL_QUARTER_FULL (MXC_V_I3C_TARG_FIFOCTRL_TX_THD_LVL_QUARTER_FULL << MXC_F_I3C_TARG_FIFOCTRL_TX_THD_LVL_POS) /**< TARG_FIFOCTRL_TX_THD_LVL_QUARTER_FULL Setting */
#define MXC_V_I3C_TARG_FIFOCTRL_TX_THD_LVL_HALF_FULL   ((uint32_t)0x2UL) /**< TARG_FIFOCTRL_TX_THD_LVL_HALF_FULL Value */
#define MXC_S_I3C_TARG_FIFOCTRL_TX_THD_LVL_HALF_FULL   (MXC_V_I3C_TARG_FIFOCTRL_TX_THD_LVL_HALF_FULL << MXC_F_I3C_TARG_FIFOCTRL_TX_THD_LVL_POS) /**< TARG_FIFOCTRL_TX_THD_LVL_HALF_FULL Setting */
#define MXC_V_I3C_TARG_FIFOCTRL_TX_THD_LVL_ALMOST_FULL ((uint32_t)0x3UL) /**< TARG_FIFOCTRL_TX_THD_LVL_ALMOST_FULL Value */
#define MXC_S_I3C_TARG_FIFOCTRL_TX_THD_LVL_ALMOST_FULL (MXC_V_I3C_TARG_FIFOCTRL_TX_THD_LVL_ALMOST_FULL << MXC_F_I3C_TARG_FIFOCTRL_TX_THD_LVL_POS) /**< TARG_FIFOCTRL_TX_THD_LVL_ALMOST_FULL Setting */

#define MXC_F_I3C_TARG_FIFOCTRL_RX_THD_LVL_POS         6 /**< TARG_FIFOCTRL_RX_THD_LVL Position */
#define MXC_F_I3C_TARG_FIFOCTRL_RX_THD_LVL             ((uint32_t)(0x3UL << MXC_F_I3C_TARG_FIFOCTRL_RX_THD_LVL_POS)) /**< TARG_FIFOCTRL_RX_THD_LVL Mask */
#define MXC_V_I3C_TARG_FIFOCTRL_RX_THD_LVL_NOT_EMPTY   ((uint32_t)0x0UL) /**< TARG_FIFOCTRL_RX_THD_LVL_NOT_EMPTY Value */
#define MXC_S_I3C_TARG_FIFOCTRL_RX_THD_LVL_NOT_EMPTY   (MXC_V_I3C_TARG_FIFOCTRL_RX_THD_LVL_NOT_EMPTY << MXC_F_I3C_TARG_FIFOCTRL_RX_THD_LVL_POS) /**< TARG_FIFOCTRL_RX_THD_LVL_NOT_EMPTY Setting */
#define MXC_V_I3C_TARG_FIFOCTRL_RX_THD_LVL_QUARTER_FULL ((uint32_t)0x1UL) /**< TARG_FIFOCTRL_RX_THD_LVL_QUARTER_FULL Value */
#define MXC_S_I3C_TARG_FIFOCTRL_RX_THD_LVL_QUARTER_FULL (MXC_V_I3C_TARG_FIFOCTRL_RX_THD_LVL_QUARTER_FULL << MXC_F_I3C_TARG_FIFOCTRL_RX_THD_LVL_POS) /**< TARG_FIFOCTRL_RX_THD_LVL_QUARTER_FULL Setting */
#define MXC_V_I3C_TARG_FIFOCTRL_RX_THD_LVL_HALF_FULL   ((uint32_t)0x2UL) /**< TARG_FIFOCTRL_RX_THD_LVL_HALF_FULL Value */
#define MXC_S_I3C_TARG_FIFOCTRL_RX_THD_LVL_HALF_FULL   (MXC_V_I3C_TARG_FIFOCTRL_RX_THD_LVL_HALF_FULL << MXC_F_I3C_TARG_FIFOCTRL_RX_THD_LVL_POS) /**< TARG_FIFOCTRL_RX_THD_LVL_HALF_FULL Setting */
#define MXC_V_I3C_TARG_FIFOCTRL_RX_THD_LVL_3_QUARTER_FULL ((uint32_t)0x3UL) /**< TARG_FIFOCTRL_RX_THD_LVL_3_QUARTER_FULL Value */
#define MXC_S_I3C_TARG_FIFOCTRL_RX_THD_LVL_3_QUARTER_FULL (MXC_V_I3C_TARG_FIFOCTRL_RX_THD_LVL_3_QUARTER_FULL << MXC_F_I3C_TARG_FIFOCTRL_RX_THD_LVL_POS) /**< TARG_FIFOCTRL_RX_THD_LVL_3_QUARTER_FULL Setting */

#define MXC_F_I3C_TARG_FIFOCTRL_TX_LVL_POS             16 /**< TARG_FIFOCTRL_TX_LVL Position */
#define MXC_F_I3C_TARG_FIFOCTRL_TX_LVL                 ((uint32_t)(0x3FUL << MXC_F_I3C_TARG_FIFOCTRL_TX_LVL_POS)) /**< TARG_FIFOCTRL_TX_LVL Mask */

#define MXC_F_I3C_TARG_FIFOCTRL_RX_LVL_POS             24 /**< TARG_FIFOCTRL_RX_LVL Position */
#define MXC_F_I3C_TARG_FIFOCTRL_RX_LVL                 ((uint32_t)(0x3FUL << MXC_F_I3C_TARG_FIFOCTRL_RX_LVL_POS)) /**< TARG_FIFOCTRL_RX_LVL Mask */

#define MXC_F_I3C_TARG_FIFOCTRL_TX_FULL_POS            30 /**< TARG_FIFOCTRL_TX_FULL Position */
#define MXC_F_I3C_TARG_FIFOCTRL_TX_FULL                ((uint32_t)(0x1UL << MXC_F_I3C_TARG_FIFOCTRL_TX_FULL_POS)) /**< TARG_FIFOCTRL_TX_FULL Mask */

#define MXC_F_I3C_TARG_FIFOCTRL_RX_EM_POS              31 /**< TARG_FIFOCTRL_RX_EM Position */
#define MXC_F_I3C_TARG_FIFOCTRL_RX_EM                  ((uint32_t)(0x1UL << MXC_F_I3C_TARG_FIFOCTRL_RX_EM_POS)) /**< TARG_FIFOCTRL_RX_EM Mask */

/**@} end of group I3C_TARG_FIFOCTRL_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_TARG_TXFIFO8 I3C_TARG_TXFIFO8
 * @brief    Target Write Byte Data Register.
 * @{
 */
#define MXC_F_I3C_TARG_TXFIFO8_DATA_POS                0 /**< TARG_TXFIFO8_DATA Position */
#define MXC_F_I3C_TARG_TXFIFO8_DATA                    ((uint32_t)(0xFFUL << MXC_F_I3C_TARG_TXFIFO8_DATA_POS)) /**< TARG_TXFIFO8_DATA Mask */

#define MXC_F_I3C_TARG_TXFIFO8_END_POS                 8 /**< TARG_TXFIFO8_END Position */
#define MXC_F_I3C_TARG_TXFIFO8_END                     ((uint32_t)(0x1UL << MXC_F_I3C_TARG_TXFIFO8_END_POS)) /**< TARG_TXFIFO8_END Mask */

#define MXC_F_I3C_TARG_TXFIFO8_END2_POS                16 /**< TARG_TXFIFO8_END2 Position */
#define MXC_F_I3C_TARG_TXFIFO8_END2                    ((uint32_t)(0x1UL << MXC_F_I3C_TARG_TXFIFO8_END2_POS)) /**< TARG_TXFIFO8_END2 Mask */

/**@} end of group I3C_TARG_TXFIFO8_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_TARG_TXFIFO8E I3C_TARG_TXFIFO8E
 * @brief    Target Write Byte Data as End Register.
 * @{
 */
#define MXC_F_I3C_TARG_TXFIFO8E_DATA_POS               0 /**< TARG_TXFIFO8E_DATA Position */
#define MXC_F_I3C_TARG_TXFIFO8E_DATA                   ((uint32_t)(0xFFUL << MXC_F_I3C_TARG_TXFIFO8E_DATA_POS)) /**< TARG_TXFIFO8E_DATA Mask */

/**@} end of group I3C_TARG_TXFIFO8E_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_TARG_TXFIFO16 I3C_TARG_TXFIFO16
 * @brief    Target Write Half-Word Data Register.
 * @{
 */
#define MXC_F_I3C_TARG_TXFIFO16_DATA_POS               0 /**< TARG_TXFIFO16_DATA Position */
#define MXC_F_I3C_TARG_TXFIFO16_DATA                   ((uint32_t)(0xFFFFUL << MXC_F_I3C_TARG_TXFIFO16_DATA_POS)) /**< TARG_TXFIFO16_DATA Mask */

#define MXC_F_I3C_TARG_TXFIFO16_END_POS                16 /**< TARG_TXFIFO16_END Position */
#define MXC_F_I3C_TARG_TXFIFO16_END                    ((uint32_t)(0x1UL << MXC_F_I3C_TARG_TXFIFO16_END_POS)) /**< TARG_TXFIFO16_END Mask */

/**@} end of group I3C_TARG_TXFIFO16_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_TARG_TXFIFO16E I3C_TARG_TXFIFO16E
 * @brief    Target Write Half-Word Data as End Register.
 * @{
 */
#define MXC_F_I3C_TARG_TXFIFO16E_DATA_POS              0 /**< TARG_TXFIFO16E_DATA Position */
#define MXC_F_I3C_TARG_TXFIFO16E_DATA                  ((uint32_t)(0xFFFFUL << MXC_F_I3C_TARG_TXFIFO16E_DATA_POS)) /**< TARG_TXFIFO16E_DATA Mask */

/**@} end of group I3C_TARG_TXFIFO16E_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_TARG_RXFIFO8 I3C_TARG_RXFIFO8
 * @brief    Target Read Byte Data Register.
 * @{
 */
#define MXC_F_I3C_TARG_RXFIFO8_DATA_POS                0 /**< TARG_RXFIFO8_DATA Position */
#define MXC_F_I3C_TARG_RXFIFO8_DATA                    ((uint32_t)(0xFFUL << MXC_F_I3C_TARG_RXFIFO8_DATA_POS)) /**< TARG_RXFIFO8_DATA Mask */

/**@} end of group I3C_TARG_RXFIFO8_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_TARG_RXFIFO16 I3C_TARG_RXFIFO16
 * @brief    Target Read Half-Word Data Register.
 * @{
 */
#define MXC_F_I3C_TARG_RXFIFO16_DATA_POS               0 /**< TARG_RXFIFO16_DATA Position */
#define MXC_F_I3C_TARG_RXFIFO16_DATA                   ((uint32_t)(0xFFFFUL << MXC_F_I3C_TARG_RXFIFO16_DATA_POS)) /**< TARG_RXFIFO16_DATA Mask */

/**@} end of group I3C_TARG_RXFIFO16_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_TARG_TXFIFO8O I3C_TARG_TXFIFO8O
 * @brief    Target Byte-Only Write Byte Data Register.
 * @{
 */
#define MXC_F_I3C_TARG_TXFIFO8O_DATA_POS               0 /**< TARG_TXFIFO8O_DATA Position */
#define MXC_F_I3C_TARG_TXFIFO8O_DATA                   ((uint32_t)(0xFFUL << MXC_F_I3C_TARG_TXFIFO8O_DATA_POS)) /**< TARG_TXFIFO8O_DATA Mask */

/**@} end of group I3C_TARG_TXFIFO8O_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_TARG_CAP0 I3C_TARG_CAP0
 * @brief    Target Capabilities 0 Register.
 * @{
 */
#define MXC_F_I3C_TARG_CAP0_MAPCNT_POS                 0 /**< TARG_CAP0_MAPCNT Position */
#define MXC_F_I3C_TARG_CAP0_MAPCNT                     ((uint32_t)(0xFUL << MXC_F_I3C_TARG_CAP0_MAPCNT_POS)) /**< TARG_CAP0_MAPCNT Mask */

#define MXC_F_I3C_TARG_CAP0_I2C_10BADDR_POS            4 /**< TARG_CAP0_I2C_10BADDR Position */
#define MXC_F_I3C_TARG_CAP0_I2C_10BADDR                ((uint32_t)(0x1UL << MXC_F_I3C_TARG_CAP0_I2C_10BADDR_POS)) /**< TARG_CAP0_I2C_10BADDR Mask */

#define MXC_F_I3C_TARG_CAP0_I2C_SWRST_POS              5 /**< TARG_CAP0_I2C_SWRST Position */
#define MXC_F_I3C_TARG_CAP0_I2C_SWRST                  ((uint32_t)(0x1UL << MXC_F_I3C_TARG_CAP0_I2C_SWRST_POS)) /**< TARG_CAP0_I2C_SWRST Mask */

#define MXC_F_I3C_TARG_CAP0_I2C_DEVID_POS              6 /**< TARG_CAP0_I2C_DEVID Position */
#define MXC_F_I3C_TARG_CAP0_I2C_DEVID                  ((uint32_t)(0x1UL << MXC_F_I3C_TARG_CAP0_I2C_DEVID_POS)) /**< TARG_CAP0_I2C_DEVID Mask */

#define MXC_F_I3C_TARG_CAP0_FIFO32_REG_POS             7 /**< TARG_CAP0_FIFO32_REG Position */
#define MXC_F_I3C_TARG_CAP0_FIFO32_REG                 ((uint32_t)(0x1UL << MXC_F_I3C_TARG_CAP0_FIFO32_REG_POS)) /**< TARG_CAP0_FIFO32_REG Mask */

#define MXC_F_I3C_TARG_CAP0_EXTIBI_POS                 8 /**< TARG_CAP0_EXTIBI Position */
#define MXC_F_I3C_TARG_CAP0_EXTIBI                     ((uint32_t)(0x1UL << MXC_F_I3C_TARG_CAP0_EXTIBI_POS)) /**< TARG_CAP0_EXTIBI Mask */

#define MXC_F_I3C_TARG_CAP0_EXTIBI_REG_POS             9 /**< TARG_CAP0_EXTIBI_REG Position */
#define MXC_F_I3C_TARG_CAP0_EXTIBI_REG                 ((uint32_t)(0x1UL << MXC_F_I3C_TARG_CAP0_EXTIBI_REG_POS)) /**< TARG_CAP0_EXTIBI_REG Mask */

#define MXC_F_I3C_TARG_CAP0_HDRBT_LANES_POS            12 /**< TARG_CAP0_HDRBT_LANES Position */
#define MXC_F_I3C_TARG_CAP0_HDRBT_LANES                ((uint32_t)(0x3UL << MXC_F_I3C_TARG_CAP0_HDRBT_LANES_POS)) /**< TARG_CAP0_HDRBT_LANES Mask */

#define MXC_F_I3C_TARG_CAP0_CCC_V1_1_POS               16 /**< TARG_CAP0_CCC_V1_1 Position */
#define MXC_F_I3C_TARG_CAP0_CCC_V1_1                   ((uint32_t)(0x1UL << MXC_F_I3C_TARG_CAP0_CCC_V1_1_POS)) /**< TARG_CAP0_CCC_V1_1 Mask */

#define MXC_F_I3C_TARG_CAP0_TARG_RST_POS               17 /**< TARG_CAP0_TARG_RST Position */
#define MXC_F_I3C_TARG_CAP0_TARG_RST                   ((uint32_t)(0x1UL << MXC_F_I3C_TARG_CAP0_TARG_RST_POS)) /**< TARG_CAP0_TARG_RST Mask */

#define MXC_F_I3C_TARG_CAP0_GROUPADDR_POS              18 /**< TARG_CAP0_GROUPADDR Position */
#define MXC_F_I3C_TARG_CAP0_GROUPADDR                  ((uint32_t)(0x3UL << MXC_F_I3C_TARG_CAP0_GROUPADDR_POS)) /**< TARG_CAP0_GROUPADDR Mask */

#define MXC_F_I3C_TARG_CAP0_AASA_CCC_POS               21 /**< TARG_CAP0_AASA_CCC Position */
#define MXC_F_I3C_TARG_CAP0_AASA_CCC                   ((uint32_t)(0x1UL << MXC_F_I3C_TARG_CAP0_AASA_CCC_POS)) /**< TARG_CAP0_AASA_CCC Mask */

#define MXC_F_I3C_TARG_CAP0_T2T_SUBSC_POS              22 /**< TARG_CAP0_T2T_SUBSC Position */
#define MXC_F_I3C_TARG_CAP0_T2T_SUBSC                  ((uint32_t)(0x1UL << MXC_F_I3C_TARG_CAP0_T2T_SUBSC_POS)) /**< TARG_CAP0_T2T_SUBSC Mask */

#define MXC_F_I3C_TARG_CAP0_T2T_WR_POS                 23 /**< TARG_CAP0_T2T_WR Position */
#define MXC_F_I3C_TARG_CAP0_T2T_WR                     ((uint32_t)(0x1UL << MXC_F_I3C_TARG_CAP0_T2T_WR_POS)) /**< TARG_CAP0_T2T_WR Mask */

/**@} end of group I3C_TARG_CAP0_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_TARG_CAP1 I3C_TARG_CAP1
 * @brief    TARG_Capabilities 1 Register.
 * @{
 */
#define MXC_F_I3C_TARG_CAP1_PROVID_POS                 0 /**< TARG_CAP1_PROVID Position */
#define MXC_F_I3C_TARG_CAP1_PROVID                     ((uint32_t)(0x3UL << MXC_F_I3C_TARG_CAP1_PROVID_POS)) /**< TARG_CAP1_PROVID Mask */

#define MXC_F_I3C_TARG_CAP1_PROVID_REG_POS             2 /**< TARG_CAP1_PROVID_REG Position */
#define MXC_F_I3C_TARG_CAP1_PROVID_REG                 ((uint32_t)(0xFUL << MXC_F_I3C_TARG_CAP1_PROVID_REG_POS)) /**< TARG_CAP1_PROVID_REG Mask */

#define MXC_F_I3C_TARG_CAP1_HDR_MODES_POS              6 /**< TARG_CAP1_HDR_MODES Position */
#define MXC_F_I3C_TARG_CAP1_HDR_MODES                  ((uint32_t)(0x7UL << MXC_F_I3C_TARG_CAP1_HDR_MODES_POS)) /**< TARG_CAP1_HDR_MODES Mask */

#define MXC_F_I3C_TARG_CAP1_CONT_POS                   9 /**< TARG_CAP1_CONT Position */
#define MXC_F_I3C_TARG_CAP1_CONT                       ((uint32_t)(0x1UL << MXC_F_I3C_TARG_CAP1_CONT_POS)) /**< TARG_CAP1_CONT Mask */

#define MXC_F_I3C_TARG_CAP1_STATADDR_POS               10 /**< TARG_CAP1_STATADDR Position */
#define MXC_F_I3C_TARG_CAP1_STATADDR                   ((uint32_t)(0x3UL << MXC_F_I3C_TARG_CAP1_STATADDR_POS)) /**< TARG_CAP1_STATADDR Mask */

#define MXC_F_I3C_TARG_CAP1_CCCH_POS                   12 /**< TARG_CAP1_CCCH Position */
#define MXC_F_I3C_TARG_CAP1_CCCH                       ((uint32_t)(0xFUL << MXC_F_I3C_TARG_CAP1_CCCH_POS)) /**< TARG_CAP1_CCCH Mask */
#define MXC_V_I3C_TARG_CAP1_CCCH_BASIC                 ((uint32_t)0x1UL) /**< TARG_CAP1_CCCH_BASIC Value */
#define MXC_S_I3C_TARG_CAP1_CCCH_BASIC                 (MXC_V_I3C_TARG_CAP1_CCCH_BASIC << MXC_F_I3C_TARG_CAP1_CCCH_POS) /**< TARG_CAP1_CCCH_BASIC Setting */
#define MXC_V_I3C_TARG_CAP1_CCCH_LIMITS                ((uint32_t)0x2UL) /**< TARG_CAP1_CCCH_LIMITS Value */
#define MXC_S_I3C_TARG_CAP1_CCCH_LIMITS                (MXC_V_I3C_TARG_CAP1_CCCH_LIMITS << MXC_F_I3C_TARG_CAP1_CCCH_POS) /**< TARG_CAP1_CCCH_LIMITS Setting */
#define MXC_V_I3C_TARG_CAP1_CCCH_INTACT                ((uint32_t)0x4UL) /**< TARG_CAP1_CCCH_INTACT Value */
#define MXC_S_I3C_TARG_CAP1_CCCH_INTACT                (MXC_V_I3C_TARG_CAP1_CCCH_INTACT << MXC_F_I3C_TARG_CAP1_CCCH_POS) /**< TARG_CAP1_CCCH_INTACT Setting */
#define MXC_V_I3C_TARG_CAP1_CCCH_VENDOR                ((uint32_t)0x8UL) /**< TARG_CAP1_CCCH_VENDOR Value */
#define MXC_S_I3C_TARG_CAP1_CCCH_VENDOR                (MXC_V_I3C_TARG_CAP1_CCCH_VENDOR << MXC_F_I3C_TARG_CAP1_CCCH_POS) /**< TARG_CAP1_CCCH_VENDOR Setting */

#define MXC_F_I3C_TARG_CAP1_IBI_EVENTS_POS             16 /**< TARG_CAP1_IBI_EVENTS Position */
#define MXC_F_I3C_TARG_CAP1_IBI_EVENTS                 ((uint32_t)(0x1FUL << MXC_F_I3C_TARG_CAP1_IBI_EVENTS_POS)) /**< TARG_CAP1_IBI_EVENTS Mask */
#define MXC_V_I3C_TARG_CAP1_IBI_EVENTS_IBI             ((uint32_t)0x1UL) /**< TARG_CAP1_IBI_EVENTS_IBI Value */
#define MXC_S_I3C_TARG_CAP1_IBI_EVENTS_IBI             (MXC_V_I3C_TARG_CAP1_IBI_EVENTS_IBI << MXC_F_I3C_TARG_CAP1_IBI_EVENTS_POS) /**< TARG_CAP1_IBI_EVENTS_IBI Setting */
#define MXC_V_I3C_TARG_CAP1_IBI_EVENTS_PAYLOAD         ((uint32_t)0x2UL) /**< TARG_CAP1_IBI_EVENTS_PAYLOAD Value */
#define MXC_S_I3C_TARG_CAP1_IBI_EVENTS_PAYLOAD         (MXC_V_I3C_TARG_CAP1_IBI_EVENTS_PAYLOAD << MXC_F_I3C_TARG_CAP1_IBI_EVENTS_POS) /**< TARG_CAP1_IBI_EVENTS_PAYLOAD Setting */
#define MXC_V_I3C_TARG_CAP1_IBI_EVENTS_CONTREQ         ((uint32_t)0x4UL) /**< TARG_CAP1_IBI_EVENTS_CONTREQ Value */
#define MXC_S_I3C_TARG_CAP1_IBI_EVENTS_CONTREQ         (MXC_V_I3C_TARG_CAP1_IBI_EVENTS_CONTREQ << MXC_F_I3C_TARG_CAP1_IBI_EVENTS_POS) /**< TARG_CAP1_IBI_EVENTS_CONTREQ Setting */
#define MXC_V_I3C_TARG_CAP1_IBI_EVENTS_HJ              ((uint32_t)0x8UL) /**< TARG_CAP1_IBI_EVENTS_HJ Value */
#define MXC_S_I3C_TARG_CAP1_IBI_EVENTS_HJ              (MXC_V_I3C_TARG_CAP1_IBI_EVENTS_HJ << MXC_F_I3C_TARG_CAP1_IBI_EVENTS_POS) /**< TARG_CAP1_IBI_EVENTS_HJ Setting */
#define MXC_V_I3C_TARG_CAP1_IBI_EVENTS_BAMATCH         ((uint32_t)0x10UL) /**< TARG_CAP1_IBI_EVENTS_BAMATCH Value */
#define MXC_S_I3C_TARG_CAP1_IBI_EVENTS_BAMATCH         (MXC_V_I3C_TARG_CAP1_IBI_EVENTS_BAMATCH << MXC_F_I3C_TARG_CAP1_IBI_EVENTS_POS) /**< TARG_CAP1_IBI_EVENTS_BAMATCH Setting */

#define MXC_F_I3C_TARG_CAP1_TIMECTRL_POS               21 /**< TARG_CAP1_TIMECTRL Position */
#define MXC_F_I3C_TARG_CAP1_TIMECTRL                   ((uint32_t)(0x1UL << MXC_F_I3C_TARG_CAP1_TIMECTRL_POS)) /**< TARG_CAP1_TIMECTRL Mask */

#define MXC_F_I3C_TARG_CAP1_EXTFIFO_POS                23 /**< TARG_CAP1_EXTFIFO Position */
#define MXC_F_I3C_TARG_CAP1_EXTFIFO                    ((uint32_t)(0x7UL << MXC_F_I3C_TARG_CAP1_EXTFIFO_POS)) /**< TARG_CAP1_EXTFIFO Mask */

#define MXC_F_I3C_TARG_CAP1_TXFIFO_CFG_POS             26 /**< TARG_CAP1_TXFIFO_CFG Position */
#define MXC_F_I3C_TARG_CAP1_TXFIFO_CFG                 ((uint32_t)(0x3UL << MXC_F_I3C_TARG_CAP1_TXFIFO_CFG_POS)) /**< TARG_CAP1_TXFIFO_CFG Mask */

#define MXC_F_I3C_TARG_CAP1_RXFIFO_CFG_POS             28 /**< TARG_CAP1_RXFIFO_CFG Position */
#define MXC_F_I3C_TARG_CAP1_RXFIFO_CFG                 ((uint32_t)(0x3UL << MXC_F_I3C_TARG_CAP1_RXFIFO_CFG_POS)) /**< TARG_CAP1_RXFIFO_CFG Mask */

#define MXC_F_I3C_TARG_CAP1_INTR_POS                   30 /**< TARG_CAP1_INTR Position */
#define MXC_F_I3C_TARG_CAP1_INTR                       ((uint32_t)(0x1UL << MXC_F_I3C_TARG_CAP1_INTR_POS)) /**< TARG_CAP1_INTR Mask */

#define MXC_F_I3C_TARG_CAP1_DMA_POS                    31 /**< TARG_CAP1_DMA Position */
#define MXC_F_I3C_TARG_CAP1_DMA                        ((uint32_t)(0x1UL << MXC_F_I3C_TARG_CAP1_DMA_POS)) /**< TARG_CAP1_DMA Mask */

/**@} end of group I3C_TARG_CAP1_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_TARG_DYNADDR I3C_TARG_DYNADDR
 * @brief    Target Dynamic Address Register.
 * @{
 */
#define MXC_F_I3C_TARG_DYNADDR_VALID_POS               0 /**< TARG_DYNADDR_VALID Position */
#define MXC_F_I3C_TARG_DYNADDR_VALID                   ((uint32_t)(0x1UL << MXC_F_I3C_TARG_DYNADDR_VALID_POS)) /**< TARG_DYNADDR_VALID Mask */

#define MXC_F_I3C_TARG_DYNADDR_ADDR_POS                1 /**< TARG_DYNADDR_ADDR Position */
#define MXC_F_I3C_TARG_DYNADDR_ADDR                    ((uint32_t)(0x7FUL << MXC_F_I3C_TARG_DYNADDR_ADDR_POS)) /**< TARG_DYNADDR_ADDR Mask */

#define MXC_F_I3C_TARG_DYNADDR_CAUSE_POS               8 /**< TARG_DYNADDR_CAUSE Position */
#define MXC_F_I3C_TARG_DYNADDR_CAUSE                   ((uint32_t)(0x7UL << MXC_F_I3C_TARG_DYNADDR_CAUSE_POS)) /**< TARG_DYNADDR_CAUSE Mask */

/**@} end of group I3C_TARG_DYNADDR_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_TARG_MAXLIMITS I3C_TARG_MAXLIMITS
 * @brief    Maximum Limits Register.
 * @{
 */
#define MXC_F_I3C_TARG_MAXLIMITS_RX_POS                0 /**< TARG_MAXLIMITS_RX Position */
#define MXC_F_I3C_TARG_MAXLIMITS_RX                    ((uint32_t)(0xFFFUL << MXC_F_I3C_TARG_MAXLIMITS_RX_POS)) /**< TARG_MAXLIMITS_RX Mask */

#define MXC_F_I3C_TARG_MAXLIMITS_TX_POS                16 /**< TARG_MAXLIMITS_TX Position */
#define MXC_F_I3C_TARG_MAXLIMITS_TX                    ((uint32_t)(0xFFFUL << MXC_F_I3C_TARG_MAXLIMITS_TX_POS)) /**< TARG_MAXLIMITS_TX Mask */

/**@} end of group I3C_TARG_MAXLIMITS_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_TARG_IDEXT I3C_TARG_IDEXT
 * @brief    ID Extension Register.
 * @{
 */
#define MXC_F_I3C_TARG_IDEXT_DEVCHAR_POS               8 /**< TARG_IDEXT_DEVCHAR Position */
#define MXC_F_I3C_TARG_IDEXT_DEVCHAR                   ((uint32_t)(0xFFUL << MXC_F_I3C_TARG_IDEXT_DEVCHAR_POS)) /**< TARG_IDEXT_DEVCHAR Mask */

#define MXC_F_I3C_TARG_IDEXT_BUSCHAR_POS               16 /**< TARG_IDEXT_BUSCHAR Position */
#define MXC_F_I3C_TARG_IDEXT_BUSCHAR                   ((uint32_t)(0xFFUL << MXC_F_I3C_TARG_IDEXT_BUSCHAR_POS)) /**< TARG_IDEXT_BUSCHAR Mask */

/**@} end of group I3C_TARG_IDEXT_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_TARG_MSGLAST I3C_TARG_MSGLAST
 * @brief    Target Matching Address Index Register.
 * @{
 */
#define MXC_F_I3C_TARG_MSGLAST_IDX_POS                 0 /**< TARG_MSGLAST_IDX Position */
#define MXC_F_I3C_TARG_MSGLAST_IDX                     ((uint32_t)(0xFUL << MXC_F_I3C_TARG_MSGLAST_IDX_POS)) /**< TARG_MSGLAST_IDX Mask */

#define MXC_F_I3C_TARG_MSGLAST_STATADDR_POS            4 /**< TARG_MSGLAST_STATADDR Position */
#define MXC_F_I3C_TARG_MSGLAST_STATADDR                ((uint32_t)(0x1UL << MXC_F_I3C_TARG_MSGLAST_STATADDR_POS)) /**< TARG_MSGLAST_STATADDR Mask */

#define MXC_F_I3C_TARG_MSGLAST_GROUP_POS               5 /**< TARG_MSGLAST_GROUP Position */
#define MXC_F_I3C_TARG_MSGLAST_GROUP                   ((uint32_t)(0x1UL << MXC_F_I3C_TARG_MSGLAST_GROUP_POS)) /**< TARG_MSGLAST_GROUP Mask */

#define MXC_F_I3C_TARG_MSGLAST_MODE_POS                6 /**< TARG_MSGLAST_MODE Position */
#define MXC_F_I3C_TARG_MSGLAST_MODE                    ((uint32_t)(0x3UL << MXC_F_I3C_TARG_MSGLAST_MODE_POS)) /**< TARG_MSGLAST_MODE Mask */
#define MXC_V_I3C_TARG_MSGLAST_MODE_DYN_STAT_ADDR      ((uint32_t)0x0UL) /**< TARG_MSGLAST_MODE_DYN_STAT_ADDR Value */
#define MXC_S_I3C_TARG_MSGLAST_MODE_DYN_STAT_ADDR      (MXC_V_I3C_TARG_MSGLAST_MODE_DYN_STAT_ADDR << MXC_F_I3C_TARG_MSGLAST_MODE_POS) /**< TARG_MSGLAST_MODE_DYN_STAT_ADDR Setting */
#define MXC_V_I3C_TARG_MSGLAST_MODE_HDR_DDR            ((uint32_t)0x1UL) /**< TARG_MSGLAST_MODE_HDR_DDR Value */
#define MXC_S_I3C_TARG_MSGLAST_MODE_HDR_DDR            (MXC_V_I3C_TARG_MSGLAST_MODE_HDR_DDR << MXC_F_I3C_TARG_MSGLAST_MODE_POS) /**< TARG_MSGLAST_MODE_HDR_DDR Setting */
#define MXC_V_I3C_TARG_MSGLAST_MODE_HDR_BT             ((uint32_t)0x2UL) /**< TARG_MSGLAST_MODE_HDR_BT Value */
#define MXC_S_I3C_TARG_MSGLAST_MODE_HDR_BT             (MXC_V_I3C_TARG_MSGLAST_MODE_HDR_BT << MXC_F_I3C_TARG_MSGLAST_MODE_POS) /**< TARG_MSGLAST_MODE_HDR_BT Setting */

#define MXC_F_I3C_TARG_MSGLAST_PREV_IDX_POS            8 /**< TARG_MSGLAST_PREV_IDX Position */
#define MXC_F_I3C_TARG_MSGLAST_PREV_IDX                ((uint32_t)(0xFUL << MXC_F_I3C_TARG_MSGLAST_PREV_IDX_POS)) /**< TARG_MSGLAST_PREV_IDX Mask */

#define MXC_F_I3C_TARG_MSGLAST_PREV_GROUP_POS          13 /**< TARG_MSGLAST_PREV_GROUP Position */
#define MXC_F_I3C_TARG_MSGLAST_PREV_GROUP              ((uint32_t)(0x1UL << MXC_F_I3C_TARG_MSGLAST_PREV_GROUP_POS)) /**< TARG_MSGLAST_PREV_GROUP Mask */

#define MXC_F_I3C_TARG_MSGLAST_PREV_MODE_POS           14 /**< TARG_MSGLAST_PREV_MODE Position */
#define MXC_F_I3C_TARG_MSGLAST_PREV_MODE               ((uint32_t)(0x3UL << MXC_F_I3C_TARG_MSGLAST_PREV_MODE_POS)) /**< TARG_MSGLAST_PREV_MODE Mask */
#define MXC_V_I3C_TARG_MSGLAST_PREV_MODE_DYN_STAT_ADDR ((uint32_t)0x0UL) /**< TARG_MSGLAST_PREV_MODE_DYN_STAT_ADDR Value */
#define MXC_S_I3C_TARG_MSGLAST_PREV_MODE_DYN_STAT_ADDR (MXC_V_I3C_TARG_MSGLAST_PREV_MODE_DYN_STAT_ADDR << MXC_F_I3C_TARG_MSGLAST_PREV_MODE_POS) /**< TARG_MSGLAST_PREV_MODE_DYN_STAT_ADDR Setting */
#define MXC_V_I3C_TARG_MSGLAST_PREV_MODE_HDR_DDR       ((uint32_t)0x1UL) /**< TARG_MSGLAST_PREV_MODE_HDR_DDR Value */
#define MXC_S_I3C_TARG_MSGLAST_PREV_MODE_HDR_DDR       (MXC_V_I3C_TARG_MSGLAST_PREV_MODE_HDR_DDR << MXC_F_I3C_TARG_MSGLAST_PREV_MODE_POS) /**< TARG_MSGLAST_PREV_MODE_HDR_DDR Setting */
#define MXC_V_I3C_TARG_MSGLAST_PREV_MODE_HDR_BT        ((uint32_t)0x2UL) /**< TARG_MSGLAST_PREV_MODE_HDR_BT Value */
#define MXC_S_I3C_TARG_MSGLAST_PREV_MODE_HDR_BT        (MXC_V_I3C_TARG_MSGLAST_PREV_MODE_HDR_BT << MXC_F_I3C_TARG_MSGLAST_PREV_MODE_POS) /**< TARG_MSGLAST_PREV_MODE_HDR_BT Setting */

#define MXC_F_I3C_TARG_MSGLAST_SECPREV_IDX_POS         16 /**< TARG_MSGLAST_SECPREV_IDX Position */
#define MXC_F_I3C_TARG_MSGLAST_SECPREV_IDX             ((uint32_t)(0xFUL << MXC_F_I3C_TARG_MSGLAST_SECPREV_IDX_POS)) /**< TARG_MSGLAST_SECPREV_IDX Mask */

#define MXC_F_I3C_TARG_MSGLAST_SECPREV_GROUP_POS       21 /**< TARG_MSGLAST_SECPREV_GROUP Position */
#define MXC_F_I3C_TARG_MSGLAST_SECPREV_GROUP           ((uint32_t)(0x1UL << MXC_F_I3C_TARG_MSGLAST_SECPREV_GROUP_POS)) /**< TARG_MSGLAST_SECPREV_GROUP Mask */

#define MXC_F_I3C_TARG_MSGLAST_SECPREV_MODE_POS        22 /**< TARG_MSGLAST_SECPREV_MODE Position */
#define MXC_F_I3C_TARG_MSGLAST_SECPREV_MODE            ((uint32_t)(0x3UL << MXC_F_I3C_TARG_MSGLAST_SECPREV_MODE_POS)) /**< TARG_MSGLAST_SECPREV_MODE Mask */
#define MXC_V_I3C_TARG_MSGLAST_SECPREV_MODE_DYN_STAT_ADDR ((uint32_t)0x0UL) /**< TARG_MSGLAST_SECPREV_MODE_DYN_STAT_ADDR Value */
#define MXC_S_I3C_TARG_MSGLAST_SECPREV_MODE_DYN_STAT_ADDR (MXC_V_I3C_TARG_MSGLAST_SECPREV_MODE_DYN_STAT_ADDR << MXC_F_I3C_TARG_MSGLAST_SECPREV_MODE_POS) /**< TARG_MSGLAST_SECPREV_MODE_DYN_STAT_ADDR Setting */
#define MXC_V_I3C_TARG_MSGLAST_SECPREV_MODE_HDR_DDR    ((uint32_t)0x1UL) /**< TARG_MSGLAST_SECPREV_MODE_HDR_DDR Value */
#define MXC_S_I3C_TARG_MSGLAST_SECPREV_MODE_HDR_DDR    (MXC_V_I3C_TARG_MSGLAST_SECPREV_MODE_HDR_DDR << MXC_F_I3C_TARG_MSGLAST_SECPREV_MODE_POS) /**< TARG_MSGLAST_SECPREV_MODE_HDR_DDR Setting */
#define MXC_V_I3C_TARG_MSGLAST_SECPREV_MODE_HDR_BT     ((uint32_t)0x2UL) /**< TARG_MSGLAST_SECPREV_MODE_HDR_BT Value */
#define MXC_S_I3C_TARG_MSGLAST_SECPREV_MODE_HDR_BT     (MXC_V_I3C_TARG_MSGLAST_SECPREV_MODE_HDR_BT << MXC_F_I3C_TARG_MSGLAST_SECPREV_MODE_POS) /**< TARG_MSGLAST_SECPREV_MODE_HDR_BT Setting */

/**@} end of group I3C_TARG_MSGLAST_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_CONT_CTRL1 I3C_CONT_CTRL1
 * @brief    Controller Control 1 Register.
 * @{
 */
#define MXC_F_I3C_CONT_CTRL1_REQ_POS                   0 /**< CONT_CTRL1_REQ Position */
#define MXC_F_I3C_CONT_CTRL1_REQ                       ((uint32_t)(0x7UL << MXC_F_I3C_CONT_CTRL1_REQ_POS)) /**< CONT_CTRL1_REQ Mask */
#define MXC_V_I3C_CONT_CTRL1_REQ_NONE                  ((uint32_t)0x0UL) /**< CONT_CTRL1_REQ_NONE Value */
#define MXC_S_I3C_CONT_CTRL1_REQ_NONE                  (MXC_V_I3C_CONT_CTRL1_REQ_NONE << MXC_F_I3C_CONT_CTRL1_REQ_POS) /**< CONT_CTRL1_REQ_NONE Setting */
#define MXC_V_I3C_CONT_CTRL1_REQ_EMIT_START            ((uint32_t)0x1UL) /**< CONT_CTRL1_REQ_EMIT_START Value */
#define MXC_S_I3C_CONT_CTRL1_REQ_EMIT_START            (MXC_V_I3C_CONT_CTRL1_REQ_EMIT_START << MXC_F_I3C_CONT_CTRL1_REQ_POS) /**< CONT_CTRL1_REQ_EMIT_START Setting */
#define MXC_V_I3C_CONT_CTRL1_REQ_EMIT_STOP             ((uint32_t)0x2UL) /**< CONT_CTRL1_REQ_EMIT_STOP Value */
#define MXC_S_I3C_CONT_CTRL1_REQ_EMIT_STOP             (MXC_V_I3C_CONT_CTRL1_REQ_EMIT_STOP << MXC_F_I3C_CONT_CTRL1_REQ_POS) /**< CONT_CTRL1_REQ_EMIT_STOP Setting */
#define MXC_V_I3C_CONT_CTRL1_REQ_IBI_ACKNACK           ((uint32_t)0x3UL) /**< CONT_CTRL1_REQ_IBI_ACKNACK Value */
#define MXC_S_I3C_CONT_CTRL1_REQ_IBI_ACKNACK           (MXC_V_I3C_CONT_CTRL1_REQ_IBI_ACKNACK << MXC_F_I3C_CONT_CTRL1_REQ_POS) /**< CONT_CTRL1_REQ_IBI_ACKNACK Setting */
#define MXC_V_I3C_CONT_CTRL1_REQ_PROCESS_DAA           ((uint32_t)0x4UL) /**< CONT_CTRL1_REQ_PROCESS_DAA Value */
#define MXC_S_I3C_CONT_CTRL1_REQ_PROCESS_DAA           (MXC_V_I3C_CONT_CTRL1_REQ_PROCESS_DAA << MXC_F_I3C_CONT_CTRL1_REQ_POS) /**< CONT_CTRL1_REQ_PROCESS_DAA Setting */
#define MXC_V_I3C_CONT_CTRL1_REQ_EXIT_RST              ((uint32_t)0x6UL) /**< CONT_CTRL1_REQ_EXIT_RST Value */
#define MXC_S_I3C_CONT_CTRL1_REQ_EXIT_RST              (MXC_V_I3C_CONT_CTRL1_REQ_EXIT_RST << MXC_F_I3C_CONT_CTRL1_REQ_POS) /**< CONT_CTRL1_REQ_EXIT_RST Setting */
#define MXC_V_I3C_CONT_CTRL1_REQ_AUTO_IBI              ((uint32_t)0x7UL) /**< CONT_CTRL1_REQ_AUTO_IBI Value */
#define MXC_S_I3C_CONT_CTRL1_REQ_AUTO_IBI              (MXC_V_I3C_CONT_CTRL1_REQ_AUTO_IBI << MXC_F_I3C_CONT_CTRL1_REQ_POS) /**< CONT_CTRL1_REQ_AUTO_IBI Setting */

#define MXC_F_I3C_CONT_CTRL1_TYPE_POS                  4 /**< CONT_CTRL1_TYPE Position */
#define MXC_F_I3C_CONT_CTRL1_TYPE                      ((uint32_t)(0x3UL << MXC_F_I3C_CONT_CTRL1_TYPE_POS)) /**< CONT_CTRL1_TYPE Mask */

#define MXC_F_I3C_CONT_CTRL1_IBIRESP_POS               6 /**< CONT_CTRL1_IBIRESP Position */
#define MXC_F_I3C_CONT_CTRL1_IBIRESP                   ((uint32_t)(0x3UL << MXC_F_I3C_CONT_CTRL1_IBIRESP_POS)) /**< CONT_CTRL1_IBIRESP Mask */

#define MXC_F_I3C_CONT_CTRL1_RDWR_DIR_POS              8 /**< CONT_CTRL1_RDWR_DIR Position */
#define MXC_F_I3C_CONT_CTRL1_RDWR_DIR                  ((uint32_t)(0x1UL << MXC_F_I3C_CONT_CTRL1_RDWR_DIR_POS)) /**< CONT_CTRL1_RDWR_DIR Mask */

#define MXC_F_I3C_CONT_CTRL1_ADDR_POS                  9 /**< CONT_CTRL1_ADDR Position */
#define MXC_F_I3C_CONT_CTRL1_ADDR                      ((uint32_t)(0x7FUL << MXC_F_I3C_CONT_CTRL1_ADDR_POS)) /**< CONT_CTRL1_ADDR Mask */

#define MXC_F_I3C_CONT_CTRL1_TERM_RD_POS               16 /**< CONT_CTRL1_TERM_RD Position */
#define MXC_F_I3C_CONT_CTRL1_TERM_RD                   ((uint32_t)(0xFFUL << MXC_F_I3C_CONT_CTRL1_TERM_RD_POS)) /**< CONT_CTRL1_TERM_RD Mask */

/**@} end of group I3C_CONT_CTRL1_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_CONT_STATUS I3C_CONT_STATUS
 * @brief    Controller Status Register.
 * @{
 */
#define MXC_F_I3C_CONT_STATUS_STATE_POS                0 /**< CONT_STATUS_STATE Position */
#define MXC_F_I3C_CONT_STATUS_STATE                    ((uint32_t)(0x7UL << MXC_F_I3C_CONT_STATUS_STATE_POS)) /**< CONT_STATUS_STATE Mask */
#define MXC_V_I3C_CONT_STATUS_STATE_IDLE               ((uint32_t)0x0UL) /**< CONT_STATUS_STATE_IDLE Value */
#define MXC_S_I3C_CONT_STATUS_STATE_IDLE               (MXC_V_I3C_CONT_STATUS_STATE_IDLE << MXC_F_I3C_CONT_STATUS_STATE_POS) /**< CONT_STATUS_STATE_IDLE Setting */
#define MXC_V_I3C_CONT_STATUS_STATE_TARG_REQ           ((uint32_t)0x1UL) /**< CONT_STATUS_STATE_TARG_REQ Value */
#define MXC_S_I3C_CONT_STATUS_STATE_TARG_REQ           (MXC_V_I3C_CONT_STATUS_STATE_TARG_REQ << MXC_F_I3C_CONT_STATUS_STATE_POS) /**< CONT_STATUS_STATE_TARG_REQ Setting */
#define MXC_V_I3C_CONT_STATUS_STATE_SDR_TXSDRMSG       ((uint32_t)0x2UL) /**< CONT_STATUS_STATE_SDR_TXSDRMSG Value */
#define MXC_S_I3C_CONT_STATUS_STATE_SDR_TXSDRMSG       (MXC_V_I3C_CONT_STATUS_STATE_SDR_TXSDRMSG << MXC_F_I3C_CONT_STATUS_STATE_POS) /**< CONT_STATUS_STATE_SDR_TXSDRMSG Setting */
#define MXC_V_I3C_CONT_STATUS_STATE_SDR_NORM           ((uint32_t)0x3UL) /**< CONT_STATUS_STATE_SDR_NORM Value */
#define MXC_S_I3C_CONT_STATUS_STATE_SDR_NORM           (MXC_V_I3C_CONT_STATUS_STATE_SDR_NORM << MXC_F_I3C_CONT_STATUS_STATE_POS) /**< CONT_STATUS_STATE_SDR_NORM Setting */
#define MXC_V_I3C_CONT_STATUS_STATE_DDR                ((uint32_t)0x4UL) /**< CONT_STATUS_STATE_DDR Value */
#define MXC_S_I3C_CONT_STATUS_STATE_DDR                (MXC_V_I3C_CONT_STATUS_STATE_DDR << MXC_F_I3C_CONT_STATUS_STATE_POS) /**< CONT_STATUS_STATE_DDR Setting */
#define MXC_V_I3C_CONT_STATUS_STATE_DAA                ((uint32_t)0x5UL) /**< CONT_STATUS_STATE_DAA Value */
#define MXC_S_I3C_CONT_STATUS_STATE_DAA                (MXC_V_I3C_CONT_STATUS_STATE_DAA << MXC_F_I3C_CONT_STATUS_STATE_POS) /**< CONT_STATUS_STATE_DAA Setting */
#define MXC_V_I3C_CONT_STATUS_STATE_IBI_ACKNACK        ((uint32_t)0x6UL) /**< CONT_STATUS_STATE_IBI_ACKNACK Value */
#define MXC_S_I3C_CONT_STATUS_STATE_IBI_ACKNACK        (MXC_V_I3C_CONT_STATUS_STATE_IBI_ACKNACK << MXC_F_I3C_CONT_STATUS_STATE_POS) /**< CONT_STATUS_STATE_IBI_ACKNACK Setting */
#define MXC_V_I3C_CONT_STATUS_STATE_IBI_RX             ((uint32_t)0x7UL) /**< CONT_STATUS_STATE_IBI_RX Value */
#define MXC_S_I3C_CONT_STATUS_STATE_IBI_RX             (MXC_V_I3C_CONT_STATUS_STATE_IBI_RX << MXC_F_I3C_CONT_STATUS_STATE_POS) /**< CONT_STATUS_STATE_IBI_RX Setting */

#define MXC_F_I3C_CONT_STATUS_WAIT_POS                 4 /**< CONT_STATUS_WAIT Position */
#define MXC_F_I3C_CONT_STATUS_WAIT                     ((uint32_t)(0x1UL << MXC_F_I3C_CONT_STATUS_WAIT_POS)) /**< CONT_STATUS_WAIT Mask */

#define MXC_F_I3C_CONT_STATUS_NACK_POS                 5 /**< CONT_STATUS_NACK Position */
#define MXC_F_I3C_CONT_STATUS_NACK                     ((uint32_t)(0x1UL << MXC_F_I3C_CONT_STATUS_NACK_POS)) /**< CONT_STATUS_NACK Mask */

#define MXC_F_I3C_CONT_STATUS_IBITYPE_POS              6 /**< CONT_STATUS_IBITYPE Position */
#define MXC_F_I3C_CONT_STATUS_IBITYPE                  ((uint32_t)(0x3UL << MXC_F_I3C_CONT_STATUS_IBITYPE_POS)) /**< CONT_STATUS_IBITYPE Mask */
#define MXC_V_I3C_CONT_STATUS_IBITYPE_NONE             ((uint32_t)0x0UL) /**< CONT_STATUS_IBITYPE_NONE Value */
#define MXC_S_I3C_CONT_STATUS_IBITYPE_NONE             (MXC_V_I3C_CONT_STATUS_IBITYPE_NONE << MXC_F_I3C_CONT_STATUS_IBITYPE_POS) /**< CONT_STATUS_IBITYPE_NONE Setting */
#define MXC_V_I3C_CONT_STATUS_IBITYPE_IBI              ((uint32_t)0x1UL) /**< CONT_STATUS_IBITYPE_IBI Value */
#define MXC_S_I3C_CONT_STATUS_IBITYPE_IBI              (MXC_V_I3C_CONT_STATUS_IBITYPE_IBI << MXC_F_I3C_CONT_STATUS_IBITYPE_POS) /**< CONT_STATUS_IBITYPE_IBI Setting */
#define MXC_V_I3C_CONT_STATUS_IBITYPE_CONT_REQ         ((uint32_t)0x2UL) /**< CONT_STATUS_IBITYPE_CONT_REQ Value */
#define MXC_S_I3C_CONT_STATUS_IBITYPE_CONT_REQ         (MXC_V_I3C_CONT_STATUS_IBITYPE_CONT_REQ << MXC_F_I3C_CONT_STATUS_IBITYPE_POS) /**< CONT_STATUS_IBITYPE_CONT_REQ Setting */
#define MXC_V_I3C_CONT_STATUS_IBITYPE_HOTJOIN_REQ      ((uint32_t)0x3UL) /**< CONT_STATUS_IBITYPE_HOTJOIN_REQ Value */
#define MXC_S_I3C_CONT_STATUS_IBITYPE_HOTJOIN_REQ      (MXC_V_I3C_CONT_STATUS_IBITYPE_HOTJOIN_REQ << MXC_F_I3C_CONT_STATUS_IBITYPE_POS) /**< CONT_STATUS_IBITYPE_HOTJOIN_REQ Setting */

#define MXC_F_I3C_CONT_STATUS_TARG_START_POS           8 /**< CONT_STATUS_TARG_START Position */
#define MXC_F_I3C_CONT_STATUS_TARG_START               ((uint32_t)(0x1UL << MXC_F_I3C_CONT_STATUS_TARG_START_POS)) /**< CONT_STATUS_TARG_START Mask */

#define MXC_F_I3C_CONT_STATUS_REQ_DONE_POS             9 /**< CONT_STATUS_REQ_DONE Position */
#define MXC_F_I3C_CONT_STATUS_REQ_DONE                 ((uint32_t)(0x1UL << MXC_F_I3C_CONT_STATUS_REQ_DONE_POS)) /**< CONT_STATUS_REQ_DONE Mask */

#define MXC_F_I3C_CONT_STATUS_DONE_POS                 10 /**< CONT_STATUS_DONE Position */
#define MXC_F_I3C_CONT_STATUS_DONE                     ((uint32_t)(0x1UL << MXC_F_I3C_CONT_STATUS_DONE_POS)) /**< CONT_STATUS_DONE Mask */

#define MXC_F_I3C_CONT_STATUS_RX_RDY_POS               11 /**< CONT_STATUS_RX_RDY Position */
#define MXC_F_I3C_CONT_STATUS_RX_RDY                   ((uint32_t)(0x1UL << MXC_F_I3C_CONT_STATUS_RX_RDY_POS)) /**< CONT_STATUS_RX_RDY Mask */

#define MXC_F_I3C_CONT_STATUS_TX_NFULL_POS             12 /**< CONT_STATUS_TX_NFULL Position */
#define MXC_F_I3C_CONT_STATUS_TX_NFULL                 ((uint32_t)(0x1UL << MXC_F_I3C_CONT_STATUS_TX_NFULL_POS)) /**< CONT_STATUS_TX_NFULL Mask */

#define MXC_F_I3C_CONT_STATUS_IBI_WON_POS              13 /**< CONT_STATUS_IBI_WON Position */
#define MXC_F_I3C_CONT_STATUS_IBI_WON                  ((uint32_t)(0x1UL << MXC_F_I3C_CONT_STATUS_IBI_WON_POS)) /**< CONT_STATUS_IBI_WON Mask */

#define MXC_F_I3C_CONT_STATUS_ERRWARN_POS              15 /**< CONT_STATUS_ERRWARN Position */
#define MXC_F_I3C_CONT_STATUS_ERRWARN                  ((uint32_t)(0x1UL << MXC_F_I3C_CONT_STATUS_ERRWARN_POS)) /**< CONT_STATUS_ERRWARN Mask */

#define MXC_F_I3C_CONT_STATUS_CONT_TRANS_POS           19 /**< CONT_STATUS_CONT_TRANS Position */
#define MXC_F_I3C_CONT_STATUS_CONT_TRANS               ((uint32_t)(0x1UL << MXC_F_I3C_CONT_STATUS_CONT_TRANS_POS)) /**< CONT_STATUS_CONT_TRANS Mask */

#define MXC_F_I3C_CONT_STATUS_IBI_ADDR_POS             24 /**< CONT_STATUS_IBI_ADDR Position */
#define MXC_F_I3C_CONT_STATUS_IBI_ADDR                 ((uint32_t)(0x7FUL << MXC_F_I3C_CONT_STATUS_IBI_ADDR_POS)) /**< CONT_STATUS_IBI_ADDR Mask */

/**@} end of group I3C_CONT_STATUS_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_CONT_IBIRULES I3C_CONT_IBIRULES
 * @brief    Controller IBI Registry and Rules Register.
 * @{
 */
#define MXC_F_I3C_CONT_IBIRULES_ADDR0_POS              0 /**< CONT_IBIRULES_ADDR0 Position */
#define MXC_F_I3C_CONT_IBIRULES_ADDR0                  ((uint32_t)(0x3FUL << MXC_F_I3C_CONT_IBIRULES_ADDR0_POS)) /**< CONT_IBIRULES_ADDR0 Mask */

#define MXC_F_I3C_CONT_IBIRULES_ADDR1_POS              6 /**< CONT_IBIRULES_ADDR1 Position */
#define MXC_F_I3C_CONT_IBIRULES_ADDR1                  ((uint32_t)(0x3FUL << MXC_F_I3C_CONT_IBIRULES_ADDR1_POS)) /**< CONT_IBIRULES_ADDR1 Mask */

#define MXC_F_I3C_CONT_IBIRULES_ADDR2_POS              12 /**< CONT_IBIRULES_ADDR2 Position */
#define MXC_F_I3C_CONT_IBIRULES_ADDR2                  ((uint32_t)(0x3FUL << MXC_F_I3C_CONT_IBIRULES_ADDR2_POS)) /**< CONT_IBIRULES_ADDR2 Mask */

#define MXC_F_I3C_CONT_IBIRULES_ADDR3_POS              18 /**< CONT_IBIRULES_ADDR3 Position */
#define MXC_F_I3C_CONT_IBIRULES_ADDR3                  ((uint32_t)(0x3FUL << MXC_F_I3C_CONT_IBIRULES_ADDR3_POS)) /**< CONT_IBIRULES_ADDR3 Mask */

#define MXC_F_I3C_CONT_IBIRULES_ADDR4_POS              24 /**< CONT_IBIRULES_ADDR4 Position */
#define MXC_F_I3C_CONT_IBIRULES_ADDR4                  ((uint32_t)(0x3FUL << MXC_F_I3C_CONT_IBIRULES_ADDR4_POS)) /**< CONT_IBIRULES_ADDR4 Mask */

#define MXC_F_I3C_CONT_IBIRULES_MSB0_POS               30 /**< CONT_IBIRULES_MSB0 Position */
#define MXC_F_I3C_CONT_IBIRULES_MSB0                   ((uint32_t)(0x1UL << MXC_F_I3C_CONT_IBIRULES_MSB0_POS)) /**< CONT_IBIRULES_MSB0 Mask */

#define MXC_F_I3C_CONT_IBIRULES_NOBYTE_POS             31 /**< CONT_IBIRULES_NOBYTE Position */
#define MXC_F_I3C_CONT_IBIRULES_NOBYTE                 ((uint32_t)(0x1UL << MXC_F_I3C_CONT_IBIRULES_NOBYTE_POS)) /**< CONT_IBIRULES_NOBYTE Mask */

/**@} end of group I3C_CONT_IBIRULES_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_CONT_INTEN I3C_CONT_INTEN
 * @brief    Controller Interrupt Enable Register.
 * @{
 */
#define MXC_F_I3C_CONT_INTEN_TARG_START_POS            8 /**< CONT_INTEN_TARG_START Position */
#define MXC_F_I3C_CONT_INTEN_TARG_START                ((uint32_t)(0x1UL << MXC_F_I3C_CONT_INTEN_TARG_START_POS)) /**< CONT_INTEN_TARG_START Mask */

#define MXC_F_I3C_CONT_INTEN_REQ_DONE_POS              9 /**< CONT_INTEN_REQ_DONE Position */
#define MXC_F_I3C_CONT_INTEN_REQ_DONE                  ((uint32_t)(0x1UL << MXC_F_I3C_CONT_INTEN_REQ_DONE_POS)) /**< CONT_INTEN_REQ_DONE Mask */

#define MXC_F_I3C_CONT_INTEN_DONE_POS                  10 /**< CONT_INTEN_DONE Position */
#define MXC_F_I3C_CONT_INTEN_DONE                      ((uint32_t)(0x1UL << MXC_F_I3C_CONT_INTEN_DONE_POS)) /**< CONT_INTEN_DONE Mask */

#define MXC_F_I3C_CONT_INTEN_RX_RDY_POS                11 /**< CONT_INTEN_RX_RDY Position */
#define MXC_F_I3C_CONT_INTEN_RX_RDY                    ((uint32_t)(0x1UL << MXC_F_I3C_CONT_INTEN_RX_RDY_POS)) /**< CONT_INTEN_RX_RDY Mask */

#define MXC_F_I3C_CONT_INTEN_TX_NFULL_POS              12 /**< CONT_INTEN_TX_NFULL Position */
#define MXC_F_I3C_CONT_INTEN_TX_NFULL                  ((uint32_t)(0x1UL << MXC_F_I3C_CONT_INTEN_TX_NFULL_POS)) /**< CONT_INTEN_TX_NFULL Mask */

#define MXC_F_I3C_CONT_INTEN_IBI_WON_POS               13 /**< CONT_INTEN_IBI_WON Position */
#define MXC_F_I3C_CONT_INTEN_IBI_WON                   ((uint32_t)(0x1UL << MXC_F_I3C_CONT_INTEN_IBI_WON_POS)) /**< CONT_INTEN_IBI_WON Mask */

#define MXC_F_I3C_CONT_INTEN_ERRWARN_POS               15 /**< CONT_INTEN_ERRWARN Position */
#define MXC_F_I3C_CONT_INTEN_ERRWARN                   ((uint32_t)(0x1UL << MXC_F_I3C_CONT_INTEN_ERRWARN_POS)) /**< CONT_INTEN_ERRWARN Mask */

#define MXC_F_I3C_CONT_INTEN_NOW_CONT_POS              19 /**< CONT_INTEN_NOW_CONT Position */
#define MXC_F_I3C_CONT_INTEN_NOW_CONT                  ((uint32_t)(0x1UL << MXC_F_I3C_CONT_INTEN_NOW_CONT_POS)) /**< CONT_INTEN_NOW_CONT Mask */

/**@} end of group I3C_CONT_INTEN_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_CONT_INTCLR I3C_CONT_INTCLR
 * @brief    Controller Interrupt Clear Register.
 * @{
 */
#define MXC_F_I3C_CONT_INTCLR_TARG_START_POS           8 /**< CONT_INTCLR_TARG_START Position */
#define MXC_F_I3C_CONT_INTCLR_TARG_START               ((uint32_t)(0x1UL << MXC_F_I3C_CONT_INTCLR_TARG_START_POS)) /**< CONT_INTCLR_TARG_START Mask */

#define MXC_F_I3C_CONT_INTCLR_REQ_DONE_POS             9 /**< CONT_INTCLR_REQ_DONE Position */
#define MXC_F_I3C_CONT_INTCLR_REQ_DONE                 ((uint32_t)(0x1UL << MXC_F_I3C_CONT_INTCLR_REQ_DONE_POS)) /**< CONT_INTCLR_REQ_DONE Mask */

#define MXC_F_I3C_CONT_INTCLR_DONE_POS                 10 /**< CONT_INTCLR_DONE Position */
#define MXC_F_I3C_CONT_INTCLR_DONE                     ((uint32_t)(0x1UL << MXC_F_I3C_CONT_INTCLR_DONE_POS)) /**< CONT_INTCLR_DONE Mask */

#define MXC_F_I3C_CONT_INTCLR_RX_RDY_POS               11 /**< CONT_INTCLR_RX_RDY Position */
#define MXC_F_I3C_CONT_INTCLR_RX_RDY                   ((uint32_t)(0x1UL << MXC_F_I3C_CONT_INTCLR_RX_RDY_POS)) /**< CONT_INTCLR_RX_RDY Mask */

#define MXC_F_I3C_CONT_INTCLR_TX_NFULL_POS             12 /**< CONT_INTCLR_TX_NFULL Position */
#define MXC_F_I3C_CONT_INTCLR_TX_NFULL                 ((uint32_t)(0x1UL << MXC_F_I3C_CONT_INTCLR_TX_NFULL_POS)) /**< CONT_INTCLR_TX_NFULL Mask */

#define MXC_F_I3C_CONT_INTCLR_IBI_WON_POS              13 /**< CONT_INTCLR_IBI_WON Position */
#define MXC_F_I3C_CONT_INTCLR_IBI_WON                  ((uint32_t)(0x1UL << MXC_F_I3C_CONT_INTCLR_IBI_WON_POS)) /**< CONT_INTCLR_IBI_WON Mask */

#define MXC_F_I3C_CONT_INTCLR_ERRWARN_POS              15 /**< CONT_INTCLR_ERRWARN Position */
#define MXC_F_I3C_CONT_INTCLR_ERRWARN                  ((uint32_t)(0x1UL << MXC_F_I3C_CONT_INTCLR_ERRWARN_POS)) /**< CONT_INTCLR_ERRWARN Mask */

#define MXC_F_I3C_CONT_INTCLR_NOW_CONT_POS             19 /**< CONT_INTCLR_NOW_CONT Position */
#define MXC_F_I3C_CONT_INTCLR_NOW_CONT                 ((uint32_t)(0x1UL << MXC_F_I3C_CONT_INTCLR_NOW_CONT_POS)) /**< CONT_INTCLR_NOW_CONT Mask */

/**@} end of group I3C_CONT_INTCLR_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_CONT_INTFL I3C_CONT_INTFL
 * @brief    Controller Interrupt Flag Register.
 * @{
 */
#define MXC_F_I3C_CONT_INTFL_TARG_START_POS            8 /**< CONT_INTFL_TARG_START Position */
#define MXC_F_I3C_CONT_INTFL_TARG_START                ((uint32_t)(0x1UL << MXC_F_I3C_CONT_INTFL_TARG_START_POS)) /**< CONT_INTFL_TARG_START Mask */

#define MXC_F_I3C_CONT_INTFL_REQ_DONE_POS              9 /**< CONT_INTFL_REQ_DONE Position */
#define MXC_F_I3C_CONT_INTFL_REQ_DONE                  ((uint32_t)(0x1UL << MXC_F_I3C_CONT_INTFL_REQ_DONE_POS)) /**< CONT_INTFL_REQ_DONE Mask */

#define MXC_F_I3C_CONT_INTFL_DONE_POS                  10 /**< CONT_INTFL_DONE Position */
#define MXC_F_I3C_CONT_INTFL_DONE                      ((uint32_t)(0x1UL << MXC_F_I3C_CONT_INTFL_DONE_POS)) /**< CONT_INTFL_DONE Mask */

#define MXC_F_I3C_CONT_INTFL_RX_RDY_POS                11 /**< CONT_INTFL_RX_RDY Position */
#define MXC_F_I3C_CONT_INTFL_RX_RDY                    ((uint32_t)(0x1UL << MXC_F_I3C_CONT_INTFL_RX_RDY_POS)) /**< CONT_INTFL_RX_RDY Mask */

#define MXC_F_I3C_CONT_INTFL_TX_NFULL_POS              12 /**< CONT_INTFL_TX_NFULL Position */
#define MXC_F_I3C_CONT_INTFL_TX_NFULL                  ((uint32_t)(0x1UL << MXC_F_I3C_CONT_INTFL_TX_NFULL_POS)) /**< CONT_INTFL_TX_NFULL Mask */

#define MXC_F_I3C_CONT_INTFL_IBI_WON_POS               13 /**< CONT_INTFL_IBI_WON Position */
#define MXC_F_I3C_CONT_INTFL_IBI_WON                   ((uint32_t)(0x1UL << MXC_F_I3C_CONT_INTFL_IBI_WON_POS)) /**< CONT_INTFL_IBI_WON Mask */

#define MXC_F_I3C_CONT_INTFL_ERRWARN_POS               15 /**< CONT_INTFL_ERRWARN Position */
#define MXC_F_I3C_CONT_INTFL_ERRWARN                   ((uint32_t)(0x1UL << MXC_F_I3C_CONT_INTFL_ERRWARN_POS)) /**< CONT_INTFL_ERRWARN Mask */

#define MXC_F_I3C_CONT_INTFL_NOW_CONT_POS              19 /**< CONT_INTFL_NOW_CONT Position */
#define MXC_F_I3C_CONT_INTFL_NOW_CONT                  ((uint32_t)(0x1UL << MXC_F_I3C_CONT_INTFL_NOW_CONT_POS)) /**< CONT_INTFL_NOW_CONT Mask */

/**@} end of group I3C_CONT_INTFL_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_CONT_ERRWARN I3C_CONT_ERRWARN
 * @brief    Controller Error and Warning Register.
 * @{
 */
#define MXC_F_I3C_CONT_ERRWARN_NACK_POS                2 /**< CONT_ERRWARN_NACK Position */
#define MXC_F_I3C_CONT_ERRWARN_NACK                    ((uint32_t)(0x1UL << MXC_F_I3C_CONT_ERRWARN_NACK_POS)) /**< CONT_ERRWARN_NACK Mask */

#define MXC_F_I3C_CONT_ERRWARN_TX_ABT_POS              3 /**< CONT_ERRWARN_TX_ABT Position */
#define MXC_F_I3C_CONT_ERRWARN_TX_ABT                  ((uint32_t)(0x1UL << MXC_F_I3C_CONT_ERRWARN_TX_ABT_POS)) /**< CONT_ERRWARN_TX_ABT Mask */

#define MXC_F_I3C_CONT_ERRWARN_RX_TERM_POS             4 /**< CONT_ERRWARN_RX_TERM Position */
#define MXC_F_I3C_CONT_ERRWARN_RX_TERM                 ((uint32_t)(0x1UL << MXC_F_I3C_CONT_ERRWARN_RX_TERM_POS)) /**< CONT_ERRWARN_RX_TERM Mask */

#define MXC_F_I3C_CONT_ERRWARN_HDR_PAR_POS             9 /**< CONT_ERRWARN_HDR_PAR Position */
#define MXC_F_I3C_CONT_ERRWARN_HDR_PAR                 ((uint32_t)(0x1UL << MXC_F_I3C_CONT_ERRWARN_HDR_PAR_POS)) /**< CONT_ERRWARN_HDR_PAR Mask */

#define MXC_F_I3C_CONT_ERRWARN_HDR_CRC_POS             10 /**< CONT_ERRWARN_HDR_CRC Position */
#define MXC_F_I3C_CONT_ERRWARN_HDR_CRC                 ((uint32_t)(0x1UL << MXC_F_I3C_CONT_ERRWARN_HDR_CRC_POS)) /**< CONT_ERRWARN_HDR_CRC Mask */

#define MXC_F_I3C_CONT_ERRWARN_RX_UNR_POS              16 /**< CONT_ERRWARN_RX_UNR Position */
#define MXC_F_I3C_CONT_ERRWARN_RX_UNR                  ((uint32_t)(0x1UL << MXC_F_I3C_CONT_ERRWARN_RX_UNR_POS)) /**< CONT_ERRWARN_RX_UNR Mask */

#define MXC_F_I3C_CONT_ERRWARN_TX_OVR_POS              17 /**< CONT_ERRWARN_TX_OVR Position */
#define MXC_F_I3C_CONT_ERRWARN_TX_OVR                  ((uint32_t)(0x1UL << MXC_F_I3C_CONT_ERRWARN_TX_OVR_POS)) /**< CONT_ERRWARN_TX_OVR Mask */

#define MXC_F_I3C_CONT_ERRWARN_MSG_POS                 18 /**< CONT_ERRWARN_MSG Position */
#define MXC_F_I3C_CONT_ERRWARN_MSG                     ((uint32_t)(0x1UL << MXC_F_I3C_CONT_ERRWARN_MSG_POS)) /**< CONT_ERRWARN_MSG Mask */

#define MXC_F_I3C_CONT_ERRWARN_INV_REQ_POS             19 /**< CONT_ERRWARN_INV_REQ Position */
#define MXC_F_I3C_CONT_ERRWARN_INV_REQ                 ((uint32_t)(0x1UL << MXC_F_I3C_CONT_ERRWARN_INV_REQ_POS)) /**< CONT_ERRWARN_INV_REQ Mask */

#define MXC_F_I3C_CONT_ERRWARN_TO_POS                  20 /**< CONT_ERRWARN_TO Position */
#define MXC_F_I3C_CONT_ERRWARN_TO                      ((uint32_t)(0x1UL << MXC_F_I3C_CONT_ERRWARN_TO_POS)) /**< CONT_ERRWARN_TO Mask */

/**@} end of group I3C_CONT_ERRWARN_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_CONT_DMACTRL I3C_CONT_DMACTRL
 * @brief    Controller DMA Control Register.
 * @{
 */
#define MXC_F_I3C_CONT_DMACTRL_RX_EN_POS               0 /**< CONT_DMACTRL_RX_EN Position */
#define MXC_F_I3C_CONT_DMACTRL_RX_EN                   ((uint32_t)(0x3UL << MXC_F_I3C_CONT_DMACTRL_RX_EN_POS)) /**< CONT_DMACTRL_RX_EN Mask */
#define MXC_V_I3C_CONT_DMACTRL_RX_EN_DIS               ((uint32_t)0x0UL) /**< CONT_DMACTRL_RX_EN_DIS Value */
#define MXC_S_I3C_CONT_DMACTRL_RX_EN_DIS               (MXC_V_I3C_CONT_DMACTRL_RX_EN_DIS << MXC_F_I3C_CONT_DMACTRL_RX_EN_POS) /**< CONT_DMACTRL_RX_EN_DIS Setting */
#define MXC_V_I3C_CONT_DMACTRL_RX_EN_ONE_FR            ((uint32_t)0x1UL) /**< CONT_DMACTRL_RX_EN_ONE_FR Value */
#define MXC_S_I3C_CONT_DMACTRL_RX_EN_ONE_FR            (MXC_V_I3C_CONT_DMACTRL_RX_EN_ONE_FR << MXC_F_I3C_CONT_DMACTRL_RX_EN_POS) /**< CONT_DMACTRL_RX_EN_ONE_FR Setting */
#define MXC_V_I3C_CONT_DMACTRL_RX_EN_EN                ((uint32_t)0x2UL) /**< CONT_DMACTRL_RX_EN_EN Value */
#define MXC_S_I3C_CONT_DMACTRL_RX_EN_EN                (MXC_V_I3C_CONT_DMACTRL_RX_EN_EN << MXC_F_I3C_CONT_DMACTRL_RX_EN_POS) /**< CONT_DMACTRL_RX_EN_EN Setting */

#define MXC_F_I3C_CONT_DMACTRL_TX_EN_POS               2 /**< CONT_DMACTRL_TX_EN Position */
#define MXC_F_I3C_CONT_DMACTRL_TX_EN                   ((uint32_t)(0x3UL << MXC_F_I3C_CONT_DMACTRL_TX_EN_POS)) /**< CONT_DMACTRL_TX_EN Mask */
#define MXC_V_I3C_CONT_DMACTRL_TX_EN_DIS               ((uint32_t)0x0UL) /**< CONT_DMACTRL_TX_EN_DIS Value */
#define MXC_S_I3C_CONT_DMACTRL_TX_EN_DIS               (MXC_V_I3C_CONT_DMACTRL_TX_EN_DIS << MXC_F_I3C_CONT_DMACTRL_TX_EN_POS) /**< CONT_DMACTRL_TX_EN_DIS Setting */
#define MXC_V_I3C_CONT_DMACTRL_TX_EN_ONE_FR            ((uint32_t)0x1UL) /**< CONT_DMACTRL_TX_EN_ONE_FR Value */
#define MXC_S_I3C_CONT_DMACTRL_TX_EN_ONE_FR            (MXC_V_I3C_CONT_DMACTRL_TX_EN_ONE_FR << MXC_F_I3C_CONT_DMACTRL_TX_EN_POS) /**< CONT_DMACTRL_TX_EN_ONE_FR Setting */
#define MXC_V_I3C_CONT_DMACTRL_TX_EN_EN                ((uint32_t)0x2UL) /**< CONT_DMACTRL_TX_EN_EN Value */
#define MXC_S_I3C_CONT_DMACTRL_TX_EN_EN                (MXC_V_I3C_CONT_DMACTRL_TX_EN_EN << MXC_F_I3C_CONT_DMACTRL_TX_EN_POS) /**< CONT_DMACTRL_TX_EN_EN Setting */

#define MXC_F_I3C_CONT_DMACTRL_WIDTH_POS               4 /**< CONT_DMACTRL_WIDTH Position */
#define MXC_F_I3C_CONT_DMACTRL_WIDTH                   ((uint32_t)(0x3UL << MXC_F_I3C_CONT_DMACTRL_WIDTH_POS)) /**< CONT_DMACTRL_WIDTH Mask */
#define MXC_V_I3C_CONT_DMACTRL_WIDTH_BYTE              ((uint32_t)0x0UL) /**< CONT_DMACTRL_WIDTH_BYTE Value */
#define MXC_S_I3C_CONT_DMACTRL_WIDTH_BYTE              (MXC_V_I3C_CONT_DMACTRL_WIDTH_BYTE << MXC_F_I3C_CONT_DMACTRL_WIDTH_POS) /**< CONT_DMACTRL_WIDTH_BYTE Setting */
#define MXC_V_I3C_CONT_DMACTRL_WIDTH_HALFWORD          ((uint32_t)0x2UL) /**< CONT_DMACTRL_WIDTH_HALFWORD Value */
#define MXC_S_I3C_CONT_DMACTRL_WIDTH_HALFWORD          (MXC_V_I3C_CONT_DMACTRL_WIDTH_HALFWORD << MXC_F_I3C_CONT_DMACTRL_WIDTH_POS) /**< CONT_DMACTRL_WIDTH_HALFWORD Setting */

/**@} end of group I3C_CONT_DMACTRL_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_CONT_FIFOCTRL I3C_CONT_FIFOCTRL
 * @brief    Controller FIFO Control Register.
 * @{
 */
#define MXC_F_I3C_CONT_FIFOCTRL_TX_FLUSH_POS           0 /**< CONT_FIFOCTRL_TX_FLUSH Position */
#define MXC_F_I3C_CONT_FIFOCTRL_TX_FLUSH               ((uint32_t)(0x1UL << MXC_F_I3C_CONT_FIFOCTRL_TX_FLUSH_POS)) /**< CONT_FIFOCTRL_TX_FLUSH Mask */

#define MXC_F_I3C_CONT_FIFOCTRL_RX_FLUSH_POS           1 /**< CONT_FIFOCTRL_RX_FLUSH Position */
#define MXC_F_I3C_CONT_FIFOCTRL_RX_FLUSH               ((uint32_t)(0x1UL << MXC_F_I3C_CONT_FIFOCTRL_RX_FLUSH_POS)) /**< CONT_FIFOCTRL_RX_FLUSH Mask */

#define MXC_F_I3C_CONT_FIFOCTRL_UNLOCK_POS             3 /**< CONT_FIFOCTRL_UNLOCK Position */
#define MXC_F_I3C_CONT_FIFOCTRL_UNLOCK                 ((uint32_t)(0x1UL << MXC_F_I3C_CONT_FIFOCTRL_UNLOCK_POS)) /**< CONT_FIFOCTRL_UNLOCK Mask */

#define MXC_F_I3C_CONT_FIFOCTRL_TX_THD_LVL_POS         4 /**< CONT_FIFOCTRL_TX_THD_LVL Position */
#define MXC_F_I3C_CONT_FIFOCTRL_TX_THD_LVL             ((uint32_t)(0x3UL << MXC_F_I3C_CONT_FIFOCTRL_TX_THD_LVL_POS)) /**< CONT_FIFOCTRL_TX_THD_LVL Mask */
#define MXC_V_I3C_CONT_FIFOCTRL_TX_THD_LVL_EMPTY       ((uint32_t)0x0UL) /**< CONT_FIFOCTRL_TX_THD_LVL_EMPTY Value */
#define MXC_S_I3C_CONT_FIFOCTRL_TX_THD_LVL_EMPTY       (MXC_V_I3C_CONT_FIFOCTRL_TX_THD_LVL_EMPTY << MXC_F_I3C_CONT_FIFOCTRL_TX_THD_LVL_POS) /**< CONT_FIFOCTRL_TX_THD_LVL_EMPTY Setting */
#define MXC_V_I3C_CONT_FIFOCTRL_TX_THD_LVL_QUARTER_FULL ((uint32_t)0x1UL) /**< CONT_FIFOCTRL_TX_THD_LVL_QUARTER_FULL Value */
#define MXC_S_I3C_CONT_FIFOCTRL_TX_THD_LVL_QUARTER_FULL (MXC_V_I3C_CONT_FIFOCTRL_TX_THD_LVL_QUARTER_FULL << MXC_F_I3C_CONT_FIFOCTRL_TX_THD_LVL_POS) /**< CONT_FIFOCTRL_TX_THD_LVL_QUARTER_FULL Setting */
#define MXC_V_I3C_CONT_FIFOCTRL_TX_THD_LVL_HALF_FULL   ((uint32_t)0x2UL) /**< CONT_FIFOCTRL_TX_THD_LVL_HALF_FULL Value */
#define MXC_S_I3C_CONT_FIFOCTRL_TX_THD_LVL_HALF_FULL   (MXC_V_I3C_CONT_FIFOCTRL_TX_THD_LVL_HALF_FULL << MXC_F_I3C_CONT_FIFOCTRL_TX_THD_LVL_POS) /**< CONT_FIFOCTRL_TX_THD_LVL_HALF_FULL Setting */
#define MXC_V_I3C_CONT_FIFOCTRL_TX_THD_LVL_ALMOST_FULL ((uint32_t)0x3UL) /**< CONT_FIFOCTRL_TX_THD_LVL_ALMOST_FULL Value */
#define MXC_S_I3C_CONT_FIFOCTRL_TX_THD_LVL_ALMOST_FULL (MXC_V_I3C_CONT_FIFOCTRL_TX_THD_LVL_ALMOST_FULL << MXC_F_I3C_CONT_FIFOCTRL_TX_THD_LVL_POS) /**< CONT_FIFOCTRL_TX_THD_LVL_ALMOST_FULL Setting */

#define MXC_F_I3C_CONT_FIFOCTRL_RX_THD_LVL_POS         6 /**< CONT_FIFOCTRL_RX_THD_LVL Position */
#define MXC_F_I3C_CONT_FIFOCTRL_RX_THD_LVL             ((uint32_t)(0x3UL << MXC_F_I3C_CONT_FIFOCTRL_RX_THD_LVL_POS)) /**< CONT_FIFOCTRL_RX_THD_LVL Mask */
#define MXC_V_I3C_CONT_FIFOCTRL_RX_THD_LVL_NOT_EMPTY   ((uint32_t)0x0UL) /**< CONT_FIFOCTRL_RX_THD_LVL_NOT_EMPTY Value */
#define MXC_S_I3C_CONT_FIFOCTRL_RX_THD_LVL_NOT_EMPTY   (MXC_V_I3C_CONT_FIFOCTRL_RX_THD_LVL_NOT_EMPTY << MXC_F_I3C_CONT_FIFOCTRL_RX_THD_LVL_POS) /**< CONT_FIFOCTRL_RX_THD_LVL_NOT_EMPTY Setting */
#define MXC_V_I3C_CONT_FIFOCTRL_RX_THD_LVL_QUARTER_FULL ((uint32_t)0x1UL) /**< CONT_FIFOCTRL_RX_THD_LVL_QUARTER_FULL Value */
#define MXC_S_I3C_CONT_FIFOCTRL_RX_THD_LVL_QUARTER_FULL (MXC_V_I3C_CONT_FIFOCTRL_RX_THD_LVL_QUARTER_FULL << MXC_F_I3C_CONT_FIFOCTRL_RX_THD_LVL_POS) /**< CONT_FIFOCTRL_RX_THD_LVL_QUARTER_FULL Setting */
#define MXC_V_I3C_CONT_FIFOCTRL_RX_THD_LVL_HALF_FULL   ((uint32_t)0x2UL) /**< CONT_FIFOCTRL_RX_THD_LVL_HALF_FULL Value */
#define MXC_S_I3C_CONT_FIFOCTRL_RX_THD_LVL_HALF_FULL   (MXC_V_I3C_CONT_FIFOCTRL_RX_THD_LVL_HALF_FULL << MXC_F_I3C_CONT_FIFOCTRL_RX_THD_LVL_POS) /**< CONT_FIFOCTRL_RX_THD_LVL_HALF_FULL Setting */
#define MXC_V_I3C_CONT_FIFOCTRL_RX_THD_LVL_3_QUARTER_FULL ((uint32_t)0x3UL) /**< CONT_FIFOCTRL_RX_THD_LVL_3_QUARTER_FULL Value */
#define MXC_S_I3C_CONT_FIFOCTRL_RX_THD_LVL_3_QUARTER_FULL (MXC_V_I3C_CONT_FIFOCTRL_RX_THD_LVL_3_QUARTER_FULL << MXC_F_I3C_CONT_FIFOCTRL_RX_THD_LVL_POS) /**< CONT_FIFOCTRL_RX_THD_LVL_3_QUARTER_FULL Setting */

#define MXC_F_I3C_CONT_FIFOCTRL_TX_LVL_POS             16 /**< CONT_FIFOCTRL_TX_LVL Position */
#define MXC_F_I3C_CONT_FIFOCTRL_TX_LVL                 ((uint32_t)(0x3FUL << MXC_F_I3C_CONT_FIFOCTRL_TX_LVL_POS)) /**< CONT_FIFOCTRL_TX_LVL Mask */

#define MXC_F_I3C_CONT_FIFOCTRL_RX_LVL_POS             24 /**< CONT_FIFOCTRL_RX_LVL Position */
#define MXC_F_I3C_CONT_FIFOCTRL_RX_LVL                 ((uint32_t)(0x3FUL << MXC_F_I3C_CONT_FIFOCTRL_RX_LVL_POS)) /**< CONT_FIFOCTRL_RX_LVL Mask */

#define MXC_F_I3C_CONT_FIFOCTRL_TX_FULL_POS            30 /**< CONT_FIFOCTRL_TX_FULL Position */
#define MXC_F_I3C_CONT_FIFOCTRL_TX_FULL                ((uint32_t)(0x1UL << MXC_F_I3C_CONT_FIFOCTRL_TX_FULL_POS)) /**< CONT_FIFOCTRL_TX_FULL Mask */

#define MXC_F_I3C_CONT_FIFOCTRL_RX_EM_POS              31 /**< CONT_FIFOCTRL_RX_EM Position */
#define MXC_F_I3C_CONT_FIFOCTRL_RX_EM                  ((uint32_t)(0x1UL << MXC_F_I3C_CONT_FIFOCTRL_RX_EM_POS)) /**< CONT_FIFOCTRL_RX_EM Mask */

/**@} end of group I3C_CONT_FIFOCTRL_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_CONT_TXFIFO8 I3C_CONT_TXFIFO8
 * @brief    Controller Write Byte Data Register.
 * @{
 */
#define MXC_F_I3C_CONT_TXFIFO8_DATA_POS                0 /**< CONT_TXFIFO8_DATA Position */
#define MXC_F_I3C_CONT_TXFIFO8_DATA                    ((uint32_t)(0xFFUL << MXC_F_I3C_CONT_TXFIFO8_DATA_POS)) /**< CONT_TXFIFO8_DATA Mask */

#define MXC_F_I3C_CONT_TXFIFO8_END_POS                 8 /**< CONT_TXFIFO8_END Position */
#define MXC_F_I3C_CONT_TXFIFO8_END                     ((uint32_t)(0x1UL << MXC_F_I3C_CONT_TXFIFO8_END_POS)) /**< CONT_TXFIFO8_END Mask */

#define MXC_F_I3C_CONT_TXFIFO8_END2_POS                16 /**< CONT_TXFIFO8_END2 Position */
#define MXC_F_I3C_CONT_TXFIFO8_END2                    ((uint32_t)(0x1UL << MXC_F_I3C_CONT_TXFIFO8_END2_POS)) /**< CONT_TXFIFO8_END2 Mask */

/**@} end of group I3C_CONT_TXFIFO8_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_CONT_TXFIFO8E I3C_CONT_TXFIFO8E
 * @brief    Controller Write Byte Data as End Register.
 * @{
 */
#define MXC_F_I3C_CONT_TXFIFO8E_DATA_POS               0 /**< CONT_TXFIFO8E_DATA Position */
#define MXC_F_I3C_CONT_TXFIFO8E_DATA                   ((uint32_t)(0xFFUL << MXC_F_I3C_CONT_TXFIFO8E_DATA_POS)) /**< CONT_TXFIFO8E_DATA Mask */

/**@} end of group I3C_CONT_TXFIFO8E_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_CONT_TXFIFO16 I3C_CONT_TXFIFO16
 * @brief    Controller Write Half-Word Data Register.
 * @{
 */
#define MXC_F_I3C_CONT_TXFIFO16_DATA_POS               0 /**< CONT_TXFIFO16_DATA Position */
#define MXC_F_I3C_CONT_TXFIFO16_DATA                   ((uint32_t)(0xFFFFUL << MXC_F_I3C_CONT_TXFIFO16_DATA_POS)) /**< CONT_TXFIFO16_DATA Mask */

#define MXC_F_I3C_CONT_TXFIFO16_END_POS                16 /**< CONT_TXFIFO16_END Position */
#define MXC_F_I3C_CONT_TXFIFO16_END                    ((uint32_t)(0x1UL << MXC_F_I3C_CONT_TXFIFO16_END_POS)) /**< CONT_TXFIFO16_END Mask */

/**@} end of group I3C_CONT_TXFIFO16_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_CONT_TXFIFO16E I3C_CONT_TXFIFO16E
 * @brief    Controller Write Half-Word Data as End Register.
 * @{
 */
#define MXC_F_I3C_CONT_TXFIFO16E_DATA_POS              0 /**< CONT_TXFIFO16E_DATA Position */
#define MXC_F_I3C_CONT_TXFIFO16E_DATA                  ((uint32_t)(0xFFFFUL << MXC_F_I3C_CONT_TXFIFO16E_DATA_POS)) /**< CONT_TXFIFO16E_DATA Mask */

/**@} end of group I3C_CONT_TXFIFO16E_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_CONT_RXFIFO8 I3C_CONT_RXFIFO8
 * @brief    Controller Read Byte Data Register.
 * @{
 */
#define MXC_F_I3C_CONT_RXFIFO8_DATA_POS                0 /**< CONT_RXFIFO8_DATA Position */
#define MXC_F_I3C_CONT_RXFIFO8_DATA                    ((uint32_t)(0xFFUL << MXC_F_I3C_CONT_RXFIFO8_DATA_POS)) /**< CONT_RXFIFO8_DATA Mask */

/**@} end of group I3C_CONT_RXFIFO8_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_CONT_RXFIFO16 I3C_CONT_RXFIFO16
 * @brief    Controller Read Half-Word Data Register.
 * @{
 */
#define MXC_F_I3C_CONT_RXFIFO16_DATA_POS               0 /**< CONT_RXFIFO16_DATA Position */
#define MXC_F_I3C_CONT_RXFIFO16_DATA                   ((uint32_t)(0xFFFFUL << MXC_F_I3C_CONT_RXFIFO16_DATA_POS)) /**< CONT_RXFIFO16_DATA Mask */

/**@} end of group I3C_CONT_RXFIFO16_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_CONT_TXFIFO8O I3C_CONT_TXFIFO8O
 * @brief    Controller Byte-Only Write Byte Data Register.
 * @{
 */
#define MXC_F_I3C_CONT_TXFIFO8O_DATA_POS               0 /**< CONT_TXFIFO8O_DATA Position */
#define MXC_F_I3C_CONT_TXFIFO8O_DATA                   ((uint32_t)(0xFFUL << MXC_F_I3C_CONT_TXFIFO8O_DATA_POS)) /**< CONT_TXFIFO8O_DATA Mask */

/**@} end of group I3C_CONT_TXFIFO8O_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_CONT_TXSDRMSG_CTRL I3C_CONT_TXSDRMSG_CTRL
 * @brief    Controller Start or Continue SDR Message Register.
 * @{
 */
#define MXC_F_I3C_CONT_TXSDRMSG_CTRL_RDWR_DIR_POS      0 /**< CONT_TXSDRMSG_CTRL_RDWR_DIR Position */
#define MXC_F_I3C_CONT_TXSDRMSG_CTRL_RDWR_DIR          ((uint32_t)(0x1UL << MXC_F_I3C_CONT_TXSDRMSG_CTRL_RDWR_DIR_POS)) /**< CONT_TXSDRMSG_CTRL_RDWR_DIR Mask */

#define MXC_F_I3C_CONT_TXSDRMSG_CTRL_ADDR_POS          1 /**< CONT_TXSDRMSG_CTRL_ADDR Position */
#define MXC_F_I3C_CONT_TXSDRMSG_CTRL_ADDR              ((uint32_t)(0x7FUL << MXC_F_I3C_CONT_TXSDRMSG_CTRL_ADDR_POS)) /**< CONT_TXSDRMSG_CTRL_ADDR Mask */

#define MXC_F_I3C_CONT_TXSDRMSG_CTRL_END_POS           8 /**< CONT_TXSDRMSG_CTRL_END Position */
#define MXC_F_I3C_CONT_TXSDRMSG_CTRL_END               ((uint32_t)(0x1UL << MXC_F_I3C_CONT_TXSDRMSG_CTRL_END_POS)) /**< CONT_TXSDRMSG_CTRL_END Mask */

#define MXC_F_I3C_CONT_TXSDRMSG_CTRL_I2C_EN_POS        10 /**< CONT_TXSDRMSG_CTRL_I2C_EN Position */
#define MXC_F_I3C_CONT_TXSDRMSG_CTRL_I2C_EN            ((uint32_t)(0x1UL << MXC_F_I3C_CONT_TXSDRMSG_CTRL_I2C_EN_POS)) /**< CONT_TXSDRMSG_CTRL_I2C_EN Mask */

#define MXC_F_I3C_CONT_TXSDRMSG_CTRL_LEN_POS           11 /**< CONT_TXSDRMSG_CTRL_LEN Position */
#define MXC_F_I3C_CONT_TXSDRMSG_CTRL_LEN               ((uint32_t)(0x1FUL << MXC_F_I3C_CONT_TXSDRMSG_CTRL_LEN_POS)) /**< CONT_TXSDRMSG_CTRL_LEN Mask */

/**@} end of group I3C_CONT_TXSDRMSG_CTRL_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_CONT_TXSDRMSG_FIFO I3C_CONT_TXSDRMSG_FIFO
 * @brief    Controller Start or Continue SDR Message Register.
 * @{
 */
#define MXC_F_I3C_CONT_TXSDRMSG_FIFO_DATA_POS          0 /**< CONT_TXSDRMSG_FIFO_DATA Position */
#define MXC_F_I3C_CONT_TXSDRMSG_FIFO_DATA              ((uint32_t)(0xFFFFUL << MXC_F_I3C_CONT_TXSDRMSG_FIFO_DATA_POS)) /**< CONT_TXSDRMSG_FIFO_DATA Mask */

/**@} end of group I3C_CONT_TXSDRMSG_FIFO_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_CONT_RXSDRMSG I3C_CONT_RXSDRMSG
 * @brief    Controller Read SDR Message Data Register.
 * @{
 */
#define MXC_F_I3C_CONT_RXSDRMSG_DATA_POS               0 /**< CONT_RXSDRMSG_DATA Position */
#define MXC_F_I3C_CONT_RXSDRMSG_DATA                   ((uint32_t)(0xFFFFUL << MXC_F_I3C_CONT_RXSDRMSG_DATA_POS)) /**< CONT_RXSDRMSG_DATA Mask */

/**@} end of group I3C_CONT_RXSDRMSG_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_CONT_TXDDRMSG I3C_CONT_TXDDRMSG
 * @brief    Controller Start or Continue DDR Message Register.
 * @{
 */
#define MXC_F_I3C_CONT_TXDDRMSG_MSG_POS                0 /**< CONT_TXDDRMSG_MSG Position */
#define MXC_F_I3C_CONT_TXDDRMSG_MSG                    ((uint32_t)(0xFFFFUL << MXC_F_I3C_CONT_TXDDRMSG_MSG_POS)) /**< CONT_TXDDRMSG_MSG Mask */

/**@} end of group I3C_CONT_TXDDRMSG_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_CONT_RXDDR16 I3C_CONT_RXDDR16
 * @brief    Controller Read DDR Message Data Register.
 * @{
 */
#define MXC_F_I3C_CONT_RXDDR16_DATA_POS                0 /**< CONT_RXDDR16_DATA Position */
#define MXC_F_I3C_CONT_RXDDR16_DATA                    ((uint32_t)(0xFFFFUL << MXC_F_I3C_CONT_RXDDR16_DATA_POS)) /**< CONT_RXDDR16_DATA Mask */

/**@} end of group I3C_CONT_RXDDR16_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_CONT_DYNADDR I3C_CONT_DYNADDR
 * @brief    Controller Dynamic Address Register.
 * @{
 */
#define MXC_F_I3C_CONT_DYNADDR_ADDR_POS                1 /**< CONT_DYNADDR_ADDR Position */
#define MXC_F_I3C_CONT_DYNADDR_ADDR                    ((uint32_t)(0x7FUL << MXC_F_I3C_CONT_DYNADDR_ADDR_POS)) /**< CONT_DYNADDR_ADDR Mask */

#define MXC_F_I3C_CONT_DYNADDR_VALID_POS               8 /**< CONT_DYNADDR_VALID Position */
#define MXC_F_I3C_CONT_DYNADDR_VALID                   ((uint32_t)(0x1UL << MXC_F_I3C_CONT_DYNADDR_VALID_POS)) /**< CONT_DYNADDR_VALID Mask */

/**@} end of group I3C_CONT_DYNADDR_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_TARG_GROUPDEF I3C_TARG_GROUPDEF
 * @brief    Target Group Definition Register.
 * @{
 */
#define MXC_F_I3C_TARG_GROUPDEF_ADDR_EN_POS            0 /**< TARG_GROUPDEF_ADDR_EN Position */
#define MXC_F_I3C_TARG_GROUPDEF_ADDR_EN                ((uint32_t)(0x1UL << MXC_F_I3C_TARG_GROUPDEF_ADDR_EN_POS)) /**< TARG_GROUPDEF_ADDR_EN Mask */

#define MXC_F_I3C_TARG_GROUPDEF_ADDR_POS               1 /**< TARG_GROUPDEF_ADDR Position */
#define MXC_F_I3C_TARG_GROUPDEF_ADDR                   ((uint32_t)(0x7FUL << MXC_F_I3C_TARG_GROUPDEF_ADDR_POS)) /**< TARG_GROUPDEF_ADDR Mask */

/**@} end of group I3C_TARG_GROUPDEF_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_TARG_MAPCTRL0 I3C_TARG_MAPCTRL0
 * @brief    Target Primary Map Control Register.
 * @{
 */
#define MXC_F_I3C_TARG_MAPCTRL0_DYNADDR_EN_POS         0 /**< TARG_MAPCTRL0_DYNADDR_EN Position */
#define MXC_F_I3C_TARG_MAPCTRL0_DYNADDR_EN             ((uint32_t)(0x1UL << MXC_F_I3C_TARG_MAPCTRL0_DYNADDR_EN_POS)) /**< TARG_MAPCTRL0_DYNADDR_EN Mask */

#define MXC_F_I3C_TARG_MAPCTRL0_DYNADDR_POS            1 /**< TARG_MAPCTRL0_DYNADDR Position */
#define MXC_F_I3C_TARG_MAPCTRL0_DYNADDR                ((uint32_t)(0x7FUL << MXC_F_I3C_TARG_MAPCTRL0_DYNADDR_POS)) /**< TARG_MAPCTRL0_DYNADDR Mask */

#define MXC_F_I3C_TARG_MAPCTRL0_CAUSE_POS              8 /**< TARG_MAPCTRL0_CAUSE Position */
#define MXC_F_I3C_TARG_MAPCTRL0_CAUSE                  ((uint32_t)(0x7UL << MXC_F_I3C_TARG_MAPCTRL0_CAUSE_POS)) /**< TARG_MAPCTRL0_CAUSE Mask */

/**@} end of group I3C_TARG_MAPCTRL0_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_TARG_MAPCTRL1 I3C_TARG_MAPCTRL1
 * @brief    Target Map Control 1 Register.
 * @{
 */
#define MXC_F_I3C_TARG_MAPCTRL1_EN_POS                 0 /**< TARG_MAPCTRL1_EN Position */
#define MXC_F_I3C_TARG_MAPCTRL1_EN                     ((uint32_t)(0x1UL << MXC_F_I3C_TARG_MAPCTRL1_EN_POS)) /**< TARG_MAPCTRL1_EN Mask */

#define MXC_F_I3C_TARG_MAPCTRL1_ADDR_POS               1 /**< TARG_MAPCTRL1_ADDR Position */
#define MXC_F_I3C_TARG_MAPCTRL1_ADDR                   ((uint32_t)(0x7FUL << MXC_F_I3C_TARG_MAPCTRL1_ADDR_POS)) /**< TARG_MAPCTRL1_ADDR Mask */

#define MXC_F_I3C_TARG_MAPCTRL1_STATADDR_EN_POS        8 /**< TARG_MAPCTRL1_STATADDR_EN Position */
#define MXC_F_I3C_TARG_MAPCTRL1_STATADDR_EN            ((uint32_t)(0x1UL << MXC_F_I3C_TARG_MAPCTRL1_STATADDR_EN_POS)) /**< TARG_MAPCTRL1_STATADDR_EN Mask */

#define MXC_F_I3C_TARG_MAPCTRL1_STATADDR_10B_POS       9 /**< TARG_MAPCTRL1_STATADDR_10B Position */
#define MXC_F_I3C_TARG_MAPCTRL1_STATADDR_10B           ((uint32_t)(0x7UL << MXC_F_I3C_TARG_MAPCTRL1_STATADDR_10B_POS)) /**< TARG_MAPCTRL1_STATADDR_10B Mask */

#define MXC_F_I3C_TARG_MAPCTRL1_NACK_POS               12 /**< TARG_MAPCTRL1_NACK Position */
#define MXC_F_I3C_TARG_MAPCTRL1_NACK                   ((uint32_t)(0x1UL << MXC_F_I3C_TARG_MAPCTRL1_NACK_POS)) /**< TARG_MAPCTRL1_NACK Mask */

/**@} end of group I3C_TARG_MAPCTRL1_Register */

/**
 * @ingroup  i3c_registers
 * @defgroup I3C_TARG_MAPCTRL2 I3C_TARG_MAPCTRL2
 * @brief    Target Map Control 2 Register.
 * @{
 */
#define MXC_F_I3C_TARG_MAPCTRL2_EN_POS                 0 /**< TARG_MAPCTRL2_EN Position */
#define MXC_F_I3C_TARG_MAPCTRL2_EN                     ((uint32_t)(0x1UL << MXC_F_I3C_TARG_MAPCTRL2_EN_POS)) /**< TARG_MAPCTRL2_EN Mask */

#define MXC_F_I3C_TARG_MAPCTRL2_ADDR_POS               1 /**< TARG_MAPCTRL2_ADDR Position */
#define MXC_F_I3C_TARG_MAPCTRL2_ADDR                   ((uint32_t)(0x7FUL << MXC_F_I3C_TARG_MAPCTRL2_ADDR_POS)) /**< TARG_MAPCTRL2_ADDR Mask */

#define MXC_F_I3C_TARG_MAPCTRL2_STATADDR_EN_POS        8 /**< TARG_MAPCTRL2_STATADDR_EN Position */
#define MXC_F_I3C_TARG_MAPCTRL2_STATADDR_EN            ((uint32_t)(0x1UL << MXC_F_I3C_TARG_MAPCTRL2_STATADDR_EN_POS)) /**< TARG_MAPCTRL2_STATADDR_EN Mask */

#define MXC_F_I3C_TARG_MAPCTRL2_NACK_POS               12 /**< TARG_MAPCTRL2_NACK Position */
#define MXC_F_I3C_TARG_MAPCTRL2_NACK                   ((uint32_t)(0x1UL << MXC_F_I3C_TARG_MAPCTRL2_NACK_POS)) /**< TARG_MAPCTRL2_NACK Mask */

#define MXC_F_I3C_TARG_MAPCTRL2_AUTO_EN_POS            13 /**< TARG_MAPCTRL2_AUTO_EN Position */
#define MXC_F_I3C_TARG_MAPCTRL2_AUTO_EN                ((uint32_t)(0x1UL << MXC_F_I3C_TARG_MAPCTRL2_AUTO_EN_POS)) /**< TARG_MAPCTRL2_AUTO_EN Mask */

#define MXC_F_I3C_TARG_MAPCTRL2_PID_POS                14 /**< TARG_MAPCTRL2_PID Position */
#define MXC_F_I3C_TARG_MAPCTRL2_PID                    ((uint32_t)(0x3FFFFUL << MXC_F_I3C_TARG_MAPCTRL2_PID_POS)) /**< TARG_MAPCTRL2_PID Mask */

/**@} end of group I3C_TARG_MAPCTRL2_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_I3C_REGS_H_
