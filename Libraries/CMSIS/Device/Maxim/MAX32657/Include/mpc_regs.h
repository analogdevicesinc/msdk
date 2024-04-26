/**
 * @file    mpc_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the MPC Peripheral Module.
 * @note    This file is @generated.
 * @ingroup mpc_registers
 */

/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. All Rights Reserved.
 * (now owned by Analog Devices, Inc.),
 * Copyright (C) 2023 Analog Devices, Inc. All Rights Reserved. This software
 * is proprietary to Analog Devices, Inc. and its licensors.
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_MPC_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_MPC_REGS_H_

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
 * @ingroup     mpc
 * @defgroup    mpc_registers MPC_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the MPC Peripheral Module.
 * @details     Memory Protection Control.
 */

/**
 * @ingroup mpc_registers
 * Structure type to access the MPC Registers.
 */
typedef struct {
    __IO uint32_t ctrl;                 /**< <tt>\b 0x00:</tt> MPC CTRL Register */
    __R  uint32_t rsv_0x4_0xf[3];
    __I  uint32_t blk_max;              /**< <tt>\b 0x10:</tt> MPC BLK_MAX Register */
    __I  uint32_t blk_cfg;              /**< <tt>\b 0x14:</tt> MPC BLK_CFG Register */
    __IO uint32_t blk_idx;              /**< <tt>\b 0x18:</tt> MPC BLK_IDX Register */
    __IO uint32_t blk_lutn;             /**< <tt>\b 0x1C:</tt> MPC BLK_LUTN Register */
    __I  uint32_t int_stat;             /**< <tt>\b 0x20:</tt> MPC INT_STAT Register */
    __O  uint32_t int_clear;            /**< <tt>\b 0x24:</tt> MPC INT_CLEAR Register */
    __IO uint32_t int_en;               /**< <tt>\b 0x28:</tt> MPC INT_EN Register */
    __I  uint32_t int_info1;            /**< <tt>\b 0x2C:</tt> MPC INT_INFO1 Register */
    __I  uint32_t int_info2;            /**< <tt>\b 0x30:</tt> MPC INT_INFO2 Register */
    __IO uint32_t int_set;              /**< <tt>\b 0x34:</tt> MPC INT_SET Register */
    __R  uint32_t rsv_0x38_0xfcf[998];
    __I  uint32_t pidr4;                /**< <tt>\b 0xFD0:</tt> MPC PIDR4 Register */
    __I  uint32_t pidr5;                /**< <tt>\b 0xFD4:</tt> MPC PIDR5 Register */
    __I  uint32_t pidr6;                /**< <tt>\b 0xFD8:</tt> MPC PIDR6 Register */
    __I  uint32_t pidr7;                /**< <tt>\b 0xFDC:</tt> MPC PIDR7 Register */
    __I  uint32_t pidr0;                /**< <tt>\b 0xFE0:</tt> MPC PIDR0 Register */
    __I  uint32_t pidr1;                /**< <tt>\b 0xFE4:</tt> MPC PIDR1 Register */
    __I  uint32_t pidr2;                /**< <tt>\b 0xFE8:</tt> MPC PIDR2 Register */
    __I  uint32_t pidr3;                /**< <tt>\b 0xFEC:</tt> MPC PIDR3 Register */
    __I  uint32_t cidr0;                /**< <tt>\b 0xFF0:</tt> MPC CIDR0 Register */
    __I  uint32_t cidr1;                /**< <tt>\b 0xFF4:</tt> MPC CIDR1 Register */
    __I  uint32_t cidr2;                /**< <tt>\b 0xFF8:</tt> MPC CIDR2 Register */
    __I  uint32_t cidr3;                /**< <tt>\b 0xFFC:</tt> MPC CIDR3 Register */
} mxc_mpc_regs_t;

/* Register offsets for module MPC */
/**
 * @ingroup    mpc_registers
 * @defgroup   MPC_Register_Offsets Register Offsets
 * @brief      MPC Peripheral Register Offsets from the MPC Base Peripheral Address.
 * @{
 */
#define MXC_R_MPC_CTRL                     ((uint32_t)0x00000000UL) /**< Offset from MPC Base Address: <tt> 0x0000</tt> */
#define MXC_R_MPC_BLK_MAX                  ((uint32_t)0x00000010UL) /**< Offset from MPC Base Address: <tt> 0x0010</tt> */
#define MXC_R_MPC_BLK_CFG                  ((uint32_t)0x00000014UL) /**< Offset from MPC Base Address: <tt> 0x0014</tt> */
#define MXC_R_MPC_BLK_IDX                  ((uint32_t)0x00000018UL) /**< Offset from MPC Base Address: <tt> 0x0018</tt> */
#define MXC_R_MPC_BLK_LUTN                 ((uint32_t)0x0000001CUL) /**< Offset from MPC Base Address: <tt> 0x001C</tt> */
#define MXC_R_MPC_INT_STAT                 ((uint32_t)0x00000020UL) /**< Offset from MPC Base Address: <tt> 0x0020</tt> */
#define MXC_R_MPC_INT_CLEAR                ((uint32_t)0x00000024UL) /**< Offset from MPC Base Address: <tt> 0x0024</tt> */
#define MXC_R_MPC_INT_EN                   ((uint32_t)0x00000028UL) /**< Offset from MPC Base Address: <tt> 0x0028</tt> */
#define MXC_R_MPC_INT_INFO1                ((uint32_t)0x0000002CUL) /**< Offset from MPC Base Address: <tt> 0x002C</tt> */
#define MXC_R_MPC_INT_INFO2                ((uint32_t)0x00000030UL) /**< Offset from MPC Base Address: <tt> 0x0030</tt> */
#define MXC_R_MPC_INT_SET                  ((uint32_t)0x00000034UL) /**< Offset from MPC Base Address: <tt> 0x0034</tt> */
#define MXC_R_MPC_PIDR4                    ((uint32_t)0x00000FD0UL) /**< Offset from MPC Base Address: <tt> 0x0FD0</tt> */
#define MXC_R_MPC_PIDR5                    ((uint32_t)0x00000FD4UL) /**< Offset from MPC Base Address: <tt> 0x0FD4</tt> */
#define MXC_R_MPC_PIDR6                    ((uint32_t)0x00000FD8UL) /**< Offset from MPC Base Address: <tt> 0x0FD8</tt> */
#define MXC_R_MPC_PIDR7                    ((uint32_t)0x00000FDCUL) /**< Offset from MPC Base Address: <tt> 0x0FDC</tt> */
#define MXC_R_MPC_PIDR0                    ((uint32_t)0x00000FE0UL) /**< Offset from MPC Base Address: <tt> 0x0FE0</tt> */
#define MXC_R_MPC_PIDR1                    ((uint32_t)0x00000FE4UL) /**< Offset from MPC Base Address: <tt> 0x0FE4</tt> */
#define MXC_R_MPC_PIDR2                    ((uint32_t)0x00000FE8UL) /**< Offset from MPC Base Address: <tt> 0x0FE8</tt> */
#define MXC_R_MPC_PIDR3                    ((uint32_t)0x00000FECUL) /**< Offset from MPC Base Address: <tt> 0x0FEC</tt> */
#define MXC_R_MPC_CIDR0                    ((uint32_t)0x00000FF0UL) /**< Offset from MPC Base Address: <tt> 0x0FF0</tt> */
#define MXC_R_MPC_CIDR1                    ((uint32_t)0x00000FF4UL) /**< Offset from MPC Base Address: <tt> 0x0FF4</tt> */
#define MXC_R_MPC_CIDR2                    ((uint32_t)0x00000FF8UL) /**< Offset from MPC Base Address: <tt> 0x0FF8</tt> */
#define MXC_R_MPC_CIDR3                    ((uint32_t)0x00000FFCUL) /**< Offset from MPC Base Address: <tt> 0x0FFC</tt> */
/**@} end of group mpc_registers */

/**
 * @ingroup  mpc_registers
 * @defgroup MPC_CTRL MPC_CTRL
 * @brief    Control register.
 * @{
 */
#define MXC_F_MPC_CTRL_CFG_SEC_RESP_POS                4 /**< CTRL_CFG_SEC_RESP Position */
#define MXC_F_MPC_CTRL_CFG_SEC_RESP                    ((uint32_t)(0x1UL << MXC_F_MPC_CTRL_CFG_SEC_RESP_POS)) /**< CTRL_CFG_SEC_RESP Mask */

#define MXC_F_MPC_CTRL_DATA_IF_GATE_REQ_POS            6 /**< CTRL_DATA_IF_GATE_REQ Position */
#define MXC_F_MPC_CTRL_DATA_IF_GATE_REQ                ((uint32_t)(0x1UL << MXC_F_MPC_CTRL_DATA_IF_GATE_REQ_POS)) /**< CTRL_DATA_IF_GATE_REQ Mask */

#define MXC_F_MPC_CTRL_DATA_IF_GATE_ACK_POS            7 /**< CTRL_DATA_IF_GATE_ACK Position */
#define MXC_F_MPC_CTRL_DATA_IF_GATE_ACK                ((uint32_t)(0x1UL << MXC_F_MPC_CTRL_DATA_IF_GATE_ACK_POS)) /**< CTRL_DATA_IF_GATE_ACK Mask */

#define MXC_F_MPC_CTRL_AUTOINCREMENT_POS               8 /**< CTRL_AUTOINCREMENT Position */
#define MXC_F_MPC_CTRL_AUTOINCREMENT                   ((uint32_t)(0x1UL << MXC_F_MPC_CTRL_AUTOINCREMENT_POS)) /**< CTRL_AUTOINCREMENT Mask */

#define MXC_F_MPC_CTRL_SEC_LOCKDOWN_POS                31 /**< CTRL_SEC_LOCKDOWN Position */
#define MXC_F_MPC_CTRL_SEC_LOCKDOWN                    ((uint32_t)(0x1UL << MXC_F_MPC_CTRL_SEC_LOCKDOWN_POS)) /**< CTRL_SEC_LOCKDOWN Mask */

/**@} end of group MPC_CTRL_Register */

/**
 * @ingroup  mpc_registers
 * @defgroup MPC_BLK_MAX MPC_BLK_MAX
 * @brief    Maximum value of block based index register.
 * @{
 */
#define MXC_F_MPC_BLK_MAX_IND_MAX_POS                  0 /**< BLK_MAX_IND_MAX Position */
#define MXC_F_MPC_BLK_MAX_IND_MAX                      ((uint32_t)(0xFFFFFFFFUL << MXC_F_MPC_BLK_MAX_IND_MAX_POS)) /**< BLK_MAX_IND_MAX Mask */

/**@} end of group MPC_BLK_MAX_Register */

/**
 * @ingroup  mpc_registers
 * @defgroup MPC_BLK_CFG MPC_BLK_CFG
 * @brief    Block configuration.
 * @{
 */
#define MXC_F_MPC_BLK_CFG_BLOCK_SIZE_POS               0 /**< BLK_CFG_BLOCK_SIZE Position */
#define MXC_F_MPC_BLK_CFG_BLOCK_SIZE                   ((uint32_t)(0xFUL << MXC_F_MPC_BLK_CFG_BLOCK_SIZE_POS)) /**< BLK_CFG_BLOCK_SIZE Mask */

#define MXC_F_MPC_BLK_CFG_INIT_POS                     31 /**< BLK_CFG_INIT Position */
#define MXC_F_MPC_BLK_CFG_INIT                         ((uint32_t)(0x1UL << MXC_F_MPC_BLK_CFG_INIT_POS)) /**< BLK_CFG_INIT Mask */

/**@} end of group MPC_BLK_CFG_Register */

/**
 * @ingroup  mpc_registers
 * @defgroup MPC_BLK_IDX MPC_BLK_IDX
 * @brief    Index value for accessing block based look up table.
 * @{
 */
#define MXC_F_MPC_BLK_IDX_INDEX_POS                    0 /**< BLK_IDX_INDEX Position */
#define MXC_F_MPC_BLK_IDX_INDEX                        ((uint32_t)(0xFFFFFFFFUL << MXC_F_MPC_BLK_IDX_INDEX_POS)) /**< BLK_IDX_INDEX Mask */

/**@} end of group MPC_BLK_IDX_Register */

/**
 * @ingroup  mpc_registers
 * @defgroup MPC_BLK_LUTN MPC_BLK_LUTN
 * @brief    Block based LUT. Access to block based look up configuration space pointed to by
 *           BLK_IDX.
 * @{
 */
#define MXC_F_MPC_BLK_LUTN_BLOCK_POS                   0 /**< BLK_LUTN_BLOCK Position */
#define MXC_F_MPC_BLK_LUTN_BLOCK                       ((uint32_t)(0xFFFFFFFFUL << MXC_F_MPC_BLK_LUTN_BLOCK_POS)) /**< BLK_LUTN_BLOCK Mask */
#define MXC_V_MPC_BLK_LUTN_BLOCK_SECURE                ((uint32_t)0x0UL) /**< BLK_LUTN_BLOCK_SECURE Value */
#define MXC_S_MPC_BLK_LUTN_BLOCK_SECURE                (MXC_V_MPC_BLK_LUTN_BLOCK_SECURE << MXC_F_MPC_BLK_LUTN_BLOCK_POS) /**< BLK_LUTN_BLOCK_SECURE Setting */
#define MXC_V_MPC_BLK_LUTN_BLOCK_NONSECURE             ((uint32_t)0x1UL) /**< BLK_LUTN_BLOCK_NONSECURE Value */
#define MXC_S_MPC_BLK_LUTN_BLOCK_NONSECURE             (MXC_V_MPC_BLK_LUTN_BLOCK_NONSECURE << MXC_F_MPC_BLK_LUTN_BLOCK_POS) /**< BLK_LUTN_BLOCK_NONSECURE Setting */

/**@} end of group MPC_BLK_LUTN_Register */

/**
 * @ingroup  mpc_registers
 * @defgroup MPC_INT_STAT MPC_INT_STAT
 * @brief    Interrupt status register.
 * @{
 */
#define MXC_F_MPC_INT_STAT_MPC_IRQ_FL_POS              0 /**< INT_STAT_MPC_IRQ_FL Position */
#define MXC_F_MPC_INT_STAT_MPC_IRQ_FL                  ((uint32_t)(0x1UL << MXC_F_MPC_INT_STAT_MPC_IRQ_FL_POS)) /**< INT_STAT_MPC_IRQ_FL Mask */

/**@} end of group MPC_INT_STAT_Register */

/**
 * @ingroup  mpc_registers
 * @defgroup MPC_INT_CLEAR MPC_INT_CLEAR
 * @brief    Interrupt enable register.
 * @{
 */
#define MXC_F_MPC_INT_CLEAR_MPC_IRQ_CLR_POS            0 /**< INT_CLEAR_MPC_IRQ_CLR Position */
#define MXC_F_MPC_INT_CLEAR_MPC_IRQ_CLR                ((uint32_t)(0x1UL << MXC_F_MPC_INT_CLEAR_MPC_IRQ_CLR_POS)) /**< INT_CLEAR_MPC_IRQ_CLR Mask */

/**@} end of group MPC_INT_CLEAR_Register */

/**
 * @ingroup  mpc_registers
 * @defgroup MPC_INT_EN MPC_INT_EN
 * @brief    Interrupt enable register.
 * @{
 */
#define MXC_F_MPC_INT_EN_MPC_IRQ_EN_POS                0 /**< INT_EN_MPC_IRQ_EN Position */
#define MXC_F_MPC_INT_EN_MPC_IRQ_EN                    ((uint32_t)(0x1UL << MXC_F_MPC_INT_EN_MPC_IRQ_EN_POS)) /**< INT_EN_MPC_IRQ_EN Mask */

/**@} end of group MPC_INT_EN_Register */

/**
 * @ingroup  mpc_registers
 * @defgroup MPC_INT_INFO1 MPC_INT_INFO1
 * @brief    First security violating address. Subsequent security violationg transfers are
 *           not captured in this register until mpc_irq is cleared.
 * @{
 */
#define MXC_F_MPC_INT_INFO1_HADDR_POS                  0 /**< INT_INFO1_HADDR Position */
#define MXC_F_MPC_INT_INFO1_HADDR                      ((uint32_t)(0xFFFFFFFFUL << MXC_F_MPC_INT_INFO1_HADDR_POS)) /**< INT_INFO1_HADDR Mask */

/**@} end of group MPC_INT_INFO1_Register */

/**
 * @ingroup  mpc_registers
 * @defgroup MPC_INT_INFO2 MPC_INT_INFO2
 * @brief    Additional control bits of the first security violating address.
 * @{
 */
#define MXC_F_MPC_INT_INFO2_HMASTER_POS                0 /**< INT_INFO2_HMASTER Position */
#define MXC_F_MPC_INT_INFO2_HMASTER                    ((uint32_t)(0xFFFFUL << MXC_F_MPC_INT_INFO2_HMASTER_POS)) /**< INT_INFO2_HMASTER Mask */

#define MXC_F_MPC_INT_INFO2_HNONSEC_POS                16 /**< INT_INFO2_HNONSEC Position */
#define MXC_F_MPC_INT_INFO2_HNONSEC                    ((uint32_t)(0x1UL << MXC_F_MPC_INT_INFO2_HNONSEC_POS)) /**< INT_INFO2_HNONSEC Mask */

#define MXC_F_MPC_INT_INFO2_CFG_NS_POS                 17 /**< INT_INFO2_CFG_NS Position */
#define MXC_F_MPC_INT_INFO2_CFG_NS                     ((uint32_t)(0x1UL << MXC_F_MPC_INT_INFO2_CFG_NS_POS)) /**< INT_INFO2_CFG_NS Mask */

/**@} end of group MPC_INT_INFO2_Register */

/**
 * @ingroup  mpc_registers
 * @defgroup MPC_INT_SET MPC_INT_SET
 * @brief    Interrupt setting.
 * @{
 */
#define MXC_F_MPC_INT_SET_MPC_IRQ_POS                  0 /**< INT_SET_MPC_IRQ Position */
#define MXC_F_MPC_INT_SET_MPC_IRQ                      ((uint32_t)(0x1UL << MXC_F_MPC_INT_SET_MPC_IRQ_POS)) /**< INT_SET_MPC_IRQ Mask */

/**@} end of group MPC_INT_SET_Register */

/**
 * @ingroup  mpc_registers
 * @defgroup MPC_PIDR4 MPC_PIDR4
 * @brief    Peripheral ID 4.
 * @{
 */
#define MXC_F_MPC_PIDR4_JEP106_C_CODE_POS              0 /**< PIDR4_JEP106_C_CODE Position */
#define MXC_F_MPC_PIDR4_JEP106_C_CODE                  ((uint32_t)(0xFUL << MXC_F_MPC_PIDR4_JEP106_C_CODE_POS)) /**< PIDR4_JEP106_C_CODE Mask */

#define MXC_F_MPC_PIDR4_BLOCK_COUNT_POS                4 /**< PIDR4_BLOCK_COUNT Position */
#define MXC_F_MPC_PIDR4_BLOCK_COUNT                    ((uint32_t)(0xFUL << MXC_F_MPC_PIDR4_BLOCK_COUNT_POS)) /**< PIDR4_BLOCK_COUNT Mask */

/**@} end of group MPC_PIDR4_Register */

/**
 * @ingroup  mpc_registers
 * @defgroup MPC_PIDR5 MPC_PIDR5
 * @brief    Peripheral ID 5.
 * @{
 */
#define MXC_F_MPC_PIDR5_PID5_POS                       0 /**< PIDR5_PID5 Position */
#define MXC_F_MPC_PIDR5_PID5                           ((uint32_t)(0xFFFFFFFFUL << MXC_F_MPC_PIDR5_PID5_POS)) /**< PIDR5_PID5 Mask */

/**@} end of group MPC_PIDR5_Register */

/**
 * @ingroup  mpc_registers
 * @defgroup MPC_PIDR6 MPC_PIDR6
 * @brief    Peripheral ID 6.
 * @{
 */
#define MXC_F_MPC_PIDR6_PID6_POS                       0 /**< PIDR6_PID6 Position */
#define MXC_F_MPC_PIDR6_PID6                           ((uint32_t)(0xFFFFFFFFUL << MXC_F_MPC_PIDR6_PID6_POS)) /**< PIDR6_PID6 Mask */

/**@} end of group MPC_PIDR6_Register */

/**
 * @ingroup  mpc_registers
 * @defgroup MPC_PIDR7 MPC_PIDR7
 * @brief    Peripheral ID 7.
 * @{
 */
#define MXC_F_MPC_PIDR7_PID7_POS                       0 /**< PIDR7_PID7 Position */
#define MXC_F_MPC_PIDR7_PID7                           ((uint32_t)(0xFFFFFFFFUL << MXC_F_MPC_PIDR7_PID7_POS)) /**< PIDR7_PID7 Mask */

/**@} end of group MPC_PIDR7_Register */

/**
 * @ingroup  mpc_registers
 * @defgroup MPC_PIDR0 MPC_PIDR0
 * @brief    Peripheral ID 0.
 * @{
 */
#define MXC_F_MPC_PIDR0_PART_NUMBER_POS                0 /**< PIDR0_PART_NUMBER Position */
#define MXC_F_MPC_PIDR0_PART_NUMBER                    ((uint32_t)(0xFFUL << MXC_F_MPC_PIDR0_PART_NUMBER_POS)) /**< PIDR0_PART_NUMBER Mask */

/**@} end of group MPC_PIDR0_Register */

/**
 * @ingroup  mpc_registers
 * @defgroup MPC_PIDR1 MPC_PIDR1
 * @brief    Peripheral ID 1.
 * @{
 */
#define MXC_F_MPC_PIDR1_PART_NUMBER_POS                0 /**< PIDR1_PART_NUMBER Position */
#define MXC_F_MPC_PIDR1_PART_NUMBER                    ((uint32_t)(0xFUL << MXC_F_MPC_PIDR1_PART_NUMBER_POS)) /**< PIDR1_PART_NUMBER Mask */

#define MXC_F_MPC_PIDR1_JEP106_ID_3_0_POS              4 /**< PIDR1_JEP106_ID_3_0 Position */
#define MXC_F_MPC_PIDR1_JEP106_ID_3_0                  ((uint32_t)(0xFUL << MXC_F_MPC_PIDR1_JEP106_ID_3_0_POS)) /**< PIDR1_JEP106_ID_3_0 Mask */

/**@} end of group MPC_PIDR1_Register */

/**
 * @ingroup  mpc_registers
 * @defgroup MPC_PIDR2 MPC_PIDR2
 * @brief    Peripheral ID 2.
 * @{
 */
#define MXC_F_MPC_PIDR2_JEP106_ID_6_4_POS              0 /**< PIDR2_JEP106_ID_6_4 Position */
#define MXC_F_MPC_PIDR2_JEP106_ID_6_4                  ((uint32_t)(0x7UL << MXC_F_MPC_PIDR2_JEP106_ID_6_4_POS)) /**< PIDR2_JEP106_ID_6_4 Mask */

#define MXC_F_MPC_PIDR2_JEDEC_USED_POS                 3 /**< PIDR2_JEDEC_USED Position */
#define MXC_F_MPC_PIDR2_JEDEC_USED                     ((uint32_t)(0x1UL << MXC_F_MPC_PIDR2_JEDEC_USED_POS)) /**< PIDR2_JEDEC_USED Mask */

#define MXC_F_MPC_PIDR2_REVISION_POS                   4 /**< PIDR2_REVISION Position */
#define MXC_F_MPC_PIDR2_REVISION                       ((uint32_t)(0xFUL << MXC_F_MPC_PIDR2_REVISION_POS)) /**< PIDR2_REVISION Mask */

/**@} end of group MPC_PIDR2_Register */

/**
 * @ingroup  mpc_registers
 * @defgroup MPC_PIDR3 MPC_PIDR3
 * @brief    Peripheral ID 3.
 * @{
 */
#define MXC_F_MPC_PIDR3_CUSTOMER_MOD_NO_POS            0 /**< PIDR3_CUSTOMER_MOD_NO Position */
#define MXC_F_MPC_PIDR3_CUSTOMER_MOD_NO                ((uint32_t)(0xFUL << MXC_F_MPC_PIDR3_CUSTOMER_MOD_NO_POS)) /**< PIDR3_CUSTOMER_MOD_NO Mask */

#define MXC_F_MPC_PIDR3_ECO_REV_POS                    4 /**< PIDR3_ECO_REV Position */
#define MXC_F_MPC_PIDR3_ECO_REV                        ((uint32_t)(0xFUL << MXC_F_MPC_PIDR3_ECO_REV_POS)) /**< PIDR3_ECO_REV Mask */

/**@} end of group MPC_PIDR3_Register */

/**
 * @ingroup  mpc_registers
 * @defgroup MPC_CIDR0 MPC_CIDR0
 * @brief    Component ID 0.
 * @{
 */
#define MXC_F_MPC_CIDR0_CID0_POS                       0 /**< CIDR0_CID0 Position */
#define MXC_F_MPC_CIDR0_CID0                           ((uint32_t)(0xFFFFFFFFUL << MXC_F_MPC_CIDR0_CID0_POS)) /**< CIDR0_CID0 Mask */

/**@} end of group MPC_CIDR0_Register */

/**
 * @ingroup  mpc_registers
 * @defgroup MPC_CIDR1 MPC_CIDR1
 * @brief    Component ID 1.
 * @{
 */
#define MXC_F_MPC_CIDR1_CID1_POS                       0 /**< CIDR1_CID1 Position */
#define MXC_F_MPC_CIDR1_CID1                           ((uint32_t)(0xFFFFFFFFUL << MXC_F_MPC_CIDR1_CID1_POS)) /**< CIDR1_CID1 Mask */

/**@} end of group MPC_CIDR1_Register */

/**
 * @ingroup  mpc_registers
 * @defgroup MPC_CIDR2 MPC_CIDR2
 * @brief    Component ID 2.
 * @{
 */
#define MXC_F_MPC_CIDR2_CID2_POS                       0 /**< CIDR2_CID2 Position */
#define MXC_F_MPC_CIDR2_CID2                           ((uint32_t)(0xFFFFFFFFUL << MXC_F_MPC_CIDR2_CID2_POS)) /**< CIDR2_CID2 Mask */

/**@} end of group MPC_CIDR2_Register */

/**
 * @ingroup  mpc_registers
 * @defgroup MPC_CIDR3 MPC_CIDR3
 * @brief    Component ID 3.
 * @{
 */
#define MXC_F_MPC_CIDR3_CID3_POS                       0 /**< CIDR3_CID3 Position */
#define MXC_F_MPC_CIDR3_CID3                           ((uint32_t)(0xFFFFFFFFUL << MXC_F_MPC_CIDR3_CID3_POS)) /**< CIDR3_CID3 Mask */

/**@} end of group MPC_CIDR3_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_MPC_REGS_H_
