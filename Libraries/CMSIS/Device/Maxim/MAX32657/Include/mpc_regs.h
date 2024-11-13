/**
 * @file    mpc_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the MPC Peripheral Module.
 * @note    This file is @generated.
 * @ingroup mpc_registers
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
#ifdef __cplusplus
#define __I volatile
#else
#define __I volatile const
#endif
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
 * @details     Memory Protection Controller.
 */

/**
 * @ingroup mpc_registers
 * Structure type to access the MPC Registers.
 */
typedef struct {
    __IO uint32_t ctrl;                 /**< <tt>\b 0x0000:</tt> MPC CTRL Register */
    __R  uint32_t rsv_0x4_0xf[3];
    __I  uint32_t blk_max;              /**< <tt>\b 0x0010:</tt> MPC BLK_MAX Register */
    __I  uint32_t blk_cfg;              /**< <tt>\b 0x0014:</tt> MPC BLK_CFG Register */
    __IO uint32_t blk_idx;              /**< <tt>\b 0x0018:</tt> MPC BLK_IDX Register */
    __IO uint32_t blk_lut;              /**< <tt>\b 0x001C:</tt> MPC BLK_LUT Register */
    __I  uint32_t int_stat;             /**< <tt>\b 0x0020:</tt> MPC INT_STAT Register */
    __O  uint32_t int_clear;            /**< <tt>\b 0x0024:</tt> MPC INT_CLEAR Register */
    __IO uint32_t int_en;               /**< <tt>\b 0x0028:</tt> MPC INT_EN Register */
    __I  uint32_t int_info1;            /**< <tt>\b 0x002C:</tt> MPC INT_INFO1 Register */
    __I  uint32_t int_info2;            /**< <tt>\b 0x0030:</tt> MPC INT_INFO2 Register */
    __O  uint32_t int_set;              /**< <tt>\b 0x0034:</tt> MPC INT_SET Register */
    __R  uint32_t rsv_0x38_0xfcf[998];
    __I  uint32_t pidr4;                /**< <tt>\b 0x0FD0:</tt> MPC PIDR4 Register */
    __I  uint32_t pidr5;                /**< <tt>\b 0x0FD4:</tt> MPC PIDR5 Register */
    __I  uint32_t pidr6;                /**< <tt>\b 0x0FD8:</tt> MPC PIDR6 Register */
    __I  uint32_t pidr7;                /**< <tt>\b 0x0FDC:</tt> MPC PIDR7 Register */
    __I  uint32_t pidr0;                /**< <tt>\b 0x0FE0:</tt> MPC PIDR0 Register */
    __I  uint32_t pidr1;                /**< <tt>\b 0x0FE4:</tt> MPC PIDR1 Register */
    __I  uint32_t pidr2;                /**< <tt>\b 0x0FE8:</tt> MPC PIDR2 Register */
    __I  uint32_t pidr3;                /**< <tt>\b 0x0FEC:</tt> MPC PIDR3 Register */
    __I  uint32_t cidr0;                /**< <tt>\b 0x0FF0:</tt> MPC CIDR0 Register */
    __I  uint32_t cidr1;                /**< <tt>\b 0x0FF4:</tt> MPC CIDR1 Register */
    __I  uint32_t cidr2;                /**< <tt>\b 0x0FF8:</tt> MPC CIDR2 Register */
    __I  uint32_t cidr3;                /**< <tt>\b 0x0FFC:</tt> MPC CIDR3 Register */
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
#define MXC_R_MPC_BLK_LUT                  ((uint32_t)0x0000001CUL) /**< Offset from MPC Base Address: <tt> 0x001C</tt> */
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
 * @brief    Control Register.
 * @{
 */
#define MXC_F_MPC_CTRL_SEC_ERR_POS                     4 /**< CTRL_SEC_ERR Position */
#define MXC_F_MPC_CTRL_SEC_ERR                         ((uint32_t)(0x1UL << MXC_F_MPC_CTRL_SEC_ERR_POS)) /**< CTRL_SEC_ERR Mask */

#define MXC_F_MPC_CTRL_DATAIF_REQ_POS                  6 /**< CTRL_DATAIF_REQ Position */
#define MXC_F_MPC_CTRL_DATAIF_REQ                      ((uint32_t)(0x1UL << MXC_F_MPC_CTRL_DATAIF_REQ_POS)) /**< CTRL_DATAIF_REQ Mask */

#define MXC_F_MPC_CTRL_DATAIF_ACK_POS                  7 /**< CTRL_DATAIF_ACK Position */
#define MXC_F_MPC_CTRL_DATAIF_ACK                      ((uint32_t)(0x1UL << MXC_F_MPC_CTRL_DATAIF_ACK_POS)) /**< CTRL_DATAIF_ACK Mask */

#define MXC_F_MPC_CTRL_AUTO_INC_POS                    8 /**< CTRL_AUTO_INC Position */
#define MXC_F_MPC_CTRL_AUTO_INC                        ((uint32_t)(0x1UL << MXC_F_MPC_CTRL_AUTO_INC_POS)) /**< CTRL_AUTO_INC Mask */

#define MXC_F_MPC_CTRL_SEC_LOCKDOWN_POS                31 /**< CTRL_SEC_LOCKDOWN Position */
#define MXC_F_MPC_CTRL_SEC_LOCKDOWN                    ((uint32_t)(0x1UL << MXC_F_MPC_CTRL_SEC_LOCKDOWN_POS)) /**< CTRL_SEC_LOCKDOWN Mask */

/**@} end of group MPC_CTRL_Register */

/**
 * @ingroup  mpc_registers
 * @defgroup MPC_BLK_MAX MPC_BLK_MAX
 * @brief    Maximum value of block-based index register.
 * @{
 */
#define MXC_F_MPC_BLK_MAX_VAL_POS                      0 /**< BLK_MAX_VAL Position */
#define MXC_F_MPC_BLK_MAX_VAL                          ((uint32_t)(0xFFFFFFFFUL << MXC_F_MPC_BLK_MAX_VAL_POS)) /**< BLK_MAX_VAL Mask */

/**@} end of group MPC_BLK_MAX_Register */

/**
 * @ingroup  mpc_registers
 * @defgroup MPC_BLK_CFG MPC_BLK_CFG
 * @brief    Block Control Register.
 * @{
 */
#define MXC_F_MPC_BLK_CFG_SIZE_POS                     0 /**< BLK_CFG_SIZE Position */
#define MXC_F_MPC_BLK_CFG_SIZE                         ((uint32_t)(0xFUL << MXC_F_MPC_BLK_CFG_SIZE_POS)) /**< BLK_CFG_SIZE Mask */

#define MXC_F_MPC_BLK_CFG_INIT_ST_POS                  31 /**< BLK_CFG_INIT_ST Position */
#define MXC_F_MPC_BLK_CFG_INIT_ST                      ((uint32_t)(0x1UL << MXC_F_MPC_BLK_CFG_INIT_ST_POS)) /**< BLK_CFG_INIT_ST Mask */

/**@} end of group MPC_BLK_CFG_Register */

/**
 * @ingroup  mpc_registers
 * @defgroup MPC_BLK_IDX MPC_BLK_IDX
 * @brief    Block Index Register.
 * @{
 */
#define MXC_F_MPC_BLK_IDX_IDX_POS                      0 /**< BLK_IDX_IDX Position */
#define MXC_F_MPC_BLK_IDX_IDX                          ((uint32_t)(0xFFFFFFFFUL << MXC_F_MPC_BLK_IDX_IDX_POS)) /**< BLK_IDX_IDX Mask */

/**@} end of group MPC_BLK_IDX_Register */

/**
 * @ingroup  mpc_registers
 * @defgroup MPC_BLK_LUT MPC_BLK_LUT
 * @brief    Block-based gating Look Up Table Register.
 * @{
 */
#define MXC_F_MPC_BLK_LUT_ACCESS_POS                   0 /**< BLK_LUT_ACCESS Position */
#define MXC_F_MPC_BLK_LUT_ACCESS                       ((uint32_t)(0xFFFFFFFFUL << MXC_F_MPC_BLK_LUT_ACCESS_POS)) /**< BLK_LUT_ACCESS Mask */

/**@} end of group MPC_BLK_LUT_Register */

/**
 * @ingroup  mpc_registers
 * @defgroup MPC_INT_STAT MPC_INT_STAT
 * @brief    Interrupt Flag Register.
 * @{
 */
#define MXC_F_MPC_INT_STAT_MPC_IRQ_POS                 0 /**< INT_STAT_MPC_IRQ Position */
#define MXC_F_MPC_INT_STAT_MPC_IRQ                     ((uint32_t)(0x1UL << MXC_F_MPC_INT_STAT_MPC_IRQ_POS)) /**< INT_STAT_MPC_IRQ Mask */

/**@} end of group MPC_INT_STAT_Register */

/**
 * @ingroup  mpc_registers
 * @defgroup MPC_INT_CLEAR MPC_INT_CLEAR
 * @brief    Interrupt Clear Register.
 * @{
 */
#define MXC_F_MPC_INT_CLEAR_MPC_IRQ_POS                0 /**< INT_CLEAR_MPC_IRQ Position */
#define MXC_F_MPC_INT_CLEAR_MPC_IRQ                    ((uint32_t)(0x1UL << MXC_F_MPC_INT_CLEAR_MPC_IRQ_POS)) /**< INT_CLEAR_MPC_IRQ Mask */

/**@} end of group MPC_INT_CLEAR_Register */

/**
 * @ingroup  mpc_registers
 * @defgroup MPC_INT_EN MPC_INT_EN
 * @brief    Interrupt Enable Register.
 * @{
 */
#define MXC_F_MPC_INT_EN_MPC_IRQ_POS                   0 /**< INT_EN_MPC_IRQ Position */
#define MXC_F_MPC_INT_EN_MPC_IRQ                       ((uint32_t)(0x1UL << MXC_F_MPC_INT_EN_MPC_IRQ_POS)) /**< INT_EN_MPC_IRQ Mask */

/**@} end of group MPC_INT_EN_Register */

/**
 * @ingroup  mpc_registers
 * @defgroup MPC_INT_INFO1 MPC_INT_INFO1
 * @brief    Interrupt Info 1 Register.
 * @{
 */
#define MXC_F_MPC_INT_INFO1_HADDR_POS                  0 /**< INT_INFO1_HADDR Position */
#define MXC_F_MPC_INT_INFO1_HADDR                      ((uint32_t)(0xFFFFFFFFUL << MXC_F_MPC_INT_INFO1_HADDR_POS)) /**< INT_INFO1_HADDR Mask */

/**@} end of group MPC_INT_INFO1_Register */

/**
 * @ingroup  mpc_registers
 * @defgroup MPC_INT_INFO2 MPC_INT_INFO2
 * @brief    Interrupt Info 2 Register.
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
 * @brief    Interrupt Set Debug Register.
 * @{
 */
#define MXC_F_MPC_INT_SET_MPC_IRQ_POS                  0 /**< INT_SET_MPC_IRQ Position */
#define MXC_F_MPC_INT_SET_MPC_IRQ                      ((uint32_t)(0x1UL << MXC_F_MPC_INT_SET_MPC_IRQ_POS)) /**< INT_SET_MPC_IRQ Mask */

/**@} end of group MPC_INT_SET_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_MPC_REGS_H_
