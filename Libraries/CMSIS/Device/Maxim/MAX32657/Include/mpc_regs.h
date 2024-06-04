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
 * @details     Memory Protection Controller.
 */

/**
 * @ingroup mpc_registers
 * Structure type to access the MPC Registers.
 */
typedef struct {
    __IO uint32_t ctrl;                 /**< <tt>\b 0x0000:</tt> MPC CTRL Register */
    __R  uint32_t rsv_0x4_0xf[3];
    __I  uint32_t blkmax;               /**< <tt>\b 0x0010:</tt> MPC BLKMAX Register */
    __I  uint32_t blkctrl;              /**< <tt>\b 0x0014:</tt> MPC BLKCTRL Register */
    __IO uint32_t blkidx;               /**< <tt>\b 0x0018:</tt> MPC BLKIDX Register */
    __IO uint32_t blklut;               /**< <tt>\b 0x001C:</tt> MPC BLKLUT Register */
    __I  uint32_t intfl;                /**< <tt>\b 0x0020:</tt> MPC INTFL Register */
    __O  uint32_t intclr;               /**< <tt>\b 0x0024:</tt> MPC INTCLR Register */
    __IO uint32_t inten;                /**< <tt>\b 0x0028:</tt> MPC INTEN Register */
    __I  uint32_t intinfo0;             /**< <tt>\b 0x002C:</tt> MPC INTINFO0 Register */
    __I  uint32_t intinfo1;             /**< <tt>\b 0x0030:</tt> MPC INTINFO1 Register */
    __O  uint32_t intset;               /**< <tt>\b 0x0034:</tt> MPC INTSET Register */
    __R  uint32_t rsv_0x38_0xfcf[998];
    __I  uint32_t periphid4;            /**< <tt>\b 0x0FD0:</tt> MPC PERIPHID4 Register */
    __R  uint32_t rsv_0xfd4_0xfdf[3];
    __I  uint32_t periphid0;            /**< <tt>\b 0x0FE0:</tt> MPC PERIPHID0 Register */
    __I  uint32_t periphid1;            /**< <tt>\b 0x0FE4:</tt> MPC PERIPHID1 Register */
    __I  uint32_t periphid2;            /**< <tt>\b 0x0FE8:</tt> MPC PERIPHID2 Register */
    __I  uint32_t periphid3;            /**< <tt>\b 0x0FEC:</tt> MPC PERIPHID3 Register */
    __I  uint32_t compid[4];            /**< <tt>\b 0x0FF0:</tt> MPC COMPID Register */
} mxc_mpc_regs_t;

/* Register offsets for module MPC */
/**
 * @ingroup    mpc_registers
 * @defgroup   MPC_Register_Offsets Register Offsets
 * @brief      MPC Peripheral Register Offsets from the MPC Base Peripheral Address.
 * @{
 */
#define MXC_R_MPC_CTRL                     ((uint32_t)0x00000000UL) /**< Offset from MPC Base Address: <tt> 0x0000</tt> */
#define MXC_R_MPC_BLKMAX                   ((uint32_t)0x00000010UL) /**< Offset from MPC Base Address: <tt> 0x0010</tt> */
#define MXC_R_MPC_BLKCTRL                  ((uint32_t)0x00000014UL) /**< Offset from MPC Base Address: <tt> 0x0014</tt> */
#define MXC_R_MPC_BLKIDX                   ((uint32_t)0x00000018UL) /**< Offset from MPC Base Address: <tt> 0x0018</tt> */
#define MXC_R_MPC_BLKLUT                   ((uint32_t)0x0000001CUL) /**< Offset from MPC Base Address: <tt> 0x001C</tt> */
#define MXC_R_MPC_INTFL                    ((uint32_t)0x00000020UL) /**< Offset from MPC Base Address: <tt> 0x0020</tt> */
#define MXC_R_MPC_INTCLR                   ((uint32_t)0x00000024UL) /**< Offset from MPC Base Address: <tt> 0x0024</tt> */
#define MXC_R_MPC_INTEN                    ((uint32_t)0x00000028UL) /**< Offset from MPC Base Address: <tt> 0x0028</tt> */
#define MXC_R_MPC_INTINFO0                 ((uint32_t)0x0000002CUL) /**< Offset from MPC Base Address: <tt> 0x002C</tt> */
#define MXC_R_MPC_INTINFO1                 ((uint32_t)0x00000030UL) /**< Offset from MPC Base Address: <tt> 0x0030</tt> */
#define MXC_R_MPC_INTSET                   ((uint32_t)0x00000034UL) /**< Offset from MPC Base Address: <tt> 0x0034</tt> */
#define MXC_R_MPC_PERIPHID4                ((uint32_t)0x00000FD0UL) /**< Offset from MPC Base Address: <tt> 0x0FD0</tt> */
#define MXC_R_MPC_PERIPHID0                ((uint32_t)0x00000FE0UL) /**< Offset from MPC Base Address: <tt> 0x0FE0</tt> */
#define MXC_R_MPC_PERIPHID1                ((uint32_t)0x00000FE4UL) /**< Offset from MPC Base Address: <tt> 0x0FE4</tt> */
#define MXC_R_MPC_PERIPHID2                ((uint32_t)0x00000FE8UL) /**< Offset from MPC Base Address: <tt> 0x0FE8</tt> */
#define MXC_R_MPC_PERIPHID3                ((uint32_t)0x00000FECUL) /**< Offset from MPC Base Address: <tt> 0x0FEC</tt> */
#define MXC_R_MPC_COMPID                   ((uint32_t)0x00000FF0UL) /**< Offset from MPC Base Address: <tt> 0x0FF0</tt> */
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
 * @defgroup MPC_BLKMAX MPC_BLKMAX
 * @brief    Maximum value of block-based index register.
 * @{
 */
#define MXC_F_MPC_BLKMAX_VAL_POS                       0 /**< BLKMAX_VAL Position */
#define MXC_F_MPC_BLKMAX_VAL                           ((uint32_t)(0xFFFFFFFFUL << MXC_F_MPC_BLKMAX_VAL_POS)) /**< BLKMAX_VAL Mask */

/**@} end of group MPC_BLKMAX_Register */

/**
 * @ingroup  mpc_registers
 * @defgroup MPC_BLKCTRL MPC_BLKCTRL
 * @brief    Block Control Register.
 * @{
 */
#define MXC_F_MPC_BLKCTRL_SIZE_POS                     0 /**< BLKCTRL_SIZE Position */
#define MXC_F_MPC_BLKCTRL_SIZE                         ((uint32_t)(0xFUL << MXC_F_MPC_BLKCTRL_SIZE_POS)) /**< BLKCTRL_SIZE Mask */

#define MXC_F_MPC_BLKCTRL_INIT_ST_POS                  31 /**< BLKCTRL_INIT_ST Position */
#define MXC_F_MPC_BLKCTRL_INIT_ST                      ((uint32_t)(0x1UL << MXC_F_MPC_BLKCTRL_INIT_ST_POS)) /**< BLKCTRL_INIT_ST Mask */

/**@} end of group MPC_BLKCTRL_Register */

/**
 * @ingroup  mpc_registers
 * @defgroup MPC_BLKIDX MPC_BLKIDX
 * @brief    Block Index Register.
 * @{
 */
#define MXC_F_MPC_BLKIDX_IDX_POS                       0 /**< BLKIDX_IDX Position */
#define MXC_F_MPC_BLKIDX_IDX                           ((uint32_t)(0xFFFFFFFFUL << MXC_F_MPC_BLKIDX_IDX_POS)) /**< BLKIDX_IDX Mask */

/**@} end of group MPC_BLKIDX_Register */

/**
 * @ingroup  mpc_registers
 * @defgroup MPC_BLKLUT MPC_BLKLUT
 * @brief    Block-based gating Look Up Table Register.
 * @{
 */
#define MXC_F_MPC_BLKLUT_ACCESS_POS                    0 /**< BLKLUT_ACCESS Position */
#define MXC_F_MPC_BLKLUT_ACCESS                        ((uint32_t)(0xFFFFFFFFUL << MXC_F_MPC_BLKLUT_ACCESS_POS)) /**< BLKLUT_ACCESS Mask */

/**@} end of group MPC_BLKLUT_Register */

/**
 * @ingroup  mpc_registers
 * @defgroup MPC_INTFL MPC_INTFL
 * @brief    Interrupt Flag Register.
 * @{
 */
#define MXC_F_MPC_INTFL_MPC_POS                        0 /**< INTFL_MPC Position */
#define MXC_F_MPC_INTFL_MPC                            ((uint32_t)(0x1UL << MXC_F_MPC_INTFL_MPC_POS)) /**< INTFL_MPC Mask */

/**@} end of group MPC_INTFL_Register */

/**
 * @ingroup  mpc_registers
 * @defgroup MPC_INTCLR MPC_INTCLR
 * @brief    Interrupt Clear Register.
 * @{
 */
#define MXC_F_MPC_INTCLR_MPC_POS                       0 /**< INTCLR_MPC Position */
#define MXC_F_MPC_INTCLR_MPC                           ((uint32_t)(0x1UL << MXC_F_MPC_INTCLR_MPC_POS)) /**< INTCLR_MPC Mask */

/**@} end of group MPC_INTCLR_Register */

/**
 * @ingroup  mpc_registers
 * @defgroup MPC_INTEN MPC_INTEN
 * @brief    Interrupt Enable Register.
 * @{
 */
#define MXC_F_MPC_INTEN_MPC_POS                        0 /**< INTEN_MPC Position */
#define MXC_F_MPC_INTEN_MPC                            ((uint32_t)(0x1UL << MXC_F_MPC_INTEN_MPC_POS)) /**< INTEN_MPC Mask */

/**@} end of group MPC_INTEN_Register */

/**
 * @ingroup  mpc_registers
 * @defgroup MPC_INTINFO0 MPC_INTINFO0
 * @brief    Interrupt Info 0 Register.
 * @{
 */
#define MXC_F_MPC_INTINFO0_HADDR_POS                   0 /**< INTINFO0_HADDR Position */
#define MXC_F_MPC_INTINFO0_HADDR                       ((uint32_t)(0xFFFFFFFFUL << MXC_F_MPC_INTINFO0_HADDR_POS)) /**< INTINFO0_HADDR Mask */

/**@} end of group MPC_INTINFO0_Register */

/**
 * @ingroup  mpc_registers
 * @defgroup MPC_INTINFO1 MPC_INTINFO1
 * @brief    Interrupt Info 1 Register.
 * @{
 */
#define MXC_F_MPC_INTINFO1_HMASTER_POS                 0 /**< INTINFO1_HMASTER Position */
#define MXC_F_MPC_INTINFO1_HMASTER                     ((uint32_t)(0xFFFFUL << MXC_F_MPC_INTINFO1_HMASTER_POS)) /**< INTINFO1_HMASTER Mask */

#define MXC_F_MPC_INTINFO1_HNONSEC_POS                 16 /**< INTINFO1_HNONSEC Position */
#define MXC_F_MPC_INTINFO1_HNONSEC                     ((uint32_t)(0x1UL << MXC_F_MPC_INTINFO1_HNONSEC_POS)) /**< INTINFO1_HNONSEC Mask */

#define MXC_F_MPC_INTINFO1_CFG_NS_POS                  17 /**< INTINFO1_CFG_NS Position */
#define MXC_F_MPC_INTINFO1_CFG_NS                      ((uint32_t)(0x1UL << MXC_F_MPC_INTINFO1_CFG_NS_POS)) /**< INTINFO1_CFG_NS Mask */

/**@} end of group MPC_INTINFO1_Register */

/**
 * @ingroup  mpc_registers
 * @defgroup MPC_INTSET MPC_INTSET
 * @brief    Interrupt Set Debug Register.
 * @{
 */
#define MXC_F_MPC_INTSET_MPC_POS                       0 /**< INTSET_MPC Position */
#define MXC_F_MPC_INTSET_MPC                           ((uint32_t)(0xFFFFUL << MXC_F_MPC_INTSET_MPC_POS)) /**< INTSET_MPC Mask */

/**@} end of group MPC_INTSET_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_MPC_REGS_H_
