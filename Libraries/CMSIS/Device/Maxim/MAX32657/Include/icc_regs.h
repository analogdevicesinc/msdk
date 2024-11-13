/**
 * @file    icc_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the ICC Peripheral Module.
 * @note    This file is @generated.
 * @ingroup icc_registers
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_ICC_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_ICC_REGS_H_

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
 * @ingroup     icc
 * @defgroup    icc_registers ICC_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the ICC Peripheral Module.
 * @details     Instruction Cache Controller Registers
 */

/**
 * @ingroup icc_registers
 * Structure type to access the ICC Registers.
 */
typedef struct {
    __IO uint32_t lbound;               /**< <tt>\b 0x0000:</tt> ICC LBOUND Register */
    __IO uint32_t hbound;               /**< <tt>\b 0x004:</tt> ICC HBOUND Register */
} mxc_icc_reg_regs_t;

typedef struct {
    __I  uint32_t info;                 /**< <tt>\b 0x0000:</tt> ICC INFO Register */
    __I  uint32_t sz;                   /**< <tt>\b 0x0004:</tt> ICC SZ Register */
    __R  uint32_t rsv_0x8_0xff[62];
    __IO uint32_t ctrl;                 /**< <tt>\b 0x0100:</tt> ICC CTRL Register */
    __R  uint32_t rsv_0x104_0x1ff[63];
    __IO uint32_t way;                  /**< <tt>\b 0x0200:</tt> ICC WAY Register */
    __IO uint32_t regctrl;              /**< <tt>\b 0x0204:</tt> ICC REGCTRL Register */
    __IO mxc_icc_reg_regs_t region[4];  /**< <tt>\b 0x0208:</tt> ICC REGION Register */
    __R  uint32_t rsv_0x228_0x2ff[54];
    __IO uint32_t pfmctrl;              /**< <tt>\b 0x0300:</tt> ICC PFMCTRL Register */
    __IO uint32_t pfmcnt;               /**< <tt>\b 0x0304:</tt> ICC PFMCNT Register */
    __R  uint32_t rsv_0x308_0x6ff[254];
    __IO uint32_t invalidate;           /**< <tt>\b 0x0700:</tt> ICC INVALIDATE Register */
} mxc_icc_regs_t;

/* Register offsets for module ICC */
/**
 * @ingroup    icc_registers
 * @defgroup   ICC_Register_Offsets Register Offsets
 * @brief      ICC Peripheral Register Offsets from the ICC Base Peripheral Address.
 * @{
 */
#define MXC_R_ICC_LBOUND                   ((uint32_t)0x00000000UL) /**< Offset from ICC Base Address: <tt> 0x0000</tt> */
#define MXC_R_ICC_HBOUND                   ((uint32_t)0x00000004UL) /**< Offset from ICC Base Address: <tt> 0x0004</tt> */
#define MXC_R_ICC_INFO                     ((uint32_t)0x00000000UL) /**< Offset from ICC Base Address: <tt> 0x0000</tt> */
#define MXC_R_ICC_SZ                       ((uint32_t)0x00000004UL) /**< Offset from ICC Base Address: <tt> 0x0004</tt> */
#define MXC_R_ICC_CTRL                     ((uint32_t)0x00000100UL) /**< Offset from ICC Base Address: <tt> 0x0100</tt> */
#define MXC_R_ICC_WAY                      ((uint32_t)0x00000200UL) /**< Offset from ICC Base Address: <tt> 0x0200</tt> */
#define MXC_R_ICC_REGCTRL                  ((uint32_t)0x00000204UL) /**< Offset from ICC Base Address: <tt> 0x0204</tt> */
#define MXC_R_ICC_REGION                   ((uint32_t)0x00000208UL) /**< Offset from ICC Base Address: <tt> 0x0208</tt> */
#define MXC_R_ICC_PFMCTRL                  ((uint32_t)0x00000300UL) /**< Offset from ICC Base Address: <tt> 0x0300</tt> */
#define MXC_R_ICC_PFMCNT                   ((uint32_t)0x00000304UL) /**< Offset from ICC Base Address: <tt> 0x0304</tt> */
#define MXC_R_ICC_INVALIDATE               ((uint32_t)0x00000700UL) /**< Offset from ICC Base Address: <tt> 0x0700</tt> */
/**@} end of group icc_registers */

/**
 * @ingroup  icc_registers
 * @defgroup ICC_INFO ICC_INFO
 * @brief    Cache ID Register.
 * @{
 */
#define MXC_F_ICC_INFO_RELNUM_POS                      0 /**< INFO_RELNUM Position */
#define MXC_F_ICC_INFO_RELNUM                          ((uint32_t)(0x3FUL << MXC_F_ICC_INFO_RELNUM_POS)) /**< INFO_RELNUM Mask */

#define MXC_F_ICC_INFO_PARTNUM_POS                     6 /**< INFO_PARTNUM Position */
#define MXC_F_ICC_INFO_PARTNUM                         ((uint32_t)(0xFUL << MXC_F_ICC_INFO_PARTNUM_POS)) /**< INFO_PARTNUM Mask */

#define MXC_F_ICC_INFO_ID_POS                          10 /**< INFO_ID Position */
#define MXC_F_ICC_INFO_ID                              ((uint32_t)(0x3FUL << MXC_F_ICC_INFO_ID_POS)) /**< INFO_ID Mask */

/**@} end of group ICC_INFO_Register */

/**
 * @ingroup  icc_registers
 * @defgroup ICC_SZ ICC_SZ
 * @brief    Memory Configuration Register.
 * @{
 */
#define MXC_F_ICC_SZ_CCH_POS                           0 /**< SZ_CCH Position */
#define MXC_F_ICC_SZ_CCH                               ((uint32_t)(0xFFFFUL << MXC_F_ICC_SZ_CCH_POS)) /**< SZ_CCH Mask */

#define MXC_F_ICC_SZ_MEM_POS                           16 /**< SZ_MEM Position */
#define MXC_F_ICC_SZ_MEM                               ((uint32_t)(0xFFFFUL << MXC_F_ICC_SZ_MEM_POS)) /**< SZ_MEM Mask */

/**@} end of group ICC_SZ_Register */

/**
 * @ingroup  icc_registers
 * @defgroup ICC_CTRL ICC_CTRL
 * @brief    Cache Control and Status Register.
 * @{
 */
#define MXC_F_ICC_CTRL_EN_POS                          0 /**< CTRL_EN Position */
#define MXC_F_ICC_CTRL_EN                              ((uint32_t)(0x1UL << MXC_F_ICC_CTRL_EN_POS)) /**< CTRL_EN Mask */

#define MXC_F_ICC_CTRL_RDY_POS                         16 /**< CTRL_RDY Position */
#define MXC_F_ICC_CTRL_RDY                             ((uint32_t)(0x1UL << MXC_F_ICC_CTRL_RDY_POS)) /**< CTRL_RDY Mask */

/**@} end of group ICC_CTRL_Register */

/**
 * @ingroup  icc_registers
 * @defgroup ICC_WAY ICC_WAY
 * @brief    Cache Way Control Register.
 * @{
 */
#define MXC_F_ICC_WAY_WAY_POS                          0 /**< WAY_WAY Position */
#define MXC_F_ICC_WAY_WAY                              ((uint32_t)(0x7UL << MXC_F_ICC_WAY_WAY_POS)) /**< WAY_WAY Mask */
#define MXC_V_ICC_WAY_WAY_1                            ((uint32_t)0x1UL) /**< WAY_WAY_1 Value */
#define MXC_S_ICC_WAY_WAY_1                            (MXC_V_ICC_WAY_WAY_1 << MXC_F_ICC_WAY_WAY_POS) /**< WAY_WAY_1 Setting */
#define MXC_V_ICC_WAY_WAY_2                            ((uint32_t)0x2UL) /**< WAY_WAY_2 Value */
#define MXC_S_ICC_WAY_WAY_2                            (MXC_V_ICC_WAY_WAY_2 << MXC_F_ICC_WAY_WAY_POS) /**< WAY_WAY_2 Setting */
#define MXC_V_ICC_WAY_WAY_4                            ((uint32_t)0x4UL) /**< WAY_WAY_4 Value */
#define MXC_S_ICC_WAY_WAY_4                            (MXC_V_ICC_WAY_WAY_4 << MXC_F_ICC_WAY_WAY_POS) /**< WAY_WAY_4 Setting */

/**@} end of group ICC_WAY_Register */

/**
 * @ingroup  icc_registers
 * @defgroup ICC_REGCTRL ICC_REGCTRL
 * @brief    Regional Control Register.
 * @{
 */
#define MXC_F_ICC_REGCTRL_EN_POS                       0 /**< REGCTRL_EN Position */
#define MXC_F_ICC_REGCTRL_EN                           ((uint32_t)(0xFFUL << MXC_F_ICC_REGCTRL_EN_POS)) /**< REGCTRL_EN Mask */

#define MXC_F_ICC_REGCTRL_EXC_POS                      8 /**< REGCTRL_EXC Position */
#define MXC_F_ICC_REGCTRL_EXC                          ((uint32_t)(0xFFUL << MXC_F_ICC_REGCTRL_EXC_POS)) /**< REGCTRL_EXC Mask */

/**@} end of group ICC_REGCTRL_Register */

/**
 * @ingroup  icc_registers
 * @defgroup ICC_PFMCTRL ICC_PFMCTRL
 * @brief    Performance Control Register.
 * @{
 */
#define MXC_F_ICC_PFMCTRL_EN_POS                       0 /**< PFMCTRL_EN Position */
#define MXC_F_ICC_PFMCTRL_EN                           ((uint32_t)(0x1UL << MXC_F_ICC_PFMCTRL_EN_POS)) /**< PFMCTRL_EN Mask */

/**@} end of group ICC_PFMCTRL_Register */

/**
 * @ingroup  icc_registers
 * @defgroup ICC_PFMCNT ICC_PFMCNT
 * @brief    Performance Counter Register.
 * @{
 */
#define MXC_F_ICC_PFMCNT_CNT_POS                       0 /**< PFMCNT_CNT Position */
#define MXC_F_ICC_PFMCNT_CNT                           ((uint32_t)(0xFFFFFFFFUL << MXC_F_ICC_PFMCNT_CNT_POS)) /**< PFMCNT_CNT Mask */

/**@} end of group ICC_PFMCNT_Register */

/**
 * @ingroup  icc_registers
 * @defgroup ICC_INVALIDATE ICC_INVALIDATE
 * @brief    Invalidate All Registers.
 * @{
 */
#define MXC_F_ICC_INVALIDATE_INVALID_POS               0 /**< INVALIDATE_INVALID Position */
#define MXC_F_ICC_INVALIDATE_INVALID                   ((uint32_t)(0xFFFFFFFFUL << MXC_F_ICC_INVALIDATE_INVALID_POS)) /**< INVALIDATE_INVALID Mask */

/**@} end of group ICC_INVALIDATE_Register */

/**
 * @ingroup  icc_registers
 * @defgroup ICC_LBOUND ICC_LBOUND
 * @brief    Regional Low Bound Register.
 * @{
 */
#define MXC_F_ICC_REG_LBOUND_BOUND_POS                 0 /**< REG_LBOUND_BOUND Position */
#define MXC_F_ICC_REG_LBOUND_BOUND                     ((uint32_t)(0xFFFFFFFFUL << MXC_F_ICC_REG_LBOUND_BOUND_POS)) /**< REG_LBOUND_BOUND Mask */

/**@} end of group ICC_REG_LBOUND_Register */

/**
 * @ingroup  icc_registers
 * @defgroup ICC_HBOUND ICC_HBOUND
 * @brief    DMA Channel Status Register.
 * @{
 */
#define MXC_F_ICC_REG_HBOUND_BOUND_POS                 0 /**< REG_HBOUND_BOUND Position */
#define MXC_F_ICC_REG_HBOUND_BOUND                     ((uint32_t)(0xFFFFFFFFUL << MXC_F_ICC_REG_HBOUND_BOUND_POS)) /**< REG_HBOUND_BOUND Mask */

/**@} end of group ICC_REG_HBOUND_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_ICC_REGS_H_
