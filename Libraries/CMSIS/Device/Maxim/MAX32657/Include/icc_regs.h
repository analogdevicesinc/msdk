/**
 * @file    icc_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the ICC Peripheral Module.
 * @note    This file is @generated.
 * @ingroup icc_registers
 */

/******************************************************************************
 *
 * Analog Devices, Inc.),
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
    __IO uint32_t region_lbound;        /**< <tt>\b 0x0:</tt> ICC REGION_LBOUND Register */
    __IO uint32_t region_hbound;        /**< <tt>\b 0x4:</tt> ICC REGION_HBOUND Register */
} mxc_icc_region_regs_t;

typedef struct {
    __I  uint32_t cache_id;             /**< <tt>\b 0x0000:</tt> ICC CACHE_ID Register */
    __I  uint32_t memcfg;               /**< <tt>\b 0x0004:</tt> ICC MEMCFG Register */
    __R  uint32_t rsv_0x8_0xff[62];
    __IO uint32_t cache_ctrl;           /**< <tt>\b 0x0100:</tt> ICC CACHE_CTRL Register */
    __R  uint32_t rsv_0x104_0x1ff[63];
    __IO uint32_t cache_way;            /**< <tt>\b 0x0200:</tt> ICC CACHE_WAY Register */
    __IO uint32_t region_ctrl;          /**< <tt>\b 0x0204:</tt> ICC REGION_CTRL Register */
    __IO uint32_t region[15];           /**< <tt>\b 0x0208:</tt> ICC REGION Register */
    __R  uint32_t rsv_0x244_0x2ff[47];
    __IO uint32_t pfm_ctrl;             /**< <tt>\b 0x0300:</tt> ICC PFM_CTRL Register */
    __IO uint32_t pfm_cntr;             /**< <tt>\b 0x0304:</tt> ICC PFM_CNTR Register */
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
#define MXC_R_ICC_REGION_LBOUND            ((uint32_t)0x00000000UL) /**< Offset from ICC Base Address: <tt> 0x0000</tt> */
#define MXC_R_ICC_REGION_HBOUND            ((uint32_t)0x00000004UL) /**< Offset from ICC Base Address: <tt> 0x0004</tt> */
#define MXC_R_ICC_CACHE_ID                 ((uint32_t)0x00000000UL) /**< Offset from ICC Base Address: <tt> 0x0000</tt> */
#define MXC_R_ICC_MEMCFG                   ((uint32_t)0x00000004UL) /**< Offset from ICC Base Address: <tt> 0x0004</tt> */
#define MXC_R_ICC_CACHE_CTRL               ((uint32_t)0x00000100UL) /**< Offset from ICC Base Address: <tt> 0x0100</tt> */
#define MXC_R_ICC_CACHE_WAY                ((uint32_t)0x00000200UL) /**< Offset from ICC Base Address: <tt> 0x0200</tt> */
#define MXC_R_ICC_REGION_CTRL              ((uint32_t)0x00000204UL) /**< Offset from ICC Base Address: <tt> 0x0204</tt> */
#define MXC_R_ICC_REGION                   ((uint32_t)0x00000208UL) /**< Offset from ICC Base Address: <tt> 0x0208</tt> */
#define MXC_R_ICC_PFM_CTRL                 ((uint32_t)0x00000300UL) /**< Offset from ICC Base Address: <tt> 0x0300</tt> */
#define MXC_R_ICC_PFM_CNTR                 ((uint32_t)0x00000304UL) /**< Offset from ICC Base Address: <tt> 0x0304</tt> */
#define MXC_R_ICC_INVALIDATE               ((uint32_t)0x00000700UL) /**< Offset from ICC Base Address: <tt> 0x0700</tt> */
/**@} end of group icc_registers */

/**
 * @ingroup  icc_registers
 * @defgroup ICC_CACHE_ID ICC_CACHE_ID
 * @brief    Cache ID Register.
 * @{
 */
#define MXC_F_ICC_CACHE_ID_RELNUM_POS                  0 /**< CACHE_ID_RELNUM Position */
#define MXC_F_ICC_CACHE_ID_RELNUM                      ((uint32_t)(0x3FUL << MXC_F_ICC_CACHE_ID_RELNUM_POS)) /**< CACHE_ID_RELNUM Mask */

#define MXC_F_ICC_CACHE_ID_PARTNUM_POS                 6 /**< CACHE_ID_PARTNUM Position */
#define MXC_F_ICC_CACHE_ID_PARTNUM                     ((uint32_t)(0xFUL << MXC_F_ICC_CACHE_ID_PARTNUM_POS)) /**< CACHE_ID_PARTNUM Mask */

#define MXC_F_ICC_CACHE_ID_CCHID_POS                   10 /**< CACHE_ID_CCHID Position */
#define MXC_F_ICC_CACHE_ID_CCHID                       ((uint32_t)(0x3FUL << MXC_F_ICC_CACHE_ID_CCHID_POS)) /**< CACHE_ID_CCHID Mask */

/**@} end of group ICC_CACHE_ID_Register */

/**
 * @ingroup  icc_registers
 * @defgroup ICC_MEMCFG ICC_MEMCFG
 * @brief    Memory Configuration Register.
 * @{
 */
#define MXC_F_ICC_MEMCFG_CCHSZ_POS                     0 /**< MEMCFG_CCHSZ Position */
#define MXC_F_ICC_MEMCFG_CCHSZ                         ((uint32_t)(0xFFFFUL << MXC_F_ICC_MEMCFG_CCHSZ_POS)) /**< MEMCFG_CCHSZ Mask */

#define MXC_F_ICC_MEMCFG_MEMSZ_POS                     16 /**< MEMCFG_MEMSZ Position */
#define MXC_F_ICC_MEMCFG_MEMSZ                         ((uint32_t)(0xFFFFUL << MXC_F_ICC_MEMCFG_MEMSZ_POS)) /**< MEMCFG_MEMSZ Mask */

/**@} end of group ICC_MEMCFG_Register */

/**
 * @ingroup  icc_registers
 * @defgroup ICC_CACHE_CTRL ICC_CACHE_CTRL
 * @brief    Cache Control and Status Register.
 * @{
 */
#define MXC_F_ICC_CACHE_CTRL_CACHE_EN_POS              0 /**< CACHE_CTRL_CACHE_EN Position */
#define MXC_F_ICC_CACHE_CTRL_CACHE_EN                  ((uint32_t)(0x1UL << MXC_F_ICC_CACHE_CTRL_CACHE_EN_POS)) /**< CACHE_CTRL_CACHE_EN Mask */

#define MXC_F_ICC_CACHE_CTRL_CACHE_RDY_POS             16 /**< CACHE_CTRL_CACHE_RDY Position */
#define MXC_F_ICC_CACHE_CTRL_CACHE_RDY                 ((uint32_t)(0x1UL << MXC_F_ICC_CACHE_CTRL_CACHE_RDY_POS)) /**< CACHE_CTRL_CACHE_RDY Mask */

/**@} end of group ICC_CACHE_CTRL_Register */

/**
 * @ingroup  icc_registers
 * @defgroup ICC_CACHE_WAY ICC_CACHE_WAY
 * @brief    Cache Way.
 * @{
 */
#define MXC_F_ICC_CACHE_WAY_CACHEWAY_POS               0 /**< CACHE_WAY_CACHEWAY Position */
#define MXC_F_ICC_CACHE_WAY_CACHEWAY                   ((uint32_t)(0x7UL << MXC_F_ICC_CACHE_WAY_CACHEWAY_POS)) /**< CACHE_WAY_CACHEWAY Mask */
#define MXC_V_ICC_CACHE_WAY_CACHEWAY_WAY1              ((uint32_t)0x1UL) /**< CACHE_WAY_CACHEWAY_WAY1 Value */
#define MXC_S_ICC_CACHE_WAY_CACHEWAY_WAY1              (MXC_V_ICC_CACHE_WAY_CACHEWAY_WAY1 << MXC_F_ICC_CACHE_WAY_CACHEWAY_POS) /**< CACHE_WAY_CACHEWAY_WAY1 Setting */
#define MXC_V_ICC_CACHE_WAY_CACHEWAY_WAY2              ((uint32_t)0x2UL) /**< CACHE_WAY_CACHEWAY_WAY2 Value */
#define MXC_S_ICC_CACHE_WAY_CACHEWAY_WAY2              (MXC_V_ICC_CACHE_WAY_CACHEWAY_WAY2 << MXC_F_ICC_CACHE_WAY_CACHEWAY_POS) /**< CACHE_WAY_CACHEWAY_WAY2 Setting */
#define MXC_V_ICC_CACHE_WAY_CACHEWAY_WAY4              ((uint32_t)0x4UL) /**< CACHE_WAY_CACHEWAY_WAY4 Value */
#define MXC_S_ICC_CACHE_WAY_CACHEWAY_WAY4              (MXC_V_ICC_CACHE_WAY_CACHEWAY_WAY4 << MXC_F_ICC_CACHE_WAY_CACHEWAY_POS) /**< CACHE_WAY_CACHEWAY_WAY4 Setting */

/**@} end of group ICC_CACHE_WAY_Register */

/**
 * @ingroup  icc_registers
 * @defgroup ICC_REGION_CTRL ICC_REGION_CTRL
 * @brief    Regional Control.
 * @{
 */
#define MXC_F_ICC_REGION_CTRL_REGIONAL_EN_POS          0 /**< REGION_CTRL_REGIONAL_EN Position */
#define MXC_F_ICC_REGION_CTRL_REGIONAL_EN              ((uint32_t)(0xFFUL << MXC_F_ICC_REGION_CTRL_REGIONAL_EN_POS)) /**< REGION_CTRL_REGIONAL_EN Mask */

#define MXC_F_ICC_REGION_CTRL_REGIONAL_EX_POS          16 /**< REGION_CTRL_REGIONAL_EX Position */
#define MXC_F_ICC_REGION_CTRL_REGIONAL_EX              ((uint32_t)(0xFFUL << MXC_F_ICC_REGION_CTRL_REGIONAL_EX_POS)) /**< REGION_CTRL_REGIONAL_EX Mask */

/**@} end of group ICC_REGION_CTRL_Register */

/**
 * @ingroup  icc_registers
 * @defgroup ICC_PFM_CTRL ICC_PFM_CTRL
 * @brief    Performance Control.
 * @{
 */
#define MXC_F_ICC_PFM_CTRL_PFM_EN_POS                  0 /**< PFM_CTRL_PFM_EN Position */
#define MXC_F_ICC_PFM_CTRL_PFM_EN                      ((uint32_t)(0x1UL << MXC_F_ICC_PFM_CTRL_PFM_EN_POS)) /**< PFM_CTRL_PFM_EN Mask */

/**@} end of group ICC_PFM_CTRL_Register */

/**
 * @ingroup  icc_registers
 * @defgroup ICC_PFM_CNTR ICC_PFM_CNTR
 * @brief    Performance Counter.
 * @{
 */
#define MXC_F_ICC_PFM_CNTR_PFM_CNTR_POS                0 /**< PFM_CNTR_PFM_CNTR Position */
#define MXC_F_ICC_PFM_CNTR_PFM_CNTR                    ((uint32_t)(0xFFFFFFFFUL << MXC_F_ICC_PFM_CNTR_PFM_CNTR_POS)) /**< PFM_CNTR_PFM_CNTR Mask */

/**@} end of group ICC_PFM_CNTR_Register */

/**
 * @ingroup  icc_registers
 * @defgroup ICC_INVALIDATE ICC_INVALIDATE
 * @brief    Invalidate All Registers.
 * @{
 */
#define MXC_F_ICC_INVALIDATE_IA_POS                    0 /**< INVALIDATE_IA Position */
#define MXC_F_ICC_INVALIDATE_IA                        ((uint32_t)(0xFFFFFFFFUL << MXC_F_ICC_INVALIDATE_IA_POS)) /**< INVALIDATE_IA Mask */

/**@} end of group ICC_INVALIDATE_Register */

/**
 * @ingroup  icc_registers
 * @defgroup ICC_REGION_LBOUND ICC_REGION_LBOUND
 * @brief    Regional Low Bound.
 * @{
 */
#define MXC_F_ICC_REGION_LBOUND_REGION_LBOUND_POS      0 /**< REGION_LBOUND_REGION_LBOUND Position */
#define MXC_F_ICC_REGION_LBOUND_REGION_LBOUND          ((uint32_t)(0xFFFFFFFFUL << MXC_F_ICC_REGION_LBOUND_REGION_LBOUND_POS)) /**< REGION_LBOUND_REGION_LBOUND Mask */

/**@} end of group ICC_REGION_LBOUND_Register */

/**
 * @ingroup  icc_registers
 * @defgroup ICC_REGION_HBOUND ICC_REGION_HBOUND
 * @brief    Regional High Bound.
 * @{
 */
#define MXC_F_ICC_REGION_HBOUND_REGION_HBOUND_POS      0 /**< REGION_HBOUND_REGION_HBOUND Position */
#define MXC_F_ICC_REGION_HBOUND_REGION_HBOUND          ((uint32_t)(0xFFFFFFFFUL << MXC_F_ICC_REGION_HBOUND_REGION_HBOUND_POS)) /**< REGION_HBOUND_REGION_HBOUND Mask */

/**@} end of group ICC_REGION_HBOUND_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_ICC_REGS_H_
