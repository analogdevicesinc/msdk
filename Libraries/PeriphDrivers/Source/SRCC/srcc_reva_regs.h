/**
 * @file    srcc_reva_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the SRCC_REVA Peripheral Module.
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

#ifndef _SRCC_REVA_REGS_H_
#define _SRCC_REVA_REGS_H_

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
 * @ingroup     srcc_reva
 * @defgroup    srcc_reva_registers SRCC_REVA_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the SRCC_REVA Peripheral Module.
 * @details SPIX Cache Controller Registers.
 */

/**
 * @ingroup srcc_reva_registers
 * Structure type to access the SRCC_REVA Registers.
 */
typedef struct {
    __I  uint32_t cache_id;             /**< <tt>\b 0x0000:</tt> SRCC_REVA CACHE_ID Register */
    __I  uint32_t memcfg;               /**< <tt>\b 0x0004:</tt> SRCC_REVA MEMCFG Register */
    __R  uint32_t rsv_0x8_0xff[62];
    __IO uint32_t cache_ctrl;           /**< <tt>\b 0x0100:</tt> SRCC_REVA CACHE_CTRL Register */
    __R  uint32_t rsv_0x104_0x6ff[383];
    __IO uint32_t invalidate;           /**< <tt>\b 0x0700:</tt> SRCC_REVA INVALIDATE Register */
} mxc_srcc_reva_regs_t;

/* Register offsets for module SRCC_REVA */
/**
 * @ingroup    srcc_reva_registers
 * @defgroup   SRCC_REVA_Register_Offsets Register Offsets
 * @brief      SRCC_REVA Peripheral Register Offsets from the SRCC_REVA Base Peripheral Address. 
 * @{
 */
 #define MXC_R_SRCC_REVA_CACHE_ID           ((uint32_t)0x00000000UL) /**< Offset from SRCC_REVA Base Address: <tt> 0x0000</tt> */ 
 #define MXC_R_SRCC_REVA_MEMCFG             ((uint32_t)0x00000004UL) /**< Offset from SRCC_REVA Base Address: <tt> 0x0004</tt> */ 
 #define MXC_R_SRCC_REVA_CACHE_CTRL         ((uint32_t)0x00000100UL) /**< Offset from SRCC_REVA Base Address: <tt> 0x0100</tt> */ 
 #define MXC_R_SRCC_REVA_INVALIDATE         ((uint32_t)0x00000700UL) /**< Offset from SRCC_REVA Base Address: <tt> 0x0700</tt> */ 
/**@} end of group srcc_reva_registers */

/**
 * @ingroup  srcc_reva_registers
 * @defgroup SRCC_REVA_CACHE_ID SRCC_REVA_CACHE_ID
 * @brief    Cache ID Register.
 * @{
 */
 #define MXC_F_SRCC_REVA_CACHE_ID_RELNUM_POS            0 /**< CACHE_ID_RELNUM Position */
 #define MXC_F_SRCC_REVA_CACHE_ID_RELNUM                ((uint32_t)(0x3FUL << MXC_F_SRCC_REVA_CACHE_ID_RELNUM_POS)) /**< CACHE_ID_RELNUM Mask */

 #define MXC_F_SRCC_REVA_CACHE_ID_PARTNUM_POS           6 /**< CACHE_ID_PARTNUM Position */
 #define MXC_F_SRCC_REVA_CACHE_ID_PARTNUM               ((uint32_t)(0xFUL << MXC_F_SRCC_REVA_CACHE_ID_PARTNUM_POS)) /**< CACHE_ID_PARTNUM Mask */

 #define MXC_F_SRCC_REVA_CACHE_ID_CCHID_POS             10 /**< CACHE_ID_CCHID Position */
 #define MXC_F_SRCC_REVA_CACHE_ID_CCHID                 ((uint32_t)(0x3FUL << MXC_F_SRCC_REVA_CACHE_ID_CCHID_POS)) /**< CACHE_ID_CCHID Mask */

/**@} end of group SRCC_REVA_CACHE_ID_Register */

/**
 * @ingroup  srcc_reva_registers
 * @defgroup SRCC_REVA_MEMCFG SRCC_REVA_MEMCFG
 * @brief    Memory Configuration Register.
 * @{
 */
 #define MXC_F_SRCC_REVA_MEMCFG_CCHSZ_POS               0 /**< MEMCFG_CCHSZ Position */
 #define MXC_F_SRCC_REVA_MEMCFG_CCHSZ                   ((uint32_t)(0xFFFFUL << MXC_F_SRCC_REVA_MEMCFG_CCHSZ_POS)) /**< MEMCFG_CCHSZ Mask */

 #define MXC_F_SRCC_REVA_MEMCFG_MEMSZ_POS               16 /**< MEMCFG_MEMSZ Position */
 #define MXC_F_SRCC_REVA_MEMCFG_MEMSZ                   ((uint32_t)(0xFFFFUL << MXC_F_SRCC_REVA_MEMCFG_MEMSZ_POS)) /**< MEMCFG_MEMSZ Mask */

/**@} end of group SRCC_REVA_MEMCFG_Register */

/**
 * @ingroup  srcc_reva_registers
 * @defgroup SRCC_REVA_CACHE_CTRL SRCC_REVA_CACHE_CTRL
 * @brief    Cache Control and Status Register.
 * @{
 */
 #define MXC_F_SRCC_REVA_CACHE_CTRL_CACHE_EN_POS        0 /**< CACHE_CTRL_CACHE_EN Position */
 #define MXC_F_SRCC_REVA_CACHE_CTRL_CACHE_EN            ((uint32_t)(0x1UL << MXC_F_SRCC_REVA_CACHE_CTRL_CACHE_EN_POS)) /**< CACHE_CTRL_CACHE_EN Mask */

 #define MXC_F_SRCC_REVA_CACHE_CTRL_WRITE_ALLOC_EN_POS  1 /**< CACHE_CTRL_WRITE_ALLOC_EN Position */
 #define MXC_F_SRCC_REVA_CACHE_CTRL_WRITE_ALLOC_EN      ((uint32_t)(0x1UL << MXC_F_SRCC_REVA_CACHE_CTRL_WRITE_ALLOC_EN_POS)) /**< CACHE_CTRL_WRITE_ALLOC_EN Mask */

 #define MXC_F_SRCC_REVA_CACHE_CTRL_CWFST_DIS_POS       2 /**< CACHE_CTRL_CWFST_DIS Position */
 #define MXC_F_SRCC_REVA_CACHE_CTRL_CWFST_DIS           ((uint32_t)(0x1UL << MXC_F_SRCC_REVA_CACHE_CTRL_CWFST_DIS_POS)) /**< CACHE_CTRL_CWFST_DIS Mask */

 #define MXC_F_SRCC_REVA_CACHE_CTRL_CACHE_RDY_POS       16 /**< CACHE_CTRL_CACHE_RDY Position */
 #define MXC_F_SRCC_REVA_CACHE_CTRL_CACHE_RDY           ((uint32_t)(0x1UL << MXC_F_SRCC_REVA_CACHE_CTRL_CACHE_RDY_POS)) /**< CACHE_CTRL_CACHE_RDY Mask */

/**@} end of group SRCC_REVA_CACHE_CTRL_Register */

/**
 * @ingroup  srcc_reva_registers
 * @defgroup SRCC_REVA_INVALIDATE SRCC_REVA_INVALIDATE
 * @brief    Invalidate All Cache Contents. Any time this register location is written
 *           (regardless of the data value), the cache controller immediately begins
 *           invalidating the entire contents of the cache memory. The cache will be in
 *           bypass mode until the invalidate operation is complete. System software can
 *           examine the Cache Ready bit (CACHE_CTRL.CACHE_RDY) to determine when the
 *           invalidate operation is complete. Note that it is not necessary to disable the
 *           cache controller prior to beginning this operation. Reads from this register
 *           always return 0.
 * @{
 */
 #define MXC_F_SRCC_REVA_INVALIDATE_IA_POS              0 /**< INVALIDATE_IA Position */
 #define MXC_F_SRCC_REVA_INVALIDATE_IA                  ((uint32_t)(0xFFFFFFFFUL << MXC_F_SRCC_REVA_INVALIDATE_IA_POS)) /**< INVALIDATE_IA Mask */

/**@} end of group SRCC_REVA_INVALIDATE_Register */

#ifdef __cplusplus
}
#endif

#endif /* _SRCC_REVA_REGS_H_ */
