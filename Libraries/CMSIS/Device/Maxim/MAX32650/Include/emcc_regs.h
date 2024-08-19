/**
 * @file    emcc_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the EMCC Peripheral Module.
 * @note    This file is @generated.
 * @ingroup emcc_registers
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32650_INCLUDE_EMCC_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32650_INCLUDE_EMCC_REGS_H_

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
 * @ingroup     emcc
 * @defgroup    emcc_registers EMCC_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the EMCC Peripheral Module.
 * @details     External Memory Cache Controller Registers.
 */

/**
 * @ingroup emcc_registers
 * Structure type to access the EMCC Registers.
 */
typedef struct {
    __I  uint32_t cache_id;             /**< <tt>\b 0x0000:</tt> EMCC CACHE_ID Register */
    __I  uint32_t mem_size;             /**< <tt>\b 0x0004:</tt> EMCC MEM_SIZE Register */
    __R  uint32_t rsv_0x8_0xff[62];
    __IO uint32_t cache_ctrl;           /**< <tt>\b 0x0100:</tt> EMCC CACHE_CTRL Register */
    __R  uint32_t rsv_0x104_0x6ff[383];
    __IO uint32_t invalidate;           /**< <tt>\b 0x0700:</tt> EMCC INVALIDATE Register */
} mxc_emcc_regs_t;

/* Register offsets for module EMCC */
/**
 * @ingroup    emcc_registers
 * @defgroup   EMCC_Register_Offsets Register Offsets
 * @brief      EMCC Peripheral Register Offsets from the EMCC Base Peripheral Address.
 * @{
 */
#define MXC_R_EMCC_CACHE_ID                ((uint32_t)0x00000000UL) /**< Offset from EMCC Base Address: <tt> 0x0000</tt> */
#define MXC_R_EMCC_MEM_SIZE                ((uint32_t)0x00000004UL) /**< Offset from EMCC Base Address: <tt> 0x0004</tt> */
#define MXC_R_EMCC_CACHE_CTRL              ((uint32_t)0x00000100UL) /**< Offset from EMCC Base Address: <tt> 0x0100</tt> */
#define MXC_R_EMCC_INVALIDATE              ((uint32_t)0x00000700UL) /**< Offset from EMCC Base Address: <tt> 0x0700</tt> */
/**@} end of group emcc_registers */

/**
 * @ingroup  emcc_registers
 * @defgroup EMCC_CACHE_ID EMCC_CACHE_ID
 * @brief    Cache ID Register.
 * @{
 */
#define MXC_F_EMCC_CACHE_ID_RELNUM_POS                 0 /**< CACHE_ID_RELNUM Position */
#define MXC_F_EMCC_CACHE_ID_RELNUM                     ((uint32_t)(0x3FUL << MXC_F_EMCC_CACHE_ID_RELNUM_POS)) /**< CACHE_ID_RELNUM Mask */

#define MXC_F_EMCC_CACHE_ID_PARTNUM_POS                6 /**< CACHE_ID_PARTNUM Position */
#define MXC_F_EMCC_CACHE_ID_PARTNUM                    ((uint32_t)(0xFUL << MXC_F_EMCC_CACHE_ID_PARTNUM_POS)) /**< CACHE_ID_PARTNUM Mask */

#define MXC_F_EMCC_CACHE_ID_CCHID_POS                  10 /**< CACHE_ID_CCHID Position */
#define MXC_F_EMCC_CACHE_ID_CCHID                      ((uint32_t)(0x3FUL << MXC_F_EMCC_CACHE_ID_CCHID_POS)) /**< CACHE_ID_CCHID Mask */

/**@} end of group EMCC_CACHE_ID_Register */

/**
 * @ingroup  emcc_registers
 * @defgroup EMCC_MEM_SIZE EMCC_MEM_SIZE
 * @brief    Memory Configuration Register.
 * @{
 */
#define MXC_F_EMCC_MEM_SIZE_CCHSZ_POS                  0 /**< MEM_SIZE_CCHSZ Position */
#define MXC_F_EMCC_MEM_SIZE_CCHSZ                      ((uint32_t)(0xFFFFUL << MXC_F_EMCC_MEM_SIZE_CCHSZ_POS)) /**< MEM_SIZE_CCHSZ Mask */

#define MXC_F_EMCC_MEM_SIZE_MEMSZ_POS                  16 /**< MEM_SIZE_MEMSZ Position */
#define MXC_F_EMCC_MEM_SIZE_MEMSZ                      ((uint32_t)(0xFFFFUL << MXC_F_EMCC_MEM_SIZE_MEMSZ_POS)) /**< MEM_SIZE_MEMSZ Mask */

/**@} end of group EMCC_MEM_SIZE_Register */

/**
 * @ingroup  emcc_registers
 * @defgroup EMCC_CACHE_CTRL EMCC_CACHE_CTRL
 * @brief    Cache Control and Status Register.
 * @{
 */
#define MXC_F_EMCC_CACHE_CTRL_ENABLE_POS               0 /**< CACHE_CTRL_ENABLE Position */
#define MXC_F_EMCC_CACHE_CTRL_ENABLE                   ((uint32_t)(0x1UL << MXC_F_EMCC_CACHE_CTRL_ENABLE_POS)) /**< CACHE_CTRL_ENABLE Mask */
#define MXC_V_EMCC_CACHE_CTRL_ENABLE_DIS               ((uint32_t)0x0UL) /**< CACHE_CTRL_ENABLE_DIS Value */
#define MXC_S_EMCC_CACHE_CTRL_ENABLE_DIS               (MXC_V_EMCC_CACHE_CTRL_ENABLE_DIS << MXC_F_EMCC_CACHE_CTRL_ENABLE_POS) /**< CACHE_CTRL_ENABLE_DIS Setting */
#define MXC_V_EMCC_CACHE_CTRL_ENABLE_EN                ((uint32_t)0x1UL) /**< CACHE_CTRL_ENABLE_EN Value */
#define MXC_S_EMCC_CACHE_CTRL_ENABLE_EN                (MXC_V_EMCC_CACHE_CTRL_ENABLE_EN << MXC_F_EMCC_CACHE_CTRL_ENABLE_POS) /**< CACHE_CTRL_ENABLE_EN Setting */

#define MXC_F_EMCC_CACHE_CTRL_WRITE_ALLOC_POS          1 /**< CACHE_CTRL_WRITE_ALLOC Position */
#define MXC_F_EMCC_CACHE_CTRL_WRITE_ALLOC              ((uint32_t)(0x1UL << MXC_F_EMCC_CACHE_CTRL_WRITE_ALLOC_POS)) /**< CACHE_CTRL_WRITE_ALLOC Mask */
#define MXC_V_EMCC_CACHE_CTRL_WRITE_ALLOC_DIS          ((uint32_t)0x0UL) /**< CACHE_CTRL_WRITE_ALLOC_DIS Value */
#define MXC_S_EMCC_CACHE_CTRL_WRITE_ALLOC_DIS          (MXC_V_EMCC_CACHE_CTRL_WRITE_ALLOC_DIS << MXC_F_EMCC_CACHE_CTRL_WRITE_ALLOC_POS) /**< CACHE_CTRL_WRITE_ALLOC_DIS Setting */
#define MXC_V_EMCC_CACHE_CTRL_WRITE_ALLOC_EN           ((uint32_t)0x1UL) /**< CACHE_CTRL_WRITE_ALLOC_EN Value */
#define MXC_S_EMCC_CACHE_CTRL_WRITE_ALLOC_EN           (MXC_V_EMCC_CACHE_CTRL_WRITE_ALLOC_EN << MXC_F_EMCC_CACHE_CTRL_WRITE_ALLOC_POS) /**< CACHE_CTRL_WRITE_ALLOC_EN Setting */

#define MXC_F_EMCC_CACHE_CTRL_CWFST_DIS_POS            2 /**< CACHE_CTRL_CWFST_DIS Position */
#define MXC_F_EMCC_CACHE_CTRL_CWFST_DIS                ((uint32_t)(0x1UL << MXC_F_EMCC_CACHE_CTRL_CWFST_DIS_POS)) /**< CACHE_CTRL_CWFST_DIS Mask */
#define MXC_V_EMCC_CACHE_CTRL_CWFST_DIS_DIS            ((uint32_t)0x1UL) /**< CACHE_CTRL_CWFST_DIS_DIS Value */
#define MXC_S_EMCC_CACHE_CTRL_CWFST_DIS_DIS            (MXC_V_EMCC_CACHE_CTRL_CWFST_DIS_DIS << MXC_F_EMCC_CACHE_CTRL_CWFST_DIS_POS) /**< CACHE_CTRL_CWFST_DIS_DIS Setting */
#define MXC_V_EMCC_CACHE_CTRL_CWFST_DIS_EN             ((uint32_t)0x0UL) /**< CACHE_CTRL_CWFST_DIS_EN Value */
#define MXC_S_EMCC_CACHE_CTRL_CWFST_DIS_EN             (MXC_V_EMCC_CACHE_CTRL_CWFST_DIS_EN << MXC_F_EMCC_CACHE_CTRL_CWFST_DIS_POS) /**< CACHE_CTRL_CWFST_DIS_EN Setting */

#define MXC_F_EMCC_CACHE_CTRL_READY_POS                16 /**< CACHE_CTRL_READY Position */
#define MXC_F_EMCC_CACHE_CTRL_READY                    ((uint32_t)(0x1UL << MXC_F_EMCC_CACHE_CTRL_READY_POS)) /**< CACHE_CTRL_READY Mask */
#define MXC_V_EMCC_CACHE_CTRL_READY_NOTREADY           ((uint32_t)0x0UL) /**< CACHE_CTRL_READY_NOTREADY Value */
#define MXC_S_EMCC_CACHE_CTRL_READY_NOTREADY           (MXC_V_EMCC_CACHE_CTRL_READY_NOTREADY << MXC_F_EMCC_CACHE_CTRL_READY_POS) /**< CACHE_CTRL_READY_NOTREADY Setting */
#define MXC_V_EMCC_CACHE_CTRL_READY_READY              ((uint32_t)0x1UL) /**< CACHE_CTRL_READY_READY Value */
#define MXC_S_EMCC_CACHE_CTRL_READY_READY              (MXC_V_EMCC_CACHE_CTRL_READY_READY << MXC_F_EMCC_CACHE_CTRL_READY_POS) /**< CACHE_CTRL_READY_READY Setting */

/**@} end of group EMCC_CACHE_CTRL_Register */

/**
 * @ingroup  emcc_registers
 * @defgroup EMCC_INVALIDATE EMCC_INVALIDATE
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
#define MXC_F_EMCC_INVALIDATE_IA_POS                   0 /**< INVALIDATE_IA Position */
#define MXC_F_EMCC_INVALIDATE_IA                       ((uint32_t)(0xFFFFFFFFUL << MXC_F_EMCC_INVALIDATE_IA_POS)) /**< INVALIDATE_IA Mask */

/**@} end of group EMCC_INVALIDATE_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32650_INCLUDE_EMCC_REGS_H_
