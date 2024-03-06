/**
 * @file    tpu_reva_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the TPU_REVA Peripheral Module.
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

#ifndef _TPU_REVA_REGS_H_
#define _TPU_REVA_REGS_H_

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
 * @ingroup     tpu_reva
 * @defgroup    tpu_reva_registers TPU_REVA_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the TPU_REVA Peripheral Module.
 * @details The Trust Protection Unit used to assist the computationally intensive operations of several common cryptographic algorithms.
 */

/**
 * @ingroup tpu_reva_registers
 * Structure type to access the TPU_REVA Registers.
 */
typedef struct {
    __IO uint32_t ctrl;                 /**< <tt>\b 0x00:</tt> TPU_REVA CTRL Register */
    __IO uint32_t cipher_ctrl;          /**< <tt>\b 0x04:</tt> TPU_REVA CIPHER_CTRL Register */
    __IO uint32_t hash_ctrl;            /**< <tt>\b 0x08:</tt> TPU_REVA HASH_CTRL Register */
    __IO uint32_t crc_ctrl;             /**< <tt>\b 0x0C:</tt> TPU_REVA CRC_CTRL Register */
    __IO uint32_t dma_src;              /**< <tt>\b 0x10:</tt> TPU_REVA DMA_SRC Register */
    __IO uint32_t dma_dst;              /**< <tt>\b 0x14:</tt> TPU_REVA DMA_DST Register */
    __IO uint32_t dma_cnt;              /**< <tt>\b 0x18:</tt> TPU_REVA DMA_CNT Register */
    __IO uint32_t maa_ctrl;             /**< <tt>\b 0x1C:</tt> TPU_REVA MAA_CTRL Register */
    __O  uint32_t data_in[4];           /**< <tt>\b 0x20:</tt> TPU_REVA DATA_IN Register */
    __I  uint32_t data_out[4];          /**< <tt>\b 0x30:</tt> TPU_REVA DATA_OUT Register */
    __IO uint32_t crc_poly;             /**< <tt>\b 0x40:</tt> TPU_REVA CRC_POLY Register */
    __IO uint32_t crc_val;              /**< <tt>\b 0x44:</tt> TPU_REVA CRC_VAL Register */
    __I  uint32_t crc_prng;             /**< <tt>\b 0x48:</tt> TPU_REVA CRC_PRNG Register */
    __IO uint32_t ham_ecc;              /**< <tt>\b 0x4C:</tt> TPU_REVA HAM_ECC Register */
    __IO uint32_t cipher_init[4];       /**< <tt>\b 0x50:</tt> TPU_REVA CIPHER_INIT Register */
    __O  uint32_t cipher_key[8];        /**< <tt>\b 0x60:</tt> TPU_REVA CIPHER_KEY Register */
    __IO uint32_t hash_digest[16];      /**< <tt>\b 0x80:</tt> TPU_REVA HASH_DIGEST Register */
    __IO uint32_t hash_msg_sz[4];       /**< <tt>\b 0xC0:</tt> TPU_REVA HASH_MSG_SZ Register */
    __IO uint32_t maa_maws;             /**< <tt>\b 0xD0:</tt> TPU_REVA MAA_MAWS Register */
} mxc_tpu_reva_regs_t;

/* Register offsets for module TPU_REVA */
/**
 * @ingroup    tpu_reva_registers
 * @defgroup   TPU_REVA_Register_Offsets Register Offsets
 * @brief      TPU_REVA Peripheral Register Offsets from the TPU_REVA Base Peripheral Address. 
 * @{
 */
 #define MXC_R_TPU_REVA_CTRL                ((uint32_t)0x00000000UL) /**< Offset from TPU_REVA Base Address: <tt> 0x0000</tt> */ 
 #define MXC_R_TPU_REVA_CIPHER_CTRL         ((uint32_t)0x00000004UL) /**< Offset from TPU_REVA Base Address: <tt> 0x0004</tt> */ 
 #define MXC_R_TPU_REVA_HASH_CTRL           ((uint32_t)0x00000008UL) /**< Offset from TPU_REVA Base Address: <tt> 0x0008</tt> */ 
 #define MXC_R_TPU_REVA_CRC_CTRL            ((uint32_t)0x0000000CUL) /**< Offset from TPU_REVA Base Address: <tt> 0x000C</tt> */ 
 #define MXC_R_TPU_REVA_DMA_SRC             ((uint32_t)0x00000010UL) /**< Offset from TPU_REVA Base Address: <tt> 0x0010</tt> */ 
 #define MXC_R_TPU_REVA_DMA_DST             ((uint32_t)0x00000014UL) /**< Offset from TPU_REVA Base Address: <tt> 0x0014</tt> */ 
 #define MXC_R_TPU_REVA_DMA_CNT             ((uint32_t)0x00000018UL) /**< Offset from TPU_REVA Base Address: <tt> 0x0018</tt> */ 
 #define MXC_R_TPU_REVA_MAA_CTRL            ((uint32_t)0x0000001CUL) /**< Offset from TPU_REVA Base Address: <tt> 0x001C</tt> */ 
 #define MXC_R_TPU_REVA_DATA_IN             ((uint32_t)0x00000020UL) /**< Offset from TPU_REVA Base Address: <tt> 0x0020</tt> */ 
 #define MXC_R_TPU_REVA_DATA_OUT            ((uint32_t)0x00000030UL) /**< Offset from TPU_REVA Base Address: <tt> 0x0030</tt> */ 
 #define MXC_R_TPU_REVA_CRC_POLY            ((uint32_t)0x00000040UL) /**< Offset from TPU_REVA Base Address: <tt> 0x0040</tt> */ 
 #define MXC_R_TPU_REVA_CRC_VAL             ((uint32_t)0x00000044UL) /**< Offset from TPU_REVA Base Address: <tt> 0x0044</tt> */ 
 #define MXC_R_TPU_REVA_CRC_PRNG            ((uint32_t)0x00000048UL) /**< Offset from TPU_REVA Base Address: <tt> 0x0048</tt> */ 
 #define MXC_R_TPU_REVA_HAM_ECC             ((uint32_t)0x0000004CUL) /**< Offset from TPU_REVA Base Address: <tt> 0x004C</tt> */ 
 #define MXC_R_TPU_REVA_CIPHER_INIT         ((uint32_t)0x00000050UL) /**< Offset from TPU_REVA Base Address: <tt> 0x0050</tt> */ 
 #define MXC_R_TPU_REVA_CIPHER_KEY          ((uint32_t)0x00000060UL) /**< Offset from TPU_REVA Base Address: <tt> 0x0060</tt> */ 
 #define MXC_R_TPU_REVA_HASH_DIGEST         ((uint32_t)0x00000080UL) /**< Offset from TPU_REVA Base Address: <tt> 0x0080</tt> */ 
 #define MXC_R_TPU_REVA_HASH_MSG_SZ         ((uint32_t)0x000000C0UL) /**< Offset from TPU_REVA Base Address: <tt> 0x00C0</tt> */ 
 #define MXC_R_TPU_REVA_MAA_MAWS            ((uint32_t)0x000000D0UL) /**< Offset from TPU_REVA Base Address: <tt> 0x00D0</tt> */ 
/**@} end of group tpu_reva_registers */

/**
 * @ingroup  tpu_reva_registers
 * @defgroup TPU_REVA_CTRL TPU_REVA_CTRL
 * @brief    Crypto Control Register.
 * @{
 */
 #define MXC_F_TPU_REVA_CTRL_RST_POS                    0 /**< CTRL_RST Position */
 #define MXC_F_TPU_REVA_CTRL_RST                        ((uint32_t)(0x1UL << MXC_F_TPU_REVA_CTRL_RST_POS)) /**< CTRL_RST Mask */

 #define MXC_F_TPU_REVA_CTRL_INT_POS                    1 /**< CTRL_INT Position */
 #define MXC_F_TPU_REVA_CTRL_INT                        ((uint32_t)(0x1UL << MXC_F_TPU_REVA_CTRL_INT_POS)) /**< CTRL_INT Mask */

 #define MXC_F_TPU_REVA_CTRL_SRC_POS                    2 /**< CTRL_SRC Position */
 #define MXC_F_TPU_REVA_CTRL_SRC                        ((uint32_t)(0x1UL << MXC_F_TPU_REVA_CTRL_SRC_POS)) /**< CTRL_SRC Mask */

 #define MXC_F_TPU_REVA_CTRL_BSO_POS                    4 /**< CTRL_BSO Position */
 #define MXC_F_TPU_REVA_CTRL_BSO                        ((uint32_t)(0x1UL << MXC_F_TPU_REVA_CTRL_BSO_POS)) /**< CTRL_BSO Mask */

 #define MXC_F_TPU_REVA_CTRL_BSI_POS                    5 /**< CTRL_BSI Position */
 #define MXC_F_TPU_REVA_CTRL_BSI                        ((uint32_t)(0x1UL << MXC_F_TPU_REVA_CTRL_BSI_POS)) /**< CTRL_BSI Mask */

 #define MXC_F_TPU_REVA_CTRL_WAIT_EN_POS                6 /**< CTRL_WAIT_EN Position */
 #define MXC_F_TPU_REVA_CTRL_WAIT_EN                    ((uint32_t)(0x1UL << MXC_F_TPU_REVA_CTRL_WAIT_EN_POS)) /**< CTRL_WAIT_EN Mask */

 #define MXC_F_TPU_REVA_CTRL_WAIT_POL_POS               7 /**< CTRL_WAIT_POL Position */
 #define MXC_F_TPU_REVA_CTRL_WAIT_POL                   ((uint32_t)(0x1UL << MXC_F_TPU_REVA_CTRL_WAIT_POL_POS)) /**< CTRL_WAIT_POL Mask */

 #define MXC_F_TPU_REVA_CTRL_WRSRC_POS                  8 /**< CTRL_WRSRC Position */
 #define MXC_F_TPU_REVA_CTRL_WRSRC                      ((uint32_t)(0x3UL << MXC_F_TPU_REVA_CTRL_WRSRC_POS)) /**< CTRL_WRSRC Mask */
 #define MXC_V_TPU_REVA_CTRL_WRSRC_NONE                 ((uint32_t)0x0UL) /**< CTRL_WRSRC_NONE Value */
 #define MXC_S_TPU_REVA_CTRL_WRSRC_NONE                 (MXC_V_TPU_REVA_CTRL_WRSRC_NONE << MXC_F_TPU_REVA_CTRL_WRSRC_POS) /**< CTRL_WRSRC_NONE Setting */
 #define MXC_V_TPU_REVA_CTRL_WRSRC_CIPHEROUTPUT         ((uint32_t)0x1UL) /**< CTRL_WRSRC_CIPHEROUTPUT Value */
 #define MXC_S_TPU_REVA_CTRL_WRSRC_CIPHEROUTPUT         (MXC_V_TPU_REVA_CTRL_WRSRC_CIPHEROUTPUT << MXC_F_TPU_REVA_CTRL_WRSRC_POS) /**< CTRL_WRSRC_CIPHEROUTPUT Setting */
 #define MXC_V_TPU_REVA_CTRL_WRSRC_READFIFO             ((uint32_t)0x2UL) /**< CTRL_WRSRC_READFIFO Value */
 #define MXC_S_TPU_REVA_CTRL_WRSRC_READFIFO             (MXC_V_TPU_REVA_CTRL_WRSRC_READFIFO << MXC_F_TPU_REVA_CTRL_WRSRC_POS) /**< CTRL_WRSRC_READFIFO Setting */

 #define MXC_F_TPU_REVA_CTRL_RDSRC_POS                  10 /**< CTRL_RDSRC Position */
 #define MXC_F_TPU_REVA_CTRL_RDSRC                      ((uint32_t)(0x3UL << MXC_F_TPU_REVA_CTRL_RDSRC_POS)) /**< CTRL_RDSRC Mask */
 #define MXC_V_TPU_REVA_CTRL_RDSRC_DMADISABLED          ((uint32_t)0x0UL) /**< CTRL_RDSRC_DMADISABLED Value */
 #define MXC_S_TPU_REVA_CTRL_RDSRC_DMADISABLED          (MXC_V_TPU_REVA_CTRL_RDSRC_DMADISABLED << MXC_F_TPU_REVA_CTRL_RDSRC_POS) /**< CTRL_RDSRC_DMADISABLED Setting */
 #define MXC_V_TPU_REVA_CTRL_RDSRC_DMAORAPB             ((uint32_t)0x1UL) /**< CTRL_RDSRC_DMAORAPB Value */
 #define MXC_S_TPU_REVA_CTRL_RDSRC_DMAORAPB             (MXC_V_TPU_REVA_CTRL_RDSRC_DMAORAPB << MXC_F_TPU_REVA_CTRL_RDSRC_POS) /**< CTRL_RDSRC_DMAORAPB Setting */
 #define MXC_V_TPU_REVA_CTRL_RDSRC_RNG                  ((uint32_t)0x2UL) /**< CTRL_RDSRC_RNG Value */
 #define MXC_S_TPU_REVA_CTRL_RDSRC_RNG                  (MXC_V_TPU_REVA_CTRL_RDSRC_RNG << MXC_F_TPU_REVA_CTRL_RDSRC_POS) /**< CTRL_RDSRC_RNG Setting */

 #define MXC_F_TPU_REVA_CTRL_FLAG_MODE_POS              14 /**< CTRL_FLAG_MODE Position */
 #define MXC_F_TPU_REVA_CTRL_FLAG_MODE                  ((uint32_t)(0x1UL << MXC_F_TPU_REVA_CTRL_FLAG_MODE_POS)) /**< CTRL_FLAG_MODE Mask */

 #define MXC_F_TPU_REVA_CTRL_DMADNE_MSK_POS             15 /**< CTRL_DMADNE_MSK Position */
 #define MXC_F_TPU_REVA_CTRL_DMADNE_MSK                 ((uint32_t)(0x1UL << MXC_F_TPU_REVA_CTRL_DMADNE_MSK_POS)) /**< CTRL_DMADNE_MSK Mask */

 #define MXC_F_TPU_REVA_CTRL_DMA_DONE_POS               24 /**< CTRL_DMA_DONE Position */
 #define MXC_F_TPU_REVA_CTRL_DMA_DONE                   ((uint32_t)(0x1UL << MXC_F_TPU_REVA_CTRL_DMA_DONE_POS)) /**< CTRL_DMA_DONE Mask */

 #define MXC_F_TPU_REVA_CTRL_GLS_DONE_POS               25 /**< CTRL_GLS_DONE Position */
 #define MXC_F_TPU_REVA_CTRL_GLS_DONE                   ((uint32_t)(0x1UL << MXC_F_TPU_REVA_CTRL_GLS_DONE_POS)) /**< CTRL_GLS_DONE Mask */

 #define MXC_F_TPU_REVA_CTRL_HSH_DONE_POS               26 /**< CTRL_HSH_DONE Position */
 #define MXC_F_TPU_REVA_CTRL_HSH_DONE                   ((uint32_t)(0x1UL << MXC_F_TPU_REVA_CTRL_HSH_DONE_POS)) /**< CTRL_HSH_DONE Mask */

 #define MXC_F_TPU_REVA_CTRL_CPH_DONE_POS               27 /**< CTRL_CPH_DONE Position */
 #define MXC_F_TPU_REVA_CTRL_CPH_DONE                   ((uint32_t)(0x1UL << MXC_F_TPU_REVA_CTRL_CPH_DONE_POS)) /**< CTRL_CPH_DONE Mask */

 #define MXC_F_TPU_REVA_CTRL_MAA_DONE_POS               28 /**< CTRL_MAA_DONE Position */
 #define MXC_F_TPU_REVA_CTRL_MAA_DONE                   ((uint32_t)(0x1UL << MXC_F_TPU_REVA_CTRL_MAA_DONE_POS)) /**< CTRL_MAA_DONE Mask */

 #define MXC_F_TPU_REVA_CTRL_ERR_POS                    29 /**< CTRL_ERR Position */
 #define MXC_F_TPU_REVA_CTRL_ERR                        ((uint32_t)(0x1UL << MXC_F_TPU_REVA_CTRL_ERR_POS)) /**< CTRL_ERR Mask */

 #define MXC_F_TPU_REVA_CTRL_RDY_POS                    30 /**< CTRL_RDY Position */
 #define MXC_F_TPU_REVA_CTRL_RDY                        ((uint32_t)(0x1UL << MXC_F_TPU_REVA_CTRL_RDY_POS)) /**< CTRL_RDY Mask */

 #define MXC_F_TPU_REVA_CTRL_DONE_POS                   31 /**< CTRL_DONE Position */
 #define MXC_F_TPU_REVA_CTRL_DONE                       ((uint32_t)(0x1UL << MXC_F_TPU_REVA_CTRL_DONE_POS)) /**< CTRL_DONE Mask */

/**@} end of group TPU_REVA_CTRL_Register */

/**
 * @ingroup  tpu_reva_registers
 * @defgroup TPU_REVA_CIPHER_CTRL TPU_REVA_CIPHER_CTRL
 * @brief    Cipher Control Register.
 * @{
 */
 #define MXC_F_TPU_REVA_CIPHER_CTRL_ENC_POS             0 /**< CIPHER_CTRL_ENC Position */
 #define MXC_F_TPU_REVA_CIPHER_CTRL_ENC                 ((uint32_t)(0x1UL << MXC_F_TPU_REVA_CIPHER_CTRL_ENC_POS)) /**< CIPHER_CTRL_ENC Mask */

 #define MXC_F_TPU_REVA_CIPHER_CTRL_KEY_POS             1 /**< CIPHER_CTRL_KEY Position */
 #define MXC_F_TPU_REVA_CIPHER_CTRL_KEY                 ((uint32_t)(0x1UL << MXC_F_TPU_REVA_CIPHER_CTRL_KEY_POS)) /**< CIPHER_CTRL_KEY Mask */

 #define MXC_F_TPU_REVA_CIPHER_CTRL_SRC_POS             2 /**< CIPHER_CTRL_SRC Position */
 #define MXC_F_TPU_REVA_CIPHER_CTRL_SRC                 ((uint32_t)(0x3UL << MXC_F_TPU_REVA_CIPHER_CTRL_SRC_POS)) /**< CIPHER_CTRL_SRC Mask */
 #define MXC_V_TPU_REVA_CIPHER_CTRL_SRC_CIPHERKEY       ((uint32_t)0x0UL) /**< CIPHER_CTRL_SRC_CIPHERKEY Value */
 #define MXC_S_TPU_REVA_CIPHER_CTRL_SRC_CIPHERKEY       (MXC_V_TPU_REVA_CIPHER_CTRL_SRC_CIPHERKEY << MXC_F_TPU_REVA_CIPHER_CTRL_SRC_POS) /**< CIPHER_CTRL_SRC_CIPHERKEY Setting */
 #define MXC_V_TPU_REVA_CIPHER_CTRL_SRC_REGFILE         ((uint32_t)0x2UL) /**< CIPHER_CTRL_SRC_REGFILE Value */
 #define MXC_S_TPU_REVA_CIPHER_CTRL_SRC_REGFILE         (MXC_V_TPU_REVA_CIPHER_CTRL_SRC_REGFILE << MXC_F_TPU_REVA_CIPHER_CTRL_SRC_POS) /**< CIPHER_CTRL_SRC_REGFILE Setting */
 #define MXC_V_TPU_REVA_CIPHER_CTRL_SRC_QSPIKEY_REGFILE ((uint32_t)0x3UL) /**< CIPHER_CTRL_SRC_QSPIKEY_REGFILE Value */
 #define MXC_S_TPU_REVA_CIPHER_CTRL_SRC_QSPIKEY_REGFILE (MXC_V_TPU_REVA_CIPHER_CTRL_SRC_QSPIKEY_REGFILE << MXC_F_TPU_REVA_CIPHER_CTRL_SRC_POS) /**< CIPHER_CTRL_SRC_QSPIKEY_REGFILE Setting */

 #define MXC_F_TPU_REVA_CIPHER_CTRL_CIPHER_POS          4 /**< CIPHER_CTRL_CIPHER Position */
 #define MXC_F_TPU_REVA_CIPHER_CTRL_CIPHER              ((uint32_t)(0x7UL << MXC_F_TPU_REVA_CIPHER_CTRL_CIPHER_POS)) /**< CIPHER_CTRL_CIPHER Mask */
 #define MXC_V_TPU_REVA_CIPHER_CTRL_CIPHER_DIS          ((uint32_t)0x0UL) /**< CIPHER_CTRL_CIPHER_DIS Value */
 #define MXC_S_TPU_REVA_CIPHER_CTRL_CIPHER_DIS          (MXC_V_TPU_REVA_CIPHER_CTRL_CIPHER_DIS << MXC_F_TPU_REVA_CIPHER_CTRL_CIPHER_POS) /**< CIPHER_CTRL_CIPHER_DIS Setting */
 #define MXC_V_TPU_REVA_CIPHER_CTRL_CIPHER_AES128       ((uint32_t)0x1UL) /**< CIPHER_CTRL_CIPHER_AES128 Value */
 #define MXC_S_TPU_REVA_CIPHER_CTRL_CIPHER_AES128       (MXC_V_TPU_REVA_CIPHER_CTRL_CIPHER_AES128 << MXC_F_TPU_REVA_CIPHER_CTRL_CIPHER_POS) /**< CIPHER_CTRL_CIPHER_AES128 Setting */
 #define MXC_V_TPU_REVA_CIPHER_CTRL_CIPHER_AES192       ((uint32_t)0x2UL) /**< CIPHER_CTRL_CIPHER_AES192 Value */
 #define MXC_S_TPU_REVA_CIPHER_CTRL_CIPHER_AES192       (MXC_V_TPU_REVA_CIPHER_CTRL_CIPHER_AES192 << MXC_F_TPU_REVA_CIPHER_CTRL_CIPHER_POS) /**< CIPHER_CTRL_CIPHER_AES192 Setting */
 #define MXC_V_TPU_REVA_CIPHER_CTRL_CIPHER_AES256       ((uint32_t)0x3UL) /**< CIPHER_CTRL_CIPHER_AES256 Value */
 #define MXC_S_TPU_REVA_CIPHER_CTRL_CIPHER_AES256       (MXC_V_TPU_REVA_CIPHER_CTRL_CIPHER_AES256 << MXC_F_TPU_REVA_CIPHER_CTRL_CIPHER_POS) /**< CIPHER_CTRL_CIPHER_AES256 Setting */
 #define MXC_V_TPU_REVA_CIPHER_CTRL_CIPHER_DES          ((uint32_t)0x4UL) /**< CIPHER_CTRL_CIPHER_DES Value */
 #define MXC_S_TPU_REVA_CIPHER_CTRL_CIPHER_DES          (MXC_V_TPU_REVA_CIPHER_CTRL_CIPHER_DES << MXC_F_TPU_REVA_CIPHER_CTRL_CIPHER_POS) /**< CIPHER_CTRL_CIPHER_DES Setting */
 #define MXC_V_TPU_REVA_CIPHER_CTRL_CIPHER_TDES         ((uint32_t)0x5UL) /**< CIPHER_CTRL_CIPHER_TDES Value */
 #define MXC_S_TPU_REVA_CIPHER_CTRL_CIPHER_TDES         (MXC_V_TPU_REVA_CIPHER_CTRL_CIPHER_TDES << MXC_F_TPU_REVA_CIPHER_CTRL_CIPHER_POS) /**< CIPHER_CTRL_CIPHER_TDES Setting */

 #define MXC_F_TPU_REVA_CIPHER_CTRL_MODE_POS            8 /**< CIPHER_CTRL_MODE Position */
 #define MXC_F_TPU_REVA_CIPHER_CTRL_MODE                ((uint32_t)(0x7UL << MXC_F_TPU_REVA_CIPHER_CTRL_MODE_POS)) /**< CIPHER_CTRL_MODE Mask */
 #define MXC_V_TPU_REVA_CIPHER_CTRL_MODE_ECB            ((uint32_t)0x0UL) /**< CIPHER_CTRL_MODE_ECB Value */
 #define MXC_S_TPU_REVA_CIPHER_CTRL_MODE_ECB            (MXC_V_TPU_REVA_CIPHER_CTRL_MODE_ECB << MXC_F_TPU_REVA_CIPHER_CTRL_MODE_POS) /**< CIPHER_CTRL_MODE_ECB Setting */
 #define MXC_V_TPU_REVA_CIPHER_CTRL_MODE_CBC            ((uint32_t)0x1UL) /**< CIPHER_CTRL_MODE_CBC Value */
 #define MXC_S_TPU_REVA_CIPHER_CTRL_MODE_CBC            (MXC_V_TPU_REVA_CIPHER_CTRL_MODE_CBC << MXC_F_TPU_REVA_CIPHER_CTRL_MODE_POS) /**< CIPHER_CTRL_MODE_CBC Setting */
 #define MXC_V_TPU_REVA_CIPHER_CTRL_MODE_CFB            ((uint32_t)0x2UL) /**< CIPHER_CTRL_MODE_CFB Value */
 #define MXC_S_TPU_REVA_CIPHER_CTRL_MODE_CFB            (MXC_V_TPU_REVA_CIPHER_CTRL_MODE_CFB << MXC_F_TPU_REVA_CIPHER_CTRL_MODE_POS) /**< CIPHER_CTRL_MODE_CFB Setting */
 #define MXC_V_TPU_REVA_CIPHER_CTRL_MODE_OFB            ((uint32_t)0x3UL) /**< CIPHER_CTRL_MODE_OFB Value */
 #define MXC_S_TPU_REVA_CIPHER_CTRL_MODE_OFB            (MXC_V_TPU_REVA_CIPHER_CTRL_MODE_OFB << MXC_F_TPU_REVA_CIPHER_CTRL_MODE_POS) /**< CIPHER_CTRL_MODE_OFB Setting */
 #define MXC_V_TPU_REVA_CIPHER_CTRL_MODE_CTR            ((uint32_t)0x4UL) /**< CIPHER_CTRL_MODE_CTR Value */
 #define MXC_S_TPU_REVA_CIPHER_CTRL_MODE_CTR            (MXC_V_TPU_REVA_CIPHER_CTRL_MODE_CTR << MXC_F_TPU_REVA_CIPHER_CTRL_MODE_POS) /**< CIPHER_CTRL_MODE_CTR Setting */

/**@} end of group TPU_REVA_CIPHER_CTRL_Register */

/**
 * @ingroup  tpu_reva_registers
 * @defgroup TPU_REVA_HASH_CTRL TPU_REVA_HASH_CTRL
 * @brief    HASH Control Register.
 * @{
 */
 #define MXC_F_TPU_REVA_HASH_CTRL_INIT_POS              0 /**< HASH_CTRL_INIT Position */
 #define MXC_F_TPU_REVA_HASH_CTRL_INIT                  ((uint32_t)(0x1UL << MXC_F_TPU_REVA_HASH_CTRL_INIT_POS)) /**< HASH_CTRL_INIT Mask */

 #define MXC_F_TPU_REVA_HASH_CTRL_XOR_POS               1 /**< HASH_CTRL_XOR Position */
 #define MXC_F_TPU_REVA_HASH_CTRL_XOR                   ((uint32_t)(0x1UL << MXC_F_TPU_REVA_HASH_CTRL_XOR_POS)) /**< HASH_CTRL_XOR Mask */

 #define MXC_F_TPU_REVA_HASH_CTRL_HASH_POS              2 /**< HASH_CTRL_HASH Position */
 #define MXC_F_TPU_REVA_HASH_CTRL_HASH                  ((uint32_t)(0x7UL << MXC_F_TPU_REVA_HASH_CTRL_HASH_POS)) /**< HASH_CTRL_HASH Mask */
 #define MXC_V_TPU_REVA_HASH_CTRL_HASH_DIS              ((uint32_t)0x0UL) /**< HASH_CTRL_HASH_DIS Value */
 #define MXC_S_TPU_REVA_HASH_CTRL_HASH_DIS              (MXC_V_TPU_REVA_HASH_CTRL_HASH_DIS << MXC_F_TPU_REVA_HASH_CTRL_HASH_POS) /**< HASH_CTRL_HASH_DIS Setting */
 #define MXC_V_TPU_REVA_HASH_CTRL_HASH_SHA1             ((uint32_t)0x1UL) /**< HASH_CTRL_HASH_SHA1 Value */
 #define MXC_S_TPU_REVA_HASH_CTRL_HASH_SHA1             (MXC_V_TPU_REVA_HASH_CTRL_HASH_SHA1 << MXC_F_TPU_REVA_HASH_CTRL_HASH_POS) /**< HASH_CTRL_HASH_SHA1 Setting */
 #define MXC_V_TPU_REVA_HASH_CTRL_HASH_SHA224           ((uint32_t)0x2UL) /**< HASH_CTRL_HASH_SHA224 Value */
 #define MXC_S_TPU_REVA_HASH_CTRL_HASH_SHA224           (MXC_V_TPU_REVA_HASH_CTRL_HASH_SHA224 << MXC_F_TPU_REVA_HASH_CTRL_HASH_POS) /**< HASH_CTRL_HASH_SHA224 Setting */
 #define MXC_V_TPU_REVA_HASH_CTRL_HASH_SHA256           ((uint32_t)0x3UL) /**< HASH_CTRL_HASH_SHA256 Value */
 #define MXC_S_TPU_REVA_HASH_CTRL_HASH_SHA256           (MXC_V_TPU_REVA_HASH_CTRL_HASH_SHA256 << MXC_F_TPU_REVA_HASH_CTRL_HASH_POS) /**< HASH_CTRL_HASH_SHA256 Setting */
 #define MXC_V_TPU_REVA_HASH_CTRL_HASH_SHA384           ((uint32_t)0x4UL) /**< HASH_CTRL_HASH_SHA384 Value */
 #define MXC_S_TPU_REVA_HASH_CTRL_HASH_SHA384           (MXC_V_TPU_REVA_HASH_CTRL_HASH_SHA384 << MXC_F_TPU_REVA_HASH_CTRL_HASH_POS) /**< HASH_CTRL_HASH_SHA384 Setting */
 #define MXC_V_TPU_REVA_HASH_CTRL_HASH_SHA512           ((uint32_t)0x5UL) /**< HASH_CTRL_HASH_SHA512 Value */
 #define MXC_S_TPU_REVA_HASH_CTRL_HASH_SHA512           (MXC_V_TPU_REVA_HASH_CTRL_HASH_SHA512 << MXC_F_TPU_REVA_HASH_CTRL_HASH_POS) /**< HASH_CTRL_HASH_SHA512 Setting */

 #define MXC_F_TPU_REVA_HASH_CTRL_LAST_POS              5 /**< HASH_CTRL_LAST Position */
 #define MXC_F_TPU_REVA_HASH_CTRL_LAST                  ((uint32_t)(0x1UL << MXC_F_TPU_REVA_HASH_CTRL_LAST_POS)) /**< HASH_CTRL_LAST Mask */

/**@} end of group TPU_REVA_HASH_CTRL_Register */

/**
 * @ingroup  tpu_reva_registers
 * @defgroup TPU_REVA_CRC_CTRL TPU_REVA_CRC_CTRL
 * @brief    CRC Control Register.
 * @{
 */
 #define MXC_F_TPU_REVA_CRC_CTRL_CRC_EN_POS             0 /**< CRC_CTRL_CRC_EN Position */
 #define MXC_F_TPU_REVA_CRC_CTRL_CRC_EN                 ((uint32_t)(0x1UL << MXC_F_TPU_REVA_CRC_CTRL_CRC_EN_POS)) /**< CRC_CTRL_CRC_EN Mask */

 #define MXC_F_TPU_REVA_CRC_CTRL_MSB_POS                1 /**< CRC_CTRL_MSB Position */
 #define MXC_F_TPU_REVA_CRC_CTRL_MSB                    ((uint32_t)(0x1UL << MXC_F_TPU_REVA_CRC_CTRL_MSB_POS)) /**< CRC_CTRL_MSB Mask */

 #define MXC_F_TPU_REVA_CRC_CTRL_PRNG_POS               2 /**< CRC_CTRL_PRNG Position */
 #define MXC_F_TPU_REVA_CRC_CTRL_PRNG                   ((uint32_t)(0x1UL << MXC_F_TPU_REVA_CRC_CTRL_PRNG_POS)) /**< CRC_CTRL_PRNG Mask */

 #define MXC_F_TPU_REVA_CRC_CTRL_ENT_POS                3 /**< CRC_CTRL_ENT Position */
 #define MXC_F_TPU_REVA_CRC_CTRL_ENT                    ((uint32_t)(0x1UL << MXC_F_TPU_REVA_CRC_CTRL_ENT_POS)) /**< CRC_CTRL_ENT Mask */

 #define MXC_F_TPU_REVA_CRC_CTRL_HAM_POS                4 /**< CRC_CTRL_HAM Position */
 #define MXC_F_TPU_REVA_CRC_CTRL_HAM                    ((uint32_t)(0x1UL << MXC_F_TPU_REVA_CRC_CTRL_HAM_POS)) /**< CRC_CTRL_HAM Mask */

 #define MXC_F_TPU_REVA_CRC_CTRL_HRST_POS               5 /**< CRC_CTRL_HRST Position */
 #define MXC_F_TPU_REVA_CRC_CTRL_HRST                   ((uint32_t)(0x1UL << MXC_F_TPU_REVA_CRC_CTRL_HRST_POS)) /**< CRC_CTRL_HRST Mask */

/**@} end of group TPU_REVA_CRC_CTRL_Register */

/**
 * @ingroup  tpu_reva_registers
 * @defgroup TPU_REVA_DMA_SRC TPU_REVA_DMA_SRC
 * @brief    Crypto DMA Source Address.
 * @{
 */
 #define MXC_F_TPU_REVA_DMA_SRC_SRC_ADDR_POS            0 /**< DMA_SRC_SRC_ADDR Position */
 #define MXC_F_TPU_REVA_DMA_SRC_SRC_ADDR                ((uint32_t)(0xFFFFFFFFUL << MXC_F_TPU_REVA_DMA_SRC_SRC_ADDR_POS)) /**< DMA_SRC_SRC_ADDR Mask */

/**@} end of group TPU_REVA_DMA_SRC_Register */

/**
 * @ingroup  tpu_reva_registers
 * @defgroup TPU_REVA_DMA_DST TPU_REVA_DMA_DST
 * @brief    Crypto DMA Destination Address.
 * @{
 */
 #define MXC_F_TPU_REVA_DMA_DST_DST_ADDR_POS            0 /**< DMA_DST_DST_ADDR Position */
 #define MXC_F_TPU_REVA_DMA_DST_DST_ADDR                ((uint32_t)(0xFFFFFFFFUL << MXC_F_TPU_REVA_DMA_DST_DST_ADDR_POS)) /**< DMA_DST_DST_ADDR Mask */

/**@} end of group TPU_REVA_DMA_DST_Register */

/**
 * @ingroup  tpu_reva_registers
 * @defgroup TPU_REVA_DMA_CNT TPU_REVA_DMA_CNT
 * @brief    Crypto DMA Byte Count.
 * @{
 */
 #define MXC_F_TPU_REVA_DMA_CNT_COUNT_POS               0 /**< DMA_CNT_COUNT Position */
 #define MXC_F_TPU_REVA_DMA_CNT_COUNT                   ((uint32_t)(0xFFFFFFFFUL << MXC_F_TPU_REVA_DMA_CNT_COUNT_POS)) /**< DMA_CNT_COUNT Mask */

/**@} end of group TPU_REVA_DMA_CNT_Register */

/**
 * @ingroup  tpu_reva_registers
 * @defgroup TPU_REVA_MAA_CTRL TPU_REVA_MAA_CTRL
 * @brief    MAA Control Register.
 * @{
 */
 #define MXC_F_TPU_REVA_MAA_CTRL_STC_POS                0 /**< MAA_CTRL_STC Position */
 #define MXC_F_TPU_REVA_MAA_CTRL_STC                    ((uint32_t)(0x1UL << MXC_F_TPU_REVA_MAA_CTRL_STC_POS)) /**< MAA_CTRL_STC Mask */

 #define MXC_F_TPU_REVA_MAA_CTRL_CLC_POS                1 /**< MAA_CTRL_CLC Position */
 #define MXC_F_TPU_REVA_MAA_CTRL_CLC                    ((uint32_t)(0x7UL << MXC_F_TPU_REVA_MAA_CTRL_CLC_POS)) /**< MAA_CTRL_CLC Mask */
 #define MXC_V_TPU_REVA_MAA_CTRL_CLC_EXP                ((uint32_t)0x0UL) /**< MAA_CTRL_CLC_EXP Value */
 #define MXC_S_TPU_REVA_MAA_CTRL_CLC_EXP                (MXC_V_TPU_REVA_MAA_CTRL_CLC_EXP << MXC_F_TPU_REVA_MAA_CTRL_CLC_POS) /**< MAA_CTRL_CLC_EXP Setting */
 #define MXC_V_TPU_REVA_MAA_CTRL_CLC_SQ                 ((uint32_t)0x1UL) /**< MAA_CTRL_CLC_SQ Value */
 #define MXC_S_TPU_REVA_MAA_CTRL_CLC_SQ                 (MXC_V_TPU_REVA_MAA_CTRL_CLC_SQ << MXC_F_TPU_REVA_MAA_CTRL_CLC_POS) /**< MAA_CTRL_CLC_SQ Setting */
 #define MXC_V_TPU_REVA_MAA_CTRL_CLC_MUL                ((uint32_t)0x2UL) /**< MAA_CTRL_CLC_MUL Value */
 #define MXC_S_TPU_REVA_MAA_CTRL_CLC_MUL                (MXC_V_TPU_REVA_MAA_CTRL_CLC_MUL << MXC_F_TPU_REVA_MAA_CTRL_CLC_POS) /**< MAA_CTRL_CLC_MUL Setting */
 #define MXC_V_TPU_REVA_MAA_CTRL_CLC_SQMUL              ((uint32_t)0x3UL) /**< MAA_CTRL_CLC_SQMUL Value */
 #define MXC_S_TPU_REVA_MAA_CTRL_CLC_SQMUL              (MXC_V_TPU_REVA_MAA_CTRL_CLC_SQMUL << MXC_F_TPU_REVA_MAA_CTRL_CLC_POS) /**< MAA_CTRL_CLC_SQMUL Setting */
 #define MXC_V_TPU_REVA_MAA_CTRL_CLC_ADD                ((uint32_t)0x4UL) /**< MAA_CTRL_CLC_ADD Value */
 #define MXC_S_TPU_REVA_MAA_CTRL_CLC_ADD                (MXC_V_TPU_REVA_MAA_CTRL_CLC_ADD << MXC_F_TPU_REVA_MAA_CTRL_CLC_POS) /**< MAA_CTRL_CLC_ADD Setting */
 #define MXC_V_TPU_REVA_MAA_CTRL_CLC_SUB                ((uint32_t)0x5UL) /**< MAA_CTRL_CLC_SUB Value */
 #define MXC_S_TPU_REVA_MAA_CTRL_CLC_SUB                (MXC_V_TPU_REVA_MAA_CTRL_CLC_SUB << MXC_F_TPU_REVA_MAA_CTRL_CLC_POS) /**< MAA_CTRL_CLC_SUB Setting */

 #define MXC_F_TPU_REVA_MAA_CTRL_OCALC_POS              4 /**< MAA_CTRL_OCALC Position */
 #define MXC_F_TPU_REVA_MAA_CTRL_OCALC                  ((uint32_t)(0x1UL << MXC_F_TPU_REVA_MAA_CTRL_OCALC_POS)) /**< MAA_CTRL_OCALC Mask */

 #define MXC_F_TPU_REVA_MAA_CTRL_MAAER_POS              7 /**< MAA_CTRL_MAAER Position */
 #define MXC_F_TPU_REVA_MAA_CTRL_MAAER                  ((uint32_t)(0x1UL << MXC_F_TPU_REVA_MAA_CTRL_MAAER_POS)) /**< MAA_CTRL_MAAER Mask */

 #define MXC_F_TPU_REVA_MAA_CTRL_AMS_POS                8 /**< MAA_CTRL_AMS Position */
 #define MXC_F_TPU_REVA_MAA_CTRL_AMS                    ((uint32_t)(0x3UL << MXC_F_TPU_REVA_MAA_CTRL_AMS_POS)) /**< MAA_CTRL_AMS Mask */

 #define MXC_F_TPU_REVA_MAA_CTRL_BMS_POS                10 /**< MAA_CTRL_BMS Position */
 #define MXC_F_TPU_REVA_MAA_CTRL_BMS                    ((uint32_t)(0x3UL << MXC_F_TPU_REVA_MAA_CTRL_BMS_POS)) /**< MAA_CTRL_BMS Mask */

 #define MXC_F_TPU_REVA_MAA_CTRL_EMS_POS                12 /**< MAA_CTRL_EMS Position */
 #define MXC_F_TPU_REVA_MAA_CTRL_EMS                    ((uint32_t)(0x3UL << MXC_F_TPU_REVA_MAA_CTRL_EMS_POS)) /**< MAA_CTRL_EMS Mask */

 #define MXC_F_TPU_REVA_MAA_CTRL_MMS_POS                14 /**< MAA_CTRL_MMS Position */
 #define MXC_F_TPU_REVA_MAA_CTRL_MMS                    ((uint32_t)(0x3UL << MXC_F_TPU_REVA_MAA_CTRL_MMS_POS)) /**< MAA_CTRL_MMS Mask */

 #define MXC_F_TPU_REVA_MAA_CTRL_AMA_POS                16 /**< MAA_CTRL_AMA Position */
 #define MXC_F_TPU_REVA_MAA_CTRL_AMA                    ((uint32_t)(0xFUL << MXC_F_TPU_REVA_MAA_CTRL_AMA_POS)) /**< MAA_CTRL_AMA Mask */

 #define MXC_F_TPU_REVA_MAA_CTRL_BMA_POS                20 /**< MAA_CTRL_BMA Position */
 #define MXC_F_TPU_REVA_MAA_CTRL_BMA                    ((uint32_t)(0xFUL << MXC_F_TPU_REVA_MAA_CTRL_BMA_POS)) /**< MAA_CTRL_BMA Mask */

 #define MXC_F_TPU_REVA_MAA_CTRL_RMA_POS                24 /**< MAA_CTRL_RMA Position */
 #define MXC_F_TPU_REVA_MAA_CTRL_RMA                    ((uint32_t)(0xFUL << MXC_F_TPU_REVA_MAA_CTRL_RMA_POS)) /**< MAA_CTRL_RMA Mask */

 #define MXC_F_TPU_REVA_MAA_CTRL_TMA_POS                28 /**< MAA_CTRL_TMA Position */
 #define MXC_F_TPU_REVA_MAA_CTRL_TMA                    ((uint32_t)(0xFUL << MXC_F_TPU_REVA_MAA_CTRL_TMA_POS)) /**< MAA_CTRL_TMA Mask */

/**@} end of group TPU_REVA_MAA_CTRL_Register */

/**
 * @ingroup  tpu_reva_registers
 * @defgroup TPU_REVA_DATA_IN TPU_REVA_DATA_IN
 * @brief    Crypto Data Input. Data input can be written to this register instead of using
 *           the DMA. This register writes to the FIFO. This register occupies four
 *           successive words to allow the use of multi-store instructions. Words can be
 *           written to any location, they will be placed in the FIFO in the order they are
 *           written. The endian swap input control bit affects this register.
 * @{
 */
 #define MXC_F_TPU_REVA_DATA_IN_DATA_POS                0 /**< DATA_IN_DATA Position */
 #define MXC_F_TPU_REVA_DATA_IN_DATA                    ((uint32_t)(0xFFFFFFFFUL << MXC_F_TPU_REVA_DATA_IN_DATA_POS)) /**< DATA_IN_DATA Mask */

/**@} end of group TPU_REVA_DATA_IN_Register */

/**
 * @ingroup  tpu_reva_registers
 * @defgroup TPU_REVA_DATA_OUT TPU_REVA_DATA_OUT
 * @brief    Crypto Data Output. Resulting data from cipher calculation. Data is placed in
 *           the lower words of these four registers depending on the algorithm. For block
 *           cipher modes, this register holds the result of most recent encryption or
 *           decryption operation. These registers are affected by the endian swap bits.
 * @{
 */
 #define MXC_F_TPU_REVA_DATA_OUT_DATA_POS               0 /**< DATA_OUT_DATA Position */
 #define MXC_F_TPU_REVA_DATA_OUT_DATA                   ((uint32_t)(0xFFFFFFFFUL << MXC_F_TPU_REVA_DATA_OUT_DATA_POS)) /**< DATA_OUT_DATA Mask */

/**@} end of group TPU_REVA_DATA_OUT_Register */

/**
 * @ingroup  tpu_reva_registers
 * @defgroup TPU_REVA_CRC_POLY TPU_REVA_CRC_POLY
 * @brief    CRC Polynomial. The polynomial to be used for Galois Field calculations (CRC or
 *           LFSR) should be written to this register. This register is affected by the MSB
 *           control bit.
 * @{
 */
 #define MXC_F_TPU_REVA_CRC_POLY_SRC_ADDR_POS           0 /**< CRC_POLY_SRC_ADDR Position */
 #define MXC_F_TPU_REVA_CRC_POLY_SRC_ADDR               ((uint32_t)(0xFFFFFFFFUL << MXC_F_TPU_REVA_CRC_POLY_SRC_ADDR_POS)) /**< CRC_POLY_SRC_ADDR Mask */

/**@} end of group TPU_REVA_CRC_POLY_Register */

/**
 * @ingroup  tpu_reva_registers
 * @defgroup TPU_REVA_CRC_VAL TPU_REVA_CRC_VAL
 * @brief    CRC Value. This is the state for the Galois Field. This register holds the
 *           result of a CRC calculation or the current state of the LFSR. This register is
 *           affected by the MSB control bit.
 * @{
 */
 #define MXC_F_TPU_REVA_CRC_VAL_VAL_POS                 0 /**< CRC_VAL_VAL Position */
 #define MXC_F_TPU_REVA_CRC_VAL_VAL                     ((uint32_t)(0xFFFFFFFFUL << MXC_F_TPU_REVA_CRC_VAL_VAL_POS)) /**< CRC_VAL_VAL Mask */

/**@} end of group TPU_REVA_CRC_VAL_Register */

/**
 * @ingroup  tpu_reva_registers
 * @defgroup TPU_REVA_CRC_PRNG TPU_REVA_CRC_PRNG
 * @brief    Pseudo Random Value. Output of the Galois Field shift register. This holds the
 *           resulting pseudo-random number if entropy is disabled or true random number if
 *           entropy is enabled.
 * @{
 */
 #define MXC_F_TPU_REVA_CRC_PRNG_PRNG_POS               0 /**< CRC_PRNG_PRNG Position */
 #define MXC_F_TPU_REVA_CRC_PRNG_PRNG                   ((uint32_t)(0xFFFFFFFFUL << MXC_F_TPU_REVA_CRC_PRNG_PRNG_POS)) /**< CRC_PRNG_PRNG Mask */

/**@} end of group TPU_REVA_CRC_PRNG_Register */

/**
 * @ingroup  tpu_reva_registers
 * @defgroup TPU_REVA_HAM_ECC TPU_REVA_HAM_ECC
 * @brief    Hamming ECC Register.
 * @{
 */
 #define MXC_F_TPU_REVA_HAM_ECC_ECC_POS                 0 /**< HAM_ECC_ECC Position */
 #define MXC_F_TPU_REVA_HAM_ECC_ECC                     ((uint32_t)(0xFFFFUL << MXC_F_TPU_REVA_HAM_ECC_ECC_POS)) /**< HAM_ECC_ECC Mask */

 #define MXC_F_TPU_REVA_HAM_ECC_PAR_POS                 16 /**< HAM_ECC_PAR Position */
 #define MXC_F_TPU_REVA_HAM_ECC_PAR                     ((uint32_t)(0x1UL << MXC_F_TPU_REVA_HAM_ECC_PAR_POS)) /**< HAM_ECC_PAR Mask */

/**@} end of group TPU_REVA_HAM_ECC_Register */

/**
 * @ingroup  tpu_reva_registers
 * @defgroup TPU_REVA_CIPHER_INIT TPU_REVA_CIPHER_INIT
 * @brief    Initial Vector. For block cipher operations that use CBC, CFB, OFB, or CNTR
 *           modes, this register holds the initial value. This register is updated with each
 *           encryption or decryption operation. This register is affected by the endian swap
 *           bits.
 * @{
 */
 #define MXC_F_TPU_REVA_CIPHER_INIT_IVEC_POS            0 /**< CIPHER_INIT_IVEC Position */
 #define MXC_F_TPU_REVA_CIPHER_INIT_IVEC                ((uint32_t)(0xFFFFFFFFUL << MXC_F_TPU_REVA_CIPHER_INIT_IVEC_POS)) /**< CIPHER_INIT_IVEC Mask */

/**@} end of group TPU_REVA_CIPHER_INIT_Register */

/**
 * @ingroup  tpu_reva_registers
 * @defgroup TPU_REVA_CIPHER_KEY TPU_REVA_CIPHER_KEY
 * @brief    Cipher Key.  This register holds the key used for block cipher operations. The
 *           lower words are used for block ciphers that use shorter key lengths. This
 *           register is affected by the endian swap input control bits.
 * @{
 */
 #define MXC_F_TPU_REVA_CIPHER_KEY_KEY_POS              0 /**< CIPHER_KEY_KEY Position */
 #define MXC_F_TPU_REVA_CIPHER_KEY_KEY                  ((uint32_t)(0xFFFFFFFFUL << MXC_F_TPU_REVA_CIPHER_KEY_KEY_POS)) /**< CIPHER_KEY_KEY Mask */

/**@} end of group TPU_REVA_CIPHER_KEY_Register */

/**
 * @ingroup  tpu_reva_registers
 * @defgroup TPU_REVA_HASH_DIGEST TPU_REVA_HASH_DIGEST
 * @brief    This register holds the calculated hash value. This register is affected by the
 *           endian swap bits.
 * @{
 */
 #define MXC_F_TPU_REVA_HASH_DIGEST_HASH_POS            0 /**< HASH_DIGEST_HASH Position */
 #define MXC_F_TPU_REVA_HASH_DIGEST_HASH                ((uint32_t)(0xFFFFFFFFUL << MXC_F_TPU_REVA_HASH_DIGEST_HASH_POS)) /**< HASH_DIGEST_HASH Mask */

/**@} end of group TPU_REVA_HASH_DIGEST_Register */

/**
 * @ingroup  tpu_reva_registers
 * @defgroup TPU_REVA_HASH_MSG_SZ TPU_REVA_HASH_MSG_SZ
 * @brief    Message Size. This register holds the lowest 32-bit of message size in bytes.
 * @{
 */
 #define MXC_F_TPU_REVA_HASH_MSG_SZ_MSGSZ_POS           0 /**< HASH_MSG_SZ_MSGSZ Position */
 #define MXC_F_TPU_REVA_HASH_MSG_SZ_MSGSZ               ((uint32_t)(0xFFFFFFFFUL << MXC_F_TPU_REVA_HASH_MSG_SZ_MSGSZ_POS)) /**< HASH_MSG_SZ_MSGSZ Mask */

/**@} end of group TPU_REVA_HASH_MSG_SZ_Register */

/**
 * @ingroup  tpu_reva_registers
 * @defgroup TPU_REVA_MAA_MAWS TPU_REVA_MAA_MAWS
 * @brief    MAA Word Size. This register defines the number of bits for a modular operation.
 *           This register must be set to a valid value prior to the MAA operation start.
 *           Valid values are from 1 to 2048.  Invalid values are ignored and will not
 *           initiate a MAA operation.
 * @{
 */
 #define MXC_F_TPU_REVA_MAA_MAWS_MSGSZ_POS              0 /**< MAA_MAWS_MSGSZ Position */
 #define MXC_F_TPU_REVA_MAA_MAWS_MSGSZ                  ((uint32_t)(0xFFFUL << MXC_F_TPU_REVA_MAA_MAWS_MSGSZ_POS)) /**< MAA_MAWS_MSGSZ Mask */

/**@} end of group TPU_REVA_MAA_MAWS_Register */

#ifdef __cplusplus
}
#endif

#endif /* _TPU_REVA_REGS_H_ */
