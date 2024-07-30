/**
 * @file    spixfm_reva_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the SPIXFM_REVA Peripheral Module.
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

#ifndef _SPIXFM_REVA_REGS_H_
#define _SPIXFM_REVA_REGS_H_

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
 * @ingroup     spixfm_reva
 * @defgroup    spixfm_reva_registers SPIXFM_REVA_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the SPIXFM_REVA Peripheral Module.
 * @details SPIXF Master
 */

/**
 * @ingroup spixfm_reva_registers
 * Structure type to access the SPIXFM_REVA Registers.
 */
typedef struct {
    __IO uint32_t cfg;                  /**< <tt>\b 0x00:</tt> SPIXFM_REVA CFG Register */
    __IO uint32_t fetch_ctrl;           /**< <tt>\b 0x04:</tt> SPIXFM_REVA FETCH_CTRL Register */
    __IO uint32_t mode_ctrl;            /**< <tt>\b 0x08:</tt> SPIXFM_REVA MODE_CTRL Register */
    __IO uint32_t mode_data;            /**< <tt>\b 0x0C:</tt> SPIXFM_REVA MODE_DATA Register */
    __IO uint32_t fb_ctrl;              /**< <tt>\b 0x10:</tt> SPIXFM_REVA FB_CTRL Register */
    __R  uint32_t rsv_0x14_0x1b[2];
    __IO uint32_t io_ctrl;              /**< <tt>\b 0x1C:</tt> SPIXFM_REVA IO_CTRL Register */
    __IO uint32_t sec_ctrl;             /**< <tt>\b 0x20:</tt> SPIXFM_REVA SEC_CTRL Register */
    __IO uint32_t bus_idle;             /**< <tt>\b 0x24:</tt> SPIXFM_REVA BUS_IDLE Register */
} mxc_spixfm_reva_regs_t;

/* Register offsets for module SPIXFM_REVA */
/**
 * @ingroup    spixfm_reva_registers
 * @defgroup   SPIXFM_REVA_Register_Offsets Register Offsets
 * @brief      SPIXFM_REVA Peripheral Register Offsets from the SPIXFM_REVA Base Peripheral Address. 
 * @{
 */
 #define MXC_R_SPIXFM_REVA_CFG              ((uint32_t)0x00000000UL) /**< Offset from SPIXFM_REVA Base Address: <tt> 0x0000</tt> */ 
 #define MXC_R_SPIXFM_REVA_FETCH_CTRL       ((uint32_t)0x00000004UL) /**< Offset from SPIXFM_REVA Base Address: <tt> 0x0004</tt> */ 
 #define MXC_R_SPIXFM_REVA_MODE_CTRL        ((uint32_t)0x00000008UL) /**< Offset from SPIXFM_REVA Base Address: <tt> 0x0008</tt> */ 
 #define MXC_R_SPIXFM_REVA_MODE_DATA        ((uint32_t)0x0000000CUL) /**< Offset from SPIXFM_REVA Base Address: <tt> 0x000C</tt> */ 
 #define MXC_R_SPIXFM_REVA_FB_CTRL          ((uint32_t)0x00000010UL) /**< Offset from SPIXFM_REVA Base Address: <tt> 0x0010</tt> */ 
 #define MXC_R_SPIXFM_REVA_IO_CTRL          ((uint32_t)0x0000001CUL) /**< Offset from SPIXFM_REVA Base Address: <tt> 0x001C</tt> */ 
 #define MXC_R_SPIXFM_REVA_SEC_CTRL         ((uint32_t)0x00000020UL) /**< Offset from SPIXFM_REVA Base Address: <tt> 0x0020</tt> */ 
 #define MXC_R_SPIXFM_REVA_BUS_IDLE         ((uint32_t)0x00000024UL) /**< Offset from SPIXFM_REVA Base Address: <tt> 0x0024</tt> */ 
/**@} end of group spixfm_reva_registers */

/**
 * @ingroup  spixfm_reva_registers
 * @defgroup SPIXFM_REVA_CFG SPIXFM_REVA_CFG
 * @brief    SPIX Configuration Register.
 * @{
 */
 #define MXC_F_SPIXFM_REVA_CFG_MODE_POS                 0 /**< CFG_MODE Position */
 #define MXC_F_SPIXFM_REVA_CFG_MODE                     ((uint32_t)(0x3UL << MXC_F_SPIXFM_REVA_CFG_MODE_POS)) /**< CFG_MODE Mask */
 #define MXC_V_SPIXFM_REVA_CFG_MODE_SCLK_HI_SAMPLE_RISING ((uint32_t)0x0UL) /**< CFG_MODE_SCLK_HI_SAMPLE_RISING Value */
 #define MXC_S_SPIXFM_REVA_CFG_MODE_SCLK_HI_SAMPLE_RISING (MXC_V_SPIXFM_REVA_CFG_MODE_SCLK_HI_SAMPLE_RISING << MXC_F_SPIXFM_REVA_CFG_MODE_POS) /**< CFG_MODE_SCLK_HI_SAMPLE_RISING Setting */
 #define MXC_V_SPIXFM_REVA_CFG_MODE_SCLK_LO_SAMPLE_FAILLING ((uint32_t)0x3UL) /**< CFG_MODE_SCLK_LO_SAMPLE_FAILLING Value */
 #define MXC_S_SPIXFM_REVA_CFG_MODE_SCLK_LO_SAMPLE_FAILLING (MXC_V_SPIXFM_REVA_CFG_MODE_SCLK_LO_SAMPLE_FAILLING << MXC_F_SPIXFM_REVA_CFG_MODE_POS) /**< CFG_MODE_SCLK_LO_SAMPLE_FAILLING Setting */

 #define MXC_F_SPIXFM_REVA_CFG_SSPOL_POS                2 /**< CFG_SSPOL Position */
 #define MXC_F_SPIXFM_REVA_CFG_SSPOL                    ((uint32_t)(0x1UL << MXC_F_SPIXFM_REVA_CFG_SSPOL_POS)) /**< CFG_SSPOL Mask */

 #define MXC_F_SPIXFM_REVA_CFG_SSEL_POS                 4 /**< CFG_SSEL Position */
 #define MXC_F_SPIXFM_REVA_CFG_SSEL                     ((uint32_t)(0x7UL << MXC_F_SPIXFM_REVA_CFG_SSEL_POS)) /**< CFG_SSEL Mask */

 #define MXC_F_SPIXFM_REVA_CFG_LO_CLK_POS               8 /**< CFG_LO_CLK Position */
 #define MXC_F_SPIXFM_REVA_CFG_LO_CLK                   ((uint32_t)(0xFUL << MXC_F_SPIXFM_REVA_CFG_LO_CLK_POS)) /**< CFG_LO_CLK Mask */

 #define MXC_F_SPIXFM_REVA_CFG_HI_CLK_POS               12 /**< CFG_HI_CLK Position */
 #define MXC_F_SPIXFM_REVA_CFG_HI_CLK                   ((uint32_t)(0xFUL << MXC_F_SPIXFM_REVA_CFG_HI_CLK_POS)) /**< CFG_HI_CLK Mask */

 #define MXC_F_SPIXFM_REVA_CFG_SSACT_POS                16 /**< CFG_SSACT Position */
 #define MXC_F_SPIXFM_REVA_CFG_SSACT                    ((uint32_t)(0x3UL << MXC_F_SPIXFM_REVA_CFG_SSACT_POS)) /**< CFG_SSACT Mask */
 #define MXC_V_SPIXFM_REVA_CFG_SSACT_OFF                ((uint32_t)0x0UL) /**< CFG_SSACT_OFF Value */
 #define MXC_S_SPIXFM_REVA_CFG_SSACT_OFF                (MXC_V_SPIXFM_REVA_CFG_SSACT_OFF << MXC_F_SPIXFM_REVA_CFG_SSACT_POS) /**< CFG_SSACT_OFF Setting */
 #define MXC_V_SPIXFM_REVA_CFG_SSACT_FOR_2_MOD_CLK      ((uint32_t)0x1UL) /**< CFG_SSACT_FOR_2_MOD_CLK Value */
 #define MXC_S_SPIXFM_REVA_CFG_SSACT_FOR_2_MOD_CLK      (MXC_V_SPIXFM_REVA_CFG_SSACT_FOR_2_MOD_CLK << MXC_F_SPIXFM_REVA_CFG_SSACT_POS) /**< CFG_SSACT_FOR_2_MOD_CLK Setting */
 #define MXC_V_SPIXFM_REVA_CFG_SSACT_FOR_4_MOD_CLK      ((uint32_t)0x2UL) /**< CFG_SSACT_FOR_4_MOD_CLK Value */
 #define MXC_S_SPIXFM_REVA_CFG_SSACT_FOR_4_MOD_CLK      (MXC_V_SPIXFM_REVA_CFG_SSACT_FOR_4_MOD_CLK << MXC_F_SPIXFM_REVA_CFG_SSACT_POS) /**< CFG_SSACT_FOR_4_MOD_CLK Setting */
 #define MXC_V_SPIXFM_REVA_CFG_SSACT_FOR_8_MOD_CLK      ((uint32_t)0x3UL) /**< CFG_SSACT_FOR_8_MOD_CLK Value */
 #define MXC_S_SPIXFM_REVA_CFG_SSACT_FOR_8_MOD_CLK      (MXC_V_SPIXFM_REVA_CFG_SSACT_FOR_8_MOD_CLK << MXC_F_SPIXFM_REVA_CFG_SSACT_POS) /**< CFG_SSACT_FOR_8_MOD_CLK Setting */

 #define MXC_F_SPIXFM_REVA_CFG_SSIACT_POS               18 /**< CFG_SSIACT Position */
 #define MXC_F_SPIXFM_REVA_CFG_SSIACT                   ((uint32_t)(0x3UL << MXC_F_SPIXFM_REVA_CFG_SSIACT_POS)) /**< CFG_SSIACT Mask */
 #define MXC_V_SPIXFM_REVA_CFG_SSIACT_FOR_1_MOD_CLK     ((uint32_t)0x0UL) /**< CFG_SSIACT_FOR_1_MOD_CLK Value */
 #define MXC_S_SPIXFM_REVA_CFG_SSIACT_FOR_1_MOD_CLK     (MXC_V_SPIXFM_REVA_CFG_SSIACT_FOR_1_MOD_CLK << MXC_F_SPIXFM_REVA_CFG_SSIACT_POS) /**< CFG_SSIACT_FOR_1_MOD_CLK Setting */
 #define MXC_V_SPIXFM_REVA_CFG_SSIACT_FOR_3_MOD_CLK     ((uint32_t)0x1UL) /**< CFG_SSIACT_FOR_3_MOD_CLK Value */
 #define MXC_S_SPIXFM_REVA_CFG_SSIACT_FOR_3_MOD_CLK     (MXC_V_SPIXFM_REVA_CFG_SSIACT_FOR_3_MOD_CLK << MXC_F_SPIXFM_REVA_CFG_SSIACT_POS) /**< CFG_SSIACT_FOR_3_MOD_CLK Setting */
 #define MXC_V_SPIXFM_REVA_CFG_SSIACT_FOR_5_MOD_CLK     ((uint32_t)0x2UL) /**< CFG_SSIACT_FOR_5_MOD_CLK Value */
 #define MXC_S_SPIXFM_REVA_CFG_SSIACT_FOR_5_MOD_CLK     (MXC_V_SPIXFM_REVA_CFG_SSIACT_FOR_5_MOD_CLK << MXC_F_SPIXFM_REVA_CFG_SSIACT_POS) /**< CFG_SSIACT_FOR_5_MOD_CLK Setting */
 #define MXC_V_SPIXFM_REVA_CFG_SSIACT_FOR_9_MOD_CLK     ((uint32_t)0x3UL) /**< CFG_SSIACT_FOR_9_MOD_CLK Value */
 #define MXC_S_SPIXFM_REVA_CFG_SSIACT_FOR_9_MOD_CLK     (MXC_V_SPIXFM_REVA_CFG_SSIACT_FOR_9_MOD_CLK << MXC_F_SPIXFM_REVA_CFG_SSIACT_POS) /**< CFG_SSIACT_FOR_9_MOD_CLK Setting */

/**@} end of group SPIXFM_REVA_CFG_Register */

/**
 * @ingroup  spixfm_reva_registers
 * @defgroup SPIXFM_REVA_FETCH_CTRL SPIXFM_REVA_FETCH_CTRL
 * @brief    SPIX Fetch Control Register.
 * @{
 */
 #define MXC_F_SPIXFM_REVA_FETCH_CTRL_CMDVAL_POS        0 /**< FETCH_CTRL_CMDVAL Position */
 #define MXC_F_SPIXFM_REVA_FETCH_CTRL_CMDVAL            ((uint32_t)(0xFFUL << MXC_F_SPIXFM_REVA_FETCH_CTRL_CMDVAL_POS)) /**< FETCH_CTRL_CMDVAL Mask */

 #define MXC_F_SPIXFM_REVA_FETCH_CTRL_CMD_WIDTH_POS     8 /**< FETCH_CTRL_CMD_WIDTH Position */
 #define MXC_F_SPIXFM_REVA_FETCH_CTRL_CMD_WIDTH         ((uint32_t)(0x3UL << MXC_F_SPIXFM_REVA_FETCH_CTRL_CMD_WIDTH_POS)) /**< FETCH_CTRL_CMD_WIDTH Mask */
 #define MXC_V_SPIXFM_REVA_FETCH_CTRL_CMD_WIDTH_SINGLE  ((uint32_t)0x0UL) /**< FETCH_CTRL_CMD_WIDTH_SINGLE Value */
 #define MXC_S_SPIXFM_REVA_FETCH_CTRL_CMD_WIDTH_SINGLE  (MXC_V_SPIXFM_REVA_FETCH_CTRL_CMD_WIDTH_SINGLE << MXC_F_SPIXFM_REVA_FETCH_CTRL_CMD_WIDTH_POS) /**< FETCH_CTRL_CMD_WIDTH_SINGLE Setting */
 #define MXC_V_SPIXFM_REVA_FETCH_CTRL_CMD_WIDTH_DUAL_IO ((uint32_t)0x1UL) /**< FETCH_CTRL_CMD_WIDTH_DUAL_IO Value */
 #define MXC_S_SPIXFM_REVA_FETCH_CTRL_CMD_WIDTH_DUAL_IO (MXC_V_SPIXFM_REVA_FETCH_CTRL_CMD_WIDTH_DUAL_IO << MXC_F_SPIXFM_REVA_FETCH_CTRL_CMD_WIDTH_POS) /**< FETCH_CTRL_CMD_WIDTH_DUAL_IO Setting */
 #define MXC_V_SPIXFM_REVA_FETCH_CTRL_CMD_WIDTH_QUAD_IO ((uint32_t)0x2UL) /**< FETCH_CTRL_CMD_WIDTH_QUAD_IO Value */
 #define MXC_S_SPIXFM_REVA_FETCH_CTRL_CMD_WIDTH_QUAD_IO (MXC_V_SPIXFM_REVA_FETCH_CTRL_CMD_WIDTH_QUAD_IO << MXC_F_SPIXFM_REVA_FETCH_CTRL_CMD_WIDTH_POS) /**< FETCH_CTRL_CMD_WIDTH_QUAD_IO Setting */
 #define MXC_V_SPIXFM_REVA_FETCH_CTRL_CMD_WIDTH_INVALID ((uint32_t)0x3UL) /**< FETCH_CTRL_CMD_WIDTH_INVALID Value */
 #define MXC_S_SPIXFM_REVA_FETCH_CTRL_CMD_WIDTH_INVALID (MXC_V_SPIXFM_REVA_FETCH_CTRL_CMD_WIDTH_INVALID << MXC_F_SPIXFM_REVA_FETCH_CTRL_CMD_WIDTH_POS) /**< FETCH_CTRL_CMD_WIDTH_INVALID Setting */

 #define MXC_F_SPIXFM_REVA_FETCH_CTRL_ADDR_WIDTH_POS    10 /**< FETCH_CTRL_ADDR_WIDTH Position */
 #define MXC_F_SPIXFM_REVA_FETCH_CTRL_ADDR_WIDTH        ((uint32_t)(0x3UL << MXC_F_SPIXFM_REVA_FETCH_CTRL_ADDR_WIDTH_POS)) /**< FETCH_CTRL_ADDR_WIDTH Mask */
 #define MXC_V_SPIXFM_REVA_FETCH_CTRL_ADDR_WIDTH_SINGLE ((uint32_t)0x0UL) /**< FETCH_CTRL_ADDR_WIDTH_SINGLE Value */
 #define MXC_S_SPIXFM_REVA_FETCH_CTRL_ADDR_WIDTH_SINGLE (MXC_V_SPIXFM_REVA_FETCH_CTRL_ADDR_WIDTH_SINGLE << MXC_F_SPIXFM_REVA_FETCH_CTRL_ADDR_WIDTH_POS) /**< FETCH_CTRL_ADDR_WIDTH_SINGLE Setting */
 #define MXC_V_SPIXFM_REVA_FETCH_CTRL_ADDR_WIDTH_DUAL_IO ((uint32_t)0x1UL) /**< FETCH_CTRL_ADDR_WIDTH_DUAL_IO Value */
 #define MXC_S_SPIXFM_REVA_FETCH_CTRL_ADDR_WIDTH_DUAL_IO (MXC_V_SPIXFM_REVA_FETCH_CTRL_ADDR_WIDTH_DUAL_IO << MXC_F_SPIXFM_REVA_FETCH_CTRL_ADDR_WIDTH_POS) /**< FETCH_CTRL_ADDR_WIDTH_DUAL_IO Setting */
 #define MXC_V_SPIXFM_REVA_FETCH_CTRL_ADDR_WIDTH_QUAD_IO ((uint32_t)0x2UL) /**< FETCH_CTRL_ADDR_WIDTH_QUAD_IO Value */
 #define MXC_S_SPIXFM_REVA_FETCH_CTRL_ADDR_WIDTH_QUAD_IO (MXC_V_SPIXFM_REVA_FETCH_CTRL_ADDR_WIDTH_QUAD_IO << MXC_F_SPIXFM_REVA_FETCH_CTRL_ADDR_WIDTH_POS) /**< FETCH_CTRL_ADDR_WIDTH_QUAD_IO Setting */
 #define MXC_V_SPIXFM_REVA_FETCH_CTRL_ADDR_WIDTH_INVALID ((uint32_t)0x3UL) /**< FETCH_CTRL_ADDR_WIDTH_INVALID Value */
 #define MXC_S_SPIXFM_REVA_FETCH_CTRL_ADDR_WIDTH_INVALID (MXC_V_SPIXFM_REVA_FETCH_CTRL_ADDR_WIDTH_INVALID << MXC_F_SPIXFM_REVA_FETCH_CTRL_ADDR_WIDTH_POS) /**< FETCH_CTRL_ADDR_WIDTH_INVALID Setting */

 #define MXC_F_SPIXFM_REVA_FETCH_CTRL_DATA_WIDTH_POS    12 /**< FETCH_CTRL_DATA_WIDTH Position */
 #define MXC_F_SPIXFM_REVA_FETCH_CTRL_DATA_WIDTH        ((uint32_t)(0x3UL << MXC_F_SPIXFM_REVA_FETCH_CTRL_DATA_WIDTH_POS)) /**< FETCH_CTRL_DATA_WIDTH Mask */
 #define MXC_V_SPIXFM_REVA_FETCH_CTRL_DATA_WIDTH_SINGLE ((uint32_t)0x0UL) /**< FETCH_CTRL_DATA_WIDTH_SINGLE Value */
 #define MXC_S_SPIXFM_REVA_FETCH_CTRL_DATA_WIDTH_SINGLE (MXC_V_SPIXFM_REVA_FETCH_CTRL_DATA_WIDTH_SINGLE << MXC_F_SPIXFM_REVA_FETCH_CTRL_DATA_WIDTH_POS) /**< FETCH_CTRL_DATA_WIDTH_SINGLE Setting */
 #define MXC_V_SPIXFM_REVA_FETCH_CTRL_DATA_WIDTH_DUAL_IO ((uint32_t)0x1UL) /**< FETCH_CTRL_DATA_WIDTH_DUAL_IO Value */
 #define MXC_S_SPIXFM_REVA_FETCH_CTRL_DATA_WIDTH_DUAL_IO (MXC_V_SPIXFM_REVA_FETCH_CTRL_DATA_WIDTH_DUAL_IO << MXC_F_SPIXFM_REVA_FETCH_CTRL_DATA_WIDTH_POS) /**< FETCH_CTRL_DATA_WIDTH_DUAL_IO Setting */
 #define MXC_V_SPIXFM_REVA_FETCH_CTRL_DATA_WIDTH_QUAD_IO ((uint32_t)0x2UL) /**< FETCH_CTRL_DATA_WIDTH_QUAD_IO Value */
 #define MXC_S_SPIXFM_REVA_FETCH_CTRL_DATA_WIDTH_QUAD_IO (MXC_V_SPIXFM_REVA_FETCH_CTRL_DATA_WIDTH_QUAD_IO << MXC_F_SPIXFM_REVA_FETCH_CTRL_DATA_WIDTH_POS) /**< FETCH_CTRL_DATA_WIDTH_QUAD_IO Setting */
 #define MXC_V_SPIXFM_REVA_FETCH_CTRL_DATA_WIDTH_INVALID ((uint32_t)0x3UL) /**< FETCH_CTRL_DATA_WIDTH_INVALID Value */
 #define MXC_S_SPIXFM_REVA_FETCH_CTRL_DATA_WIDTH_INVALID (MXC_V_SPIXFM_REVA_FETCH_CTRL_DATA_WIDTH_INVALID << MXC_F_SPIXFM_REVA_FETCH_CTRL_DATA_WIDTH_POS) /**< FETCH_CTRL_DATA_WIDTH_INVALID Setting */

 #define MXC_F_SPIXFM_REVA_FETCH_CTRL_FOUR_BYTE_ADDR_POS 16 /**< FETCH_CTRL_FOUR_BYTE_ADDR Position */
 #define MXC_F_SPIXFM_REVA_FETCH_CTRL_FOUR_BYTE_ADDR    ((uint32_t)(0x1UL << MXC_F_SPIXFM_REVA_FETCH_CTRL_FOUR_BYTE_ADDR_POS)) /**< FETCH_CTRL_FOUR_BYTE_ADDR Mask */

/**@} end of group SPIXFM_REVA_FETCH_CTRL_Register */

/**
 * @ingroup  spixfm_reva_registers
 * @defgroup SPIXFM_REVA_MODE_CTRL SPIXFM_REVA_MODE_CTRL
 * @brief    SPIX Mode Control Register.
 * @{
 */
 #define MXC_F_SPIXFM_REVA_MODE_CTRL_MDCLK_POS          0 /**< MODE_CTRL_MDCLK Position */
 #define MXC_F_SPIXFM_REVA_MODE_CTRL_MDCLK              ((uint32_t)(0xFUL << MXC_F_SPIXFM_REVA_MODE_CTRL_MDCLK_POS)) /**< MODE_CTRL_MDCLK Mask */

 #define MXC_F_SPIXFM_REVA_MODE_CTRL_NO_CMD_POS         8 /**< MODE_CTRL_NO_CMD Position */
 #define MXC_F_SPIXFM_REVA_MODE_CTRL_NO_CMD             ((uint32_t)(0x1UL << MXC_F_SPIXFM_REVA_MODE_CTRL_NO_CMD_POS)) /**< MODE_CTRL_NO_CMD Mask */

 #define MXC_F_SPIXFM_REVA_MODE_CTRL_MODE_SEND_POS      9 /**< MODE_CTRL_MODE_SEND Position */
 #define MXC_F_SPIXFM_REVA_MODE_CTRL_MODE_SEND          ((uint32_t)(0x1UL << MXC_F_SPIXFM_REVA_MODE_CTRL_MODE_SEND_POS)) /**< MODE_CTRL_MODE_SEND Mask */

/**@} end of group SPIXFM_REVA_MODE_CTRL_Register */

/**
 * @ingroup  spixfm_reva_registers
 * @defgroup SPIXFM_REVA_MODE_DATA SPIXFM_REVA_MODE_DATA
 * @brief    SPIX Mode Data Register.
 * @{
 */
 #define MXC_F_SPIXFM_REVA_MODE_DATA_DATA_POS           0 /**< MODE_DATA_DATA Position */
 #define MXC_F_SPIXFM_REVA_MODE_DATA_DATA               ((uint32_t)(0xFFFFUL << MXC_F_SPIXFM_REVA_MODE_DATA_DATA_POS)) /**< MODE_DATA_DATA Mask */

 #define MXC_F_SPIXFM_REVA_MODE_DATA_OUT_EN_POS         16 /**< MODE_DATA_OUT_EN Position */
 #define MXC_F_SPIXFM_REVA_MODE_DATA_OUT_EN             ((uint32_t)(0xFFFFUL << MXC_F_SPIXFM_REVA_MODE_DATA_OUT_EN_POS)) /**< MODE_DATA_OUT_EN Mask */

/**@} end of group SPIXFM_REVA_MODE_DATA_Register */

/**
 * @ingroup  spixfm_reva_registers
 * @defgroup SPIXFM_REVA_FB_CTRL SPIXFM_REVA_FB_CTRL
 * @brief    SPIX Feedback Control Register.
 * @{
 */
 #define MXC_F_SPIXFM_REVA_FB_CTRL_FB_EN_POS            0 /**< FB_CTRL_FB_EN Position */
 #define MXC_F_SPIXFM_REVA_FB_CTRL_FB_EN                ((uint32_t)(0x1UL << MXC_F_SPIXFM_REVA_FB_CTRL_FB_EN_POS)) /**< FB_CTRL_FB_EN Mask */

 #define MXC_F_SPIXFM_REVA_FB_CTRL_INVERT_EN_POS        1 /**< FB_CTRL_INVERT_EN Position */
 #define MXC_F_SPIXFM_REVA_FB_CTRL_INVERT_EN            ((uint32_t)(0x1UL << MXC_F_SPIXFM_REVA_FB_CTRL_INVERT_EN_POS)) /**< FB_CTRL_INVERT_EN Mask */

/**@} end of group SPIXFM_REVA_FB_CTRL_Register */

/**
 * @ingroup  spixfm_reva_registers
 * @defgroup SPIXFM_REVA_IO_CTRL SPIXFM_REVA_IO_CTRL
 * @brief    SPIX IO Control Register.
 * @{
 */
 #define MXC_F_SPIXFM_REVA_IO_CTRL_SCLK_DS_POS          0 /**< IO_CTRL_SCLK_DS Position */
 #define MXC_F_SPIXFM_REVA_IO_CTRL_SCLK_DS              ((uint32_t)(0x1UL << MXC_F_SPIXFM_REVA_IO_CTRL_SCLK_DS_POS)) /**< IO_CTRL_SCLK_DS Mask */

 #define MXC_F_SPIXFM_REVA_IO_CTRL_SS_DS_POS            1 /**< IO_CTRL_SS_DS Position */
 #define MXC_F_SPIXFM_REVA_IO_CTRL_SS_DS                ((uint32_t)(0x1UL << MXC_F_SPIXFM_REVA_IO_CTRL_SS_DS_POS)) /**< IO_CTRL_SS_DS Mask */

 #define MXC_F_SPIXFM_REVA_IO_CTRL_SDIO_DS_POS          2 /**< IO_CTRL_SDIO_DS Position */
 #define MXC_F_SPIXFM_REVA_IO_CTRL_SDIO_DS              ((uint32_t)(0x1UL << MXC_F_SPIXFM_REVA_IO_CTRL_SDIO_DS_POS)) /**< IO_CTRL_SDIO_DS Mask */

 #define MXC_F_SPIXFM_REVA_IO_CTRL_PU_PD_CTRL_POS       3 /**< IO_CTRL_PU_PD_CTRL Position */
 #define MXC_F_SPIXFM_REVA_IO_CTRL_PU_PD_CTRL           ((uint32_t)(0x3UL << MXC_F_SPIXFM_REVA_IO_CTRL_PU_PD_CTRL_POS)) /**< IO_CTRL_PU_PD_CTRL Mask */
 #define MXC_V_SPIXFM_REVA_IO_CTRL_PU_PD_CTRL_TRI_STATE ((uint32_t)0x0UL) /**< IO_CTRL_PU_PD_CTRL_TRI_STATE Value */
 #define MXC_S_SPIXFM_REVA_IO_CTRL_PU_PD_CTRL_TRI_STATE (MXC_V_SPIXFM_REVA_IO_CTRL_PU_PD_CTRL_TRI_STATE << MXC_F_SPIXFM_REVA_IO_CTRL_PU_PD_CTRL_POS) /**< IO_CTRL_PU_PD_CTRL_TRI_STATE Setting */
 #define MXC_V_SPIXFM_REVA_IO_CTRL_PU_PD_CTRL_PULL_UP   ((uint32_t)0x1UL) /**< IO_CTRL_PU_PD_CTRL_PULL_UP Value */
 #define MXC_S_SPIXFM_REVA_IO_CTRL_PU_PD_CTRL_PULL_UP   (MXC_V_SPIXFM_REVA_IO_CTRL_PU_PD_CTRL_PULL_UP << MXC_F_SPIXFM_REVA_IO_CTRL_PU_PD_CTRL_POS) /**< IO_CTRL_PU_PD_CTRL_PULL_UP Setting */
 #define MXC_V_SPIXFM_REVA_IO_CTRL_PU_PD_CTRL_PULL_DOWN ((uint32_t)0x2UL) /**< IO_CTRL_PU_PD_CTRL_PULL_DOWN Value */
 #define MXC_S_SPIXFM_REVA_IO_CTRL_PU_PD_CTRL_PULL_DOWN (MXC_V_SPIXFM_REVA_IO_CTRL_PU_PD_CTRL_PULL_DOWN << MXC_F_SPIXFM_REVA_IO_CTRL_PU_PD_CTRL_POS) /**< IO_CTRL_PU_PD_CTRL_PULL_DOWN Setting */

/**@} end of group SPIXFM_REVA_IO_CTRL_Register */

/**
 * @ingroup  spixfm_reva_registers
 * @defgroup SPIXFM_REVA_SEC_CTRL SPIXFM_REVA_SEC_CTRL
 * @brief    SPIX Memory Security Control Register.
 * @{
 */
 #define MXC_F_SPIXFM_REVA_SEC_CTRL_DEC_EN_POS          0 /**< SEC_CTRL_DEC_EN Position */
 #define MXC_F_SPIXFM_REVA_SEC_CTRL_DEC_EN              ((uint32_t)(0x1UL << MXC_F_SPIXFM_REVA_SEC_CTRL_DEC_EN_POS)) /**< SEC_CTRL_DEC_EN Mask */

 #define MXC_F_SPIXFM_REVA_SEC_CTRL_AUTH_DISABLE_POS    1 /**< SEC_CTRL_AUTH_DISABLE Position */
 #define MXC_F_SPIXFM_REVA_SEC_CTRL_AUTH_DISABLE        ((uint32_t)(0x1UL << MXC_F_SPIXFM_REVA_SEC_CTRL_AUTH_DISABLE_POS)) /**< SEC_CTRL_AUTH_DISABLE Mask */

/**@} end of group SPIXFM_REVA_SEC_CTRL_Register */

/**
 * @ingroup  spixfm_reva_registers
 * @defgroup SPIXFM_REVA_BUS_IDLE SPIXFM_REVA_BUS_IDLE
 * @brief    Bus Idle
 * @{
 */
 #define MXC_F_SPIXFM_REVA_BUS_IDLE_BUSIDLE_POS         0 /**< BUS_IDLE_BUSIDLE Position */
 #define MXC_F_SPIXFM_REVA_BUS_IDLE_BUSIDLE             ((uint32_t)(0xFFFFUL << MXC_F_SPIXFM_REVA_BUS_IDLE_BUSIDLE_POS)) /**< BUS_IDLE_BUSIDLE Mask */

/**@} end of group SPIXFM_REVA_BUS_IDLE_Register */

#ifdef __cplusplus
}
#endif

#endif /* _SPIXFM_REVA_REGS_H_ */
