/**
 * @file    spixf_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the SPIXF Peripheral Module.
 * @note    This file is @generated.
 * @ingroup spixf_registers
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32650_INCLUDE_SPIXF_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32650_INCLUDE_SPIXF_REGS_H_

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
 * @ingroup     spixf
 * @defgroup    spixf_registers SPIXF_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the SPIXF Peripheral Module.
 * @details     SPIXF Master
 */

/**
 * @ingroup spixf_registers
 * Structure type to access the SPIXF Registers.
 */
typedef struct {
    __IO uint32_t cfg;                  /**< <tt>\b 0x00:</tt> SPIXF CFG Register */
    __IO uint32_t fetch_ctrl;           /**< <tt>\b 0x04:</tt> SPIXF FETCH_CTRL Register */
    __IO uint32_t mode_ctrl;            /**< <tt>\b 0x08:</tt> SPIXF MODE_CTRL Register */
    __IO uint32_t mode_data;            /**< <tt>\b 0x0C:</tt> SPIXF MODE_DATA Register */
    __IO uint32_t fb_ctrl;              /**< <tt>\b 0x10:</tt> SPIXF FB_CTRL Register */
    __R  uint32_t rsv_0x14_0x1b[2];
    __IO uint32_t io_ctrl;              /**< <tt>\b 0x1C:</tt> SPIXF IO_CTRL Register */
    __IO uint32_t sec_ctrl;             /**< <tt>\b 0x20:</tt> SPIXF SEC_CTRL Register */
    __IO uint32_t bus_idle;             /**< <tt>\b 0x24:</tt> SPIXF BUS_IDLE Register */
} mxc_spixf_regs_t;

/* Register offsets for module SPIXF */
/**
 * @ingroup    spixf_registers
 * @defgroup   SPIXF_Register_Offsets Register Offsets
 * @brief      SPIXF Peripheral Register Offsets from the SPIXF Base Peripheral Address.
 * @{
 */
#define MXC_R_SPIXF_CFG                    ((uint32_t)0x00000000UL) /**< Offset from SPIXF Base Address: <tt> 0x0000</tt> */
#define MXC_R_SPIXF_FETCH_CTRL             ((uint32_t)0x00000004UL) /**< Offset from SPIXF Base Address: <tt> 0x0004</tt> */
#define MXC_R_SPIXF_MODE_CTRL              ((uint32_t)0x00000008UL) /**< Offset from SPIXF Base Address: <tt> 0x0008</tt> */
#define MXC_R_SPIXF_MODE_DATA              ((uint32_t)0x0000000CUL) /**< Offset from SPIXF Base Address: <tt> 0x000C</tt> */
#define MXC_R_SPIXF_FB_CTRL                ((uint32_t)0x00000010UL) /**< Offset from SPIXF Base Address: <tt> 0x0010</tt> */
#define MXC_R_SPIXF_IO_CTRL                ((uint32_t)0x0000001CUL) /**< Offset from SPIXF Base Address: <tt> 0x001C</tt> */
#define MXC_R_SPIXF_SEC_CTRL               ((uint32_t)0x00000020UL) /**< Offset from SPIXF Base Address: <tt> 0x0020</tt> */
#define MXC_R_SPIXF_BUS_IDLE               ((uint32_t)0x00000024UL) /**< Offset from SPIXF Base Address: <tt> 0x0024</tt> */
/**@} end of group spixf_registers */

/**
 * @ingroup  spixf_registers
 * @defgroup SPIXF_CFG SPIXF_CFG
 * @brief    SPIX Configuration Register.
 * @{
 */
#define MXC_F_SPIXF_CFG_MODE_POS                       0 /**< CFG_MODE Position */
#define MXC_F_SPIXF_CFG_MODE                           ((uint32_t)(0x3UL << MXC_F_SPIXF_CFG_MODE_POS)) /**< CFG_MODE Mask */
#define MXC_V_SPIXF_CFG_MODE_MODE0                     ((uint32_t)0x0UL) /**< CFG_MODE_MODE0 Value */
#define MXC_S_SPIXF_CFG_MODE_MODE0                     (MXC_V_SPIXF_CFG_MODE_MODE0 << MXC_F_SPIXF_CFG_MODE_POS) /**< CFG_MODE_MODE0 Setting */
#define MXC_V_SPIXF_CFG_MODE_MODE3                     ((uint32_t)0x3UL) /**< CFG_MODE_MODE3 Value */
#define MXC_S_SPIXF_CFG_MODE_MODE3                     (MXC_V_SPIXF_CFG_MODE_MODE3 << MXC_F_SPIXF_CFG_MODE_POS) /**< CFG_MODE_MODE3 Setting */

#define MXC_F_SPIXF_CFG_SSPOL_POS                      2 /**< CFG_SSPOL Position */
#define MXC_F_SPIXF_CFG_SSPOL                          ((uint32_t)(0x1UL << MXC_F_SPIXF_CFG_SSPOL_POS)) /**< CFG_SSPOL Mask */
#define MXC_V_SPIXF_CFG_SSPOL_ACTIVEHI                 ((uint32_t)0x0UL) /**< CFG_SSPOL_ACTIVEHI Value */
#define MXC_S_SPIXF_CFG_SSPOL_ACTIVEHI                 (MXC_V_SPIXF_CFG_SSPOL_ACTIVEHI << MXC_F_SPIXF_CFG_SSPOL_POS) /**< CFG_SSPOL_ACTIVEHI Setting */
#define MXC_V_SPIXF_CFG_SSPOL_ACTIVELO                 ((uint32_t)0x1UL) /**< CFG_SSPOL_ACTIVELO Value */
#define MXC_S_SPIXF_CFG_SSPOL_ACTIVELO                 (MXC_V_SPIXF_CFG_SSPOL_ACTIVELO << MXC_F_SPIXF_CFG_SSPOL_POS) /**< CFG_SSPOL_ACTIVELO Setting */

#define MXC_F_SPIXF_CFG_SSEL_POS                       4 /**< CFG_SSEL Position */
#define MXC_F_SPIXF_CFG_SSEL                           ((uint32_t)(0x7UL << MXC_F_SPIXF_CFG_SSEL_POS)) /**< CFG_SSEL Mask */
#define MXC_V_SPIXF_CFG_SSEL_SLAVE0                    ((uint32_t)0x0UL) /**< CFG_SSEL_SLAVE0 Value */
#define MXC_S_SPIXF_CFG_SSEL_SLAVE0                    (MXC_V_SPIXF_CFG_SSEL_SLAVE0 << MXC_F_SPIXF_CFG_SSEL_POS) /**< CFG_SSEL_SLAVE0 Setting */

#define MXC_F_SPIXF_CFG_LOCLK_POS                      8 /**< CFG_LOCLK Position */
#define MXC_F_SPIXF_CFG_LOCLK                          ((uint32_t)(0xFUL << MXC_F_SPIXF_CFG_LOCLK_POS)) /**< CFG_LOCLK Mask */

#define MXC_F_SPIXF_CFG_HICLK_POS                      12 /**< CFG_HICLK Position */
#define MXC_F_SPIXF_CFG_HICLK                          ((uint32_t)(0xFUL << MXC_F_SPIXF_CFG_HICLK_POS)) /**< CFG_HICLK Mask */

#define MXC_F_SPIXF_CFG_SSACT_POS                      16 /**< CFG_SSACT Position */
#define MXC_F_SPIXF_CFG_SSACT                          ((uint32_t)(0x3UL << MXC_F_SPIXF_CFG_SSACT_POS)) /**< CFG_SSACT Mask */
#define MXC_V_SPIXF_CFG_SSACT_OFF                      ((uint32_t)0x0UL) /**< CFG_SSACT_OFF Value */
#define MXC_S_SPIXF_CFG_SSACT_OFF                      (MXC_V_SPIXF_CFG_SSACT_OFF << MXC_F_SPIXF_CFG_SSACT_POS) /**< CFG_SSACT_OFF Setting */
#define MXC_V_SPIXF_CFG_SSACT_2CLK                     ((uint32_t)0x1UL) /**< CFG_SSACT_2CLK Value */
#define MXC_S_SPIXF_CFG_SSACT_2CLK                     (MXC_V_SPIXF_CFG_SSACT_2CLK << MXC_F_SPIXF_CFG_SSACT_POS) /**< CFG_SSACT_2CLK Setting */
#define MXC_V_SPIXF_CFG_SSACT_4CLK                     ((uint32_t)0x2UL) /**< CFG_SSACT_4CLK Value */
#define MXC_S_SPIXF_CFG_SSACT_4CLK                     (MXC_V_SPIXF_CFG_SSACT_4CLK << MXC_F_SPIXF_CFG_SSACT_POS) /**< CFG_SSACT_4CLK Setting */
#define MXC_V_SPIXF_CFG_SSACT_8CLK                     ((uint32_t)0x3UL) /**< CFG_SSACT_8CLK Value */
#define MXC_S_SPIXF_CFG_SSACT_8CLK                     (MXC_V_SPIXF_CFG_SSACT_8CLK << MXC_F_SPIXF_CFG_SSACT_POS) /**< CFG_SSACT_8CLK Setting */

#define MXC_F_SPIXF_CFG_SSIACT_POS                     18 /**< CFG_SSIACT Position */
#define MXC_F_SPIXF_CFG_SSIACT                         ((uint32_t)(0x3UL << MXC_F_SPIXF_CFG_SSIACT_POS)) /**< CFG_SSIACT Mask */
#define MXC_V_SPIXF_CFG_SSIACT_1CLK                    ((uint32_t)0x0UL) /**< CFG_SSIACT_1CLK Value */
#define MXC_S_SPIXF_CFG_SSIACT_1CLK                    (MXC_V_SPIXF_CFG_SSIACT_1CLK << MXC_F_SPIXF_CFG_SSIACT_POS) /**< CFG_SSIACT_1CLK Setting */
#define MXC_V_SPIXF_CFG_SSIACT_3CLK                    ((uint32_t)0x1UL) /**< CFG_SSIACT_3CLK Value */
#define MXC_S_SPIXF_CFG_SSIACT_3CLK                    (MXC_V_SPIXF_CFG_SSIACT_3CLK << MXC_F_SPIXF_CFG_SSIACT_POS) /**< CFG_SSIACT_3CLK Setting */
#define MXC_V_SPIXF_CFG_SSIACT_5CLK                    ((uint32_t)0x2UL) /**< CFG_SSIACT_5CLK Value */
#define MXC_S_SPIXF_CFG_SSIACT_5CLK                    (MXC_V_SPIXF_CFG_SSIACT_5CLK << MXC_F_SPIXF_CFG_SSIACT_POS) /**< CFG_SSIACT_5CLK Setting */
#define MXC_V_SPIXF_CFG_SSIACT_9CLK                    ((uint32_t)0x3UL) /**< CFG_SSIACT_9CLK Value */
#define MXC_S_SPIXF_CFG_SSIACT_9CLK                    (MXC_V_SPIXF_CFG_SSIACT_9CLK << MXC_F_SPIXF_CFG_SSIACT_POS) /**< CFG_SSIACT_9CLK Setting */

/**@} end of group SPIXF_CFG_Register */

/**
 * @ingroup  spixf_registers
 * @defgroup SPIXF_FETCH_CTRL SPIXF_FETCH_CTRL
 * @brief    SPIX Fetch Control Register.
 * @{
 */
#define MXC_F_SPIXF_FETCH_CTRL_CMDVAL_POS              0 /**< FETCH_CTRL_CMDVAL Position */
#define MXC_F_SPIXF_FETCH_CTRL_CMDVAL                  ((uint32_t)(0xFFUL << MXC_F_SPIXF_FETCH_CTRL_CMDVAL_POS)) /**< FETCH_CTRL_CMDVAL Mask */

#define MXC_F_SPIXF_FETCH_CTRL_CMDWTH_POS              8 /**< FETCH_CTRL_CMDWTH Position */
#define MXC_F_SPIXF_FETCH_CTRL_CMDWTH                  ((uint32_t)(0x3UL << MXC_F_SPIXF_FETCH_CTRL_CMDWTH_POS)) /**< FETCH_CTRL_CMDWTH Mask */
#define MXC_V_SPIXF_FETCH_CTRL_CMDWTH_MONO             ((uint32_t)0x0UL) /**< FETCH_CTRL_CMDWTH_MONO Value */
#define MXC_S_SPIXF_FETCH_CTRL_CMDWTH_MONO             (MXC_V_SPIXF_FETCH_CTRL_CMDWTH_MONO << MXC_F_SPIXF_FETCH_CTRL_CMDWTH_POS) /**< FETCH_CTRL_CMDWTH_MONO Setting */
#define MXC_V_SPIXF_FETCH_CTRL_CMDWTH_DUAL             ((uint32_t)0x1UL) /**< FETCH_CTRL_CMDWTH_DUAL Value */
#define MXC_S_SPIXF_FETCH_CTRL_CMDWTH_DUAL             (MXC_V_SPIXF_FETCH_CTRL_CMDWTH_DUAL << MXC_F_SPIXF_FETCH_CTRL_CMDWTH_POS) /**< FETCH_CTRL_CMDWTH_DUAL Setting */
#define MXC_V_SPIXF_FETCH_CTRL_CMDWTH_QUAD             ((uint32_t)0x2UL) /**< FETCH_CTRL_CMDWTH_QUAD Value */
#define MXC_S_SPIXF_FETCH_CTRL_CMDWTH_QUAD             (MXC_V_SPIXF_FETCH_CTRL_CMDWTH_QUAD << MXC_F_SPIXF_FETCH_CTRL_CMDWTH_POS) /**< FETCH_CTRL_CMDWTH_QUAD Setting */
#define MXC_V_SPIXF_FETCH_CTRL_CMDWTH_INVALID          ((uint32_t)0x3UL) /**< FETCH_CTRL_CMDWTH_INVALID Value */
#define MXC_S_SPIXF_FETCH_CTRL_CMDWTH_INVALID          (MXC_V_SPIXF_FETCH_CTRL_CMDWTH_INVALID << MXC_F_SPIXF_FETCH_CTRL_CMDWTH_POS) /**< FETCH_CTRL_CMDWTH_INVALID Setting */

#define MXC_F_SPIXF_FETCH_CTRL_ADDR_WIDTH_POS          10 /**< FETCH_CTRL_ADDR_WIDTH Position */
#define MXC_F_SPIXF_FETCH_CTRL_ADDR_WIDTH              ((uint32_t)(0x3UL << MXC_F_SPIXF_FETCH_CTRL_ADDR_WIDTH_POS)) /**< FETCH_CTRL_ADDR_WIDTH Mask */
#define MXC_V_SPIXF_FETCH_CTRL_ADDR_WIDTH_SINGLE       ((uint32_t)0x0UL) /**< FETCH_CTRL_ADDR_WIDTH_SINGLE Value */
#define MXC_S_SPIXF_FETCH_CTRL_ADDR_WIDTH_SINGLE       (MXC_V_SPIXF_FETCH_CTRL_ADDR_WIDTH_SINGLE << MXC_F_SPIXF_FETCH_CTRL_ADDR_WIDTH_POS) /**< FETCH_CTRL_ADDR_WIDTH_SINGLE Setting */
#define MXC_V_SPIXF_FETCH_CTRL_ADDR_WIDTH_DUAL         ((uint32_t)0x1UL) /**< FETCH_CTRL_ADDR_WIDTH_DUAL Value */
#define MXC_S_SPIXF_FETCH_CTRL_ADDR_WIDTH_DUAL         (MXC_V_SPIXF_FETCH_CTRL_ADDR_WIDTH_DUAL << MXC_F_SPIXF_FETCH_CTRL_ADDR_WIDTH_POS) /**< FETCH_CTRL_ADDR_WIDTH_DUAL Setting */
#define MXC_V_SPIXF_FETCH_CTRL_ADDR_WIDTH_QUAD         ((uint32_t)0x2UL) /**< FETCH_CTRL_ADDR_WIDTH_QUAD Value */
#define MXC_S_SPIXF_FETCH_CTRL_ADDR_WIDTH_QUAD         (MXC_V_SPIXF_FETCH_CTRL_ADDR_WIDTH_QUAD << MXC_F_SPIXF_FETCH_CTRL_ADDR_WIDTH_POS) /**< FETCH_CTRL_ADDR_WIDTH_QUAD Setting */
#define MXC_V_SPIXF_FETCH_CTRL_ADDR_WIDTH_INVALID      ((uint32_t)0x3UL) /**< FETCH_CTRL_ADDR_WIDTH_INVALID Value */
#define MXC_S_SPIXF_FETCH_CTRL_ADDR_WIDTH_INVALID      (MXC_V_SPIXF_FETCH_CTRL_ADDR_WIDTH_INVALID << MXC_F_SPIXF_FETCH_CTRL_ADDR_WIDTH_POS) /**< FETCH_CTRL_ADDR_WIDTH_INVALID Setting */

#define MXC_F_SPIXF_FETCH_CTRL_DATA_WIDTH_POS          12 /**< FETCH_CTRL_DATA_WIDTH Position */
#define MXC_F_SPIXF_FETCH_CTRL_DATA_WIDTH              ((uint32_t)(0x3UL << MXC_F_SPIXF_FETCH_CTRL_DATA_WIDTH_POS)) /**< FETCH_CTRL_DATA_WIDTH Mask */
#define MXC_V_SPIXF_FETCH_CTRL_DATA_WIDTH_SINGLE       ((uint32_t)0x0UL) /**< FETCH_CTRL_DATA_WIDTH_SINGLE Value */
#define MXC_S_SPIXF_FETCH_CTRL_DATA_WIDTH_SINGLE       (MXC_V_SPIXF_FETCH_CTRL_DATA_WIDTH_SINGLE << MXC_F_SPIXF_FETCH_CTRL_DATA_WIDTH_POS) /**< FETCH_CTRL_DATA_WIDTH_SINGLE Setting */
#define MXC_V_SPIXF_FETCH_CTRL_DATA_WIDTH_DUAL         ((uint32_t)0x1UL) /**< FETCH_CTRL_DATA_WIDTH_DUAL Value */
#define MXC_S_SPIXF_FETCH_CTRL_DATA_WIDTH_DUAL         (MXC_V_SPIXF_FETCH_CTRL_DATA_WIDTH_DUAL << MXC_F_SPIXF_FETCH_CTRL_DATA_WIDTH_POS) /**< FETCH_CTRL_DATA_WIDTH_DUAL Setting */
#define MXC_V_SPIXF_FETCH_CTRL_DATA_WIDTH_QUAD         ((uint32_t)0x2UL) /**< FETCH_CTRL_DATA_WIDTH_QUAD Value */
#define MXC_S_SPIXF_FETCH_CTRL_DATA_WIDTH_QUAD         (MXC_V_SPIXF_FETCH_CTRL_DATA_WIDTH_QUAD << MXC_F_SPIXF_FETCH_CTRL_DATA_WIDTH_POS) /**< FETCH_CTRL_DATA_WIDTH_QUAD Setting */
#define MXC_V_SPIXF_FETCH_CTRL_DATA_WIDTH_INVALID      ((uint32_t)0x3UL) /**< FETCH_CTRL_DATA_WIDTH_INVALID Value */
#define MXC_S_SPIXF_FETCH_CTRL_DATA_WIDTH_INVALID      (MXC_V_SPIXF_FETCH_CTRL_DATA_WIDTH_INVALID << MXC_F_SPIXF_FETCH_CTRL_DATA_WIDTH_POS) /**< FETCH_CTRL_DATA_WIDTH_INVALID Setting */

#define MXC_F_SPIXF_FETCH_CTRL_ADDR4_POS               16 /**< FETCH_CTRL_ADDR4 Position */
#define MXC_F_SPIXF_FETCH_CTRL_ADDR4                   ((uint32_t)(0x1UL << MXC_F_SPIXF_FETCH_CTRL_ADDR4_POS)) /**< FETCH_CTRL_ADDR4 Mask */
#define MXC_V_SPIXF_FETCH_CTRL_ADDR4_3BYTE             ((uint32_t)0x0UL) /**< FETCH_CTRL_ADDR4_3BYTE Value */
#define MXC_S_SPIXF_FETCH_CTRL_ADDR4_3BYTE             (MXC_V_SPIXF_FETCH_CTRL_ADDR4_3BYTE << MXC_F_SPIXF_FETCH_CTRL_ADDR4_POS) /**< FETCH_CTRL_ADDR4_3BYTE Setting */
#define MXC_V_SPIXF_FETCH_CTRL_ADDR4_4BYTE             ((uint32_t)0x1UL) /**< FETCH_CTRL_ADDR4_4BYTE Value */
#define MXC_S_SPIXF_FETCH_CTRL_ADDR4_4BYTE             (MXC_V_SPIXF_FETCH_CTRL_ADDR4_4BYTE << MXC_F_SPIXF_FETCH_CTRL_ADDR4_POS) /**< FETCH_CTRL_ADDR4_4BYTE Setting */

/**@} end of group SPIXF_FETCH_CTRL_Register */

/**
 * @ingroup  spixf_registers
 * @defgroup SPIXF_MODE_CTRL SPIXF_MODE_CTRL
 * @brief    SPIX Mode Control Register.
 * @{
 */
#define MXC_F_SPIXF_MODE_CTRL_MDCLK_POS                0 /**< MODE_CTRL_MDCLK Position */
#define MXC_F_SPIXF_MODE_CTRL_MDCLK                    ((uint32_t)(0xFUL << MXC_F_SPIXF_MODE_CTRL_MDCLK_POS)) /**< MODE_CTRL_MDCLK Mask */

#define MXC_F_SPIXF_MODE_CTRL_NOCMD_POS                8 /**< MODE_CTRL_NOCMD Position */
#define MXC_F_SPIXF_MODE_CTRL_NOCMD                    ((uint32_t)(0x1UL << MXC_F_SPIXF_MODE_CTRL_NOCMD_POS)) /**< MODE_CTRL_NOCMD Mask */
#define MXC_V_SPIXF_MODE_CTRL_NOCMD_ALWAYS             ((uint32_t)0x0UL) /**< MODE_CTRL_NOCMD_ALWAYS Value */
#define MXC_S_SPIXF_MODE_CTRL_NOCMD_ALWAYS             (MXC_V_SPIXF_MODE_CTRL_NOCMD_ALWAYS << MXC_F_SPIXF_MODE_CTRL_NOCMD_POS) /**< MODE_CTRL_NOCMD_ALWAYS Setting */
#define MXC_V_SPIXF_MODE_CTRL_NOCMD_ONCE               ((uint32_t)0x1UL) /**< MODE_CTRL_NOCMD_ONCE Value */
#define MXC_S_SPIXF_MODE_CTRL_NOCMD_ONCE               (MXC_V_SPIXF_MODE_CTRL_NOCMD_ONCE << MXC_F_SPIXF_MODE_CTRL_NOCMD_POS) /**< MODE_CTRL_NOCMD_ONCE Setting */

#define MXC_F_SPIXF_MODE_CTRL_MODE_SEND_POS            9 /**< MODE_CTRL_MODE_SEND Position */
#define MXC_F_SPIXF_MODE_CTRL_MODE_SEND                ((uint32_t)(0x1UL << MXC_F_SPIXF_MODE_CTRL_MODE_SEND_POS)) /**< MODE_CTRL_MODE_SEND Mask */
#define MXC_V_SPIXF_MODE_CTRL_MODE_SEND_NEXT           ((uint32_t)0x1UL) /**< MODE_CTRL_MODE_SEND_NEXT Value */
#define MXC_S_SPIXF_MODE_CTRL_MODE_SEND_NEXT           (MXC_V_SPIXF_MODE_CTRL_MODE_SEND_NEXT << MXC_F_SPIXF_MODE_CTRL_MODE_SEND_POS) /**< MODE_CTRL_MODE_SEND_NEXT Setting */

/**@} end of group SPIXF_MODE_CTRL_Register */

/**
 * @ingroup  spixf_registers
 * @defgroup SPIXF_MODE_DATA SPIXF_MODE_DATA
 * @brief    SPIX Mode Data Register.
 * @{
 */
#define MXC_F_SPIXF_MODE_DATA_MDDATA_POS               0 /**< MODE_DATA_MDDATA Position */
#define MXC_F_SPIXF_MODE_DATA_MDDATA                   ((uint32_t)(0xFFFFUL << MXC_F_SPIXF_MODE_DATA_MDDATA_POS)) /**< MODE_DATA_MDDATA Mask */

#define MXC_F_SPIXF_MODE_DATA_MDOE_POS                 16 /**< MODE_DATA_MDOE Position */
#define MXC_F_SPIXF_MODE_DATA_MDOE                     ((uint32_t)(0xFFFFUL << MXC_F_SPIXF_MODE_DATA_MDOE_POS)) /**< MODE_DATA_MDOE Mask */

/**@} end of group SPIXF_MODE_DATA_Register */

/**
 * @ingroup  spixf_registers
 * @defgroup SPIXF_FB_CTRL SPIXF_FB_CTRL
 * @brief    SPIX Feedback Control Register.
 * @{
 */
#define MXC_F_SPIXF_FB_CTRL_FBMD_POS                   0 /**< FB_CTRL_FBMD Position */
#define MXC_F_SPIXF_FB_CTRL_FBMD                       ((uint32_t)(0x1UL << MXC_F_SPIXF_FB_CTRL_FBMD_POS)) /**< FB_CTRL_FBMD Mask */
#define MXC_V_SPIXF_FB_CTRL_FBMD_DIS                   ((uint32_t)0x0UL) /**< FB_CTRL_FBMD_DIS Value */
#define MXC_S_SPIXF_FB_CTRL_FBMD_DIS                   (MXC_V_SPIXF_FB_CTRL_FBMD_DIS << MXC_F_SPIXF_FB_CTRL_FBMD_POS) /**< FB_CTRL_FBMD_DIS Setting */
#define MXC_V_SPIXF_FB_CTRL_FBMD_EN                    ((uint32_t)0x1UL) /**< FB_CTRL_FBMD_EN Value */
#define MXC_S_SPIXF_FB_CTRL_FBMD_EN                    (MXC_V_SPIXF_FB_CTRL_FBMD_EN << MXC_F_SPIXF_FB_CTRL_FBMD_POS) /**< FB_CTRL_FBMD_EN Setting */

#define MXC_F_SPIXF_FB_CTRL_FBINV_POS                  1 /**< FB_CTRL_FBINV Position */
#define MXC_F_SPIXF_FB_CTRL_FBINV                      ((uint32_t)(0x1UL << MXC_F_SPIXF_FB_CTRL_FBINV_POS)) /**< FB_CTRL_FBINV Mask */
#define MXC_V_SPIXF_FB_CTRL_FBINV_DIS                  ((uint32_t)0x0UL) /**< FB_CTRL_FBINV_DIS Value */
#define MXC_S_SPIXF_FB_CTRL_FBINV_DIS                  (MXC_V_SPIXF_FB_CTRL_FBINV_DIS << MXC_F_SPIXF_FB_CTRL_FBINV_POS) /**< FB_CTRL_FBINV_DIS Setting */
#define MXC_V_SPIXF_FB_CTRL_FBINV_EN                   ((uint32_t)0x1UL) /**< FB_CTRL_FBINV_EN Value */
#define MXC_S_SPIXF_FB_CTRL_FBINV_EN                   (MXC_V_SPIXF_FB_CTRL_FBINV_EN << MXC_F_SPIXF_FB_CTRL_FBINV_POS) /**< FB_CTRL_FBINV_EN Setting */

/**@} end of group SPIXF_FB_CTRL_Register */

/**
 * @ingroup  spixf_registers
 * @defgroup SPIXF_IO_CTRL SPIXF_IO_CTRL
 * @brief    SPIX IO Control Register.
 * @{
 */
#define MXC_F_SPIXF_IO_CTRL_SCK_DS_POS                 0 /**< IO_CTRL_SCK_DS Position */
#define MXC_F_SPIXF_IO_CTRL_SCK_DS                     ((uint32_t)(0x1UL << MXC_F_SPIXF_IO_CTRL_SCK_DS_POS)) /**< IO_CTRL_SCK_DS Mask */
#define MXC_V_SPIXF_IO_CTRL_SCK_DS_LOW                 ((uint32_t)0x0UL) /**< IO_CTRL_SCK_DS_LOW Value */
#define MXC_S_SPIXF_IO_CTRL_SCK_DS_LOW                 (MXC_V_SPIXF_IO_CTRL_SCK_DS_LOW << MXC_F_SPIXF_IO_CTRL_SCK_DS_POS) /**< IO_CTRL_SCK_DS_LOW Setting */
#define MXC_V_SPIXF_IO_CTRL_SCK_DS_HIGH                ((uint32_t)0x1UL) /**< IO_CTRL_SCK_DS_HIGH Value */
#define MXC_S_SPIXF_IO_CTRL_SCK_DS_HIGH                (MXC_V_SPIXF_IO_CTRL_SCK_DS_HIGH << MXC_F_SPIXF_IO_CTRL_SCK_DS_POS) /**< IO_CTRL_SCK_DS_HIGH Setting */

#define MXC_F_SPIXF_IO_CTRL_SS_DS_POS                  1 /**< IO_CTRL_SS_DS Position */
#define MXC_F_SPIXF_IO_CTRL_SS_DS                      ((uint32_t)(0x1UL << MXC_F_SPIXF_IO_CTRL_SS_DS_POS)) /**< IO_CTRL_SS_DS Mask */
#define MXC_V_SPIXF_IO_CTRL_SS_DS_LOW                  ((uint32_t)0x0UL) /**< IO_CTRL_SS_DS_LOW Value */
#define MXC_S_SPIXF_IO_CTRL_SS_DS_LOW                  (MXC_V_SPIXF_IO_CTRL_SS_DS_LOW << MXC_F_SPIXF_IO_CTRL_SS_DS_POS) /**< IO_CTRL_SS_DS_LOW Setting */
#define MXC_V_SPIXF_IO_CTRL_SS_DS_HIGH                 ((uint32_t)0x1UL) /**< IO_CTRL_SS_DS_HIGH Value */
#define MXC_S_SPIXF_IO_CTRL_SS_DS_HIGH                 (MXC_V_SPIXF_IO_CTRL_SS_DS_HIGH << MXC_F_SPIXF_IO_CTRL_SS_DS_POS) /**< IO_CTRL_SS_DS_HIGH Setting */

#define MXC_F_SPIXF_IO_CTRL_SDIO_DS_POS                2 /**< IO_CTRL_SDIO_DS Position */
#define MXC_F_SPIXF_IO_CTRL_SDIO_DS                    ((uint32_t)(0x1UL << MXC_F_SPIXF_IO_CTRL_SDIO_DS_POS)) /**< IO_CTRL_SDIO_DS Mask */
#define MXC_V_SPIXF_IO_CTRL_SDIO_DS_LOW                ((uint32_t)0x0UL) /**< IO_CTRL_SDIO_DS_LOW Value */
#define MXC_S_SPIXF_IO_CTRL_SDIO_DS_LOW                (MXC_V_SPIXF_IO_CTRL_SDIO_DS_LOW << MXC_F_SPIXF_IO_CTRL_SDIO_DS_POS) /**< IO_CTRL_SDIO_DS_LOW Setting */
#define MXC_V_SPIXF_IO_CTRL_SDIO_DS_HIGH               ((uint32_t)0x1UL) /**< IO_CTRL_SDIO_DS_HIGH Value */
#define MXC_S_SPIXF_IO_CTRL_SDIO_DS_HIGH               (MXC_V_SPIXF_IO_CTRL_SDIO_DS_HIGH << MXC_F_SPIXF_IO_CTRL_SDIO_DS_POS) /**< IO_CTRL_SDIO_DS_HIGH Setting */

#define MXC_F_SPIXF_IO_CTRL_PUPDCTRL_POS               3 /**< IO_CTRL_PUPDCTRL Position */
#define MXC_F_SPIXF_IO_CTRL_PUPDCTRL                   ((uint32_t)(0x3UL << MXC_F_SPIXF_IO_CTRL_PUPDCTRL_POS)) /**< IO_CTRL_PUPDCTRL Mask */
#define MXC_V_SPIXF_IO_CTRL_PUPDCTRL_TRI_STATE         ((uint32_t)0x0UL) /**< IO_CTRL_PUPDCTRL_TRI_STATE Value */
#define MXC_S_SPIXF_IO_CTRL_PUPDCTRL_TRI_STATE         (MXC_V_SPIXF_IO_CTRL_PUPDCTRL_TRI_STATE << MXC_F_SPIXF_IO_CTRL_PUPDCTRL_POS) /**< IO_CTRL_PUPDCTRL_TRI_STATE Setting */
#define MXC_V_SPIXF_IO_CTRL_PUPDCTRL_PULL_UP           ((uint32_t)0x1UL) /**< IO_CTRL_PUPDCTRL_PULL_UP Value */
#define MXC_S_SPIXF_IO_CTRL_PUPDCTRL_PULL_UP           (MXC_V_SPIXF_IO_CTRL_PUPDCTRL_PULL_UP << MXC_F_SPIXF_IO_CTRL_PUPDCTRL_POS) /**< IO_CTRL_PUPDCTRL_PULL_UP Setting */
#define MXC_V_SPIXF_IO_CTRL_PUPDCTRL_PULL_DOWN         ((uint32_t)0x2UL) /**< IO_CTRL_PUPDCTRL_PULL_DOWN Value */
#define MXC_S_SPIXF_IO_CTRL_PUPDCTRL_PULL_DOWN         (MXC_V_SPIXF_IO_CTRL_PUPDCTRL_PULL_DOWN << MXC_F_SPIXF_IO_CTRL_PUPDCTRL_POS) /**< IO_CTRL_PUPDCTRL_PULL_DOWN Setting */

/**@} end of group SPIXF_IO_CTRL_Register */

/**
 * @ingroup  spixf_registers
 * @defgroup SPIXF_SEC_CTRL SPIXF_SEC_CTRL
 * @brief    SPIX Memory Security Control Register.
 * @{
 */
#define MXC_F_SPIXF_SEC_CTRL_DEC_EN_POS                0 /**< SEC_CTRL_DEC_EN Position */
#define MXC_F_SPIXF_SEC_CTRL_DEC_EN                    ((uint32_t)(0x1UL << MXC_F_SPIXF_SEC_CTRL_DEC_EN_POS)) /**< SEC_CTRL_DEC_EN Mask */
#define MXC_V_SPIXF_SEC_CTRL_DEC_EN_DIS                ((uint32_t)0x0UL) /**< SEC_CTRL_DEC_EN_DIS Value */
#define MXC_S_SPIXF_SEC_CTRL_DEC_EN_DIS                (MXC_V_SPIXF_SEC_CTRL_DEC_EN_DIS << MXC_F_SPIXF_SEC_CTRL_DEC_EN_POS) /**< SEC_CTRL_DEC_EN_DIS Setting */
#define MXC_V_SPIXF_SEC_CTRL_DEC_EN_EN                 ((uint32_t)0x1UL) /**< SEC_CTRL_DEC_EN_EN Value */
#define MXC_S_SPIXF_SEC_CTRL_DEC_EN_EN                 (MXC_V_SPIXF_SEC_CTRL_DEC_EN_EN << MXC_F_SPIXF_SEC_CTRL_DEC_EN_POS) /**< SEC_CTRL_DEC_EN_EN Setting */

#define MXC_F_SPIXF_SEC_CTRL_AUTH_DISABLE_POS          1 /**< SEC_CTRL_AUTH_DISABLE Position */
#define MXC_F_SPIXF_SEC_CTRL_AUTH_DISABLE              ((uint32_t)(0x1UL << MXC_F_SPIXF_SEC_CTRL_AUTH_DISABLE_POS)) /**< SEC_CTRL_AUTH_DISABLE Mask */
#define MXC_V_SPIXF_SEC_CTRL_AUTH_DISABLE_EN           ((uint32_t)0x0UL) /**< SEC_CTRL_AUTH_DISABLE_EN Value */
#define MXC_S_SPIXF_SEC_CTRL_AUTH_DISABLE_EN           (MXC_V_SPIXF_SEC_CTRL_AUTH_DISABLE_EN << MXC_F_SPIXF_SEC_CTRL_AUTH_DISABLE_POS) /**< SEC_CTRL_AUTH_DISABLE_EN Setting */
#define MXC_V_SPIXF_SEC_CTRL_AUTH_DISABLE_DIS          ((uint32_t)0x1UL) /**< SEC_CTRL_AUTH_DISABLE_DIS Value */
#define MXC_S_SPIXF_SEC_CTRL_AUTH_DISABLE_DIS          (MXC_V_SPIXF_SEC_CTRL_AUTH_DISABLE_DIS << MXC_F_SPIXF_SEC_CTRL_AUTH_DISABLE_POS) /**< SEC_CTRL_AUTH_DISABLE_DIS Setting */

/**@} end of group SPIXF_SEC_CTRL_Register */

/**
 * @ingroup  spixf_registers
 * @defgroup SPIXF_BUS_IDLE SPIXF_BUS_IDLE
 * @brief    Bus Idle
 * @{
 */
#define MXC_F_SPIXF_BUS_IDLE_BUSIDLE_POS               0 /**< BUS_IDLE_BUSIDLE Position */
#define MXC_F_SPIXF_BUS_IDLE_BUSIDLE                   ((uint32_t)(0xFFFFUL << MXC_F_SPIXF_BUS_IDLE_BUSIDLE_POS)) /**< BUS_IDLE_BUSIDLE Mask */

/**@} end of group SPIXF_BUS_IDLE_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32650_INCLUDE_SPIXF_REGS_H_
