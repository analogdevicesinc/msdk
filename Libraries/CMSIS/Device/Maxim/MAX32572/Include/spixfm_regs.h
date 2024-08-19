/**
 * @file    spixfm_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the SPIXFM Peripheral Module.
 * @note    This file is @generated.
 * @ingroup spixfm_registers
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32572_INCLUDE_SPIXFM_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32572_INCLUDE_SPIXFM_REGS_H_

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
 * @ingroup     spixfm
 * @defgroup    spixfm_registers SPIXFM_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the SPIXFM Peripheral Module.
 * @details     SPIXF Master
 */

/**
 * @ingroup spixfm_registers
 * Structure type to access the SPIXFM Registers.
 */
typedef struct {
    __IO uint32_t ctrl;                 /**< <tt>\b 0x00:</tt> SPIXFM CTRL Register */
    __IO uint32_t fetchctrl;            /**< <tt>\b 0x04:</tt> SPIXFM FETCHCTRL Register */
    __IO uint32_t modectrl;             /**< <tt>\b 0x08:</tt> SPIXFM MODECTRL Register */
    __IO uint32_t modedata;             /**< <tt>\b 0x0C:</tt> SPIXFM MODEDATA Register */
    __IO uint32_t fbctrl;               /**< <tt>\b 0x10:</tt> SPIXFM FBCTRL Register */
    __R  uint32_t rsv_0x14_0x1b[2];
    __IO uint32_t ioctrl;               /**< <tt>\b 0x1C:</tt> SPIXFM IOCTRL Register */
    __IO uint32_t memsecctrl;           /**< <tt>\b 0x20:</tt> SPIXFM MEMSECCTRL Register */
    __IO uint32_t busidle;              /**< <tt>\b 0x24:</tt> SPIXFM BUSIDLE Register */
    __IO uint32_t authoffset;           /**< <tt>\b 0x28:</tt> SPIXFM AUTHOFFSET Register */
    __IO uint32_t bypass_mode;          /**< <tt>\b 0x2C:</tt> SPIXFM BYPASS_MODE Register */
} mxc_spixfm_regs_t;

/* Register offsets for module SPIXFM */
/**
 * @ingroup    spixfm_registers
 * @defgroup   SPIXFM_Register_Offsets Register Offsets
 * @brief      SPIXFM Peripheral Register Offsets from the SPIXFM Base Peripheral Address.
 * @{
 */
#define MXC_R_SPIXFM_CTRL                  ((uint32_t)0x00000000UL) /**< Offset from SPIXFM Base Address: <tt> 0x0000</tt> */
#define MXC_R_SPIXFM_FETCHCTRL             ((uint32_t)0x00000004UL) /**< Offset from SPIXFM Base Address: <tt> 0x0004</tt> */
#define MXC_R_SPIXFM_MODECTRL              ((uint32_t)0x00000008UL) /**< Offset from SPIXFM Base Address: <tt> 0x0008</tt> */
#define MXC_R_SPIXFM_MODEDATA              ((uint32_t)0x0000000CUL) /**< Offset from SPIXFM Base Address: <tt> 0x000C</tt> */
#define MXC_R_SPIXFM_FBCTRL                ((uint32_t)0x00000010UL) /**< Offset from SPIXFM Base Address: <tt> 0x0010</tt> */
#define MXC_R_SPIXFM_IOCTRL                ((uint32_t)0x0000001CUL) /**< Offset from SPIXFM Base Address: <tt> 0x001C</tt> */
#define MXC_R_SPIXFM_MEMSECCTRL            ((uint32_t)0x00000020UL) /**< Offset from SPIXFM Base Address: <tt> 0x0020</tt> */
#define MXC_R_SPIXFM_BUSIDLE               ((uint32_t)0x00000024UL) /**< Offset from SPIXFM Base Address: <tt> 0x0024</tt> */
#define MXC_R_SPIXFM_AUTHOFFSET            ((uint32_t)0x00000028UL) /**< Offset from SPIXFM Base Address: <tt> 0x0028</tt> */
#define MXC_R_SPIXFM_BYPASS_MODE           ((uint32_t)0x0000002CUL) /**< Offset from SPIXFM Base Address: <tt> 0x002C</tt> */
/**@} end of group spixfm_registers */

/**
 * @ingroup  spixfm_registers
 * @defgroup SPIXFM_CTRL SPIXFM_CTRL
 * @brief    SPIX Control Register.
 * @{
 */
#define MXC_F_SPIXFM_CTRL_MODE_POS                     0 /**< CTRL_MODE Position */
#define MXC_F_SPIXFM_CTRL_MODE                         ((uint32_t)(0x3UL << MXC_F_SPIXFM_CTRL_MODE_POS)) /**< CTRL_MODE Mask */
#define MXC_V_SPIXFM_CTRL_MODE_SCLK_HI_SAMPLE_RISING   ((uint32_t)0x0UL) /**< CTRL_MODE_SCLK_HI_SAMPLE_RISING Value */
#define MXC_S_SPIXFM_CTRL_MODE_SCLK_HI_SAMPLE_RISING   (MXC_V_SPIXFM_CTRL_MODE_SCLK_HI_SAMPLE_RISING << MXC_F_SPIXFM_CTRL_MODE_POS) /**< CTRL_MODE_SCLK_HI_SAMPLE_RISING Setting */
#define MXC_V_SPIXFM_CTRL_MODE_SCLK_LO_SAMPLE_FAILLING ((uint32_t)0x3UL) /**< CTRL_MODE_SCLK_LO_SAMPLE_FAILLING Value */
#define MXC_S_SPIXFM_CTRL_MODE_SCLK_LO_SAMPLE_FAILLING (MXC_V_SPIXFM_CTRL_MODE_SCLK_LO_SAMPLE_FAILLING << MXC_F_SPIXFM_CTRL_MODE_POS) /**< CTRL_MODE_SCLK_LO_SAMPLE_FAILLING Setting */

#define MXC_F_SPIXFM_CTRL_SSPOL_POS                    2 /**< CTRL_SSPOL Position */
#define MXC_F_SPIXFM_CTRL_SSPOL                        ((uint32_t)(0x1UL << MXC_F_SPIXFM_CTRL_SSPOL_POS)) /**< CTRL_SSPOL Mask */

#define MXC_F_SPIXFM_CTRL_SSEL_POS                     4 /**< CTRL_SSEL Position */
#define MXC_F_SPIXFM_CTRL_SSEL                         ((uint32_t)(0x7UL << MXC_F_SPIXFM_CTRL_SSEL_POS)) /**< CTRL_SSEL Mask */

#define MXC_F_SPIXFM_CTRL_LOCLK_POS                    8 /**< CTRL_LOCLK Position */
#define MXC_F_SPIXFM_CTRL_LOCLK                        ((uint32_t)(0xFUL << MXC_F_SPIXFM_CTRL_LOCLK_POS)) /**< CTRL_LOCLK Mask */

#define MXC_F_SPIXFM_CTRL_HICLK_POS                    12 /**< CTRL_HICLK Position */
#define MXC_F_SPIXFM_CTRL_HICLK                        ((uint32_t)(0xFUL << MXC_F_SPIXFM_CTRL_HICLK_POS)) /**< CTRL_HICLK Mask */

#define MXC_F_SPIXFM_CTRL_SSACT_POS                    16 /**< CTRL_SSACT Position */
#define MXC_F_SPIXFM_CTRL_SSACT                        ((uint32_t)(0x3UL << MXC_F_SPIXFM_CTRL_SSACT_POS)) /**< CTRL_SSACT Mask */
#define MXC_V_SPIXFM_CTRL_SSACT_OFF                    ((uint32_t)0x0UL) /**< CTRL_SSACT_OFF Value */
#define MXC_S_SPIXFM_CTRL_SSACT_OFF                    (MXC_V_SPIXFM_CTRL_SSACT_OFF << MXC_F_SPIXFM_CTRL_SSACT_POS) /**< CTRL_SSACT_OFF Setting */
#define MXC_V_SPIXFM_CTRL_SSACT_FOR_2_MOD_CLK          ((uint32_t)0x1UL) /**< CTRL_SSACT_FOR_2_MOD_CLK Value */
#define MXC_S_SPIXFM_CTRL_SSACT_FOR_2_MOD_CLK          (MXC_V_SPIXFM_CTRL_SSACT_FOR_2_MOD_CLK << MXC_F_SPIXFM_CTRL_SSACT_POS) /**< CTRL_SSACT_FOR_2_MOD_CLK Setting */
#define MXC_V_SPIXFM_CTRL_SSACT_FOR_4_MOD_CLK          ((uint32_t)0x2UL) /**< CTRL_SSACT_FOR_4_MOD_CLK Value */
#define MXC_S_SPIXFM_CTRL_SSACT_FOR_4_MOD_CLK          (MXC_V_SPIXFM_CTRL_SSACT_FOR_4_MOD_CLK << MXC_F_SPIXFM_CTRL_SSACT_POS) /**< CTRL_SSACT_FOR_4_MOD_CLK Setting */
#define MXC_V_SPIXFM_CTRL_SSACT_FOR_8_MOD_CLK          ((uint32_t)0x3UL) /**< CTRL_SSACT_FOR_8_MOD_CLK Value */
#define MXC_S_SPIXFM_CTRL_SSACT_FOR_8_MOD_CLK          (MXC_V_SPIXFM_CTRL_SSACT_FOR_8_MOD_CLK << MXC_F_SPIXFM_CTRL_SSACT_POS) /**< CTRL_SSACT_FOR_8_MOD_CLK Setting */

#define MXC_F_SPIXFM_CTRL_SSINACT_POS                  18 /**< CTRL_SSINACT Position */
#define MXC_F_SPIXFM_CTRL_SSINACT                      ((uint32_t)(0x3UL << MXC_F_SPIXFM_CTRL_SSINACT_POS)) /**< CTRL_SSINACT Mask */
#define MXC_V_SPIXFM_CTRL_SSINACT_FOR_1_MOD_CLK        ((uint32_t)0x0UL) /**< CTRL_SSINACT_FOR_1_MOD_CLK Value */
#define MXC_S_SPIXFM_CTRL_SSINACT_FOR_1_MOD_CLK        (MXC_V_SPIXFM_CTRL_SSINACT_FOR_1_MOD_CLK << MXC_F_SPIXFM_CTRL_SSINACT_POS) /**< CTRL_SSINACT_FOR_1_MOD_CLK Setting */
#define MXC_V_SPIXFM_CTRL_SSINACT_FOR_3_MOD_CLK        ((uint32_t)0x1UL) /**< CTRL_SSINACT_FOR_3_MOD_CLK Value */
#define MXC_S_SPIXFM_CTRL_SSINACT_FOR_3_MOD_CLK        (MXC_V_SPIXFM_CTRL_SSINACT_FOR_3_MOD_CLK << MXC_F_SPIXFM_CTRL_SSINACT_POS) /**< CTRL_SSINACT_FOR_3_MOD_CLK Setting */
#define MXC_V_SPIXFM_CTRL_SSINACT_FOR_5_MOD_CLK        ((uint32_t)0x2UL) /**< CTRL_SSINACT_FOR_5_MOD_CLK Value */
#define MXC_S_SPIXFM_CTRL_SSINACT_FOR_5_MOD_CLK        (MXC_V_SPIXFM_CTRL_SSINACT_FOR_5_MOD_CLK << MXC_F_SPIXFM_CTRL_SSINACT_POS) /**< CTRL_SSINACT_FOR_5_MOD_CLK Setting */
#define MXC_V_SPIXFM_CTRL_SSINACT_FOR_9_MOD_CLK        ((uint32_t)0x3UL) /**< CTRL_SSINACT_FOR_9_MOD_CLK Value */
#define MXC_S_SPIXFM_CTRL_SSINACT_FOR_9_MOD_CLK        (MXC_V_SPIXFM_CTRL_SSINACT_FOR_9_MOD_CLK << MXC_F_SPIXFM_CTRL_SSINACT_POS) /**< CTRL_SSINACT_FOR_9_MOD_CLK Setting */

/**@} end of group SPIXFM_CTRL_Register */

/**
 * @ingroup  spixfm_registers
 * @defgroup SPIXFM_FETCHCTRL SPIXFM_FETCHCTRL
 * @brief    SPIX Fetch Control Register.
 * @{
 */
#define MXC_F_SPIXFM_FETCHCTRL_CMD_VAL_POS             0 /**< FETCHCTRL_CMD_VAL Position */
#define MXC_F_SPIXFM_FETCHCTRL_CMD_VAL                 ((uint32_t)(0xFFUL << MXC_F_SPIXFM_FETCHCTRL_CMD_VAL_POS)) /**< FETCHCTRL_CMD_VAL Mask */

#define MXC_F_SPIXFM_FETCHCTRL_CMD_WDTH_POS            8 /**< FETCHCTRL_CMD_WDTH Position */
#define MXC_F_SPIXFM_FETCHCTRL_CMD_WDTH                ((uint32_t)(0x3UL << MXC_F_SPIXFM_FETCHCTRL_CMD_WDTH_POS)) /**< FETCHCTRL_CMD_WDTH Mask */
#define MXC_V_SPIXFM_FETCHCTRL_CMD_WDTH_SINGLE         ((uint32_t)0x0UL) /**< FETCHCTRL_CMD_WDTH_SINGLE Value */
#define MXC_S_SPIXFM_FETCHCTRL_CMD_WDTH_SINGLE         (MXC_V_SPIXFM_FETCHCTRL_CMD_WDTH_SINGLE << MXC_F_SPIXFM_FETCHCTRL_CMD_WDTH_POS) /**< FETCHCTRL_CMD_WDTH_SINGLE Setting */
#define MXC_V_SPIXFM_FETCHCTRL_CMD_WDTH_DUAL_IO        ((uint32_t)0x1UL) /**< FETCHCTRL_CMD_WDTH_DUAL_IO Value */
#define MXC_S_SPIXFM_FETCHCTRL_CMD_WDTH_DUAL_IO        (MXC_V_SPIXFM_FETCHCTRL_CMD_WDTH_DUAL_IO << MXC_F_SPIXFM_FETCHCTRL_CMD_WDTH_POS) /**< FETCHCTRL_CMD_WDTH_DUAL_IO Setting */
#define MXC_V_SPIXFM_FETCHCTRL_CMD_WDTH_QUAD_IO        ((uint32_t)0x2UL) /**< FETCHCTRL_CMD_WDTH_QUAD_IO Value */
#define MXC_S_SPIXFM_FETCHCTRL_CMD_WDTH_QUAD_IO        (MXC_V_SPIXFM_FETCHCTRL_CMD_WDTH_QUAD_IO << MXC_F_SPIXFM_FETCHCTRL_CMD_WDTH_POS) /**< FETCHCTRL_CMD_WDTH_QUAD_IO Setting */
#define MXC_V_SPIXFM_FETCHCTRL_CMD_WDTH_INVALID        ((uint32_t)0x3UL) /**< FETCHCTRL_CMD_WDTH_INVALID Value */
#define MXC_S_SPIXFM_FETCHCTRL_CMD_WDTH_INVALID        (MXC_V_SPIXFM_FETCHCTRL_CMD_WDTH_INVALID << MXC_F_SPIXFM_FETCHCTRL_CMD_WDTH_POS) /**< FETCHCTRL_CMD_WDTH_INVALID Setting */

#define MXC_F_SPIXFM_FETCHCTRL_ADDR_WDTH_POS           10 /**< FETCHCTRL_ADDR_WDTH Position */
#define MXC_F_SPIXFM_FETCHCTRL_ADDR_WDTH               ((uint32_t)(0x3UL << MXC_F_SPIXFM_FETCHCTRL_ADDR_WDTH_POS)) /**< FETCHCTRL_ADDR_WDTH Mask */
#define MXC_V_SPIXFM_FETCHCTRL_ADDR_WDTH_SINGLE        ((uint32_t)0x0UL) /**< FETCHCTRL_ADDR_WDTH_SINGLE Value */
#define MXC_S_SPIXFM_FETCHCTRL_ADDR_WDTH_SINGLE        (MXC_V_SPIXFM_FETCHCTRL_ADDR_WDTH_SINGLE << MXC_F_SPIXFM_FETCHCTRL_ADDR_WDTH_POS) /**< FETCHCTRL_ADDR_WDTH_SINGLE Setting */
#define MXC_V_SPIXFM_FETCHCTRL_ADDR_WDTH_DUAL_IO       ((uint32_t)0x1UL) /**< FETCHCTRL_ADDR_WDTH_DUAL_IO Value */
#define MXC_S_SPIXFM_FETCHCTRL_ADDR_WDTH_DUAL_IO       (MXC_V_SPIXFM_FETCHCTRL_ADDR_WDTH_DUAL_IO << MXC_F_SPIXFM_FETCHCTRL_ADDR_WDTH_POS) /**< FETCHCTRL_ADDR_WDTH_DUAL_IO Setting */
#define MXC_V_SPIXFM_FETCHCTRL_ADDR_WDTH_QUAD_IO       ((uint32_t)0x2UL) /**< FETCHCTRL_ADDR_WDTH_QUAD_IO Value */
#define MXC_S_SPIXFM_FETCHCTRL_ADDR_WDTH_QUAD_IO       (MXC_V_SPIXFM_FETCHCTRL_ADDR_WDTH_QUAD_IO << MXC_F_SPIXFM_FETCHCTRL_ADDR_WDTH_POS) /**< FETCHCTRL_ADDR_WDTH_QUAD_IO Setting */
#define MXC_V_SPIXFM_FETCHCTRL_ADDR_WDTH_INVALID       ((uint32_t)0x3UL) /**< FETCHCTRL_ADDR_WDTH_INVALID Value */
#define MXC_S_SPIXFM_FETCHCTRL_ADDR_WDTH_INVALID       (MXC_V_SPIXFM_FETCHCTRL_ADDR_WDTH_INVALID << MXC_F_SPIXFM_FETCHCTRL_ADDR_WDTH_POS) /**< FETCHCTRL_ADDR_WDTH_INVALID Setting */

#define MXC_F_SPIXFM_FETCHCTRL_DATA_WDTH_POS           12 /**< FETCHCTRL_DATA_WDTH Position */
#define MXC_F_SPIXFM_FETCHCTRL_DATA_WDTH               ((uint32_t)(0x3UL << MXC_F_SPIXFM_FETCHCTRL_DATA_WDTH_POS)) /**< FETCHCTRL_DATA_WDTH Mask */
#define MXC_V_SPIXFM_FETCHCTRL_DATA_WDTH_SINGLE        ((uint32_t)0x0UL) /**< FETCHCTRL_DATA_WDTH_SINGLE Value */
#define MXC_S_SPIXFM_FETCHCTRL_DATA_WDTH_SINGLE        (MXC_V_SPIXFM_FETCHCTRL_DATA_WDTH_SINGLE << MXC_F_SPIXFM_FETCHCTRL_DATA_WDTH_POS) /**< FETCHCTRL_DATA_WDTH_SINGLE Setting */
#define MXC_V_SPIXFM_FETCHCTRL_DATA_WDTH_DUAL_IO       ((uint32_t)0x1UL) /**< FETCHCTRL_DATA_WDTH_DUAL_IO Value */
#define MXC_S_SPIXFM_FETCHCTRL_DATA_WDTH_DUAL_IO       (MXC_V_SPIXFM_FETCHCTRL_DATA_WDTH_DUAL_IO << MXC_F_SPIXFM_FETCHCTRL_DATA_WDTH_POS) /**< FETCHCTRL_DATA_WDTH_DUAL_IO Setting */
#define MXC_V_SPIXFM_FETCHCTRL_DATA_WDTH_QUAD_IO       ((uint32_t)0x2UL) /**< FETCHCTRL_DATA_WDTH_QUAD_IO Value */
#define MXC_S_SPIXFM_FETCHCTRL_DATA_WDTH_QUAD_IO       (MXC_V_SPIXFM_FETCHCTRL_DATA_WDTH_QUAD_IO << MXC_F_SPIXFM_FETCHCTRL_DATA_WDTH_POS) /**< FETCHCTRL_DATA_WDTH_QUAD_IO Setting */
#define MXC_V_SPIXFM_FETCHCTRL_DATA_WDTH_INVALID       ((uint32_t)0x3UL) /**< FETCHCTRL_DATA_WDTH_INVALID Value */
#define MXC_S_SPIXFM_FETCHCTRL_DATA_WDTH_INVALID       (MXC_V_SPIXFM_FETCHCTRL_DATA_WDTH_INVALID << MXC_F_SPIXFM_FETCHCTRL_DATA_WDTH_POS) /**< FETCHCTRL_DATA_WDTH_INVALID Setting */

#define MXC_F_SPIXFM_FETCHCTRL_4BADDR_POS              16 /**< FETCHCTRL_4BADDR Position */
#define MXC_F_SPIXFM_FETCHCTRL_4BADDR                  ((uint32_t)(0x1UL << MXC_F_SPIXFM_FETCHCTRL_4BADDR_POS)) /**< FETCHCTRL_4BADDR Mask */

/**@} end of group SPIXFM_FETCHCTRL_Register */

/**
 * @ingroup  spixfm_registers
 * @defgroup SPIXFM_MODECTRL SPIXFM_MODECTRL
 * @brief    SPIX Mode Control Register.
 * @{
 */
#define MXC_F_SPIXFM_MODECTRL_MDCLK_POS                0 /**< MODECTRL_MDCLK Position */
#define MXC_F_SPIXFM_MODECTRL_MDCLK                    ((uint32_t)(0xFUL << MXC_F_SPIXFM_MODECTRL_MDCLK_POS)) /**< MODECTRL_MDCLK Mask */

#define MXC_F_SPIXFM_MODECTRL_NOCMD_POS                8 /**< MODECTRL_NOCMD Position */
#define MXC_F_SPIXFM_MODECTRL_NOCMD                    ((uint32_t)(0x1UL << MXC_F_SPIXFM_MODECTRL_NOCMD_POS)) /**< MODECTRL_NOCMD Mask */

#define MXC_F_SPIXFM_MODECTRL_EXIT_NOCMD_POS           9 /**< MODECTRL_EXIT_NOCMD Position */
#define MXC_F_SPIXFM_MODECTRL_EXIT_NOCMD               ((uint32_t)(0x1UL << MXC_F_SPIXFM_MODECTRL_EXIT_NOCMD_POS)) /**< MODECTRL_EXIT_NOCMD Mask */

/**@} end of group SPIXFM_MODECTRL_Register */

/**
 * @ingroup  spixfm_registers
 * @defgroup SPIXFM_MODEDATA SPIXFM_MODEDATA
 * @brief    SPIX Mode Data Register.
 * @{
 */
#define MXC_F_SPIXFM_MODEDATA_DATA_POS                 0 /**< MODEDATA_DATA Position */
#define MXC_F_SPIXFM_MODEDATA_DATA                     ((uint32_t)(0xFFFFUL << MXC_F_SPIXFM_MODEDATA_DATA_POS)) /**< MODEDATA_DATA Mask */

#define MXC_F_SPIXFM_MODEDATA_OUT_EN_POS               16 /**< MODEDATA_OUT_EN Position */
#define MXC_F_SPIXFM_MODEDATA_OUT_EN                   ((uint32_t)(0xFFFFUL << MXC_F_SPIXFM_MODEDATA_OUT_EN_POS)) /**< MODEDATA_OUT_EN Mask */

/**@} end of group SPIXFM_MODEDATA_Register */

/**
 * @ingroup  spixfm_registers
 * @defgroup SPIXFM_FBCTRL SPIXFM_FBCTRL
 * @brief    SPIX Feedback Control Register.
 * @{
 */
#define MXC_F_SPIXFM_FBCTRL_EN_POS                     0 /**< FBCTRL_EN Position */
#define MXC_F_SPIXFM_FBCTRL_EN                         ((uint32_t)(0x1UL << MXC_F_SPIXFM_FBCTRL_EN_POS)) /**< FBCTRL_EN Mask */

#define MXC_F_SPIXFM_FBCTRL_INVERT_POS                 1 /**< FBCTRL_INVERT Position */
#define MXC_F_SPIXFM_FBCTRL_INVERT                     ((uint32_t)(0x1UL << MXC_F_SPIXFM_FBCTRL_INVERT_POS)) /**< FBCTRL_INVERT Mask */

/**@} end of group SPIXFM_FBCTRL_Register */

/**
 * @ingroup  spixfm_registers
 * @defgroup SPIXFM_IOCTRL SPIXFM_IOCTRL
 * @brief    SPIX IO Control Register.
 * @{
 */
#define MXC_F_SPIXFM_IOCTRL_SCLK_DS_POS                0 /**< IOCTRL_SCLK_DS Position */
#define MXC_F_SPIXFM_IOCTRL_SCLK_DS                    ((uint32_t)(0x1UL << MXC_F_SPIXFM_IOCTRL_SCLK_DS_POS)) /**< IOCTRL_SCLK_DS Mask */

#define MXC_F_SPIXFM_IOCTRL_SS_DS_POS                  1 /**< IOCTRL_SS_DS Position */
#define MXC_F_SPIXFM_IOCTRL_SS_DS                      ((uint32_t)(0x1UL << MXC_F_SPIXFM_IOCTRL_SS_DS_POS)) /**< IOCTRL_SS_DS Mask */

#define MXC_F_SPIXFM_IOCTRL_SDIO_DS_POS                2 /**< IOCTRL_SDIO_DS Position */
#define MXC_F_SPIXFM_IOCTRL_SDIO_DS                    ((uint32_t)(0x1UL << MXC_F_SPIXFM_IOCTRL_SDIO_DS_POS)) /**< IOCTRL_SDIO_DS Mask */

#define MXC_F_SPIXFM_IOCTRL_PADCTRL_POS                3 /**< IOCTRL_PADCTRL Position */
#define MXC_F_SPIXFM_IOCTRL_PADCTRL                    ((uint32_t)(0x3UL << MXC_F_SPIXFM_IOCTRL_PADCTRL_POS)) /**< IOCTRL_PADCTRL Mask */
#define MXC_V_SPIXFM_IOCTRL_PADCTRL_TRI_STATE          ((uint32_t)0x0UL) /**< IOCTRL_PADCTRL_TRI_STATE Value */
#define MXC_S_SPIXFM_IOCTRL_PADCTRL_TRI_STATE          (MXC_V_SPIXFM_IOCTRL_PADCTRL_TRI_STATE << MXC_F_SPIXFM_IOCTRL_PADCTRL_POS) /**< IOCTRL_PADCTRL_TRI_STATE Setting */
#define MXC_V_SPIXFM_IOCTRL_PADCTRL_PULL_UP            ((uint32_t)0x1UL) /**< IOCTRL_PADCTRL_PULL_UP Value */
#define MXC_S_SPIXFM_IOCTRL_PADCTRL_PULL_UP            (MXC_V_SPIXFM_IOCTRL_PADCTRL_PULL_UP << MXC_F_SPIXFM_IOCTRL_PADCTRL_POS) /**< IOCTRL_PADCTRL_PULL_UP Setting */
#define MXC_V_SPIXFM_IOCTRL_PADCTRL_PULL_DOWN          ((uint32_t)0x2UL) /**< IOCTRL_PADCTRL_PULL_DOWN Value */
#define MXC_S_SPIXFM_IOCTRL_PADCTRL_PULL_DOWN          (MXC_V_SPIXFM_IOCTRL_PADCTRL_PULL_DOWN << MXC_F_SPIXFM_IOCTRL_PADCTRL_POS) /**< IOCTRL_PADCTRL_PULL_DOWN Setting */

/**@} end of group SPIXFM_IOCTRL_Register */

/**
 * @ingroup  spixfm_registers
 * @defgroup SPIXFM_MEMSECCTRL SPIXFM_MEMSECCTRL
 * @brief    SPIX Memory Security Control Register.
 * @{
 */
#define MXC_F_SPIXFM_MEMSECCTRL_DEC_EN_POS             0 /**< MEMSECCTRL_DEC_EN Position */
#define MXC_F_SPIXFM_MEMSECCTRL_DEC_EN                 ((uint32_t)(0x1UL << MXC_F_SPIXFM_MEMSECCTRL_DEC_EN_POS)) /**< MEMSECCTRL_DEC_EN Mask */

#define MXC_F_SPIXFM_MEMSECCTRL_AUTH_DIS_POS           1 /**< MEMSECCTRL_AUTH_DIS Position */
#define MXC_F_SPIXFM_MEMSECCTRL_AUTH_DIS               ((uint32_t)(0x1UL << MXC_F_SPIXFM_MEMSECCTRL_AUTH_DIS_POS)) /**< MEMSECCTRL_AUTH_DIS Mask */

#define MXC_F_SPIXFM_MEMSECCTRL_CNTOPT_EN_POS          2 /**< MEMSECCTRL_CNTOPT_EN Position */
#define MXC_F_SPIXFM_MEMSECCTRL_CNTOPT_EN              ((uint32_t)(0x1UL << MXC_F_SPIXFM_MEMSECCTRL_CNTOPT_EN_POS)) /**< MEMSECCTRL_CNTOPT_EN Mask */

#define MXC_F_SPIXFM_MEMSECCTRL_INTERL_DIS_POS         3 /**< MEMSECCTRL_INTERL_DIS Position */
#define MXC_F_SPIXFM_MEMSECCTRL_INTERL_DIS             ((uint32_t)(0x1UL << MXC_F_SPIXFM_MEMSECCTRL_INTERL_DIS_POS)) /**< MEMSECCTRL_INTERL_DIS Mask */

#define MXC_F_SPIXFM_MEMSECCTRL_AUTHERR_FL_POS         4 /**< MEMSECCTRL_AUTHERR_FL Position */
#define MXC_F_SPIXFM_MEMSECCTRL_AUTHERR_FL             ((uint32_t)(0x1UL << MXC_F_SPIXFM_MEMSECCTRL_AUTHERR_FL_POS)) /**< MEMSECCTRL_AUTHERR_FL Mask */

/**@} end of group SPIXFM_MEMSECCTRL_Register */

/**
 * @ingroup  spixfm_registers
 * @defgroup SPIXFM_BUSIDLE SPIXFM_BUSIDLE
 * @brief    Bus Idle
 * @{
 */
#define MXC_F_SPIXFM_BUSIDLE_BUSIDLE_POS               0 /**< BUSIDLE_BUSIDLE Position */
#define MXC_F_SPIXFM_BUSIDLE_BUSIDLE                   ((uint32_t)(0xFFFFUL << MXC_F_SPIXFM_BUSIDLE_BUSIDLE_POS)) /**< BUSIDLE_BUSIDLE Mask */

/**@} end of group SPIXFM_BUSIDLE_Register */

/**
 * @ingroup  spixfm_registers
 * @defgroup SPIXFM_BYPASS_MODE SPIXFM_BYPASS_MODE
 * @brief    Bypass Mode Register.
 * @{
 */
#define MXC_F_SPIXFM_BYPASS_MODE_EN_POS                0 /**< BYPASS_MODE_EN Position */
#define MXC_F_SPIXFM_BYPASS_MODE_EN                    ((uint32_t)(0x1UL << MXC_F_SPIXFM_BYPASS_MODE_EN_POS)) /**< BYPASS_MODE_EN Mask */

#define MXC_F_SPIXFM_BYPASS_MODE_FCLK_DELAY_POS        1 /**< BYPASS_MODE_FCLK_DELAY Position */
#define MXC_F_SPIXFM_BYPASS_MODE_FCLK_DELAY            ((uint32_t)(0x7UL << MXC_F_SPIXFM_BYPASS_MODE_FCLK_DELAY_POS)) /**< BYPASS_MODE_FCLK_DELAY Mask */
#define MXC_V_SPIXFM_BYPASS_MODE_FCLK_DELAY_0_NS       ((uint32_t)0x0UL) /**< BYPASS_MODE_FCLK_DELAY_0_NS Value */
#define MXC_S_SPIXFM_BYPASS_MODE_FCLK_DELAY_0_NS       (MXC_V_SPIXFM_BYPASS_MODE_FCLK_DELAY_0_NS << MXC_F_SPIXFM_BYPASS_MODE_FCLK_DELAY_POS) /**< BYPASS_MODE_FCLK_DELAY_0_NS Setting */
#define MXC_V_SPIXFM_BYPASS_MODE_FCLK_DELAY_0P5_NS     ((uint32_t)0x1UL) /**< BYPASS_MODE_FCLK_DELAY_0P5_NS Value */
#define MXC_S_SPIXFM_BYPASS_MODE_FCLK_DELAY_0P5_NS     (MXC_V_SPIXFM_BYPASS_MODE_FCLK_DELAY_0P5_NS << MXC_F_SPIXFM_BYPASS_MODE_FCLK_DELAY_POS) /**< BYPASS_MODE_FCLK_DELAY_0P5_NS Setting */
#define MXC_V_SPIXFM_BYPASS_MODE_FCLK_DELAY_1P0_NS     ((uint32_t)0x2UL) /**< BYPASS_MODE_FCLK_DELAY_1P0_NS Value */
#define MXC_S_SPIXFM_BYPASS_MODE_FCLK_DELAY_1P0_NS     (MXC_V_SPIXFM_BYPASS_MODE_FCLK_DELAY_1P0_NS << MXC_F_SPIXFM_BYPASS_MODE_FCLK_DELAY_POS) /**< BYPASS_MODE_FCLK_DELAY_1P0_NS Setting */
#define MXC_V_SPIXFM_BYPASS_MODE_FCLK_DELAY_1P5_NS     ((uint32_t)0x3UL) /**< BYPASS_MODE_FCLK_DELAY_1P5_NS Value */
#define MXC_S_SPIXFM_BYPASS_MODE_FCLK_DELAY_1P5_NS     (MXC_V_SPIXFM_BYPASS_MODE_FCLK_DELAY_1P5_NS << MXC_F_SPIXFM_BYPASS_MODE_FCLK_DELAY_POS) /**< BYPASS_MODE_FCLK_DELAY_1P5_NS Setting */
#define MXC_V_SPIXFM_BYPASS_MODE_FCLK_DELAY_2P0_NS     ((uint32_t)0x4UL) /**< BYPASS_MODE_FCLK_DELAY_2P0_NS Value */
#define MXC_S_SPIXFM_BYPASS_MODE_FCLK_DELAY_2P0_NS     (MXC_V_SPIXFM_BYPASS_MODE_FCLK_DELAY_2P0_NS << MXC_F_SPIXFM_BYPASS_MODE_FCLK_DELAY_POS) /**< BYPASS_MODE_FCLK_DELAY_2P0_NS Setting */
#define MXC_V_SPIXFM_BYPASS_MODE_FCLK_DELAY_2P5_NS     ((uint32_t)0x5UL) /**< BYPASS_MODE_FCLK_DELAY_2P5_NS Value */
#define MXC_S_SPIXFM_BYPASS_MODE_FCLK_DELAY_2P5_NS     (MXC_V_SPIXFM_BYPASS_MODE_FCLK_DELAY_2P5_NS << MXC_F_SPIXFM_BYPASS_MODE_FCLK_DELAY_POS) /**< BYPASS_MODE_FCLK_DELAY_2P5_NS Setting */
#define MXC_V_SPIXFM_BYPASS_MODE_FCLK_DELAY_3P0_NS     ((uint32_t)0x6UL) /**< BYPASS_MODE_FCLK_DELAY_3P0_NS Value */
#define MXC_S_SPIXFM_BYPASS_MODE_FCLK_DELAY_3P0_NS     (MXC_V_SPIXFM_BYPASS_MODE_FCLK_DELAY_3P0_NS << MXC_F_SPIXFM_BYPASS_MODE_FCLK_DELAY_POS) /**< BYPASS_MODE_FCLK_DELAY_3P0_NS Setting */
#define MXC_V_SPIXFM_BYPASS_MODE_FCLK_DELAY_3P5_NS     ((uint32_t)0x7UL) /**< BYPASS_MODE_FCLK_DELAY_3P5_NS Value */
#define MXC_S_SPIXFM_BYPASS_MODE_FCLK_DELAY_3P5_NS     (MXC_V_SPIXFM_BYPASS_MODE_FCLK_DELAY_3P5_NS << MXC_F_SPIXFM_BYPASS_MODE_FCLK_DELAY_POS) /**< BYPASS_MODE_FCLK_DELAY_3P5_NS Setting */

#define MXC_F_SPIXFM_BYPASS_MODE_SCLK_DELAY_POS        4 /**< BYPASS_MODE_SCLK_DELAY Position */
#define MXC_F_SPIXFM_BYPASS_MODE_SCLK_DELAY            ((uint32_t)(0x7UL << MXC_F_SPIXFM_BYPASS_MODE_SCLK_DELAY_POS)) /**< BYPASS_MODE_SCLK_DELAY Mask */
#define MXC_V_SPIXFM_BYPASS_MODE_SCLK_DELAY_0_NS       ((uint32_t)0x0UL) /**< BYPASS_MODE_SCLK_DELAY_0_NS Value */
#define MXC_S_SPIXFM_BYPASS_MODE_SCLK_DELAY_0_NS       (MXC_V_SPIXFM_BYPASS_MODE_SCLK_DELAY_0_NS << MXC_F_SPIXFM_BYPASS_MODE_SCLK_DELAY_POS) /**< BYPASS_MODE_SCLK_DELAY_0_NS Setting */
#define MXC_V_SPIXFM_BYPASS_MODE_SCLK_DELAY_0P5_NS     ((uint32_t)0x1UL) /**< BYPASS_MODE_SCLK_DELAY_0P5_NS Value */
#define MXC_S_SPIXFM_BYPASS_MODE_SCLK_DELAY_0P5_NS     (MXC_V_SPIXFM_BYPASS_MODE_SCLK_DELAY_0P5_NS << MXC_F_SPIXFM_BYPASS_MODE_SCLK_DELAY_POS) /**< BYPASS_MODE_SCLK_DELAY_0P5_NS Setting */
#define MXC_V_SPIXFM_BYPASS_MODE_SCLK_DELAY_1P0_NS     ((uint32_t)0x2UL) /**< BYPASS_MODE_SCLK_DELAY_1P0_NS Value */
#define MXC_S_SPIXFM_BYPASS_MODE_SCLK_DELAY_1P0_NS     (MXC_V_SPIXFM_BYPASS_MODE_SCLK_DELAY_1P0_NS << MXC_F_SPIXFM_BYPASS_MODE_SCLK_DELAY_POS) /**< BYPASS_MODE_SCLK_DELAY_1P0_NS Setting */
#define MXC_V_SPIXFM_BYPASS_MODE_SCLK_DELAY_1P5_NS     ((uint32_t)0x3UL) /**< BYPASS_MODE_SCLK_DELAY_1P5_NS Value */
#define MXC_S_SPIXFM_BYPASS_MODE_SCLK_DELAY_1P5_NS     (MXC_V_SPIXFM_BYPASS_MODE_SCLK_DELAY_1P5_NS << MXC_F_SPIXFM_BYPASS_MODE_SCLK_DELAY_POS) /**< BYPASS_MODE_SCLK_DELAY_1P5_NS Setting */
#define MXC_V_SPIXFM_BYPASS_MODE_SCLK_DELAY_2P0_NS     ((uint32_t)0x4UL) /**< BYPASS_MODE_SCLK_DELAY_2P0_NS Value */
#define MXC_S_SPIXFM_BYPASS_MODE_SCLK_DELAY_2P0_NS     (MXC_V_SPIXFM_BYPASS_MODE_SCLK_DELAY_2P0_NS << MXC_F_SPIXFM_BYPASS_MODE_SCLK_DELAY_POS) /**< BYPASS_MODE_SCLK_DELAY_2P0_NS Setting */
#define MXC_V_SPIXFM_BYPASS_MODE_SCLK_DELAY_2P5_NS     ((uint32_t)0x5UL) /**< BYPASS_MODE_SCLK_DELAY_2P5_NS Value */
#define MXC_S_SPIXFM_BYPASS_MODE_SCLK_DELAY_2P5_NS     (MXC_V_SPIXFM_BYPASS_MODE_SCLK_DELAY_2P5_NS << MXC_F_SPIXFM_BYPASS_MODE_SCLK_DELAY_POS) /**< BYPASS_MODE_SCLK_DELAY_2P5_NS Setting */
#define MXC_V_SPIXFM_BYPASS_MODE_SCLK_DELAY_3P0_NS     ((uint32_t)0x6UL) /**< BYPASS_MODE_SCLK_DELAY_3P0_NS Value */
#define MXC_S_SPIXFM_BYPASS_MODE_SCLK_DELAY_3P0_NS     (MXC_V_SPIXFM_BYPASS_MODE_SCLK_DELAY_3P0_NS << MXC_F_SPIXFM_BYPASS_MODE_SCLK_DELAY_POS) /**< BYPASS_MODE_SCLK_DELAY_3P0_NS Setting */
#define MXC_V_SPIXFM_BYPASS_MODE_SCLK_DELAY_3P5_NS     ((uint32_t)0x7UL) /**< BYPASS_MODE_SCLK_DELAY_3P5_NS Value */
#define MXC_S_SPIXFM_BYPASS_MODE_SCLK_DELAY_3P5_NS     (MXC_V_SPIXFM_BYPASS_MODE_SCLK_DELAY_3P5_NS << MXC_F_SPIXFM_BYPASS_MODE_SCLK_DELAY_POS) /**< BYPASS_MODE_SCLK_DELAY_3P5_NS Setting */

/**@} end of group SPIXFM_BYPASS_MODE_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32572_INCLUDE_SPIXFM_REGS_H_
