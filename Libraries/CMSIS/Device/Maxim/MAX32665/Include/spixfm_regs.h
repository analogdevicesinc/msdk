/**
 * @file    spixfm_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the SPIXFM Peripheral Module.
 */

/* ****************************************************************************
 * Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *
 *
 *************************************************************************** */

#ifndef _SPIXFM_REGS_H_
#define _SPIXFM_REGS_H_

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
 * @details SPIXF Master
 */

/**
 * @ingroup spixfm_registers
 * Structure type to access the SPIXFM Registers.
 */
typedef struct {
    __IO uint32_t cfg;                  /**< <tt>\b 0x00:</tt> SPIXFM CFG Register */
    __IO uint32_t fetch_ctrl;           /**< <tt>\b 0x04:</tt> SPIXFM FETCH_CTRL Register */
    __IO uint32_t mode_ctrl;            /**< <tt>\b 0x08:</tt> SPIXFM MODE_CTRL Register */
    __IO uint32_t mode_data;            /**< <tt>\b 0x0C:</tt> SPIXFM MODE_DATA Register */
    __IO uint32_t sclk_fb_ctrl;         /**< <tt>\b 0x10:</tt> SPIXFM SCLK_FB_CTRL Register */
    __R  uint32_t rsv_0x14_0x1b[2];
    __IO uint32_t io_ctrl;              /**< <tt>\b 0x1C:</tt> SPIXFM IO_CTRL Register */
    __IO uint32_t memseccn;             /**< <tt>\b 0x20:</tt> SPIXFM MEMSECCN Register */
    __IO uint32_t bus_idle;             /**< <tt>\b 0x24:</tt> SPIXFM BUS_IDLE Register */
} mxc_spixfm_regs_t;

/**
 * @ingroup  spixfm_registers
 * @defgroup SPIXFM_CFG SPIXFM_CFG
 * @brief    SPIX Configuration Register.
 * @{
 */
 #define MXC_F_SPIXFM_CFG_MODE_POS                      0 /**< CFG_MODE Position */
 #define MXC_F_SPIXFM_CFG_MODE                          ((uint32_t)(0x3UL << MXC_F_SPIXFM_CFG_MODE_POS)) /**< CFG_MODE Mask */
 #define MXC_V_SPIXFM_CFG_MODE_SCLK_HI_SAMPLE_RISING    ((uint32_t)0x0UL) /**< CFG_MODE_SCLK_HI_SAMPLE_RISING Value */
 #define MXC_S_SPIXFM_CFG_MODE_SCLK_HI_SAMPLE_RISING    (MXC_V_SPIXFM_CFG_MODE_SCLK_HI_SAMPLE_RISING << MXC_F_SPIXFM_CFG_MODE_POS) /**< CFG_MODE_SCLK_HI_SAMPLE_RISING Setting */
 #define MXC_V_SPIXFM_CFG_MODE_SCLK_LO_SAMPLE_FAILLING  ((uint32_t)0x3UL) /**< CFG_MODE_SCLK_LO_SAMPLE_FAILLING Value */
 #define MXC_S_SPIXFM_CFG_MODE_SCLK_LO_SAMPLE_FAILLING  (MXC_V_SPIXFM_CFG_MODE_SCLK_LO_SAMPLE_FAILLING << MXC_F_SPIXFM_CFG_MODE_POS) /**< CFG_MODE_SCLK_LO_SAMPLE_FAILLING Setting */

 #define MXC_F_SPIXFM_CFG_SSPOL_POS                     2 /**< CFG_SSPOL Position */
 #define MXC_F_SPIXFM_CFG_SSPOL                         ((uint32_t)(0x1UL << MXC_F_SPIXFM_CFG_SSPOL_POS)) /**< CFG_SSPOL Mask */

 #define MXC_F_SPIXFM_CFG_SSEL_POS                      4 /**< CFG_SSEL Position */
 #define MXC_F_SPIXFM_CFG_SSEL                          ((uint32_t)(0x7UL << MXC_F_SPIXFM_CFG_SSEL_POS)) /**< CFG_SSEL Mask */

 #define MXC_F_SPIXFM_CFG_LO_CLK_POS                    8 /**< CFG_LO_CLK Position */
 #define MXC_F_SPIXFM_CFG_LO_CLK                        ((uint32_t)(0xFUL << MXC_F_SPIXFM_CFG_LO_CLK_POS)) /**< CFG_LO_CLK Mask */

 #define MXC_F_SPIXFM_CFG_HI_CLK_POS                    12 /**< CFG_HI_CLK Position */
 #define MXC_F_SPIXFM_CFG_HI_CLK                        ((uint32_t)(0xFUL << MXC_F_SPIXFM_CFG_HI_CLK_POS)) /**< CFG_HI_CLK Mask */

 #define MXC_F_SPIXFM_CFG_SSACT_POS                     16 /**< CFG_SSACT Position */
 #define MXC_F_SPIXFM_CFG_SSACT                         ((uint32_t)(0x3UL << MXC_F_SPIXFM_CFG_SSACT_POS)) /**< CFG_SSACT Mask */
 #define MXC_V_SPIXFM_CFG_SSACT_OFF                     ((uint32_t)0x0UL) /**< CFG_SSACT_OFF Value */
 #define MXC_S_SPIXFM_CFG_SSACT_OFF                     (MXC_V_SPIXFM_CFG_SSACT_OFF << MXC_F_SPIXFM_CFG_SSACT_POS) /**< CFG_SSACT_OFF Setting */
 #define MXC_V_SPIXFM_CFG_SSACT_FOR_2_MOD_CLK           ((uint32_t)0x1UL) /**< CFG_SSACT_FOR_2_MOD_CLK Value */
 #define MXC_S_SPIXFM_CFG_SSACT_FOR_2_MOD_CLK           (MXC_V_SPIXFM_CFG_SSACT_FOR_2_MOD_CLK << MXC_F_SPIXFM_CFG_SSACT_POS) /**< CFG_SSACT_FOR_2_MOD_CLK Setting */
 #define MXC_V_SPIXFM_CFG_SSACT_FOR_4_MOD_CLK           ((uint32_t)0x2UL) /**< CFG_SSACT_FOR_4_MOD_CLK Value */
 #define MXC_S_SPIXFM_CFG_SSACT_FOR_4_MOD_CLK           (MXC_V_SPIXFM_CFG_SSACT_FOR_4_MOD_CLK << MXC_F_SPIXFM_CFG_SSACT_POS) /**< CFG_SSACT_FOR_4_MOD_CLK Setting */
 #define MXC_V_SPIXFM_CFG_SSACT_FOR_8_MOD_CLK           ((uint32_t)0x3UL) /**< CFG_SSACT_FOR_8_MOD_CLK Value */
 #define MXC_S_SPIXFM_CFG_SSACT_FOR_8_MOD_CLK           (MXC_V_SPIXFM_CFG_SSACT_FOR_8_MOD_CLK << MXC_F_SPIXFM_CFG_SSACT_POS) /**< CFG_SSACT_FOR_8_MOD_CLK Setting */

 #define MXC_F_SPIXFM_CFG_SSIACT_POS                    18 /**< CFG_SSIACT Position */
 #define MXC_F_SPIXFM_CFG_SSIACT                        ((uint32_t)(0x3UL << MXC_F_SPIXFM_CFG_SSIACT_POS)) /**< CFG_SSIACT Mask */
 #define MXC_V_SPIXFM_CFG_SSIACT_FOR_1_MOD_CLK          ((uint32_t)0x0UL) /**< CFG_SSIACT_FOR_1_MOD_CLK Value */
 #define MXC_S_SPIXFM_CFG_SSIACT_FOR_1_MOD_CLK          (MXC_V_SPIXFM_CFG_SSIACT_FOR_1_MOD_CLK << MXC_F_SPIXFM_CFG_SSIACT_POS) /**< CFG_SSIACT_FOR_1_MOD_CLK Setting */
 #define MXC_V_SPIXFM_CFG_SSIACT_FOR_3_MOD_CLK          ((uint32_t)0x1UL) /**< CFG_SSIACT_FOR_3_MOD_CLK Value */
 #define MXC_S_SPIXFM_CFG_SSIACT_FOR_3_MOD_CLK          (MXC_V_SPIXFM_CFG_SSIACT_FOR_3_MOD_CLK << MXC_F_SPIXFM_CFG_SSIACT_POS) /**< CFG_SSIACT_FOR_3_MOD_CLK Setting */
 #define MXC_V_SPIXFM_CFG_SSIACT_FOR_5_MOD_CLK          ((uint32_t)0x2UL) /**< CFG_SSIACT_FOR_5_MOD_CLK Value */
 #define MXC_S_SPIXFM_CFG_SSIACT_FOR_5_MOD_CLK          (MXC_V_SPIXFM_CFG_SSIACT_FOR_5_MOD_CLK << MXC_F_SPIXFM_CFG_SSIACT_POS) /**< CFG_SSIACT_FOR_5_MOD_CLK Setting */
 #define MXC_V_SPIXFM_CFG_SSIACT_FOR_9_MOD_CLK          ((uint32_t)0x3UL) /**< CFG_SSIACT_FOR_9_MOD_CLK Value */
 #define MXC_S_SPIXFM_CFG_SSIACT_FOR_9_MOD_CLK          (MXC_V_SPIXFM_CFG_SSIACT_FOR_9_MOD_CLK << MXC_F_SPIXFM_CFG_SSIACT_POS) /**< CFG_SSIACT_FOR_9_MOD_CLK Setting */

/**@} end of group SPIXFM_CFG_Register */

/**
 * @ingroup  spixfm_registers
 * @defgroup SPIXFM_FETCH_CTRL SPIXFM_FETCH_CTRL
 * @brief    SPIX Fetch Control Register.
 * @{
 */
 #define MXC_F_SPIXFM_FETCH_CTRL_CMDVAL_POS             0 /**< FETCH_CTRL_CMDVAL Position */
 #define MXC_F_SPIXFM_FETCH_CTRL_CMDVAL                 ((uint32_t)(0xFFUL << MXC_F_SPIXFM_FETCH_CTRL_CMDVAL_POS)) /**< FETCH_CTRL_CMDVAL Mask */

 #define MXC_F_SPIXFM_FETCH_CTRL_CMD_WIDTH_POS          8 /**< FETCH_CTRL_CMD_WIDTH Position */
 #define MXC_F_SPIXFM_FETCH_CTRL_CMD_WIDTH              ((uint32_t)(0x3UL << MXC_F_SPIXFM_FETCH_CTRL_CMD_WIDTH_POS)) /**< FETCH_CTRL_CMD_WIDTH Mask */
 #define MXC_V_SPIXFM_FETCH_CTRL_CMD_WIDTH_SINGLE       ((uint32_t)0x0UL) /**< FETCH_CTRL_CMD_WIDTH_SINGLE Value */
 #define MXC_S_SPIXFM_FETCH_CTRL_CMD_WIDTH_SINGLE       (MXC_V_SPIXFM_FETCH_CTRL_CMD_WIDTH_SINGLE << MXC_F_SPIXFM_FETCH_CTRL_CMD_WIDTH_POS) /**< FETCH_CTRL_CMD_WIDTH_SINGLE Setting */
 #define MXC_V_SPIXFM_FETCH_CTRL_CMD_WIDTH_DUAL_IO      ((uint32_t)0x1UL) /**< FETCH_CTRL_CMD_WIDTH_DUAL_IO Value */
 #define MXC_S_SPIXFM_FETCH_CTRL_CMD_WIDTH_DUAL_IO      (MXC_V_SPIXFM_FETCH_CTRL_CMD_WIDTH_DUAL_IO << MXC_F_SPIXFM_FETCH_CTRL_CMD_WIDTH_POS) /**< FETCH_CTRL_CMD_WIDTH_DUAL_IO Setting */
 #define MXC_V_SPIXFM_FETCH_CTRL_CMD_WIDTH_QUAD_IO      ((uint32_t)0x2UL) /**< FETCH_CTRL_CMD_WIDTH_QUAD_IO Value */
 #define MXC_S_SPIXFM_FETCH_CTRL_CMD_WIDTH_QUAD_IO      (MXC_V_SPIXFM_FETCH_CTRL_CMD_WIDTH_QUAD_IO << MXC_F_SPIXFM_FETCH_CTRL_CMD_WIDTH_POS) /**< FETCH_CTRL_CMD_WIDTH_QUAD_IO Setting */
 #define MXC_V_SPIXFM_FETCH_CTRL_CMD_WIDTH_INVALID      ((uint32_t)0x3UL) /**< FETCH_CTRL_CMD_WIDTH_INVALID Value */
 #define MXC_S_SPIXFM_FETCH_CTRL_CMD_WIDTH_INVALID      (MXC_V_SPIXFM_FETCH_CTRL_CMD_WIDTH_INVALID << MXC_F_SPIXFM_FETCH_CTRL_CMD_WIDTH_POS) /**< FETCH_CTRL_CMD_WIDTH_INVALID Setting */

 #define MXC_F_SPIXFM_FETCH_CTRL_ADDR_WIDTH_POS         10 /**< FETCH_CTRL_ADDR_WIDTH Position */
 #define MXC_F_SPIXFM_FETCH_CTRL_ADDR_WIDTH             ((uint32_t)(0x3UL << MXC_F_SPIXFM_FETCH_CTRL_ADDR_WIDTH_POS)) /**< FETCH_CTRL_ADDR_WIDTH Mask */
 #define MXC_V_SPIXFM_FETCH_CTRL_ADDR_WIDTH_SINGLE      ((uint32_t)0x0UL) /**< FETCH_CTRL_ADDR_WIDTH_SINGLE Value */
 #define MXC_S_SPIXFM_FETCH_CTRL_ADDR_WIDTH_SINGLE      (MXC_V_SPIXFM_FETCH_CTRL_ADDR_WIDTH_SINGLE << MXC_F_SPIXFM_FETCH_CTRL_ADDR_WIDTH_POS) /**< FETCH_CTRL_ADDR_WIDTH_SINGLE Setting */
 #define MXC_V_SPIXFM_FETCH_CTRL_ADDR_WIDTH_DUAL_IO     ((uint32_t)0x1UL) /**< FETCH_CTRL_ADDR_WIDTH_DUAL_IO Value */
 #define MXC_S_SPIXFM_FETCH_CTRL_ADDR_WIDTH_DUAL_IO     (MXC_V_SPIXFM_FETCH_CTRL_ADDR_WIDTH_DUAL_IO << MXC_F_SPIXFM_FETCH_CTRL_ADDR_WIDTH_POS) /**< FETCH_CTRL_ADDR_WIDTH_DUAL_IO Setting */
 #define MXC_V_SPIXFM_FETCH_CTRL_ADDR_WIDTH_QUAD_IO     ((uint32_t)0x2UL) /**< FETCH_CTRL_ADDR_WIDTH_QUAD_IO Value */
 #define MXC_S_SPIXFM_FETCH_CTRL_ADDR_WIDTH_QUAD_IO     (MXC_V_SPIXFM_FETCH_CTRL_ADDR_WIDTH_QUAD_IO << MXC_F_SPIXFM_FETCH_CTRL_ADDR_WIDTH_POS) /**< FETCH_CTRL_ADDR_WIDTH_QUAD_IO Setting */
 #define MXC_V_SPIXFM_FETCH_CTRL_ADDR_WIDTH_INVALID     ((uint32_t)0x3UL) /**< FETCH_CTRL_ADDR_WIDTH_INVALID Value */
 #define MXC_S_SPIXFM_FETCH_CTRL_ADDR_WIDTH_INVALID     (MXC_V_SPIXFM_FETCH_CTRL_ADDR_WIDTH_INVALID << MXC_F_SPIXFM_FETCH_CTRL_ADDR_WIDTH_POS) /**< FETCH_CTRL_ADDR_WIDTH_INVALID Setting */

 #define MXC_F_SPIXFM_FETCH_CTRL_DATA_WIDTH_POS         12 /**< FETCH_CTRL_DATA_WIDTH Position */
 #define MXC_F_SPIXFM_FETCH_CTRL_DATA_WIDTH             ((uint32_t)(0x3UL << MXC_F_SPIXFM_FETCH_CTRL_DATA_WIDTH_POS)) /**< FETCH_CTRL_DATA_WIDTH Mask */
 #define MXC_V_SPIXFM_FETCH_CTRL_DATA_WIDTH_SINGLE      ((uint32_t)0x0UL) /**< FETCH_CTRL_DATA_WIDTH_SINGLE Value */
 #define MXC_S_SPIXFM_FETCH_CTRL_DATA_WIDTH_SINGLE      (MXC_V_SPIXFM_FETCH_CTRL_DATA_WIDTH_SINGLE << MXC_F_SPIXFM_FETCH_CTRL_DATA_WIDTH_POS) /**< FETCH_CTRL_DATA_WIDTH_SINGLE Setting */
 #define MXC_V_SPIXFM_FETCH_CTRL_DATA_WIDTH_DUAL_IO     ((uint32_t)0x1UL) /**< FETCH_CTRL_DATA_WIDTH_DUAL_IO Value */
 #define MXC_S_SPIXFM_FETCH_CTRL_DATA_WIDTH_DUAL_IO     (MXC_V_SPIXFM_FETCH_CTRL_DATA_WIDTH_DUAL_IO << MXC_F_SPIXFM_FETCH_CTRL_DATA_WIDTH_POS) /**< FETCH_CTRL_DATA_WIDTH_DUAL_IO Setting */
 #define MXC_V_SPIXFM_FETCH_CTRL_DATA_WIDTH_QUAD_IO     ((uint32_t)0x2UL) /**< FETCH_CTRL_DATA_WIDTH_QUAD_IO Value */
 #define MXC_S_SPIXFM_FETCH_CTRL_DATA_WIDTH_QUAD_IO     (MXC_V_SPIXFM_FETCH_CTRL_DATA_WIDTH_QUAD_IO << MXC_F_SPIXFM_FETCH_CTRL_DATA_WIDTH_POS) /**< FETCH_CTRL_DATA_WIDTH_QUAD_IO Setting */
 #define MXC_V_SPIXFM_FETCH_CTRL_DATA_WIDTH_INVALID     ((uint32_t)0x3UL) /**< FETCH_CTRL_DATA_WIDTH_INVALID Value */
 #define MXC_S_SPIXFM_FETCH_CTRL_DATA_WIDTH_INVALID     (MXC_V_SPIXFM_FETCH_CTRL_DATA_WIDTH_INVALID << MXC_F_SPIXFM_FETCH_CTRL_DATA_WIDTH_POS) /**< FETCH_CTRL_DATA_WIDTH_INVALID Setting */

 #define MXC_F_SPIXFM_FETCH_CTRL_FOUR_BYTE_ADDR_POS     16 /**< FETCH_CTRL_FOUR_BYTE_ADDR Position */
 #define MXC_F_SPIXFM_FETCH_CTRL_FOUR_BYTE_ADDR         ((uint32_t)(0x1UL << MXC_F_SPIXFM_FETCH_CTRL_FOUR_BYTE_ADDR_POS)) /**< FETCH_CTRL_FOUR_BYTE_ADDR Mask */

/**@} end of group SPIXFM_FETCH_CTRL_Register */

/**
 * @ingroup  spixfm_registers
 * @defgroup SPIXFM_MODE_CTRL SPIXFM_MODE_CTRL
 * @brief    SPIX Mode Control Register.
 * @{
 */
 #define MXC_F_SPIXFM_MODE_CTRL_MDCLK_POS               0 /**< MODE_CTRL_MDCLK Position */
 #define MXC_F_SPIXFM_MODE_CTRL_MDCLK                   ((uint32_t)(0xFUL << MXC_F_SPIXFM_MODE_CTRL_MDCLK_POS)) /**< MODE_CTRL_MDCLK Mask */

 #define MXC_F_SPIXFM_MODE_CTRL_NO_CMD_POS              8 /**< MODE_CTRL_NO_CMD Position */
 #define MXC_F_SPIXFM_MODE_CTRL_NO_CMD                  ((uint32_t)(0x1UL << MXC_F_SPIXFM_MODE_CTRL_NO_CMD_POS)) /**< MODE_CTRL_NO_CMD Mask */

 #define MXC_F_SPIXFM_MODE_CTRL_MODE_SEND_POS           9 /**< MODE_CTRL_MODE_SEND Position */
 #define MXC_F_SPIXFM_MODE_CTRL_MODE_SEND               ((uint32_t)(0x1UL << MXC_F_SPIXFM_MODE_CTRL_MODE_SEND_POS)) /**< MODE_CTRL_MODE_SEND Mask */

/**@} end of group SPIXFM_MODE_CTRL_Register */

/**
 * @ingroup  spixfm_registers
 * @defgroup SPIXFM_MODE_DATA SPIXFM_MODE_DATA
 * @brief    SPIX Mode Data Register.
 * @{
 */
 #define MXC_F_SPIXFM_MODE_DATA_DATA_POS                0 /**< MODE_DATA_DATA Position */
 #define MXC_F_SPIXFM_MODE_DATA_DATA                    ((uint32_t)(0xFFFFUL << MXC_F_SPIXFM_MODE_DATA_DATA_POS)) /**< MODE_DATA_DATA Mask */

 #define MXC_F_SPIXFM_MODE_DATA_OUT_EN_POS              16 /**< MODE_DATA_OUT_EN Position */
 #define MXC_F_SPIXFM_MODE_DATA_OUT_EN                  ((uint32_t)(0xFFFFUL << MXC_F_SPIXFM_MODE_DATA_OUT_EN_POS)) /**< MODE_DATA_OUT_EN Mask */

/**@} end of group SPIXFM_MODE_DATA_Register */

/**
 * @ingroup  spixfm_registers
 * @defgroup SPIXFM_SCLK_FB_CTRL SPIXFM_SCLK_FB_CTRL
 * @brief    SPIX Feedback Control Register.
 * @{
 */
 #define MXC_F_SPIXFM_SCLK_FB_CTRL_FB_EN_POS            0 /**< SCLK_FB_CTRL_FB_EN Position */
 #define MXC_F_SPIXFM_SCLK_FB_CTRL_FB_EN                ((uint32_t)(0x1UL << MXC_F_SPIXFM_SCLK_FB_CTRL_FB_EN_POS)) /**< SCLK_FB_CTRL_FB_EN Mask */

 #define MXC_F_SPIXFM_SCLK_FB_CTRL_INVERT_EN_POS        1 /**< SCLK_FB_CTRL_INVERT_EN Position */
 #define MXC_F_SPIXFM_SCLK_FB_CTRL_INVERT_EN            ((uint32_t)(0x1UL << MXC_F_SPIXFM_SCLK_FB_CTRL_INVERT_EN_POS)) /**< SCLK_FB_CTRL_INVERT_EN Mask */

/**@} end of group SPIXFM_SCLK_FB_CTRL_Register */

/**
 * @ingroup  spixfm_registers
 * @defgroup SPIXFM_IO_CTRL SPIXFM_IO_CTRL
 * @brief    SPIX IO Control Register.
 * @{
 */
 #define MXC_F_SPIXFM_IO_CTRL_SCLK_DS_POS               0 /**< IO_CTRL_SCLK_DS Position */
 #define MXC_F_SPIXFM_IO_CTRL_SCLK_DS                   ((uint32_t)(0x1UL << MXC_F_SPIXFM_IO_CTRL_SCLK_DS_POS)) /**< IO_CTRL_SCLK_DS Mask */

 #define MXC_F_SPIXFM_IO_CTRL_SS_DS_POS                 1 /**< IO_CTRL_SS_DS Position */
 #define MXC_F_SPIXFM_IO_CTRL_SS_DS                     ((uint32_t)(0x1UL << MXC_F_SPIXFM_IO_CTRL_SS_DS_POS)) /**< IO_CTRL_SS_DS Mask */

 #define MXC_F_SPIXFM_IO_CTRL_SDIO_DS_POS               2 /**< IO_CTRL_SDIO_DS Position */
 #define MXC_F_SPIXFM_IO_CTRL_SDIO_DS                   ((uint32_t)(0x1UL << MXC_F_SPIXFM_IO_CTRL_SDIO_DS_POS)) /**< IO_CTRL_SDIO_DS Mask */

 #define MXC_F_SPIXFM_IO_CTRL_PU_PD_CTRL_POS            3 /**< IO_CTRL_PU_PD_CTRL Position */
 #define MXC_F_SPIXFM_IO_CTRL_PU_PD_CTRL                ((uint32_t)(0x3UL << MXC_F_SPIXFM_IO_CTRL_PU_PD_CTRL_POS)) /**< IO_CTRL_PU_PD_CTRL Mask */
 #define MXC_V_SPIXFM_IO_CTRL_PU_PD_CTRL_TRI_STATE      ((uint32_t)0x0UL) /**< IO_CTRL_PU_PD_CTRL_TRI_STATE Value */
 #define MXC_S_SPIXFM_IO_CTRL_PU_PD_CTRL_TRI_STATE      (MXC_V_SPIXFM_IO_CTRL_PU_PD_CTRL_TRI_STATE << MXC_F_SPIXFM_IO_CTRL_PU_PD_CTRL_POS) /**< IO_CTRL_PU_PD_CTRL_TRI_STATE Setting */
 #define MXC_V_SPIXFM_IO_CTRL_PU_PD_CTRL_PULL_UP        ((uint32_t)0x1UL) /**< IO_CTRL_PU_PD_CTRL_PULL_UP Value */
 #define MXC_S_SPIXFM_IO_CTRL_PU_PD_CTRL_PULL_UP        (MXC_V_SPIXFM_IO_CTRL_PU_PD_CTRL_PULL_UP << MXC_F_SPIXFM_IO_CTRL_PU_PD_CTRL_POS) /**< IO_CTRL_PU_PD_CTRL_PULL_UP Setting */
 #define MXC_V_SPIXFM_IO_CTRL_PU_PD_CTRL_PULL_DOWN      ((uint32_t)0x2UL) /**< IO_CTRL_PU_PD_CTRL_PULL_DOWN Value */
 #define MXC_S_SPIXFM_IO_CTRL_PU_PD_CTRL_PULL_DOWN      (MXC_V_SPIXFM_IO_CTRL_PU_PD_CTRL_PULL_DOWN << MXC_F_SPIXFM_IO_CTRL_PU_PD_CTRL_POS) /**< IO_CTRL_PU_PD_CTRL_PULL_DOWN Setting */

/**@} end of group SPIXFM_IO_CTRL_Register */

/**
 * @ingroup  spixfm_registers
 * @defgroup SPIXFM_MEMSECCN SPIXFM_MEMSECCN
 * @brief    SPIX Memory Security Control Register.
 * @{
 */
 #define MXC_F_SPIXFM_MEMSECCN_DECEN_POS                0 /**< MEMSECCN_DECEN Position */
 #define MXC_F_SPIXFM_MEMSECCN_DECEN                    ((uint32_t)(0x1UL << MXC_F_SPIXFM_MEMSECCN_DECEN_POS)) /**< MEMSECCN_DECEN Mask */

 #define MXC_F_SPIXFM_MEMSECCN_AUTH_DISABLE_POS         1 /**< MEMSECCN_AUTH_DISABLE Position */
 #define MXC_F_SPIXFM_MEMSECCN_AUTH_DISABLE             ((uint32_t)(0x1UL << MXC_F_SPIXFM_MEMSECCN_AUTH_DISABLE_POS)) /**< MEMSECCN_AUTH_DISABLE Mask */

/**@} end of group SPIXFM_MEMSECCN_Register */

/**
 * @ingroup  spixfm_registers
 * @defgroup SPIXFM_BUS_IDLE SPIXFM_BUS_IDLE
 * @brief    SPIXF Bus Idle Detection.
 * @{
 */
 #define MXC_F_SPIXFM_BUS_IDLE_BUSIDLE_POS              0 /**< BUS_IDLE_BUSIDLE Position */
 #define MXC_F_SPIXFM_BUS_IDLE_BUSIDLE                  ((uint32_t)(0xFFFFUL << MXC_F_SPIXFM_BUS_IDLE_BUSIDLE_POS)) /**< BUS_IDLE_BUSIDLE Mask */

/**@} end of group SPIXFM_BUS_IDLE_Register */

#ifdef __cplusplus
}
#endif

#endif /* _SPIXFM_REGS_H_ */
