/**
 * @file    owm_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the OWM Peripheral Module.
 * @note    This file is @generated.
 */

/******************************************************************************
 * Copyright (C) 2022 Maxim Integrated Products, Inc., All rights Reserved.
 * 
 * This software is protected by copyright laws of the United States and
 * of foreign countries. This material may also be protected by patent laws
 * and technology transfer regulations of the United States and of foreign
 * countries. This software is furnished under a license agreement and/or a
 * nondisclosure agreement and may only be used or reproduced in accordance
 * with the terms of those agreements. Dissemination of this information to
 * any party or parties not specified in the license agreement and/or
 * nondisclosure agreement is expressly prohibited.
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
 ******************************************************************************/

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32570_INCLUDE_OWM_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32570_INCLUDE_OWM_REGS_H_

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
 * @ingroup     owm
 * @defgroup    owm_registers OWM_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the OWM Peripheral Module.
 * @details     1-Wire Master Interface.
 */

/**
 * @ingroup owm_registers
 * Structure type to access the OWM Registers.
 */
typedef struct {
    __IO uint32_t ctrl0;                /**< <tt>\b 0x0000:</tt> OWM CTRL0 Register */
    __IO uint32_t clkdiv;               /**< <tt>\b 0x0004:</tt> OWM CLKDIV Register */
    __IO uint32_t ctrl1;                /**< <tt>\b 0x0008:</tt> OWM CTRL1 Register */
    __IO uint32_t data;                 /**< <tt>\b 0x000C:</tt> OWM DATA Register */
    __IO uint32_t intfl;                /**< <tt>\b 0x0010:</tt> OWM INTFL Register */
    __IO uint32_t inten;                /**< <tt>\b 0x0014:</tt> OWM INTEN Register */
} mxc_owm_regs_t;

/* Register offsets for module OWM */
/**
 * @ingroup    owm_registers
 * @defgroup   OWM_Register_Offsets Register Offsets
 * @brief      OWM Peripheral Register Offsets from the OWM Base Peripheral Address.
 * @{
 */
#define MXC_R_OWM_CTRL0                    ((uint32_t)0x00000000UL) /**< Offset from OWM Base Address: <tt> 0x0000</tt> */
#define MXC_R_OWM_CLKDIV                   ((uint32_t)0x00000004UL) /**< Offset from OWM Base Address: <tt> 0x0004</tt> */
#define MXC_R_OWM_CTRL1                    ((uint32_t)0x00000008UL) /**< Offset from OWM Base Address: <tt> 0x0008</tt> */
#define MXC_R_OWM_DATA                     ((uint32_t)0x0000000CUL) /**< Offset from OWM Base Address: <tt> 0x000C</tt> */
#define MXC_R_OWM_INTFL                    ((uint32_t)0x00000010UL) /**< Offset from OWM Base Address: <tt> 0x0010</tt> */
#define MXC_R_OWM_INTEN                    ((uint32_t)0x00000014UL) /**< Offset from OWM Base Address: <tt> 0x0014</tt> */
/**@} end of group owm_registers */

/**
 * @ingroup  owm_registers
 * @defgroup OWM_CTRL0 OWM_CTRL0
 * @brief    1-Wire Master Control Register.
 * @{
 */
#define MXC_F_OWM_CTRL0_LL_EN_POS                      0 /**< CTRL0_LL_EN Position */
#define MXC_F_OWM_CTRL0_LL_EN                          ((uint32_t)(0x1UL << MXC_F_OWM_CTRL0_LL_EN_POS)) /**< CTRL0_LL_EN Mask */

#define MXC_F_OWM_CTRL0_FPRESDET_POS                   1 /**< CTRL0_FPRESDET Position */
#define MXC_F_OWM_CTRL0_FPRESDET                       ((uint32_t)(0x1UL << MXC_F_OWM_CTRL0_FPRESDET_POS)) /**< CTRL0_FPRESDET Mask */

#define MXC_F_OWM_CTRL0_BB_EN_POS                      2 /**< CTRL0_BB_EN Position */
#define MXC_F_OWM_CTRL0_BB_EN                          ((uint32_t)(0x1UL << MXC_F_OWM_CTRL0_BB_EN_POS)) /**< CTRL0_BB_EN Mask */

#define MXC_F_OWM_CTRL0_EXT_PU_MODE_POS                3 /**< CTRL0_EXT_PU_MODE Position */
#define MXC_F_OWM_CTRL0_EXT_PU_MODE                    ((uint32_t)(0x1UL << MXC_F_OWM_CTRL0_EXT_PU_MODE_POS)) /**< CTRL0_EXT_PU_MODE Mask */

#define MXC_F_OWM_CTRL0_EXT_PU_EN_POS                  4 /**< CTRL0_EXT_PU_EN Position */
#define MXC_F_OWM_CTRL0_EXT_PU_EN                      ((uint32_t)(0x1UL << MXC_F_OWM_CTRL0_EXT_PU_EN_POS)) /**< CTRL0_EXT_PU_EN Mask */

#define MXC_F_OWM_CTRL0_SB_EN_POS                      5 /**< CTRL0_SB_EN Position */
#define MXC_F_OWM_CTRL0_SB_EN                          ((uint32_t)(0x1UL << MXC_F_OWM_CTRL0_SB_EN_POS)) /**< CTRL0_SB_EN Mask */

#define MXC_F_OWM_CTRL0_OD_POS                         6 /**< CTRL0_OD Position */
#define MXC_F_OWM_CTRL0_OD                             ((uint32_t)(0x1UL << MXC_F_OWM_CTRL0_OD_POS)) /**< CTRL0_OD Mask */

#define MXC_F_OWM_CTRL0_INT_PU_EN_POS                  7 /**< CTRL0_INT_PU_EN Position */
#define MXC_F_OWM_CTRL0_INT_PU_EN                      ((uint32_t)(0x1UL << MXC_F_OWM_CTRL0_INT_PU_EN_POS)) /**< CTRL0_INT_PU_EN Mask */

/**@} end of group OWM_CTRL0_Register */

/**
 * @ingroup  owm_registers
 * @defgroup OWM_CLKDIV OWM_CLKDIV
 * @brief    1-Wire Master Clock Divisor.
 * @{
 */
#define MXC_F_OWM_CLKDIV_DIVISOR_POS                   0 /**< CLKDIV_DIVISOR Position */
#define MXC_F_OWM_CLKDIV_DIVISOR                       ((uint32_t)(0xFFUL << MXC_F_OWM_CLKDIV_DIVISOR_POS)) /**< CLKDIV_DIVISOR Mask */

/**@} end of group OWM_CLKDIV_Register */

/**
 * @ingroup  owm_registers
 * @defgroup OWM_CTRL1 OWM_CTRL1
 * @brief    1-Wire Master Control/Status.
 * @{
 */
#define MXC_F_OWM_CTRL1_RST_POS                        0 /**< CTRL1_RST Position */
#define MXC_F_OWM_CTRL1_RST                            ((uint32_t)(0x1UL << MXC_F_OWM_CTRL1_RST_POS)) /**< CTRL1_RST Mask */

#define MXC_F_OWM_CTRL1_SRA_EN_POS                     1 /**< CTRL1_SRA_EN Position */
#define MXC_F_OWM_CTRL1_SRA_EN                         ((uint32_t)(0x1UL << MXC_F_OWM_CTRL1_SRA_EN_POS)) /**< CTRL1_SRA_EN Mask */

#define MXC_F_OWM_CTRL1_BB_OUT_EN_POS                  2 /**< CTRL1_BB_OUT_EN Position */
#define MXC_F_OWM_CTRL1_BB_OUT_EN                      ((uint32_t)(0x1UL << MXC_F_OWM_CTRL1_BB_OUT_EN_POS)) /**< CTRL1_BB_OUT_EN Mask */

#define MXC_F_OWM_CTRL1_INPUT_ST_POS                   3 /**< CTRL1_INPUT_ST Position */
#define MXC_F_OWM_CTRL1_INPUT_ST                       ((uint32_t)(0x1UL << MXC_F_OWM_CTRL1_INPUT_ST_POS)) /**< CTRL1_INPUT_ST Mask */

#define MXC_F_OWM_CTRL1_OD_SPEC_ST_POS                 4 /**< CTRL1_OD_SPEC_ST Position */
#define MXC_F_OWM_CTRL1_OD_SPEC_ST                     ((uint32_t)(0x1UL << MXC_F_OWM_CTRL1_OD_SPEC_ST_POS)) /**< CTRL1_OD_SPEC_ST Mask */

#define MXC_F_OWM_CTRL1_PRESDET_ST_POS                 5 /**< CTRL1_PRESDET_ST Position */
#define MXC_F_OWM_CTRL1_PRESDET_ST                     ((uint32_t)(0x1UL << MXC_F_OWM_CTRL1_PRESDET_ST_POS)) /**< CTRL1_PRESDET_ST Mask */

/**@} end of group OWM_CTRL1_Register */

/**
 * @ingroup  owm_registers
 * @defgroup OWM_DATA OWM_DATA
 * @brief    1-Wire Master Data Buffer.
 * @{
 */
#define MXC_F_OWM_DATA_DATA_POS                        0 /**< DATA_DATA Position */
#define MXC_F_OWM_DATA_DATA                            ((uint32_t)(0xFFUL << MXC_F_OWM_DATA_DATA_POS)) /**< DATA_DATA Mask */

/**@} end of group OWM_DATA_Register */

/**
 * @ingroup  owm_registers
 * @defgroup OWM_INTFL OWM_INTFL
 * @brief    1-Wire Master Interrupt Flags.
 * @{
 */
#define MXC_F_OWM_INTFL_RST_DONE_POS                   0 /**< INTFL_RST_DONE Position */
#define MXC_F_OWM_INTFL_RST_DONE                       ((uint32_t)(0x1UL << MXC_F_OWM_INTFL_RST_DONE_POS)) /**< INTFL_RST_DONE Mask */

#define MXC_F_OWM_INTFL_TX_EM_POS                      1 /**< INTFL_TX_EM Position */
#define MXC_F_OWM_INTFL_TX_EM                          ((uint32_t)(0x1UL << MXC_F_OWM_INTFL_TX_EM_POS)) /**< INTFL_TX_EM Mask */

#define MXC_F_OWM_INTFL_RX_RDY_POS                     2 /**< INTFL_RX_RDY Position */
#define MXC_F_OWM_INTFL_RX_RDY                         ((uint32_t)(0x1UL << MXC_F_OWM_INTFL_RX_RDY_POS)) /**< INTFL_RX_RDY Mask */

#define MXC_F_OWM_INTFL_LINE_SHORT_POS                 3 /**< INTFL_LINE_SHORT Position */
#define MXC_F_OWM_INTFL_LINE_SHORT                     ((uint32_t)(0x1UL << MXC_F_OWM_INTFL_LINE_SHORT_POS)) /**< INTFL_LINE_SHORT Mask */

#define MXC_F_OWM_INTFL_LINE_LOW_POS                   4 /**< INTFL_LINE_LOW Position */
#define MXC_F_OWM_INTFL_LINE_LOW                       ((uint32_t)(0x1UL << MXC_F_OWM_INTFL_LINE_LOW_POS)) /**< INTFL_LINE_LOW Mask */

/**@} end of group OWM_INTFL_Register */

/**
 * @ingroup  owm_registers
 * @defgroup OWM_INTEN OWM_INTEN
 * @brief    1-Wire Master Interrupt Enables.
 * @{
 */
#define MXC_F_OWM_INTEN_RST_DONE_POS                   0 /**< INTEN_RST_DONE Position */
#define MXC_F_OWM_INTEN_RST_DONE                       ((uint32_t)(0x1UL << MXC_F_OWM_INTEN_RST_DONE_POS)) /**< INTEN_RST_DONE Mask */

#define MXC_F_OWM_INTEN_TX_EM_POS                      1 /**< INTEN_TX_EM Position */
#define MXC_F_OWM_INTEN_TX_EM                          ((uint32_t)(0x1UL << MXC_F_OWM_INTEN_TX_EM_POS)) /**< INTEN_TX_EM Mask */

#define MXC_F_OWM_INTEN_RX_RDY_POS                     2 /**< INTEN_RX_RDY Position */
#define MXC_F_OWM_INTEN_RX_RDY                         ((uint32_t)(0x1UL << MXC_F_OWM_INTEN_RX_RDY_POS)) /**< INTEN_RX_RDY Mask */

#define MXC_F_OWM_INTEN_LINE_SHORT_POS                 3 /**< INTEN_LINE_SHORT Position */
#define MXC_F_OWM_INTEN_LINE_SHORT                     ((uint32_t)(0x1UL << MXC_F_OWM_INTEN_LINE_SHORT_POS)) /**< INTEN_LINE_SHORT Mask */

#define MXC_F_OWM_INTEN_LINE_LOW_POS                   4 /**< INTEN_LINE_LOW Position */
#define MXC_F_OWM_INTEN_LINE_LOW                       ((uint32_t)(0x1UL << MXC_F_OWM_INTEN_LINE_LOW_POS)) /**< INTEN_LINE_LOW Mask */

/**@} end of group OWM_INTEN_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32570_INCLUDE_OWM_REGS_H_
