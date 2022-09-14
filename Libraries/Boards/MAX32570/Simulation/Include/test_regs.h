/**
 * @file    test_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the TEST Peripheral Module.
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

#ifndef _TEST_REGS_H_
#define _TEST_REGS_H_

/* **** Includes **** */
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
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
 * @ingroup     test
 * @defgroup    test_registers Registers
 * @brief       Registers, Bit Masks and Bit Positions for the TEST Peripheral Module.
 * @description Test Modes
 */

/**
 * @ingroup test_registers
 * Structure type to access the TEST Registers.
 */
typedef struct {
    __IO uint32_t tm;                   /**< <tt>\b 0x00:<\tt> TEST TM Register */
    __IO uint32_t trcn;                 /**< <tt>\b 0x04:<\tt> TEST TRCN Register */
    __IO uint32_t tclk;                 /**< <tt>\b 0x08:<\tt> TEST TCLK Register */
    __IO uint32_t tmr0;                 /**< <tt>\b 0x0C:<\tt> TEST TMR0 Register */
    __IO uint32_t tmr1;                 /**< <tt>\b 0x10:<\tt> TEST TMR1 Register */
    __IO uint32_t tmr2;                 /**< <tt>\b 0x14:<\tt> TEST TMR2 Register */
    __IO uint32_t tmr3;                 /**< <tt>\b 0x18:<\tt> TEST TMR3 Register */
    __IO uint32_t stcr;                 /**< <tt>\b 0x1C:<\tt> TEST STCR Register */
    __IO uint32_t mbcn;                 /**< <tt>\b 0x20:<\tt> TEST MBCN Register */
    __IO uint32_t mbstat;               /**< <tt>\b 0x24:<\tt> TEST MBSTAT Register */
    __IO uint32_t mbdsel;               /**< <tt>\b 0x28:<\tt> TEST MBDSEL Register */
    __IO uint32_t mbdiag;               /**< <tt>\b 0x2C:<\tt> TEST MBDIAG Register */
} mxc_test_regs_t;

/**
 * @ingroup  test_registers
 * @defgroup TM_Register
 * @brief    Test Mode
 * @{
 */
#define MXC_F_TEST_TM_TME_POS                          (0) /**< TM_TME Position */
#define MXC_F_TEST_TM_TME                              ((uint32_t)(0x1 << MXC_F_TEST_TM_TME_POS)) /**< TM_TME Mask */

#define MXC_F_TEST_TM_TTAP_EN_POS                      (1) /**< TM_TTAP_EN Position */
#define MXC_F_TEST_TM_TTAP_EN                          ((uint32_t)(0x1 << MXC_F_TEST_TM_TTAP_EN_POS)) /**< TM_TTAP_EN Mask */

#define MXC_F_TEST_TM_TTAP_PIF_POS                     (2) /**< TM_TTAP_PIF Position */
#define MXC_F_TEST_TM_TTAP_PIF                         ((uint32_t)(0x1 << MXC_F_TEST_TM_TTAP_PIF_POS)) /**< TM_TTAP_PIF Mask */

#define MXC_F_TEST_TM_SCAN_POS                         (3) /**< TM_SCAN Position */
#define MXC_F_TEST_TM_SCAN                             ((uint32_t)(0x1 << MXC_F_TEST_TM_SCAN_POS)) /**< TM_SCAN Mask */

#define MXC_F_TEST_TM_SCANFB_POS                       (4) /**< TM_SCANFB Position */
#define MXC_F_TEST_TM_SCANFB                           ((uint32_t)(0x1 << MXC_F_TEST_TM_SCANFB_POS)) /**< TM_SCANFB Mask */

#define MXC_F_TEST_TM_SRT_POS                          (5) /**< TM_SRT Position */
#define MXC_F_TEST_TM_SRT                              ((uint32_t)(0x1 << MXC_F_TEST_TM_SRT_POS)) /**< TM_SRT Mask */

#define MXC_F_TEST_TM_IDDQ_POS                         (6) /**< TM_IDDQ Position */
#define MXC_F_TEST_TM_IDDQ                             ((uint32_t)(0x1 << MXC_F_TEST_TM_IDDQ_POS)) /**< TM_IDDQ Mask */

#define MXC_F_TEST_TM_XTALSEL_POS                      (7) /**< TM_XTALSEL Position */
#define MXC_F_TEST_TM_XTALSEL                          ((uint32_t)(0x1 << MXC_F_TEST_TM_XTALSEL_POS)) /**< TM_XTALSEL Mask */

#define MXC_F_TEST_TM_LDOOFF_POS                       (9) /**< TM_LDOOFF Position */
#define MXC_F_TEST_TM_LDOOFF                           ((uint32_t)(0x1 << MXC_F_TEST_TM_LDOOFF_POS)) /**< TM_LDOOFF Mask */

#define MXC_F_TEST_TM_FTM_POS                          (10) /**< TM_FTM Position */
#define MXC_F_TEST_TM_FTM                              ((uint32_t)(0x1 << MXC_F_TEST_TM_FTM_POS)) /**< TM_FTM Mask */

#define MXC_F_TEST_TM_SFTM_POS                         (11) /**< TM_SFTM Position */
#define MXC_F_TEST_TM_SFTM                             ((uint32_t)(0x1 << MXC_F_TEST_TM_SFTM_POS)) /**< TM_SFTM Mask */

#define MXC_F_TEST_TM_FBIST_POS                        (12) /**< TM_FBIST Position */
#define MXC_F_TEST_TM_FBIST                            ((uint32_t)(0x1 << MXC_F_TEST_TM_FBIST_POS)) /**< TM_FBIST Mask */

#define MXC_F_TEST_TM_SFBIST_POS                       (13) /**< TM_SFBIST Position */
#define MXC_F_TEST_TM_SFBIST                           ((uint32_t)(0x1 << MXC_F_TEST_TM_SFBIST_POS)) /**< TM_SFBIST Mask */

#define MXC_F_TEST_TM_POR_POS                          (14) /**< TM_POR Position */
#define MXC_F_TEST_TM_POR                              ((uint32_t)(0x1 << MXC_F_TEST_TM_POR_POS)) /**< TM_POR Mask */

#define MXC_F_TEST_TM_BOR_POS                          (15) /**< TM_BOR Position */
#define MXC_F_TEST_TM_BOR                              ((uint32_t)(0x1 << MXC_F_TEST_TM_BOR_POS)) /**< TM_BOR Mask */

#define MXC_F_TEST_TM_SCAN_MD_POS                      (16) /**< TM_SCAN_MD Position */
#define MXC_F_TEST_TM_SCAN_MD                          ((uint32_t)(0x7 << MXC_F_TEST_TM_SCAN_MD_POS)) /**< TM_SCAN_MD Mask */

#define MXC_F_TEST_TM_CKT_POS                          (20) /**< TM_CKT Position */
#define MXC_F_TEST_TM_CKT                              ((uint32_t)(0x1 << MXC_F_TEST_TM_CKT_POS)) /**< TM_CKT Mask */

#define MXC_F_TEST_TM_DOFF_POS                         (21) /**< TM_DOFF Position */
#define MXC_F_TEST_TM_DOFF                             ((uint32_t)(0x1 << MXC_F_TEST_TM_DOFF_POS)) /**< TM_DOFF Mask */

#define MXC_F_TEST_TM_DCW_POS                          (22) /**< TM_DCW Position */
#define MXC_F_TEST_TM_DCW                              ((uint32_t)(0x1 << MXC_F_TEST_TM_DCW_POS)) /**< TM_DCW Mask */

#define MXC_F_TEST_TM_SVM_BG_POS                       (24) /**< TM_SVM_BG Position */
#define MXC_F_TEST_TM_SVM_BG                           ((uint32_t)(0x1 << MXC_F_TEST_TM_SVM_BG_POS)) /**< TM_SVM_BG Mask */

#define MXC_F_TEST_TM_SVM_HV_POS                       (25) /**< TM_SVM_HV Position */
#define MXC_F_TEST_TM_SVM_HV                           ((uint32_t)(0x1 << MXC_F_TEST_TM_SVM_HV_POS)) /**< TM_SVM_HV Mask */

#define MXC_F_TEST_TM_PWCN_BOD_POS                     (26) /**< TM_PWCN_BOD Position */
#define MXC_F_TEST_TM_PWCN_BOD                         ((uint32_t)(0x1 << MXC_F_TEST_TM_PWCN_BOD_POS)) /**< TM_PWCN_BOD Mask */

#define MXC_F_TEST_TM_VMAIN_MON_PD_POS                 (27) /**< TM_VMAIN_MON_PD Position */
#define MXC_F_TEST_TM_VMAIN_MON_PD                     ((uint32_t)(0x1 << MXC_F_TEST_TM_VMAIN_MON_PD_POS)) /**< TM_VMAIN_MON_PD Mask */

#define MXC_F_TEST_TM_RETLDO_EN_POS                    (28) /**< TM_RETLDO_EN Position */
#define MXC_F_TEST_TM_RETLDO_EN                        ((uint32_t)(0x1 << MXC_F_TEST_TM_RETLDO_EN_POS)) /**< TM_RETLDO_EN Mask */

#define MXC_F_TEST_TM_NMI_DIS_POS                      (28) /**< TM_NMI_DIS Position */
#define MXC_F_TEST_TM_NMI_DIS                          ((uint32_t)(0x7FFFFFFF << MXC_F_TEST_TM_NMI_DIS_POS)) /**< TM_NMI_DIS Mask */

/**@} end of group TM_Register */

/**
 * @ingroup  test_registers
 * @defgroup TRCN_Register
 * @brief    Trim Control Register.
 * @{
 */
#define MXC_F_TEST_TRCN_CLKDIV_POS                     (0) /**< TRCN_CLKDIV Position */
#define MXC_F_TEST_TRCN_CLKDIV                         ((uint32_t)(0xFF << MXC_F_TEST_TRCN_CLKDIV_POS)) /**< TRCN_CLKDIV Mask */

/**@} end of group TRCN_Register */

/**
 * @ingroup  test_registers
 * @defgroup TCLK_Register
 * @brief    Test Clock Control Register.
 * @{
 */
#define MXC_F_TEST_TCLK_VCLKSRC_POS                    (0) /**< TCLK_VCLKSRC Position */
#define MXC_F_TEST_TCLK_VCLKSRC                        ((uint32_t)(0xFF << MXC_F_TEST_TCLK_VCLKSRC_POS)) /**< TCLK_VCLKSRC Mask */

#define MXC_F_TEST_TCLK_VCLKDIV_POS                    (8) /**< TCLK_VCLKDIV Position */
#define MXC_F_TEST_TCLK_VCLKDIV                        ((uint32_t)(0xF << MXC_F_TEST_TCLK_VCLKDIV_POS)) /**< TCLK_VCLKDIV Mask */

#define MXC_F_TEST_TCLK_VCLK_MODE_POS                  (12) /**< TCLK_VCLK_MODE Position */
#define MXC_F_TEST_TCLK_VCLK_MODE                      ((uint32_t)(0x1F << MXC_F_TEST_TCLK_VCLK_MODE_POS)) /**< TCLK_VCLK_MODE Mask */

/**@} end of group TCLK_Register */

/**
 * @ingroup  test_registers
 * @defgroup TMR0_Register
 * @brief    Security Test Mode Register.
 * @{
 */
#define MXC_F_TEST_TMR0_IDS_MODE0_POS                  (19) /**< TMR0_IDS_MODE0 Position */
#define MXC_F_TEST_TMR0_IDS_MODE0                      ((uint32_t)(0x1 << MXC_F_TEST_TMR0_IDS_MODE0_POS)) /**< TMR0_IDS_MODE0 Mask */

/**@} end of group TMR0_Register */

/**
 * @ingroup  test_registers
 * @defgroup TMR1_Register
 * @brief    Clock Test Mode Register.
 * @{
 */
#define MXC_F_TEST_TMR1_XTAL_TM1_POS                   (0) /**< TMR1_XTAL_TM1 Position */
#define MXC_F_TEST_TMR1_XTAL_TM1                       ((uint32_t)(0x1 << MXC_F_TEST_TMR1_XTAL_TM1_POS)) /**< TMR1_XTAL_TM1 Mask */

#define MXC_F_TEST_TMR1_XTAL_TM2_POS                   (1) /**< TMR1_XTAL_TM2 Position */
#define MXC_F_TEST_TMR1_XTAL_TM2                       ((uint32_t)(0x1 << MXC_F_TEST_TMR1_XTAL_TM2_POS)) /**< TMR1_XTAL_TM2 Mask */

#define MXC_F_TEST_TMR1_XTAL_PD_POS                    (2) /**< TMR1_XTAL_PD Position */
#define MXC_F_TEST_TMR1_XTAL_PD                        ((uint32_t)(0x1 << MXC_F_TEST_TMR1_XTAL_PD_POS)) /**< TMR1_XTAL_PD Mask */

#define MXC_F_TEST_TMR1_XTAL_BP_POS                    (3) /**< TMR1_XTAL_BP Position */
#define MXC_F_TEST_TMR1_XTAL_BP                        ((uint32_t)(0x1 << MXC_F_TEST_TMR1_XTAL_BP_POS)) /**< TMR1_XTAL_BP Mask */

#define MXC_F_TEST_TMR1_PLL0_IB_POS                    (4) /**< TMR1_PLL0_IB Position */
#define MXC_F_TEST_TMR1_PLL0_IB                        ((uint32_t)(0x1 << MXC_F_TEST_TMR1_PLL0_IB_POS)) /**< TMR1_PLL0_IB Mask */

#define MXC_F_TEST_TMR1_PLL0_VCO_POS                   (5) /**< TMR1_PLL0_VCO Position */
#define MXC_F_TEST_TMR1_PLL0_VCO                       ((uint32_t)(0x1 << MXC_F_TEST_TMR1_PLL0_VCO_POS)) /**< TMR1_PLL0_VCO Mask */

#define MXC_F_TEST_TMR1_PLL0_SEL_POS                   (6) /**< TMR1_PLL0_SEL Position */
#define MXC_F_TEST_TMR1_PLL0_SEL                       ((uint32_t)(0x1 << MXC_F_TEST_TMR1_PLL0_SEL_POS)) /**< TMR1_PLL0_SEL Mask */

#define MXC_F_TEST_TMR1_PLL0_CK_BYP_POS                (7) /**< TMR1_PLL0_CK_BYP Position */
#define MXC_F_TEST_TMR1_PLL0_CK_BYP                    ((uint32_t)(0x1 << MXC_F_TEST_TMR1_PLL0_CK_BYP_POS)) /**< TMR1_PLL0_CK_BYP Mask */

#define MXC_F_TEST_TMR1_HIRC_PD_POS                    (11) /**< TMR1_HIRC_PD Position */
#define MXC_F_TEST_TMR1_HIRC_PD                        ((uint32_t)(0x1 << MXC_F_TEST_TMR1_HIRC_PD_POS)) /**< TMR1_HIRC_PD Mask */

#define MXC_F_TEST_TMR1_96M_PD_POS                     (12) /**< TMR1_96M_PD Position */
#define MXC_F_TEST_TMR1_96M_PD                         ((uint32_t)(0x1 << MXC_F_TEST_TMR1_96M_PD_POS)) /**< TMR1_96M_PD Mask */

#define MXC_F_TEST_TMR1_8M_PD_POS                      (13) /**< TMR1_8M_PD Position */
#define MXC_F_TEST_TMR1_8M_PD                          ((uint32_t)(0x1 << MXC_F_TEST_TMR1_8M_PD_POS)) /**< TMR1_8M_PD Mask */

#define MXC_F_TEST_TMR1_8K_PD_POS                      (14) /**< TMR1_8K_PD Position */
#define MXC_F_TEST_TMR1_8K_PD                          ((uint32_t)(0x1 << MXC_F_TEST_TMR1_8K_PD_POS)) /**< TMR1_8K_PD Mask */

#define MXC_F_TEST_TMR1_6K_PD_POS                      (15) /**< TMR1_6K_PD Position */
#define MXC_F_TEST_TMR1_6K_PD                          ((uint32_t)(0x1 << MXC_F_TEST_TMR1_6K_PD_POS)) /**< TMR1_6K_PD Mask */

#define MXC_F_TEST_TMR1_X32D_POS                       (16) /**< TMR1_X32D Position */
#define MXC_F_TEST_TMR1_X32D                           ((uint32_t)(0x1 << MXC_F_TEST_TMR1_X32D_POS)) /**< TMR1_X32D Mask */

#define MXC_F_TEST_TMR1_32KBYP_POS                     (17) /**< TMR1_32KBYP Position */
#define MXC_F_TEST_TMR1_32KBYP                         ((uint32_t)(0x1 << MXC_F_TEST_TMR1_32KBYP_POS)) /**< TMR1_32KBYP Mask */

#define MXC_F_TEST_TMR1_RESV4_POS                      (19) /**< TMR1_RESV4 Position */
#define MXC_F_TEST_TMR1_RESV4                          ((uint32_t)(0x1 << MXC_F_TEST_TMR1_RESV4_POS)) /**< TMR1_RESV4 Mask */

/**@} end of group TMR1_Register */

/**
 * @ingroup  test_registers
 * @defgroup TMR2_Register
 * @brief    USB, HYPERBUS, I2C, and ADC Test Mode Register.
 * @{
 */
#define MXC_F_TEST_TMR2_USB_TM0_POS                    (0) /**< TMR2_USB_TM0 Position */
#define MXC_F_TEST_TMR2_USB_TM0                        ((uint32_t)(0x1 << MXC_F_TEST_TMR2_USB_TM0_POS)) /**< TMR2_USB_TM0 Mask */

#define MXC_F_TEST_TMR2_USB_TM1_POS                    (1) /**< TMR2_USB_TM1 Position */
#define MXC_F_TEST_TMR2_USB_TM1                        ((uint32_t)(0x1 << MXC_F_TEST_TMR2_USB_TM1_POS)) /**< TMR2_USB_TM1 Mask */

#define MXC_F_TEST_TMR2_RESV0_POS                      (2) /**< TMR2_RESV0 Position */
#define MXC_F_TEST_TMR2_RESV0                          ((uint32_t)(0x1 << MXC_F_TEST_TMR2_RESV0_POS)) /**< TMR2_RESV0 Mask */

#define MXC_F_TEST_TMR2_USB_TMEN_POS                   (3) /**< TMR2_USB_TMEN Position */
#define MXC_F_TEST_TMR2_USB_TMEN                       ((uint32_t)(0x1 << MXC_F_TEST_TMR2_USB_TMEN_POS)) /**< TMR2_USB_TMEN Mask */

#define MXC_F_TEST_TMR2_USB_XTEST_POS                  (4) /**< TMR2_USB_XTEST Position */
#define MXC_F_TEST_TMR2_USB_XTEST                      ((uint32_t)(0x1 << MXC_F_TEST_TMR2_USB_XTEST_POS)) /**< TMR2_USB_XTEST Mask */

#define MXC_F_TEST_TMR2_USB_VBUSDET_POS                (5) /**< TMR2_USB_VBUSDET Position */
#define MXC_F_TEST_TMR2_USB_VBUSDET                    ((uint32_t)(0x1 << MXC_F_TEST_TMR2_USB_VBUSDET_POS)) /**< TMR2_USB_VBUSDET Mask */

#define MXC_F_TEST_TMR2_IRRX_SDA0_POS                  (8) /**< TMR2_IRRX_SDA0 Position */
#define MXC_F_TEST_TMR2_IRRX_SDA0                      ((uint32_t)(0x1 << MXC_F_TEST_TMR2_IRRX_SDA0_POS)) /**< TMR2_IRRX_SDA0 Mask */

#define MXC_F_TEST_TMR2_IRRX_SCL0_POS                  (9) /**< TMR2_IRRX_SCL0 Position */
#define MXC_F_TEST_TMR2_IRRX_SCL0                      ((uint32_t)(0x1 << MXC_F_TEST_TMR2_IRRX_SCL0_POS)) /**< TMR2_IRRX_SCL0 Mask */

#define MXC_F_TEST_TMR2_IRRX_SDA1_POS                  (10) /**< TMR2_IRRX_SDA1 Position */
#define MXC_F_TEST_TMR2_IRRX_SDA1                      ((uint32_t)(0x1 << MXC_F_TEST_TMR2_IRRX_SDA1_POS)) /**< TMR2_IRRX_SDA1 Mask */

#define MXC_F_TEST_TMR2_IRRX_SCL1_POS                  (11) /**< TMR2_IRRX_SCL1 Position */
#define MXC_F_TEST_TMR2_IRRX_SCL1                      ((uint32_t)(0x1 << MXC_F_TEST_TMR2_IRRX_SCL1_POS)) /**< TMR2_IRRX_SCL1 Mask */

#define MXC_F_TEST_TMR2_SDA0_DGEN_POS                  (12) /**< TMR2_SDA0_DGEN Position */
#define MXC_F_TEST_TMR2_SDA0_DGEN                      ((uint32_t)(0x1 << MXC_F_TEST_TMR2_SDA0_DGEN_POS)) /**< TMR2_SDA0_DGEN Mask */

#define MXC_F_TEST_TMR2_SCL0_DGEN_POS                  (13) /**< TMR2_SCL0_DGEN Position */
#define MXC_F_TEST_TMR2_SCL0_DGEN                      ((uint32_t)(0x1 << MXC_F_TEST_TMR2_SCL0_DGEN_POS)) /**< TMR2_SCL0_DGEN Mask */

#define MXC_F_TEST_TMR2_SDA1_DGEN_POS                  (14) /**< TMR2_SDA1_DGEN Position */
#define MXC_F_TEST_TMR2_SDA1_DGEN                      ((uint32_t)(0x1 << MXC_F_TEST_TMR2_SDA1_DGEN_POS)) /**< TMR2_SDA1_DGEN Mask */

#define MXC_F_TEST_TMR2_SCL1_DGEN_POS                  (15) /**< TMR2_SCL1_DGEN Position */
#define MXC_F_TEST_TMR2_SCL1_DGEN                      ((uint32_t)(0x1 << MXC_F_TEST_TMR2_SCL1_DGEN_POS)) /**< TMR2_SCL1_DGEN Mask */

#define MXC_F_TEST_TMR2_SDA0_DGBYP_POS                 (16) /**< TMR2_SDA0_DGBYP Position */
#define MXC_F_TEST_TMR2_SDA0_DGBYP                     ((uint32_t)(0x1 << MXC_F_TEST_TMR2_SDA0_DGBYP_POS)) /**< TMR2_SDA0_DGBYP Mask */

#define MXC_F_TEST_TMR2_SCL0_DGBYP_POS                 (17) /**< TMR2_SCL0_DGBYP Position */
#define MXC_F_TEST_TMR2_SCL0_DGBYP                     ((uint32_t)(0x1 << MXC_F_TEST_TMR2_SCL0_DGBYP_POS)) /**< TMR2_SCL0_DGBYP Mask */

#define MXC_F_TEST_TMR2_SDA1_DGBYP_POS                 (18) /**< TMR2_SDA1_DGBYP Position */
#define MXC_F_TEST_TMR2_SDA1_DGBYP                     ((uint32_t)(0x1 << MXC_F_TEST_TMR2_SDA1_DGBYP_POS)) /**< TMR2_SDA1_DGBYP Mask */

#define MXC_F_TEST_TMR2_SCL1_DGBYP_POS                 (19) /**< TMR2_SCL1_DGBYP Position */
#define MXC_F_TEST_TMR2_SCL1_DGBYP                     ((uint32_t)(0x1 << MXC_F_TEST_TMR2_SCL1_DGBYP_POS)) /**< TMR2_SCL1_DGBYP Mask */

#define MXC_F_TEST_TMR2_ADCOUT_POS                     (20) /**< TMR2_ADCOUT Position */
#define MXC_F_TEST_TMR2_ADCOUT                         ((uint32_t)(0x1 << MXC_F_TEST_TMR2_ADCOUT_POS)) /**< TMR2_ADCOUT Mask */

#define MXC_F_TEST_TMR2_RDS_BYP_POS                    (24) /**< TMR2_RDS_BYP Position */
#define MXC_F_TEST_TMR2_RDS_BYP                        ((uint32_t)(0x1 << MXC_F_TEST_TMR2_RDS_BYP_POS)) /**< TMR2_RDS_BYP Mask */

#define MXC_F_TEST_TMR2_HYP_CKEN_POS                   (25) /**< TMR2_HYP_CKEN Position */
#define MXC_F_TEST_TMR2_HYP_CKEN                       ((uint32_t)(0x1 << MXC_F_TEST_TMR2_HYP_CKEN_POS)) /**< TMR2_HYP_CKEN Mask */

/**@} end of group TMR2_Register */

/**
 * @ingroup  test_registers
 * @defgroup TMR3_Register
 * @brief    Memory Test Mode Register.
 * @{
 */
#define MXC_F_TEST_TMR3_ROM_TEST1_POS                  (0) /**< TMR3_ROM_TEST1 Position */
#define MXC_F_TEST_TMR3_ROM_TEST1                      ((uint32_t)(0x1 << MXC_F_TEST_TMR3_ROM_TEST1_POS)) /**< TMR3_ROM_TEST1 Mask */

#define MXC_F_TEST_TMR3_SRAM_TEST1_POS                 (1) /**< TMR3_SRAM_TEST1 Position */
#define MXC_F_TEST_TMR3_SRAM_TEST1                     ((uint32_t)(0x1 << MXC_F_TEST_TMR3_SRAM_TEST1_POS)) /**< TMR3_SRAM_TEST1 Mask */

/**@} end of group TMR3_Register */

/**
 * @ingroup  test_registers
 * @defgroup MBCN_Register
 * @brief    Memory BIST Control Register.
 * @{
 */
#define MXC_F_TEST_MBCN_SYSRAM1_POS                    (0) /**< MBCN_SYSRAM1 Position */
#define MXC_F_TEST_MBCN_SYSRAM1                        ((uint32_t)(0x1 << MXC_F_TEST_MBCN_SYSRAM1_POS)) /**< MBCN_SYSRAM1 Mask */

#define MXC_F_TEST_MBCN_SYSRAM2_POS                    (1) /**< MBCN_SYSRAM2 Position */
#define MXC_F_TEST_MBCN_SYSRAM2                        ((uint32_t)(0x1 << MXC_F_TEST_MBCN_SYSRAM2_POS)) /**< MBCN_SYSRAM2 Mask */

#define MXC_F_TEST_MBCN_ICACHE_POS                     (3) /**< MBCN_ICACHE Position */
#define MXC_F_TEST_MBCN_ICACHE                         ((uint32_t)(0x1 << MXC_F_TEST_MBCN_ICACHE_POS)) /**< MBCN_ICACHE Mask */

#define MXC_F_TEST_MBCN_CRYPTO_POS                     (5) /**< MBCN_CRYPTO Position */
#define MXC_F_TEST_MBCN_CRYPTO                         ((uint32_t)(0x1 << MXC_F_TEST_MBCN_CRYPTO_POS)) /**< MBCN_CRYPTO Mask */

#define MXC_F_TEST_MBCN_ICACHEXIP_POS                  (11) /**< MBCN_ICACHEXIP Position */
#define MXC_F_TEST_MBCN_ICACHEXIP                      ((uint32_t)(0x1 << MXC_F_TEST_MBCN_ICACHEXIP_POS)) /**< MBCN_ICACHEXIP Mask */

#define MXC_F_TEST_MBCN_SYSRAM3_POS                    (16) /**< MBCN_SYSRAM3 Position */
#define MXC_F_TEST_MBCN_SYSRAM3                        ((uint32_t)(0x1 << MXC_F_TEST_MBCN_SYSRAM3_POS)) /**< MBCN_SYSRAM3 Mask */

#define MXC_F_TEST_MBCN_SYSRAM4_POS                    (17) /**< MBCN_SYSRAM4 Position */
#define MXC_F_TEST_MBCN_SYSRAM4                        ((uint32_t)(0x1 << MXC_F_TEST_MBCN_SYSRAM4_POS)) /**< MBCN_SYSRAM4 Mask */

/**@} end of group MBCN_Register */

/**
 * @ingroup  test_registers
 * @defgroup MBSTAT_Register
 * @brief    Memory BIST Status Register.
 * @{
 */
#define MXC_F_TEST_MBSTAT_SYSRAM1_POS                  (0) /**< MBSTAT_SYSRAM1 Position */
#define MXC_F_TEST_MBSTAT_SYSRAM1                      ((uint32_t)(0x1 << MXC_F_TEST_MBSTAT_SYSRAM1_POS)) /**< MBSTAT_SYSRAM1 Mask */

#define MXC_F_TEST_MBSTAT_SYSRAM2_POS                  (1) /**< MBSTAT_SYSRAM2 Position */
#define MXC_F_TEST_MBSTAT_SYSRAM2                      ((uint32_t)(0x1 << MXC_F_TEST_MBSTAT_SYSRAM2_POS)) /**< MBSTAT_SYSRAM2 Mask */

#define MXC_F_TEST_MBSTAT_ICACHE_POS                   (3) /**< MBSTAT_ICACHE Position */
#define MXC_F_TEST_MBSTAT_ICACHE                       ((uint32_t)(0x1 << MXC_F_TEST_MBSTAT_ICACHE_POS)) /**< MBSTAT_ICACHE Mask */

#define MXC_F_TEST_MBSTAT_CRYPTO_POS                   (5) /**< MBSTAT_CRYPTO Position */
#define MXC_F_TEST_MBSTAT_CRYPTO                       ((uint32_t)(0x1 << MXC_F_TEST_MBSTAT_CRYPTO_POS)) /**< MBSTAT_CRYPTO Mask */

#define MXC_F_TEST_MBSTAT_ICACHEXIP_POS                (11) /**< MBSTAT_ICACHEXIP Position */
#define MXC_F_TEST_MBSTAT_ICACHEXIP                    ((uint32_t)(0x1 << MXC_F_TEST_MBSTAT_ICACHEXIP_POS)) /**< MBSTAT_ICACHEXIP Mask */

#define MXC_F_TEST_MBSTAT_SYSRAM3_POS                  (16) /**< MBSTAT_SYSRAM3 Position */
#define MXC_F_TEST_MBSTAT_SYSRAM3                      ((uint32_t)(0x1 << MXC_F_TEST_MBSTAT_SYSRAM3_POS)) /**< MBSTAT_SYSRAM3 Mask */

#define MXC_F_TEST_MBSTAT_SYSRAM4_POS                  (17) /**< MBSTAT_SYSRAM4 Position */
#define MXC_F_TEST_MBSTAT_SYSRAM4                      ((uint32_t)(0x1 << MXC_F_TEST_MBSTAT_SYSRAM4_POS)) /**< MBSTAT_SYSRAM4 Mask */

/**@} end of group MBSTAT_Register */

/*******************************************************************************/
/*                                                                        Test */
#define MXC_BASE_TEST                    ((uint32_t)0x40000C00UL)
#define MXC_TEST                         ((mxc_test_regs_t*)MXC_BASE_TEST)
#define MXC_TEST_INSTANCES               (1)

#ifdef __cplusplus
}
#endif

#endif /* _TEST_REGS_H_ */

