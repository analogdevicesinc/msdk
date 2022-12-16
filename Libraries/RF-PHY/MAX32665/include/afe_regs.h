/* *****************************************************************************
 * Copyright (C) Analog Devices, All rights Reserved.
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
 **************************************************************************** */

/**
 * @file    afe_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the AFE Peripheral Module.
 */

#ifndef MAX32665_INCLUDE_AFE_REGS_H_
#define MAX32665_INCLUDE_AFE_REGS_H_

/**************************************************************************************************
  Includes
**************************************************************************************************/

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************************************
  Macros
**************************************************************************************************/

#if defined (__ICCARM__)
#pragma system_include
#endif

#if defined (__CC_ARM)
#pragma anon_unions
#endif
/*/ @cond */
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
/*/ @endcond */

/* **** Definitions **** */

/**
 * @ingroup     afe
 * @defgroup    afe_registers AFE_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the AFE Peripheral Module.
 * @details PAN2G5 AFE registers
 */

/**
 * @ingroup afe_registers
 * Structure type to access the AFE Registers.
 */
typedef struct {
    __R  uint8_t rsv_0x0;
    __IO uint8_t  reg1;                           /**< <tt>\b 0x0001:</tt> AFE REG1 Register */
    __IO uint8_t  reg2;                           /**< <tt>\b 0x0002:</tt> AFE REG2 Register */
    __IO uint8_t  reg3;                           /**< <tt>\b 0x0003:</tt> AFE REG3 Register */
    __IO uint8_t  reg4;                           /**< <tt>\b 0x0004:</tt> AFE REG4 Register */
    __IO uint8_t  reg5;                           /**< <tt>\b 0x0005:</tt> AFE REG5 Register */
    __IO uint8_t  reg6;                           /**< <tt>\b 0x0006:</tt> AFE REG6 Register */
    __IO uint8_t  reg7;                           /**< <tt>\b 0x0007:</tt> AFE REG7 Register */
    __IO uint8_t  reg8;                           /**< <tt>\b 0x0008:</tt> AFE REG8 Register */
    __IO uint8_t  reg9;                           /**< <tt>\b 0x0009:</tt> AFE REG9 Register */
    __IO uint8_t  reg10;                          /**< <tt>\b 0x000a:</tt> AFE REG10 Register */
    __IO uint8_t  reg11;                          /**< <tt>\b 0x000b:</tt> AFE REG11 Register */
    __IO uint8_t  reg12;                          /**< <tt>\b 0x000c:</tt> AFE REG12 Register */
    __IO uint8_t  reg13;                          /**< <tt>\b 0x000d:</tt> AFE REG13 Register */
    __IO uint8_t  reg14;                          /**< <tt>\b 0x000e:</tt> AFE REG14 Register */
    __IO uint8_t  reg15;                          /**< <tt>\b 0x000f:</tt> AFE REG15 Register */
    __R  uint32_t rsv_0x10_0x1f[4];
    __IO uint8_t  rega1;                          /**< <tt>\b 0x0020:</tt> AFE REGA1 Register */
    __IO uint8_t  rega2;                          /**< <tt>\b 0x0021:</tt> AFE REGA2 Register */
    __IO uint8_t  rega3;                          /**< <tt>\b 0x0022:</tt> AFE REGA3 Register */
    __IO uint8_t  rega4;                          /**< <tt>\b 0x0023:</tt> AFE REGA4 Register */
    __IO uint8_t  rega5;                          /**< <tt>\b 0x0024:</tt> AFE REGA5 Register */
    __IO uint8_t  rega6;                          /**< <tt>\b 0x0025:</tt> AFE REGA6 Register */
    __IO uint8_t  rega7;                          /**< <tt>\b 0x0026:</tt> AFE REGA7 Register */
    __IO uint8_t  rega8;                          /**< <tt>\b 0x0027:</tt> AFE REGA8 Register */
    __IO int8_t   rega9;                          /**< <tt>\b 0x0028:</tt> AFE REGA9 Register */
    __IO int8_t   rega10;                         /**< <tt>\b 0x0029:</tt> AFE REGA10 Register */
    __IO uint8_t  rega11;                         /**< <tt>\b 0x002a:</tt> AFE REGA11 Register */
    __IO uint8_t  rega12;                         /**< <tt>\b 0x002b:</tt> AFE REGA12 Register */
    __IO uint8_t  rega13;                         /**< <tt>\b 0x002c:</tt> AFE REGA13 Register */
    __IO uint8_t  rega14;                         /**< <tt>\b 0x002d:</tt> AFE REGA14 Register */
    __IO uint8_t  rega15;                         /**< <tt>\b 0x002e:</tt> AFE REGA15 Register */
    __IO uint8_t  rega16;                         /**< <tt>\b 0x002f:</tt> AFE REGA16 Register */
    __IO uint8_t  regb1;                          /**< <tt>\b 0x0030:</tt> AFE REGB1 Register */
    __IO uint8_t  regb2;                          /**< <tt>\b 0x0031:</tt> AFE REGB2 Register */
    __IO uint8_t  regb3;                          /**< <tt>\b 0x0032:</tt> AFE REGB3 Register */
    __IO uint8_t  regb4;                          /**< <tt>\b 0x0033:</tt> AFE REGB4 Register */
    __IO uint8_t  regb5;                          /**< <tt>\b 0x0034:</tt> AFE REGB5 Register */
    __IO uint8_t  regb6;                          /**< <tt>\b 0x0035:</tt> AFE REGB6 Register */
    __IO uint8_t  regb7;                          /**< <tt>\b 0x0036:</tt> AFE REGB7 Register */
} mxc_afe_regs_t;

/* Register offsets for module AFE */
/**
 * @ingroup    afe_registers
 * @defgroup   AFE_Register_Offsets Register Offsets
 * @brief      AFE Peripheral Register Offsets from the AFE Base Peripheral Address.
 * @{
 */
#define MXC_R_AFE_REG1                               ((uint32_t)0x00000001UL) /**< Offset from AFE Base Address: <tt> 0x0001</tt> */
#define MXC_R_AFE_REG2                               ((uint32_t)0x00000002UL) /**< Offset from AFE Base Address: <tt> 0x0002</tt> */
#define MXC_R_AFE_REG3                               ((uint32_t)0x00000003UL) /**< Offset from AFE Base Address: <tt> 0x0003</tt> */
#define MXC_R_AFE_REG4                               ((uint32_t)0x00000004UL) /**< Offset from AFE Base Address: <tt> 0x0004</tt> */
#define MXC_R_AFE_REG5                               ((uint32_t)0x00000005UL) /**< Offset from AFE Base Address: <tt> 0x0005</tt> */
#define MXC_R_AFE_REG6                               ((uint32_t)0x00000006UL) /**< Offset from AFE Base Address: <tt> 0x0006</tt> */
#define MXC_R_AFE_REG7                               ((uint32_t)0x00000007UL) /**< Offset from AFE Base Address: <tt> 0x0007</tt> */
#define MXC_R_AFE_REG8                               ((uint32_t)0x00000008UL) /**< Offset from AFE Base Address: <tt> 0x0008</tt> */
#define MXC_R_AFE_REG9                               ((uint32_t)0x00000009UL) /**< Offset from AFE Base Address: <tt> 0x0009</tt> */
#define MXC_R_AFE_REG10                              ((uint32_t)0x0000000AUL) /**< Offset from AFE Base Address: <tt> 0x000A</tt> */
#define MXC_R_AFE_REG11                              ((uint32_t)0x0000000BUL) /**< Offset from AFE Base Address: <tt> 0x000B</tt> */
#define MXC_R_AFE_REG12                              ((uint32_t)0x0000000CUL) /**< Offset from AFE Base Address: <tt> 0x000C</tt> */
#define MXC_R_AFE_REG13                              ((uint32_t)0x0000000DUL) /**< Offset from AFE Base Address: <tt> 0x000D</tt> */
#define MXC_R_AFE_REG14                              ((uint32_t)0x0000000EUL) /**< Offset from AFE Base Address: <tt> 0x000E</tt> */
#define MXC_R_AFE_REG15                              ((uint32_t)0x0000000FUL) /**< Offset from AFE Base Address: <tt> 0x000F</tt> */
#define MXC_R_AFE_REGA1                              ((uint32_t)0x00000020UL) /**< Offset from AFE Base Address: <tt> 0x0020</tt> */
#define MXC_R_AFE_REGA2                              ((uint32_t)0x00000021UL) /**< Offset from AFE Base Address: <tt> 0x0021</tt> */
#define MXC_R_AFE_REGA3                              ((uint32_t)0x00000022UL) /**< Offset from AFE Base Address: <tt> 0x0022</tt> */
#define MXC_R_AFE_REGA4                              ((uint32_t)0x00000023UL) /**< Offset from AFE Base Address: <tt> 0x0023</tt> */
#define MXC_R_AFE_REGA5                              ((uint32_t)0x00000024UL) /**< Offset from AFE Base Address: <tt> 0x0024</tt> */
#define MXC_R_AFE_REGA6                              ((uint32_t)0x00000025UL) /**< Offset from AFE Base Address: <tt> 0x0025</tt> */
#define MXC_R_AFE_REGA7                              ((uint32_t)0x00000026UL) /**< Offset from AFE Base Address: <tt> 0x0026</tt> */
#define MXC_R_AFE_REGA8                              ((uint32_t)0x00000027UL) /**< Offset from AFE Base Address: <tt> 0x0027</tt> */
#define MXC_R_AFE_REGA9                              ((uint32_t)0x00000028UL) /**< Offset from AFE Base Address: <tt> 0x0028</tt> */
#define MXC_R_AFE_REGA10                             ((uint32_t)0x00000029UL) /**< Offset from AFE Base Address: <tt> 0x0029</tt> */
#define MXC_R_AFE_REGA11                             ((uint32_t)0x0000002AUL) /**< Offset from AFE Base Address: <tt> 0x002A</tt> */
#define MXC_R_AFE_REGA12                             ((uint32_t)0x0000002BUL) /**< Offset from AFE Base Address: <tt> 0x002B</tt> */
#define MXC_R_AFE_REGA13                             ((uint32_t)0x0000002CUL) /**< Offset from AFE Base Address: <tt> 0x002C</tt> */
#define MXC_R_AFE_REGA14                             ((uint32_t)0x0000002DUL) /**< Offset from AFE Base Address: <tt> 0x002D</tt> */
#define MXC_R_AFE_REGA15                             ((uint32_t)0x0000002EUL) /**< Offset from AFE Base Address: <tt> 0x002E</tt> */
#define MXC_R_AFE_REGA16                             ((uint32_t)0x0000002FUL) /**< Offset from AFE Base Address: <tt> 0x002F</tt> */
#define MXC_R_AFE_REGB1                              ((uint32_t)0x00000030UL) /**< Offset from AFE Base Address: <tt> 0x0030</tt> */
#define MXC_R_AFE_REGB2                              ((uint32_t)0x00000031UL) /**< Offset from AFE Base Address: <tt> 0x0031</tt> */
#define MXC_R_AFE_REGB3                              ((uint32_t)0x00000032UL) /**< Offset from AFE Base Address: <tt> 0x0032</tt> */
#define MXC_R_AFE_REGB4                              ((uint32_t)0x00000033UL) /**< Offset from AFE Base Address: <tt> 0x0033</tt> */
#define MXC_R_AFE_REGB5                              ((uint32_t)0x00000034UL) /**< Offset from AFE Base Address: <tt> 0x0034</tt> */
#define MXC_R_AFE_REGB6                              ((uint32_t)0x00000035UL) /**< Offset from AFE Base Address: <tt> 0x0035</tt> */
#define MXC_R_AFE_REGB7                              ((uint32_t)0x00000036UL) /**< Offset from AFE Base Address: <tt> 0x0036</tt> */
/**@} end of group afe_registers */

/**
 * @ingroup  afe_registers
 * @defgroup AFE_REG1 AFE_REG1
 * @brief    reg1
 * @{
 */
#define MXC_F_AFE_REG1_FM_DAC_EN_POS                   0 /**< REG1_FM_DAC_EN Position */
#define MXC_F_AFE_REG1_FM_DAC_EN                       ((uint8_t)(0x1UL << MXC_F_AFE_REG1_FM_DAC_EN_POS)) /**< REG1_FM_DAC_EN Mask */

#define MXC_F_AFE_REG1_CP_EN_POS                       1 /**< REG1_CP_EN Position */
#define MXC_F_AFE_REG1_CP_EN                           ((uint8_t)(0x1UL << MXC_F_AFE_REG1_CP_EN_POS)) /**< REG1_CP_EN Mask */

#define MXC_F_AFE_REG1_VCO_EN_POS                      2 /**< REG1_VCO_EN Position */
#define MXC_F_AFE_REG1_VCO_EN                          ((uint8_t)(0x1UL << MXC_F_AFE_REG1_VCO_EN_POS)) /**< REG1_VCO_EN Mask */

#define MXC_F_AFE_REG1_LO_BUF_EN_POS                   3 /**< REG1_LO_BUF_EN Position */
#define MXC_F_AFE_REG1_LO_BUF_EN                       ((uint8_t)(0x1UL << MXC_F_AFE_REG1_LO_BUF_EN_POS)) /**< REG1_LO_BUF_EN Mask */

#define MXC_F_AFE_REG1_DIV_EN_POS                      4 /**< REG1_DIV_EN Position */
#define MXC_F_AFE_REG1_DIV_EN                          ((uint8_t)(0x1UL << MXC_F_AFE_REG1_DIV_EN_POS)) /**< REG1_DIV_EN Mask */

#define MXC_F_AFE_REG1_PA_EN_POS                       5 /**< REG1_PA_EN Position */
#define MXC_F_AFE_REG1_PA_EN                           ((uint8_t)(0x1UL << MXC_F_AFE_REG1_PA_EN_POS)) /**< REG1_PA_EN Mask */

#define MXC_F_AFE_REG1_TX_SW_EN_POS                    6 /**< REG1_TX_SW_EN Position */
#define MXC_F_AFE_REG1_TX_SW_EN                        ((uint8_t)(0x1UL << MXC_F_AFE_REG1_TX_SW_EN_POS)) /**< REG1_TX_SW_EN Mask */

#define MXC_F_AFE_REG1_PLL_BIAS_EN_POS                 7 /**< REG1_PLL_BIAS_EN Position */
#define MXC_F_AFE_REG1_PLL_BIAS_EN                     ((uint8_t)(0x1UL << MXC_F_AFE_REG1_PLL_BIAS_EN_POS)) /**< REG1_PLL_BIAS_EN Mask */

/**@} end of group AFE_REG1_Register */

/**
 * @ingroup  afe_registers
 * @defgroup AFE_REG2 AFE_REG2
 * @brief    reg2
 * @{
 */
#define MXC_F_AFE_REG2_CLK_DAC_EN_POS                  6 /**< REG2_CLK_DAC_EN Position */
#define MXC_F_AFE_REG2_CLK_DAC_EN                      ((uint8_t)(0x1UL << MXC_F_AFE_REG2_CLK_DAC_EN_POS)) /**< REG2_CLK_DAC_EN Mask */

#define MXC_F_AFE_REG2_CLK_PA_EN_POS                   7 /**< REG2_CLK_PA_EN Position */
#define MXC_F_AFE_REG2_CLK_PA_EN                       ((uint8_t)(0x1UL << MXC_F_AFE_REG2_CLK_PA_EN_POS)) /**< REG2_CLK_PA_EN Mask */

/**@} end of group AFE_REG2_Register */

/**
 * @ingroup  afe_registers
 * @defgroup AFE_REG3 AFE_REG3
 * @brief    reg3
 * @{
 */
#define MXC_F_AFE_REG3_TM_IBOUT_POS                    0 /**< REG3_TM_IBOUT Position */
#define MXC_F_AFE_REG3_TM_IBOUT                        ((uint8_t)(0x7UL << MXC_F_AFE_REG3_TM_IBOUT_POS)) /**< REG3_TM_IBOUT Mask */
#define MXC_V_AFE_REG3_TM_IBOUT_GND_VCO                ((uint8_t)0x0UL) /**< REG3_TM_IBOUT_GND_VCO Value */
#define MXC_S_AFE_REG3_TM_IBOUT_GND_VCO                (MXC_V_AFE_REG3_TM_IBOUT_GND_VCO << MXC_F_AFE_REG3_TM_IBOUT_POS) /**< REG3_TM_IBOUT_GND_VCO Setting */
#define MXC_V_AFE_REG3_TM_IBOUT_IB_VCO                 ((uint8_t)0x1UL) /**< REG3_TM_IBOUT_IB_VCO Value */
#define MXC_S_AFE_REG3_TM_IBOUT_IB_VCO                 (MXC_V_AFE_REG3_TM_IBOUT_IB_VCO << MXC_F_AFE_REG3_TM_IBOUT_POS) /**< REG3_TM_IBOUT_IB_VCO Setting */
#define MXC_V_AFE_REG3_TM_IBOUT_IB_CP                  ((uint8_t)0x2UL) /**< REG3_TM_IBOUT_IB_CP Value */
#define MXC_S_AFE_REG3_TM_IBOUT_IB_CP                  (MXC_V_AFE_REG3_TM_IBOUT_IB_CP << MXC_F_AFE_REG3_TM_IBOUT_POS) /**< REG3_TM_IBOUT_IB_CP Setting */
#define MXC_V_AFE_REG3_TM_IBOUT_IB_FM_MOD              ((uint8_t)0x3UL) /**< REG3_TM_IBOUT_IB_FM_MOD Value */
#define MXC_S_AFE_REG3_TM_IBOUT_IB_FM_MOD              (MXC_V_AFE_REG3_TM_IBOUT_IB_FM_MOD << MXC_F_AFE_REG3_TM_IBOUT_POS) /**< REG3_TM_IBOUT_IB_FM_MOD Setting */
#define MXC_V_AFE_REG3_TM_IBOUT_IB_LOBUF               ((uint8_t)0x4UL) /**< REG3_TM_IBOUT_IB_LOBUF Value */
#define MXC_S_AFE_REG3_TM_IBOUT_IB_LOBUF               (MXC_V_AFE_REG3_TM_IBOUT_IB_LOBUF << MXC_F_AFE_REG3_TM_IBOUT_POS) /**< REG3_TM_IBOUT_IB_LOBUF Setting */
#define MXC_V_AFE_REG3_TM_IBOUT_IB_DIV                 ((uint8_t)0x5UL) /**< REG3_TM_IBOUT_IB_DIV Value */
#define MXC_S_AFE_REG3_TM_IBOUT_IB_DIV                 (MXC_V_AFE_REG3_TM_IBOUT_IB_DIV << MXC_F_AFE_REG3_TM_IBOUT_POS) /**< REG3_TM_IBOUT_IB_DIV Setting */
#define MXC_V_AFE_REG3_TM_IBOUT_IB_REF                 ((uint8_t)0x6UL) /**< REG3_TM_IBOUT_IB_REF Value */
#define MXC_S_AFE_REG3_TM_IBOUT_IB_REF                 (MXC_V_AFE_REG3_TM_IBOUT_IB_REF << MXC_F_AFE_REG3_TM_IBOUT_POS) /**< REG3_TM_IBOUT_IB_REF Setting */
#define MXC_V_AFE_REG3_TM_IBOUT_IB_MON                 ((uint8_t)0x7UL) /**< REG3_TM_IBOUT_IB_MON Value */
#define MXC_S_AFE_REG3_TM_IBOUT_IB_MON                 (MXC_V_AFE_REG3_TM_IBOUT_IB_MON << MXC_F_AFE_REG3_TM_IBOUT_POS) /**< REG3_TM_IBOUT_IB_MON Setting */

#define MXC_F_AFE_REG3_TM_VTUNE_OUT_POS                3 /**< REG3_TM_VTUNE_OUT Position */
#define MXC_F_AFE_REG3_TM_VTUNE_OUT                    ((uint8_t)(0x1UL << MXC_F_AFE_REG3_TM_VTUNE_OUT_POS)) /**< REG3_TM_VTUNE_OUT Mask */

#define MXC_F_AFE_REG3_ADC_TEST_EN_POS                 4 /**< REG3_ADC_TEST_EN Position */
#define MXC_F_AFE_REG3_ADC_TEST_EN                     ((uint8_t)(0x1UL << MXC_F_AFE_REG3_ADC_TEST_EN_POS)) /**< REG3_ADC_TEST_EN Mask */

#define MXC_F_AFE_REG3_CLK_DATA_SEL_POS                5 /**< REG3_CLK_DATA_SEL Position */
#define MXC_F_AFE_REG3_CLK_DATA_SEL                    ((uint8_t)(0x1UL << MXC_F_AFE_REG3_CLK_DATA_SEL_POS)) /**< REG3_CLK_DATA_SEL Mask */
#define MXC_V_AFE_REG3_CLK_DATA_SEL_CLK_DATA           ((uint8_t)0x0UL) /**< REG3_CLK_DATA_SEL_CLK_DATA Value */
#define MXC_S_AFE_REG3_CLK_DATA_SEL_CLK_DATA           (MXC_V_AFE_REG3_CLK_DATA_SEL_CLK_DATA << MXC_F_AFE_REG3_CLK_DATA_SEL_POS) /**< REG3_CLK_DATA_SEL_CLK_DATA Setting */
#define MXC_V_AFE_REG3_CLK_DATA_SEL_XO_OUT             ((uint8_t)0x1UL) /**< REG3_CLK_DATA_SEL_XO_OUT Value */
#define MXC_S_AFE_REG3_CLK_DATA_SEL_XO_OUT             (MXC_V_AFE_REG3_CLK_DATA_SEL_XO_OUT << MXC_F_AFE_REG3_CLK_DATA_SEL_POS) /**< REG3_CLK_DATA_SEL_XO_OUT Setting */

#define MXC_F_AFE_REG3_LF_UGB_EN_POS                   6 /**< REG3_LF_UGB_EN Position */
#define MXC_F_AFE_REG3_LF_UGB_EN                       ((uint8_t)(0x1UL << MXC_F_AFE_REG3_LF_UGB_EN_POS)) /**< REG3_LF_UGB_EN Mask */

/**@} end of group AFE_REG3_Register */

/**
 * @ingroup  afe_registers
 * @defgroup AFE_REG4 AFE_REG4
 * @brief    reg4
 * @{
 */
#define MXC_F_AFE_REG4_CP_OFS_POS                      3 /**< REG4_CP_OFS Position */
#define MXC_F_AFE_REG4_CP_OFS                          ((uint8_t)(0x3UL << MXC_F_AFE_REG4_CP_OFS_POS)) /**< REG4_CP_OFS Mask */

/**@} end of group AFE_REG4_Register */

/**
 * @ingroup  afe_registers
 * @defgroup AFE_REG5 AFE_REG5
 * @brief    reg5
 * @{
 */
#define MXC_F_AFE_REG5_TM_FMDAC_OUT_POS                6 /**< REG5_TM_FMDAC_OUT Position */
#define MXC_F_AFE_REG5_TM_FMDAC_OUT                    ((uint8_t)(0x3UL << MXC_F_AFE_REG5_TM_FMDAC_OUT_POS)) /**< REG5_TM_FMDAC_OUT Mask */

/**@} end of group AFE_REG5_Register */

/**
 * @ingroup  afe_registers
 * @defgroup AFE_REG7 AFE_REG7
 * @brief    reg7
 * @{
 */
#define MXC_F_AFE_REG7_PVTD_TRM_POS                    0 /**< REG7_PVTD_TRM Position */
#define MXC_F_AFE_REG7_PVTD_TRM                        ((uint8_t)(0x1FUL << MXC_F_AFE_REG7_PVTD_TRM_POS)) /**< REG7_PVTD_TRM Mask */

#define MXC_F_AFE_REG7_PVTD_EN_POS                     5 /**< REG7_PVTD_EN Position */
#define MXC_F_AFE_REG7_PVTD_EN                         ((uint8_t)(0x1UL << MXC_F_AFE_REG7_PVTD_EN_POS)) /**< REG7_PVTD_EN Mask */

#define MXC_F_AFE_REG7_SW_ADC_MUX_POS                  0 /**< REG7_SW_ADC_MUX Position */
#define MXC_F_AFE_REG7_SW_ADC_MUX                      ((uint8_t)(0x1FUL << MXC_F_AFE_REG7_SW_ADC_MUX_POS)) /**< REG7_SW_ADC_MUX Mask */
#define MXC_V_AFE_REG7_SW_ADC_MUX_SW_ADC               ((uint8_t)0x0UL) /**< REG7_SW_ADC_MUX_SW_ADC Value */
#define MXC_S_AFE_REG7_SW_ADC_MUX_SW_ADC               (MXC_V_AFE_REG7_SW_ADC_MUX_SW_ADC << MXC_F_AFE_REG7_SW_ADC_MUX_POS) /**< REG7_SW_ADC_MUX_SW_ADC Setting */
#define MXC_V_AFE_REG7_SW_ADC_MUX_ADC_FLAG_I           ((uint8_t)0x1UL) /**< REG7_SW_ADC_MUX_ADC_FLAG_I Value */
#define MXC_S_AFE_REG7_SW_ADC_MUX_ADC_FLAG_I           (MXC_V_AFE_REG7_SW_ADC_MUX_ADC_FLAG_I << MXC_F_AFE_REG7_SW_ADC_MUX_POS) /**< REG7_SW_ADC_MUX_ADC_FLAG_I Setting */
#define MXC_V_AFE_REG7_SW_ADC_MUX_ADC_FLAG_Q           ((uint8_t)0x2UL) /**< REG7_SW_ADC_MUX_ADC_FLAG_Q Value */
#define MXC_S_AFE_REG7_SW_ADC_MUX_ADC_FLAG_Q           (MXC_V_AFE_REG7_SW_ADC_MUX_ADC_FLAG_Q << MXC_F_AFE_REG7_SW_ADC_MUX_POS) /**< REG7_SW_ADC_MUX_ADC_FLAG_Q Setting */
#define MXC_V_AFE_REG7_SW_ADC_MUX_TX_FB_OUT_D          ((uint8_t)0x3UL) /**< REG7_SW_ADC_MUX_TX_FB_OUT_D Value */
#define MXC_S_AFE_REG7_SW_ADC_MUX_TX_FB_OUT_D          (MXC_V_AFE_REG7_SW_ADC_MUX_TX_FB_OUT_D << MXC_F_AFE_REG7_SW_ADC_MUX_POS) /**< REG7_SW_ADC_MUX_TX_FB_OUT_D Setting */
#define MXC_V_AFE_REG7_SW_ADC_MUX_PVTD_FLAG            ((uint8_t)0x4UL) /**< REG7_SW_ADC_MUX_PVTD_FLAG Value */
#define MXC_S_AFE_REG7_SW_ADC_MUX_PVTD_FLAG            (MXC_V_AFE_REG7_SW_ADC_MUX_PVTD_FLAG << MXC_F_AFE_REG7_SW_ADC_MUX_POS) /**< REG7_SW_ADC_MUX_PVTD_FLAG Setting */
#define MXC_V_AFE_REG7_SW_ADC_MUX_PVTD_OUT_D           ((uint8_t)0x5UL) /**< REG7_SW_ADC_MUX_PVTD_OUT_D Value */
#define MXC_S_AFE_REG7_SW_ADC_MUX_PVTD_OUT_D           (MXC_V_AFE_REG7_SW_ADC_MUX_PVTD_OUT_D << MXC_F_AFE_REG7_SW_ADC_MUX_POS) /**< REG7_SW_ADC_MUX_PVTD_OUT_D Setting */

/**@} end of group AFE_REG7_Register */

/**
 * @ingroup  afe_registers
 * @defgroup AFE_REG8 AFE_REG8
 * @brief    reg8
 * @{
 */
#define MXC_F_AFE_REG8_D2S_MUX_POS                     6 /**< REG8_D2S_MUX Position */
#define MXC_F_AFE_REG8_D2S_MUX                         ((uint8_t)(0x1UL << MXC_F_AFE_REG8_D2S_MUX_POS)) /**< REG8_D2S_MUX Mask */
#define MXC_V_AFE_REG8_D2S_MUX_IF_MIXER                ((uint8_t)0x0UL) /**< REG8_D2S_MUX_IF_MIXER Value */
#define MXC_S_AFE_REG8_D2S_MUX_IF_MIXER                (MXC_V_AFE_REG8_D2S_MUX_IF_MIXER << MXC_F_AFE_REG8_D2S_MUX_POS) /**< REG8_D2S_MUX_IF_MIXER Setting */
#define MXC_V_AFE_REG8_D2S_MUX_LPF                     ((uint8_t)0x1UL) /**< REG8_D2S_MUX_LPF Value */
#define MXC_S_AFE_REG8_D2S_MUX_LPF                     (MXC_V_AFE_REG8_D2S_MUX_LPF << MXC_F_AFE_REG8_D2S_MUX_POS) /**< REG8_D2S_MUX_LPF Setting */

#define MXC_F_AFE_REG8_SWAP_IQ_POS                     7 /**< REG8_SWAP_IQ Position */
#define MXC_F_AFE_REG8_SWAP_IQ                         ((uint8_t)(0x1UL << MXC_F_AFE_REG8_SWAP_IQ_POS)) /**< REG8_SWAP_IQ Mask */

/**@} end of group AFE_REG8_Register */

/**
 * @ingroup  afe_registers
 * @defgroup AFE_REG9 AFE_REG9
 * @brief    reg9
 * @{
 */
#define MXC_F_AFE_REG9_LNA_EN_POS                      0 /**< REG9_LNA_EN Position */
#define MXC_F_AFE_REG9_LNA_EN                          ((uint8_t)(0x1UL << MXC_F_AFE_REG9_LNA_EN_POS)) /**< REG9_LNA_EN Mask */

#define MXC_F_AFE_REG9_MX1_EN_POS                      1 /**< REG9_MX1_EN Position */
#define MXC_F_AFE_REG9_MX1_EN                          ((uint8_t)(0x1UL << MXC_F_AFE_REG9_MX1_EN_POS)) /**< REG9_MX1_EN Mask */

#define MXC_F_AFE_REG9_MX2_EN_POS                      2 /**< REG9_MX2_EN Position */
#define MXC_F_AFE_REG9_MX2_EN                          ((uint8_t)(0x1UL << MXC_F_AFE_REG9_MX2_EN_POS)) /**< REG9_MX2_EN Mask */

#define MXC_F_AFE_REG9_LPF_EN_POS                      3 /**< REG9_LPF_EN Position */
#define MXC_F_AFE_REG9_LPF_EN                          ((uint8_t)(0x1UL << MXC_F_AFE_REG9_LPF_EN_POS)) /**< REG9_LPF_EN Mask */

#define MXC_F_AFE_REG9_CLK_ADC_EN_POS                  4 /**< REG9_CLK_ADC_EN Position */
#define MXC_F_AFE_REG9_CLK_ADC_EN                      ((uint8_t)(0x1UL << MXC_F_AFE_REG9_CLK_ADC_EN_POS)) /**< REG9_CLK_ADC_EN Mask */

#define MXC_F_AFE_REG9_D2S_BUF_EN_POS                  5 /**< REG9_D2S_BUF_EN Position */
#define MXC_F_AFE_REG9_D2S_BUF_EN                      ((uint8_t)(0x1UL << MXC_F_AFE_REG9_D2S_BUF_EN_POS)) /**< REG9_D2S_BUF_EN Mask */

#define MXC_F_AFE_REG9_CON_SW_RW_POS                   6 /**< REG9_CON_SW_RW Position */
#define MXC_F_AFE_REG9_CON_SW_RW                       ((uint8_t)(0x1UL << MXC_F_AFE_REG9_CON_SW_RW_POS)) /**< REG9_CON_SW_RW Mask */

#define MXC_F_AFE_REG9_BIAS_EN_POS                     7 /**< REG9_BIAS_EN Position */
#define MXC_F_AFE_REG9_BIAS_EN                         ((uint8_t)(0x1UL << MXC_F_AFE_REG9_BIAS_EN_POS)) /**< REG9_BIAS_EN Mask */

/**@} end of group AFE_REG9_Register */

/**
 * @ingroup  afe_registers
 * @defgroup AFE_REG10 AFE_REG10
 * @brief    reg10
 * @{
 */
#define MXC_F_AFE_REG10_LNA_HG_POS                     0 /**< REG10_LNA_HG Position */
#define MXC_F_AFE_REG10_LNA_HG                         ((uint8_t)(0x3UL << MXC_F_AFE_REG10_LNA_HG_POS)) /**< REG10_LNA_HG Mask */
#define MXC_V_AFE_REG10_LNA_HG_N15_3                   ((uint8_t)0x0UL) /**< REG10_LNA_HG_N15_3 Value */
#define MXC_S_AFE_REG10_LNA_HG_N15_3                   (MXC_V_AFE_REG10_LNA_HG_N15_3 << MXC_F_AFE_REG10_LNA_HG_POS) /**< REG10_LNA_HG_N15_3 Setting */
#define MXC_V_AFE_REG10_LNA_HG_1_5                     ((uint8_t)0x1UL) /**< REG10_LNA_HG_1_5 Value */
#define MXC_S_AFE_REG10_LNA_HG_1_5                     (MXC_V_AFE_REG10_LNA_HG_1_5 << MXC_F_AFE_REG10_LNA_HG_POS) /**< REG10_LNA_HG_1_5 Setting */
#define MXC_V_AFE_REG10_LNA_HG_12_7                    ((uint8_t)0x2UL) /**< REG10_LNA_HG_12_7 Value */
#define MXC_S_AFE_REG10_LNA_HG_12_7                    (MXC_V_AFE_REG10_LNA_HG_12_7 << MXC_F_AFE_REG10_LNA_HG_POS) /**< REG10_LNA_HG_12_7 Setting */
#define MXC_V_AFE_REG10_LNA_HG_24                      ((uint8_t)0x3UL) /**< REG10_LNA_HG_24 Value */
#define MXC_S_AFE_REG10_LNA_HG_24                      (MXC_V_AFE_REG10_LNA_HG_24 << MXC_F_AFE_REG10_LNA_HG_POS) /**< REG10_LNA_HG_24 Setting */

#define MXC_F_AFE_REG10_MX1_HG_POS                     2 /**< REG10_MX1_HG Position */
#define MXC_F_AFE_REG10_MX1_HG                         ((uint8_t)(0x1UL << MXC_F_AFE_REG10_MX1_HG_POS)) /**< REG10_MX1_HG Mask */

#define MXC_F_AFE_REG10_MX2_HG_POS                     3 /**< REG10_MX2_HG Position */
#define MXC_F_AFE_REG10_MX2_HG                         ((uint8_t)(0x1UL << MXC_F_AFE_REG10_MX2_HG_POS)) /**< REG10_MX2_HG Mask */

#define MXC_F_AFE_REG10_LPF_GAIN_POS                   4 /**< REG10_LPF_GAIN Position */
#define MXC_F_AFE_REG10_LPF_GAIN                       ((uint8_t)(0xFUL << MXC_F_AFE_REG10_LPF_GAIN_POS)) /**< REG10_LPF_GAIN Mask */
#define MXC_V_AFE_REG10_LPF_GAIN_0DB                   ((uint8_t)0x0UL) /**< REG10_LPF_GAIN_0DB Value */
#define MXC_S_AFE_REG10_LPF_GAIN_0DB                   (MXC_V_AFE_REG10_LPF_GAIN_0DB << MXC_F_AFE_REG10_LPF_GAIN_POS) /**< REG10_LPF_GAIN_0DB Setting */
#define MXC_V_AFE_REG10_LPF_GAIN_3DB                   ((uint8_t)0x1UL) /**< REG10_LPF_GAIN_3DB Value */
#define MXC_S_AFE_REG10_LPF_GAIN_3DB                   (MXC_V_AFE_REG10_LPF_GAIN_3DB << MXC_F_AFE_REG10_LPF_GAIN_POS) /**< REG10_LPF_GAIN_3DB Setting */
#define MXC_V_AFE_REG10_LPF_GAIN_6DB                   ((uint8_t)0x2UL) /**< REG10_LPF_GAIN_6DB Value */
#define MXC_S_AFE_REG10_LPF_GAIN_6DB                   (MXC_V_AFE_REG10_LPF_GAIN_6DB << MXC_F_AFE_REG10_LPF_GAIN_POS) /**< REG10_LPF_GAIN_6DB Setting */
#define MXC_V_AFE_REG10_LPF_GAIN_9DB                   ((uint8_t)0x3UL) /**< REG10_LPF_GAIN_9DB Value */
#define MXC_S_AFE_REG10_LPF_GAIN_9DB                   (MXC_V_AFE_REG10_LPF_GAIN_9DB << MXC_F_AFE_REG10_LPF_GAIN_POS) /**< REG10_LPF_GAIN_9DB Setting */
#define MXC_V_AFE_REG10_LPF_GAIN_12DB                  ((uint8_t)0x4UL) /**< REG10_LPF_GAIN_12DB Value */
#define MXC_S_AFE_REG10_LPF_GAIN_12DB                  (MXC_V_AFE_REG10_LPF_GAIN_12DB << MXC_F_AFE_REG10_LPF_GAIN_POS) /**< REG10_LPF_GAIN_12DB Setting */
#define MXC_V_AFE_REG10_LPF_GAIN_15DB                  ((uint8_t)0x5UL) /**< REG10_LPF_GAIN_15DB Value */
#define MXC_S_AFE_REG10_LPF_GAIN_15DB                  (MXC_V_AFE_REG10_LPF_GAIN_15DB << MXC_F_AFE_REG10_LPF_GAIN_POS) /**< REG10_LPF_GAIN_15DB Setting */
#define MXC_V_AFE_REG10_LPF_GAIN_18DB                  ((uint8_t)0x6UL) /**< REG10_LPF_GAIN_18DB Value */
#define MXC_S_AFE_REG10_LPF_GAIN_18DB                  (MXC_V_AFE_REG10_LPF_GAIN_18DB << MXC_F_AFE_REG10_LPF_GAIN_POS) /**< REG10_LPF_GAIN_18DB Setting */
#define MXC_V_AFE_REG10_LPF_GAIN_21DB                  ((uint8_t)0x7UL) /**< REG10_LPF_GAIN_21DB Value */
#define MXC_S_AFE_REG10_LPF_GAIN_21DB                  (MXC_V_AFE_REG10_LPF_GAIN_21DB << MXC_F_AFE_REG10_LPF_GAIN_POS) /**< REG10_LPF_GAIN_21DB Setting */
#define MXC_V_AFE_REG10_LPF_GAIN_24DB                  ((uint8_t)0x8UL) /**< REG10_LPF_GAIN_24DB Value */
#define MXC_S_AFE_REG10_LPF_GAIN_24DB                  (MXC_V_AFE_REG10_LPF_GAIN_24DB << MXC_F_AFE_REG10_LPF_GAIN_POS) /**< REG10_LPF_GAIN_24DB Setting */
#define MXC_V_AFE_REG10_LPF_GAIN_27DB                  ((uint8_t)0x9UL) /**< REG10_LPF_GAIN_27DB Value */
#define MXC_S_AFE_REG10_LPF_GAIN_27DB                  (MXC_V_AFE_REG10_LPF_GAIN_27DB << MXC_F_AFE_REG10_LPF_GAIN_POS) /**< REG10_LPF_GAIN_27DB Setting */
#define MXC_V_AFE_REG10_LPF_GAIN_30DB                  ((uint8_t)0xAUL) /**< REG10_LPF_GAIN_30DB Value */
#define MXC_S_AFE_REG10_LPF_GAIN_30DB                  (MXC_V_AFE_REG10_LPF_GAIN_30DB << MXC_F_AFE_REG10_LPF_GAIN_POS) /**< REG10_LPF_GAIN_30DB Setting */
#define MXC_V_AFE_REG10_LPF_GAIN_33DB                  ((uint8_t)0xBUL) /**< REG10_LPF_GAIN_33DB Value */
#define MXC_S_AFE_REG10_LPF_GAIN_33DB                  (MXC_V_AFE_REG10_LPF_GAIN_33DB << MXC_F_AFE_REG10_LPF_GAIN_POS) /**< REG10_LPF_GAIN_33DB Setting */
#define MXC_V_AFE_REG10_LPF_GAIN_36DB                  ((uint8_t)0xCUL) /**< REG10_LPF_GAIN_36DB Value */
#define MXC_S_AFE_REG10_LPF_GAIN_36DB                  (MXC_V_AFE_REG10_LPF_GAIN_36DB << MXC_F_AFE_REG10_LPF_GAIN_POS) /**< REG10_LPF_GAIN_36DB Setting */

/**@} end of group AFE_REG10_Register */

/**
 * @ingroup  afe_registers
 * @defgroup AFE_REG11 AFE_REG11
 * @brief    reg11
 * @{
 */
#define MXC_F_AFE_REG11_LNA_BC_POS                     0 /**< REG11_LNA_BC Position */
#define MXC_F_AFE_REG11_LNA_BC                         ((uint8_t)(0x7UL << MXC_F_AFE_REG11_LNA_BC_POS)) /**< REG11_LNA_BC Mask */

#define MXC_F_AFE_REG11_MX1_BW_POS                     3 /**< REG11_MX1_BW Position */
#define MXC_F_AFE_REG11_MX1_BW                         ((uint8_t)(0x7UL << MXC_F_AFE_REG11_MX1_BW_POS)) /**< REG11_MX1_BW Mask */

#define MXC_F_AFE_REG11_DC_DAC_EN_POS                  6 /**< REG11_DC_DAC_EN Position */
#define MXC_F_AFE_REG11_DC_DAC_EN                      ((uint8_t)(0x1UL << MXC_F_AFE_REG11_DC_DAC_EN_POS)) /**< REG11_DC_DAC_EN Mask */

#define MXC_F_AFE_REG11_CLK_DC_DAC_POS                 7 /**< REG11_CLK_DC_DAC Position */
#define MXC_F_AFE_REG11_CLK_DC_DAC                     ((uint8_t)(0x1UL << MXC_F_AFE_REG11_CLK_DC_DAC_POS)) /**< REG11_CLK_DC_DAC Mask */

/**@} end of group AFE_REG11_Register */

/**
 * @ingroup  afe_registers
 * @defgroup AFE_REG12 AFE_REG12
 * @brief    reg12
 * @{
 */
#define MXC_F_AFE_REG12_LPF_BW_POS                     0 /**< REG12_LPF_BW Position */
#define MXC_F_AFE_REG12_LPF_BW                         ((uint8_t)(0x7UL << MXC_F_AFE_REG12_LPF_BW_POS)) /**< REG12_LPF_BW Mask */
#define MXC_V_AFE_REG12_LPF_BW_0_76_BW                 ((uint8_t)0x0UL) /**< REG12_LPF_BW_0_76_BW Value */
#define MXC_S_AFE_REG12_LPF_BW_0_76_BW                 (MXC_V_AFE_REG12_LPF_BW_0_76_BW << MXC_F_AFE_REG12_LPF_BW_POS) /**< REG12_LPF_BW_0_76_BW Setting */
#define MXC_V_AFE_REG12_LPF_BW_1_0_BW                  ((uint8_t)0x4UL) /**< REG12_LPF_BW_1_0_BW Value */
#define MXC_S_AFE_REG12_LPF_BW_1_0_BW                  (MXC_V_AFE_REG12_LPF_BW_1_0_BW << MXC_F_AFE_REG12_LPF_BW_POS) /**< REG12_LPF_BW_1_0_BW Setting */
#define MXC_V_AFE_REG12_LPF_BW_1_66_BW                 ((uint8_t)0x7UL) /**< REG12_LPF_BW_1_66_BW Value */
#define MXC_S_AFE_REG12_LPF_BW_1_66_BW                 (MXC_V_AFE_REG12_LPF_BW_1_66_BW << MXC_F_AFE_REG12_LPF_BW_POS) /**< REG12_LPF_BW_1_66_BW Setting */

#define MXC_F_AFE_REG12_LPF_MODE_POS                   3 /**< REG12_LPF_MODE Position */
#define MXC_F_AFE_REG12_LPF_MODE                       ((uint8_t)(0x1UL << MXC_F_AFE_REG12_LPF_MODE_POS)) /**< REG12_LPF_MODE Mask */
#define MXC_V_AFE_REG12_LPF_MODE_500K                  ((uint8_t)0x0UL) /**< REG12_LPF_MODE_500K Value */
#define MXC_S_AFE_REG12_LPF_MODE_500K                  (MXC_V_AFE_REG12_LPF_MODE_500K << MXC_F_AFE_REG12_LPF_MODE_POS) /**< REG12_LPF_MODE_500K Setting */
#define MXC_V_AFE_REG12_LPF_MODE_1M                    ((uint8_t)0x1UL) /**< REG12_LPF_MODE_1M Value */
#define MXC_S_AFE_REG12_LPF_MODE_1M                    (MXC_V_AFE_REG12_LPF_MODE_1M << MXC_F_AFE_REG12_LPF_MODE_POS) /**< REG12_LPF_MODE_1M Setting */

#define MXC_F_AFE_REG12_ADC_SF_POS                     4 /**< REG12_ADC_SF Position */
#define MXC_F_AFE_REG12_ADC_SF                         ((uint8_t)(0x1UL << MXC_F_AFE_REG12_ADC_SF_POS)) /**< REG12_ADC_SF Mask */
#define MXC_V_AFE_REG12_ADC_SF_4_8M                    ((uint8_t)0x0UL) /**< REG12_ADC_SF_4_8M Value */
#define MXC_S_AFE_REG12_ADC_SF_4_8M                    (MXC_V_AFE_REG12_ADC_SF_4_8M << MXC_F_AFE_REG12_ADC_SF_POS) /**< REG12_ADC_SF_4_8M Setting */
#define MXC_V_AFE_REG12_ADC_SF_8M                      ((uint8_t)0x1UL) /**< REG12_ADC_SF_8M Value */
#define MXC_S_AFE_REG12_ADC_SF_8M                      (MXC_V_AFE_REG12_ADC_SF_8M << MXC_F_AFE_REG12_ADC_SF_POS) /**< REG12_ADC_SF_8M Setting */

/**@} end of group AFE_REG12_Register */

/**
 * @ingroup  afe_registers
 * @defgroup AFE_REG13 AFE_REG13
 * @brief    reg13
 * @{
 */
#define MXC_F_AFE_REG13_IB_OP_POS                      0 /**< REG13_IB_OP Position */
#define MXC_F_AFE_REG13_IB_OP                          ((uint8_t)(0x3UL << MXC_F_AFE_REG13_IB_OP_POS)) /**< REG13_IB_OP Mask */

#define MXC_F_AFE_REG13_EN_SWING_POS                   2 /**< REG13_EN_SWING Position */
#define MXC_F_AFE_REG13_EN_SWING                       ((uint8_t)(0x7UL << MXC_F_AFE_REG13_EN_SWING_POS)) /**< REG13_EN_SWING Mask */
#define MXC_V_AFE_REG13_EN_SWING_HIZ                   ((uint8_t)0x0UL) /**< REG13_EN_SWING_HIZ Value */
#define MXC_S_AFE_REG13_EN_SWING_HIZ                   (MXC_V_AFE_REG13_EN_SWING_HIZ << MXC_F_AFE_REG13_EN_SWING_POS) /**< REG13_EN_SWING_HIZ Setting */
#define MXC_V_AFE_REG13_EN_SWING_PLL_TX_VCO            ((uint8_t)0x1UL) /**< REG13_EN_SWING_PLL_TX_VCO Value */
#define MXC_S_AFE_REG13_EN_SWING_PLL_TX_VCO            (MXC_V_AFE_REG13_EN_SWING_PLL_TX_VCO << MXC_F_AFE_REG13_EN_SWING_POS) /**< REG13_EN_SWING_PLL_TX_VCO Setting */
#define MXC_V_AFE_REG13_EN_SWING_PLL_LOBUF             ((uint8_t)0x2UL) /**< REG13_EN_SWING_PLL_LOBUF Value */
#define MXC_S_AFE_REG13_EN_SWING_PLL_LOBUF             (MXC_V_AFE_REG13_EN_SWING_PLL_LOBUF << MXC_F_AFE_REG13_EN_SWING_POS) /**< REG13_EN_SWING_PLL_LOBUF Setting */
#define MXC_V_AFE_REG13_EN_SWING_PLL_FM_DAC            ((uint8_t)0x4UL) /**< REG13_EN_SWING_PLL_FM_DAC Value */
#define MXC_S_AFE_REG13_EN_SWING_PLL_FM_DAC            (MXC_V_AFE_REG13_EN_SWING_PLL_FM_DAC << MXC_F_AFE_REG13_EN_SWING_POS) /**< REG13_EN_SWING_PLL_FM_DAC Setting */
#define MXC_V_AFE_REG13_EN_SWING_LPF_CM                ((uint8_t)0x5UL) /**< REG13_EN_SWING_LPF_CM Value */
#define MXC_S_AFE_REG13_EN_SWING_LPF_CM                (MXC_V_AFE_REG13_EN_SWING_LPF_CM << MXC_F_AFE_REG13_EN_SWING_POS) /**< REG13_EN_SWING_LPF_CM Setting */
#define MXC_V_AFE_REG13_EN_SWING_VTUNE                 ((uint8_t)0x6UL) /**< REG13_EN_SWING_VTUNE Value */
#define MXC_S_AFE_REG13_EN_SWING_VTUNE                 (MXC_V_AFE_REG13_EN_SWING_VTUNE << MXC_F_AFE_REG13_EN_SWING_POS) /**< REG13_EN_SWING_VTUNE Setting */
#define MXC_V_AFE_REG13_EN_SWING_VSS                   ((uint8_t)0x7UL) /**< REG13_EN_SWING_VSS Value */
#define MXC_S_AFE_REG13_EN_SWING_VSS                   (MXC_V_AFE_REG13_EN_SWING_VSS << MXC_F_AFE_REG13_EN_SWING_POS) /**< REG13_EN_SWING_VSS Setting */

#define MXC_F_AFE_REG13_MON_SW_POS                     5 /**< REG13_MON_SW Position */
#define MXC_F_AFE_REG13_MON_SW                         ((uint8_t)(0x3UL << MXC_F_AFE_REG13_MON_SW_POS)) /**< REG13_MON_SW Mask */
#define MXC_V_AFE_REG13_MON_SW_IQ                      ((uint8_t)0x0UL) /**< REG13_MON_SW_IQ Value */
#define MXC_S_AFE_REG13_MON_SW_IQ                      (MXC_V_AFE_REG13_MON_SW_IQ << MXC_F_AFE_REG13_MON_SW_POS) /**< REG13_MON_SW_IQ Setting */
#define MXC_V_AFE_REG13_MON_SW_SW_ADC                  ((uint8_t)0x1UL) /**< REG13_MON_SW_SW_ADC Value */
#define MXC_S_AFE_REG13_MON_SW_SW_ADC                  (MXC_V_AFE_REG13_MON_SW_SW_ADC << MXC_F_AFE_REG13_MON_SW_POS) /**< REG13_MON_SW_SW_ADC Setting */
#define MXC_V_AFE_REG13_MON_SW_FM_CNT                  ((uint8_t)0x3UL) /**< REG13_MON_SW_FM_CNT Value */
#define MXC_S_AFE_REG13_MON_SW_FM_CNT                  (MXC_V_AFE_REG13_MON_SW_FM_CNT << MXC_F_AFE_REG13_MON_SW_POS) /**< REG13_MON_SW_FM_CNT Setting */

#define MXC_F_AFE_REG13_SW_ADC_EN_POS                  7 /**< REG13_SW_ADC_EN Position */
#define MXC_F_AFE_REG13_SW_ADC_EN                      ((uint8_t)(0x1UL << MXC_F_AFE_REG13_SW_ADC_EN_POS)) /**< REG13_SW_ADC_EN Mask */

/**@} end of group AFE_REG13_Register */

/**
 * @ingroup  afe_registers
 * @defgroup AFE_REG14 AFE_REG14
 * @brief    reg14
 * @{
 */
#define MXC_F_AFE_REG14_FM_CAL_EN_POS                  0 /**< REG14_FM_CAL_EN Position */
#define MXC_F_AFE_REG14_FM_CAL_EN                      ((uint8_t)(0x1UL << MXC_F_AFE_REG14_FM_CAL_EN_POS)) /**< REG14_FM_CAL_EN Mask */

#define MXC_F_AFE_REG14_START_FM_CNT_POS               1 /**< REG14_START_FM_CNT Position */
#define MXC_F_AFE_REG14_START_FM_CNT                   ((uint8_t)(0x1UL << MXC_F_AFE_REG14_START_FM_CNT_POS)) /**< REG14_START_FM_CNT Mask */

#define MXC_F_AFE_REG14_FM_CNT_ADDR_POS                2 /**< REG14_FM_CNT_ADDR Position */
#define MXC_F_AFE_REG14_FM_CNT_ADDR                    ((uint8_t)(0x7UL << MXC_F_AFE_REG14_FM_CNT_ADDR_POS)) /**< REG14_FM_CNT_ADDR Mask */

#define MXC_F_AFE_REG14_FM_RES_POS                     5 /**< REG14_FM_RES Position */
#define MXC_F_AFE_REG14_FM_RES                         ((uint8_t)(0x3UL << MXC_F_AFE_REG14_FM_RES_POS)) /**< REG14_FM_RES Mask */

#define MXC_F_AFE_REG14_TM_CP_UP_POS                   7 /**< REG14_TM_CP_UP Position */
#define MXC_F_AFE_REG14_TM_CP_UP                       ((uint8_t)(0x1UL << MXC_F_AFE_REG14_TM_CP_UP_POS)) /**< REG14_TM_CP_UP Mask */

/**@} end of group AFE_REG14_Register */

/**
 * @ingroup  afe_registers
 * @defgroup AFE_REG15 AFE_REG15
 * @brief    reg15
 * @{
 */
#define MXC_F_AFE_REG15_TM_EXT_CLK_POS                 0 /**< REG15_TM_EXT_CLK Position */
#define MXC_F_AFE_REG15_TM_EXT_CLK                     ((uint8_t)(0x1FUL << MXC_F_AFE_REG15_TM_EXT_CLK_POS)) /**< REG15_TM_EXT_CLK Mask */

#define MXC_F_AFE_REG15_TM_CLK_OUT_SEL_POS             5 /**< REG15_TM_CLK_OUT_SEL Position */
#define MXC_F_AFE_REG15_TM_CLK_OUT_SEL                 ((uint8_t)(0x3UL << MXC_F_AFE_REG15_TM_CLK_OUT_SEL_POS)) /**< REG15_TM_CLK_OUT_SEL Mask */

#define MXC_F_AFE_REG15_TCLK_SRC_POS                   7 /**< REG15_TCLK_SRC Position */
#define MXC_F_AFE_REG15_TCLK_SRC                       ((uint8_t)(0x1UL << MXC_F_AFE_REG15_TCLK_SRC_POS)) /**< REG15_TCLK_SRC Mask */

/**@} end of group AFE_REG15_Register */

/**
 * @ingroup  afe_registers
 * @defgroup AFE_REGA1 AFE_REGA1
 * @brief    rega1
 * @{
 */
#define MXC_F_AFE_REGA1_FRAC_EN_POS                    1 /**< REGA1_FRAC_EN Position */
#define MXC_F_AFE_REGA1_FRAC_EN                        ((uint8_t)(0x1UL << MXC_F_AFE_REGA1_FRAC_EN_POS)) /**< REGA1_FRAC_EN Mask */

#define MXC_F_AFE_REGA1_DIV_FRAC_POS                   4 /**< REGA1_DIV_FRAC Position */
#define MXC_F_AFE_REGA1_DIV_FRAC                       ((uint8_t)(0xFUL << MXC_F_AFE_REGA1_DIV_FRAC_POS)) /**< REGA1_DIV_FRAC Mask */

/**@} end of group AFE_REGA1_Register */

/**
 * @ingroup  afe_registers
 * @defgroup AFE_REGA2 AFE_REGA2
 * @brief    rega2
 * @{
 */
#define MXC_F_AFE_REGA2_DIV_FRAC_POS                   0 /**< REGA2_DIV_FRAC Position */
#define MXC_F_AFE_REGA2_DIV_FRAC                       ((uint8_t)(0xFFUL << MXC_F_AFE_REGA2_DIV_FRAC_POS)) /**< REGA2_DIV_FRAC Mask */

/**@} end of group AFE_REGA2_Register */

/**
 * @ingroup  afe_registers
 * @defgroup AFE_REGA3 AFE_REGA3
 * @brief    rega3
 * @{
 */
#define MXC_F_AFE_REGA3_DIV_FRAC_POS                   0 /**< REGA3_DIV_FRAC Position */
#define MXC_F_AFE_REGA3_DIV_FRAC                       ((uint8_t)(0x7UL << MXC_F_AFE_REGA3_DIV_FRAC_POS)) /**< REGA3_DIV_FRAC Mask */

#define MXC_F_AFE_REGA3_DIV_INT_POS                    3 /**< REGA3_DIV_INT Position */
#define MXC_F_AFE_REGA3_DIV_INT                        ((uint8_t)(0x1FUL << MXC_F_AFE_REGA3_DIV_INT_POS)) /**< REGA3_DIV_INT Mask */

/**@} end of group AFE_REGA3_Register */

/**
 * @ingroup  afe_registers
 * @defgroup AFE_REGA4 AFE_REGA4
 * @brief    rega4
 * @{
 */
#define MXC_F_AFE_REGA4_SEL_HFP_POS                    3 /**< REGA4_SEL_HFP Position */
#define MXC_F_AFE_REGA4_SEL_HFP                        ((uint8_t)(0x1UL << MXC_F_AFE_REGA4_SEL_HFP_POS)) /**< REGA4_SEL_HFP Mask */
#define MXC_V_AFE_REGA4_SEL_HFP_FM_EN                  ((uint8_t)0x1UL) /**< REGA4_SEL_HFP_FM_EN Value */
#define MXC_S_AFE_REGA4_SEL_HFP_FM_EN                  (MXC_V_AFE_REGA4_SEL_HFP_FM_EN << MXC_F_AFE_REGA4_SEL_HFP_POS) /**< REGA4_SEL_HFP_FM_EN Setting */
#define MXC_V_AFE_REGA4_SEL_HFP_FM_DIS                 ((uint8_t)0x0UL) /**< REGA4_SEL_HFP_FM_DIS Value */
#define MXC_S_AFE_REGA4_SEL_HFP_FM_DIS                 (MXC_V_AFE_REGA4_SEL_HFP_FM_DIS << MXC_F_AFE_REGA4_SEL_HFP_POS) /**< REGA4_SEL_HFP_FM_DIS Setting */

#define MXC_F_AFE_REGA4_F_DEV_POS                      7 /**< REGA4_F_DEV Position */
#define MXC_F_AFE_REGA4_F_DEV                          ((uint8_t)(0x1UL << MXC_F_AFE_REGA4_F_DEV_POS)) /**< REGA4_F_DEV Mask */

/**@} end of group AFE_REGA4_Register */

/**
 * @ingroup  afe_registers
 * @defgroup AFE_REGA5 AFE_REGA5
 * @brief    rega5
 * @{
 */
#define MXC_F_AFE_REGA5_F_DEV_POS                      0 /**< REGA5_F_DEV Position */
#define MXC_F_AFE_REGA5_F_DEV                          ((uint8_t)(0xFFUL << MXC_F_AFE_REGA5_F_DEV_POS)) /**< REGA5_F_DEV Mask */

/**@} end of group AFE_REGA5_Register */

/**
 * @ingroup  afe_registers
 * @defgroup AFE_REGA6 AFE_REGA6
 * @brief    rega6
 * @{
 */
#define MXC_F_AFE_REGA6_DL_DIV_POS                     0 /**< REGA6_DL_DIV Position */
#define MXC_F_AFE_REGA6_DL_DIV                         ((uint8_t)(0xFUL << MXC_F_AFE_REGA6_DL_DIV_POS)) /**< REGA6_DL_DIV Mask */

#define MXC_F_AFE_REGA6_DL_DAC_POS                     4 /**< REGA6_DL_DAC Position */
#define MXC_F_AFE_REGA6_DL_DAC                         ((uint8_t)(0xFUL << MXC_F_AFE_REGA6_DL_DAC_POS)) /**< REGA6_DL_DAC Mask */

/**@} end of group AFE_REGA6_Register */

/**
 * @ingroup  afe_registers
 * @defgroup AFE_REGA7 AFE_REGA7
 * @brief    rega7
 * @{
 */
#define MXC_F_AFE_REGA7_DL_AMP_POS                     0 /**< REGA7_DL_AMP Position */
#define MXC_F_AFE_REGA7_DL_AMP                         ((uint8_t)(0xFUL << MXC_F_AFE_REGA7_DL_AMP_POS)) /**< REGA7_DL_AMP Mask */

#define MXC_F_AFE_REGA7_AMP_COEF_POS                   4 /**< REGA7_AMP_COEF Position */
#define MXC_F_AFE_REGA7_AMP_COEF                       ((uint8_t)(0xFUL << MXC_F_AFE_REGA7_AMP_COEF_POS)) /**< REGA7_AMP_COEF Mask */

/**@} end of group AFE_REGA7_Register */

/**
 * @ingroup  afe_registers
 * @defgroup AFE_REGA8 AFE_REGA8
 * @brief    rega8
 * @{
 */
#define MXC_F_AFE_REGA8_DATA_CLK_EN_POS                0 /**< REGA8_DATA_CLK_EN Position */
#define MXC_F_AFE_REGA8_DATA_CLK_EN                    ((uint8_t)(0x1UL << MXC_F_AFE_REGA8_DATA_CLK_EN_POS)) /**< REGA8_DATA_CLK_EN Mask */

#define MXC_F_AFE_REGA8_SDMA_EN_POS                    1 /**< REGA8_SDMA_EN Position */
#define MXC_F_AFE_REGA8_SDMA_EN                        ((uint8_t)(0x1UL << MXC_F_AFE_REGA8_SDMA_EN_POS)) /**< REGA8_SDMA_EN Mask */

#define MXC_F_AFE_REGA8_SEL_AMP_POS                    2 /**< REGA8_SEL_AMP Position */
#define MXC_F_AFE_REGA8_SEL_AMP                        ((uint8_t)(0x1UL << MXC_F_AFE_REGA8_SEL_AMP_POS)) /**< REGA8_SEL_AMP Mask */
#define MXC_V_AFE_REGA8_SEL_AMP_INTERNAL               ((uint8_t)0x0UL) /**< REGA8_SEL_AMP_INTERNAL Value */
#define MXC_S_AFE_REGA8_SEL_AMP_INTERNAL               (MXC_V_AFE_REGA8_SEL_AMP_INTERNAL << MXC_F_AFE_REGA8_SEL_AMP_POS) /**< REGA8_SEL_AMP_INTERNAL Setting */
#define MXC_V_AFE_REGA8_SEL_AMP_EXTERNAL               ((uint8_t)0x1UL) /**< REGA8_SEL_AMP_EXTERNAL Value */
#define MXC_S_AFE_REGA8_SEL_AMP_EXTERNAL               (MXC_V_AFE_REGA8_SEL_AMP_EXTERNAL << MXC_F_AFE_REGA8_SEL_AMP_POS) /**< REGA8_SEL_AMP_EXTERNAL Setting */

#define MXC_F_AFE_REGA8_AM_FIX_POS                     3 /**< REGA8_AM_FIX Position */
#define MXC_F_AFE_REGA8_AM_FIX                         ((uint8_t)(0x1UL << MXC_F_AFE_REGA8_AM_FIX_POS)) /**< REGA8_AM_FIX Mask */
#define MXC_V_AFE_REGA8_AM_FIX_AMP_COEF                ((uint8_t)0x1UL) /**< REGA8_AM_FIX_AMP_COEF Value */
#define MXC_S_AFE_REGA8_AM_FIX_AMP_COEF                (MXC_V_AFE_REGA8_AM_FIX_AMP_COEF << MXC_F_AFE_REGA8_AM_FIX_POS) /**< REGA8_AM_FIX_AMP_COEF Setting */
#define MXC_V_AFE_REGA8_AM_FIX_INTERNAL                ((uint8_t)0x0UL) /**< REGA8_AM_FIX_INTERNAL Value */
#define MXC_S_AFE_REGA8_AM_FIX_INTERNAL                (MXC_V_AFE_REGA8_AM_FIX_INTERNAL << MXC_F_AFE_REGA8_AM_FIX_POS) /**< REGA8_AM_FIX_INTERNAL Setting */

#define MXC_F_AFE_REGA8_FDEV_D2_POS                    4 /**< REGA8_FDEV_D2 Position */
#define MXC_F_AFE_REGA8_FDEV_D2                        ((uint8_t)(0x1UL << MXC_F_AFE_REGA8_FDEV_D2_POS)) /**< REGA8_FDEV_D2 Mask */
#define MXC_V_AFE_REGA8_FDEV_D2_2K                     ((uint8_t)0x0UL) /**< REGA8_FDEV_D2_2K Value */
#define MXC_S_AFE_REGA8_FDEV_D2_2K                     (MXC_V_AFE_REGA8_FDEV_D2_2K << MXC_F_AFE_REGA8_FDEV_D2_POS) /**< REGA8_FDEV_D2_2K Setting */
#define MXC_V_AFE_REGA8_FDEV_D2_4K                     ((uint8_t)0x1UL) /**< REGA8_FDEV_D2_4K Value */
#define MXC_S_AFE_REGA8_FDEV_D2_4K                     (MXC_V_AFE_REGA8_FDEV_D2_4K << MXC_F_AFE_REGA8_FDEV_D2_POS) /**< REGA8_FDEV_D2_4K Setting */

#define MXC_F_AFE_REGA8_DCOCBY_EN_POS                  5 /**< REGA8_DCOCBY_EN Position */
#define MXC_F_AFE_REGA8_DCOCBY_EN                      ((uint8_t)(0x1UL << MXC_F_AFE_REGA8_DCOCBY_EN_POS)) /**< REGA8_DCOCBY_EN Mask */

#define MXC_F_AFE_REGA8_BLE_24MHZ_POS                  6 /**< REGA8_BLE_24MHZ Position */
#define MXC_F_AFE_REGA8_BLE_24MHZ                      ((uint8_t)(0x1UL << MXC_F_AFE_REGA8_BLE_24MHZ_POS)) /**< REGA8_BLE_24MHZ Mask */
#define MXC_V_AFE_REGA8_BLE_24MHZ_24M                  ((uint8_t)0x1UL) /**< REGA8_BLE_24MHZ_24M Value */
#define MXC_S_AFE_REGA8_BLE_24MHZ_24M                  (MXC_V_AFE_REGA8_BLE_24MHZ_24M << MXC_F_AFE_REGA8_BLE_24MHZ_POS) /**< REGA8_BLE_24MHZ_24M Setting */
#define MXC_V_AFE_REGA8_BLE_24MHZ_32M                  ((uint8_t)0x0UL) /**< REGA8_BLE_24MHZ_32M Value */
#define MXC_S_AFE_REGA8_BLE_24MHZ_32M                  (MXC_V_AFE_REGA8_BLE_24MHZ_32M << MXC_F_AFE_REGA8_BLE_24MHZ_POS) /**< REGA8_BLE_24MHZ_32M Setting */

#define MXC_F_AFE_REGA8_INV_FM_POL_POS                 7 /**< REGA8_INV_FM_POL Position */
#define MXC_F_AFE_REGA8_INV_FM_POL                     ((uint8_t)(0x1UL << MXC_F_AFE_REGA8_INV_FM_POL_POS)) /**< REGA8_INV_FM_POL Mask */
#define MXC_V_AFE_REGA8_INV_FM_POL_INV                 ((uint8_t)0x1UL) /**< REGA8_INV_FM_POL_INV Value */
#define MXC_S_AFE_REGA8_INV_FM_POL_INV                 (MXC_V_AFE_REGA8_INV_FM_POL_INV << MXC_F_AFE_REGA8_INV_FM_POL_POS) /**< REGA8_INV_FM_POL_INV Setting */
#define MXC_V_AFE_REGA8_INV_FM_POL_NON_INV             ((uint8_t)0x0UL) /**< REGA8_INV_FM_POL_NON_INV Value */
#define MXC_S_AFE_REGA8_INV_FM_POL_NON_INV             (MXC_V_AFE_REGA8_INV_FM_POL_NON_INV << MXC_F_AFE_REGA8_INV_FM_POL_POS) /**< REGA8_INV_FM_POL_NON_INV Setting */

/**@} end of group AFE_REGA8_Register */

/**
 * @ingroup  afe_registers
 * @defgroup AFE_REGA9 AFE_REGA9
 * @brief    rega9 *signed*
 * @{
 */
#define MXC_F_AFE_REGA9_DC_I_POS                       0 /**< REGA9_DC_I Position */
#define MXC_F_AFE_REGA9_DC_I                           ((uint8_t)(0xFFUL << MXC_F_AFE_REGA9_DC_I_POS)) /**< REGA9_DC_I Mask */

/**@} end of group AFE_REGA9_Register */

/**
 * @ingroup  afe_registers
 * @defgroup AFE_REGA10 AFE_REGA10
 * @brief    rega10 *signed*
 * @{
 */
#define MXC_F_AFE_REGA10_DC_Q_POS                      0 /**< REGA10_DC_Q Position */
#define MXC_F_AFE_REGA10_DC_Q                          ((uint8_t)(0xFFUL << MXC_F_AFE_REGA10_DC_Q_POS)) /**< REGA10_DC_Q Mask */

/**@} end of group AFE_REGA10_Register */

/**
 * @ingroup  afe_registers
 * @defgroup AFE_REGA11 AFE_REGA11
 * @brief    rega11
 * @{
 */
#define MXC_F_AFE_REGA11_IB_DIV_DAC_POS                0 /**< REGA11_IB_DIV_DAC Position */
#define MXC_F_AFE_REGA11_IB_DIV_DAC                    ((uint8_t)(0xFUL << MXC_F_AFE_REGA11_IB_DIV_DAC_POS)) /**< REGA11_IB_DIV_DAC Mask */

#define MXC_F_AFE_REGA11_PLL_IB_RES_POS                4 /**< REGA11_PLL_IB_RES Position */
#define MXC_F_AFE_REGA11_PLL_IB_RES                    ((uint8_t)(0xFUL << MXC_F_AFE_REGA11_PLL_IB_RES_POS)) /**< REGA11_PLL_IB_RES Mask */

/**@} end of group AFE_REGA11_Register */

/**
 * @ingroup  afe_registers
 * @defgroup AFE_REGA12 AFE_REGA12
 * @brief    rega12
 * @{
 */
#define MXC_F_AFE_REGA12_IB_VDO_DAC_POS                0 /**< REGA12_IB_VDO_DAC Position */
#define MXC_F_AFE_REGA12_IB_VDO_DAC                    ((uint8_t)(0x7FUL << MXC_F_AFE_REGA12_IB_VDO_DAC_POS)) /**< REGA12_IB_VDO_DAC Mask */

/**@} end of group AFE_REGA12_Register */

/**
 * @ingroup  afe_registers
 * @defgroup AFE_REGA13 AFE_REGA13
 * @brief    rega13
 * @{
 */
#define MXC_F_AFE_REGA13_IB_CP_DAC_POS                 0 /**< REGA13_IB_CP_DAC Position */
#define MXC_F_AFE_REGA13_IB_CP_DAC                     ((uint8_t)(0x1FUL << MXC_F_AFE_REGA13_IB_CP_DAC_POS)) /**< REGA13_IB_CP_DAC Mask */

/**@} end of group AFE_REGA13_Register */

/**
 * @ingroup  afe_registers
 * @defgroup AFE_REGA14 AFE_REGA14
 * @brief    rega14
 * @{
 */
#define MXC_F_AFE_REGA14_IB_REF_DAC_POS                0 /**< REGA14_IB_REF_DAC Position */
#define MXC_F_AFE_REGA14_IB_REF_DAC                    ((uint8_t)(0xFUL << MXC_F_AFE_REGA14_IB_REF_DAC_POS)) /**< REGA14_IB_REF_DAC Mask */

#define MXC_F_AFE_REGA14_IB_FM_DAC_POS                 4 /**< REGA14_IB_FM_DAC Position */
#define MXC_F_AFE_REGA14_IB_FM_DAC                     ((uint8_t)(0xFUL << MXC_F_AFE_REGA14_IB_FM_DAC_POS)) /**< REGA14_IB_FM_DAC Mask */

/**@} end of group AFE_REGA14_Register */

/**
 * @ingroup  afe_registers
 * @defgroup AFE_REGA15 AFE_REGA15
 * @brief    rega15
 * @{
 */
#define MXC_F_AFE_REGA15_IB_LOBUF_DAC_POS              0 /**< REGA15_IB_LOBUF_DAC Position */
#define MXC_F_AFE_REGA15_IB_LOBUF_DAC                  ((uint8_t)(0x7FUL << MXC_F_AFE_REGA15_IB_LOBUF_DAC_POS)) /**< REGA15_IB_LOBUF_DAC Mask */

/**@} end of group AFE_REGA15_Register */

/**
 * @ingroup  afe_registers
 * @defgroup AFE_REGA16 AFE_REGA16
 * @brief    rega16
 * @{
 */
#define MXC_F_AFE_REGA16_IB_MX2_TRM_POS                0 /**< REGA16_IB_MX2_TRM Position */
#define MXC_F_AFE_REGA16_IB_MX2_TRM                    ((uint8_t)(0xFUL << MXC_F_AFE_REGA16_IB_MX2_TRM_POS)) /**< REGA16_IB_MX2_TRM Mask */

#define MXC_F_AFE_REGA16_IB_LNA_TRM_POS                4 /**< REGA16_IB_LNA_TRM Position */
#define MXC_F_AFE_REGA16_IB_LNA_TRM                    ((uint8_t)(0xFUL << MXC_F_AFE_REGA16_IB_LNA_TRM_POS)) /**< REGA16_IB_LNA_TRM Mask */

/**@} end of group AFE_REGA16_Register */

/**
 * @ingroup  afe_registers
 * @defgroup AFE_REGB1 AFE_REGB1
 * @brief    regb1
 * @{
 */
#define MXC_F_AFE_REGB1_IB_MX1_TRM_POS                 0 /**< REGB1_IB_MX1_TRM Position */
#define MXC_F_AFE_REGB1_IB_MX1_TRM                     ((uint8_t)(0x7UL << MXC_F_AFE_REGB1_IB_MX1_TRM_POS)) /**< REGB1_IB_MX1_TRM Mask */

#define MXC_F_AFE_REGB1_IB_MX1_TC_POS                  4 /**< REGB1_IB_MX1_TC Position */
#define MXC_F_AFE_REGB1_IB_MX1_TC                      ((uint8_t)(0x7UL << MXC_F_AFE_REGB1_IB_MX1_TC_POS)) /**< REGB1_IB_MX1_TC Mask */

/**@} end of group AFE_REGB1_Register */

/**
 * @ingroup  afe_registers
 * @defgroup AFE_REGB2 AFE_REGB2
 * @brief    regb2
 * @{
 */
#define MXC_F_AFE_REGB2_ANA_MUX_SEL_POS                0 /**< REGB2_ANA_MUX_SEL Position */
#define MXC_F_AFE_REGB2_ANA_MUX_SEL                    ((uint8_t)(0x7UL << MXC_F_AFE_REGB2_ANA_MUX_SEL_POS)) /**< REGB2_ANA_MUX_SEL Mask */
#define MXC_V_AFE_REGB2_ANA_MUX_SEL_GND_ANA            ((uint8_t)0x0UL) /**< REGB2_ANA_MUX_SEL_GND_ANA Value */
#define MXC_S_AFE_REGB2_ANA_MUX_SEL_GND_ANA            (MXC_V_AFE_REGB2_ANA_MUX_SEL_GND_ANA << MXC_F_AFE_REGB2_ANA_MUX_SEL_POS) /**< REGB2_ANA_MUX_SEL_GND_ANA Setting */
#define MXC_V_AFE_REGB2_ANA_MUX_SEL_TM_RX_IB           ((uint8_t)0x1UL) /**< REGB2_ANA_MUX_SEL_TM_RX_IB Value */
#define MXC_S_AFE_REGB2_ANA_MUX_SEL_TM_RX_IB           (MXC_V_AFE_REGB2_ANA_MUX_SEL_TM_RX_IB << MXC_F_AFE_REGB2_ANA_MUX_SEL_POS) /**< REGB2_ANA_MUX_SEL_TM_RX_IB Setting */
#define MXC_V_AFE_REGB2_ANA_MUX_SEL_TM_PLL_IB          ((uint8_t)0x2UL) /**< REGB2_ANA_MUX_SEL_TM_PLL_IB Value */
#define MXC_S_AFE_REGB2_ANA_MUX_SEL_TM_PLL_IB          (MXC_V_AFE_REGB2_ANA_MUX_SEL_TM_PLL_IB << MXC_F_AFE_REGB2_ANA_MUX_SEL_POS) /**< REGB2_ANA_MUX_SEL_TM_PLL_IB Setting */
#define MXC_V_AFE_REGB2_ANA_MUX_SEL_IF_CM              ((uint8_t)0x3UL) /**< REGB2_ANA_MUX_SEL_IF_CM Value */
#define MXC_S_AFE_REGB2_ANA_MUX_SEL_IF_CM              (MXC_V_AFE_REGB2_ANA_MUX_SEL_IF_CM << MXC_F_AFE_REGB2_ANA_MUX_SEL_POS) /**< REGB2_ANA_MUX_SEL_IF_CM Setting */
#define MXC_V_AFE_REGB2_ANA_MUX_SEL_LPF_CM             ((uint8_t)0x4UL) /**< REGB2_ANA_MUX_SEL_LPF_CM Value */
#define MXC_S_AFE_REGB2_ANA_MUX_SEL_LPF_CM             (MXC_V_AFE_REGB2_ANA_MUX_SEL_LPF_CM << MXC_F_AFE_REGB2_ANA_MUX_SEL_POS) /**< REGB2_ANA_MUX_SEL_LPF_CM Setting */
#define MXC_V_AFE_REGB2_ANA_MUX_SEL_VTUNE_OUT          ((uint8_t)0x5UL) /**< REGB2_ANA_MUX_SEL_VTUNE_OUT Value */
#define MXC_S_AFE_REGB2_ANA_MUX_SEL_VTUNE_OUT          (MXC_V_AFE_REGB2_ANA_MUX_SEL_VTUNE_OUT << MXC_F_AFE_REGB2_ANA_MUX_SEL_POS) /**< REGB2_ANA_MUX_SEL_VTUNE_OUT Setting */

#define MXC_F_AFE_REGB2_TM_RX_IB_EN_POS                4 /**< REGB2_TM_RX_IB_EN Position */
#define MXC_F_AFE_REGB2_TM_RX_IB_EN                    ((uint8_t)(0x3UL << MXC_F_AFE_REGB2_TM_RX_IB_EN_POS)) /**< REGB2_TM_RX_IB_EN Mask */
#define MXC_V_AFE_REGB2_TM_RX_IB_EN_GND_MX2            ((uint8_t)0x0UL) /**< REGB2_TM_RX_IB_EN_GND_MX2 Value */
#define MXC_S_AFE_REGB2_TM_RX_IB_EN_GND_MX2            (MXC_V_AFE_REGB2_TM_RX_IB_EN_GND_MX2 << MXC_F_AFE_REGB2_TM_RX_IB_EN_POS) /**< REGB2_TM_RX_IB_EN_GND_MX2 Setting */
#define MXC_V_AFE_REGB2_TM_RX_IB_EN_LNA_IB             ((uint8_t)0x1UL) /**< REGB2_TM_RX_IB_EN_LNA_IB Value */
#define MXC_S_AFE_REGB2_TM_RX_IB_EN_LNA_IB             (MXC_V_AFE_REGB2_TM_RX_IB_EN_LNA_IB << MXC_F_AFE_REGB2_TM_RX_IB_EN_POS) /**< REGB2_TM_RX_IB_EN_LNA_IB Setting */
#define MXC_V_AFE_REGB2_TM_RX_IB_EN_RF_MIXER_IB        ((uint8_t)0x2UL) /**< REGB2_TM_RX_IB_EN_RF_MIXER_IB Value */
#define MXC_S_AFE_REGB2_TM_RX_IB_EN_RF_MIXER_IB        (MXC_V_AFE_REGB2_TM_RX_IB_EN_RF_MIXER_IB << MXC_F_AFE_REGB2_TM_RX_IB_EN_POS) /**< REGB2_TM_RX_IB_EN_RF_MIXER_IB Setting */
#define MXC_V_AFE_REGB2_TM_RX_IB_EN_IF_MIXER_IB        ((uint8_t)0x3UL) /**< REGB2_TM_RX_IB_EN_IF_MIXER_IB Value */
#define MXC_S_AFE_REGB2_TM_RX_IB_EN_IF_MIXER_IB        (MXC_V_AFE_REGB2_TM_RX_IB_EN_IF_MIXER_IB << MXC_F_AFE_REGB2_TM_RX_IB_EN_POS) /**< REGB2_TM_RX_IB_EN_IF_MIXER_IB Setting */

#define MXC_F_AFE_REGB2_TM_IF_LPF_CMO_SEL_POS          6 /**< REGB2_TM_IF_LPF_CMO_SEL Position */
#define MXC_F_AFE_REGB2_TM_IF_LPF_CMO_SEL              ((uint8_t)(0x1UL << MXC_F_AFE_REGB2_TM_IF_LPF_CMO_SEL_POS)) /**< REGB2_TM_IF_LPF_CMO_SEL Mask */
#define MXC_V_AFE_REGB2_TM_IF_LPF_CMO_SEL_Q            ((uint8_t)0x0UL) /**< REGB2_TM_IF_LPF_CMO_SEL_Q Value */
#define MXC_S_AFE_REGB2_TM_IF_LPF_CMO_SEL_Q            (MXC_V_AFE_REGB2_TM_IF_LPF_CMO_SEL_Q << MXC_F_AFE_REGB2_TM_IF_LPF_CMO_SEL_POS) /**< REGB2_TM_IF_LPF_CMO_SEL_Q Setting */
#define MXC_V_AFE_REGB2_TM_IF_LPF_CMO_SEL_I            ((uint8_t)0x1UL) /**< REGB2_TM_IF_LPF_CMO_SEL_I Value */
#define MXC_S_AFE_REGB2_TM_IF_LPF_CMO_SEL_I            (MXC_V_AFE_REGB2_TM_IF_LPF_CMO_SEL_I << MXC_F_AFE_REGB2_TM_IF_LPF_CMO_SEL_POS) /**< REGB2_TM_IF_LPF_CMO_SEL_I Setting */

#define MXC_F_AFE_REGB2_TM_IF_LPF_CMO_EN_POS           7 /**< REGB2_TM_IF_LPF_CMO_EN Position */
#define MXC_F_AFE_REGB2_TM_IF_LPF_CMO_EN               ((uint8_t)(0x1UL << MXC_F_AFE_REGB2_TM_IF_LPF_CMO_EN_POS)) /**< REGB2_TM_IF_LPF_CMO_EN Mask */

/**@} end of group AFE_REGB2_Register */

/**
 * @ingroup  afe_registers
 * @defgroup AFE_REGB3 AFE_REGB3
 * @brief    regb3
 * @{
 */
#define MXC_F_AFE_REGB3_B_OSC_POS                      0 /**< REGB3_B_OSC Position */
#define MXC_F_AFE_REGB3_B_OSC                          ((uint8_t)(0x7FUL << MXC_F_AFE_REGB3_B_OSC_POS)) /**< REGB3_B_OSC Mask */

#define MXC_F_AFE_REGB3_TX_RXB_POS                     7 /**< REGB3_TX_RXB Position */
#define MXC_F_AFE_REGB3_TX_RXB                         ((uint8_t)(0x1UL << MXC_F_AFE_REGB3_TX_RXB_POS)) /**< REGB3_TX_RXB Mask */

/**@} end of group AFE_REGB3_Register */

/**
 * @ingroup  afe_registers
 * @defgroup AFE_REGB4 AFE_REGB4
 * @brief    regb4
 * @{
 */
#define MXC_F_AFE_REGB4_PA_LSB_POS                     0 /**< REGB4_PA_LSB Position */
#define MXC_F_AFE_REGB4_PA_LSB                         ((uint8_t)(0x7UL << MXC_F_AFE_REGB4_PA_LSB_POS)) /**< REGB4_PA_LSB Mask */

#define MXC_F_AFE_REGB4_PA_CON_DT_POS                  3 /**< REGB4_PA_CON_DT Position */
#define MXC_F_AFE_REGB4_PA_CON_DT                      ((uint8_t)(0x1FUL << MXC_F_AFE_REGB4_PA_CON_DT_POS)) /**< REGB4_PA_CON_DT Mask */

/**@} end of group AFE_REGB4_Register */

/**
 * @ingroup  afe_registers
 * @defgroup AFE_REGB5 AFE_REGB5
 * @brief    regb5
 * @{
 */
#define MXC_F_AFE_REGB5_PA_DT_VCM_POS                  0 /**< REGB5_PA_DT_VCM Position */
#define MXC_F_AFE_REGB5_PA_DT_VCM                      ((uint8_t)(0x3FUL << MXC_F_AFE_REGB5_PA_DT_VCM_POS)) /**< REGB5_PA_DT_VCM Mask */

#define MXC_F_AFE_REGB5_TX_DTCAL_EN_POS                7 /**< REGB5_TX_DTCAL_EN Position */
#define MXC_F_AFE_REGB5_TX_DTCAL_EN                    ((uint8_t)(0x1UL << MXC_F_AFE_REGB5_TX_DTCAL_EN_POS)) /**< REGB5_TX_DTCAL_EN Mask */

/**@} end of group AFE_REGB5_Register */

/**
 * @ingroup  afe_registers
 * @defgroup AFE_REGB6 AFE_REGB6
 * @brief    regb6
 * @{
 */
#define MXC_F_AFE_REGB6_PA_CP_POS                      0 /**< REGB6_PA_CP Position */
#define MXC_F_AFE_REGB6_PA_CP                          ((uint8_t)(0xFUL << MXC_F_AFE_REGB6_PA_CP_POS)) /**< REGB6_PA_CP Mask */

#define MXC_F_AFE_REGB6_ADC_SF_POS                     5 /**< REGB6_ADC_SF Position */
#define MXC_F_AFE_REGB6_ADC_SF                         ((uint8_t)(0x7UL << MXC_F_AFE_REGB6_ADC_SF_POS)) /**< REGB6_ADC_SF Mask */
#define MXC_V_AFE_REGB6_ADC_SF_D2                      ((uint8_t)0x0UL) /**< REGB6_ADC_SF_D2 Value */
#define MXC_S_AFE_REGB6_ADC_SF_D2                      (MXC_V_AFE_REGB6_ADC_SF_D2 << MXC_F_AFE_REGB6_ADC_SF_POS) /**< REGB6_ADC_SF_D2 Setting */
#define MXC_V_AFE_REGB6_ADC_SF_D3                      ((uint8_t)0x1UL) /**< REGB6_ADC_SF_D3 Value */
#define MXC_S_AFE_REGB6_ADC_SF_D3                      (MXC_V_AFE_REGB6_ADC_SF_D3 << MXC_F_AFE_REGB6_ADC_SF_POS) /**< REGB6_ADC_SF_D3 Setting */
#define MXC_V_AFE_REGB6_ADC_SF_D4                      ((uint8_t)0x2UL) /**< REGB6_ADC_SF_D4 Value */
#define MXC_S_AFE_REGB6_ADC_SF_D4                      (MXC_V_AFE_REGB6_ADC_SF_D4 << MXC_F_AFE_REGB6_ADC_SF_POS) /**< REGB6_ADC_SF_D4 Setting */
#define MXC_V_AFE_REGB6_ADC_SF_D6                      ((uint8_t)0x3UL) /**< REGB6_ADC_SF_D6 Value */
#define MXC_S_AFE_REGB6_ADC_SF_D6                      (MXC_V_AFE_REGB6_ADC_SF_D6 << MXC_F_AFE_REGB6_ADC_SF_POS) /**< REGB6_ADC_SF_D6 Setting */
#define MXC_V_AFE_REGB6_ADC_SF_D8                      ((uint8_t)0x4UL) /**< REGB6_ADC_SF_D8 Value */
#define MXC_S_AFE_REGB6_ADC_SF_D8                      (MXC_V_AFE_REGB6_ADC_SF_D8 << MXC_F_AFE_REGB6_ADC_SF_POS) /**< REGB6_ADC_SF_D8 Setting */

/**@} end of group AFE_REGB6_Register */

/**
 * @ingroup  afe_registers
 * @defgroup AFE_REGB7 AFE_REGB7
 * @brief    regb7
 * @{
 */
#define MXC_F_AFE_REGB7_VCM1_RX_POS                    0 /**< REGB7_VCM1_RX Position */
#define MXC_F_AFE_REGB7_VCM1_RX                        ((uint8_t)(0xFUL << MXC_F_AFE_REGB7_VCM1_RX_POS)) /**< REGB7_VCM1_RX Mask */

#define MXC_F_AFE_REGB7_VCM2_RX_POS                    4 /**< REGB7_VCM2_RX Position */
#define MXC_F_AFE_REGB7_VCM2_RX                        ((uint8_t)(0xFUL << MXC_F_AFE_REGB7_VCM2_RX_POS)) /**< REGB7_VCM2_RX Mask */

/**@} end of group AFE_REGB7_Register */

#ifdef __cplusplus
}
#endif

#endif // MAX32665_INCLUDE_AFE_REGS_H_
