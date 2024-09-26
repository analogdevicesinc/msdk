/**
 * @file    scn_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the SCN Peripheral Module.
 * @note    This file is @generated.
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32570_INCLUDE_SCN_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32570_INCLUDE_SCN_REGS_H_

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
 * @ingroup     scn
 * @defgroup    scn_registers SCN_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the SCN Peripheral Module.
 * @details     Smart Card Interface.
 */

/**
 * @ingroup scn_registers
 * Structure type to access the SCN Registers.
 */
typedef struct {
    __IO uint32_t cr;                   /**< <tt>\b 0x00:</tt> SCN CR Register */
    __IO uint32_t sr;                   /**< <tt>\b 0x04:</tt> SCN SR Register */
    __IO uint32_t pn;                   /**< <tt>\b 0x08:</tt> SCN PN Register */
    __IO uint32_t etur;                 /**< <tt>\b 0x0C:</tt> SCN ETUR Register */
    __IO uint32_t gtr;                  /**< <tt>\b 0x10:</tt> SCN GTR Register */
    __IO uint32_t wt0r;                 /**< <tt>\b 0x14:</tt> SCN WT0R Register */
    __IO uint32_t wt1r;                 /**< <tt>\b 0x18:</tt> SCN WT1R Register */
    __IO uint32_t ier;                  /**< <tt>\b 0x1C:</tt> SCN IER Register */
    __IO uint32_t isr;                  /**< <tt>\b 0x20:</tt> SCN ISR Register */
    __IO uint32_t txr;                  /**< <tt>\b 0x24:</tt> SCN TXR Register */
    __IO uint32_t rxr;                  /**< <tt>\b 0x28:</tt> SCN RXR Register */
    __IO uint32_t ccr;                  /**< <tt>\b 0x2C:</tt> SCN CCR Register */
} mxc_scn_regs_t;

/* Register offsets for module SCN */
/**
 * @ingroup    scn_registers
 * @defgroup   SCN_Register_Offsets Register Offsets
 * @brief      SCN Peripheral Register Offsets from the SCN Base Peripheral Address.
 * @{
 */
#define MXC_R_SCN_CR                       ((uint32_t)0x00000000UL) /**< Offset from SCN Base Address: <tt> 0x0000</tt> */
#define MXC_R_SCN_SR                       ((uint32_t)0x00000004UL) /**< Offset from SCN Base Address: <tt> 0x0004</tt> */
#define MXC_R_SCN_PN                       ((uint32_t)0x00000008UL) /**< Offset from SCN Base Address: <tt> 0x0008</tt> */
#define MXC_R_SCN_ETUR                     ((uint32_t)0x0000000CUL) /**< Offset from SCN Base Address: <tt> 0x000C</tt> */
#define MXC_R_SCN_GTR                      ((uint32_t)0x00000010UL) /**< Offset from SCN Base Address: <tt> 0x0010</tt> */
#define MXC_R_SCN_WT0R                     ((uint32_t)0x00000014UL) /**< Offset from SCN Base Address: <tt> 0x0014</tt> */
#define MXC_R_SCN_WT1R                     ((uint32_t)0x00000018UL) /**< Offset from SCN Base Address: <tt> 0x0018</tt> */
#define MXC_R_SCN_IER                      ((uint32_t)0x0000001CUL) /**< Offset from SCN Base Address: <tt> 0x001C</tt> */
#define MXC_R_SCN_ISR                      ((uint32_t)0x00000020UL) /**< Offset from SCN Base Address: <tt> 0x0020</tt> */
#define MXC_R_SCN_TXR                      ((uint32_t)0x00000024UL) /**< Offset from SCN Base Address: <tt> 0x0024</tt> */
#define MXC_R_SCN_RXR                      ((uint32_t)0x00000028UL) /**< Offset from SCN Base Address: <tt> 0x0028</tt> */
#define MXC_R_SCN_CCR                      ((uint32_t)0x0000002CUL) /**< Offset from SCN Base Address: <tt> 0x002C</tt> */
/**@} end of group scn_registers */

/**
 * @ingroup  scn_registers
 * @defgroup SCN_CR SCN_CR
 * @brief    Control Register.
 * @{
 */
#define MXC_F_SCN_CR_CONV_POS                          0 /**< CR_CONV Position */
#define MXC_F_SCN_CR_CONV                              ((uint32_t)(0x1UL << MXC_F_SCN_CR_CONV_POS)) /**< CR_CONV Mask */

#define MXC_F_SCN_CR_CREP_POS                          1 /**< CR_CREP Position */
#define MXC_F_SCN_CR_CREP                              ((uint32_t)(0x1UL << MXC_F_SCN_CR_CREP_POS)) /**< CR_CREP Mask */

#define MXC_F_SCN_CR_WTEN_POS                          2 /**< CR_WTEN Position */
#define MXC_F_SCN_CR_WTEN                              ((uint32_t)(0x1UL << MXC_F_SCN_CR_WTEN_POS)) /**< CR_WTEN Mask */

#define MXC_F_SCN_CR_UART_POS                          3 /**< CR_UART Position */
#define MXC_F_SCN_CR_UART                              ((uint32_t)(0x1UL << MXC_F_SCN_CR_UART_POS)) /**< CR_UART Mask */

#define MXC_F_SCN_CR_CCEN_POS                          4 /**< CR_CCEN Position */
#define MXC_F_SCN_CR_CCEN                              ((uint32_t)(0x1UL << MXC_F_SCN_CR_CCEN_POS)) /**< CR_CCEN Mask */

#define MXC_F_SCN_CR_RXFLUSH_POS                       5 /**< CR_RXFLUSH Position */
#define MXC_F_SCN_CR_RXFLUSH                           ((uint32_t)(0x1UL << MXC_F_SCN_CR_RXFLUSH_POS)) /**< CR_RXFLUSH Mask */

#define MXC_F_SCN_CR_TXFLUSH_POS                       6 /**< CR_TXFLUSH Position */
#define MXC_F_SCN_CR_TXFLUSH                           ((uint32_t)(0x1UL << MXC_F_SCN_CR_TXFLUSH_POS)) /**< CR_TXFLUSH Mask */

#define MXC_F_SCN_CR_RXTHD_POS                         8 /**< CR_RXTHD Position */
#define MXC_F_SCN_CR_RXTHD                             ((uint32_t)(0xFUL << MXC_F_SCN_CR_RXTHD_POS)) /**< CR_RXTHD Mask */

#define MXC_F_SCN_CR_TXTHD_POS                         12 /**< CR_TXTHD Position */
#define MXC_F_SCN_CR_TXTHD                             ((uint32_t)(0xFUL << MXC_F_SCN_CR_TXTHD_POS)) /**< CR_TXTHD Mask */

/**@} end of group SCN_CR_Register */

/**
 * @ingroup  scn_registers
 * @defgroup SCN_SR SCN_SR
 * @brief    Status Register.
 * @{
 */
#define MXC_F_SCN_SR_PAR_POS                           0 /**< SR_PAR Position */
#define MXC_F_SCN_SR_PAR                               ((uint32_t)(0x1UL << MXC_F_SCN_SR_PAR_POS)) /**< SR_PAR Mask */

#define MXC_F_SCN_SR_WTOV_POS                          1 /**< SR_WTOV Position */
#define MXC_F_SCN_SR_WTOV                              ((uint32_t)(0x1UL << MXC_F_SCN_SR_WTOV_POS)) /**< SR_WTOV Mask */

#define MXC_F_SCN_SR_CCOV_POS                          2 /**< SR_CCOV Position */
#define MXC_F_SCN_SR_CCOV                              ((uint32_t)(0x1UL << MXC_F_SCN_SR_CCOV_POS)) /**< SR_CCOV Mask */

#define MXC_F_SCN_SR_TXCF_POS                          3 /**< SR_TXCF Position */
#define MXC_F_SCN_SR_TXCF                              ((uint32_t)(0x1UL << MXC_F_SCN_SR_TXCF_POS)) /**< SR_TXCF Mask */

#define MXC_F_SCN_SR_RXEMPTY_POS                       4 /**< SR_RXEMPTY Position */
#define MXC_F_SCN_SR_RXEMPTY                           ((uint32_t)(0x1UL << MXC_F_SCN_SR_RXEMPTY_POS)) /**< SR_RXEMPTY Mask */

#define MXC_F_SCN_SR_RXFULL_POS                        5 /**< SR_RXFULL Position */
#define MXC_F_SCN_SR_RXFULL                            ((uint32_t)(0x1UL << MXC_F_SCN_SR_RXFULL_POS)) /**< SR_RXFULL Mask */

#define MXC_F_SCN_SR_TXEMPTY_POS                       6 /**< SR_TXEMPTY Position */
#define MXC_F_SCN_SR_TXEMPTY                           ((uint32_t)(0x1UL << MXC_F_SCN_SR_TXEMPTY_POS)) /**< SR_TXEMPTY Mask */

#define MXC_F_SCN_SR_TXFULL_POS                        7 /**< SR_TXFULL Position */
#define MXC_F_SCN_SR_TXFULL                            ((uint32_t)(0x1UL << MXC_F_SCN_SR_TXFULL_POS)) /**< SR_TXFULL Mask */

#define MXC_F_SCN_SR_RXELT_POS                         8 /**< SR_RXELT Position */
#define MXC_F_SCN_SR_RXELT                             ((uint32_t)(0xFUL << MXC_F_SCN_SR_RXELT_POS)) /**< SR_RXELT Mask */

#define MXC_F_SCN_SR_TXELT_POS                         12 /**< SR_TXELT Position */
#define MXC_F_SCN_SR_TXELT                             ((uint32_t)(0xFUL << MXC_F_SCN_SR_TXELT_POS)) /**< SR_TXELT Mask */

/**@} end of group SCN_SR_Register */

/**
 * @ingroup  scn_registers
 * @defgroup SCN_PN SCN_PN
 * @brief    Pin Register,
 * @{
 */
#define MXC_F_SCN_PN_CRDRST_POS                        0 /**< PN_CRDRST Position */
#define MXC_F_SCN_PN_CRDRST                            ((uint32_t)(0x1UL << MXC_F_SCN_PN_CRDRST_POS)) /**< PN_CRDRST Mask */

#define MXC_F_SCN_PN_CRDCLK_POS                        1 /**< PN_CRDCLK Position */
#define MXC_F_SCN_PN_CRDCLK                            ((uint32_t)(0x1UL << MXC_F_SCN_PN_CRDCLK_POS)) /**< PN_CRDCLK Mask */

#define MXC_F_SCN_PN_CRDIO_POS                         2 /**< PN_CRDIO Position */
#define MXC_F_SCN_PN_CRDIO                             ((uint32_t)(0x1UL << MXC_F_SCN_PN_CRDIO_POS)) /**< PN_CRDIO Mask */

#define MXC_F_SCN_PN_CRDC4_POS                         3 /**< PN_CRDC4 Position */
#define MXC_F_SCN_PN_CRDC4                             ((uint32_t)(0x1UL << MXC_F_SCN_PN_CRDC4_POS)) /**< PN_CRDC4 Mask */

#define MXC_F_SCN_PN_CRDC8_POS                         4 /**< PN_CRDC8 Position */
#define MXC_F_SCN_PN_CRDC8                             ((uint32_t)(0x1UL << MXC_F_SCN_PN_CRDC8_POS)) /**< PN_CRDC8 Mask */

#define MXC_F_SCN_PN_CLKSEL_POS                        5 /**< PN_CLKSEL Position */
#define MXC_F_SCN_PN_CLKSEL                            ((uint32_t)(0x1UL << MXC_F_SCN_PN_CLKSEL_POS)) /**< PN_CLKSEL Mask */

/**@} end of group SCN_PN_Register */

/**
 * @ingroup  scn_registers
 * @defgroup SCN_ETUR SCN_ETUR
 * @brief    ETU Register.
 * @{
 */
#define MXC_F_SCN_ETUR_ETU_POS                         0 /**< ETUR_ETU Position */
#define MXC_F_SCN_ETUR_ETU                             ((uint32_t)(0x7FFFUL << MXC_F_SCN_ETUR_ETU_POS)) /**< ETUR_ETU Mask */

#define MXC_F_SCN_ETUR_COMP_POS                        15 /**< ETUR_COMP Position */
#define MXC_F_SCN_ETUR_COMP                            ((uint32_t)(0x1UL << MXC_F_SCN_ETUR_COMP_POS)) /**< ETUR_COMP Mask */

#define MXC_F_SCN_ETUR_HALF_POS                        16 /**< ETUR_HALF Position */
#define MXC_F_SCN_ETUR_HALF                            ((uint32_t)(0x1UL << MXC_F_SCN_ETUR_HALF_POS)) /**< ETUR_HALF Mask */

/**@} end of group SCN_ETUR_Register */

/**
 * @ingroup  scn_registers
 * @defgroup SCN_GTR SCN_GTR
 * @brief    Guard Time Register.
 * @{
 */
#define MXC_F_SCN_GTR_GT_POS                           0 /**< GTR_GT Position */
#define MXC_F_SCN_GTR_GT                               ((uint32_t)(0xFFFFUL << MXC_F_SCN_GTR_GT_POS)) /**< GTR_GT Mask */

/**@} end of group SCN_GTR_Register */

/**
 * @ingroup  scn_registers
 * @defgroup SCN_WT0R SCN_WT0R
 * @brief    Waiting Time 0 Register.
 * @{
 */
#define MXC_F_SCN_WT0R_WT_POS                          0 /**< WT0R_WT Position */
#define MXC_F_SCN_WT0R_WT                              ((uint32_t)(0xFFFFFFFFUL << MXC_F_SCN_WT0R_WT_POS)) /**< WT0R_WT Mask */

/**@} end of group SCN_WT0R_Register */

/**
 * @ingroup  scn_registers
 * @defgroup SCN_WT1R SCN_WT1R
 * @brief    Waiting Time 1 Register.
 * @{
 */
#define MXC_F_SCN_WT1R_WT_POS                          0 /**< WT1R_WT Position */
#define MXC_F_SCN_WT1R_WT                              ((uint32_t)(0xFFUL << MXC_F_SCN_WT1R_WT_POS)) /**< WT1R_WT Mask */

/**@} end of group SCN_WT1R_Register */

/**
 * @ingroup  scn_registers
 * @defgroup SCN_IER SCN_IER
 * @brief    Interrupt Enable Register.
 * @{
 */
#define MXC_F_SCN_IER_PARIE_POS                        0 /**< IER_PARIE Position */
#define MXC_F_SCN_IER_PARIE                            ((uint32_t)(0x1UL << MXC_F_SCN_IER_PARIE_POS)) /**< IER_PARIE Mask */

#define MXC_F_SCN_IER_WTIE_POS                         1 /**< IER_WTIE Position */
#define MXC_F_SCN_IER_WTIE                             ((uint32_t)(0x1UL << MXC_F_SCN_IER_WTIE_POS)) /**< IER_WTIE Mask */

#define MXC_F_SCN_IER_CTIE_POS                         2 /**< IER_CTIE Position */
#define MXC_F_SCN_IER_CTIE                             ((uint32_t)(0x1UL << MXC_F_SCN_IER_CTIE_POS)) /**< IER_CTIE Mask */

#define MXC_F_SCN_IER_TCIE_POS                         3 /**< IER_TCIE Position */
#define MXC_F_SCN_IER_TCIE                             ((uint32_t)(0x1UL << MXC_F_SCN_IER_TCIE_POS)) /**< IER_TCIE Mask */

#define MXC_F_SCN_IER_RXEIE_POS                        4 /**< IER_RXEIE Position */
#define MXC_F_SCN_IER_RXEIE                            ((uint32_t)(0x1UL << MXC_F_SCN_IER_RXEIE_POS)) /**< IER_RXEIE Mask */

#define MXC_F_SCN_IER_RXTIE_POS                        5 /**< IER_RXTIE Position */
#define MXC_F_SCN_IER_RXTIE                            ((uint32_t)(0x1UL << MXC_F_SCN_IER_RXTIE_POS)) /**< IER_RXTIE Mask */

#define MXC_F_SCN_IER_RXFIE_POS                        6 /**< IER_RXFIE Position */
#define MXC_F_SCN_IER_RXFIE                            ((uint32_t)(0x1UL << MXC_F_SCN_IER_RXFIE_POS)) /**< IER_RXFIE Mask */

#define MXC_F_SCN_IER_TXEIE_POS                        7 /**< IER_TXEIE Position */
#define MXC_F_SCN_IER_TXEIE                            ((uint32_t)(0x1UL << MXC_F_SCN_IER_TXEIE_POS)) /**< IER_TXEIE Mask */

#define MXC_F_SCN_IER_TXTIE_POS                        8 /**< IER_TXTIE Position */
#define MXC_F_SCN_IER_TXTIE                            ((uint32_t)(0x1UL << MXC_F_SCN_IER_TXTIE_POS)) /**< IER_TXTIE Mask */

/**@} end of group SCN_IER_Register */

/**
 * @ingroup  scn_registers
 * @defgroup SCN_ISR SCN_ISR
 * @brief    Interrupt Status Register.
 * @{
 */
#define MXC_F_SCN_ISR_PARIS_POS                        0 /**< ISR_PARIS Position */
#define MXC_F_SCN_ISR_PARIS                            ((uint32_t)(0x1UL << MXC_F_SCN_ISR_PARIS_POS)) /**< ISR_PARIS Mask */

#define MXC_F_SCN_ISR_WTIS_POS                         1 /**< ISR_WTIS Position */
#define MXC_F_SCN_ISR_WTIS                             ((uint32_t)(0x1UL << MXC_F_SCN_ISR_WTIS_POS)) /**< ISR_WTIS Mask */

#define MXC_F_SCN_ISR_CTIS_POS                         2 /**< ISR_CTIS Position */
#define MXC_F_SCN_ISR_CTIS                             ((uint32_t)(0x1UL << MXC_F_SCN_ISR_CTIS_POS)) /**< ISR_CTIS Mask */

#define MXC_F_SCN_ISR_TCIS_POS                         3 /**< ISR_TCIS Position */
#define MXC_F_SCN_ISR_TCIS                             ((uint32_t)(0x1UL << MXC_F_SCN_ISR_TCIS_POS)) /**< ISR_TCIS Mask */

#define MXC_F_SCN_ISR_RXEIS_POS                        4 /**< ISR_RXEIS Position */
#define MXC_F_SCN_ISR_RXEIS                            ((uint32_t)(0x1UL << MXC_F_SCN_ISR_RXEIS_POS)) /**< ISR_RXEIS Mask */

#define MXC_F_SCN_ISR_RXTIS_POS                        5 /**< ISR_RXTIS Position */
#define MXC_F_SCN_ISR_RXTIS                            ((uint32_t)(0x1UL << MXC_F_SCN_ISR_RXTIS_POS)) /**< ISR_RXTIS Mask */

#define MXC_F_SCN_ISR_RXFIS_POS                        6 /**< ISR_RXFIS Position */
#define MXC_F_SCN_ISR_RXFIS                            ((uint32_t)(0x1UL << MXC_F_SCN_ISR_RXFIS_POS)) /**< ISR_RXFIS Mask */

#define MXC_F_SCN_ISR_TXEIS_POS                        7 /**< ISR_TXEIS Position */
#define MXC_F_SCN_ISR_TXEIS                            ((uint32_t)(0x1UL << MXC_F_SCN_ISR_TXEIS_POS)) /**< ISR_TXEIS Mask */

#define MXC_F_SCN_ISR_TXTIS_POS                        8 /**< ISR_TXTIS Position */
#define MXC_F_SCN_ISR_TXTIS                            ((uint32_t)(0x1UL << MXC_F_SCN_ISR_TXTIS_POS)) /**< ISR_TXTIS Mask */

/**@} end of group SCN_ISR_Register */

/**
 * @ingroup  scn_registers
 * @defgroup SCN_TXR SCN_TXR
 * @brief    Transmit Register.
 * @{
 */
#define MXC_F_SCN_TXR_DATA_POS                         0 /**< TXR_DATA Position */
#define MXC_F_SCN_TXR_DATA                             ((uint32_t)(0xFFUL << MXC_F_SCN_TXR_DATA_POS)) /**< TXR_DATA Mask */

/**@} end of group SCN_TXR_Register */

/**
 * @ingroup  scn_registers
 * @defgroup SCN_RXR SCN_RXR
 * @brief    Receive Register.
 * @{
 */
#define MXC_F_SCN_RXR_DATA_POS                         0 /**< RXR_DATA Position */
#define MXC_F_SCN_RXR_DATA                             ((uint32_t)(0xFFUL << MXC_F_SCN_RXR_DATA_POS)) /**< RXR_DATA Mask */

#define MXC_F_SCN_RXR_PARER_POS                        8 /**< RXR_PARER Position */
#define MXC_F_SCN_RXR_PARER                            ((uint32_t)(0x1UL << MXC_F_SCN_RXR_PARER_POS)) /**< RXR_PARER Mask */

/**@} end of group SCN_RXR_Register */

/**
 * @ingroup  scn_registers
 * @defgroup SCN_CCR SCN_CCR
 * @brief    Clock Counter Register,
 * @{
 */
#define MXC_F_SCN_CCR_CCYC_POS                         0 /**< CCR_CCYC Position */
#define MXC_F_SCN_CCR_CCYC                             ((uint32_t)(0xFFFFFFUL << MXC_F_SCN_CCR_CCYC_POS)) /**< CCR_CCYC Mask */

#define MXC_F_SCN_CCR_MAN_POS                          31 /**< CCR_MAN Position */
#define MXC_F_SCN_CCR_MAN                              ((uint32_t)(0x1UL << MXC_F_SCN_CCR_MAN_POS)) /**< CCR_MAN Mask */

/**@} end of group SCN_CCR_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32570_INCLUDE_SCN_REGS_H_
