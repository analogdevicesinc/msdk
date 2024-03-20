/**
 * @file    spimss_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the SPIMSS Peripheral Module.
 * @note    This file is @generated.
 * @ingroup spimss_registers
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32650_INCLUDE_SPIMSS_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32650_INCLUDE_SPIMSS_REGS_H_

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
 * @ingroup     spimss
 * @defgroup    spimss_registers SPIMSS_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the SPIMSS Peripheral Module.
 * @details     Serial Peripheral Interface.
 */

/**
 * @ingroup spimss_registers
 * Structure type to access the SPIMSS Registers.
 */
typedef struct {
    __IO uint32_t data;                 /**< <tt>\b 0x00:</tt> SPIMSS DATA Register */
    __IO uint32_t ctrl;                 /**< <tt>\b 0x04:</tt> SPIMSS CTRL Register */
    __IO uint32_t int_fl;               /**< <tt>\b 0x08:</tt> SPIMSS INT_FL Register */
    __IO uint32_t mod;                  /**< <tt>\b 0x0C:</tt> SPIMSS MOD Register */
    __R  uint32_t rsv_0x10;
    __IO uint32_t brg;                  /**< <tt>\b 0x14:</tt> SPIMSS BRG Register */
    __IO uint32_t dma;                  /**< <tt>\b 0x18:</tt> SPIMSS DMA Register */
    __IO uint32_t i2s_ctrl;             /**< <tt>\b 0x1C:</tt> SPIMSS I2S_CTRL Register */
} mxc_spimss_regs_t;

/* Register offsets for module SPIMSS */
/**
 * @ingroup    spimss_registers
 * @defgroup   SPIMSS_Register_Offsets Register Offsets
 * @brief      SPIMSS Peripheral Register Offsets from the SPIMSS Base Peripheral Address.
 * @{
 */
#define MXC_R_SPIMSS_DATA                  ((uint32_t)0x00000000UL) /**< Offset from SPIMSS Base Address: <tt> 0x0000</tt> */
#define MXC_R_SPIMSS_CTRL                  ((uint32_t)0x00000004UL) /**< Offset from SPIMSS Base Address: <tt> 0x0004</tt> */
#define MXC_R_SPIMSS_INT_FL                ((uint32_t)0x00000008UL) /**< Offset from SPIMSS Base Address: <tt> 0x0008</tt> */
#define MXC_R_SPIMSS_MOD                   ((uint32_t)0x0000000CUL) /**< Offset from SPIMSS Base Address: <tt> 0x000C</tt> */
#define MXC_R_SPIMSS_BRG                   ((uint32_t)0x00000014UL) /**< Offset from SPIMSS Base Address: <tt> 0x0014</tt> */
#define MXC_R_SPIMSS_DMA                   ((uint32_t)0x00000018UL) /**< Offset from SPIMSS Base Address: <tt> 0x0018</tt> */
#define MXC_R_SPIMSS_I2S_CTRL              ((uint32_t)0x0000001CUL) /**< Offset from SPIMSS Base Address: <tt> 0x001C</tt> */
/**@} end of group spimss_registers */

/**
 * @ingroup  spimss_registers
 * @defgroup SPIMSS_DATA SPIMSS_DATA
 * @brief    SPI 16-bit Data Access
 * @{
 */
#define MXC_F_SPIMSS_DATA_DATA_POS                     0 /**< DATA_DATA Position */
#define MXC_F_SPIMSS_DATA_DATA                         ((uint32_t)(0xFFFFUL << MXC_F_SPIMSS_DATA_DATA_POS)) /**< DATA_DATA Mask */

/**@} end of group SPIMSS_DATA_Register */

/**
 * @ingroup  spimss_registers
 * @defgroup SPIMSS_CTRL SPIMSS_CTRL
 * @brief    SPI Control Register.
 * @{
 */
#define MXC_F_SPIMSS_CTRL_START_POS                    0 /**< CTRL_START Position */
#define MXC_F_SPIMSS_CTRL_START                        ((uint32_t)(0x1UL << MXC_F_SPIMSS_CTRL_START_POS)) /**< CTRL_START Mask */
#define MXC_V_SPIMSS_CTRL_START_STOP                   ((uint32_t)0x0UL) /**< CTRL_START_STOP Value */
#define MXC_S_SPIMSS_CTRL_START_STOP                   (MXC_V_SPIMSS_CTRL_START_STOP << MXC_F_SPIMSS_CTRL_START_POS) /**< CTRL_START_STOP Setting */
#define MXC_V_SPIMSS_CTRL_START_START                  ((uint32_t)0x1UL) /**< CTRL_START_START Value */
#define MXC_S_SPIMSS_CTRL_START_START                  (MXC_V_SPIMSS_CTRL_START_START << MXC_F_SPIMSS_CTRL_START_POS) /**< CTRL_START_START Setting */

#define MXC_F_SPIMSS_CTRL_MMEN_POS                     1 /**< CTRL_MMEN Position */
#define MXC_F_SPIMSS_CTRL_MMEN                         ((uint32_t)(0x1UL << MXC_F_SPIMSS_CTRL_MMEN_POS)) /**< CTRL_MMEN Mask */
#define MXC_V_SPIMSS_CTRL_MMEN_SLAVE                   ((uint32_t)0x0UL) /**< CTRL_MMEN_SLAVE Value */
#define MXC_S_SPIMSS_CTRL_MMEN_SLAVE                   (MXC_V_SPIMSS_CTRL_MMEN_SLAVE << MXC_F_SPIMSS_CTRL_MMEN_POS) /**< CTRL_MMEN_SLAVE Setting */
#define MXC_V_SPIMSS_CTRL_MMEN_MASTER                  ((uint32_t)0x1UL) /**< CTRL_MMEN_MASTER Value */
#define MXC_S_SPIMSS_CTRL_MMEN_MASTER                  (MXC_V_SPIMSS_CTRL_MMEN_MASTER << MXC_F_SPIMSS_CTRL_MMEN_POS) /**< CTRL_MMEN_MASTER Setting */

#define MXC_F_SPIMSS_CTRL_OD_OUT_EN_POS                2 /**< CTRL_OD_OUT_EN Position */
#define MXC_F_SPIMSS_CTRL_OD_OUT_EN                    ((uint32_t)(0x1UL << MXC_F_SPIMSS_CTRL_OD_OUT_EN_POS)) /**< CTRL_OD_OUT_EN Mask */
#define MXC_V_SPIMSS_CTRL_OD_OUT_EN_DIS                ((uint32_t)0x0UL) /**< CTRL_OD_OUT_EN_DIS Value */
#define MXC_S_SPIMSS_CTRL_OD_OUT_EN_DIS                (MXC_V_SPIMSS_CTRL_OD_OUT_EN_DIS << MXC_F_SPIMSS_CTRL_OD_OUT_EN_POS) /**< CTRL_OD_OUT_EN_DIS Setting */
#define MXC_V_SPIMSS_CTRL_OD_OUT_EN_EN                 ((uint32_t)0x1UL) /**< CTRL_OD_OUT_EN_EN Value */
#define MXC_S_SPIMSS_CTRL_OD_OUT_EN_EN                 (MXC_V_SPIMSS_CTRL_OD_OUT_EN_EN << MXC_F_SPIMSS_CTRL_OD_OUT_EN_POS) /**< CTRL_OD_OUT_EN_EN Setting */

#define MXC_F_SPIMSS_CTRL_CLKPOL_POS                   3 /**< CTRL_CLKPOL Position */
#define MXC_F_SPIMSS_CTRL_CLKPOL                       ((uint32_t)(0x1UL << MXC_F_SPIMSS_CTRL_CLKPOL_POS)) /**< CTRL_CLKPOL Mask */
#define MXC_V_SPIMSS_CTRL_CLKPOL_IDLELO                ((uint32_t)0x0UL) /**< CTRL_CLKPOL_IDLELO Value */
#define MXC_S_SPIMSS_CTRL_CLKPOL_IDLELO                (MXC_V_SPIMSS_CTRL_CLKPOL_IDLELO << MXC_F_SPIMSS_CTRL_CLKPOL_POS) /**< CTRL_CLKPOL_IDLELO Setting */
#define MXC_V_SPIMSS_CTRL_CLKPOL_IDLEHI                ((uint32_t)0x1UL) /**< CTRL_CLKPOL_IDLEHI Value */
#define MXC_S_SPIMSS_CTRL_CLKPOL_IDLEHI                (MXC_V_SPIMSS_CTRL_CLKPOL_IDLEHI << MXC_F_SPIMSS_CTRL_CLKPOL_POS) /**< CTRL_CLKPOL_IDLEHI Setting */

#define MXC_F_SPIMSS_CTRL_PHASE_POS                    4 /**< CTRL_PHASE Position */
#define MXC_F_SPIMSS_CTRL_PHASE                        ((uint32_t)(0x1UL << MXC_F_SPIMSS_CTRL_PHASE_POS)) /**< CTRL_PHASE Mask */
#define MXC_V_SPIMSS_CTRL_PHASE_ACTIVEEDGE             ((uint32_t)0x0UL) /**< CTRL_PHASE_ACTIVEEDGE Value */
#define MXC_S_SPIMSS_CTRL_PHASE_ACTIVEEDGE             (MXC_V_SPIMSS_CTRL_PHASE_ACTIVEEDGE << MXC_F_SPIMSS_CTRL_PHASE_POS) /**< CTRL_PHASE_ACTIVEEDGE Setting */
#define MXC_V_SPIMSS_CTRL_PHASE_INACTIVEEDGE           ((uint32_t)0x1UL) /**< CTRL_PHASE_INACTIVEEDGE Value */
#define MXC_S_SPIMSS_CTRL_PHASE_INACTIVEEDGE           (MXC_V_SPIMSS_CTRL_PHASE_INACTIVEEDGE << MXC_F_SPIMSS_CTRL_PHASE_POS) /**< CTRL_PHASE_INACTIVEEDGE Setting */

#define MXC_F_SPIMSS_CTRL_BIRQ_POS                     5 /**< CTRL_BIRQ Position */
#define MXC_F_SPIMSS_CTRL_BIRQ                         ((uint32_t)(0x1UL << MXC_F_SPIMSS_CTRL_BIRQ_POS)) /**< CTRL_BIRQ Mask */
#define MXC_V_SPIMSS_CTRL_BIRQ_DIS                     ((uint32_t)0x0UL) /**< CTRL_BIRQ_DIS Value */
#define MXC_S_SPIMSS_CTRL_BIRQ_DIS                     (MXC_V_SPIMSS_CTRL_BIRQ_DIS << MXC_F_SPIMSS_CTRL_BIRQ_POS) /**< CTRL_BIRQ_DIS Setting */
#define MXC_V_SPIMSS_CTRL_BIRQ_EN                      ((uint32_t)0x1UL) /**< CTRL_BIRQ_EN Value */
#define MXC_S_SPIMSS_CTRL_BIRQ_EN                      (MXC_V_SPIMSS_CTRL_BIRQ_EN << MXC_F_SPIMSS_CTRL_BIRQ_POS) /**< CTRL_BIRQ_EN Setting */

#define MXC_F_SPIMSS_CTRL_STR_POS                      6 /**< CTRL_STR Position */
#define MXC_F_SPIMSS_CTRL_STR                          ((uint32_t)(0x1UL << MXC_F_SPIMSS_CTRL_STR_POS)) /**< CTRL_STR Mask */
#define MXC_V_SPIMSS_CTRL_STR_COMPLETE                 ((uint32_t)0x0UL) /**< CTRL_STR_COMPLETE Value */
#define MXC_S_SPIMSS_CTRL_STR_COMPLETE                 (MXC_V_SPIMSS_CTRL_STR_COMPLETE << MXC_F_SPIMSS_CTRL_STR_POS) /**< CTRL_STR_COMPLETE Setting */
#define MXC_V_SPIMSS_CTRL_STR_START                    ((uint32_t)0x1UL) /**< CTRL_STR_START Value */
#define MXC_S_SPIMSS_CTRL_STR_START                    (MXC_V_SPIMSS_CTRL_STR_START << MXC_F_SPIMSS_CTRL_STR_POS) /**< CTRL_STR_START Setting */

#define MXC_F_SPIMSS_CTRL_IRQE_POS                     7 /**< CTRL_IRQE Position */
#define MXC_F_SPIMSS_CTRL_IRQE                         ((uint32_t)(0x1UL << MXC_F_SPIMSS_CTRL_IRQE_POS)) /**< CTRL_IRQE Mask */
#define MXC_V_SPIMSS_CTRL_IRQE_DIS                     ((uint32_t)0x0UL) /**< CTRL_IRQE_DIS Value */
#define MXC_S_SPIMSS_CTRL_IRQE_DIS                     (MXC_V_SPIMSS_CTRL_IRQE_DIS << MXC_F_SPIMSS_CTRL_IRQE_POS) /**< CTRL_IRQE_DIS Setting */
#define MXC_V_SPIMSS_CTRL_IRQE_EN                      ((uint32_t)0x1UL) /**< CTRL_IRQE_EN Value */
#define MXC_S_SPIMSS_CTRL_IRQE_EN                      (MXC_V_SPIMSS_CTRL_IRQE_EN << MXC_F_SPIMSS_CTRL_IRQE_POS) /**< CTRL_IRQE_EN Setting */

/**@} end of group SPIMSS_CTRL_Register */

/**
 * @ingroup  spimss_registers
 * @defgroup SPIMSS_INT_FL SPIMSS_INT_FL
 * @brief    SPI Interrupt Flag Register.
 * @{
 */
#define MXC_F_SPIMSS_INT_FL_SLAS_POS                   0 /**< INT_FL_SLAS Position */
#define MXC_F_SPIMSS_INT_FL_SLAS                       ((uint32_t)(0x1UL << MXC_F_SPIMSS_INT_FL_SLAS_POS)) /**< INT_FL_SLAS Mask */
#define MXC_V_SPIMSS_INT_FL_SLAS_SELECTED              ((uint32_t)0x0UL) /**< INT_FL_SLAS_SELECTED Value */
#define MXC_S_SPIMSS_INT_FL_SLAS_SELECTED              (MXC_V_SPIMSS_INT_FL_SLAS_SELECTED << MXC_F_SPIMSS_INT_FL_SLAS_POS) /**< INT_FL_SLAS_SELECTED Setting */
#define MXC_V_SPIMSS_INT_FL_SLAS_NOTSELECTED           ((uint32_t)0x1UL) /**< INT_FL_SLAS_NOTSELECTED Value */
#define MXC_S_SPIMSS_INT_FL_SLAS_NOTSELECTED           (MXC_V_SPIMSS_INT_FL_SLAS_NOTSELECTED << MXC_F_SPIMSS_INT_FL_SLAS_POS) /**< INT_FL_SLAS_NOTSELECTED Setting */

#define MXC_F_SPIMSS_INT_FL_TXST_POS                   1 /**< INT_FL_TXST Position */
#define MXC_F_SPIMSS_INT_FL_TXST                       ((uint32_t)(0x1UL << MXC_F_SPIMSS_INT_FL_TXST_POS)) /**< INT_FL_TXST Mask */
#define MXC_V_SPIMSS_INT_FL_TXST_IDLE                  ((uint32_t)0x0UL) /**< INT_FL_TXST_IDLE Value */
#define MXC_S_SPIMSS_INT_FL_TXST_IDLE                  (MXC_V_SPIMSS_INT_FL_TXST_IDLE << MXC_F_SPIMSS_INT_FL_TXST_POS) /**< INT_FL_TXST_IDLE Setting */
#define MXC_V_SPIMSS_INT_FL_TXST_BUSY                  ((uint32_t)0x1UL) /**< INT_FL_TXST_BUSY Value */
#define MXC_S_SPIMSS_INT_FL_TXST_BUSY                  (MXC_V_SPIMSS_INT_FL_TXST_BUSY << MXC_F_SPIMSS_INT_FL_TXST_POS) /**< INT_FL_TXST_BUSY Setting */

#define MXC_F_SPIMSS_INT_FL_TUND_POS                   2 /**< INT_FL_TUND Position */
#define MXC_F_SPIMSS_INT_FL_TUND                       ((uint32_t)(0x1UL << MXC_F_SPIMSS_INT_FL_TUND_POS)) /**< INT_FL_TUND Mask */
#define MXC_V_SPIMSS_INT_FL_TUND_NOEVENT               ((uint32_t)0x0UL) /**< INT_FL_TUND_NOEVENT Value */
#define MXC_S_SPIMSS_INT_FL_TUND_NOEVENT               (MXC_V_SPIMSS_INT_FL_TUND_NOEVENT << MXC_F_SPIMSS_INT_FL_TUND_POS) /**< INT_FL_TUND_NOEVENT Setting */
#define MXC_V_SPIMSS_INT_FL_TUND_UNDERRUN              ((uint32_t)0x1UL) /**< INT_FL_TUND_UNDERRUN Value */
#define MXC_S_SPIMSS_INT_FL_TUND_UNDERRUN              (MXC_V_SPIMSS_INT_FL_TUND_UNDERRUN << MXC_F_SPIMSS_INT_FL_TUND_POS) /**< INT_FL_TUND_UNDERRUN Setting */

#define MXC_F_SPIMSS_INT_FL_ROVR_POS                   3 /**< INT_FL_ROVR Position */
#define MXC_F_SPIMSS_INT_FL_ROVR                       ((uint32_t)(0x1UL << MXC_F_SPIMSS_INT_FL_ROVR_POS)) /**< INT_FL_ROVR Mask */
#define MXC_V_SPIMSS_INT_FL_ROVR_NOEVENT               ((uint32_t)0x0UL) /**< INT_FL_ROVR_NOEVENT Value */
#define MXC_S_SPIMSS_INT_FL_ROVR_NOEVENT               (MXC_V_SPIMSS_INT_FL_ROVR_NOEVENT << MXC_F_SPIMSS_INT_FL_ROVR_POS) /**< INT_FL_ROVR_NOEVENT Setting */
#define MXC_V_SPIMSS_INT_FL_ROVR_OVERRUN               ((uint32_t)0x1UL) /**< INT_FL_ROVR_OVERRUN Value */
#define MXC_S_SPIMSS_INT_FL_ROVR_OVERRUN               (MXC_V_SPIMSS_INT_FL_ROVR_OVERRUN << MXC_F_SPIMSS_INT_FL_ROVR_POS) /**< INT_FL_ROVR_OVERRUN Setting */

#define MXC_F_SPIMSS_INT_FL_ABT_POS                    4 /**< INT_FL_ABT Position */
#define MXC_F_SPIMSS_INT_FL_ABT                        ((uint32_t)(0x1UL << MXC_F_SPIMSS_INT_FL_ABT_POS)) /**< INT_FL_ABT Mask */
#define MXC_V_SPIMSS_INT_FL_ABT_NOEVENT                ((uint32_t)0x0UL) /**< INT_FL_ABT_NOEVENT Value */
#define MXC_S_SPIMSS_INT_FL_ABT_NOEVENT                (MXC_V_SPIMSS_INT_FL_ABT_NOEVENT << MXC_F_SPIMSS_INT_FL_ABT_POS) /**< INT_FL_ABT_NOEVENT Setting */
#define MXC_V_SPIMSS_INT_FL_ABT_ABORTED                ((uint32_t)0x1UL) /**< INT_FL_ABT_ABORTED Value */
#define MXC_S_SPIMSS_INT_FL_ABT_ABORTED                (MXC_V_SPIMSS_INT_FL_ABT_ABORTED << MXC_F_SPIMSS_INT_FL_ABT_POS) /**< INT_FL_ABT_ABORTED Setting */

#define MXC_F_SPIMSS_INT_FL_COL_POS                    5 /**< INT_FL_COL Position */
#define MXC_F_SPIMSS_INT_FL_COL                        ((uint32_t)(0x1UL << MXC_F_SPIMSS_INT_FL_COL_POS)) /**< INT_FL_COL Mask */
#define MXC_V_SPIMSS_INT_FL_COL_NOEVENT                ((uint32_t)0x0UL) /**< INT_FL_COL_NOEVENT Value */
#define MXC_S_SPIMSS_INT_FL_COL_NOEVENT                (MXC_V_SPIMSS_INT_FL_COL_NOEVENT << MXC_F_SPIMSS_INT_FL_COL_POS) /**< INT_FL_COL_NOEVENT Setting */
#define MXC_V_SPIMSS_INT_FL_COL_COLLISION              ((uint32_t)0x1UL) /**< INT_FL_COL_COLLISION Value */
#define MXC_S_SPIMSS_INT_FL_COL_COLLISION              (MXC_V_SPIMSS_INT_FL_COL_COLLISION << MXC_F_SPIMSS_INT_FL_COL_POS) /**< INT_FL_COL_COLLISION Setting */

#define MXC_F_SPIMSS_INT_FL_TOVR_POS                   6 /**< INT_FL_TOVR Position */
#define MXC_F_SPIMSS_INT_FL_TOVR                       ((uint32_t)(0x1UL << MXC_F_SPIMSS_INT_FL_TOVR_POS)) /**< INT_FL_TOVR Mask */
#define MXC_V_SPIMSS_INT_FL_TOVR_NOEVENT               ((uint32_t)0x0UL) /**< INT_FL_TOVR_NOEVENT Value */
#define MXC_S_SPIMSS_INT_FL_TOVR_NOEVENT               (MXC_V_SPIMSS_INT_FL_TOVR_NOEVENT << MXC_F_SPIMSS_INT_FL_TOVR_POS) /**< INT_FL_TOVR_NOEVENT Setting */
#define MXC_V_SPIMSS_INT_FL_TOVR_OVERRUN               ((uint32_t)0x1UL) /**< INT_FL_TOVR_OVERRUN Value */
#define MXC_S_SPIMSS_INT_FL_TOVR_OVERRUN               (MXC_V_SPIMSS_INT_FL_TOVR_OVERRUN << MXC_F_SPIMSS_INT_FL_TOVR_POS) /**< INT_FL_TOVR_OVERRUN Setting */

#define MXC_F_SPIMSS_INT_FL_IRQ_POS                    7 /**< INT_FL_IRQ Position */
#define MXC_F_SPIMSS_INT_FL_IRQ                        ((uint32_t)(0x1UL << MXC_F_SPIMSS_INT_FL_IRQ_POS)) /**< INT_FL_IRQ Mask */
#define MXC_V_SPIMSS_INT_FL_IRQ_INACTIVE               ((uint32_t)0x0UL) /**< INT_FL_IRQ_INACTIVE Value */
#define MXC_S_SPIMSS_INT_FL_IRQ_INACTIVE               (MXC_V_SPIMSS_INT_FL_IRQ_INACTIVE << MXC_F_SPIMSS_INT_FL_IRQ_POS) /**< INT_FL_IRQ_INACTIVE Setting */
#define MXC_V_SPIMSS_INT_FL_IRQ_PENDING                ((uint32_t)0x1UL) /**< INT_FL_IRQ_PENDING Value */
#define MXC_S_SPIMSS_INT_FL_IRQ_PENDING                (MXC_V_SPIMSS_INT_FL_IRQ_PENDING << MXC_F_SPIMSS_INT_FL_IRQ_POS) /**< INT_FL_IRQ_PENDING Setting */

/**@} end of group SPIMSS_INT_FL_Register */

/**
 * @ingroup  spimss_registers
 * @defgroup SPIMSS_MOD SPIMSS_MOD
 * @brief    SPI Mode Register.
 * @{
 */
#define MXC_F_SPIMSS_MOD_SSV_POS                       0 /**< MOD_SSV Position */
#define MXC_F_SPIMSS_MOD_SSV                           ((uint32_t)(0x1UL << MXC_F_SPIMSS_MOD_SSV_POS)) /**< MOD_SSV Mask */
#define MXC_V_SPIMSS_MOD_SSV_LO                        ((uint32_t)0x0UL) /**< MOD_SSV_LO Value */
#define MXC_S_SPIMSS_MOD_SSV_LO                        (MXC_V_SPIMSS_MOD_SSV_LO << MXC_F_SPIMSS_MOD_SSV_POS) /**< MOD_SSV_LO Setting */
#define MXC_V_SPIMSS_MOD_SSV_HI                        ((uint32_t)0x1UL) /**< MOD_SSV_HI Value */
#define MXC_S_SPIMSS_MOD_SSV_HI                        (MXC_V_SPIMSS_MOD_SSV_HI << MXC_F_SPIMSS_MOD_SSV_POS) /**< MOD_SSV_HI Setting */

#define MXC_F_SPIMSS_MOD_SSEL_POS                      1 /**< MOD_SSEL Position */
#define MXC_F_SPIMSS_MOD_SSEL                          ((uint32_t)(0x1UL << MXC_F_SPIMSS_MOD_SSEL_POS)) /**< MOD_SSEL Mask */
#define MXC_V_SPIMSS_MOD_SSEL_INPUT                    ((uint32_t)0x0UL) /**< MOD_SSEL_INPUT Value */
#define MXC_S_SPIMSS_MOD_SSEL_INPUT                    (MXC_V_SPIMSS_MOD_SSEL_INPUT << MXC_F_SPIMSS_MOD_SSEL_POS) /**< MOD_SSEL_INPUT Setting */
#define MXC_V_SPIMSS_MOD_SSEL_OUTPUT                   ((uint32_t)0x1UL) /**< MOD_SSEL_OUTPUT Value */
#define MXC_S_SPIMSS_MOD_SSEL_OUTPUT                   (MXC_V_SPIMSS_MOD_SSEL_OUTPUT << MXC_F_SPIMSS_MOD_SSEL_POS) /**< MOD_SSEL_OUTPUT Setting */

#define MXC_F_SPIMSS_MOD_NUMBITS_POS                   2 /**< MOD_NUMBITS Position */
#define MXC_F_SPIMSS_MOD_NUMBITS                       ((uint32_t)(0xFUL << MXC_F_SPIMSS_MOD_NUMBITS_POS)) /**< MOD_NUMBITS Mask */
#define MXC_V_SPIMSS_MOD_NUMBITS_16BITS                ((uint32_t)0x0UL) /**< MOD_NUMBITS_16BITS Value */
#define MXC_S_SPIMSS_MOD_NUMBITS_16BITS                (MXC_V_SPIMSS_MOD_NUMBITS_16BITS << MXC_F_SPIMSS_MOD_NUMBITS_POS) /**< MOD_NUMBITS_16BITS Setting */
#define MXC_V_SPIMSS_MOD_NUMBITS_1BITS                 ((uint32_t)0x1UL) /**< MOD_NUMBITS_1BITS Value */
#define MXC_S_SPIMSS_MOD_NUMBITS_1BITS                 (MXC_V_SPIMSS_MOD_NUMBITS_1BITS << MXC_F_SPIMSS_MOD_NUMBITS_POS) /**< MOD_NUMBITS_1BITS Setting */
#define MXC_V_SPIMSS_MOD_NUMBITS_2BITS                 ((uint32_t)0x2UL) /**< MOD_NUMBITS_2BITS Value */
#define MXC_S_SPIMSS_MOD_NUMBITS_2BITS                 (MXC_V_SPIMSS_MOD_NUMBITS_2BITS << MXC_F_SPIMSS_MOD_NUMBITS_POS) /**< MOD_NUMBITS_2BITS Setting */
#define MXC_V_SPIMSS_MOD_NUMBITS_3BITS                 ((uint32_t)0x3UL) /**< MOD_NUMBITS_3BITS Value */
#define MXC_S_SPIMSS_MOD_NUMBITS_3BITS                 (MXC_V_SPIMSS_MOD_NUMBITS_3BITS << MXC_F_SPIMSS_MOD_NUMBITS_POS) /**< MOD_NUMBITS_3BITS Setting */
#define MXC_V_SPIMSS_MOD_NUMBITS_4BITS                 ((uint32_t)0x4UL) /**< MOD_NUMBITS_4BITS Value */
#define MXC_S_SPIMSS_MOD_NUMBITS_4BITS                 (MXC_V_SPIMSS_MOD_NUMBITS_4BITS << MXC_F_SPIMSS_MOD_NUMBITS_POS) /**< MOD_NUMBITS_4BITS Setting */
#define MXC_V_SPIMSS_MOD_NUMBITS_5BITS                 ((uint32_t)0x5UL) /**< MOD_NUMBITS_5BITS Value */
#define MXC_S_SPIMSS_MOD_NUMBITS_5BITS                 (MXC_V_SPIMSS_MOD_NUMBITS_5BITS << MXC_F_SPIMSS_MOD_NUMBITS_POS) /**< MOD_NUMBITS_5BITS Setting */
#define MXC_V_SPIMSS_MOD_NUMBITS_6BITS                 ((uint32_t)0x6UL) /**< MOD_NUMBITS_6BITS Value */
#define MXC_S_SPIMSS_MOD_NUMBITS_6BITS                 (MXC_V_SPIMSS_MOD_NUMBITS_6BITS << MXC_F_SPIMSS_MOD_NUMBITS_POS) /**< MOD_NUMBITS_6BITS Setting */
#define MXC_V_SPIMSS_MOD_NUMBITS_7BITS                 ((uint32_t)0x7UL) /**< MOD_NUMBITS_7BITS Value */
#define MXC_S_SPIMSS_MOD_NUMBITS_7BITS                 (MXC_V_SPIMSS_MOD_NUMBITS_7BITS << MXC_F_SPIMSS_MOD_NUMBITS_POS) /**< MOD_NUMBITS_7BITS Setting */
#define MXC_V_SPIMSS_MOD_NUMBITS_8BITS                 ((uint32_t)0x8UL) /**< MOD_NUMBITS_8BITS Value */
#define MXC_S_SPIMSS_MOD_NUMBITS_8BITS                 (MXC_V_SPIMSS_MOD_NUMBITS_8BITS << MXC_F_SPIMSS_MOD_NUMBITS_POS) /**< MOD_NUMBITS_8BITS Setting */
#define MXC_V_SPIMSS_MOD_NUMBITS_9BITS                 ((uint32_t)0x9UL) /**< MOD_NUMBITS_9BITS Value */
#define MXC_S_SPIMSS_MOD_NUMBITS_9BITS                 (MXC_V_SPIMSS_MOD_NUMBITS_9BITS << MXC_F_SPIMSS_MOD_NUMBITS_POS) /**< MOD_NUMBITS_9BITS Setting */
#define MXC_V_SPIMSS_MOD_NUMBITS_10BITS                ((uint32_t)0xAUL) /**< MOD_NUMBITS_10BITS Value */
#define MXC_S_SPIMSS_MOD_NUMBITS_10BITS                (MXC_V_SPIMSS_MOD_NUMBITS_10BITS << MXC_F_SPIMSS_MOD_NUMBITS_POS) /**< MOD_NUMBITS_10BITS Setting */
#define MXC_V_SPIMSS_MOD_NUMBITS_11BITS                ((uint32_t)0xBUL) /**< MOD_NUMBITS_11BITS Value */
#define MXC_S_SPIMSS_MOD_NUMBITS_11BITS                (MXC_V_SPIMSS_MOD_NUMBITS_11BITS << MXC_F_SPIMSS_MOD_NUMBITS_POS) /**< MOD_NUMBITS_11BITS Setting */
#define MXC_V_SPIMSS_MOD_NUMBITS_12BITS                ((uint32_t)0xCUL) /**< MOD_NUMBITS_12BITS Value */
#define MXC_S_SPIMSS_MOD_NUMBITS_12BITS                (MXC_V_SPIMSS_MOD_NUMBITS_12BITS << MXC_F_SPIMSS_MOD_NUMBITS_POS) /**< MOD_NUMBITS_12BITS Setting */
#define MXC_V_SPIMSS_MOD_NUMBITS_13BITS                ((uint32_t)0xDUL) /**< MOD_NUMBITS_13BITS Value */
#define MXC_S_SPIMSS_MOD_NUMBITS_13BITS                (MXC_V_SPIMSS_MOD_NUMBITS_13BITS << MXC_F_SPIMSS_MOD_NUMBITS_POS) /**< MOD_NUMBITS_13BITS Setting */
#define MXC_V_SPIMSS_MOD_NUMBITS_14BITS                ((uint32_t)0xEUL) /**< MOD_NUMBITS_14BITS Value */
#define MXC_S_SPIMSS_MOD_NUMBITS_14BITS                (MXC_V_SPIMSS_MOD_NUMBITS_14BITS << MXC_F_SPIMSS_MOD_NUMBITS_POS) /**< MOD_NUMBITS_14BITS Setting */
#define MXC_V_SPIMSS_MOD_NUMBITS_15BITS                ((uint32_t)0xFUL) /**< MOD_NUMBITS_15BITS Value */
#define MXC_S_SPIMSS_MOD_NUMBITS_15BITS                (MXC_V_SPIMSS_MOD_NUMBITS_15BITS << MXC_F_SPIMSS_MOD_NUMBITS_POS) /**< MOD_NUMBITS_15BITS Setting */

#define MXC_F_SPIMSS_MOD_TX_ALIGN_POS                  7 /**< MOD_TX_ALIGN Position */
#define MXC_F_SPIMSS_MOD_TX_ALIGN                      ((uint32_t)(0x1UL << MXC_F_SPIMSS_MOD_TX_ALIGN_POS)) /**< MOD_TX_ALIGN Mask */
#define MXC_V_SPIMSS_MOD_TX_ALIGN_LSB                  ((uint32_t)0x0UL) /**< MOD_TX_ALIGN_LSB Value */
#define MXC_S_SPIMSS_MOD_TX_ALIGN_LSB                  (MXC_V_SPIMSS_MOD_TX_ALIGN_LSB << MXC_F_SPIMSS_MOD_TX_ALIGN_POS) /**< MOD_TX_ALIGN_LSB Setting */
#define MXC_V_SPIMSS_MOD_TX_ALIGN_MSB                  ((uint32_t)0x1UL) /**< MOD_TX_ALIGN_MSB Value */
#define MXC_S_SPIMSS_MOD_TX_ALIGN_MSB                  (MXC_V_SPIMSS_MOD_TX_ALIGN_MSB << MXC_F_SPIMSS_MOD_TX_ALIGN_POS) /**< MOD_TX_ALIGN_MSB Setting */

/**@} end of group SPIMSS_MOD_Register */

/**
 * @ingroup  spimss_registers
 * @defgroup SPIMSS_BRG SPIMSS_BRG
 * @brief    Baud Rate Reload Value. The SPI Baud Rate register is a 16-bit reload value for
 *           the SPI Baud Rate Generator. The reload value must be greater than or equal to
 *           0002H for proper SPI operation (maximum baud rate is PCLK frequency divided by
 *           4).
 * @{
 */
#define MXC_F_SPIMSS_BRG_DIV_POS                       0 /**< BRG_DIV Position */
#define MXC_F_SPIMSS_BRG_DIV                           ((uint32_t)(0xFFFFUL << MXC_F_SPIMSS_BRG_DIV_POS)) /**< BRG_DIV Mask */

/**@} end of group SPIMSS_BRG_Register */

/**
 * @ingroup  spimss_registers
 * @defgroup SPIMSS_DMA SPIMSS_DMA
 * @brief    SPI DMA Register.
 * @{
 */
#define MXC_F_SPIMSS_DMA_TX_FIFO_LVL_POS               0 /**< DMA_TX_FIFO_LVL Position */
#define MXC_F_SPIMSS_DMA_TX_FIFO_LVL                   ((uint32_t)(0x7UL << MXC_F_SPIMSS_DMA_TX_FIFO_LVL_POS)) /**< DMA_TX_FIFO_LVL Mask */
#define MXC_V_SPIMSS_DMA_TX_FIFO_LVL_1ENTRIES          ((uint32_t)0x0UL) /**< DMA_TX_FIFO_LVL_1ENTRIES Value */
#define MXC_S_SPIMSS_DMA_TX_FIFO_LVL_1ENTRIES          (MXC_V_SPIMSS_DMA_TX_FIFO_LVL_1ENTRIES << MXC_F_SPIMSS_DMA_TX_FIFO_LVL_POS) /**< DMA_TX_FIFO_LVL_1ENTRIES Setting */
#define MXC_V_SPIMSS_DMA_TX_FIFO_LVL_2ENTRIES          ((uint32_t)0x1UL) /**< DMA_TX_FIFO_LVL_2ENTRIES Value */
#define MXC_S_SPIMSS_DMA_TX_FIFO_LVL_2ENTRIES          (MXC_V_SPIMSS_DMA_TX_FIFO_LVL_2ENTRIES << MXC_F_SPIMSS_DMA_TX_FIFO_LVL_POS) /**< DMA_TX_FIFO_LVL_2ENTRIES Setting */
#define MXC_V_SPIMSS_DMA_TX_FIFO_LVL_3ENTRIES          ((uint32_t)0x2UL) /**< DMA_TX_FIFO_LVL_3ENTRIES Value */
#define MXC_S_SPIMSS_DMA_TX_FIFO_LVL_3ENTRIES          (MXC_V_SPIMSS_DMA_TX_FIFO_LVL_3ENTRIES << MXC_F_SPIMSS_DMA_TX_FIFO_LVL_POS) /**< DMA_TX_FIFO_LVL_3ENTRIES Setting */
#define MXC_V_SPIMSS_DMA_TX_FIFO_LVL_4ENTRIES          ((uint32_t)0x3UL) /**< DMA_TX_FIFO_LVL_4ENTRIES Value */
#define MXC_S_SPIMSS_DMA_TX_FIFO_LVL_4ENTRIES          (MXC_V_SPIMSS_DMA_TX_FIFO_LVL_4ENTRIES << MXC_F_SPIMSS_DMA_TX_FIFO_LVL_POS) /**< DMA_TX_FIFO_LVL_4ENTRIES Setting */
#define MXC_V_SPIMSS_DMA_TX_FIFO_LVL_5ENTRIES          ((uint32_t)0x4UL) /**< DMA_TX_FIFO_LVL_5ENTRIES Value */
#define MXC_S_SPIMSS_DMA_TX_FIFO_LVL_5ENTRIES          (MXC_V_SPIMSS_DMA_TX_FIFO_LVL_5ENTRIES << MXC_F_SPIMSS_DMA_TX_FIFO_LVL_POS) /**< DMA_TX_FIFO_LVL_5ENTRIES Setting */
#define MXC_V_SPIMSS_DMA_TX_FIFO_LVL_6ENTRIES          ((uint32_t)0x5UL) /**< DMA_TX_FIFO_LVL_6ENTRIES Value */
#define MXC_S_SPIMSS_DMA_TX_FIFO_LVL_6ENTRIES          (MXC_V_SPIMSS_DMA_TX_FIFO_LVL_6ENTRIES << MXC_F_SPIMSS_DMA_TX_FIFO_LVL_POS) /**< DMA_TX_FIFO_LVL_6ENTRIES Setting */
#define MXC_V_SPIMSS_DMA_TX_FIFO_LVL_7ENTRIES          ((uint32_t)0x6UL) /**< DMA_TX_FIFO_LVL_7ENTRIES Value */
#define MXC_S_SPIMSS_DMA_TX_FIFO_LVL_7ENTRIES          (MXC_V_SPIMSS_DMA_TX_FIFO_LVL_7ENTRIES << MXC_F_SPIMSS_DMA_TX_FIFO_LVL_POS) /**< DMA_TX_FIFO_LVL_7ENTRIES Setting */
#define MXC_V_SPIMSS_DMA_TX_FIFO_LVL_8ENTRIES          ((uint32_t)0x7UL) /**< DMA_TX_FIFO_LVL_8ENTRIES Value */
#define MXC_S_SPIMSS_DMA_TX_FIFO_LVL_8ENTRIES          (MXC_V_SPIMSS_DMA_TX_FIFO_LVL_8ENTRIES << MXC_F_SPIMSS_DMA_TX_FIFO_LVL_POS) /**< DMA_TX_FIFO_LVL_8ENTRIES Setting */

#define MXC_F_SPIMSS_DMA_TX_FIFO_CLR_POS               4 /**< DMA_TX_FIFO_CLR Position */
#define MXC_F_SPIMSS_DMA_TX_FIFO_CLR                   ((uint32_t)(0x1UL << MXC_F_SPIMSS_DMA_TX_FIFO_CLR_POS)) /**< DMA_TX_FIFO_CLR Mask */
#define MXC_V_SPIMSS_DMA_TX_FIFO_CLR_CLEAR             ((uint32_t)0x1UL) /**< DMA_TX_FIFO_CLR_CLEAR Value */
#define MXC_S_SPIMSS_DMA_TX_FIFO_CLR_CLEAR             (MXC_V_SPIMSS_DMA_TX_FIFO_CLR_CLEAR << MXC_F_SPIMSS_DMA_TX_FIFO_CLR_POS) /**< DMA_TX_FIFO_CLR_CLEAR Setting */

#define MXC_F_SPIMSS_DMA_TX_FIFO_CNT_POS               8 /**< DMA_TX_FIFO_CNT Position */
#define MXC_F_SPIMSS_DMA_TX_FIFO_CNT                   ((uint32_t)(0xFUL << MXC_F_SPIMSS_DMA_TX_FIFO_CNT_POS)) /**< DMA_TX_FIFO_CNT Mask */

#define MXC_F_SPIMSS_DMA_TX_DMA_EN_POS                 15 /**< DMA_TX_DMA_EN Position */
#define MXC_F_SPIMSS_DMA_TX_DMA_EN                     ((uint32_t)(0x1UL << MXC_F_SPIMSS_DMA_TX_DMA_EN_POS)) /**< DMA_TX_DMA_EN Mask */
#define MXC_V_SPIMSS_DMA_TX_DMA_EN_DIS                 ((uint32_t)0x0UL) /**< DMA_TX_DMA_EN_DIS Value */
#define MXC_S_SPIMSS_DMA_TX_DMA_EN_DIS                 (MXC_V_SPIMSS_DMA_TX_DMA_EN_DIS << MXC_F_SPIMSS_DMA_TX_DMA_EN_POS) /**< DMA_TX_DMA_EN_DIS Setting */
#define MXC_V_SPIMSS_DMA_TX_DMA_EN_EN                  ((uint32_t)0x1UL) /**< DMA_TX_DMA_EN_EN Value */
#define MXC_S_SPIMSS_DMA_TX_DMA_EN_EN                  (MXC_V_SPIMSS_DMA_TX_DMA_EN_EN << MXC_F_SPIMSS_DMA_TX_DMA_EN_POS) /**< DMA_TX_DMA_EN_EN Setting */

#define MXC_F_SPIMSS_DMA_RX_FIFO_LVL_POS               16 /**< DMA_RX_FIFO_LVL Position */
#define MXC_F_SPIMSS_DMA_RX_FIFO_LVL                   ((uint32_t)(0x7UL << MXC_F_SPIMSS_DMA_RX_FIFO_LVL_POS)) /**< DMA_RX_FIFO_LVL Mask */
#define MXC_V_SPIMSS_DMA_RX_FIFO_LVL_1ENTRIES          ((uint32_t)0x0UL) /**< DMA_RX_FIFO_LVL_1ENTRIES Value */
#define MXC_S_SPIMSS_DMA_RX_FIFO_LVL_1ENTRIES          (MXC_V_SPIMSS_DMA_RX_FIFO_LVL_1ENTRIES << MXC_F_SPIMSS_DMA_RX_FIFO_LVL_POS) /**< DMA_RX_FIFO_LVL_1ENTRIES Setting */
#define MXC_V_SPIMSS_DMA_RX_FIFO_LVL_2ENTRIES          ((uint32_t)0x1UL) /**< DMA_RX_FIFO_LVL_2ENTRIES Value */
#define MXC_S_SPIMSS_DMA_RX_FIFO_LVL_2ENTRIES          (MXC_V_SPIMSS_DMA_RX_FIFO_LVL_2ENTRIES << MXC_F_SPIMSS_DMA_RX_FIFO_LVL_POS) /**< DMA_RX_FIFO_LVL_2ENTRIES Setting */
#define MXC_V_SPIMSS_DMA_RX_FIFO_LVL_3ENTRIES          ((uint32_t)0x2UL) /**< DMA_RX_FIFO_LVL_3ENTRIES Value */
#define MXC_S_SPIMSS_DMA_RX_FIFO_LVL_3ENTRIES          (MXC_V_SPIMSS_DMA_RX_FIFO_LVL_3ENTRIES << MXC_F_SPIMSS_DMA_RX_FIFO_LVL_POS) /**< DMA_RX_FIFO_LVL_3ENTRIES Setting */
#define MXC_V_SPIMSS_DMA_RX_FIFO_LVL_4ENTRIES          ((uint32_t)0x3UL) /**< DMA_RX_FIFO_LVL_4ENTRIES Value */
#define MXC_S_SPIMSS_DMA_RX_FIFO_LVL_4ENTRIES          (MXC_V_SPIMSS_DMA_RX_FIFO_LVL_4ENTRIES << MXC_F_SPIMSS_DMA_RX_FIFO_LVL_POS) /**< DMA_RX_FIFO_LVL_4ENTRIES Setting */
#define MXC_V_SPIMSS_DMA_RX_FIFO_LVL_5ENTRIES          ((uint32_t)0x4UL) /**< DMA_RX_FIFO_LVL_5ENTRIES Value */
#define MXC_S_SPIMSS_DMA_RX_FIFO_LVL_5ENTRIES          (MXC_V_SPIMSS_DMA_RX_FIFO_LVL_5ENTRIES << MXC_F_SPIMSS_DMA_RX_FIFO_LVL_POS) /**< DMA_RX_FIFO_LVL_5ENTRIES Setting */
#define MXC_V_SPIMSS_DMA_RX_FIFO_LVL_6ENTRIES          ((uint32_t)0x5UL) /**< DMA_RX_FIFO_LVL_6ENTRIES Value */
#define MXC_S_SPIMSS_DMA_RX_FIFO_LVL_6ENTRIES          (MXC_V_SPIMSS_DMA_RX_FIFO_LVL_6ENTRIES << MXC_F_SPIMSS_DMA_RX_FIFO_LVL_POS) /**< DMA_RX_FIFO_LVL_6ENTRIES Setting */
#define MXC_V_SPIMSS_DMA_RX_FIFO_LVL_7ENTRIES          ((uint32_t)0x6UL) /**< DMA_RX_FIFO_LVL_7ENTRIES Value */
#define MXC_S_SPIMSS_DMA_RX_FIFO_LVL_7ENTRIES          (MXC_V_SPIMSS_DMA_RX_FIFO_LVL_7ENTRIES << MXC_F_SPIMSS_DMA_RX_FIFO_LVL_POS) /**< DMA_RX_FIFO_LVL_7ENTRIES Setting */
#define MXC_V_SPIMSS_DMA_RX_FIFO_LVL_8ENTRIES          ((uint32_t)0x7UL) /**< DMA_RX_FIFO_LVL_8ENTRIES Value */
#define MXC_S_SPIMSS_DMA_RX_FIFO_LVL_8ENTRIES          (MXC_V_SPIMSS_DMA_RX_FIFO_LVL_8ENTRIES << MXC_F_SPIMSS_DMA_RX_FIFO_LVL_POS) /**< DMA_RX_FIFO_LVL_8ENTRIES Setting */

#define MXC_F_SPIMSS_DMA_RX_FIFO_CLR_POS               20 /**< DMA_RX_FIFO_CLR Position */
#define MXC_F_SPIMSS_DMA_RX_FIFO_CLR                   ((uint32_t)(0x1UL << MXC_F_SPIMSS_DMA_RX_FIFO_CLR_POS)) /**< DMA_RX_FIFO_CLR Mask */
#define MXC_V_SPIMSS_DMA_RX_FIFO_CLR_CLEAR             ((uint32_t)0x1UL) /**< DMA_RX_FIFO_CLR_CLEAR Value */
#define MXC_S_SPIMSS_DMA_RX_FIFO_CLR_CLEAR             (MXC_V_SPIMSS_DMA_RX_FIFO_CLR_CLEAR << MXC_F_SPIMSS_DMA_RX_FIFO_CLR_POS) /**< DMA_RX_FIFO_CLR_CLEAR Setting */

#define MXC_F_SPIMSS_DMA_RX_FIFO_CNT_POS               24 /**< DMA_RX_FIFO_CNT Position */
#define MXC_F_SPIMSS_DMA_RX_FIFO_CNT                   ((uint32_t)(0xFUL << MXC_F_SPIMSS_DMA_RX_FIFO_CNT_POS)) /**< DMA_RX_FIFO_CNT Mask */

#define MXC_F_SPIMSS_DMA_RX_DMA_EN_POS                 31 /**< DMA_RX_DMA_EN Position */
#define MXC_F_SPIMSS_DMA_RX_DMA_EN                     ((uint32_t)(0x1UL << MXC_F_SPIMSS_DMA_RX_DMA_EN_POS)) /**< DMA_RX_DMA_EN Mask */
#define MXC_V_SPIMSS_DMA_RX_DMA_EN_DIS                 ((uint32_t)0x0UL) /**< DMA_RX_DMA_EN_DIS Value */
#define MXC_S_SPIMSS_DMA_RX_DMA_EN_DIS                 (MXC_V_SPIMSS_DMA_RX_DMA_EN_DIS << MXC_F_SPIMSS_DMA_RX_DMA_EN_POS) /**< DMA_RX_DMA_EN_DIS Setting */
#define MXC_V_SPIMSS_DMA_RX_DMA_EN_EN                  ((uint32_t)0x1UL) /**< DMA_RX_DMA_EN_EN Value */
#define MXC_S_SPIMSS_DMA_RX_DMA_EN_EN                  (MXC_V_SPIMSS_DMA_RX_DMA_EN_EN << MXC_F_SPIMSS_DMA_RX_DMA_EN_POS) /**< DMA_RX_DMA_EN_EN Setting */

/**@} end of group SPIMSS_DMA_Register */

/**
 * @ingroup  spimss_registers
 * @defgroup SPIMSS_I2S_CTRL SPIMSS_I2S_CTRL
 * @brief    I2S Control Register.
 * @{
 */
#define MXC_F_SPIMSS_I2S_CTRL_I2S_EN_POS               0 /**< I2S_CTRL_I2S_EN Position */
#define MXC_F_SPIMSS_I2S_CTRL_I2S_EN                   ((uint32_t)(0x1UL << MXC_F_SPIMSS_I2S_CTRL_I2S_EN_POS)) /**< I2S_CTRL_I2S_EN Mask */
#define MXC_V_SPIMSS_I2S_CTRL_I2S_EN_DIS               ((uint32_t)0x0UL) /**< I2S_CTRL_I2S_EN_DIS Value */
#define MXC_S_SPIMSS_I2S_CTRL_I2S_EN_DIS               (MXC_V_SPIMSS_I2S_CTRL_I2S_EN_DIS << MXC_F_SPIMSS_I2S_CTRL_I2S_EN_POS) /**< I2S_CTRL_I2S_EN_DIS Setting */
#define MXC_V_SPIMSS_I2S_CTRL_I2S_EN_EN                ((uint32_t)0x1UL) /**< I2S_CTRL_I2S_EN_EN Value */
#define MXC_S_SPIMSS_I2S_CTRL_I2S_EN_EN                (MXC_V_SPIMSS_I2S_CTRL_I2S_EN_EN << MXC_F_SPIMSS_I2S_CTRL_I2S_EN_POS) /**< I2S_CTRL_I2S_EN_EN Setting */

#define MXC_F_SPIMSS_I2S_CTRL_I2S_MUTE_POS             1 /**< I2S_CTRL_I2S_MUTE Position */
#define MXC_F_SPIMSS_I2S_CTRL_I2S_MUTE                 ((uint32_t)(0x1UL << MXC_F_SPIMSS_I2S_CTRL_I2S_MUTE_POS)) /**< I2S_CTRL_I2S_MUTE Mask */
#define MXC_V_SPIMSS_I2S_CTRL_I2S_MUTE_NORMAL          ((uint32_t)0x0UL) /**< I2S_CTRL_I2S_MUTE_NORMAL Value */
#define MXC_S_SPIMSS_I2S_CTRL_I2S_MUTE_NORMAL          (MXC_V_SPIMSS_I2S_CTRL_I2S_MUTE_NORMAL << MXC_F_SPIMSS_I2S_CTRL_I2S_MUTE_POS) /**< I2S_CTRL_I2S_MUTE_NORMAL Setting */
#define MXC_V_SPIMSS_I2S_CTRL_I2S_MUTE_MUTED           ((uint32_t)0x1UL) /**< I2S_CTRL_I2S_MUTE_MUTED Value */
#define MXC_S_SPIMSS_I2S_CTRL_I2S_MUTE_MUTED           (MXC_V_SPIMSS_I2S_CTRL_I2S_MUTE_MUTED << MXC_F_SPIMSS_I2S_CTRL_I2S_MUTE_POS) /**< I2S_CTRL_I2S_MUTE_MUTED Setting */

#define MXC_F_SPIMSS_I2S_CTRL_I2S_PAUSE_POS            2 /**< I2S_CTRL_I2S_PAUSE Position */
#define MXC_F_SPIMSS_I2S_CTRL_I2S_PAUSE                ((uint32_t)(0x1UL << MXC_F_SPIMSS_I2S_CTRL_I2S_PAUSE_POS)) /**< I2S_CTRL_I2S_PAUSE Mask */
#define MXC_V_SPIMSS_I2S_CTRL_I2S_PAUSE_NORMAL         ((uint32_t)0x0UL) /**< I2S_CTRL_I2S_PAUSE_NORMAL Value */
#define MXC_S_SPIMSS_I2S_CTRL_I2S_PAUSE_NORMAL         (MXC_V_SPIMSS_I2S_CTRL_I2S_PAUSE_NORMAL << MXC_F_SPIMSS_I2S_CTRL_I2S_PAUSE_POS) /**< I2S_CTRL_I2S_PAUSE_NORMAL Setting */
#define MXC_V_SPIMSS_I2S_CTRL_I2S_PAUSE_PAUSE          ((uint32_t)0x1UL) /**< I2S_CTRL_I2S_PAUSE_PAUSE Value */
#define MXC_S_SPIMSS_I2S_CTRL_I2S_PAUSE_PAUSE          (MXC_V_SPIMSS_I2S_CTRL_I2S_PAUSE_PAUSE << MXC_F_SPIMSS_I2S_CTRL_I2S_PAUSE_POS) /**< I2S_CTRL_I2S_PAUSE_PAUSE Setting */

#define MXC_F_SPIMSS_I2S_CTRL_I2S_MONO_POS             3 /**< I2S_CTRL_I2S_MONO Position */
#define MXC_F_SPIMSS_I2S_CTRL_I2S_MONO                 ((uint32_t)(0x1UL << MXC_F_SPIMSS_I2S_CTRL_I2S_MONO_POS)) /**< I2S_CTRL_I2S_MONO Mask */
#define MXC_V_SPIMSS_I2S_CTRL_I2S_MONO_STEREO          ((uint32_t)0x0UL) /**< I2S_CTRL_I2S_MONO_STEREO Value */
#define MXC_S_SPIMSS_I2S_CTRL_I2S_MONO_STEREO          (MXC_V_SPIMSS_I2S_CTRL_I2S_MONO_STEREO << MXC_F_SPIMSS_I2S_CTRL_I2S_MONO_POS) /**< I2S_CTRL_I2S_MONO_STEREO Setting */
#define MXC_V_SPIMSS_I2S_CTRL_I2S_MONO_MONO            ((uint32_t)0x1UL) /**< I2S_CTRL_I2S_MONO_MONO Value */
#define MXC_S_SPIMSS_I2S_CTRL_I2S_MONO_MONO            (MXC_V_SPIMSS_I2S_CTRL_I2S_MONO_MONO << MXC_F_SPIMSS_I2S_CTRL_I2S_MONO_POS) /**< I2S_CTRL_I2S_MONO_MONO Setting */

#define MXC_F_SPIMSS_I2S_CTRL_I2S_LJ_POS               4 /**< I2S_CTRL_I2S_LJ Position */
#define MXC_F_SPIMSS_I2S_CTRL_I2S_LJ                   ((uint32_t)(0x1UL << MXC_F_SPIMSS_I2S_CTRL_I2S_LJ_POS)) /**< I2S_CTRL_I2S_LJ Mask */
#define MXC_V_SPIMSS_I2S_CTRL_I2S_LJ_LAG               ((uint32_t)0x0UL) /**< I2S_CTRL_I2S_LJ_LAG Value */
#define MXC_S_SPIMSS_I2S_CTRL_I2S_LJ_LAG               (MXC_V_SPIMSS_I2S_CTRL_I2S_LJ_LAG << MXC_F_SPIMSS_I2S_CTRL_I2S_LJ_POS) /**< I2S_CTRL_I2S_LJ_LAG Setting */
#define MXC_V_SPIMSS_I2S_CTRL_I2S_LJ_SYNCRONIZED       ((uint32_t)0x1UL) /**< I2S_CTRL_I2S_LJ_SYNCRONIZED Value */
#define MXC_S_SPIMSS_I2S_CTRL_I2S_LJ_SYNCRONIZED       (MXC_V_SPIMSS_I2S_CTRL_I2S_LJ_SYNCRONIZED << MXC_F_SPIMSS_I2S_CTRL_I2S_LJ_POS) /**< I2S_CTRL_I2S_LJ_SYNCRONIZED Setting */

/**@} end of group SPIMSS_I2S_CTRL_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32650_INCLUDE_SPIMSS_REGS_H_
