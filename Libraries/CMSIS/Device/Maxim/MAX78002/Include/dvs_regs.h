/**
 * @file    dvs_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the DVS Peripheral Module.
 * @note    This file is @generated.
 * @ingroup dvs_registers
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX78002_INCLUDE_DVS_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX78002_INCLUDE_DVS_REGS_H_

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
 * @ingroup     dvs
 * @defgroup    dvs_registers DVS_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the DVS Peripheral Module.
 * @details     Dynamic Voltage Scaling
 */

/**
 * @ingroup dvs_registers
 * Structure type to access the DVS Registers.
 */
typedef struct {
    __IO uint32_t ctl;                  /**< <tt>\b 0x00:</tt> DVS CTL Register */
    __IO uint32_t stat;                 /**< <tt>\b 0x04:</tt> DVS STAT Register */
    __IO uint32_t direct;               /**< <tt>\b 0x08:</tt> DVS DIRECT Register */
    __IO uint32_t mon;                  /**< <tt>\b 0x00C:</tt> DVS MON Register */
    __IO uint32_t adj_up;               /**< <tt>\b 0x010:</tt> DVS ADJ_UP Register */
    __IO uint32_t adj_dwn;              /**< <tt>\b 0x014:</tt> DVS ADJ_DWN Register */
    __IO uint32_t thres_cmp;            /**< <tt>\b 0x018:</tt> DVS THRES_CMP Register */
    __IO uint32_t tap_sel[5];           /**< <tt>\b 0x1C:</tt> DVS TAP_SEL Register */
} mxc_dvs_regs_t;

/* Register offsets for module DVS */
/**
 * @ingroup    dvs_registers
 * @defgroup   DVS_Register_Offsets Register Offsets
 * @brief      DVS Peripheral Register Offsets from the DVS Base Peripheral Address.
 * @{
 */
#define MXC_R_DVS_CTL                      ((uint32_t)0x00000000UL) /**< Offset from DVS Base Address: <tt> 0x0000</tt> */
#define MXC_R_DVS_STAT                     ((uint32_t)0x00000004UL) /**< Offset from DVS Base Address: <tt> 0x0004</tt> */
#define MXC_R_DVS_DIRECT                   ((uint32_t)0x00000008UL) /**< Offset from DVS Base Address: <tt> 0x0008</tt> */
#define MXC_R_DVS_MON                      ((uint32_t)0x0000000CUL) /**< Offset from DVS Base Address: <tt> 0x000C</tt> */
#define MXC_R_DVS_ADJ_UP                   ((uint32_t)0x00000010UL) /**< Offset from DVS Base Address: <tt> 0x0010</tt> */
#define MXC_R_DVS_ADJ_DWN                  ((uint32_t)0x00000014UL) /**< Offset from DVS Base Address: <tt> 0x0014</tt> */
#define MXC_R_DVS_THRES_CMP                ((uint32_t)0x00000018UL) /**< Offset from DVS Base Address: <tt> 0x0018</tt> */
#define MXC_R_DVS_TAP_SEL                  ((uint32_t)0x0000001CUL) /**< Offset from DVS Base Address: <tt> 0x001C</tt> */
/**@} end of group dvs_registers */

/**
 * @ingroup  dvs_registers
 * @defgroup DVS_CTL DVS_CTL
 * @brief    Control Register
 * @{
 */
#define MXC_F_DVS_CTL_MON_ENA_POS                      0 /**< CTL_MON_ENA Position */
#define MXC_F_DVS_CTL_MON_ENA                          ((uint32_t)(0x1UL << MXC_F_DVS_CTL_MON_ENA_POS)) /**< CTL_MON_ENA Mask */

#define MXC_F_DVS_CTL_ADJ_ENA_POS                      1 /**< CTL_ADJ_ENA Position */
#define MXC_F_DVS_CTL_ADJ_ENA                          ((uint32_t)(0x1UL << MXC_F_DVS_CTL_ADJ_ENA_POS)) /**< CTL_ADJ_ENA Mask */

#define MXC_F_DVS_CTL_PS_FB_DIS_POS                    2 /**< CTL_PS_FB_DIS Position */
#define MXC_F_DVS_CTL_PS_FB_DIS                        ((uint32_t)(0x1UL << MXC_F_DVS_CTL_PS_FB_DIS_POS)) /**< CTL_PS_FB_DIS Mask */

#define MXC_F_DVS_CTL_CTRL_TAP_ENA_POS                 3 /**< CTL_CTRL_TAP_ENA Position */
#define MXC_F_DVS_CTL_CTRL_TAP_ENA                     ((uint32_t)(0x1UL << MXC_F_DVS_CTL_CTRL_TAP_ENA_POS)) /**< CTL_CTRL_TAP_ENA Mask */

#define MXC_F_DVS_CTL_PROP_DLY_POS                     4 /**< CTL_PROP_DLY Position */
#define MXC_F_DVS_CTL_PROP_DLY                         ((uint32_t)(0x3UL << MXC_F_DVS_CTL_PROP_DLY_POS)) /**< CTL_PROP_DLY Mask */

#define MXC_F_DVS_CTL_MON_ONESHOT_POS                  6 /**< CTL_MON_ONESHOT Position */
#define MXC_F_DVS_CTL_MON_ONESHOT                      ((uint32_t)(0x1UL << MXC_F_DVS_CTL_MON_ONESHOT_POS)) /**< CTL_MON_ONESHOT Mask */

#define MXC_F_DVS_CTL_GO_DIRECT_POS                    7 /**< CTL_GO_DIRECT Position */
#define MXC_F_DVS_CTL_GO_DIRECT                        ((uint32_t)(0x1UL << MXC_F_DVS_CTL_GO_DIRECT_POS)) /**< CTL_GO_DIRECT Mask */

#define MXC_F_DVS_CTL_DIRECT_REG_POS                   8 /**< CTL_DIRECT_REG Position */
#define MXC_F_DVS_CTL_DIRECT_REG                       ((uint32_t)(0x1UL << MXC_F_DVS_CTL_DIRECT_REG_POS)) /**< CTL_DIRECT_REG Mask */

#define MXC_F_DVS_CTL_PRIME_ENA_POS                    9 /**< CTL_PRIME_ENA Position */
#define MXC_F_DVS_CTL_PRIME_ENA                        ((uint32_t)(0x1UL << MXC_F_DVS_CTL_PRIME_ENA_POS)) /**< CTL_PRIME_ENA Mask */

#define MXC_F_DVS_CTL_LIMIT_IE_POS                     10 /**< CTL_LIMIT_IE Position */
#define MXC_F_DVS_CTL_LIMIT_IE                         ((uint32_t)(0x1UL << MXC_F_DVS_CTL_LIMIT_IE_POS)) /**< CTL_LIMIT_IE Mask */

#define MXC_F_DVS_CTL_RANGE_IE_POS                     11 /**< CTL_RANGE_IE Position */
#define MXC_F_DVS_CTL_RANGE_IE                         ((uint32_t)(0x1UL << MXC_F_DVS_CTL_RANGE_IE_POS)) /**< CTL_RANGE_IE Mask */

#define MXC_F_DVS_CTL_ADJ_IE_POS                       12 /**< CTL_ADJ_IE Position */
#define MXC_F_DVS_CTL_ADJ_IE                           ((uint32_t)(0x1UL << MXC_F_DVS_CTL_ADJ_IE_POS)) /**< CTL_ADJ_IE Mask */

#define MXC_F_DVS_CTL_REF_SEL_POS                      13 /**< CTL_REF_SEL Position */
#define MXC_F_DVS_CTL_REF_SEL                          ((uint32_t)(0xFUL << MXC_F_DVS_CTL_REF_SEL_POS)) /**< CTL_REF_SEL Mask */

#define MXC_F_DVS_CTL_INC_VAL_POS                      17 /**< CTL_INC_VAL Position */
#define MXC_F_DVS_CTL_INC_VAL                          ((uint32_t)(0x7UL << MXC_F_DVS_CTL_INC_VAL_POS)) /**< CTL_INC_VAL Mask */

#define MXC_F_DVS_CTL_DVS_PS_APB_DIS_POS               20 /**< CTL_DVS_PS_APB_DIS Position */
#define MXC_F_DVS_CTL_DVS_PS_APB_DIS                   ((uint32_t)(0x1UL << MXC_F_DVS_CTL_DVS_PS_APB_DIS_POS)) /**< CTL_DVS_PS_APB_DIS Mask */

#define MXC_F_DVS_CTL_DVS_HI_RANGE_ANY_POS             21 /**< CTL_DVS_HI_RANGE_ANY Position */
#define MXC_F_DVS_CTL_DVS_HI_RANGE_ANY                 ((uint32_t)(0x1UL << MXC_F_DVS_CTL_DVS_HI_RANGE_ANY_POS)) /**< CTL_DVS_HI_RANGE_ANY Mask */

#define MXC_F_DVS_CTL_FB_TO_IE_POS                     22 /**< CTL_FB_TO_IE Position */
#define MXC_F_DVS_CTL_FB_TO_IE                         ((uint32_t)(0x1UL << MXC_F_DVS_CTL_FB_TO_IE_POS)) /**< CTL_FB_TO_IE Mask */

#define MXC_F_DVS_CTL_FC_LV_IE_POS                     23 /**< CTL_FC_LV_IE Position */
#define MXC_F_DVS_CTL_FC_LV_IE                         ((uint32_t)(0x1UL << MXC_F_DVS_CTL_FC_LV_IE_POS)) /**< CTL_FC_LV_IE Mask */

#define MXC_F_DVS_CTL_PD_ACK_ENA_POS                   24 /**< CTL_PD_ACK_ENA Position */
#define MXC_F_DVS_CTL_PD_ACK_ENA                       ((uint32_t)(0x1UL << MXC_F_DVS_CTL_PD_ACK_ENA_POS)) /**< CTL_PD_ACK_ENA Mask */

#define MXC_F_DVS_CTL_ADJ_ABORT_POS                    25 /**< CTL_ADJ_ABORT Position */
#define MXC_F_DVS_CTL_ADJ_ABORT                        ((uint32_t)(0x1UL << MXC_F_DVS_CTL_ADJ_ABORT_POS)) /**< CTL_ADJ_ABORT Mask */

/**@} end of group DVS_CTL_Register */

/**
 * @ingroup  dvs_registers
 * @defgroup DVS_STAT DVS_STAT
 * @brief    Status Fields
 * @{
 */
#define MXC_F_DVS_STAT_DVS_STATE_POS                   0 /**< STAT_DVS_STATE Position */
#define MXC_F_DVS_STAT_DVS_STATE                       ((uint32_t)(0xFUL << MXC_F_DVS_STAT_DVS_STATE_POS)) /**< STAT_DVS_STATE Mask */

#define MXC_F_DVS_STAT_ADJ_UP_ENA_POS                  4 /**< STAT_ADJ_UP_ENA Position */
#define MXC_F_DVS_STAT_ADJ_UP_ENA                      ((uint32_t)(0x1UL << MXC_F_DVS_STAT_ADJ_UP_ENA_POS)) /**< STAT_ADJ_UP_ENA Mask */

#define MXC_F_DVS_STAT_ADJ_DWN_ENA_POS                 5 /**< STAT_ADJ_DWN_ENA Position */
#define MXC_F_DVS_STAT_ADJ_DWN_ENA                     ((uint32_t)(0x1UL << MXC_F_DVS_STAT_ADJ_DWN_ENA_POS)) /**< STAT_ADJ_DWN_ENA Mask */

#define MXC_F_DVS_STAT_ADJ_ACTIVE_POS                  6 /**< STAT_ADJ_ACTIVE Position */
#define MXC_F_DVS_STAT_ADJ_ACTIVE                      ((uint32_t)(0x1UL << MXC_F_DVS_STAT_ADJ_ACTIVE_POS)) /**< STAT_ADJ_ACTIVE Mask */

#define MXC_F_DVS_STAT_CTR_TAP_OK_POS                  7 /**< STAT_CTR_TAP_OK Position */
#define MXC_F_DVS_STAT_CTR_TAP_OK                      ((uint32_t)(0x1UL << MXC_F_DVS_STAT_CTR_TAP_OK_POS)) /**< STAT_CTR_TAP_OK Mask */

#define MXC_F_DVS_STAT_CTR_TAP_SEL_POS                 8 /**< STAT_CTR_TAP_SEL Position */
#define MXC_F_DVS_STAT_CTR_TAP_SEL                     ((uint32_t)(0x1UL << MXC_F_DVS_STAT_CTR_TAP_SEL_POS)) /**< STAT_CTR_TAP_SEL Mask */

#define MXC_F_DVS_STAT_SLOW_TRIP_DET_POS               9 /**< STAT_SLOW_TRIP_DET Position */
#define MXC_F_DVS_STAT_SLOW_TRIP_DET                   ((uint32_t)(0x1UL << MXC_F_DVS_STAT_SLOW_TRIP_DET_POS)) /**< STAT_SLOW_TRIP_DET Mask */

#define MXC_F_DVS_STAT_FAST_TRIP_DET_POS               10 /**< STAT_FAST_TRIP_DET Position */
#define MXC_F_DVS_STAT_FAST_TRIP_DET                   ((uint32_t)(0x1UL << MXC_F_DVS_STAT_FAST_TRIP_DET_POS)) /**< STAT_FAST_TRIP_DET Mask */

#define MXC_F_DVS_STAT_PS_IN_RANGE_POS                 11 /**< STAT_PS_IN_RANGE Position */
#define MXC_F_DVS_STAT_PS_IN_RANGE                     ((uint32_t)(0x1UL << MXC_F_DVS_STAT_PS_IN_RANGE_POS)) /**< STAT_PS_IN_RANGE Mask */

#define MXC_F_DVS_STAT_PS_VCNTR_POS                    12 /**< STAT_PS_VCNTR Position */
#define MXC_F_DVS_STAT_PS_VCNTR                        ((uint32_t)(0x7FUL << MXC_F_DVS_STAT_PS_VCNTR_POS)) /**< STAT_PS_VCNTR Mask */

#define MXC_F_DVS_STAT_MON_DLY_OK_POS                  19 /**< STAT_MON_DLY_OK Position */
#define MXC_F_DVS_STAT_MON_DLY_OK                      ((uint32_t)(0x1UL << MXC_F_DVS_STAT_MON_DLY_OK_POS)) /**< STAT_MON_DLY_OK Mask */

#define MXC_F_DVS_STAT_ADJ_DLY_OK_POS                  20 /**< STAT_ADJ_DLY_OK Position */
#define MXC_F_DVS_STAT_ADJ_DLY_OK                      ((uint32_t)(0x1UL << MXC_F_DVS_STAT_ADJ_DLY_OK_POS)) /**< STAT_ADJ_DLY_OK Mask */

#define MXC_F_DVS_STAT_LO_LIMIT_DET_POS                21 /**< STAT_LO_LIMIT_DET Position */
#define MXC_F_DVS_STAT_LO_LIMIT_DET                    ((uint32_t)(0x1UL << MXC_F_DVS_STAT_LO_LIMIT_DET_POS)) /**< STAT_LO_LIMIT_DET Mask */

#define MXC_F_DVS_STAT_HI_LIMIT_DET_POS                22 /**< STAT_HI_LIMIT_DET Position */
#define MXC_F_DVS_STAT_HI_LIMIT_DET                    ((uint32_t)(0x1UL << MXC_F_DVS_STAT_HI_LIMIT_DET_POS)) /**< STAT_HI_LIMIT_DET Mask */

#define MXC_F_DVS_STAT_VALID_TAP_POS                   23 /**< STAT_VALID_TAP Position */
#define MXC_F_DVS_STAT_VALID_TAP                       ((uint32_t)(0x1UL << MXC_F_DVS_STAT_VALID_TAP_POS)) /**< STAT_VALID_TAP Mask */

#define MXC_F_DVS_STAT_LIMIT_ERR_POS                   24 /**< STAT_LIMIT_ERR Position */
#define MXC_F_DVS_STAT_LIMIT_ERR                       ((uint32_t)(0x1UL << MXC_F_DVS_STAT_LIMIT_ERR_POS)) /**< STAT_LIMIT_ERR Mask */

#define MXC_F_DVS_STAT_RANGE_ERR_POS                   25 /**< STAT_RANGE_ERR Position */
#define MXC_F_DVS_STAT_RANGE_ERR                       ((uint32_t)(0x1UL << MXC_F_DVS_STAT_RANGE_ERR_POS)) /**< STAT_RANGE_ERR Mask */

#define MXC_F_DVS_STAT_ADJ_ERR_POS                     26 /**< STAT_ADJ_ERR Position */
#define MXC_F_DVS_STAT_ADJ_ERR                         ((uint32_t)(0x1UL << MXC_F_DVS_STAT_ADJ_ERR_POS)) /**< STAT_ADJ_ERR Mask */

#define MXC_F_DVS_STAT_REF_SEL_ERR_POS                 27 /**< STAT_REF_SEL_ERR Position */
#define MXC_F_DVS_STAT_REF_SEL_ERR                     ((uint32_t)(0x1UL << MXC_F_DVS_STAT_REF_SEL_ERR_POS)) /**< STAT_REF_SEL_ERR Mask */

#define MXC_F_DVS_STAT_FB_TO_ERR_POS                   28 /**< STAT_FB_TO_ERR Position */
#define MXC_F_DVS_STAT_FB_TO_ERR                       ((uint32_t)(0x1UL << MXC_F_DVS_STAT_FB_TO_ERR_POS)) /**< STAT_FB_TO_ERR Mask */

#define MXC_F_DVS_STAT_FB_TO_ERR_S_POS                 29 /**< STAT_FB_TO_ERR_S Position */
#define MXC_F_DVS_STAT_FB_TO_ERR_S                     ((uint32_t)(0x1UL << MXC_F_DVS_STAT_FB_TO_ERR_S_POS)) /**< STAT_FB_TO_ERR_S Mask */

#define MXC_F_DVS_STAT_FC_LV_DET_INT_POS               30 /**< STAT_FC_LV_DET_INT Position */
#define MXC_F_DVS_STAT_FC_LV_DET_INT                   ((uint32_t)(0x1UL << MXC_F_DVS_STAT_FC_LV_DET_INT_POS)) /**< STAT_FC_LV_DET_INT Mask */

#define MXC_F_DVS_STAT_FC_LV_DET_S_POS                 31 /**< STAT_FC_LV_DET_S Position */
#define MXC_F_DVS_STAT_FC_LV_DET_S                     ((uint32_t)(0x1UL << MXC_F_DVS_STAT_FC_LV_DET_S_POS)) /**< STAT_FC_LV_DET_S Mask */

/**@} end of group DVS_STAT_Register */

/**
 * @ingroup  dvs_registers
 * @defgroup DVS_DIRECT DVS_DIRECT
 * @brief    Direct control of target voltage
 * @{
 */
#define MXC_F_DVS_DIRECT_VOLTAGE_POS                   0 /**< DIRECT_VOLTAGE Position */
#define MXC_F_DVS_DIRECT_VOLTAGE                       ((uint32_t)(0x7FUL << MXC_F_DVS_DIRECT_VOLTAGE_POS)) /**< DIRECT_VOLTAGE Mask */

/**@} end of group DVS_DIRECT_Register */

/**
 * @ingroup  dvs_registers
 * @defgroup DVS_MON DVS_MON
 * @brief    Monitor Delay
 * @{
 */
#define MXC_F_DVS_MON_DLY_POS                          0 /**< MON_DLY Position */
#define MXC_F_DVS_MON_DLY                              ((uint32_t)(0xFFFFFFUL << MXC_F_DVS_MON_DLY_POS)) /**< MON_DLY Mask */

#define MXC_F_DVS_MON_PRE_POS                          24 /**< MON_PRE Position */
#define MXC_F_DVS_MON_PRE                              ((uint32_t)(0xFFUL << MXC_F_DVS_MON_PRE_POS)) /**< MON_PRE Mask */

/**@} end of group DVS_MON_Register */

/**
 * @ingroup  dvs_registers
 * @defgroup DVS_ADJ_UP DVS_ADJ_UP
 * @brief    Up Delay Register
 * @{
 */
#define MXC_F_DVS_ADJ_UP_DLY_POS                       0 /**< ADJ_UP_DLY Position */
#define MXC_F_DVS_ADJ_UP_DLY                           ((uint32_t)(0xFFFFUL << MXC_F_DVS_ADJ_UP_DLY_POS)) /**< ADJ_UP_DLY Mask */

#define MXC_F_DVS_ADJ_UP_PRE_POS                       16 /**< ADJ_UP_PRE Position */
#define MXC_F_DVS_ADJ_UP_PRE                           ((uint32_t)(0xFFUL << MXC_F_DVS_ADJ_UP_PRE_POS)) /**< ADJ_UP_PRE Mask */

/**@} end of group DVS_ADJ_UP_Register */

/**
 * @ingroup  dvs_registers
 * @defgroup DVS_ADJ_DWN DVS_ADJ_DWN
 * @brief    Down Delay Register
 * @{
 */
#define MXC_F_DVS_ADJ_DWN_DLY_POS                      0 /**< ADJ_DWN_DLY Position */
#define MXC_F_DVS_ADJ_DWN_DLY                          ((uint32_t)(0xFFFFUL << MXC_F_DVS_ADJ_DWN_DLY_POS)) /**< ADJ_DWN_DLY Mask */

#define MXC_F_DVS_ADJ_DWN_PRE_POS                      16 /**< ADJ_DWN_PRE Position */
#define MXC_F_DVS_ADJ_DWN_PRE                          ((uint32_t)(0xFFUL << MXC_F_DVS_ADJ_DWN_PRE_POS)) /**< ADJ_DWN_PRE Mask */

/**@} end of group DVS_ADJ_DWN_Register */

/**
 * @ingroup  dvs_registers
 * @defgroup DVS_THRES_CMP DVS_THRES_CMP
 * @brief    Up Delay Register
 * @{
 */
#define MXC_F_DVS_THRES_CMP_VCNTR_THRES_CNT_POS        0 /**< THRES_CMP_VCNTR_THRES_CNT Position */
#define MXC_F_DVS_THRES_CMP_VCNTR_THRES_CNT            ((uint32_t)(0x7FUL << MXC_F_DVS_THRES_CMP_VCNTR_THRES_CNT_POS)) /**< THRES_CMP_VCNTR_THRES_CNT Mask */

#define MXC_F_DVS_THRES_CMP_VCNTR_THRES_MASK_POS       8 /**< THRES_CMP_VCNTR_THRES_MASK Position */
#define MXC_F_DVS_THRES_CMP_VCNTR_THRES_MASK           ((uint32_t)(0x7FUL << MXC_F_DVS_THRES_CMP_VCNTR_THRES_MASK_POS)) /**< THRES_CMP_VCNTR_THRES_MASK Mask */

/**@} end of group DVS_THRES_CMP_Register */

/**
 * @ingroup  dvs_registers
 * @defgroup DVS_TAP_SEL DVS_TAP_SEL
 * @brief    DVS Tap Select Register
 * @{
 */
#define MXC_F_DVS_TAP_SEL_LO_POS                       0 /**< TAP_SEL_LO Position */
#define MXC_F_DVS_TAP_SEL_LO                           ((uint32_t)(0x1FUL << MXC_F_DVS_TAP_SEL_LO_POS)) /**< TAP_SEL_LO Mask */

#define MXC_F_DVS_TAP_SEL_LO_TAP_STAT_POS              5 /**< TAP_SEL_LO_TAP_STAT Position */
#define MXC_F_DVS_TAP_SEL_LO_TAP_STAT                  ((uint32_t)(0x1UL << MXC_F_DVS_TAP_SEL_LO_TAP_STAT_POS)) /**< TAP_SEL_LO_TAP_STAT Mask */

#define MXC_F_DVS_TAP_SEL_CTR_TAP_STAT_POS             6 /**< TAP_SEL_CTR_TAP_STAT Position */
#define MXC_F_DVS_TAP_SEL_CTR_TAP_STAT                 ((uint32_t)(0x1UL << MXC_F_DVS_TAP_SEL_CTR_TAP_STAT_POS)) /**< TAP_SEL_CTR_TAP_STAT Mask */

#define MXC_F_DVS_TAP_SEL_HI_TAP_STAT_POS              7 /**< TAP_SEL_HI_TAP_STAT Position */
#define MXC_F_DVS_TAP_SEL_HI_TAP_STAT                  ((uint32_t)(0x1UL << MXC_F_DVS_TAP_SEL_HI_TAP_STAT_POS)) /**< TAP_SEL_HI_TAP_STAT Mask */

#define MXC_F_DVS_TAP_SEL_HI_POS                       8 /**< TAP_SEL_HI Position */
#define MXC_F_DVS_TAP_SEL_HI                           ((uint32_t)(0x1FUL << MXC_F_DVS_TAP_SEL_HI_POS)) /**< TAP_SEL_HI Mask */

#define MXC_F_DVS_TAP_SEL_CTR_POS                      16 /**< TAP_SEL_CTR Position */
#define MXC_F_DVS_TAP_SEL_CTR                          ((uint32_t)(0x1FUL << MXC_F_DVS_TAP_SEL_CTR_POS)) /**< TAP_SEL_CTR Mask */

#define MXC_F_DVS_TAP_SEL_COARSE_POS                   24 /**< TAP_SEL_COARSE Position */
#define MXC_F_DVS_TAP_SEL_COARSE                       ((uint32_t)(0x7UL << MXC_F_DVS_TAP_SEL_COARSE_POS)) /**< TAP_SEL_COARSE Mask */

#define MXC_F_DVS_TAP_SEL_DET_DLY_POS                  29 /**< TAP_SEL_DET_DLY Position */
#define MXC_F_DVS_TAP_SEL_DET_DLY                      ((uint32_t)(0x3UL << MXC_F_DVS_TAP_SEL_DET_DLY_POS)) /**< TAP_SEL_DET_DLY Mask */

#define MXC_F_DVS_TAP_SEL_DELAY_ACT_POS                31 /**< TAP_SEL_DELAY_ACT Position */
#define MXC_F_DVS_TAP_SEL_DELAY_ACT                    ((uint32_t)(0x1UL << MXC_F_DVS_TAP_SEL_DELAY_ACT_POS)) /**< TAP_SEL_DELAY_ACT Mask */

/**@} end of group DVS_TAP_SEL_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX78002_INCLUDE_DVS_REGS_H_
