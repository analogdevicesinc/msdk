/**
 * @file    htmr_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the HTMR Peripheral Module.
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32570_INCLUDE_HTMR_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32570_INCLUDE_HTMR_REGS_H_

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
 * @ingroup     htmr
 * @defgroup    htmr_registers HTMR_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the HTMR Peripheral Module.
 * @details     High Speed Timer Module.
 */

/**
 * @ingroup htmr_registers
 * Structure type to access the HTMR Registers.
 */
typedef struct {
    __IO uint32_t sec;                  /**< <tt>\b 0x00:</tt> HTMR SEC Register */
    __IO uint32_t ssec;                 /**< <tt>\b 0x04:</tt> HTMR SSEC Register */
    __IO uint32_t ras;                  /**< <tt>\b 0x08:</tt> HTMR RAS Register */
    __IO uint32_t rssa;                 /**< <tt>\b 0x0C:</tt> HTMR RSSA Register */
    __IO uint32_t ctrl;                 /**< <tt>\b 0x10:</tt> HTMR CTRL Register */
} mxc_htmr_regs_t;

/* Register offsets for module HTMR */
/**
 * @ingroup    htmr_registers
 * @defgroup   HTMR_Register_Offsets Register Offsets
 * @brief      HTMR Peripheral Register Offsets from the HTMR Base Peripheral Address.
 * @{
 */
#define MXC_R_HTMR_SEC                     ((uint32_t)0x00000000UL) /**< Offset from HTMR Base Address: <tt> 0x0000</tt> */
#define MXC_R_HTMR_SSEC                    ((uint32_t)0x00000004UL) /**< Offset from HTMR Base Address: <tt> 0x0004</tt> */
#define MXC_R_HTMR_RAS                     ((uint32_t)0x00000008UL) /**< Offset from HTMR Base Address: <tt> 0x0008</tt> */
#define MXC_R_HTMR_RSSA                    ((uint32_t)0x0000000CUL) /**< Offset from HTMR Base Address: <tt> 0x000C</tt> */
#define MXC_R_HTMR_CTRL                    ((uint32_t)0x00000010UL) /**< Offset from HTMR Base Address: <tt> 0x0010</tt> */
/**@} end of group htmr_registers */

/**
 * @ingroup  htmr_registers
 * @defgroup HTMR_SEC HTMR_SEC
 * @brief    HTimer Long-Interval Counter. This register contains the 32 most significant
 *           bits of the counter.
 * @{
 */
#define MXC_F_HTMR_SEC_RTS_POS                         0 /**< SEC_RTS Position */
#define MXC_F_HTMR_SEC_RTS                             ((uint32_t)(0xFFFFFFFFUL << MXC_F_HTMR_SEC_RTS_POS)) /**< SEC_RTS Mask */

/**@} end of group HTMR_SEC_Register */

/**
 * @ingroup  htmr_registers
 * @defgroup HTMR_SSEC HTMR_SSEC
 * @brief    HTimer Short Interval Counter. This counter ticks every t_htclk (16.48uS).
 *           HTIMER_SEC is incremented when this register rolls over from 0xFF to 0x00.
 * @{
 */
#define MXC_F_HTMR_SSEC_RTSS_POS                       0 /**< SSEC_RTSS Position */
#define MXC_F_HTMR_SSEC_RTSS                           ((uint32_t)(0xFFUL << MXC_F_HTMR_SSEC_RTSS_POS)) /**< SSEC_RTSS Mask */

/**@} end of group HTMR_SSEC_Register */

/**
 * @ingroup  htmr_registers
 * @defgroup HTMR_RAS HTMR_RAS
 * @brief    Long Interval Alarm.
 * @{
 */
#define MXC_F_HTMR_RAS_RAS_POS                         0 /**< RAS_RAS Position */
#define MXC_F_HTMR_RAS_RAS                             ((uint32_t)(0xFFFFFUL << MXC_F_HTMR_RAS_RAS_POS)) /**< RAS_RAS Mask */

/**@} end of group HTMR_RAS_Register */

/**
 * @ingroup  htmr_registers
 * @defgroup HTMR_RSSA HTMR_RSSA
 * @brief    HTimer Short Interval Alarm. This register contains the reload value for the
 *           short interval alarm, HTIMER_CTRL.alarm_ss_fl is raised on rollover.
 * @{
 */
#define MXC_F_HTMR_RSSA_RSSA_POS                       0 /**< RSSA_RSSA Position */
#define MXC_F_HTMR_RSSA_RSSA                           ((uint32_t)(0xFFFFFFFFUL << MXC_F_HTMR_RSSA_RSSA_POS)) /**< RSSA_RSSA Mask */

/**@} end of group HTMR_RSSA_Register */

/**
 * @ingroup  htmr_registers
 * @defgroup HTMR_CTRL HTMR_CTRL
 * @brief    HTimer Control Register.
 * @{
 */
#define MXC_F_HTMR_CTRL_HTEN_POS                       0 /**< CTRL_HTEN Position */
#define MXC_F_HTMR_CTRL_HTEN                           ((uint32_t)(0x1UL << MXC_F_HTMR_CTRL_HTEN_POS)) /**< CTRL_HTEN Mask */

#define MXC_F_HTMR_CTRL_ADE_POS                        1 /**< CTRL_ADE Position */
#define MXC_F_HTMR_CTRL_ADE                            ((uint32_t)(0x1UL << MXC_F_HTMR_CTRL_ADE_POS)) /**< CTRL_ADE Mask */

#define MXC_F_HTMR_CTRL_ASE_POS                        2 /**< CTRL_ASE Position */
#define MXC_F_HTMR_CTRL_ASE                            ((uint32_t)(0x1UL << MXC_F_HTMR_CTRL_ASE_POS)) /**< CTRL_ASE Mask */

#define MXC_F_HTMR_CTRL_BUSY_POS                       3 /**< CTRL_BUSY Position */
#define MXC_F_HTMR_CTRL_BUSY                           ((uint32_t)(0x1UL << MXC_F_HTMR_CTRL_BUSY_POS)) /**< CTRL_BUSY Mask */

#define MXC_F_HTMR_CTRL_RDY_POS                        4 /**< CTRL_RDY Position */
#define MXC_F_HTMR_CTRL_RDY                            ((uint32_t)(0x1UL << MXC_F_HTMR_CTRL_RDY_POS)) /**< CTRL_RDY Mask */

#define MXC_F_HTMR_CTRL_RDYE_POS                       5 /**< CTRL_RDYE Position */
#define MXC_F_HTMR_CTRL_RDYE                           ((uint32_t)(0x1UL << MXC_F_HTMR_CTRL_RDYE_POS)) /**< CTRL_RDYE Mask */

#define MXC_F_HTMR_CTRL_ALDF_POS                       6 /**< CTRL_ALDF Position */
#define MXC_F_HTMR_CTRL_ALDF                           ((uint32_t)(0x1UL << MXC_F_HTMR_CTRL_ALDF_POS)) /**< CTRL_ALDF Mask */

#define MXC_F_HTMR_CTRL_ALSF_POS                       7 /**< CTRL_ALSF Position */
#define MXC_F_HTMR_CTRL_ALSF                           ((uint32_t)(0x1UL << MXC_F_HTMR_CTRL_ALSF_POS)) /**< CTRL_ALSF Mask */

#define MXC_F_HTMR_CTRL_WE_POS                         15 /**< CTRL_WE Position */
#define MXC_F_HTMR_CTRL_WE                             ((uint32_t)(0x1UL << MXC_F_HTMR_CTRL_WE_POS)) /**< CTRL_WE Mask */

/**@} end of group HTMR_CTRL_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32570_INCLUDE_HTMR_REGS_H_
