/**
 * @file    fcr_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the FCR Peripheral Module.
 * @note    This file is @generated.
 * @ingroup fcr_registers
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32572_INCLUDE_FCR_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32572_INCLUDE_FCR_REGS_H_

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
 * @ingroup     fcr
 * @defgroup    fcr_registers FCR_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the FCR Peripheral Module.
 * @details     Function Control Register.
 */

/**
 * @ingroup fcr_registers
 * Structure type to access the FCR Registers.
 */
typedef struct {
    __IO uint32_t fctrl0;               /**< <tt>\b 0x00:</tt> FCR FCTRL0 Register */
    __IO uint32_t fctrl1;               /**< <tt>\b 0x04:</tt> FCR FCTRL1 Register */
    __R  uint32_t rsv_0x8;
    __IO uint32_t fctrl3;               /**< <tt>\b 0x0C:</tt> FCR FCTRL3 Register */
    __IO uint32_t urvbootaddr;          /**< <tt>\b 0x10:</tt> FCR URVBOOTADDR Register */
    __IO uint32_t urvctrl;              /**< <tt>\b 0x14:</tt> FCR URVCTRL Register */
    __R  uint32_t rsv_0x18;
    __IO uint32_t gp;                   /**< <tt>\b 0x1C:</tt> FCR GP Register */
    __IO uint32_t trimctrl;             /**< <tt>\b 0x20:</tt> FCR TRIMCTRL Register */
    __IO uint32_t erfoks;               /**< <tt>\b 0x24:</tt> FCR ERFOKS Register */
} mxc_fcr_regs_t;

/* Register offsets for module FCR */
/**
 * @ingroup    fcr_registers
 * @defgroup   FCR_Register_Offsets Register Offsets
 * @brief      FCR Peripheral Register Offsets from the FCR Base Peripheral Address.
 * @{
 */
#define MXC_R_FCR_FCTRL0                   ((uint32_t)0x00000000UL) /**< Offset from FCR Base Address: <tt> 0x0000</tt> */
#define MXC_R_FCR_FCTRL1                   ((uint32_t)0x00000004UL) /**< Offset from FCR Base Address: <tt> 0x0004</tt> */
#define MXC_R_FCR_FCTRL3                   ((uint32_t)0x0000000CUL) /**< Offset from FCR Base Address: <tt> 0x000C</tt> */
#define MXC_R_FCR_URVBOOTADDR              ((uint32_t)0x00000010UL) /**< Offset from FCR Base Address: <tt> 0x0010</tt> */
#define MXC_R_FCR_URVCTRL                  ((uint32_t)0x00000014UL) /**< Offset from FCR Base Address: <tt> 0x0014</tt> */
#define MXC_R_FCR_GP                       ((uint32_t)0x0000001CUL) /**< Offset from FCR Base Address: <tt> 0x001C</tt> */
#define MXC_R_FCR_TRIMCTRL                 ((uint32_t)0x00000020UL) /**< Offset from FCR Base Address: <tt> 0x0020</tt> */
#define MXC_R_FCR_ERFOKS                   ((uint32_t)0x00000024UL) /**< Offset from FCR Base Address: <tt> 0x0024</tt> */
/**@} end of group fcr_registers */

/**
 * @ingroup  fcr_registers
 * @defgroup FCR_FCTRL0 FCR_FCTRL0
 * @brief    Register 0.
 * @{
 */
#define MXC_F_FCR_FCTRL0_USBCLKSEL_POS                 16 /**< FCTRL0_USBCLKSEL Position */
#define MXC_F_FCR_FCTRL0_USBCLKSEL                     ((uint32_t)(0x1UL << MXC_F_FCR_FCTRL0_USBCLKSEL_POS)) /**< FCTRL0_USBCLKSEL Mask */

#define MXC_F_FCR_FCTRL0_I2C0DGEN0_POS                 20 /**< FCTRL0_I2C0DGEN0 Position */
#define MXC_F_FCR_FCTRL0_I2C0DGEN0                     ((uint32_t)(0x1UL << MXC_F_FCR_FCTRL0_I2C0DGEN0_POS)) /**< FCTRL0_I2C0DGEN0 Mask */

#define MXC_F_FCR_FCTRL0_I2C0DGEN1_POS                 21 /**< FCTRL0_I2C0DGEN1 Position */
#define MXC_F_FCR_FCTRL0_I2C0DGEN1                     ((uint32_t)(0x1UL << MXC_F_FCR_FCTRL0_I2C0DGEN1_POS)) /**< FCTRL0_I2C0DGEN1 Mask */

#define MXC_F_FCR_FCTRL0_I2C1DGEN0_POS                 22 /**< FCTRL0_I2C1DGEN0 Position */
#define MXC_F_FCR_FCTRL0_I2C1DGEN0                     ((uint32_t)(0x1UL << MXC_F_FCR_FCTRL0_I2C1DGEN0_POS)) /**< FCTRL0_I2C1DGEN0 Mask */

#define MXC_F_FCR_FCTRL0_I2C1DGEN1_POS                 23 /**< FCTRL0_I2C1DGEN1 Position */
#define MXC_F_FCR_FCTRL0_I2C1DGEN1                     ((uint32_t)(0x1UL << MXC_F_FCR_FCTRL0_I2C1DGEN1_POS)) /**< FCTRL0_I2C1DGEN1 Mask */

/**@} end of group FCR_FCTRL0_Register */

/**
 * @ingroup  fcr_registers
 * @defgroup FCR_FCTRL1 FCR_FCTRL1
 * @brief    Register 1.
 * @{
 */
#define MXC_F_FCR_FCTRL1_AC_EN_POS                     0 /**< FCTRL1_AC_EN Position */
#define MXC_F_FCR_FCTRL1_AC_EN                         ((uint32_t)(0x1UL << MXC_F_FCR_FCTRL1_AC_EN_POS)) /**< FCTRL1_AC_EN Mask */

#define MXC_F_FCR_FCTRL1_AC_RUN_POS                    1 /**< FCTRL1_AC_RUN Position */
#define MXC_F_FCR_FCTRL1_AC_RUN                        ((uint32_t)(0x1UL << MXC_F_FCR_FCTRL1_AC_RUN_POS)) /**< FCTRL1_AC_RUN Mask */

#define MXC_F_FCR_FCTRL1_LOAD_POS                      2 /**< FCTRL1_LOAD Position */
#define MXC_F_FCR_FCTRL1_LOAD                          ((uint32_t)(0x1UL << MXC_F_FCR_FCTRL1_LOAD_POS)) /**< FCTRL1_LOAD Mask */

#define MXC_F_FCR_FCTRL1_INV_GAIN_POS                  3 /**< FCTRL1_INV_GAIN Position */
#define MXC_F_FCR_FCTRL1_INV_GAIN                      ((uint32_t)(0x1UL << MXC_F_FCR_FCTRL1_INV_GAIN_POS)) /**< FCTRL1_INV_GAIN Mask */

#define MXC_F_FCR_FCTRL1_ATOMIC_POS                    4 /**< FCTRL1_ATOMIC Position */
#define MXC_F_FCR_FCTRL1_ATOMIC                        ((uint32_t)(0x1UL << MXC_F_FCR_FCTRL1_ATOMIC_POS)) /**< FCTRL1_ATOMIC Mask */

#define MXC_F_FCR_FCTRL1_MU_POS                        8 /**< FCTRL1_MU Position */
#define MXC_F_FCR_FCTRL1_MU                            ((uint32_t)(0xFFFUL << MXC_F_FCR_FCTRL1_MU_POS)) /**< FCTRL1_MU Mask */

#define MXC_F_FCR_FCTRL1_AC_TRIM_POS                   23 /**< FCTRL1_AC_TRIM Position */
#define MXC_F_FCR_FCTRL1_AC_TRIM                       ((uint32_t)(0x1FFUL << MXC_F_FCR_FCTRL1_AC_TRIM_POS)) /**< FCTRL1_AC_TRIM Mask */

/**@} end of group FCR_FCTRL1_Register */

/**
 * @ingroup  fcr_registers
 * @defgroup FCR_FCTRL3 FCR_FCTRL3
 * @brief    Register 3.
 * @{
 */
#define MXC_F_FCR_FCTRL3_DONECNT_POS                   0 /**< FCTRL3_DONECNT Position */
#define MXC_F_FCR_FCTRL3_DONECNT                       ((uint32_t)(0xFFUL << MXC_F_FCR_FCTRL3_DONECNT_POS)) /**< FCTRL3_DONECNT Mask */

/**@} end of group FCR_FCTRL3_Register */

/**
 * @ingroup  fcr_registers
 * @defgroup FCR_URVBOOTADDR FCR_URVBOOTADDR
 * @brief    Register 4.
 * @{
 */
#define MXC_F_FCR_URVBOOTADDR_BOOTADDR_POS             0 /**< URVBOOTADDR_BOOTADDR Position */
#define MXC_F_FCR_URVBOOTADDR_BOOTADDR                 ((uint32_t)(0xFFFFFFFFUL << MXC_F_FCR_URVBOOTADDR_BOOTADDR_POS)) /**< URVBOOTADDR_BOOTADDR Mask */

/**@} end of group FCR_URVBOOTADDR_Register */

/**
 * @ingroup  fcr_registers
 * @defgroup FCR_URVCTRL FCR_URVCTRL
 * @brief    Register 5.
 * @{
 */
#define MXC_F_FCR_URVCTRL_SLEEP_REQ_POS                0 /**< URVCTRL_SLEEP_REQ Position */
#define MXC_F_FCR_URVCTRL_SLEEP_REQ                    ((uint32_t)(0x1UL << MXC_F_FCR_URVCTRL_SLEEP_REQ_POS)) /**< URVCTRL_SLEEP_REQ Mask */

#define MXC_F_FCR_URVCTRL_SLEEP_ACK_POS                1 /**< URVCTRL_SLEEP_ACK Position */
#define MXC_F_FCR_URVCTRL_SLEEP_ACK                    ((uint32_t)(0x1UL << MXC_F_FCR_URVCTRL_SLEEP_ACK_POS)) /**< URVCTRL_SLEEP_ACK Mask */

/**@} end of group FCR_URVCTRL_Register */

/**
 * @ingroup  fcr_registers
 * @defgroup FCR_GP FCR_GP
 * @brief    General Purpose Register.
 * @{
 */
#define MXC_F_FCR_GP_GP_POS                            0 /**< GP_GP Position */
#define MXC_F_FCR_GP_GP                                ((uint32_t)(0xFFFFFFFFUL << MXC_F_FCR_GP_GP_POS)) /**< GP_GP Mask */

/**@} end of group FCR_GP_Register */

/**
 * @ingroup  fcr_registers
 * @defgroup FCR_TRIMCTRL FCR_TRIMCTRL
 * @brief    MSR ADC Trim Register.
 * @{
 */
#define MXC_F_FCR_TRIMCTRL_MSR_R1_POS                  0 /**< TRIMCTRL_MSR_R1 Position */
#define MXC_F_FCR_TRIMCTRL_MSR_R1                      ((uint32_t)(0x3UL << MXC_F_FCR_TRIMCTRL_MSR_R1_POS)) /**< TRIMCTRL_MSR_R1 Mask */
#define MXC_V_FCR_TRIMCTRL_MSR_R1_0K                   ((uint32_t)0x0UL) /**< TRIMCTRL_MSR_R1_0K Value */
#define MXC_S_FCR_TRIMCTRL_MSR_R1_0K                   (MXC_V_FCR_TRIMCTRL_MSR_R1_0K << MXC_F_FCR_TRIMCTRL_MSR_R1_POS) /**< TRIMCTRL_MSR_R1_0K Setting */
#define MXC_V_FCR_TRIMCTRL_MSR_R1_1P2K                 ((uint32_t)0x1UL) /**< TRIMCTRL_MSR_R1_1P2K Value */
#define MXC_S_FCR_TRIMCTRL_MSR_R1_1P2K                 (MXC_V_FCR_TRIMCTRL_MSR_R1_1P2K << MXC_F_FCR_TRIMCTRL_MSR_R1_POS) /**< TRIMCTRL_MSR_R1_1P2K Setting */
#define MXC_V_FCR_TRIMCTRL_MSR_R1_2P4K                 ((uint32_t)0x2UL) /**< TRIMCTRL_MSR_R1_2P4K Value */
#define MXC_S_FCR_TRIMCTRL_MSR_R1_2P4K                 (MXC_V_FCR_TRIMCTRL_MSR_R1_2P4K << MXC_F_FCR_TRIMCTRL_MSR_R1_POS) /**< TRIMCTRL_MSR_R1_2P4K Setting */
#define MXC_V_FCR_TRIMCTRL_MSR_R1_4P8K                 ((uint32_t)0x3UL) /**< TRIMCTRL_MSR_R1_4P8K Value */
#define MXC_S_FCR_TRIMCTRL_MSR_R1_4P8K                 (MXC_V_FCR_TRIMCTRL_MSR_R1_4P8K << MXC_F_FCR_TRIMCTRL_MSR_R1_POS) /**< TRIMCTRL_MSR_R1_4P8K Setting */

#define MXC_F_FCR_TRIMCTRL_MSR_R2_POS                  2 /**< TRIMCTRL_MSR_R2 Position */
#define MXC_F_FCR_TRIMCTRL_MSR_R2                      ((uint32_t)(0x7UL << MXC_F_FCR_TRIMCTRL_MSR_R2_POS)) /**< TRIMCTRL_MSR_R2 Mask */
#define MXC_V_FCR_TRIMCTRL_MSR_R2_OPEN                 ((uint32_t)0x0UL) /**< TRIMCTRL_MSR_R2_OPEN Value */
#define MXC_S_FCR_TRIMCTRL_MSR_R2_OPEN                 (MXC_V_FCR_TRIMCTRL_MSR_R2_OPEN << MXC_F_FCR_TRIMCTRL_MSR_R2_POS) /**< TRIMCTRL_MSR_R2_OPEN Setting */
#define MXC_V_FCR_TRIMCTRL_MSR_R2_3K                   ((uint32_t)0x4UL) /**< TRIMCTRL_MSR_R2_3K Value */
#define MXC_S_FCR_TRIMCTRL_MSR_R2_3K                   (MXC_V_FCR_TRIMCTRL_MSR_R2_3K << MXC_F_FCR_TRIMCTRL_MSR_R2_POS) /**< TRIMCTRL_MSR_R2_3K Setting */
#define MXC_V_FCR_TRIMCTRL_MSR_R2_6K                   ((uint32_t)0x5UL) /**< TRIMCTRL_MSR_R2_6K Value */
#define MXC_S_FCR_TRIMCTRL_MSR_R2_6K                   (MXC_V_FCR_TRIMCTRL_MSR_R2_6K << MXC_F_FCR_TRIMCTRL_MSR_R2_POS) /**< TRIMCTRL_MSR_R2_6K Setting */
#define MXC_V_FCR_TRIMCTRL_MSR_R2_12K                  ((uint32_t)0x6UL) /**< TRIMCTRL_MSR_R2_12K Value */
#define MXC_S_FCR_TRIMCTRL_MSR_R2_12K                  (MXC_V_FCR_TRIMCTRL_MSR_R2_12K << MXC_F_FCR_TRIMCTRL_MSR_R2_POS) /**< TRIMCTRL_MSR_R2_12K Setting */
#define MXC_V_FCR_TRIMCTRL_MSR_R2_24K                  ((uint32_t)0x7UL) /**< TRIMCTRL_MSR_R2_24K Value */
#define MXC_S_FCR_TRIMCTRL_MSR_R2_24K                  (MXC_V_FCR_TRIMCTRL_MSR_R2_24K << MXC_F_FCR_TRIMCTRL_MSR_R2_POS) /**< TRIMCTRL_MSR_R2_24K Setting */

/**@} end of group FCR_TRIMCTRL_Register */

/**
 * @ingroup  fcr_registers
 * @defgroup FCR_ERFOKS FCR_ERFOKS
 * @brief    ERFO Kick Start Register.
 * @{
 */
#define MXC_F_FCR_ERFOKS_CTRL_POS                      0 /**< ERFOKS_CTRL Position */
#define MXC_F_FCR_ERFOKS_CTRL                          ((uint32_t)(0xFFFFUL << MXC_F_FCR_ERFOKS_CTRL_POS)) /**< ERFOKS_CTRL Mask */

/**@} end of group FCR_ERFOKS_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32572_INCLUDE_FCR_REGS_H_
