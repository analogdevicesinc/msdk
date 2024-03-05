/**
 * @file    simo_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the SIMO Peripheral Module.
 * @note    This file is @generated.
 * @ingroup simo_registers
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32655_INCLUDE_SIMO_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32655_INCLUDE_SIMO_REGS_H_

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
 * @ingroup     simo
 * @defgroup    simo_registers SIMO_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the SIMO Peripheral Module.
 * @details     Single Inductor Multiple Output Switching Converter
 */

/**
 * @ingroup simo_registers
 * Structure type to access the SIMO Registers.
 */
typedef struct {
    __R  uint32_t rsv_0x0;
    __IO uint32_t vrego_a;              /**< <tt>\b 0x0004:</tt> SIMO VREGO_A Register */
    __IO uint32_t vrego_b;              /**< <tt>\b 0x0008:</tt> SIMO VREGO_B Register */
    __IO uint32_t vrego_c;              /**< <tt>\b 0x000C:</tt> SIMO VREGO_C Register */
    __IO uint32_t vrego_d;              /**< <tt>\b 0x0010:</tt> SIMO VREGO_D Register */
    __IO uint32_t ipka;                 /**< <tt>\b 0x0014:</tt> SIMO IPKA Register */
    __IO uint32_t ipkb;                 /**< <tt>\b 0x0018:</tt> SIMO IPKB Register */
    __IO uint32_t maxton;               /**< <tt>\b 0x001C:</tt> SIMO MAXTON Register */
    __I  uint32_t iload_a;              /**< <tt>\b 0x0020:</tt> SIMO ILOAD_A Register */
    __I  uint32_t iload_b;              /**< <tt>\b 0x0024:</tt> SIMO ILOAD_B Register */
    __I  uint32_t iload_c;              /**< <tt>\b 0x0028:</tt> SIMO ILOAD_C Register */
    __I  uint32_t iload_d;              /**< <tt>\b 0x002C:</tt> SIMO ILOAD_D Register */
    __IO uint32_t buck_alert_thr_a;     /**< <tt>\b 0x0030:</tt> SIMO BUCK_ALERT_THR_A Register */
    __IO uint32_t buck_alert_thr_b;     /**< <tt>\b 0x0034:</tt> SIMO BUCK_ALERT_THR_B Register */
    __IO uint32_t buck_alert_thr_c;     /**< <tt>\b 0x0038:</tt> SIMO BUCK_ALERT_THR_C Register */
    __IO uint32_t buck_alert_thr_d;     /**< <tt>\b 0x003C:</tt> SIMO BUCK_ALERT_THR_D Register */
    __I  uint32_t buck_out_ready;       /**< <tt>\b 0x0040:</tt> SIMO BUCK_OUT_READY Register */
    __I  uint32_t zero_cross_cal_a;     /**< <tt>\b 0x0044:</tt> SIMO ZERO_CROSS_CAL_A Register */
    __I  uint32_t zero_cross_cal_b;     /**< <tt>\b 0x0048:</tt> SIMO ZERO_CROSS_CAL_B Register */
    __I  uint32_t zero_cross_cal_c;     /**< <tt>\b 0x004C:</tt> SIMO ZERO_CROSS_CAL_C Register */
    __I  uint32_t zero_cross_cal_d;     /**< <tt>\b 0x0050:</tt> SIMO ZERO_CROSS_CAL_D Register */
} mxc_simo_regs_t;

/* Register offsets for module SIMO */
/**
 * @ingroup    simo_registers
 * @defgroup   SIMO_Register_Offsets Register Offsets
 * @brief      SIMO Peripheral Register Offsets from the SIMO Base Peripheral Address.
 * @{
 */
#define MXC_R_SIMO_VREGO_A                 ((uint32_t)0x00000004UL) /**< Offset from SIMO Base Address: <tt> 0x0004</tt> */
#define MXC_R_SIMO_VREGO_B                 ((uint32_t)0x00000008UL) /**< Offset from SIMO Base Address: <tt> 0x0008</tt> */
#define MXC_R_SIMO_VREGO_C                 ((uint32_t)0x0000000CUL) /**< Offset from SIMO Base Address: <tt> 0x000C</tt> */
#define MXC_R_SIMO_VREGO_D                 ((uint32_t)0x00000010UL) /**< Offset from SIMO Base Address: <tt> 0x0010</tt> */
#define MXC_R_SIMO_IPKA                    ((uint32_t)0x00000014UL) /**< Offset from SIMO Base Address: <tt> 0x0014</tt> */
#define MXC_R_SIMO_IPKB                    ((uint32_t)0x00000018UL) /**< Offset from SIMO Base Address: <tt> 0x0018</tt> */
#define MXC_R_SIMO_MAXTON                  ((uint32_t)0x0000001CUL) /**< Offset from SIMO Base Address: <tt> 0x001C</tt> */
#define MXC_R_SIMO_ILOAD_A                 ((uint32_t)0x00000020UL) /**< Offset from SIMO Base Address: <tt> 0x0020</tt> */
#define MXC_R_SIMO_ILOAD_B                 ((uint32_t)0x00000024UL) /**< Offset from SIMO Base Address: <tt> 0x0024</tt> */
#define MXC_R_SIMO_ILOAD_C                 ((uint32_t)0x00000028UL) /**< Offset from SIMO Base Address: <tt> 0x0028</tt> */
#define MXC_R_SIMO_ILOAD_D                 ((uint32_t)0x0000002CUL) /**< Offset from SIMO Base Address: <tt> 0x002C</tt> */
#define MXC_R_SIMO_BUCK_ALERT_THR_A        ((uint32_t)0x00000030UL) /**< Offset from SIMO Base Address: <tt> 0x0030</tt> */
#define MXC_R_SIMO_BUCK_ALERT_THR_B        ((uint32_t)0x00000034UL) /**< Offset from SIMO Base Address: <tt> 0x0034</tt> */
#define MXC_R_SIMO_BUCK_ALERT_THR_C        ((uint32_t)0x00000038UL) /**< Offset from SIMO Base Address: <tt> 0x0038</tt> */
#define MXC_R_SIMO_BUCK_ALERT_THR_D        ((uint32_t)0x0000003CUL) /**< Offset from SIMO Base Address: <tt> 0x003C</tt> */
#define MXC_R_SIMO_BUCK_OUT_READY          ((uint32_t)0x00000040UL) /**< Offset from SIMO Base Address: <tt> 0x0040</tt> */
#define MXC_R_SIMO_ZERO_CROSS_CAL_A        ((uint32_t)0x00000044UL) /**< Offset from SIMO Base Address: <tt> 0x0044</tt> */
#define MXC_R_SIMO_ZERO_CROSS_CAL_B        ((uint32_t)0x00000048UL) /**< Offset from SIMO Base Address: <tt> 0x0048</tt> */
#define MXC_R_SIMO_ZERO_CROSS_CAL_C        ((uint32_t)0x0000004CUL) /**< Offset from SIMO Base Address: <tt> 0x004C</tt> */
#define MXC_R_SIMO_ZERO_CROSS_CAL_D        ((uint32_t)0x00000050UL) /**< Offset from SIMO Base Address: <tt> 0x0050</tt> */
/**@} end of group simo_registers */

/**
 * @ingroup  simo_registers
 * @defgroup SIMO_VREGO_A SIMO_VREGO_A
 * @brief    Buck Voltage Regulator A Control Register
 * @{
 */
#define MXC_F_SIMO_VREGO_A_VSETA_POS                   0 /**< VREGO_A_VSETA Position */
#define MXC_F_SIMO_VREGO_A_VSETA                       ((uint32_t)(0x7FUL << MXC_F_SIMO_VREGO_A_VSETA_POS)) /**< VREGO_A_VSETA Mask */

#define MXC_F_SIMO_VREGO_A_RANGEA_POS                  7 /**< VREGO_A_RANGEA Position */
#define MXC_F_SIMO_VREGO_A_RANGEA                      ((uint32_t)(0x1UL << MXC_F_SIMO_VREGO_A_RANGEA_POS)) /**< VREGO_A_RANGEA Mask */

/**@} end of group SIMO_VREGO_A_Register */

/**
 * @ingroup  simo_registers
 * @defgroup SIMO_VREGO_B SIMO_VREGO_B
 * @brief    Buck Voltage Regulator B Control Register
 * @{
 */
#define MXC_F_SIMO_VREGO_B_VSETB_POS                   0 /**< VREGO_B_VSETB Position */
#define MXC_F_SIMO_VREGO_B_VSETB                       ((uint32_t)(0x7FUL << MXC_F_SIMO_VREGO_B_VSETB_POS)) /**< VREGO_B_VSETB Mask */

#define MXC_F_SIMO_VREGO_B_RANGEB_POS                  7 /**< VREGO_B_RANGEB Position */
#define MXC_F_SIMO_VREGO_B_RANGEB                      ((uint32_t)(0x1UL << MXC_F_SIMO_VREGO_B_RANGEB_POS)) /**< VREGO_B_RANGEB Mask */

/**@} end of group SIMO_VREGO_B_Register */

/**
 * @ingroup  simo_registers
 * @defgroup SIMO_VREGO_C SIMO_VREGO_C
 * @brief    Buck Voltage Regulator C Control Register
 * @{
 */
#define MXC_F_SIMO_VREGO_C_VSETC_POS                   0 /**< VREGO_C_VSETC Position */
#define MXC_F_SIMO_VREGO_C_VSETC                       ((uint32_t)(0x7FUL << MXC_F_SIMO_VREGO_C_VSETC_POS)) /**< VREGO_C_VSETC Mask */

#define MXC_F_SIMO_VREGO_C_RANGEC_POS                  7 /**< VREGO_C_RANGEC Position */
#define MXC_F_SIMO_VREGO_C_RANGEC                      ((uint32_t)(0x1UL << MXC_F_SIMO_VREGO_C_RANGEC_POS)) /**< VREGO_C_RANGEC Mask */

/**@} end of group SIMO_VREGO_C_Register */

/**
 * @ingroup  simo_registers
 * @defgroup SIMO_VREGO_D SIMO_VREGO_D
 * @brief    Buck Voltage Regulator D Control Register
 * @{
 */
#define MXC_F_SIMO_VREGO_D_VSETD_POS                   0 /**< VREGO_D_VSETD Position */
#define MXC_F_SIMO_VREGO_D_VSETD                       ((uint32_t)(0x7FUL << MXC_F_SIMO_VREGO_D_VSETD_POS)) /**< VREGO_D_VSETD Mask */

#define MXC_F_SIMO_VREGO_D_RANGED_POS                  7 /**< VREGO_D_RANGED Position */
#define MXC_F_SIMO_VREGO_D_RANGED                      ((uint32_t)(0x1UL << MXC_F_SIMO_VREGO_D_RANGED_POS)) /**< VREGO_D_RANGED Mask */

/**@} end of group SIMO_VREGO_D_Register */

/**
 * @ingroup  simo_registers
 * @defgroup SIMO_IPKA SIMO_IPKA
 * @brief    High Side FET Peak Current VREGO_A/VREGO_B Register
 * @{
 */
#define MXC_F_SIMO_IPKA_IPKSETA_POS                    0 /**< IPKA_IPKSETA Position */
#define MXC_F_SIMO_IPKA_IPKSETA                        ((uint32_t)(0xFUL << MXC_F_SIMO_IPKA_IPKSETA_POS)) /**< IPKA_IPKSETA Mask */

#define MXC_F_SIMO_IPKA_IPKSETB_POS                    4 /**< IPKA_IPKSETB Position */
#define MXC_F_SIMO_IPKA_IPKSETB                        ((uint32_t)(0xFUL << MXC_F_SIMO_IPKA_IPKSETB_POS)) /**< IPKA_IPKSETB Mask */

/**@} end of group SIMO_IPKA_Register */

/**
 * @ingroup  simo_registers
 * @defgroup SIMO_IPKB SIMO_IPKB
 * @brief    High Side FET Peak Current VREGO_C/VREGO_D Register
 * @{
 */
#define MXC_F_SIMO_IPKB_IPKSETC_POS                    0 /**< IPKB_IPKSETC Position */
#define MXC_F_SIMO_IPKB_IPKSETC                        ((uint32_t)(0xFUL << MXC_F_SIMO_IPKB_IPKSETC_POS)) /**< IPKB_IPKSETC Mask */

#define MXC_F_SIMO_IPKB_IPKSETD_POS                    4 /**< IPKB_IPKSETD Position */
#define MXC_F_SIMO_IPKB_IPKSETD                        ((uint32_t)(0xFUL << MXC_F_SIMO_IPKB_IPKSETD_POS)) /**< IPKB_IPKSETD Mask */

/**@} end of group SIMO_IPKB_Register */

/**
 * @ingroup  simo_registers
 * @defgroup SIMO_MAXTON SIMO_MAXTON
 * @brief    Maximum High Side FET Time On Register
 * @{
 */
#define MXC_F_SIMO_MAXTON_TONSET_POS                   0 /**< MAXTON_TONSET Position */
#define MXC_F_SIMO_MAXTON_TONSET                       ((uint32_t)(0xFUL << MXC_F_SIMO_MAXTON_TONSET_POS)) /**< MAXTON_TONSET Mask */

/**@} end of group SIMO_MAXTON_Register */

/**
 * @ingroup  simo_registers
 * @defgroup SIMO_ILOAD_A SIMO_ILOAD_A
 * @brief    Buck Cycle Count VREGO_A Register
 * @{
 */
#define MXC_F_SIMO_ILOAD_A_ILOADA_POS                  0 /**< ILOAD_A_ILOADA Position */
#define MXC_F_SIMO_ILOAD_A_ILOADA                      ((uint32_t)(0xFFUL << MXC_F_SIMO_ILOAD_A_ILOADA_POS)) /**< ILOAD_A_ILOADA Mask */

/**@} end of group SIMO_ILOAD_A_Register */

/**
 * @ingroup  simo_registers
 * @defgroup SIMO_ILOAD_B SIMO_ILOAD_B
 * @brief    Buck Cycle Count VREGO_B Register
 * @{
 */
#define MXC_F_SIMO_ILOAD_B_ILOADB_POS                  0 /**< ILOAD_B_ILOADB Position */
#define MXC_F_SIMO_ILOAD_B_ILOADB                      ((uint32_t)(0xFFUL << MXC_F_SIMO_ILOAD_B_ILOADB_POS)) /**< ILOAD_B_ILOADB Mask */

/**@} end of group SIMO_ILOAD_B_Register */

/**
 * @ingroup  simo_registers
 * @defgroup SIMO_ILOAD_C SIMO_ILOAD_C
 * @brief    Buck Cycle Count VREGO_C Register
 * @{
 */
#define MXC_F_SIMO_ILOAD_C_ILOADC_POS                  0 /**< ILOAD_C_ILOADC Position */
#define MXC_F_SIMO_ILOAD_C_ILOADC                      ((uint32_t)(0xFFUL << MXC_F_SIMO_ILOAD_C_ILOADC_POS)) /**< ILOAD_C_ILOADC Mask */

/**@} end of group SIMO_ILOAD_C_Register */

/**
 * @ingroup  simo_registers
 * @defgroup SIMO_ILOAD_D SIMO_ILOAD_D
 * @brief    Buck Cycle Count VREGO_D Register
 * @{
 */
#define MXC_F_SIMO_ILOAD_D_ILOADD_POS                  0 /**< ILOAD_D_ILOADD Position */
#define MXC_F_SIMO_ILOAD_D_ILOADD                      ((uint32_t)(0xFFUL << MXC_F_SIMO_ILOAD_D_ILOADD_POS)) /**< ILOAD_D_ILOADD Mask */

/**@} end of group SIMO_ILOAD_D_Register */

/**
 * @ingroup  simo_registers
 * @defgroup SIMO_BUCK_ALERT_THR_A SIMO_BUCK_ALERT_THR_A
 * @brief    Buck Cycle Count Alert VERGO_A Register
 * @{
 */
#define MXC_F_SIMO_BUCK_ALERT_THR_A_BUCKTHRA_POS       0 /**< BUCK_ALERT_THR_A_BUCKTHRA Position */
#define MXC_F_SIMO_BUCK_ALERT_THR_A_BUCKTHRA           ((uint32_t)(0xFFUL << MXC_F_SIMO_BUCK_ALERT_THR_A_BUCKTHRA_POS)) /**< BUCK_ALERT_THR_A_BUCKTHRA Mask */

/**@} end of group SIMO_BUCK_ALERT_THR_A_Register */

/**
 * @ingroup  simo_registers
 * @defgroup SIMO_BUCK_ALERT_THR_B SIMO_BUCK_ALERT_THR_B
 * @brief    Buck Cycle Count Alert VERGO_B Register
 * @{
 */
#define MXC_F_SIMO_BUCK_ALERT_THR_B_BUCKTHRB_POS       0 /**< BUCK_ALERT_THR_B_BUCKTHRB Position */
#define MXC_F_SIMO_BUCK_ALERT_THR_B_BUCKTHRB           ((uint32_t)(0xFFUL << MXC_F_SIMO_BUCK_ALERT_THR_B_BUCKTHRB_POS)) /**< BUCK_ALERT_THR_B_BUCKTHRB Mask */

/**@} end of group SIMO_BUCK_ALERT_THR_B_Register */

/**
 * @ingroup  simo_registers
 * @defgroup SIMO_BUCK_ALERT_THR_C SIMO_BUCK_ALERT_THR_C
 * @brief    Buck Cycle Count Alert VERGO_C Register
 * @{
 */
#define MXC_F_SIMO_BUCK_ALERT_THR_C_BUCKTHRC_POS       0 /**< BUCK_ALERT_THR_C_BUCKTHRC Position */
#define MXC_F_SIMO_BUCK_ALERT_THR_C_BUCKTHRC           ((uint32_t)(0xFFUL << MXC_F_SIMO_BUCK_ALERT_THR_C_BUCKTHRC_POS)) /**< BUCK_ALERT_THR_C_BUCKTHRC Mask */

/**@} end of group SIMO_BUCK_ALERT_THR_C_Register */

/**
 * @ingroup  simo_registers
 * @defgroup SIMO_BUCK_ALERT_THR_D SIMO_BUCK_ALERT_THR_D
 * @brief    Buck Cycle Count Alert VERGO_D Register
 * @{
 */
#define MXC_F_SIMO_BUCK_ALERT_THR_D_BUCKTHRD_POS       0 /**< BUCK_ALERT_THR_D_BUCKTHRD Position */
#define MXC_F_SIMO_BUCK_ALERT_THR_D_BUCKTHRD           ((uint32_t)(0xFFUL << MXC_F_SIMO_BUCK_ALERT_THR_D_BUCKTHRD_POS)) /**< BUCK_ALERT_THR_D_BUCKTHRD Mask */

/**@} end of group SIMO_BUCK_ALERT_THR_D_Register */

/**
 * @ingroup  simo_registers
 * @defgroup SIMO_BUCK_OUT_READY SIMO_BUCK_OUT_READY
 * @brief    Buck Regulator Output Ready Register
 * @{
 */
#define MXC_F_SIMO_BUCK_OUT_READY_BUCKOUTRDYA_POS      0 /**< BUCK_OUT_READY_BUCKOUTRDYA Position */
#define MXC_F_SIMO_BUCK_OUT_READY_BUCKOUTRDYA          ((uint32_t)(0x1UL << MXC_F_SIMO_BUCK_OUT_READY_BUCKOUTRDYA_POS)) /**< BUCK_OUT_READY_BUCKOUTRDYA Mask */

#define MXC_F_SIMO_BUCK_OUT_READY_BUCKOUTRDYB_POS      1 /**< BUCK_OUT_READY_BUCKOUTRDYB Position */
#define MXC_F_SIMO_BUCK_OUT_READY_BUCKOUTRDYB          ((uint32_t)(0x1UL << MXC_F_SIMO_BUCK_OUT_READY_BUCKOUTRDYB_POS)) /**< BUCK_OUT_READY_BUCKOUTRDYB Mask */

#define MXC_F_SIMO_BUCK_OUT_READY_BUCKOUTRDYC_POS      2 /**< BUCK_OUT_READY_BUCKOUTRDYC Position */
#define MXC_F_SIMO_BUCK_OUT_READY_BUCKOUTRDYC          ((uint32_t)(0x1UL << MXC_F_SIMO_BUCK_OUT_READY_BUCKOUTRDYC_POS)) /**< BUCK_OUT_READY_BUCKOUTRDYC Mask */

#define MXC_F_SIMO_BUCK_OUT_READY_BUCKOUTRDYD_POS      3 /**< BUCK_OUT_READY_BUCKOUTRDYD Position */
#define MXC_F_SIMO_BUCK_OUT_READY_BUCKOUTRDYD          ((uint32_t)(0x1UL << MXC_F_SIMO_BUCK_OUT_READY_BUCKOUTRDYD_POS)) /**< BUCK_OUT_READY_BUCKOUTRDYD Mask */

/**@} end of group SIMO_BUCK_OUT_READY_Register */

/**
 * @ingroup  simo_registers
 * @defgroup SIMO_ZERO_CROSS_CAL_A SIMO_ZERO_CROSS_CAL_A
 * @brief    Zero Cross Calibration VERGO_A Register
 * @{
 */
#define MXC_F_SIMO_ZERO_CROSS_CAL_A_ZXCALA_POS         0 /**< ZERO_CROSS_CAL_A_ZXCALA Position */
#define MXC_F_SIMO_ZERO_CROSS_CAL_A_ZXCALA             ((uint32_t)(0xFUL << MXC_F_SIMO_ZERO_CROSS_CAL_A_ZXCALA_POS)) /**< ZERO_CROSS_CAL_A_ZXCALA Mask */

/**@} end of group SIMO_ZERO_CROSS_CAL_A_Register */

/**
 * @ingroup  simo_registers
 * @defgroup SIMO_ZERO_CROSS_CAL_B SIMO_ZERO_CROSS_CAL_B
 * @brief    Zero Cross Calibration VERGO_B Register
 * @{
 */
#define MXC_F_SIMO_ZERO_CROSS_CAL_B_ZXCALB_POS         0 /**< ZERO_CROSS_CAL_B_ZXCALB Position */
#define MXC_F_SIMO_ZERO_CROSS_CAL_B_ZXCALB             ((uint32_t)(0xFUL << MXC_F_SIMO_ZERO_CROSS_CAL_B_ZXCALB_POS)) /**< ZERO_CROSS_CAL_B_ZXCALB Mask */

/**@} end of group SIMO_ZERO_CROSS_CAL_B_Register */

/**
 * @ingroup  simo_registers
 * @defgroup SIMO_ZERO_CROSS_CAL_C SIMO_ZERO_CROSS_CAL_C
 * @brief    Zero Cross Calibration VERGO_C Register
 * @{
 */
#define MXC_F_SIMO_ZERO_CROSS_CAL_C_ZXCALC_POS         0 /**< ZERO_CROSS_CAL_C_ZXCALC Position */
#define MXC_F_SIMO_ZERO_CROSS_CAL_C_ZXCALC             ((uint32_t)(0xFUL << MXC_F_SIMO_ZERO_CROSS_CAL_C_ZXCALC_POS)) /**< ZERO_CROSS_CAL_C_ZXCALC Mask */

/**@} end of group SIMO_ZERO_CROSS_CAL_C_Register */

/**
 * @ingroup  simo_registers
 * @defgroup SIMO_ZERO_CROSS_CAL_D SIMO_ZERO_CROSS_CAL_D
 * @brief    Zero Cross Calibration VERGO_D Register
 * @{
 */
#define MXC_F_SIMO_ZERO_CROSS_CAL_D_ZXCALD_POS         0 /**< ZERO_CROSS_CAL_D_ZXCALD Position */
#define MXC_F_SIMO_ZERO_CROSS_CAL_D_ZXCALD             ((uint32_t)(0xFUL << MXC_F_SIMO_ZERO_CROSS_CAL_D_ZXCALD_POS)) /**< ZERO_CROSS_CAL_D_ZXCALD Mask */

/**@} end of group SIMO_ZERO_CROSS_CAL_D_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32655_INCLUDE_SIMO_REGS_H_
