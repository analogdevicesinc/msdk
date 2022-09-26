/**
 * @file    sir_tm_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the SIR Peripheral Module.
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

#ifndef _SIR_TM_REGS_H_
#define _SIR_TM_REGS_H_

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
 * @ingroup     sir
 * @defgroup    sir_registers Registers
 * @brief       Registers, Bit Masks and Bit Positions for the SIR Peripheral Module.
 * @description System Initialization Registers (For Simulation)
 */

/**
 * @ingroup sir_registers
 * Structure type to access the SIR Registers.
 */
typedef struct {
    __IO uint32_t sistat;               /**< <tt>\b 0x00:<\tt> SIR SISTAT Register */
    __IO uint32_t erraddr;              /**< <tt>\b 0x04:<\tt> SIR ERRADDR Register */
    __IO uint32_t conf;                 /**< <tt>\b 0x08:<\tt> SIR CONF Register */
    __IO uint32_t conf1;                /**< <tt>\b 0x0C:<\tt> SIR CONF1 Register */
    __IO uint32_t shr4;                 /**< <tt>\b 0x10:<\tt> SIR SHR4 Register */
    __IO uint32_t shr5;                 /**< <tt>\b 0x14:<\tt> SIR SHR5 Register */
    __IO uint32_t shr6;                 /**< <tt>\b 0x18:<\tt> SIR SHR6 Register */
    __IO uint32_t shr7;                 /**< <tt>\b 0x1C:<\tt> SIR SHR7 Register */
    __IO uint32_t shr8;                 /**< <tt>\b 0x20:<\tt> SIR SHR8 Register */
    __IO uint32_t shr9;                 /**< <tt>\b 0x24:<\tt> SIR SHR9 Register */
    __IO uint32_t shr10;                /**< <tt>\b 0x28:<\tt> SIR SHR10 Register */
    __IO uint32_t shr11;                /**< <tt>\b 0x2C:<\tt> SIR SHR11 Register */
    __IO uint32_t shr12;                /**< <tt>\b 0x30:<\tt> SIR SHR12 Register */
    __IO uint32_t shr13;                /**< <tt>\b 0x34:<\tt> SIR SHR13 Register */
    __IO uint32_t shr14;                /**< <tt>\b 0x38:<\tt> SIR SHR14 Register */
    __IO uint32_t shr15;                /**< <tt>\b 0x3C:<\tt> SIR SHR15 Register */
    __IO uint32_t shr16;                /**< <tt>\b 0x40:<\tt> SIR SHR16 Register */
    __IO uint32_t shr17;                /**< <tt>\b 0x44:<\tt> SIR SHR17 Register */
    __IO uint32_t shr18;                /**< <tt>\b 0x48:<\tt> SIR SHR18 Register */
    __IO uint32_t shr19;                /**< <tt>\b 0x4C:<\tt> SIR SHR19 Register */
    __IO uint32_t shr20;                /**< <tt>\b 0x50:<\tt> SIR SHR20 Register */
    __IO uint32_t shr21;                /**< <tt>\b 0x54:<\tt> SIR SHR21 Register */
    __IO uint32_t shr22;                /**< <tt>\b 0x58:<\tt> SIR SHR22 Register */
    __IO uint32_t shr23;                /**< <tt>\b 0x5C:<\tt> SIR SHR23 Register */
    __IO uint32_t shr24;                /**< <tt>\b 0x60:<\tt> SIR SHR24 Register */
    __IO uint32_t shr25;                /**< <tt>\b 0x64:<\tt> SIR SHR25 Register */
    __I  uint32_t rsv0[38];
    __IO uint32_t funcstat;             /**< <tt>\b 0x0100:<\tt> SIR Function Status Register */
    __IO uint32_t secfuncstat;          /**< <tt>\b 0x0100:<\tt> SIR Secondary Function Status Register */
} mxc_sir_tm_regs_t;

/* Register offsets for module SIR */
/**
 * @ingroup    sir_registers
 * @defgroup   SIR_Register_Offsets Register Offsets
 * @brief      SIR Peripheral Register Offsets from the SIR Base Peripheral Address.
 * @{
 */
#define MXC_R_SIR_SISTAT                   ((uint32_t(0x00000000UL)
#define MXC_R_SIR_ERRADDR                  ((uint32_t(0x00000004UL)
#define MXC_R_SIR_CONF                     ((uint32_t(0x00000008UL)
#define MXC_R_SIR_CONF1                    ((uint32_t(0x0000000CUL)
#define MXC_R_SIR_SHR4                     ((uint32_t(0x00000010UL)
#define MXC_R_SIR_SHR5                     ((uint32_t(0x00000014UL)
#define MXC_R_SIR_SHR6                     ((uint32_t(0x00000018UL)
#define MXC_R_SIR_SHR7                     ((uint32_t(0x0000001CUL)
#define MXC_R_SIR_SHR8                     ((uint32_t(0x00000020UL)
#define MXC_R_SIR_SHR9                     ((uint32_t(0x00000024UL)
#define MXC_R_SIR_SHR10                    ((uint32_t(0x00000028UL)
#define MXC_R_SIR_SHR11                    ((uint32_t(0x0000002CUL)
#define MXC_R_SIR_SHR12                    ((uint32_t(0x00000030UL)
#define MXC_R_SIR_SHR13                    ((uint32_t(0x00000034UL)
#define MXC_R_SIR_SHR14                    ((uint32_t(0x00000038UL)
#define MXC_R_SIR_SHR15                    ((uint32_t(0x0000003CUL)
#define MXC_R_SIR_SHR16                    ((uint32_t(0x00000040UL)
#define MXC_R_SIR_SHR17                    ((uint32_t(0x00000044UL)
#define MXC_R_SIR_SHR18                    ((uint32_t(0x00000048UL)
#define MXC_R_SIR_SHR19                    ((uint32_t(0x0000004CUL)
#define MXC_R_SIR_SHR20                    ((uint32_t(0x00000050UL)
#define MXC_R_SIR_SHR21                    ((uint32_t(0x00000054UL)
#define MXC_R_SIR_SHR22                    ((uint32_t(0x00000058UL)
#define MXC_R_SIR_SHR23                    ((uint32_t(0x0000005CUL)
#define MXC_R_SIR_SHR24                    ((uint32_t(0x00000060UL)
#define MXC_R_SIR_SHR25                    ((uint32_t(0x00000064UL)
#define MXC_R_SIR_FUNCSTAT                 ((uint32_t(0x00000100UL)
#define MXC_R_SIR_SECFUNCSTAT              ((uint32_t(0x00000104UL)
/**@} end of group sir_registers */

/**
 * @ingroup  sir_registers
 * @defgroup SISTAT_Register
 * @brief    System Initialization Status Register
 * @{
 */
#define MXC_F_SIR_SISTAT_MAGIC_POS                     (0)
#define MXC_F_SIR_SISTAT_MAGIC                         (0x1 << MXC_F_SIR_SISTAT_MAGIC_POS)
#define MXC_V_SIR_SISTAT_MAGIC_MAGICNOTSET             (0x0)
#define MXC_S_SIR_SISTAT_MAGIC_MAGICNOTSET             (MXC_V_SIR_SISTAT_MAGIC_MAGICNOTSET << MXC_F_SIR_SISTAT_MAGIC_POS)
#define MXC_V_SIR_SISTAT_MAGIC_MAGICSET                (0x1)
#define MXC_S_SIR_SISTAT_MAGIC_MAGICSET                (MXC_V_SIR_SISTAT_MAGIC_MAGICSET << MXC_F_SIR_SISTAT_MAGIC_POS)

#define MXC_F_SIR_SISTAT_CRCERR_POS                    (1)
#define MXC_F_SIR_SISTAT_CRCERR                        (0x1 << MXC_F_SIR_SISTAT_CRCERR_POS)
#define MXC_V_SIR_SISTAT_CRCERR_NOERROR                (0x0)
#define MXC_S_SIR_SISTAT_CRCERR_NOERROR                (MXC_V_SIR_SISTAT_CRCERR_NOERROR << MXC_F_SIR_SISTAT_CRCERR_POS)
#define MXC_V_SIR_SISTAT_CRCERR_ERROR                  (0x1)
#define MXC_S_SIR_SISTAT_CRCERR_ERROR                  (MXC_V_SIR_SISTAT_CRCERR_ERROR << MXC_F_SIR_SISTAT_CRCERR_POS)

#define MXC_F_SIR_FUNCSTAT_FPU_EN_POS                  (0)
#define MXC_F_SIR_FUNCSTAT_FPU_EN                      (0x1 << MXC_F_SIR_FUNCSTAT_FPU_EN_POS)
#define MXC_V_SIR_FUNCSTAT_FPU_EN_EN                   (0x1)
#define MXC_S_SIR_FUNCSTAT_FPU_EN_EN                   (MXC_V_SIR_FUNCSTAT_FPU_EN_EN << MXC_F_SIR_FUNCSTAT_FPU_EN_POS)
#define MXC_V_SIR_FUNCSTAT_FPU_EN_DIS                  (0x0)
#define MXC_S_SIR_FUNCSTAT_FPU_EN_DIS                  (MXC_V_SIR_FUNCSTAT_FPU_EN_DIS << MXC_F_SIR_FUNCSTAT_FPU_EN_POS)

#define MXC_F_SIR_FUNCSTAT_USB_EN_POS                  (1)
#define MXC_F_SIR_FUNCSTAT_USB_EN                      (0x1 << MXC_F_SIR_FUNCSTAT_USB_EN_POS)
#define MXC_V_SIR_FUNCSTAT_USB_EN_EN                   (0x1)
#define MXC_S_SIR_FUNCSTAT_USB_EN_EN                   (MXC_V_SIR_FUNCSTAT_USB_EN_EN << MXC_F_SIR_FUNCSTAT_USB_EN_POS)
#define MXC_V_SIR_FUNCSTAT_USB_EN_DIS                  (0x0)
#define MXC_S_SIR_FUNCSTAT_USB_EN_DIS                  (MXC_V_SIR_FUNCSTAT_USB_EN_DIS << MXC_F_SIR_FUNCSTAT_USB_EN_POS)

#define MXC_F_SIR_FUNCSTAT_ADC_EN_POS                  (2)
#define MXC_F_SIR_FUNCSTAT_ADC_EN                      (0x1 << MXC_F_SIR_FUNCSTAT_ADC_EN_POS)
#define MXC_V_SIR_FUNCSTAT_ADC_EN_EN                   (0x1)
#define MXC_S_SIR_FUNCSTAT_ADC_EN_EN                   (MXC_V_SIR_FUNCSTAT_ADC_EN_EN << MXC_F_SIR_FUNCSTAT_ADC_EN_POS)
#define MXC_V_SIR_FUNCSTAT_ADC_EN_DIS                  (0x0)
#define MXC_S_SIR_FUNCSTAT_ADC_EN_DIS                  (MXC_V_SIR_FUNCSTAT_ADC_EN_DIS << MXC_F_SIR_FUNCSTAT_ADC_EN_POS)

#define MXC_F_SIR_FUNCSTAT_XIP_EN_POS                  (3)
#define MXC_F_SIR_FUNCSTAT_XIP_EN                      (0x1 << MXC_F_SIR_FUNCSTAT_XIP_EN_POS)
#define MXC_V_SIR_FUNCSTAT_XIP_EN_EN                   (0x1)
#define MXC_S_SIR_FUNCSTAT_XIP_EN_EN                   (MXC_V_SIR_FUNCSTAT_XIP_EN_EN << MXC_F_SIR_FUNCSTAT_XIP_EN_POS)
#define MXC_V_SIR_FUNCSTAT_XIP_EN_DIS                  (0x0)
#define MXC_S_SIR_FUNCSTAT_XIP_EN_DIS                  (MXC_V_SIR_FUNCSTAT_XIP_EN_DIS << MXC_F_SIR_FUNCSTAT_XIP_EN_POS)

#define MXC_F_SIR_FUNCSTAT_PBM_EN_POS                  (4)
#define MXC_F_SIR_FUNCSTAT_PBM_EN                      (0x1 << MXC_F_SIR_FUNCSTAT_PBM_EN_POS)
#define MXC_V_SIR_FUNCSTAT_PBM_EN_EN                   (0x1)
#define MXC_S_SIR_FUNCSTAT_PBM_EN_EN                   (MXC_V_SIR_FUNCSTAT_PBM_EN_EN << MXC_F_SIR_FUNCSTAT_PBM_EN_POS)
#define MXC_V_SIR_FUNCSTAT_PBM_EN_DIS                  (0x0)
#define MXC_S_SIR_FUNCSTAT_PBM_EN_DIS                  (MXC_V_SIR_FUNCSTAT_PBM_EN_DIS << MXC_F_SIR_FUNCSTAT_PBM_EN_POS)

#define MXC_F_SIR_FUNCSTAT_HBC_EN_POS                  (5)
#define MXC_F_SIR_FUNCSTAT_HBC_EN                      (0x1 << MXC_F_SIR_FUNCSTAT_HBC_EN_POS)
#define MXC_V_SIR_FUNCSTAT_HBC_EN_EN                   (0x1)
#define MXC_S_SIR_FUNCSTAT_HBC_EN_EN                   (MXC_V_SIR_FUNCSTAT_HBC_EN_EN << MXC_F_SIR_FUNCSTAT_HBC_EN_POS)
#define MXC_V_SIR_FUNCSTAT_HBC_EN_DIS                  (0x0)
#define MXC_S_SIR_FUNCSTAT_HBC_EN_DIS                  (MXC_V_SIR_FUNCSTAT_HBC_EN_DIS << MXC_F_SIR_FUNCSTAT_HBC_EN_POS)

#define MXC_F_SIR_FUNCSTAT_SDIO_EN_POS                 (6)
#define MXC_F_SIR_FUNCSTAT_SDIO_EN                     (0x1 << MXC_F_SIR_FUNCSTAT_SDIO_EN_POS)
#define MXC_V_SIR_FUNCSTAT_SDIO_EN_EN                  (0x1)
#define MXC_S_SIR_FUNCSTAT_SDIO_EN_EN                  (MXC_V_SIR_FUNCSTAT_SDIO_EN_EN << MXC_F_SIR_FUNCSTAT_SDIO_EN_POS)
#define MXC_V_SIR_FUNCSTAT_SDIO_EN_DIS                 (0x0)
#define MXC_S_SIR_FUNCSTAT_SDIO_EN_DIS                 (MXC_V_SIR_FUNCSTAT_SDIO_EN_DIS << MXC_F_SIR_FUNCSTAT_SDIO_EN_POS)

#define MXC_F_SIR_FUNCSTAT_SMPHR_EN_POS                (7)
#define MXC_F_SIR_FUNCSTAT_SMPHR_EN                    (0x1 << MXC_F_SIR_FUNCSTAT_SMPHR_EN_POS)
#define MXC_V_SIR_FUNCSTAT_SMPHR_EN_EN                 (0x1)
#define MXC_S_SIR_FUNCSTAT_SMPHR_EN_EN                 (MXC_V_SIR_FUNCSTAT_SMPHR_EN_EN << MXC_F_SIR_FUNCSTAT_SMPHR_EN_POS)
#define MXC_V_SIR_FUNCSTAT_SMPHR_EN_DIS                (0x0)
#define MXC_S_SIR_FUNCSTAT_SMPHR_EN_DIS                (MXC_V_SIR_FUNCSTAT_SMPHR_EN_DIS << MXC_F_SIR_FUNCSTAT_SMPHR_EN_POS)

#define MXC_F_SIR_FUNCSTAT_DCACHE_EN_POS               (8)
#define MXC_F_SIR_FUNCSTAT_DCACHE_EN                   (0x1 << MXC_F_SIR_FUNCSTAT_DCACHE_EN_POS)
#define MXC_V_SIR_FUNCSTAT_DCACHE_EN_EN                (0x1)
#define MXC_S_SIR_FUNCSTAT_DCACHE_EN_EN                (MXC_V_SIR_FUNCSTAT_DCACHE_EN_EN << MXC_F_SIR_FUNCSTAT_DCACHE_EN_POS)
#define MXC_V_SIR_FUNCSTAT_DCACHE_EN_DIS               (0x0)
#define MXC_S_SIR_FUNCSTAT_DCACHE_EN_DIS               (MXC_V_SIR_FUNCSTAT_DCACHE_EN_DIS << MXC_F_SIR_FUNCSTAT_DCACHE_EN_POS)

#define MXC_F_SIR_SECFUNCSTAT_SECBOOT_EN_POS           (0)
#define MXC_F_SIR_SECFUNCSTAT_SECBOOT_EN               (0x1 << MXC_F_SIR_SECFUNCSTAT_SECBOOT_EN_POS)
#define MXC_V_SIR_SECFUNCSTAT_SECBOOT_EN_EN            (0x1)
#define MXC_S_SIR_SECFUNCSTAT_SECBOOT_EN_EN            (MXC_V_SIR_SECFUNCSTAT_SECBOOT_EN_EN << MXC_F_SIR_SECFUNCSTAT_SECBOOT_EN_POS)
#define MXC_V_SIR_SECFUNCSTAT_SECBOOT_EN_DIS           (0x0)
#define MXC_S_SIR_SECFUNCSTAT_SECBOOT_EN_DIS           (MXC_V_SIR_SECFUNCSTAT_SECBOOT_EN_DIS << MXC_F_SIR_SECFUNCSTAT_SECBOOT_EN_POS)

#define MXC_F_SIR_SECFUNCSTAT_SERLOAD_EN_POS           (1)
#define MXC_F_SIR_SECFUNCSTAT_SERLOAD_EN               (0x1 << MXC_F_SIR_SECFUNCSTAT_SERLOAD_EN_POS)
#define MXC_V_SIR_SECFUNCSTAT_SERLOAD_EN_EN            (0x1)
#define MXC_S_SIR_SECFUNCSTAT_SERLOAD_EN_EN            (MXC_V_SIR_SECFUNCSTAT_SERLOAD_EN_EN << MXC_F_SIR_SECFUNCSTAT_SERLOAD_EN_POS)
#define MXC_V_SIR_SECFUNCSTAT_SERLOAD_EN_DIS           (0x0)
#define MXC_S_SIR_SECFUNCSTAT_SERLOAD_EN_DIS           (MXC_V_SIR_SECFUNCSTAT_SERLOAD_EN_DIS << MXC_F_SIR_SECFUNCSTAT_SERLOAD_EN_POS)

#define MXC_F_SIR_SECFUNCSTAT_TRNG_EN_POS              (2)
#define MXC_F_SIR_SECFUNCSTAT_TRNG_EN                  (0x1 << MXC_F_SIR_SECFUNCSTAT_TRNG_EN_POS)
#define MXC_V_SIR_SECFUNCSTAT_TRNG_EN_EN               (0x1)
#define MXC_S_SIR_SECFUNCSTAT_TRNG_EN_EN               (MXC_V_SIR_SECFUNCSTAT_TRNG_EN_EN << MXC_F_SIR_SECFUNCSTAT_TRNG_EN_POS)
#define MXC_V_SIR_SECFUNCSTAT_TRNG_EN_DIS              (0x0)
#define MXC_S_SIR_SECFUNCSTAT_TRNG_EN_DIS              (MXC_V_SIR_SECFUNCSTAT_TRNG_EN_DIS << MXC_F_SIR_SECFUNCSTAT_TRNG_EN_POS)

#define MXC_F_SIR_SECFUNCSTAT_AES_EN_POS               (3)
#define MXC_F_SIR_SECFUNCSTAT_AES_EN                   (0x1 << MXC_F_SIR_SECFUNCSTAT_AES_EN_POS)
#define MXC_V_SIR_SECFUNCSTAT_AES_EN_EN                (0x1)
#define MXC_S_SIR_SECFUNCSTAT_AES_EN_EN                (MXC_V_SIR_SECFUNCSTAT_AES_EN_EN << MXC_F_SIR_SECFUNCSTAT_AES_EN_POS)
#define MXC_V_SIR_SECFUNCSTAT_AES_EN_DIS               (0x0)
#define MXC_S_SIR_SECFUNCSTAT_AES_EN_DIS               (MXC_V_SIR_SECFUNCSTAT_AES_EN_DIS << MXC_F_SIR_SECFUNCSTAT_AES_EN_POS)

#define MXC_F_SIR_SECFUNCSTAT_SHA_EN_POS               (4)
#define MXC_F_SIR_SECFUNCSTAT_SHA_EN                   (0x1 << MXC_F_SIR_SECFUNCSTAT_SHA_EN_POS)
#define MXC_V_SIR_SECFUNCSTAT_SHA_EN_EN                (0x1)
#define MXC_S_SIR_SECFUNCSTAT_SHA_EN_EN                (MXC_V_SIR_SECFUNCSTAT_SHA_EN_EN << MXC_F_SIR_SECFUNCSTAT_SHA_EN_POS)
#define MXC_V_SIR_SECFUNCSTAT_SHA_EN_DIS               (0x0)
#define MXC_S_SIR_SECFUNCSTAT_SHA_EN_DIS               (MXC_V_SIR_SECFUNCSTAT_SHA_EN_DIS << MXC_F_SIR_SECFUNCSTAT_SHA_EN_POS)

#define MXC_F_SIR_SECFUNCSTAT_MAA_EN_POS               (5)
#define MXC_F_SIR_SECFUNCSTAT_MAA_EN                   (0x1 << MXC_F_SIR_SECFUNCSTAT_MAA_EN_POS)
#define MXC_V_SIR_SECFUNCSTAT_MAA_EN_EN                (0x1)
#define MXC_S_SIR_SECFUNCSTAT_MAA_EN_EN                (MXC_V_SIR_SECFUNCSTAT_MAA_EN_EN << MXC_F_SIR_SECFUNCSTAT_MAA_EN_POS)
#define MXC_V_SIR_SECFUNCSTAT_MAA_EN_DIS               (0x0)
#define MXC_S_SIR_SECFUNCSTAT_MAA_EN_DIS               (MXC_V_SIR_SECFUNCSTAT_MAA_EN_DIS << MXC_F_SIR_SECFUNCSTAT_MAA_EN_POS)


/**@} end of group SISTAT_Register */

/*******************************************************************************/
/*                                                                         SIR */
#define MXC_BASE_SIR_TM                  ((uint32_t)0x40000400UL)
#define MXC_SIR_TM                       ((mxc_sir_tm_regs_t*)MXC_BASE_SIR_TM)
#define MXC_SIR_TM_INSTANCES             (1)

#ifdef __cplusplus
}
#endif

#endif /* _SIR_TM_REGS_H_ */

