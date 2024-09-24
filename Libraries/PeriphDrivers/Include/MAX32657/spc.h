/******************************************************************************
 *
 * Copyright (C) 2024 Analog Devices, Inc.
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

#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32657_SPC_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32657_SPC_H_

#include <stdint.h>
#include <stdbool.h>
#include "mxc_device.h"
#include "spc_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup spc SPC
 * @ingroup periphlibs
 * @{
 */

/***** Definitions *****/

/**
 * @brief SPC Peripheral Definition for setting the Security and Privilege State.
 *        The SPC_APBSEC register definitions are used, but this can also be used
 *        for the SPC_APBPRIV register as well since the peripheral fields are a
 *        one-to-one match.
 */
typedef enum {
    MXC_SPC_PERIPH_GCR = MXC_S_SPC_APBSEC_PERIPH_GCR, /**< GCR */
    MXC_SPC_PERIPH_SIR = MXC_S_SPC_APBSEC_PERIPH_SIR, /**< SIR */
    MXC_SPC_PERIPH_FCR = MXC_S_SPC_APBSEC_PERIPH_FCR, /**< FCR */
    MXC_SPC_PERIPH_WDT = MXC_S_SPC_APBSEC_PERIPH_WDT, /**< WDT */
    MXC_SPC_PERIPH_AES = MXC_S_SPC_APBSEC_PERIPH_AES, /**< AES */
    MXC_SPC_PERIPH_AESKEYS = MXC_S_SPC_APBSEC_PERIPH_AESKEYS, /**< AESKEYS */
    MXC_SPC_PERIPH_GPIO0 = MXC_S_SPC_APBSEC_PERIPH_GPIO0, /**< GPIO0 */
    MXC_SPC_PERIPH_CRC = MXC_S_SPC_APBSEC_PERIPH_CRC, /**< CRC */
    MXC_SPC_PERIPH_TMR0 = MXC_S_SPC_APBSEC_PERIPH_TMR0, /**< TMR0 */
    MXC_SPC_PERIPH_TMR1 = MXC_S_SPC_APBSEC_PERIPH_TMR1, /**< TMR1 */
    MXC_SPC_PERIPH_TMR2 = MXC_S_SPC_APBSEC_PERIPH_TMR2, /**< TMR2 */
    MXC_SPC_PERIPH_TMR3 = MXC_S_SPC_APBSEC_PERIPH_TMR3, /**< TMR3 */
    MXC_SPC_PERIPH_TMR4 = MXC_S_SPC_APBSEC_PERIPH_TMR4, /**< TMR4 */
    MXC_SPC_PERIPH_TMR5 = MXC_S_SPC_APBSEC_PERIPH_TMR5, /**< TMR5 */
    MXC_SPC_PERIPH_I3C = MXC_S_SPC_APBSEC_PERIPH_I3C, /**< I3C */
    MXC_SPC_PERIPH_UART = MXC_S_SPC_APBSEC_PERIPH_UART, /**< UART */
    MXC_SPC_PERIPH_SPI = MXC_S_SPC_APBSEC_PERIPH_SPI, /**< SPI */
    MXC_SPC_PERIPH_TRNG = MXC_S_SPC_APBSEC_PERIPH_TRNG, /**< TRNG */
    MXC_SPC_PERIPH_BTLE_DBB = MXC_S_SPC_APBSEC_PERIPH_BTLE_DBB, /**< BTLE DBB */
    MXC_SPC_PERIPH_BTLE_RFFE = MXC_S_SPC_APBSEC_PERIPH_BTLE_RFFE, /**< BTLE RFFE */
    MXC_SPC_PERIPH_RSTZ = MXC_S_SPC_APBSEC_PERIPH_RSTZ, /**< RSTZ */
    MXC_SPC_PERIPH_BOOST = MXC_S_SPC_APBSEC_PERIPH_BOOST, /**< BOOST */
    MXC_SPC_PERIPH_TRIMSIR = MXC_S_SPC_APBSEC_PERIPH_TRIMSIR, /**< TRIMSIR */
    MXC_SPC_PERIPH_RTC = MXC_S_SPC_APBSEC_PERIPH_RTC, /**< RTC */
    MXC_SPC_PERIPH_WUT0 = MXC_S_SPC_APBSEC_PERIPH_WUT0, /**< WUT0 */
    MXC_SPC_PERIPH_WUT1 = MXC_S_SPC_APBSEC_PERIPH_WUT1, /**< WUT1 */
    MXC_SPC_PERIPH_PWRSEQ = MXC_S_SPC_APBSEC_PERIPH_PWRSEQ, /**< PWRSEQ */
    MXC_SPC_PERIPH_MCR = MXC_S_SPC_APBSEC_PERIPH_MCR, /**< MCR */
    MXC_SPC_PERIPH_ALL = MXC_S_SPC_APBSEC_PERIPH_ALL, /**< All */
} mxc_spc_periph_t;

/** @brief Enumeration for ARM privilege settings. */
typedef enum {
    MXC_SPC_PRIVILEGED = 0,
    MXC_SPC_UNPRIVILEGED = 1,
} mxc_spc_priv_t;

/***** Function Prototypes *****/

/**
 * @brief   Locks the SPC registers related to setting the security
 *          or privilege states. Once locked, the affected SPC registers
 *          can not be unlocked until a reset.
 */
void MXC_SPC_Lock(void);

/**
 * @brief   Locks the Cortex-M33 Core registers:
 *              - AIRCR and VTOR_S
 *              - VTOR_NS
 *              - MPU_S
 *              - MPU_NS
 *              - SAU
 */
void MXC_SPC_Core_Lock(void);

/**
 * @brief   Unlocks the Cortex-M33 Core registers:
 *              - AIRCR and VTOR_S
 *              - VTOR_NS
 *              - MPU_S
 *              - MPU_NS
 *              - SAU
 */
void MXC_SPC_Core_UnLock(void);

/**
 * @brief   Sets the privilege level of a peripheral.
 * @param   periph  Enumeration for desired peripheral.
 * @param   priv    Enumeration for desired privilege level.
 */
void MXC_SPC_SetPrivAccess(mxc_spc_periph_t periph, mxc_spc_priv_t priv);

/**
 * @brief   Sets the peripheral to only be accessible from the Secure world.
 * @param   periph  Enumeration for desired peripheral.
 */
void MXC_SPC_SetSecure(mxc_spc_periph_t periph);

/**
 * @brief   Sets the peripheral to only be accessible from the Non-Secure world.
 * @param   periph  Enumeration for desired peripheral.
 */
void MXC_SPC_SetNonSecure(mxc_spc_periph_t periph);

/**
 * @brief   Sets the privilege level for Secure DMA.
 * @param   priv    Enumeration for desired privilege level.
 */
void MXC_SPC_DMA_SetPrivAccess(mxc_spc_priv_t priv);

/**
 * @brief   Sets the pins for a GPIO instance to only be accessible from the Secure world.
 * @param   gpio    Pointer to the GPIO port's registers.
 * @param   pins    mask of pins to set to Secure mode.
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_SPC_GPIO_SetSecure(mxc_gpio_regs_t *gpio, uint32_t pins);

/**
 * @brief   Sets the pins for a GPIO instance to only be accessible from the Non-Secure world.
 * @param   gpio    Pointer to the GPIO port's registers.
 * @param   pins    mask of pins to set to Non-Secure mode.
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_SPC_GPIO_SetNonSecure(mxc_gpio_regs_t *gpio, uint32_t pins);

/**
 * @brief   Enable the interrupts for each Memory Protection Controller (MPC) region.
 * @param   intr    mask of MPC regions to interrupt for.
 */
void MXC_SPC_MPC_EnableInt(uint32_t intr);

/**
 * @brief   Disable the interrupts for each Memory Protection Controller (MPC) region.
 * @param   intr    mask of MPC regions to disable interrupt for.
 */
void MXC_SPC_MPC_DisableInt(uint32_t intr);

/**
 * @brief   Gets the Memory Protection Controller (MPC) interrupt flags that are currently set.
 * @return  MPC interrupt flags.
 */
uint32_t MXC_SPC_MPC_GetFlags(void);

/**
 * @brief   Enable the interrupts for Peripheral Protection Controllers (PPC).
 * @param   intr    mask of PPC regions to disable interrupt for.
 */
void MXC_SPC_PPC_EnableInt(uint32_t intr);

/**
 * @brief   Disable the interrupts for Peripheral Protection Controllers (PPC).
 * @param   intr    mask of PPC regions to disable interrupt for.
 */
void MXC_SPC_PPC_DisableInt(uint32_t intr);

/**
 * @brief   Gets the Peripheral Protection Controller (PPC) interrupt flags
 *          that are currently set.
 * @return  PPC interrupt flags.
 */
uint32_t MXC_SPC_PPC_GetFlags(void);

/**
 * @brief   Clears the Peripheral Protection Controllers (PPC) Interrupt Flags.
 * @param   flags   mask of PPC interrupt flags to clear.
 */
void MXC_SPC_PPC_ClearFlags(uint32_t flags);

/**
 * @brief   Sets the CODE region to be Non-Secure Callable.
 * @param   isNSC   Option to set CODE region as non-secure callable (True) or not (False).
 */
void MXC_SPC_SetCode_NSC(bool isNSC);

/**
 * @brief   Sets the CODE region to be Non-Secure Callable.
 * @param   isNSC   Option to set CODE region as non-secure callable (True) or not (False).
 */
void MXC_SPC_SetSRAM_NSC(bool isNSC);

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32657_SPC_H_
