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

#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32657_NSPC_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32657_NSPC_H_

#include "mxc_device.h"
#include "nspc_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup nspc NSPC
 * @ingroup periphlibs
 * @{
 */

/***** Definitions *****/

/**
 * @brief NSPC Peripheral Definition for setting the Privilege State.
 */
typedef enum {
    MXC_NSPC_PERIPH_GCR = MXC_S_NSPC_APBPRIV_PERIPH_GCR, /**< GCR */
    MXC_NSPC_PERIPH_SIR = MXC_S_NSPC_APBPRIV_PERIPH_SIR, /**< SIR */
    MXC_NSPC_PERIPH_FCR = MXC_S_NSPC_APBPRIV_PERIPH_FCR, /**< FCR */
    MXC_NSPC_PERIPH_WDT = MXC_S_NSPC_APBPRIV_PERIPH_WDT, /**< WDT */
    MXC_NSPC_PERIPH_AES = MXC_S_NSPC_APBPRIV_PERIPH_AES, /**< AES */
    MXC_NSPC_PERIPH_AESKEYS = MXC_S_NSPC_APBPRIV_PERIPH_AESKEYS, /**< AESKEYS */
    MXC_NSPC_PERIPH_GPIO0 = MXC_S_NSPC_APBPRIV_PERIPH_GPIO0, /**< GPIO0 */
    MXC_NSPC_PERIPH_CRC = MXC_S_NSPC_APBPRIV_PERIPH_CRC, /**< CRC */
    MXC_NSPC_PERIPH_TMR0 = MXC_S_NSPC_APBPRIV_PERIPH_TMR0, /**< TMR0 */
    MXC_NSPC_PERIPH_TMR1 = MXC_S_NSPC_APBPRIV_PERIPH_TMR1, /**< TMR1 */
    MXC_NSPC_PERIPH_TMR2 = MXC_S_NSPC_APBPRIV_PERIPH_TMR2, /**< TMR2 */
    MXC_NSPC_PERIPH_TMR3 = MXC_S_NSPC_APBPRIV_PERIPH_TMR3, /**< TMR3 */
    MXC_NSPC_PERIPH_TMR4 = MXC_S_NSPC_APBPRIV_PERIPH_TMR4, /**< TMR4 */
    MXC_NSPC_PERIPH_TMR5 = MXC_S_NSPC_APBPRIV_PERIPH_TMR5, /**< TMR5 */
    MXC_NSPC_PERIPH_I3C = MXC_S_NSPC_APBPRIV_PERIPH_I3C, /**< I3C */
    MXC_NSPC_PERIPH_UART = MXC_S_NSPC_APBPRIV_PERIPH_UART, /**< UART */
    MXC_NSPC_PERIPH_SPI = MXC_S_NSPC_APBPRIV_PERIPH_SPI, /**< SPI */
    MXC_NSPC_PERIPH_TRNG = MXC_S_NSPC_APBPRIV_PERIPH_TRNG, /**< TRNG */
    MXC_NSPC_PERIPH_BTLE_DBB = MXC_S_NSPC_APBPRIV_PERIPH_BTLE_DBB, /**< BTLE DBB */
    MXC_NSPC_PERIPH_BTLE_RFFE = MXC_S_NSPC_APBPRIV_PERIPH_BTLE_RFFE, /**< BTLE RFFE */
    MXC_NSPC_PERIPH_RSTZ = MXC_S_NSPC_APBPRIV_PERIPH_RSTZ, /**< RSTZ */
    MXC_NSPC_PERIPH_BOOST = MXC_S_NSPC_APBPRIV_PERIPH_BOOST, /**< BOOST */
    MXC_NSPC_PERIPH_TRIMSIR = MXC_S_NSPC_APBPRIV_PERIPH_TRIMSIR, /**< TRIMSIR */
    MXC_NSPC_PERIPH_RTC = MXC_S_NSPC_APBPRIV_PERIPH_RTC, /**< RTC */
    MXC_NSPC_PERIPH_WUT0 = MXC_S_NSPC_APBPRIV_PERIPH_WUT0, /**< WUT0 */
    MXC_NSPC_PERIPH_WUT1 = MXC_S_NSPC_APBPRIV_PERIPH_WUT1, /**< WUT1 */
    MXC_NSPC_PERIPH_PWRSEQ = MXC_S_NSPC_APBPRIV_PERIPH_PWRSEQ, /**< PWRSEQ */
    MXC_NSPC_PERIPH_MCR = MXC_S_NSPC_APBPRIV_PERIPH_MCR, /**< MCR */
    MXC_NSPC_PERIPH_ALL = MXC_S_NSPC_APBPRIV_PERIPH_ALL, /**< All */
} mxc_nspc_periph_t;

/** @brief Enumeration for ARM privilege settings. */
typedef enum {
    MXC_NSPC_PRIVILEGED = 0,
    MXC_NSPC_UNPRIVILEGED = 1,
} mxc_nspc_priv_t;

/***** Function Prototypes *****/

/**
 * @brief   Sets the privilege level for a Peripheral.
 * @param   periph   Enumeration for desired peripheral.
 * @param   priv     Enumeration for desired privilege level.
 */
void MXC_NSPC_SetPrivAccess(mxc_nspc_periph_t periph, mxc_nspc_priv_t priv);

/**
 * @brief   Sets the privilege level for Non-Secure DMA.
 * @param   priv     Enumeration for desired privilege level.
 */
void MXC_NSPC_DMA_SetPrivAccess(mxc_nspc_priv_t priv);

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32657_NSPC_H_
