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

/**
 * @file       spc_me30.c
 * @brief      Secure Privilege Controller (SPC) Driver for the MAX32657 (ME30).
 * @details    This driver is used to control the security and privilege policy of data 
 *              flowing to/from peripherals though the APB Peripheral Proection
 *              Controllers (PPC).
 *             The SPC is only readable via Secure Privileged access.
 */

/**
 * SPC can only be accessed from the secure world.
 */
#if CONFIG_TRUSTED_EXECUTION_SECURE == 1

/**** Includes ****/
#include <stdbool.h>
#include <stdint.h>
#include "mxc_device.h"
#include "mxc_errors.h"
#include "spc.h"
#include "spc_regs.h"
#include "gpio.h"

/**** Definitions ****/

/**** Globals ****/

/**** Functions ****/

void MXC_SPC_Lock(void)
{
    // Note: Cannot unlock SPC registers except via a reset.
    MXC_SPC->ctrl |= MXC_F_SPC_CTRL_LOCK;
}

// TODO(SW): Need to verify if this prevents the Non-Secure world from
//  accessing the VTOR register. Note: the suffix '_NS' means that
//  the secure world is accessing a non-secure register
void MXC_SPC_Core_Lock(void)
{
    // Locks the Cortex-M33 Core Registers (VTOR, SAU, MPU, etc).
    MXC_SPC->m33lock |= MXC_F_SPC_M33LOCK_AIRCR_VTOR_S;
    MXC_SPC->m33lock |= MXC_F_SPC_M33LOCK_VTOR_NS;
    MXC_SPC->m33lock |= MXC_F_SPC_M33LOCK_MPU_S;
    MXC_SPC->m33lock |= MXC_F_SPC_M33LOCK_MPU_NS;
    MXC_SPC->m33lock |= MXC_F_SPC_M33LOCK_SAU;
}

void MXC_SPC_Core_UnLock(void)
{
    MXC_SPC->m33lock &= ~MXC_F_SPC_M33LOCK_AIRCR_VTOR_S;
    MXC_SPC->m33lock &= ~MXC_F_SPC_M33LOCK_VTOR_NS;
    MXC_SPC->m33lock &= ~MXC_F_SPC_M33LOCK_MPU_S;
    MXC_SPC->m33lock &= ~MXC_F_SPC_M33LOCK_MPU_NS;
    MXC_SPC->m33lock &= ~MXC_F_SPC_M33LOCK_SAU;
}

void MXC_SPC_SetPrivAccess(mxc_spc_periph_t periph, mxc_spc_priv_t priv)
{
    if (priv == MXC_SPC_UNPRIVILEGED) {
        MXC_SPC->apbpriv |= periph;
    } else {
        MXC_SPC->apbpriv &= ~periph;
    }
}

// TODO(SW): Check names.
void MXC_SPC_SetSecure(mxc_spc_periph_t periph)
{
    MXC_SPC->apbsec &= ~periph;
}

void MXC_SPC_SetNonSecure(mxc_spc_periph_t periph)
{
    MXC_SPC->apbsec |= periph;
}

// TODO(SW): Check function name.
void MXC_SPC_DMA_SetPrivAccess(mxc_spc_priv_t priv)
{
    if (priv == MXC_SPC_UNPRIVILEGED) {
        MXC_SPC->ahbmpriv |= MXC_F_SPC_AHBMPRIV_DMA;
    } else {
        MXC_SPC->ahbmpriv &= ~MXC_F_SPC_AHBMPRIV_DMA;
    }
}

int MXC_SPC_GPIO_SetSecure(mxc_gpio_regs_t *gpio, uint32_t pins)
{
    // Added GPIO instance as a parameter to follow convention of future devices that could
    //  potentially have more than one GPIO port.
    if (gpio == MXC_GPIO0) {
        MXC_SPC->gpio0 &= ~pins;
        return E_NO_ERROR;
    } else {
        return E_BAD_PARAM;
    }
}

int MXC_SPC_GPIO_SetNonSecure(mxc_gpio_regs_t *gpio, uint32_t pins)
{
    // Added GPIO instance as a parameter to follow convention of future devices that could
    //  potentially have more than one GPIO port.
    if (gpio == MXC_GPIO0) {
        MXC_SPC->gpio0 |= pins;
        return E_NO_ERROR;
    } else {
        return E_BAD_PARAM;
    }
}

void MXC_SPC_MPC_EnableInt(uint32_t intr)
{
    MXC_SPC->mpc_inten |= intr;
}

void MXC_SPC_MPC_DisableInt(uint32_t intr)
{
    MXC_SPC->mpc_inten &= ~intr;
}

uint32_t MXC_SPC_MPC_GetFlags(void)
{
    return MXC_SPC->mpc_status;
}

void MXC_SPC_PPC_EnableInt(uint32_t intr)
{
    MXC_SPC->ppc_inten |= intr;
}

void MXC_SPC_PPC_DisableInt(uint32_t intr)
{
    MXC_SPC->ppc_inten &= ~intr;
}

uint32_t MXC_SPC_PPC_GetFlags(void)
{
    return MXC_SPC->ppc_status;
}

void MXC_SPC_PPC_ClearFlags(uint32_t flags)
{
    MXC_SPC->ppc_intclr |= flags;
}

// TODO(SW): This requires testing. ICODE
void MXC_SPC_SetCode_NSC(bool isNSC)
{
    if (isNSC) {
        MXC_SPC->nscidau |= MXC_F_SPC_NSCIDAU_CODE;
    } else {
        MXC_SPC->nscidau &= ~MXC_F_SPC_NSCIDAU_CODE;
    }
}

void MXC_SPC_SetSRAM_NSC(bool isNSC)
{
    if (isNSC) {
        MXC_SPC->nscidau |= MXC_F_SPC_NSCIDAU_SRAM;
    } else {
        MXC_SPC->nscidau &= ~MXC_F_SPC_NSCIDAU_SRAM;
    }
}

#endif
