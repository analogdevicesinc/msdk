/******************************************************************************
 *
 * Copyright (C) 2026 Analog Devices, Inc.
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

#ifndef LIBRARIES_ZEPHYR_MAX_INCLUDE_WRAP_MAX32_ICC_H_
#define LIBRARIES_ZEPHYR_MAX_INCLUDE_WRAP_MAX32_ICC_H_

/* Includes */
#include <icc.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * SoCs with per-instance ICC API: MXC_ICC_* take an `mxc_icc_regs_t *icc`.
 * Call the per-instance functions directly.
 */
#if defined(CONFIG_SOC_MAX32655) || defined(CONFIG_SOC_MAX32680) || \
    defined(CONFIG_SOC_MAX32690) || defined(CONFIG_SOC_MAX78000) || defined(CONFIG_SOC_MAX78002)

static inline void Wrap_MXC_ICC_Enable(mxc_icc_regs_t *icc)
{
    MXC_ICC_Enable(icc);
}

static inline void Wrap_MXC_ICC_Disable(mxc_icc_regs_t *icc)
{
    MXC_ICC_Disable(icc);
}

static inline void Wrap_MXC_ICC_Flush(mxc_icc_regs_t *icc)
{
    MXC_ICC_Flush(icc);
}

static inline void Wrap_MXC_ICC_Invalidate(mxc_icc_regs_t *icc)
{
    MXC_ICC_Invalidate(icc);
}

static inline void Wrap_MXC_ICC_WaitForReady(mxc_icc_regs_t *icc)
{
    MXC_ICC_WaitForReady(icc);
}

/*
 * SoCs that expose both global and instance APIs (use instance API when
 * available). These SoCs provide MXC_ICC_*Inst() functions.
 */
#elif defined(CONFIG_SOC_MAX32650) || defined(CONFIG_SOC_MAX32665) || defined(CONFIG_SOC_MAX32666)

static inline void Wrap_MXC_ICC_Enable(mxc_icc_regs_t *icc)
{
    MXC_ICC_EnableInst(icc);
}

static inline void Wrap_MXC_ICC_Disable(mxc_icc_regs_t *icc)
{
    MXC_ICC_DisableInst(icc);
}

static inline void Wrap_MXC_ICC_Flush(mxc_icc_regs_t *icc)
{
    MXC_ICC_FlushInst(icc);
}

static inline void Wrap_MXC_ICC_Invalidate(mxc_icc_regs_t *icc)
{
    MXC_ICC_InvalidateInst(icc);
}

static inline void Wrap_MXC_ICC_WaitForReady(mxc_icc_regs_t *icc)
{
    /* Prefer instance variant when present */
    MXC_ICC_WaitForReadyInst(icc);
}

/*
 * SoCs with only global ICC API (no per-instance functions).
 * Ignore the `icc` parameter and call the global functions.
 */
#else

static inline void Wrap_MXC_ICC_Enable(mxc_icc_regs_t *icc)
{
    (void)icc;
    MXC_ICC_Enable();
}

static inline void Wrap_MXC_ICC_Disable(mxc_icc_regs_t *icc)
{
    (void)icc;
    MXC_ICC_Disable();
}

static inline void Wrap_MXC_ICC_Flush(mxc_icc_regs_t *icc)
{
    (void)icc;
    MXC_ICC_Flush();
}

static inline void Wrap_MXC_ICC_Invalidate(mxc_icc_regs_t *icc)
{
    (void)icc;
    MXC_ICC_Invalidate();
}

static inline void Wrap_MXC_ICC_WaitForReady(mxc_icc_regs_t *icc)
{
    (void)icc;
    MXC_ICC_WaitForReady();
}

#endif // SOC selection

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_ZEPHYR_MAX_INCLUDE_WRAP_MAX32_ICC_H_
