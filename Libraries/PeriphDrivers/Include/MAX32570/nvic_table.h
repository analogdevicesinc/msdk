/**
 * @file    nvic_table.h
 * @brief   Interrupt vector table manipulation functions.
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

#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32570_NVIC_TABLE_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32570_NVIC_TABLE_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Set an IRQ hander callback function.  If the IRQ table is in
 * flash, this will copy it to RAM and set NVIC to RAM based table.
 *
 * @param irqn          ARM external IRQ number
 * @param irq_callback  Function to be called at IRQ context
 *
 */
void MXC_NVIC_SetVector(IRQn_Type irqn, void (*irq_callback)(void));

#if defined(__GNUC__)
#if __CM4_CMSIS_VERSION_MAIN == 0x03
// NVIC_SetVector was custom-implemented in the PeriphDrivers library for
// CMSIS version 3.  Newer versions of CMSIS provide an implementation of
// NVIC_SetVector with different functionality, so the Maxim implementation
// has been moved to MXC_NVIC_SetVector (above).

// The MSDK will move to CMSIS version 5 in the future.

// For CMSIS version 3, use MXC_NVIC_SetVector instead.
// For CMSIS version 5, you have the choice of using either.  However, only
// MXC_NVIC_SetVector will work with legacy code.
inline __attribute__((
    deprecated("Use MXC_NVIC_SetVector instead.  See nvic_table.h for more details."))) void
NVIC_SetVector(IRQn_Type irqn, void (*irq_callback)(void))
{
    MXC_NVIC_SetVector(irqn, irq_callback);
}
#endif
#endif

/**
 * @brief Copy NVIC vector table to RAM and set NVIC to RAM based table.
 *
 */
void NVIC_SetRAM(void);

/**
 * @brief      Get Interrupt Vector
 * @details    Reads an interrupt vector from interrupt vector table. The
 *             interrupt number can be positive to specify a device specific
 *             interrupt, or negative to specify a processor exception.
 * @param[in]  IRQn  Interrupt number.
 * @return     Address of interrupt handler function
 */
uint32_t MXC_NVIC_GetVector(IRQn_Type IRQn);

#if defined(__GNUC__)
#if __CM4_CMSIS_VERSION_MAIN == 0x03
// NVIC_GetVector was custom-implemented in the PeriphDrivers library for
// CMSIS version 3.  Newer versions of CMSIS provide an implementation of
// NVIC_GetVector with different functionality, so the Maxim implementation
// has been moved to MXC_NVIC_GetVector (above).

// The MSDK will move to CMSIS version 5 in the future.

// For CMSIS version 3, use MXC_NVIC_SetVector instead.
// For CMSIS version 5, you have the choice of using either.  However, only
// MXC_NVIC_GetVector will work with legacy code.
inline __attribute__((
    deprecated("Use MXC_NVIC_GetVector instead.  See nvic_table.h for more details."))) void
NVIC_GetVector(IRQn_Type irqn)
{
    MXC_NVIC_GetVector(irqn);
}
#endif
#endif

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32570_NVIC_TABLE_H_
