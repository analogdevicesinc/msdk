/**
 * @file  nvic_table.h
 * @brief Interrupt vector table manipulation functions.
 */

/*******************************************************************************
 * Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
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
 * $Date: 2020-04-20 15:48:35 -0500 (Mon, 20 Apr 2020) $
 * $Revision: 53144 $
 *
 ******************************************************************************/

#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32650_NVIC_TABLE_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32650_NVIC_TABLE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "max32650.h"

/**
 * @ingroup    syscfg
 * @defgroup   nvic NVIC Table
 * @brief      functions handling the nvic table.
 * @{
 */
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

/**@} end of group nvic */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32650_NVIC_TABLE_H_
