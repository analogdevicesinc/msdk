/******************************************************************************
 * Copyright (C) 2023 Maxim Integrated Products, Inc., All Rights Reserved.
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
 ******************************************************************************/

#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32665_CORE1_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32665_CORE1_H_

/**
 * @file    core1.h
 * @brief   Startup Code for MAX32665 Family CPU1
 * @details These functions are called at the startup of the second ARM core (CPU1/Core1)
 */

/**
 * @brief Starts the code on core 1
 *        Core1 code beings executing from Core1_Main()
 */
void Core1_Start(void);

/**
 * @brief Stops code executing in Core 1
 */
void Core1_Stop(void);

/**
 * @brief Main function for Core 1 Code
 *        The user should override this function
 *        in their application code
 */
int Core1_Main(void);

/**
 * @brief Equivalent to PreInit for Core 0
 *        Can be used for preliminary initialization
 */
void PreInit_Core1(void);

/**
 * @brief Equivalent to PreInit for Core 1
 *        Enables FPU, and ICache
 *        Sets interrupt vector
 */
void SystemInit_Core1(void);

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32665_CORE1_H_
