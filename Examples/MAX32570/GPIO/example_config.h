/******************************************************************************
 * Copyright (C) 2022 Maxim Integrated Products, Inc., All Rights Reserved.
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

#ifndef EXAMPLES_MAX32570_GPIO_EXAMPLE_CONFIG_H_
#define EXAMPLES_MAX32570_GPIO_EXAMPLE_CONFIG_H_

#include "mxc_device.h"
#include "gpio.h"
#include "board.h"

/***** Definitions *****/
#if defined(BOARD_M_EVKIT_V1) || defined(BOARD_MN_EVKIT_V1)
#define MXC_GPIO_PORT_IN MXC_GPIO0
#define MXC_GPIO_PIN_IN MXC_GPIO_PIN_16

#define MXC_GPIO_PORT_OUT MXC_GPIO2
#define MXC_GPIO_PIN_OUT MXC_GPIO_PIN_17
#else
#define MXC_GPIO_PORT_IN MXC_GPIO3
#define MXC_GPIO_PIN_IN MXC_GPIO_PIN_7

#define MXC_GPIO_PORT_OUT MXC_GPIO3
#define MXC_GPIO_PIN_OUT MXC_GPIO_PIN_5

#define MXC_GPIO_PORT_INTERRUPT_IN MXC_GPIO3
#define MXC_GPIO_PIN_INTERRUPT_IN MXC_GPIO_PIN_6

#define MXC_GPIO_PORT_INTERRUPT_STATUS MXC_GPIO3
#define MXC_GPIO_PIN_INTERRUPT_STATUS MXC_GPIO_PIN_4
#endif

#endif // EXAMPLES_MAX32570_GPIO_EXAMPLE_CONFIG_H_
