/**
 * @file   mxc_pins.h
 * @brief  This file contains constant pin configurations for the peripherals.
 */

/* *****************************************************************************
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
 * $Date: 2018-11-05 09:52:05 -0600 (Mon, 05 Nov 2018) $
 * $Revision: 38934 $
 *
 **************************************************************************** */

/* Define to prevent redundant inclusion */
#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32650_MXC_PINS_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32650_MXC_PINS_H_

/* **** Includes **** */
#include "gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/***** Global Variables *****/

// Predefined GPIO Configurations
extern const mxc_gpio_cfg_t gpio_cfg_tmr0;
extern const mxc_gpio_cfg_t gpio_cfg_tmr1;
extern const mxc_gpio_cfg_t gpio_cfg_tmr2;
extern const mxc_gpio_cfg_t gpio_cfg_tmr3;
extern const mxc_gpio_cfg_t gpio_cfg_tmr4;
extern const mxc_gpio_cfg_t gpio_cfg_tmr5;
extern const mxc_gpio_cfg_t gpio_cfg_uart0;
extern const mxc_gpio_cfg_t gpio_cfg_uart0_flow;
extern const mxc_gpio_cfg_t gpio_cfg_uart1;
extern const mxc_gpio_cfg_t gpio_cfg_uart1_flow;
extern const mxc_gpio_cfg_t gpio_cfg_uart2;
extern const mxc_gpio_cfg_t gpio_cfg_uart2_flow;
extern const mxc_gpio_cfg_t gpio_cfg_i2c0;
extern const mxc_gpio_cfg_t gpio_cfg_i2c1;
extern const mxc_gpio_cfg_t gpio_cfg_spi0_0;
extern const mxc_gpio_cfg_t gpio_cfg_spi0_1;
extern const mxc_gpio_cfg_t gpio_cfg_spi1;
extern const mxc_gpio_cfg_t gpio_cfg_spi1_ss0;
extern const mxc_gpio_cfg_t gpio_cfg_spi1_ss1;
extern const mxc_gpio_cfg_t gpio_cfg_spi1_ss2;
extern const mxc_gpio_cfg_t gpio_cfg_spi1_ss3;
extern const mxc_gpio_cfg_t gpio_cfg_spi2;
extern const mxc_gpio_cfg_t gpio_cfg_spi2_ss0;
extern const mxc_gpio_cfg_t gpio_cfg_spi2_ss1;
extern const mxc_gpio_cfg_t gpio_cfg_spi2_ss2;
extern const mxc_gpio_cfg_t gpio_cfg_spi2_ss3;
extern const mxc_gpio_cfg_t gpio_cfg_spi3;
extern const mxc_gpio_cfg_t gpio_cfg_spi3_ss0;
extern const mxc_gpio_cfg_t gpio_cfg_spi3_ss1;
extern const mxc_gpio_cfg_t gpio_cfg_spi3_ss2;
extern const mxc_gpio_cfg_t gpio_cfg_spi3_ss3;
extern const mxc_gpio_cfg_t gpio_cfg_pt0_0;
extern const mxc_gpio_cfg_t gpio_cfg_pt1_0;
extern const mxc_gpio_cfg_t gpio_cfg_pt2_0;
extern const mxc_gpio_cfg_t gpio_cfg_pt3_0;
extern const mxc_gpio_cfg_t gpio_cfg_pt4_0;
extern const mxc_gpio_cfg_t gpio_cfg_pt5_0;
extern const mxc_gpio_cfg_t gpio_cfg_pt6_0;
extern const mxc_gpio_cfg_t gpio_cfg_pt7_0;
extern const mxc_gpio_cfg_t gpio_cfg_pt8_0;
extern const mxc_gpio_cfg_t gpio_cfg_pt9_0;
extern const mxc_gpio_cfg_t gpio_cfg_pt10_0;
extern const mxc_gpio_cfg_t gpio_cfg_pt11_0;
extern const mxc_gpio_cfg_t gpio_cfg_pt12_0;
extern const mxc_gpio_cfg_t gpio_cfg_pt13_0;
extern const mxc_gpio_cfg_t gpio_cfg_pt14_0;
extern const mxc_gpio_cfg_t gpio_cfg_pt15_0;
extern const mxc_gpio_cfg_t gpio_cfg_pt0_1;
extern const mxc_gpio_cfg_t gpio_cfg_pt1_1;
extern const mxc_gpio_cfg_t gpio_cfg_pt2_1;
extern const mxc_gpio_cfg_t gpio_cfg_pt3_1;
extern const mxc_gpio_cfg_t gpio_cfg_pt4_1;
extern const mxc_gpio_cfg_t gpio_cfg_pt5_1;
extern const mxc_gpio_cfg_t gpio_cfg_pt6_1;
extern const mxc_gpio_cfg_t gpio_cfg_pt7_1;
extern const mxc_gpio_cfg_t gpio_cfg_pt8_1;
extern const mxc_gpio_cfg_t gpio_cfg_pt9_1;
extern const mxc_gpio_cfg_t gpio_cfg_pt10_1;
extern const mxc_gpio_cfg_t gpio_cfg_pt11_1;
extern const mxc_gpio_cfg_t gpio_cfg_pt12_1;
extern const mxc_gpio_cfg_t gpio_cfg_pt13_1;
extern const mxc_gpio_cfg_t gpio_cfg_pt14_1;
extern const mxc_gpio_cfg_t gpio_cfg_pt15_1;
extern const mxc_gpio_cfg_t gpio_cfg_rtcsqw;
extern const mxc_gpio_cfg_t gpio_cfg_spixfc;
extern const mxc_gpio_cfg_t gpio_cfg_spixr;
extern const mxc_gpio_cfg_t gpio_cfg_hyp;
extern const mxc_gpio_cfg_t gpio_cfg_hyp_cs0;
extern const mxc_gpio_cfg_t gpio_cfg_hyp_cs1;

extern const mxc_gpio_cfg_t gpio_cfg_sdhc_0;
extern const mxc_gpio_cfg_t gpio_cfg_sdhc_1;
extern const mxc_gpio_cfg_t gpio_cfg_owm;

extern const mxc_gpio_cfg_t gpio_cfg_clcd_0;
extern const mxc_gpio_cfg_t gpio_cfg_clcd_1;
extern const mxc_gpio_cfg_t gpio_cfg_clcd_2;

extern const mxc_gpio_cfg_t gpio_cfg_i2s;

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32650_MXC_PINS_H_
