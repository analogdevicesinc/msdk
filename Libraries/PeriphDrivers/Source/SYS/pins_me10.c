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
 * $Date$
 * $Revision$
 *
 **************************************************************************** */

/* **** Includes **** */
#include "gpio.h"

/**
 * @ingroup MXC_pins
 * @{
 */

/* **** Global Variables **** */

const mxc_gpio_cfg_t gpio_cfg_tmr0 = { MXC_GPIO3, MXC_GPIO_PIN_4, MXC_GPIO_FUNC_ALT1,
                                       MXC_GPIO_PAD_NONE };
const mxc_gpio_cfg_t gpio_cfg_tmr1 = { MXC_GPIO3, MXC_GPIO_PIN_7, MXC_GPIO_FUNC_ALT1,
                                       MXC_GPIO_PAD_NONE };
const mxc_gpio_cfg_t gpio_cfg_tmr2 = { MXC_GPIO3, MXC_GPIO_PIN_5, MXC_GPIO_FUNC_ALT1,
                                       MXC_GPIO_PAD_NONE };
const mxc_gpio_cfg_t gpio_cfg_tmr3 = { MXC_GPIO3, MXC_GPIO_PIN_8, MXC_GPIO_FUNC_ALT1,
                                       MXC_GPIO_PAD_NONE };
const mxc_gpio_cfg_t gpio_cfg_tmr4 = { MXC_GPIO3, MXC_GPIO_PIN_6, MXC_GPIO_FUNC_ALT1,
                                       MXC_GPIO_PAD_NONE };
const mxc_gpio_cfg_t gpio_cfg_tmr5 = { MXC_GPIO3, MXC_GPIO_PIN_9, MXC_GPIO_FUNC_ALT1,
                                       MXC_GPIO_PAD_NONE };

const mxc_gpio_cfg_t gpio_cfg_uart0 = { MXC_GPIO2, (MXC_GPIO_PIN_11 | MXC_GPIO_PIN_12),
                                        MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE };
const mxc_gpio_cfg_t gpio_cfg_uart0_flow = { MXC_GPIO2, (MXC_GPIO_PIN_9 | MXC_GPIO_PIN_10),
                                             MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE };
const mxc_gpio_cfg_t gpio_cfg_uart1 = { MXC_GPIO2, (MXC_GPIO_PIN_14 | MXC_GPIO_PIN_16),
                                        MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE };
const mxc_gpio_cfg_t gpio_cfg_uart1_flow = { MXC_GPIO2, (MXC_GPIO_PIN_13 | MXC_GPIO_PIN_15),
                                             MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE };
const mxc_gpio_cfg_t gpio_cfg_uart2 = { MXC_GPIO1, (MXC_GPIO_PIN_9 | MXC_GPIO_PIN_10),
                                        MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE };
const mxc_gpio_cfg_t gpio_cfg_uart2_flow = { MXC_GPIO1, (MXC_GPIO_PIN_7 | MXC_GPIO_PIN_8),
                                             MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE };

const mxc_gpio_cfg_t gpio_cfg_i2c0 = { MXC_GPIO2, (MXC_GPIO_PIN_7 | MXC_GPIO_PIN_8),
                                       MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE };
const mxc_gpio_cfg_t gpio_cfg_i2c1 = { MXC_GPIO2, (MXC_GPIO_PIN_17 | MXC_GPIO_PIN_18),
                                       MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE };

const mxc_gpio_cfg_t gpio_cfg_spi0_0 = { MXC_GPIO0, (MXC_GPIO_PIN_22), MXC_GPIO_FUNC_ALT1,
                                         MXC_GPIO_PAD_NONE };
const mxc_gpio_cfg_t gpio_cfg_spi0_1 = { MXC_GPIO3,
                                         (MXC_GPIO_PIN_1 | MXC_GPIO_PIN_2 | MXC_GPIO_PIN_3),
                                         MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE };

const mxc_gpio_cfg_t gpio_cfg_spi1 = { MXC_GPIO1,
                                       (MXC_GPIO_PIN_26 | MXC_GPIO_PIN_28 | MXC_GPIO_PIN_29),
                                       MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE };
const mxc_gpio_cfg_t gpio_cfg_spi1_ss0 = { MXC_GPIO1, (MXC_GPIO_PIN_23), MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE };
const mxc_gpio_cfg_t gpio_cfg_spi1_ss1 = { MXC_GPIO1, (MXC_GPIO_PIN_25), MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE };
const mxc_gpio_cfg_t gpio_cfg_spi1_ss2 = { MXC_GPIO1, (MXC_GPIO_PIN_24), MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE };
const mxc_gpio_cfg_t gpio_cfg_spi1_ss3 = { MXC_GPIO1, (MXC_GPIO_PIN_27), MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE };

const mxc_gpio_cfg_t gpio_cfg_spi2 = { MXC_GPIO2,
                                       (MXC_GPIO_PIN_2 | MXC_GPIO_PIN_3 | MXC_GPIO_PIN_4),
                                       MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE };
const mxc_gpio_cfg_t gpio_cfg_spi2_ss0 = { MXC_GPIO2, (MXC_GPIO_PIN_5), MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE };
const mxc_gpio_cfg_t gpio_cfg_spi2_ss1 = { MXC_GPIO2, (MXC_GPIO_PIN_1), MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE };
const mxc_gpio_cfg_t gpio_cfg_spi2_ss2 = { MXC_GPIO2, (MXC_GPIO_PIN_0), MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE };
const mxc_gpio_cfg_t gpio_cfg_spi2_ss3 = { MXC_GPIO2, (MXC_GPIO_PIN_6), MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE };

const mxc_gpio_cfg_t gpio_cfg_spi3 = { MXC_GPIO0,
                                       (MXC_GPIO_PIN_15 | MXC_GPIO_PIN_16 | MXC_GPIO_PIN_17 |
                                        MXC_GPIO_PIN_20 | MXC_GPIO_PIN_21),
                                       MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE };
const mxc_gpio_cfg_t gpio_cfg_spi3_ss0 = { MXC_GPIO0, (MXC_GPIO_PIN_19), MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE };
const mxc_gpio_cfg_t gpio_cfg_spi3_ss1 = { MXC_GPIO0, (MXC_GPIO_PIN_13), MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE };
const mxc_gpio_cfg_t gpio_cfg_spi3_ss2 = { MXC_GPIO0, (MXC_GPIO_PIN_14), MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE };
const mxc_gpio_cfg_t gpio_cfg_spi3_ss3 = { MXC_GPIO0, (MXC_GPIO_PIN_18), MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE };

const mxc_gpio_cfg_t gpio_cfg_pt0_0 = { MXC_GPIO2, MXC_GPIO_PIN_29, MXC_GPIO_FUNC_ALT1,
                                        MXC_GPIO_PAD_NONE };
const mxc_gpio_cfg_t gpio_cfg_pt1_0 = { MXC_GPIO2, MXC_GPIO_PIN_30, MXC_GPIO_FUNC_ALT1,
                                        MXC_GPIO_PAD_NONE };
const mxc_gpio_cfg_t gpio_cfg_pt2_0 = { MXC_GPIO2, MXC_GPIO_PIN_31, MXC_GPIO_FUNC_ALT1,
                                        MXC_GPIO_PAD_NONE };
const mxc_gpio_cfg_t gpio_cfg_pt3_0 = { MXC_GPIO0, MXC_GPIO_PIN_0, MXC_GPIO_FUNC_ALT1,
                                        MXC_GPIO_PAD_NONE };
const mxc_gpio_cfg_t gpio_cfg_pt4_0 = { MXC_GPIO2, MXC_GPIO_PIN_19, MXC_GPIO_FUNC_ALT1,
                                        MXC_GPIO_PAD_NONE };
const mxc_gpio_cfg_t gpio_cfg_pt5_0 = { MXC_GPIO2, MXC_GPIO_PIN_20, MXC_GPIO_FUNC_ALT1,
                                        MXC_GPIO_PAD_NONE };
const mxc_gpio_cfg_t gpio_cfg_pt6_0 = { MXC_GPIO2, MXC_GPIO_PIN_23, MXC_GPIO_FUNC_ALT1,
                                        MXC_GPIO_PAD_NONE };
const mxc_gpio_cfg_t gpio_cfg_pt7_0 = { MXC_GPIO2, MXC_GPIO_PIN_21, MXC_GPIO_FUNC_ALT1,
                                        MXC_GPIO_PAD_NONE };
const mxc_gpio_cfg_t gpio_cfg_pt8_0 = { MXC_GPIO2, MXC_GPIO_PIN_22, MXC_GPIO_FUNC_ALT1,
                                        MXC_GPIO_PAD_NONE };
const mxc_gpio_cfg_t gpio_cfg_pt9_0 = { MXC_GPIO1, MXC_GPIO_PIN_17, MXC_GPIO_FUNC_ALT1,
                                        MXC_GPIO_PAD_NONE };
const mxc_gpio_cfg_t gpio_cfg_pt10_0 = { MXC_GPIO2, MXC_GPIO_PIN_24, MXC_GPIO_FUNC_ALT1,
                                         MXC_GPIO_PAD_NONE };
const mxc_gpio_cfg_t gpio_cfg_pt11_0 = { MXC_GPIO2, MXC_GPIO_PIN_25, MXC_GPIO_FUNC_ALT1,
                                         MXC_GPIO_PAD_NONE };
const mxc_gpio_cfg_t gpio_cfg_pt12_0 = { MXC_GPIO2, MXC_GPIO_PIN_26, MXC_GPIO_FUNC_ALT1,
                                         MXC_GPIO_PAD_NONE };
const mxc_gpio_cfg_t gpio_cfg_pt13_0 = { MXC_GPIO2, MXC_GPIO_PIN_27, MXC_GPIO_FUNC_ALT1,
                                         MXC_GPIO_PAD_NONE };
const mxc_gpio_cfg_t gpio_cfg_pt14_0 = { MXC_GPIO2, MXC_GPIO_PIN_28, MXC_GPIO_FUNC_ALT1,
                                         MXC_GPIO_PAD_NONE };
const mxc_gpio_cfg_t gpio_cfg_pt15_0 = { MXC_GPIO0, MXC_GPIO_PIN_23, MXC_GPIO_FUNC_ALT1,
                                         MXC_GPIO_PAD_NONE };

const mxc_gpio_cfg_t gpio_cfg_pt0_1 = { MXC_GPIO1, MXC_GPIO_PIN_6, MXC_GPIO_FUNC_ALT2,
                                        MXC_GPIO_PAD_NONE };
const mxc_gpio_cfg_t gpio_cfg_pt1_1 = { MXC_GPIO1, MXC_GPIO_PIN_7, MXC_GPIO_FUNC_ALT2,
                                        MXC_GPIO_PAD_NONE };
const mxc_gpio_cfg_t gpio_cfg_pt2_1 = { MXC_GPIO1, MXC_GPIO_PIN_8, MXC_GPIO_FUNC_ALT2,
                                        MXC_GPIO_PAD_NONE };
const mxc_gpio_cfg_t gpio_cfg_pt3_1 = { MXC_GPIO1, MXC_GPIO_PIN_9, MXC_GPIO_FUNC_ALT2,
                                        MXC_GPIO_PAD_NONE };
const mxc_gpio_cfg_t gpio_cfg_pt4_1 = { MXC_GPIO1, MXC_GPIO_PIN_10, MXC_GPIO_FUNC_ALT2,
                                        MXC_GPIO_PAD_NONE };
const mxc_gpio_cfg_t gpio_cfg_pt5_1 = { MXC_GPIO1, MXC_GPIO_PIN_14, MXC_GPIO_FUNC_ALT2,
                                        MXC_GPIO_PAD_NONE };
const mxc_gpio_cfg_t gpio_cfg_pt6_1 = { MXC_GPIO1, MXC_GPIO_PIN_18, MXC_GPIO_FUNC_ALT2,
                                        MXC_GPIO_PAD_NONE };
const mxc_gpio_cfg_t gpio_cfg_pt7_1 = { MXC_GPIO1, MXC_GPIO_PIN_19, MXC_GPIO_FUNC_ALT2,
                                        MXC_GPIO_PAD_NONE };
const mxc_gpio_cfg_t gpio_cfg_pt8_1 = { MXC_GPIO1, MXC_GPIO_PIN_21, MXC_GPIO_FUNC_ALT2,
                                        MXC_GPIO_PAD_NONE };
const mxc_gpio_cfg_t gpio_cfg_pt9_1 = { MXC_GPIO2, MXC_GPIO_PIN_0, MXC_GPIO_FUNC_ALT2,
                                        MXC_GPIO_PAD_NONE };
const mxc_gpio_cfg_t gpio_cfg_pt10_1 = { MXC_GPIO2, MXC_GPIO_PIN_1, MXC_GPIO_FUNC_ALT2,
                                         MXC_GPIO_PAD_NONE };
const mxc_gpio_cfg_t gpio_cfg_pt11_1 = { MXC_GPIO2, MXC_GPIO_PIN_5, MXC_GPIO_FUNC_ALT2,
                                         MXC_GPIO_PAD_NONE };
const mxc_gpio_cfg_t gpio_cfg_pt12_1 = { MXC_GPIO2, MXC_GPIO_PIN_9, MXC_GPIO_FUNC_ALT2,
                                         MXC_GPIO_PAD_NONE };
const mxc_gpio_cfg_t gpio_cfg_pt13_1 = { MXC_GPIO2, MXC_GPIO_PIN_11, MXC_GPIO_FUNC_ALT2,
                                         MXC_GPIO_PAD_NONE };
const mxc_gpio_cfg_t gpio_cfg_pt14_1 = { MXC_GPIO2, MXC_GPIO_PIN_10, MXC_GPIO_FUNC_ALT2,
                                         MXC_GPIO_PAD_NONE };
const mxc_gpio_cfg_t gpio_cfg_pt15_1 = { MXC_GPIO2, MXC_GPIO_PIN_12, MXC_GPIO_FUNC_ALT2,
                                         MXC_GPIO_PAD_NONE };

const mxc_gpio_cfg_t gpio_cfg_hyp = { MXC_GPIO1,
                                      (MXC_GPIO_PIN_12 | MXC_GPIO_PIN_13 | MXC_GPIO_PIN_14 |
                                       MXC_GPIO_PIN_15 | MXC_GPIO_PIN_16 | MXC_GPIO_PIN_18 |
                                       MXC_GPIO_PIN_19 | MXC_GPIO_PIN_20 | MXC_GPIO_PIN_21),
                                      MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE };
const mxc_gpio_cfg_t gpio_cfg_hyp_cs0 = { MXC_GPIO1, MXC_GPIO_PIN_11, MXC_GPIO_FUNC_ALT1,
                                          MXC_GPIO_PAD_NONE };
const mxc_gpio_cfg_t gpio_cfg_hyp_cs1 = { MXC_GPIO3, MXC_GPIO_PIN_0, MXC_GPIO_FUNC_ALT2,
                                          MXC_GPIO_PAD_NONE };

const mxc_gpio_cfg_t gpio_cfg_spixr = { MXC_GPIO0,
                                        (MXC_GPIO_PIN_1 | MXC_GPIO_PIN_2 | MXC_GPIO_PIN_3 |
                                         MXC_GPIO_PIN_4 | MXC_GPIO_PIN_5 | MXC_GPIO_PIN_6),
                                        MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_WEAK_PULL_UP };
const mxc_gpio_cfg_t gpio_cfg_rtcsqw = { MXC_GPIO0, MXC_GPIO_PIN_31, MXC_GPIO_FUNC_ALT1,
                                         MXC_GPIO_PAD_NONE };

const mxc_gpio_cfg_t gpio_cfg_spixfc = { MXC_GPIO0,
                                         (MXC_GPIO_PIN_7 | MXC_GPIO_PIN_8 | MXC_GPIO_PIN_9 |
                                          MXC_GPIO_PIN_10 | MXC_GPIO_PIN_11 | MXC_GPIO_PIN_12),
                                         MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE };

const mxc_gpio_cfg_t gpio_cfg_sdhc_0 = { MXC_GPIO0, (MXC_GPIO_PIN_31), MXC_GPIO_FUNC_ALT2,
                                         MXC_GPIO_PAD_NONE };
const mxc_gpio_cfg_t gpio_cfg_sdhc_1 = { MXC_GPIO1,
                                         (MXC_GPIO_PIN_0 | MXC_GPIO_PIN_1 | MXC_GPIO_PIN_2 |
                                          MXC_GPIO_PIN_3 | MXC_GPIO_PIN_4 | MXC_GPIO_PIN_5 |
                                          MXC_GPIO_PIN_6),
                                         MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE };

const mxc_gpio_cfg_t gpio_cfg_owm = { MXC_GPIO1, (MXC_GPIO_PIN_30 | MXC_GPIO_PIN_31),
                                      MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE, 
                                      MXC_GPIO_VSSEL_VDDIOH };

const mxc_gpio_cfg_t gpio_cfg_clcd_0 = {
    MXC_GPIO0,
    (MXC_GPIO_PIN_13 | MXC_GPIO_PIN_14 | MXC_GPIO_PIN_15 | MXC_GPIO_PIN_16 | MXC_GPIO_PIN_17 |
     MXC_GPIO_PIN_18 | MXC_GPIO_PIN_19 | MXC_GPIO_PIN_20 | MXC_GPIO_PIN_22 | MXC_GPIO_PIN_23 |
     MXC_GPIO_PIN_24 | MXC_GPIO_PIN_25 | MXC_GPIO_PIN_30),
    MXC_GPIO_FUNC_ALT2, MXC_GPIO_PAD_NONE
};
const mxc_gpio_cfg_t gpio_cfg_clcd_1 = { MXC_GPIO1,
                                         (MXC_GPIO_PIN_3 | MXC_GPIO_PIN_20 | MXC_GPIO_PIN_23 |
                                          MXC_GPIO_PIN_24 | MXC_GPIO_PIN_25 | MXC_GPIO_PIN_26 |
                                          MXC_GPIO_PIN_27 | MXC_GPIO_PIN_28 | MXC_GPIO_PIN_29 |
                                          MXC_GPIO_PIN_30 | MXC_GPIO_PIN_31),
                                         MXC_GPIO_FUNC_ALT2, MXC_GPIO_PAD_NONE };
const mxc_gpio_cfg_t gpio_cfg_clcd_2 = { MXC_GPIO2,
                                         (MXC_GPIO_PIN_2 | MXC_GPIO_PIN_3 | MXC_GPIO_PIN_6 |
                                          MXC_GPIO_PIN_13 | MXC_GPIO_PIN_14 | MXC_GPIO_PIN_15 |
                                          MXC_GPIO_PIN_16 | MXC_GPIO_PIN_17 | MXC_GPIO_PIN_18),
                                         MXC_GPIO_FUNC_ALT2, MXC_GPIO_PAD_NONE };

const mxc_gpio_cfg_t gpio_cfg_i2s = {
    MXC_GPIO2, (MXC_GPIO_PIN_2 | MXC_GPIO_PIN_3 | MXC_GPIO_PIN_4 | MXC_GPIO_PIN_5),
    MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE
};

/**@} end of ingroup MXC_pins*/