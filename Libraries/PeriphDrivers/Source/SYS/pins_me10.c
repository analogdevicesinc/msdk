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

/* **** Includes **** */
#include "gpio.h"

/* **** Global Variables **** */

// clang-format off
const mxc_gpio_cfg_t gpio_cfg_tmr0 = { MXC_GPIO3, MXC_GPIO_PIN_4, MXC_GPIO_FUNC_ALT1,
                                       MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_tmr1 = { MXC_GPIO3, MXC_GPIO_PIN_7, MXC_GPIO_FUNC_ALT1,
                                       MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_tmr2 = { MXC_GPIO3, MXC_GPIO_PIN_5, MXC_GPIO_FUNC_ALT1,
                                       MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_tmr3 = { MXC_GPIO3, MXC_GPIO_PIN_8, MXC_GPIO_FUNC_ALT1,
                                       MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_tmr4 = { MXC_GPIO3, MXC_GPIO_PIN_6, MXC_GPIO_FUNC_ALT1,
                                       MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_tmr5 = { MXC_GPIO3, MXC_GPIO_PIN_9, MXC_GPIO_FUNC_ALT1,
                                       MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

const mxc_gpio_cfg_t gpio_cfg_uart0 = { MXC_GPIO2, (MXC_GPIO_PIN_11 | MXC_GPIO_PIN_12), MXC_GPIO_FUNC_ALT1,
                                        MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_uart0_flow = { MXC_GPIO2, (MXC_GPIO_PIN_9 | MXC_GPIO_PIN_10), MXC_GPIO_FUNC_ALT1,
                                             MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_uart1 = { MXC_GPIO2, (MXC_GPIO_PIN_14 | MXC_GPIO_PIN_16), MXC_GPIO_FUNC_ALT1,
                                        MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_uart1_flow = { MXC_GPIO2, (MXC_GPIO_PIN_13 | MXC_GPIO_PIN_15), MXC_GPIO_FUNC_ALT1,
                                             MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_uart2 = { MXC_GPIO1, (MXC_GPIO_PIN_9 | MXC_GPIO_PIN_10), MXC_GPIO_FUNC_ALT1,
                                        MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_uart2_flow = { MXC_GPIO1, (MXC_GPIO_PIN_7 | MXC_GPIO_PIN_8), MXC_GPIO_FUNC_ALT1,
                                             MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

const mxc_gpio_cfg_t gpio_cfg_i2c0 = { MXC_GPIO2, (MXC_GPIO_PIN_7 | MXC_GPIO_PIN_8), MXC_GPIO_FUNC_ALT1,
                                       MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_i2c1 = { MXC_GPIO2, (MXC_GPIO_PIN_17 | MXC_GPIO_PIN_18), MXC_GPIO_FUNC_ALT1,
                                       MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

const mxc_gpio_cfg_t gpio_cfg_spi0_0 = { MXC_GPIO0, (MXC_GPIO_PIN_22), MXC_GPIO_FUNC_ALT1,
                                         MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi0_1 = { MXC_GPIO3, (MXC_GPIO_PIN_1 | MXC_GPIO_PIN_2 | MXC_GPIO_PIN_3),
                                         MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

const mxc_gpio_cfg_t gpio_cfg_spi1 = { MXC_GPIO1, (MXC_GPIO_PIN_26 | MXC_GPIO_PIN_28 | MXC_GPIO_PIN_29),
                                       MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi1_ss0 = { MXC_GPIO1, (MXC_GPIO_PIN_23), MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi1_ss1 = { MXC_GPIO1, (MXC_GPIO_PIN_25), MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi1_ss2 = { MXC_GPIO1, (MXC_GPIO_PIN_24), MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi1_ss3 = { MXC_GPIO1, (MXC_GPIO_PIN_27), MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

const mxc_gpio_cfg_t gpio_cfg_spi2 = { MXC_GPIO2, (MXC_GPIO_PIN_2 | MXC_GPIO_PIN_3 | MXC_GPIO_PIN_4),
                                       MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi2_ss0 = { MXC_GPIO2, (MXC_GPIO_PIN_5), MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi2_ss1 = { MXC_GPIO2, (MXC_GPIO_PIN_1), MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi2_ss2 = { MXC_GPIO2, (MXC_GPIO_PIN_0), MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi2_ss3 = { MXC_GPIO2, (MXC_GPIO_PIN_6), MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

const mxc_gpio_cfg_t gpio_cfg_spi3 = { MXC_GPIO0, (MXC_GPIO_PIN_15 | MXC_GPIO_PIN_16 | MXC_GPIO_PIN_17 | MXC_GPIO_PIN_20 |
                                       MXC_GPIO_PIN_21), MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi3_ss0 = { MXC_GPIO0, (MXC_GPIO_PIN_19), MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi3_ss1 = { MXC_GPIO0, (MXC_GPIO_PIN_13), MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi3_ss2 = { MXC_GPIO0, (MXC_GPIO_PIN_14), MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi3_ss3 = { MXC_GPIO0, (MXC_GPIO_PIN_18), MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

const mxc_gpio_cfg_t gpio_cfg_pt0_0 = { MXC_GPIO2, MXC_GPIO_PIN_29, MXC_GPIO_FUNC_ALT1,
                                        MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_pt1_0 = { MXC_GPIO2, MXC_GPIO_PIN_30, MXC_GPIO_FUNC_ALT1,
                                        MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_pt2_0 = { MXC_GPIO2, MXC_GPIO_PIN_31, MXC_GPIO_FUNC_ALT1,
                                        MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_pt3_0 = { MXC_GPIO0, MXC_GPIO_PIN_0, MXC_GPIO_FUNC_ALT1,
                                        MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_pt4_0 = { MXC_GPIO2, MXC_GPIO_PIN_19, MXC_GPIO_FUNC_ALT1,
                                        MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_pt5_0 = { MXC_GPIO2, MXC_GPIO_PIN_20, MXC_GPIO_FUNC_ALT1,
                                        MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_pt6_0 = { MXC_GPIO2, MXC_GPIO_PIN_23, MXC_GPIO_FUNC_ALT1,
                                        MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_pt7_0 = { MXC_GPIO2, MXC_GPIO_PIN_21, MXC_GPIO_FUNC_ALT1,
                                        MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_pt8_0 = { MXC_GPIO2, MXC_GPIO_PIN_22, MXC_GPIO_FUNC_ALT1,
                                        MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_pt9_0 = { MXC_GPIO1, MXC_GPIO_PIN_17, MXC_GPIO_FUNC_ALT1,
                                        MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_pt10_0 = { MXC_GPIO2, MXC_GPIO_PIN_24, MXC_GPIO_FUNC_ALT1,
                                         MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_pt11_0 = { MXC_GPIO2, MXC_GPIO_PIN_25, MXC_GPIO_FUNC_ALT1,
                                         MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_pt12_0 = { MXC_GPIO2, MXC_GPIO_PIN_26, MXC_GPIO_FUNC_ALT1,
                                         MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_pt13_0 = { MXC_GPIO2, MXC_GPIO_PIN_27, MXC_GPIO_FUNC_ALT1,
                                         MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_pt14_0 = { MXC_GPIO2, MXC_GPIO_PIN_28, MXC_GPIO_FUNC_ALT1,
                                         MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_pt15_0 = { MXC_GPIO0, MXC_GPIO_PIN_23, MXC_GPIO_FUNC_ALT1,
                                         MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

const mxc_gpio_cfg_t gpio_cfg_pt0_1 = { MXC_GPIO1, MXC_GPIO_PIN_6, MXC_GPIO_FUNC_ALT2,
                                        MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_pt1_1 = { MXC_GPIO1, MXC_GPIO_PIN_7, MXC_GPIO_FUNC_ALT2,
                                        MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_pt2_1 = { MXC_GPIO1, MXC_GPIO_PIN_8, MXC_GPIO_FUNC_ALT2,
                                        MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_pt3_1 = { MXC_GPIO1, MXC_GPIO_PIN_9, MXC_GPIO_FUNC_ALT2,
                                        MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_pt4_1 = { MXC_GPIO1, MXC_GPIO_PIN_10, MXC_GPIO_FUNC_ALT2,
                                        MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_pt5_1 = { MXC_GPIO1, MXC_GPIO_PIN_14, MXC_GPIO_FUNC_ALT2,
                                        MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_pt6_1 = { MXC_GPIO1, MXC_GPIO_PIN_18, MXC_GPIO_FUNC_ALT2,
                                        MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_pt7_1 = { MXC_GPIO1, MXC_GPIO_PIN_19, MXC_GPIO_FUNC_ALT2,
                                        MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_pt8_1 = { MXC_GPIO1, MXC_GPIO_PIN_21, MXC_GPIO_FUNC_ALT2,
                                        MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_pt9_1 = { MXC_GPIO2, MXC_GPIO_PIN_0, MXC_GPIO_FUNC_ALT2,
                                        MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_pt10_1 = { MXC_GPIO2, MXC_GPIO_PIN_1, MXC_GPIO_FUNC_ALT2,
                                         MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_pt11_1 = { MXC_GPIO2, MXC_GPIO_PIN_5, MXC_GPIO_FUNC_ALT2,
                                         MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_pt12_1 = { MXC_GPIO2, MXC_GPIO_PIN_9, MXC_GPIO_FUNC_ALT2,
                                         MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_pt13_1 = { MXC_GPIO2, MXC_GPIO_PIN_11, MXC_GPIO_FUNC_ALT2,
                                         MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_pt14_1 = { MXC_GPIO2, MXC_GPIO_PIN_10, MXC_GPIO_FUNC_ALT2,
                                         MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_pt15_1 = { MXC_GPIO2, MXC_GPIO_PIN_12, MXC_GPIO_FUNC_ALT2,
                                         MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

const mxc_gpio_cfg_t gpio_cfg_hyp = { MXC_GPIO1, (MXC_GPIO_PIN_12 | MXC_GPIO_PIN_13 | MXC_GPIO_PIN_14 | MXC_GPIO_PIN_15 |
                                      MXC_GPIO_PIN_16 | MXC_GPIO_PIN_18 | MXC_GPIO_PIN_19 | MXC_GPIO_PIN_20 | MXC_GPIO_PIN_21),
                                      MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_hyp_cs0 = { MXC_GPIO1, MXC_GPIO_PIN_11, MXC_GPIO_FUNC_ALT1,
                                          MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_hyp_cs1 = { MXC_GPIO3, MXC_GPIO_PIN_0, MXC_GPIO_FUNC_ALT2,
                                          MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

const mxc_gpio_cfg_t gpio_cfg_spixr = { MXC_GPIO0, (MXC_GPIO_PIN_1 | MXC_GPIO_PIN_2 | MXC_GPIO_PIN_3 | MXC_GPIO_PIN_4 |
                                        MXC_GPIO_PIN_5 | MXC_GPIO_PIN_6), MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_WEAK_PULL_UP,
                                        MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_rtcsqw = { MXC_GPIO0, MXC_GPIO_PIN_31, MXC_GPIO_FUNC_ALT1,
                                         MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

const mxc_gpio_cfg_t gpio_cfg_spixfc = { MXC_GPIO0, (MXC_GPIO_PIN_7 | MXC_GPIO_PIN_8 | MXC_GPIO_PIN_9 | MXC_GPIO_PIN_10 |
                                         MXC_GPIO_PIN_11 | MXC_GPIO_PIN_12), MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE,
                                         MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

const mxc_gpio_cfg_t gpio_cfg_sdhc_0 = { MXC_GPIO0, (MXC_GPIO_PIN_31), MXC_GPIO_FUNC_ALT2,
                                         MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_sdhc_1 = { MXC_GPIO1, (MXC_GPIO_PIN_0 | MXC_GPIO_PIN_1 | MXC_GPIO_PIN_2 | MXC_GPIO_PIN_3 |
                                         MXC_GPIO_PIN_4 | MXC_GPIO_PIN_5 | MXC_GPIO_PIN_6), MXC_GPIO_FUNC_ALT1,
                                         MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

// 1-Wire pins need to be at 3.3V so that MXC_GPIO_VSSEL_VDDIOH is selected.
const mxc_gpio_cfg_t gpio_cfg_owm = { MXC_GPIO1, (MXC_GPIO_PIN_30 | MXC_GPIO_PIN_31), MXC_GPIO_FUNC_ALT1,
                                      MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH, MXC_GPIO_DRVSTR_0 };

const mxc_gpio_cfg_t gpio_cfg_clcd_0 = { MXC_GPIO0, (MXC_GPIO_PIN_13 | MXC_GPIO_PIN_14 | MXC_GPIO_PIN_15 | MXC_GPIO_PIN_16 |
                                         MXC_GPIO_PIN_17 | MXC_GPIO_PIN_18 | MXC_GPIO_PIN_19 | MXC_GPIO_PIN_20 | MXC_GPIO_PIN_22 |
                                         MXC_GPIO_PIN_23 | MXC_GPIO_PIN_24 | MXC_GPIO_PIN_25 | MXC_GPIO_PIN_30), MXC_GPIO_FUNC_ALT2,
                                         MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_clcd_1 = { MXC_GPIO1, (MXC_GPIO_PIN_3 | MXC_GPIO_PIN_20 | MXC_GPIO_PIN_23 | MXC_GPIO_PIN_24 |
                                         MXC_GPIO_PIN_25 | MXC_GPIO_PIN_26 | MXC_GPIO_PIN_27 | MXC_GPIO_PIN_28 | MXC_GPIO_PIN_29 |
                                         MXC_GPIO_PIN_30 | MXC_GPIO_PIN_31), MXC_GPIO_FUNC_ALT2, MXC_GPIO_PAD_NONE,
                                         MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_clcd_2 = { MXC_GPIO2, (MXC_GPIO_PIN_2 | MXC_GPIO_PIN_3 | MXC_GPIO_PIN_6 | MXC_GPIO_PIN_13 | MXC_GPIO_PIN_14 |
                                         MXC_GPIO_PIN_15 | MXC_GPIO_PIN_16 | MXC_GPIO_PIN_17 | MXC_GPIO_PIN_18), MXC_GPIO_FUNC_ALT2,
                                         MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

const mxc_gpio_cfg_t gpio_cfg_i2s = { MXC_GPIO2, (MXC_GPIO_PIN_2 | MXC_GPIO_PIN_3 | MXC_GPIO_PIN_4 | MXC_GPIO_PIN_5),
                                      MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

// SPI v2 Pin Definitions
const mxc_gpio_cfg_t gpio_cfg_spi0_standard = { MXC_GPIO3, (MXC_GPIO_PIN_1 | MXC_GPIO_PIN_2 | MXC_GPIO_PIN_3),
                                                MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi0_3wire = { MXC_GPIO3, (MXC_GPIO_PIN_2 | MXC_GPIO_PIN_3), MXC_GPIO_FUNC_ALT1,
                                             MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
// MXC_SPI0 does not support Dual or Quad mode.
const mxc_gpio_cfg_t gpio_cfg_spi1_standard = { MXC_GPIO1, (MXC_GPIO_PIN_26 | MXC_GPIO_PIN_28 | MXC_GPIO_PIN_29),
                                                MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi1_3wire = { MXC_GPIO1, (MXC_GPIO_PIN_26 | MXC_GPIO_PIN_29), MXC_GPIO_FUNC_ALT1,
                                             MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
// MXC_SPI1 does not support Dual or Quad mode.
const mxc_gpio_cfg_t gpio_cfg_spi2_standard = { MXC_GPIO2, (MXC_GPIO_PIN_2 | MXC_GPIO_PIN_3 | MXC_GPIO_PIN_4),
                                                MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi2_3wire = { MXC_GPIO2, (MXC_GPIO_PIN_2 | MXC_GPIO_PIN_4), MXC_GPIO_FUNC_ALT1,
                                             MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
// MXC_SPI2 does not support Dual or Quad mode.

const mxc_gpio_cfg_t gpio_cfg_spi3_standard = { MXC_GPIO0, (MXC_GPIO_PIN_16 | MXC_GPIO_PIN_20 | MXC_GPIO_PIN_21),
                                                MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi3_3wire = { MXC_GPIO0, (MXC_GPIO_PIN_16 | MXC_GPIO_PIN_21),
                                             MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi3_dual = { MXC_GPIO0, (MXC_GPIO_PIN_16 | MXC_GPIO_PIN_20 | MXC_GPIO_PIN_21),
                                            MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi3_quad = { MXC_GPIO0, (MXC_GPIO_PIN_15 | MXC_GPIO_PIN_16 | MXC_GPIO_PIN_17 | MXC_GPIO_PIN_20 | MXC_GPIO_PIN_21),
                                            MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

// SPI v2 Target Selects Pin Definitions
const mxc_gpio_cfg_t gpio_cfg_spi0_ts0 = { MXC_GPIO0, MXC_GPIO_PIN_22, MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi1_ts0 = { MXC_GPIO1, MXC_GPIO_PIN_23, MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi1_ts1 = { MXC_GPIO1, MXC_GPIO_PIN_25, MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi1_ts2 = { MXC_GPIO1, MXC_GPIO_PIN_24, MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi1_ts3 = { MXC_GPIO1, MXC_GPIO_PIN_27, MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi2_ts0 = { MXC_GPIO2, MXC_GPIO_PIN_5, MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi2_ts1 = { MXC_GPIO2, MXC_GPIO_PIN_1, MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi2_ts2 = { MXC_GPIO2, MXC_GPIO_PIN_0, MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi2_ts3 = { MXC_GPIO2, MXC_GPIO_PIN_6, MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi3_ts0 = { MXC_GPIO0, MXC_GPIO_PIN_19, MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi3_ts1 = { MXC_GPIO0, MXC_GPIO_PIN_13, MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi3_ts2 = { MXC_GPIO0, MXC_GPIO_PIN_14, MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi3_ts3 = { MXC_GPIO0, MXC_GPIO_PIN_18, MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
