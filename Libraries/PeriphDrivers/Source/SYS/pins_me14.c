/**
 * @file mxc_pins.c
 * @brief      This file contains constant pin configurations for the peripherals.
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

#include "gpio.h"
#include "mxc_device.h"

//Note: Some of the alternate function definitions differ between WLP and CTBGA
/***** Global Variables *****/

// clang-format off
const mxc_gpio_cfg_t gpio_cfg_tmr0 = { MXC_GPIO0, MXC_GPIO_PIN_0, MXC_GPIO_FUNC_ALT4,
                                       MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_tmr1 = { MXC_GPIO0, MXC_GPIO_PIN_1, MXC_GPIO_FUNC_ALT4,
                                       MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_tmr2 = { MXC_GPIO0, MXC_GPIO_PIN_2, MXC_GPIO_FUNC_ALT4,
                                       MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_tmr3 = { MXC_GPIO0, MXC_GPIO_PIN_3, MXC_GPIO_FUNC_ALT4,
                                       MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_tmr4 = { MXC_GPIO0, MXC_GPIO_PIN_4, MXC_GPIO_FUNC_ALT4,
                                       MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_tmr5 = { MXC_GPIO0, MXC_GPIO_PIN_5, MXC_GPIO_FUNC_ALT4,
                                       MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

const mxc_gpio_cfg_t gpio_cfg_uart0a = { MXC_GPIO0, (MXC_GPIO_PIN_9 | MXC_GPIO_PIN_10), MXC_GPIO_FUNC_ALT3,
                                         MXC_GPIO_PAD_WEAK_PULL_UP, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_uart0a_flow = { MXC_GPIO0, (MXC_GPIO_PIN_8 | MXC_GPIO_PIN_11), MXC_GPIO_FUNC_ALT3,
                                              MXC_GPIO_PAD_WEAK_PULL_UP, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_uart1a = { MXC_GPIO0, (MXC_GPIO_PIN_20 | MXC_GPIO_PIN_21), MXC_GPIO_FUNC_ALT3,
                                         MXC_GPIO_PAD_WEAK_PULL_UP, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_uart1a_flow = { MXC_GPIO0, (MXC_GPIO_PIN_22 | MXC_GPIO_PIN_23), MXC_GPIO_FUNC_ALT3,
                                              MXC_GPIO_PAD_WEAK_PULL_UP, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_uart2a = { MXC_GPIO0, (MXC_GPIO_PIN_1 | MXC_GPIO_PIN_2), MXC_GPIO_FUNC_ALT3,
                                         MXC_GPIO_PAD_WEAK_PULL_UP, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_uart2a_flow = { MXC_GPIO0, (MXC_GPIO_PIN_0 | MXC_GPIO_PIN_3), MXC_GPIO_FUNC_ALT3,
                                              MXC_GPIO_PAD_WEAK_PULL_UP, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

const mxc_gpio_cfg_t gpio_cfg_uart0b = { MXC_GPIO1, (MXC_GPIO_PIN_4 | MXC_GPIO_PIN_5), MXC_GPIO_FUNC_ALT3,
                                         MXC_GPIO_PAD_WEAK_PULL_UP, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_uart0b_flow = { MXC_GPIO1, (MXC_GPIO_PIN_6 | MXC_GPIO_PIN_7), MXC_GPIO_FUNC_ALT3,
                                              MXC_GPIO_PAD_WEAK_PULL_UP, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_uart1b = { MXC_GPIO1, (MXC_GPIO_PIN_12 | MXC_GPIO_PIN_13), MXC_GPIO_FUNC_ALT3,
                                         MXC_GPIO_PAD_WEAK_PULL_UP, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_uart1b_flow = { MXC_GPIO1, (MXC_GPIO_PIN_14 | MXC_GPIO_PIN_15), MXC_GPIO_FUNC_ALT3,
                                              MXC_GPIO_PAD_WEAK_PULL_UP, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_uart2b = { MXC_GPIO0, (MXC_GPIO_PIN_28 | MXC_GPIO_PIN_29), MXC_GPIO_FUNC_ALT3,
                                         MXC_GPIO_PAD_WEAK_PULL_UP, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_uart2b_flow = { MXC_GPIO0, (MXC_GPIO_PIN_30 | MXC_GPIO_PIN_31), MXC_GPIO_FUNC_ALT3,
                                              MXC_GPIO_PAD_WEAK_PULL_UP, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

const mxc_gpio_cfg_t gpio_cfg_i2c0 = { MXC_GPIO0, (MXC_GPIO_PIN_6 | MXC_GPIO_PIN_7), MXC_GPIO_FUNC_ALT1,
                                       MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_i2c1 = { MXC_GPIO0, (MXC_GPIO_PIN_14 | MXC_GPIO_PIN_15), MXC_GPIO_FUNC_ALT1,
                                       MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_i2c2 = { MXC_GPIO1, (MXC_GPIO_PIN_14 | MXC_GPIO_PIN_15), MXC_GPIO_FUNC_ALT1,
                                       MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

const mxc_gpio_cfg_t gpio_cfg_spi0a = { MXC_GPIO1, (MXC_GPIO_PIN_9 | MXC_GPIO_PIN_10 | MXC_GPIO_PIN_11 | MXC_GPIO_PIN_12 | MXC_GPIO_PIN_13),
                                        MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi0b = { MXC_GPIO0, (MXC_GPIO_PIN_9 | MXC_GPIO_PIN_10 | MXC_GPIO_PIN_11 | MXC_GPIO_PIN_12 | MXC_GPIO_PIN_13),
                                        MXC_GPIO_FUNC_ALT2, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi0_ss0a = { MXC_GPIO1, (MXC_GPIO_PIN_8), MXC_GPIO_FUNC_ALT1,
                                            MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi0_ss0b = { MXC_GPIO0, (MXC_GPIO_PIN_8), MXC_GPIO_FUNC_ALT2,
                                            MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi0_ss1 = { MXC_GPIO0, (MXC_GPIO_PIN_14), MXC_GPIO_FUNC_ALT2,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi0_ss2 = { MXC_GPIO0, (MXC_GPIO_PIN_15), MXC_GPIO_FUNC_ALT2,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

const mxc_gpio_cfg_t gpio_cfg_spi1 = { MXC_GPIO0, (MXC_GPIO_PIN_17 | MXC_GPIO_PIN_18 | MXC_GPIO_PIN_19 | MXC_GPIO_PIN_20 | MXC_GPIO_PIN_21),
                                       MXC_GPIO_FUNC_ALT2, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi1_ss0 = { MXC_GPIO0, (MXC_GPIO_PIN_16), MXC_GPIO_FUNC_ALT2,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi1_ss1 = { MXC_GPIO0, (MXC_GPIO_PIN_22), MXC_GPIO_FUNC_ALT2,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi1_ss2 = { MXC_GPIO0, (MXC_GPIO_PIN_23), MXC_GPIO_FUNC_ALT2,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

const mxc_gpio_cfg_t gpio_cfg_spi2 = { MXC_GPIO0, (MXC_GPIO_PIN_25 | MXC_GPIO_PIN_26 | MXC_GPIO_PIN_27 | MXC_GPIO_PIN_28 | MXC_GPIO_PIN_29),
                                       MXC_GPIO_FUNC_ALT2, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi2_ss0 = { MXC_GPIO0, (MXC_GPIO_PIN_24), MXC_GPIO_FUNC_ALT2,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi2_ss1 = { MXC_GPIO0, (MXC_GPIO_PIN_30), MXC_GPIO_FUNC_ALT2,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi2_ss2 = { MXC_GPIO0, (MXC_GPIO_PIN_31), MXC_GPIO_FUNC_ALT2,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

const mxc_gpio_cfg_t gpio_cfg_pt0 = { MXC_GPIO1, MXC_GPIO_PIN_0, MXC_GPIO_FUNC_ALT4,
                                      MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_pt1 = { MXC_GPIO1, MXC_GPIO_PIN_1, MXC_GPIO_FUNC_ALT4,
                                      MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_pt2 = { MXC_GPIO1, MXC_GPIO_PIN_2, MXC_GPIO_FUNC_ALT4,
                                      MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_pt3 = { MXC_GPIO1, MXC_GPIO_PIN_3, MXC_GPIO_FUNC_ALT4,
                                      MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_pt4 = { MXC_GPIO1, MXC_GPIO_PIN_4, MXC_GPIO_FUNC_ALT4,
                                      MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_pt5 = { MXC_GPIO1, MXC_GPIO_PIN_5, MXC_GPIO_FUNC_ALT4,
                                      MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_pt6 = { MXC_GPIO1, MXC_GPIO_PIN_6, MXC_GPIO_FUNC_ALT4,
                                      MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_pt7 = { MXC_GPIO1, MXC_GPIO_PIN_7, MXC_GPIO_FUNC_ALT4,
                                      MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_pt8 = { MXC_GPIO1, MXC_GPIO_PIN_8, MXC_GPIO_FUNC_ALT4,
                                      MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_pt9 = { MXC_GPIO1, MXC_GPIO_PIN_9, MXC_GPIO_FUNC_ALT4,
                                      MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_pt10 = { MXC_GPIO1, MXC_GPIO_PIN_10, MXC_GPIO_FUNC_ALT4,
                                       MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_pt11 = { MXC_GPIO1, MXC_GPIO_PIN_11, MXC_GPIO_FUNC_ALT4,
                                       MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_pt12 = { MXC_GPIO1, MXC_GPIO_PIN_12, MXC_GPIO_FUNC_ALT4,
                                       MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_pt13 = { MXC_GPIO1, MXC_GPIO_PIN_13, MXC_GPIO_FUNC_ALT4,
                                       MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_pt14 = { MXC_GPIO1, MXC_GPIO_PIN_14, MXC_GPIO_FUNC_ALT4,
                                       MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_pt15 = { MXC_GPIO1, MXC_GPIO_PIN_15, MXC_GPIO_FUNC_ALT4,
                                       MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

const mxc_gpio_cfg_t gpio_cfg_spixr = { MXC_GPIO0, (MXC_GPIO_PIN_8 | MXC_GPIO_PIN_9 | MXC_GPIO_PIN_10 | MXC_GPIO_PIN_11 |
                                        MXC_GPIO_PIN_12 | MXC_GPIO_PIN_13), MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_PULL_UP,
                                        MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

const mxc_gpio_cfg_t gpio_cfg_rtcsqw = { MXC_GPIO0, MXC_GPIO_PIN_19, MXC_GPIO_FUNC_OUT,
                                         MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

const mxc_gpio_cfg_t gpio_cfg_spixfc = { MXC_GPIO0, (MXC_GPIO_PIN_0 | MXC_GPIO_PIN_1 | MXC_GPIO_PIN_2 | MXC_GPIO_PIN_3 | MXC_GPIO_PIN_4 |
                                         MXC_GPIO_PIN_5), MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

const mxc_gpio_cfg_t gpio_cfg_sdhc = { MXC_GPIO1, (MXC_GPIO_PIN_0 | MXC_GPIO_PIN_1 | MXC_GPIO_PIN_2 | MXC_GPIO_PIN_3 | MXC_GPIO_PIN_4 |
                                       MXC_GPIO_PIN_5 | MXC_GPIO_PIN_6 | MXC_GPIO_PIN_7), MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE,
                                       MXC_GPIO_VSSEL_VDDIOH, MXC_GPIO_DRVSTR_0 };

const mxc_gpio_cfg_t gpio_cfg_owma = { MXC_GPIO0, (MXC_GPIO_PIN_4 | MXC_GPIO_PIN_5), MXC_GPIO_FUNC_ALT3,
                                       MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_owmb = { MXC_GPIO0, (MXC_GPIO_PIN_12 | MXC_GPIO_PIN_13), MXC_GPIO_FUNC_ALT3,
                                       MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_owmc = { MXC_GPIO0, (MXC_GPIO_PIN_24 | MXC_GPIO_PIN_25), MXC_GPIO_FUNC_ALT3,
                                       MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH, MXC_GPIO_DRVSTR_0 };

const mxc_gpio_cfg_t gpio_cfg_adc0 = { MXC_GPIO0, MXC_GPIO_PIN_16, MXC_GPIO_FUNC_ALT1,
                                       MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_adc1 = { MXC_GPIO0, MXC_GPIO_PIN_17, MXC_GPIO_FUNC_ALT1,
                                       MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_adc2 = { MXC_GPIO0, MXC_GPIO_PIN_18, MXC_GPIO_FUNC_ALT1,
                                       MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_adc3 = { MXC_GPIO0, MXC_GPIO_PIN_19, MXC_GPIO_FUNC_ALT1,
                                       MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_adc4 = { MXC_GPIO0, MXC_GPIO_PIN_20, MXC_GPIO_FUNC_ALT1,
                                       MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_adc5 = { MXC_GPIO0, MXC_GPIO_PIN_21, MXC_GPIO_FUNC_ALT1,
                                       MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_adc6 = { MXC_GPIO0, MXC_GPIO_PIN_22, MXC_GPIO_FUNC_ALT1,
                                       MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_adc7 = { MXC_GPIO0, MXC_GPIO_PIN_23, MXC_GPIO_FUNC_ALT1,
                                       MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

const mxc_gpio_cfg_t gpio_swd_core1 = { MXC_GPIO0, (MXC_GPIO_PIN_6 | MXC_GPIO_PIN_7), MXC_GPIO_FUNC_ALT3,
                                        MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

const mxc_gpio_cfg_t gpio_cfg_sdma = { MXC_GPIO1, (MXC_GPIO_PIN_0 | MXC_GPIO_PIN_1 | MXC_GPIO_PIN_2 | MXC_GPIO_PIN_3),
                                       MXC_GPIO_FUNC_ALT3, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

// SPI v2 Pin Definitions
const mxc_gpio_cfg_t gpio_cfg_spi0a_standard = { MXC_GPIO1, (MXC_GPIO_PIN_9 | MXC_GPIO_PIN_10 | MXC_GPIO_PIN_11),
                                                 MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi0a_3wire = { MXC_GPIO1, (MXC_GPIO_PIN_9 | MXC_GPIO_PIN_11), MXC_GPIO_FUNC_ALT1,
                                              MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi0a_dual = { MXC_GPIO1, (MXC_GPIO_PIN_9 | MXC_GPIO_PIN_10 | MXC_GPIO_PIN_11),
                                             MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi0a_quad = { MXC_GPIO1, (MXC_GPIO_PIN_9 | MXC_GPIO_PIN_10 | MXC_GPIO_PIN_11 | MXC_GPIO_PIN_12 | MXC_GPIO_PIN_13),
                                             MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

const mxc_gpio_cfg_t gpio_cfg_spi0b_standard = { MXC_GPIO0, (MXC_GPIO_PIN_9 | MXC_GPIO_PIN_10 | MXC_GPIO_PIN_11),
                                                 MXC_GPIO_FUNC_ALT2, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi0b_3wire = { MXC_GPIO0, (MXC_GPIO_PIN_9 | MXC_GPIO_PIN_11), MXC_GPIO_FUNC_ALT2,
                                              MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi0b_dual = { MXC_GPIO0, (MXC_GPIO_PIN_9 | MXC_GPIO_PIN_10 | MXC_GPIO_PIN_11),
                                             MXC_GPIO_FUNC_ALT2, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi0b_quad = { MXC_GPIO0, (MXC_GPIO_PIN_9 | MXC_GPIO_PIN_10 | MXC_GPIO_PIN_11 | MXC_GPIO_PIN_12 | MXC_GPIO_PIN_13),
                                             MXC_GPIO_FUNC_ALT2, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

const mxc_gpio_cfg_t gpio_cfg_spi1_standard = { MXC_GPIO0, (MXC_GPIO_PIN_17 | MXC_GPIO_PIN_18 | MXC_GPIO_PIN_19),
                                                MXC_GPIO_FUNC_ALT2, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi1_3wire = { MXC_GPIO0, (MXC_GPIO_PIN_17 | MXC_GPIO_PIN_19), MXC_GPIO_FUNC_ALT2,
                                             MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi1_dual = { MXC_GPIO0, (MXC_GPIO_PIN_17 | MXC_GPIO_PIN_18 | MXC_GPIO_PIN_19),
                                            MXC_GPIO_FUNC_ALT2, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi1_quad = { MXC_GPIO0, (MXC_GPIO_PIN_17 | MXC_GPIO_PIN_18 | MXC_GPIO_PIN_19 | MXC_GPIO_PIN_20 | MXC_GPIO_PIN_21),
                                            MXC_GPIO_FUNC_ALT2, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

const mxc_gpio_cfg_t gpio_cfg_spi2_standard = { MXC_GPIO0, (MXC_GPIO_PIN_25 | MXC_GPIO_PIN_26 | MXC_GPIO_PIN_27),
                                                MXC_GPIO_FUNC_ALT2, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi2_3wire = { MXC_GPIO0, (MXC_GPIO_PIN_25 | MXC_GPIO_PIN_27), MXC_GPIO_FUNC_ALT2,
                                             MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi2_dual = { MXC_GPIO0, (MXC_GPIO_PIN_25 | MXC_GPIO_PIN_26 | MXC_GPIO_PIN_27),
                                            MXC_GPIO_FUNC_ALT2, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi2_quad = { MXC_GPIO0, (MXC_GPIO_PIN_25 | MXC_GPIO_PIN_26 | MXC_GPIO_PIN_27 | MXC_GPIO_PIN_28 | MXC_GPIO_PIN_29),
                                            MXC_GPIO_FUNC_ALT2, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

// SPI v2 Target Selects Pin Definitions
const mxc_gpio_cfg_t gpio_cfg_spi0a_ts0 = { MXC_GPIO1, MXC_GPIO_PIN_8, MXC_GPIO_FUNC_ALT1,
                                            MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi0b_ts0 = { MXC_GPIO0, MXC_GPIO_PIN_8, MXC_GPIO_FUNC_ALT2,
                                            MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi0_ts1 = { MXC_GPIO0, MXC_GPIO_PIN_14, MXC_GPIO_FUNC_ALT2,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi0_ts2 = { MXC_GPIO0, MXC_GPIO_PIN_15, MXC_GPIO_FUNC_ALT2,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi1_ts0 = { MXC_GPIO0, MXC_GPIO_PIN_16, MXC_GPIO_FUNC_ALT2,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi1_ts1 = { MXC_GPIO0, MXC_GPIO_PIN_22, MXC_GPIO_FUNC_ALT2,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi1_ts2 = { MXC_GPIO0, MXC_GPIO_PIN_23, MXC_GPIO_FUNC_ALT2,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi2_ts0 = { MXC_GPIO0, MXC_GPIO_PIN_24, MXC_GPIO_FUNC_ALT2,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi2_ts1 = { MXC_GPIO0, MXC_GPIO_PIN_30, MXC_GPIO_FUNC_ALT2,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi2_ts2 = { MXC_GPIO0, MXC_GPIO_PIN_31, MXC_GPIO_FUNC_ALT2,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
