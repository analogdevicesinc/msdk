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

/***** Definitions *****/

/***** Global Variables *****/

// clang-format off
const mxc_gpio_cfg_t gpio_cfg_i2c0 = { MXC_GPIO0, (MXC_GPIO_PIN_0 | MXC_GPIO_PIN_1),
                                       MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE,
                                       MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_i2c1 = { MXC_GPIO2, (MXC_GPIO_PIN_18 | MXC_GPIO_PIN_19),
                                       MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE,
                                       MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_i2c2 = { MXC_GPIO0, (MXC_GPIO_PIN_6 | MXC_GPIO_PIN_7),
                                       MXC_GPIO_FUNC_ALT4, MXC_GPIO_PAD_NONE,
                                       MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_i2c2b = { MXC_GPIO1, (MXC_GPIO_PIN_20 | MXC_GPIO_PIN_21),
                                        MXC_GPIO_FUNC_ALT2, MXC_GPIO_PAD_NONE,
                                        MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_i2c2c = { MXC_GPIO1, (MXC_GPIO_PIN_30 | MXC_GPIO_PIN_31),
                                        MXC_GPIO_FUNC_ALT3, MXC_GPIO_PAD_NONE,
                                        MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

const mxc_gpio_cfg_t gpio_cfg_uart0 = { MXC_GPIO1, (MXC_GPIO_PIN_8 | MXC_GPIO_PIN_9),
                                        MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE,
                                        MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_uart0_flow = { MXC_GPIO1, (MXC_GPIO_PIN_6 | MXC_GPIO_PIN_7),
                                             MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE,
                                             MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_uart1 = { MXC_GPIO1, (MXC_GPIO_PIN_12 | MXC_GPIO_PIN_13),
                                        MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE,
                                        MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_uart1_flow = { MXC_GPIO1, (MXC_GPIO_PIN_10 | MXC_GPIO_PIN_11),
                                             MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE,
                                             MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_uart2 = { MXC_GPIO1, (MXC_GPIO_PIN_16 | MXC_GPIO_PIN_17),
                                        MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE,
                                        MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_uart2_flow = { MXC_GPIO1, (MXC_GPIO_PIN_14 | MXC_GPIO_PIN_15),
                                             MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE,
                                             MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_uart3 = { MXC_GPIO1, (MXC_GPIO_PIN_20 | MXC_GPIO_PIN_21),
                                        MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE,
                                        MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_uart3_flow = { MXC_GPIO1, (MXC_GPIO_PIN_18 | MXC_GPIO_PIN_19),
                                             MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE,
                                             MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_uart4 = { MXC_GPIO1, (MXC_GPIO_PIN_28 | MXC_GPIO_PIN_29),
                                        MXC_GPIO_FUNC_ALT2, MXC_GPIO_PAD_NONE,
                                        MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_uart4b = { MXC_GPIO2, (MXC_GPIO_PIN_26 | MXC_GPIO_PIN_27),
                                         MXC_GPIO_FUNC_ALT3, MXC_GPIO_PAD_NONE,
                                         MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_uart4c = { MXC_GPIO1, (MXC_GPIO_PIN_1 | MXC_GPIO_PIN_2),
                                         MXC_GPIO_FUNC_ALT4, MXC_GPIO_PAD_NONE,
                                         MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_uart4_flow = { MXC_GPIO2, (MXC_GPIO_PIN_0 | MXC_GPIO_PIN_1),
                                             MXC_GPIO_FUNC_ALT2, MXC_GPIO_PAD_NONE,
                                             MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_uart4b_flow = { MXC_GPIO2, (MXC_GPIO_PIN_28 | MXC_GPIO_PIN_29),
                                              MXC_GPIO_FUNC_ALT3, MXC_GPIO_PAD_NONE,
                                              MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_uart4c_flow = { MXC_GPIO1, (MXC_GPIO_PIN_3 | MXC_GPIO_PIN_4),
                                              MXC_GPIO_FUNC_ALT4, MXC_GPIO_PAD_NONE,
                                              MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_uart5 = { MXC_GPIO1, (MXC_GPIO_PIN_30 | MXC_GPIO_PIN_31),
                                        MXC_GPIO_FUNC_ALT2, MXC_GPIO_PAD_NONE,
                                        MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_uart5b = { MXC_GPIO2, (MXC_GPIO_PIN_30 | MXC_GPIO_PIN_31),
                                         MXC_GPIO_FUNC_ALT3, MXC_GPIO_PAD_NONE,
                                         MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_uart5_flow = { MXC_GPIO2, (MXC_GPIO_PIN_8 | MXC_GPIO_PIN_9),
                                             MXC_GPIO_FUNC_ALT2, MXC_GPIO_PAD_NONE,
                                             MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_uart5b_flow = { MXC_GPIO2, (MXC_GPIO_PIN_4 | MXC_GPIO_PIN_5),
                                              MXC_GPIO_FUNC_ALT3, MXC_GPIO_PAD_NONE,
                                              MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
// NOTE: uart5b flow control is split across port pins 0.31 and 1.0
const mxc_gpio_cfg_t gpio_cfg_uart5c_P1_flow = { MXC_GPIO1, MXC_GPIO_PIN_0, MXC_GPIO_FUNC_ALT4,
                                                 MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_uart5c_P0_flow = { MXC_GPIO0, MXC_GPIO_PIN_31, MXC_GPIO_FUNC_ALT4,
                                                 MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

const mxc_gpio_cfg_t gpio_cfg_spi0 = {
    MXC_GPIO0, (MXC_GPIO_PIN_2 | MXC_GPIO_PIN_3 | MXC_GPIO_PIN_4 | MXC_GPIO_PIN_5),
    MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0
};
// NOTE: SPI1 definied here with SS1 only, SS0 is on port0 by itself.
const mxc_gpio_cfg_t gpio_cfg_spi1 = {
    MXC_GPIO1, (MXC_GPIO_PIN_0 | MXC_GPIO_PIN_3 | MXC_GPIO_PIN_4 | MXC_GPIO_PIN_5),
    MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0
};
// NOTE: SPI2 defined here with SS0 only, and NOT SS1 and SS2
const mxc_gpio_cfg_t gpio_cfg_spi2 = {
    MXC_GPIO2, (MXC_GPIO_PIN_14 | MXC_GPIO_PIN_15 | MXC_GPIO_PIN_16 | MXC_GPIO_PIN_17),
    MXC_GPIO_FUNC_ALT2, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0
};
const mxc_gpio_cfg_t gpio_cfg_spi2b = {
    MXC_GPIO2, (MXC_GPIO_PIN_22 | MXC_GPIO_PIN_23 | MXC_GPIO_PIN_24 | MXC_GPIO_PIN_25),
    MXC_GPIO_FUNC_ALT3, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0
};
// NOTE: SPI3 defined here with SS0 only, and NOT SS1, SS2, or SS3
const mxc_gpio_cfg_t gpio_cfg_spi3 = { MXC_GPIO0,
                                       (MXC_GPIO_PIN_6 | MXC_GPIO_PIN_10 | MXC_GPIO_PIN_11 |
                                        MXC_GPIO_PIN_12 | MXC_GPIO_PIN_13 | MXC_GPIO_PIN_14),
                                       MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE,
                                       MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

// Timers are only defined once, depending on package, each timer could be mapped to other pins
const mxc_gpio_cfg_t gpio_cfg_tmr0 = { MXC_GPIO1, MXC_GPIO_PIN_14, MXC_GPIO_FUNC_ALT3,
                                       MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_tmr1 = { MXC_GPIO1, MXC_GPIO_PIN_15, MXC_GPIO_FUNC_ALT3,
                                       MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_tmr2 = { MXC_GPIO2, MXC_GPIO_PIN_24, MXC_GPIO_FUNC_ALT2,
                                       MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_tmr3 = { MXC_GPIO2, MXC_GPIO_PIN_25, MXC_GPIO_FUNC_ALT2,
                                       MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_tmr4 = { MXC_GPIO1, MXC_GPIO_PIN_12, MXC_GPIO_FUNC_ALT3,
                                       MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_tmr5 = { MXC_GPIO1, MXC_GPIO_PIN_13, MXC_GPIO_FUNC_ALT3,
                                       MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

// Pulse trains are only defined once, depending on package, each PT could be mapped to other pins
const mxc_gpio_cfg_t gpio_cfg_pt0 = { MXC_GPIO2, MXC_GPIO_PIN_28, MXC_GPIO_FUNC_ALT2,
                                      MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_pt1 = { MXC_GPIO1, MXC_GPIO_PIN_11, MXC_GPIO_FUNC_ALT4,
                                      MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_pt2 = { MXC_GPIO0, MXC_GPIO_PIN_11, MXC_GPIO_FUNC_ALT4,
                                      MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_pt3 = { MXC_GPIO0, MXC_GPIO_PIN_12, MXC_GPIO_FUNC_ALT4,
                                      MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_pt4 = { MXC_GPIO0, MXC_GPIO_PIN_13, MXC_GPIO_FUNC_ALT4,
                                      MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_pt5 = { MXC_GPIO1, MXC_GPIO_PIN_7, MXC_GPIO_FUNC_ALT3,
                                      MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_pt6 = { MXC_GPIO1, MXC_GPIO_PIN_0, MXC_GPIO_FUNC_ALT3,
                                      MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_pt7 = { MXC_GPIO0, MXC_GPIO_PIN_31, MXC_GPIO_FUNC_ALT3,
                                      MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

const mxc_gpio_cfg_t gpio_cfg_owm = { MXC_GPIO0, (MXC_GPIO_PIN_0 | MXC_GPIO_PIN_1),
                                      MXC_GPIO_FUNC_ALT4, MXC_GPIO_PAD_NONE,
                                      MXC_GPIO_VSSEL_VDDIOH };
const mxc_gpio_cfg_t gpio_cfg_owmb = { MXC_GPIO1, (MXC_GPIO_PIN_18 | MXC_GPIO_PIN_19),
                                       MXC_GPIO_FUNC_ALT2, MXC_GPIO_PAD_NONE,
                                       MXC_GPIO_VSSEL_VDDIOH };

// Port 0 Pins 6-14, Port 1 Pins 1-5 and 16-19, Port 2 Pins 10-19
// Other configurations are available, depending on package, to allow the use of EMAC or SDHC
// Note that both P1a and P1b must be configured for proper operation
const mxc_gpio_cfg_t gpio_cfg_P0_clcd = { MXC_GPIO0, 0x00007FC0, MXC_GPIO_FUNC_ALT3,
                                          MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_P1a_clcd = { MXC_GPIO1, 0x000F003E, MXC_GPIO_FUNC_ALT3,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_P1b_clcd = { MXC_GPIO1, 0x00300000, MXC_GPIO_FUNC_ALT4,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_P2_clcd = { MXC_GPIO2, 0x000FFC00, MXC_GPIO_FUNC_ALT3,
                                          MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

const mxc_gpio_cfg_t gpio_cfg_rtcsqw = { MXC_GPIO0, MXC_GPIO_PIN_8, MXC_GPIO_FUNC_ALT4,
                                         MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_rtcsqwb = { MXC_GPIO1, MXC_GPIO_PIN_11, MXC_GPIO_FUNC_ALT2,
                                          MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

const mxc_gpio_cfg_t gpio_cfg_sdhc = { MXC_GPIO2,
                                       (MXC_GPIO_PIN_10 | MXC_GPIO_PIN_11 | MXC_GPIO_PIN_12 |
                                        MXC_GPIO_PIN_13 | MXC_GPIO_PIN_14 | MXC_GPIO_PIN_15 |
                                        MXC_GPIO_PIN_16 | MXC_GPIO_PIN_17),
                                       MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE,
                                       MXC_GPIO_VSSEL_VDDIOH };
const mxc_gpio_cfg_t gpio_cfg_sdhcb = { MXC_GPIO1,
                                        (MXC_GPIO_PIN_14 | MXC_GPIO_PIN_15 | MXC_GPIO_PIN_22 |
                                         MXC_GPIO_PIN_23 | MXC_GPIO_PIN_24 | MXC_GPIO_PIN_25 |
                                         MXC_GPIO_PIN_26 | MXC_GPIO_PIN_27),
                                        MXC_GPIO_FUNC_ALT4, MXC_GPIO_PAD_NONE,
                                        MXC_GPIO_VSSEL_VDDIOH };

const mxc_gpio_cfg_t gpio_cfg_sc0 = { MXC_GPIO0,
                                      (MXC_GPIO_PIN_15 | MXC_GPIO_PIN_16 | MXC_GPIO_PIN_17 |
                                       MXC_GPIO_PIN_18 | MXC_GPIO_PIN_19 | MXC_GPIO_PIN_20),
                                      MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_sc1 = { MXC_GPIO0,
                                      (MXC_GPIO_PIN_22 | MXC_GPIO_PIN_23 | MXC_GPIO_PIN_24 |
                                       MXC_GPIO_PIN_25 | MXC_GPIO_PIN_26 | MXC_GPIO_PIN_27),
                                      MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

// Note that both P0 and P1 must be configured for proper operation
const mxc_gpio_cfg_t gpio_cfg_spixf = { MXC_GPIO1,
                                        (MXC_GPIO_PIN_22 | MXC_GPIO_PIN_23 | MXC_GPIO_PIN_24 |
                                         MXC_GPIO_PIN_25 | MXC_GPIO_PIN_26 | MXC_GPIO_PIN_27),
                                        MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE,
                                        MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spixr_P0 = {
    MXC_GPIO1, (MXC_GPIO_PIN_28 | MXC_GPIO_PIN_29 | MXC_GPIO_PIN_30 | MXC_GPIO_PIN_31),
    MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_PULL_UP
};
const mxc_gpio_cfg_t gpio_cfg_spixr_P1 = { MXC_GPIO2, (MXC_GPIO_PIN_0 | MXC_GPIO_PIN_1),
                                           MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_PULL_UP };

// Note that both P2a and P2b must be configured for proper operation
const mxc_gpio_cfg_t gpio_cfg_emac_P2a = { MXC_GPIO2, 0x000003FC, MXC_GPIO_FUNC_ALT4,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_emac_P2b = { MXC_GPIO2, 0xFFE00000, MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

// Note that all of the following must be configured for proper operation
const mxc_gpio_cfg_t gpio_cfg_kbd_P2 = { MXC_GPIO2, 0x000003FC, MXC_GPIO_FUNC_ALT1,
                                         MXC_GPIO_PAD_PULL_UP };

// Note that both P0 and P1 must be configured for proper operation
const mxc_gpio_cfg_t gpio_cfg_pcif_P0_BITS_0_7 = {
    MXC_GPIO0,
    (MXC_GPIO_PIN_6 | MXC_GPIO_PIN_7 | MXC_GPIO_PIN_8 | MXC_GPIO_PIN_9 | MXC_GPIO_PIN_10 |
     MXC_GPIO_PIN_11 | MXC_GPIO_PIN_12 | MXC_GPIO_PIN_13),
    MXC_GPIO_FUNC_ALT2, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH
};
const mxc_gpio_cfg_t gpio_cfg_pcif_P0_BITS_8 = { MXC_GPIO0, (MXC_GPIO_PIN_14), MXC_GPIO_FUNC_ALT2,
                                                 MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH, MXC_GPIO_DRVSTR_0};
const mxc_gpio_cfg_t gpio_cfg_pcif_P1_BITS_9 = { MXC_GPIO1, (MXC_GPIO_PIN_14), MXC_GPIO_FUNC_ALT2,
                                                 MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH, MXC_GPIO_DRVSTR_0};
const mxc_gpio_cfg_t gpio_cfg_pcif_P1_BITS_10_11 = { MXC_GPIO1, (MXC_GPIO_PIN_1 | MXC_GPIO_PIN_15),
                                                     MXC_GPIO_FUNC_ALT2, MXC_GPIO_PAD_NONE,
                                                     MXC_GPIO_VSSEL_VDDIOH };
const mxc_gpio_cfg_t gpio_cfg_pcif_hsync = { MXC_GPIO1, MXC_GPIO_PIN_2, MXC_GPIO_FUNC_ALT2,
                                             MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH, MXC_GPIO_DRVSTR_0};
const mxc_gpio_cfg_t gpio_cfg_pcif_vsync = { MXC_GPIO1, MXC_GPIO_PIN_18, MXC_GPIO_FUNC_ALT4,
                                             MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH, MXC_GPIO_DRVSTR_0};
const mxc_gpio_cfg_t gpio_cfg_pcif_pclk = { MXC_GPIO1, MXC_GPIO_PIN_19, MXC_GPIO_FUNC_ALT4,
                                            MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH, MXC_GPIO_DRVSTR_0};
const mxc_gpio_cfg_t gpio_cfg_pcif_pwrdwn = { MXC_GPIO1, MXC_GPIO_PIN_21, MXC_GPIO_FUNC_OUT,
                                              MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH, MXC_GPIO_DRVSTR_0};
