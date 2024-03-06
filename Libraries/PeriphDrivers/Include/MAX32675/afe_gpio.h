/**
 * @file    gpio.h
 * @brief   General-Purpose Input/Output (GPIO) function prototypes and data types.
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

/* Define to prevent redundant inclusion */
#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32675_AFE_GPIO_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32675_AFE_GPIO_H_

/* **** Includes **** */
#include "gpio_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/* **** Definitions **** */

/* **** Function Prototypes **** */
/**
 * @brief      Configure GPIO pin(s) and used only for internal HART pins.
 * @param      cfg   Pointer to configuration structure describing the pin.
 * @return     #E_NO_ERROR if everything is successful. See \ref MXC_Error_Codes for the list of error codes.
 */
int MXC_AFE_GPIO_Config(const mxc_gpio_cfg_t *cfg);

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32675_AFE_GPIO_H_
