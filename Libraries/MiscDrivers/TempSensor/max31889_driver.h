/**
 * @file    max31889_driver.h
 * @brief   MAX31889 IC driver header
 * @details Defines MAX31889 registers
 *          Implements helper macros
 **/

/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. All Rights Reserved.
 * (now owned by Analog Devices, Inc.),
 * Copyright (C) 2023 Analog Devices, Inc. All Rights Reserved. This software
 * is proprietary to Analog Devices, Inc. and its licensors.
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

#ifndef LIBRARIES_MISCDRIVERS_TEMPSENSOR_MAX31889_DRIVER_H_
#define LIBRARIES_MISCDRIVERS_TEMPSENSOR_MAX31889_DRIVER_H_

#include <stdint.h>
#include <stdio.h>
#include <float.h>
#include "mxc_delay.h"
#include "i2c.h"

/**@def MAX31889_I2C_SLAVE_ADDR0
 * @brief I2C slave address (option 0)
 **/
#define MAX31889_I2C_SLAVE_ADDR0 (0xA0 >> 1)

/**@def MAX31889_I2C_SLAVE_ADDR1
 * @brief I2C slave address (option 1)
 **/
#define MAX31889_I2C_SLAVE_ADDR1 (0xA6 >> 1)

/**
 * @brief Structure with sensor function pointers
 */
typedef struct {
    int (*init)(mxc_i2c_regs_t *i2c, uint8_t addr); ///< Pointer to
    int (*read)(void *buf);
} max31889_driver_t;

/**
 * @brief Prepare I2C_SensorDriver function pointers
 *
 * @return I2C_SensorDriver instance
 */
max31889_driver_t MAX31889_Open();

#endif // LIBRARIES_MISCDRIVERS_TEMPSENSOR_MAX31889_DRIVER_H_
