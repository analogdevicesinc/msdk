/**
 * @file    adt7420_driver.h
 * @brief   ADT7420 IC driver header
 * @details Defines ADT7420 registers
 *          Implements helper macros
 **/

/******************************************************************************
 *
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

#ifndef LIBRARIES_MISCDRIVERS_TEMPSENSOR_ADT7420_ADT7420_DRIVER_H_
#define LIBRARIES_MISCDRIVERS_TEMPSENSOR_ADT7420_ADT7420_DRIVER_H_

#include <stdint.h>
#include <stdio.h>
#include <float.h>
#include "mxc_delay.h"
#include "i2c.h"

/**@def ADT7420_I2C_SLAVE_ADDR
 * @brief I2C slave addresses
 **/
#define ADT7420_I2C_SLAVE_ADDR0 0x48
#define ADT7420_I2C_SLAVE_ADDR1 0x49 // Default Address For EVAL-ADT7420MBZ
#define ADT7420_I2C_SLAVE_ADDR2 0x4A
#define ADT7420_I2C_SLAVE_ADDR3 0x4B
/**@def ADT7420_I2C_SLAVE_ADDR1
 * @brief I2C slave addresses
 **/

/**
 * @brief Structure with sensor function pointers
 */
typedef struct {
    int (*init)(mxc_i2c_regs_t *i2c, uint8_t addr); ///< Pointer to
    int (*read)(void *buf);
} adt7420_driver_t;

/**
 * @brief Prepare I2C_SensorDriver function pointers
 *
 * @return I2C_SensorDriver instance
 */
adt7420_driver_t ADT7420_Open();

#endif // LIBRARIES_MISCDRIVERS_TEMPSENSOR_ADT7420_ADT7420_DRIVER_H_
