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

#ifndef EXAMPLES_MAX32680_I2C_MNGR_I2C_MNGR_I2C_MNGR_H_
#define EXAMPLES_MAX32680_I2C_MNGR_I2C_MNGR_I2C_MNGR_H_

#include <stdbool.h>

#include "i2c_regs.h"
#include "mxc_device.h"

/*
 * @brief I2C slave device configuration
 */
typedef struct {
    mxc_i2c_regs_t *i2c_instance; ///< I2C peripheral instance
    uint8_t slave_addr; ///< I2C slave
    uint32_t freq; ///< I2C bus frequency
    uint32_t timeout; ///< I2C transaction timeout
    bool clock_stretching; ///< I2C clock stretching flag
} i2c_mngr_slv_config_t;

/*
 * @brief I2C transaction
 */
typedef struct {
    i2c_mngr_slv_config_t *slave_config; ///< I2C device configuration
    uint8_t *p_rx_data; ///< RX data buffer pointer
    uint32_t rx_len; ///< RX data length
    uint8_t *p_tx_data; ///< TX data buffer pointer
    uint32_t tx_len; ///< TX data length
} i2c_mngr_txn_t;

/* Function prototypes */

/*
 * @brief Initializes I2C transaction manager
 * @return #E_NO_ERROR if succeeded, error code otherwise
 */
int I2C_MNGR_Init();

/*
 * @brief Executes I2C transaction
 * @param transaction The trancaction to execute
 * @return #E_NO_ERROR if succeeded, error code otherwise
 */
int I2C_MNGR_Transact(const i2c_mngr_txn_t *transaction);

#endif // EXAMPLES_MAX32680_I2C_MNGR_I2C_MNGR_I2C_MNGR_H_
