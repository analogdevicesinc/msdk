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

#include "i2c_mngr.h"

#include <stdio.h>

#include "i2c.h"
#include "mxc_lock.h"

/*
 * @brief I2C transaction manager
 */
typedef struct {
    uint32_t lock[MXC_I2C_INSTANCES];
    mxc_i2c_regs_t *inst0;
    mxc_i2c_regs_t *inst1;
    mxc_i2c_regs_t *inst2;
} i2c_txn_mngr_t;

static i2c_txn_mngr_t s_mngr;

/******************************************************************************/
int I2C_MNGR_Init()
{
    s_mngr.inst0 = MXC_I2C0_BUS0;
    s_mngr.inst1 = MXC_I2C1_BUS0;
    s_mngr.inst2 = MXC_I2C2_BUS0;

    return E_NO_ERROR;
}

/******************************************************************************/
int I2C_MNGR_Transact(const i2c_mngr_txn_t *transaction)
{
    mxc_i2c_regs_t *inst = transaction->slave_config->i2c_instance;
    int idx = MXC_I2C_GET_IDX(inst);
    int error;

    // Check if valid I2C instance
    if (idx < 0) {
        return E_INVALID;
    }

    // Attempt to acquire I2C lock
    error = MXC_GetLock(&s_mngr.lock[idx], 1);
    if (error != E_NO_ERROR) {
        return error;
    }

    // Initialize I2C
    error = MXC_I2C_Init(inst, 1, 0);
    if (error != E_NO_ERROR) {
        MXC_FreeLock(&s_mngr.lock[idx]);
        return error;
    }

    // Configure I2C bus settings
    error = MXC_I2C_SetFrequency(inst, transaction->slave_config->freq);
    if (error < 0) {
        MXC_FreeLock(&s_mngr.lock[idx]);
        return error;
    }
    error = MXC_I2C_SetClockStretching(inst, transaction->slave_config->clock_stretching);
    if (error != E_NO_ERROR) {
        MXC_FreeLock(&s_mngr.lock[idx]);
        return error;
    }
    MXC_I2C_SetTimeout(inst, transaction->slave_config->timeout);

    // Request I2C Transaction
    mxc_i2c_req_t reqMaster;
    reqMaster.i2c = inst;
    reqMaster.addr = transaction->slave_config->slave_addr;
    reqMaster.tx_buf = transaction->p_tx_data;
    reqMaster.tx_len = transaction->tx_len;
    reqMaster.rx_buf = transaction->p_rx_data;
    reqMaster.rx_len = transaction->rx_len;
    reqMaster.restart = 0;
    reqMaster.callback = NULL;
    error = MXC_I2C_MasterTransaction(&reqMaster);

    // Cleanup
    MXC_I2C_Shutdown(inst);
    MXC_FreeLock(&s_mngr.lock[idx]);

    return error;
};
