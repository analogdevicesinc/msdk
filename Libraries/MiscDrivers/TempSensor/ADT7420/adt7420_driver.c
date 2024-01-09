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

#include "adt7420_driver.h"

/*******************************************************************************************/
/******************************* ADT7420 (I2C) registers ***********************************/

#define ADT7420_REG_TEMP_MSB 0x00 // Temperature value MSB
#define ADT7420_REG_TEMP_LSB 0x01 // Temperature value LSB
#define ADT7420_REG_STATUS 0x02 // Status
#define ADT7420_REG_CONFIG 0x03 // Configuration
#define ADT7420_REG_T_HIGH_MSB 0x04 // Temperature HIGH setpoint MSB
#define ADT7420_REG_T_HIGH_LSB 0x05 // Temperature HIGH setpoint LSB
#define ADT7420_REG_T_LOW_MSB 0x06 // Temperature LOW setpoint MSB
#define ADT7420_REG_T_LOW_LSB 0x07 // Temperature LOW setpoint LSB
#define ADT7420_REG_T_CRIT_MSB 0x08 // Temperature CRIT setpoint MSB
#define ADT7420_REG_T_CRIT_LSB 0x09 // Temperature CRIT setpoint LSB
#define ADT7420_REG_HIST 0x0A // Temperature HYST setpoint
#define ADT7420_REG_ID 0x0B // ID
#define ADT7420_REG_RESET 0x2F // Software reset

/******************************* Bit definitions *******************************/
// STATUS REGISTER BITS
#define ADT7420_STATUS_TLOW_BIT 4
#define ADT7420_STATUS_THGH_BIT 5
#define ADT7420_STATUS_TCRI_BIT 6
#define ADT7420_STATUS_RDY_BIT 7 // Goes low when result written in TEMP Registers

//CONFIGURATION REGISTER BITS
#define ADT7420_CONFIG_FLTQ0 0 //Fault Queue BIT0
#define ADT7420_CONFIG_FLTQ1 1 //Fault Queue BIT1
#define ADT7420_CONFIG_CTPP 2 // CT Pin Polarity
#define ADT7420_CONFIG_INTPP 3 // INT Pin Polarity
#define ADT7420_CONFIG_INT_CT_MODE 4
#define ADT7420_CONFIG_OPMODE0 5
#define ADT7420_CONFIG_OPMODE1 6
#define ADT7420_CONFIG_RESOLUTION 7

/******************************* Constants *******************************/
#define ADT7420_PART_ID_VALUE 0xCB

/******************************* Useful Definitions *******************************/
#define SET_BIT(cmd, bit) cmd |= (1 << bit)
#define GET_BIT(cmd, bit) cmd &(1 << bit)
#define CLEAR_BIT(cmd, bit) cmd &= ~(1 << bit)

/******************************* Globals *******************************/
static mxc_i2c_req_t req; ///< I2C request

/******************************* Functions *******************************/
static int i2c_transfer(mxc_i2c_req_t *req, uint8_t *txData, int txSize, uint8_t *rxData,
                        int rxSize)
{
    req->tx_buf = txData; // Write data buffer
    req->tx_len = txSize; // Number of bytes to write
    req->rx_buf = rxData; // Read data buffer
    req->rx_len = rxSize; // Number of bytes to read
    return MXC_I2C_MasterTransaction(req);
}

static int i2c_write(mxc_i2c_req_t *req, uint8_t *txData, int txSize) // Necessary for future use
{
    return i2c_transfer(req, txData, txSize, NULL, 0); // Create I2C write request
}

static int i2c_read(mxc_i2c_req_t *req, uint8_t *txData, uint8_t *rxData, int rxSize)
{
    return i2c_transfer(req, txData, 1, rxData, rxSize); // Create I2C read request
}

static float adt7420_convert_to_temp(uint8_t *reading)
{
    return 0.0625f * ((reading[1] + ((uint16_t)reading[0] << 8)) >>
                      3); //uses 13-bit temperature reading by default
}

static int adt7420_read_temperature(float *temp)
{
    int ret_code = E_NO_ERROR;
    uint8_t reg[2];
    uint8_t tr[2]; //temperature reading
    uint8_t status;

    // Read status
    reg[0] = ADT7420_REG_STATUS;
    ret_code = i2c_read(&req, &reg[0], &status, 1);
    if (ret_code != E_NO_ERROR) {
        return ret_code;
    }

    // Wait
    MXC_Delay(100);

    // Check whether conversion is finished
    if ((GET_BIT(status, ADT7420_STATUS_RDY_BIT)) == 0) {
        // Temperature reading is the 13 MSB of 16 bit (2-byte) value
        reg[0] = ADT7420_REG_TEMP_MSB;
        ret_code = i2c_read(&req, &reg[0], tr, 2);
        if (ret_code != E_NO_ERROR) {
            return ret_code;
        }

        // Convert the reading to temperature
        *temp = adt7420_convert_to_temp(tr);

    } else {
        return E_BUSY;
    }

    return ret_code;
}

static int adt7420_init(mxc_i2c_regs_t *i2c, uint8_t addr)
{
    int err = E_NO_ERROR;

    req.i2c = i2c;
    req.addr = addr;
    req.tx_buf = NULL;
    req.tx_len = 0;
    req.rx_buf = NULL;
    req.rx_len = 0;
    req.restart = 0;
    req.callback = NULL;

    // Read sensor ID
    uint8_t reg = ADT7420_REG_ID;
    uint8_t id = 0;
    err = i2c_read(&req, &reg, &id, 1);
    if (err != E_NO_ERROR) {
        return err;
    }
    printf("\nThe Device ID is: 0x%02X \n", id); // Print Sensor ID
    // Verify sensor ID
    if (id != ADT7420_PART_ID_VALUE) {
        printf("\nWrong sensor ID");
        return E_NO_DEVICE;
    }

    // Wait
    MXC_Delay(100);

    return E_NO_ERROR;
}

static int adt7420_read(void *buff)
{
    // Read conversion result (Automatic conversion is done by default)
    return adt7420_read_temperature((float *)buff);
}

adt7420_driver_t ADT7420_Open(void)
{
    adt7420_driver_t SD;
    SD.init = adt7420_init;
    SD.read = adt7420_read;
    return SD;
}
