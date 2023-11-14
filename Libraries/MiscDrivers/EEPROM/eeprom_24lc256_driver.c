/******************************************************************************
 *
 * Copyright 2023 Analog Devices, Inc.
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

/***** Includes *****/
#include "eeprom_24lc256_driver.h"

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

static int i2c_write(mxc_i2c_req_t *req, uint8_t *txData, int txSize)
{
    return i2c_transfer(req, txData, txSize, NULL, 0); // Create I2C write request
}

static int i2c_read(mxc_i2c_req_t *req, uint8_t *txData, uint8_t *rxData, int rxSize)
{
    return i2c_transfer(req, txData, 2, rxData, rxSize); // Create I2C read request
}

int Eeprom_24LC256_Init(eeprom_24lc256_req_t *req, mxc_i2c_regs_t *i2c, uint8_t addr,
                        unsigned int i2c_freq)
{
    int err = E_NO_ERROR;
    int return_val = 0;

    err = MXC_I2C_Init(i2c, 1, 0);
    if (err != E_NO_ERROR) {
        return err;
    }

    return_val = MXC_I2C_SetFrequency(i2c, i2c_freq);
    if (return_val <= 0) {
        err = return_val;
        return err;
    }

    req->i2c_req.i2c = i2c;
    req->i2c_req.addr = addr;
    req->i2c_req.tx_buf = NULL;
    req->i2c_req.tx_len = 0;
    req->i2c_req.rx_buf = NULL;
    req->i2c_req.rx_len = 0;
    req->i2c_req.restart = 0;
    req->i2c_req.callback = NULL;

    return E_NO_ERROR;
}

int Eeprom_24LC256_Read(eeprom_24lc256_req_t *req, uint16_t addr, uint8_t *data_buffer,
                        uint16_t length)
{
    int err = E_NO_ERROR;
    uint16_t remaining = length;
    uint8_t *current_data_buffer = data_buffer;
    uint16_t eeprom_addr_to_read = addr;
    uint8_t send_buffer[2] = { 0x00, 0x00 };

    if (addr + length > _24LC256_EEPROM_SIZE) {
        return E_BAD_PARAM;
    }

    while (remaining) {
        if (remaining <= I2C_MAX_READ_SIZE) {
            send_buffer[0] = eeprom_addr_to_read >> 8;
            send_buffer[1] = (eeprom_addr_to_read & 0x00FF);

            // Read
            err = i2c_read(&req->i2c_req, send_buffer, current_data_buffer, remaining);
            if (err != E_NO_ERROR) {
                return err;
            } else {
                remaining = 0;
            }
        } else {
            send_buffer[0] = eeprom_addr_to_read >> 8;
            send_buffer[1] = (eeprom_addr_to_read & 0x00FF);

            // Read
            err = i2c_read(&req->i2c_req, send_buffer, current_data_buffer, I2C_MAX_READ_SIZE);
            if (err != E_NO_ERROR) {
                return err;
            }
            remaining -= I2C_MAX_READ_SIZE;
            current_data_buffer += I2C_MAX_READ_SIZE;
            eeprom_addr_to_read += I2C_MAX_READ_SIZE;
        }
    }
    return E_NO_ERROR;
}

int Eeprom_24LC256_Write_Chunk(eeprom_24lc256_req_t *req, uint16_t addr, uint8_t *data_buffer,
                               uint16_t length)
{
    int err = E_NO_ERROR;
    int i = 0;
    uint8_t send_buffer[66]; // Page size (64) + 2 bytes

    uint16_t remaining_size_until_page_end = _24LC256_EEPROM_PAGE_SIZE - (addr & 0x3F);
    if (length > remaining_size_until_page_end) {
        return E_BAD_PARAM;
    }

    send_buffer[0] = addr >> 8;
    send_buffer[1] = (addr & 0x00FF);
    for (i = 0; i < length; i++) {
        send_buffer[i + 2] = data_buffer[i];
    }

    // Write
    err = i2c_write(&req->i2c_req, send_buffer, (length + 2));
    if (err != E_NO_ERROR) {
        return err;
    }
    return E_NO_ERROR;
}

int Eeprom_24LC256_Write(eeprom_24lc256_req_t *req, uint16_t addr, uint8_t *data_buffer,
                         uint32_t length)
{
    int err = E_NO_ERROR;
    uint16_t remaining_data_length_to_write = length;
    uint16_t remaining_size_until_page_end = 0;
    uint8_t *current_data_buffer = data_buffer;
    uint16_t eeprom_addr_to_write = addr;
    if (addr + length > _24LC256_EEPROM_SIZE) {
        return E_BAD_PARAM;
    }

    // EEPROM does not support writing to multiple pages simultaneously
    // In fist operation, if we are not writing to start of a page, we should write until the end of that page.
    if ((addr % _24LC256_EEPROM_PAGE_SIZE) != 0) // If not start of a page)
    {
        remaining_size_until_page_end = _24LC256_EEPROM_PAGE_SIZE - (eeprom_addr_to_write & 0x3F);
        if (remaining_data_length_to_write <=
            remaining_size_until_page_end) // if remaining data size is smaller than remaining page size
        {
            err = Eeprom_24LC256_Write_Chunk(req, eeprom_addr_to_write, current_data_buffer,
                                             remaining_data_length_to_write);
            if (err != E_NO_ERROR) {
                return err;
            } else {
                remaining_data_length_to_write = 0;
            }
        } else {
            err = Eeprom_24LC256_Write_Chunk(
                req, eeprom_addr_to_write, current_data_buffer,
                remaining_size_until_page_end); // Write until the end of the page
            if (err != E_NO_ERROR) {
                return err;
            } else {
                remaining_data_length_to_write -= remaining_size_until_page_end;
                current_data_buffer += remaining_size_until_page_end;
                eeprom_addr_to_write += remaining_size_until_page_end;
                MXC_Delay(MXC_DELAY_MSEC(10)); // Wait for 10ms
            }
        }
    }

    while (remaining_data_length_to_write) {
        if (remaining_data_length_to_write <= _24LC256_EEPROM_PAGE_SIZE) {
            err = Eeprom_24LC256_Write_Chunk(req, eeprom_addr_to_write, current_data_buffer,
                                             remaining_data_length_to_write);
            if (err != E_NO_ERROR) {
                return err;
            }
            return E_NO_ERROR;
        } else {
            err = Eeprom_24LC256_Write_Chunk(req, eeprom_addr_to_write, current_data_buffer,
                                             _24LC256_EEPROM_PAGE_SIZE);
            if (err != E_NO_ERROR) {
                return err;
            } else {
                remaining_data_length_to_write -= _24LC256_EEPROM_PAGE_SIZE;
                current_data_buffer += _24LC256_EEPROM_PAGE_SIZE;
                eeprom_addr_to_write += _24LC256_EEPROM_PAGE_SIZE;
                MXC_Delay(MXC_DELAY_MSEC(10)); // Wait for 10ms
            }
        }
    }
    return E_NO_ERROR;
}
