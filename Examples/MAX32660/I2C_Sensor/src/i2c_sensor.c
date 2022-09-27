/*******************************************************************************
* Copyright (C) 2022 Maxim Integrated Products, Inc., All Rights Reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"),
* to deal in the Software without restriction, including without limitation
* the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
* OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*
* Except as contained in this notice, the name of Maxim Integrated
* Products, Inc. shall not be used except as stated in the Maxim Integrated
* Products, Inc. Branding Policy.
*
* The mere transfer of this software does not imply any licenses
* of trade secrets, proprietary technology, copyrights, patents,
* trademarks, maskwork rights, or any other form of intellectual
* property whatsoever. Maxim Integrated Products, Inc. retains all
* ownership rights.
*******************************************************************************
*/

#include "i2c_sensor.h"

int i2c_write(mxc_i2c_req_t *req, uint8_t *txData, int txSize)
{
    return i2c_transfer(req, txData, txSize, NULL, 0); // Create I2C write request
}

int i2c_read(mxc_i2c_req_t *req, uint8_t *txData, uint8_t *rxData, int rxSize)
{
    return i2c_transfer(req, txData, 1, rxData, rxSize); // Create I2C read request
}

int i2c_transfer(mxc_i2c_req_t *req, uint8_t *txData, int txSize, uint8_t *rxData, int rxSize)
{
    req->tx_buf = txData; // Write data buffer
    req->tx_len = txSize; // Number of bytes to write
    req->rx_buf = rxData; // Read data buffer
    req->rx_len = rxSize; // Number of bytes to read
    return MXC_I2C_MasterTransaction(req);
}