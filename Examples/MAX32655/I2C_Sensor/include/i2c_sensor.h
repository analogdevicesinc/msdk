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

#ifndef I2C_SENSOR_H
#define I2C_SENSOR_H

#include <stdint.h>
#include <stdio.h>
#include "i2c_regs.h"
#include "i2c.h"

/******************************* Definitions *******************************/
#define SET_BIT(cmd, bit) cmd |= (1 << bit)
#define GET_BIT(cmd, bit) cmd &(1 << bit)
#define CLEAR_BIT(cmd, bit) cmd &= ~(1 << bit)

/******************************* Type Definitions *******************************/
/**
 * @brief Structure with sensor function pointers
 */
typedef struct {
    int (*init)(mxc_i2c_regs_t *i2c, uint8_t addr);
    int (*read)(void *buf);
} mxc_i2c_sensor_driver_t;

/******************************* Functions *******************************/
/**
 * @brief I2C data transfer (read/write)
 * 
 * @param req pointer to I2C request instance 
 * @param txData pointer to sending buffer
 * @param txSize number of bytes to write
 * @param rxData pointer receiving buffer
 * @param rxSize number of bytes to read
 * @return int transaction result
 */
int i2c_transfer(mxc_i2c_req_t *req, uint8_t *txData, int txSize, uint8_t *rxData, int rxSize);

/**
 * @brief I2C data write
 * 
 * @param req pointer to I2C request instance 
 * @param txData pointer to sending buffer
 * @param txSize number of bytes to write
 * @return int transaction result
 */
int i2c_write(mxc_i2c_req_t *req, uint8_t *txData, int txSize);

/**
 * @brief I2C data read
 * 
 * @param req pointer to I2C request instance 
 * @param txData pointer to one byte
 * @param rxData pointer receiving buffer
 * @param rxSize number of bytes to read
 * @return int transaction result
 */
int i2c_read(mxc_i2c_req_t *req, uint8_t *txData, uint8_t *rxData, int rxSize);

#endif /* I2C_SENSOR_H */