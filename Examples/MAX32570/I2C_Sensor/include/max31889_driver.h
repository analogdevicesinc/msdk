/**
 * @file    max31889_driver.h
 * @brief   MAX31889 IC driver header
 * @details Defines MAX31889 registers
 *          Implements helper macros
 **/

/*******************************************************************************
* Copyright (C) 2022 Maxim Integrated Products, Inc., All rights Reserved.
* 
* This software is protected by copyright laws of the United States and
* of foreign countries. This material may also be protected by patent laws
* and technology transfer regulations of the United States and of foreign
* countries. This software is furnished under a license agreement and/or a
* nondisclosure agreement and may only be used or reproduced in accordance
* with the terms of those agreements. Dissemination of this information to
* any party or parties not specified in the license agreement and/or
* nondisclosure agreement is expressly prohibited.
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

#ifndef _MAX31889_DRIVER_
#define _MAX31889_DRIVER_

#include <stdint.h>
#include <stdio.h>
#include <float.h>
#include "mxc_delay.h"
#include "i2c_sensor.h"

/**@def MAX31889_I2C_SLAVE_ADDR0
 * @brief I2C slave address (option 0)
 **/
#define MAX31889_I2C_SLAVE_ADDR0 (0xA0 >> 1)

/**@def MAX31889_I2C_SLAVE_ADDR1
 * @brief I2C slave address (option 1)
 **/
#define MAX31889_I2C_SLAVE_ADDR1 (0xA6 >> 1)

/**
 * @brief Prepare I2C_SensorDriver function pointers
 *
 * @return I2C_SensorDriver instance
 */
mxc_i2c_sensor_driver_t MAX31889_Open();

#endif //_MAX31889_DRIVER_
