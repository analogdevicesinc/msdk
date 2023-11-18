/**
 * @file    eeprom_24lc256_driver.h
 * @brief   24LC256 EEPROM driver header
 * @details Defines 24LC256 EEPROM
 *          Implements helper macros
 **/

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

#ifndef LIBRARIES_MISCDRIVERS_EEPROM_EEPROM_24LC256_DRIVER_H_
#define LIBRARIES_MISCDRIVERS_EEPROM_EEPROM_24LC256_DRIVER_H_

/***** Includes *****/
#include <stdint.h>
#include <stdio.h>
#include <float.h>
#include "mxc_delay.h"
#include "i2c.h"

/***** Definitions *****/
/**@def _24LC256_EEPROM_PAGE_SIZE
 * @brief Page size of  24LC256 EEPROM
 **/
#define _24LC256_EEPROM_PAGE_SIZE 64

/**@def _24LC256_EEPROM_SIZE
 * @brief Total size of  24LC256 EEPROM = 256 Kbit
 **/
#define _24LC256_EEPROM_SIZE 0x8000 // 32 KByte = 256 Kbit

/**@def I2C
 * @brief I2C Max read size for single transaction
 **/
#define I2C_MAX_READ_SIZE 256

typedef struct _eeprom_24lc256_req_t eeprom_24lc256_req_t;

/**
 * @brief   The information required to communicate with eeprom over the I2C channel.
 *
 * This structure is used to communicate eeprom over I2C channel.
 */
struct _eeprom_24lc256_req_t {
    mxc_i2c_req_t i2c_req; ///< I2C request
};

/***** Function Prototypes *****/

/**
 * @brief   Initializes I2C registers EEPROM
 * @param   req			Pointer to details of communication with eeprom
 * @param   i2c			I2C registers
 * @param   addr		Slave I2C address of EEPROM.
 * @param   i2c_freq	The desired i2c frequency in Hertz
 * @return  #E_NO_ERROR if read succeeded, see \ref MXC_Error_Codes for a list of return codes.
 *
 */
int Eeprom_24LC256_Init(eeprom_24lc256_req_t *req, mxc_i2c_regs_t *i2c, uint8_t addr,
                        unsigned int i2c_freq);

/**
 * @brief   Reads data from EEPROM
 * @param   req			Pointer to details of communication with eeprom
 * @param   addr		Start address we want to read.
 * @param   data_buffer	Data buffer to read.
 * @param   length		Number of bytes to read.
 * @return  #E_NO_ERROR if read succeeded. see \ref MXC_Error_Codes for a list of return codes.
 *
 */
int Eeprom_24LC256_Read(eeprom_24lc256_req_t *req, uint16_t addr, uint8_t *data_buffer,
                        uint16_t length);

/**
 * @brief   Writes a small chunk of data directly to the EEPROM. The written memory should be in the same page of EEPROM (1 page = 64 bytes)
 * @param   req			Pointer to details of communication with eeprom
 * @param   addr		Address we want to write to.
 * @param   data_buffer	Data buffer to write.
 * @param   length		Number of bytes to write.
 * @returns #E_NO_ERROR if write succeeded. see \ref MXC_Error_Codes for a list of return codes.
 *
 */
int Eeprom_24LC256_Write_Chunk(eeprom_24lc256_req_t *req, uint16_t addr, uint8_t *data_buffer,
                               uint16_t length);

/**
 * @brief   Writes data to the EEPROM
 * @param   req			Pointer to details of communication with eeprom
 * @param   addr		Address we want to write to.
 * @param   data_buffer	Data buffer to write.
 * @param   length		Number of bytes to write.
 * @returns #E_NO_ERROR if write succeeded. see \ref MXC_Error_Codes for a list of return codes.
 *
 */
int Eeprom_24LC256_Write(eeprom_24lc256_req_t *req, uint16_t addr, uint8_t *data_buffer,
                         uint32_t length);

#endif // LIBRARIES_MISCDRIVERS_EEPROM_EEPROM_24LC256_DRIVER_H_
