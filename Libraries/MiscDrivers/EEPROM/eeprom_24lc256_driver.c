/******************************************************************************
 * Copyright (C) 2023 Maxim Integrated Products, Inc., All Rights Reserved.
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
 *
 ******************************************************************************/

#include "eeprom_24lc256_driver.h"

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

static int i2c_write(mxc_i2c_req_t *req, uint8_t *txData, int txSize)
{
    return i2c_transfer(req, txData, txSize, NULL, 0); // Create I2C write request
}

static int i2c_read(mxc_i2c_req_t *req, uint8_t *txData, uint8_t *rxData, int rxSize)
{
    return i2c_transfer(req, txData, 2, rxData, rxSize); // Create I2C read request
}

/**
 * @brief   Initializes I2C registers EEPROM
 * @param   i2c		I2C registers
 * @param   addr	Slave I2C address of EEPROM.
 * @returns #E_NO_ERROR if init succeeded.
 *
 */
static int eeprom_24LC256_init(mxc_i2c_regs_t *i2c, uint8_t addr)
{
    req.i2c = i2c;
    req.addr = addr;
    req.tx_buf = NULL;
    req.tx_len = 0;
    req.rx_buf = NULL;
    req.rx_len = 0;
    req.restart = 0;
    req.callback = NULL;

    return E_NO_ERROR;
}

/**
 * @brief   Writes data from EEPROM
 * @param   addr		Start address we want to read.
 * @param   data_buffer	Data buffer to read.
 * @param   length		Number of bytes to read.
 * @returns #E_NO_ERROR if read succeeded. non-zero if an error occurred.
 *
 */
static int eeprom_24LC256_read(uint16_t addr, uint8_t* data_buffer, uint16_t length)
{
    int err = E_NO_ERROR;
    uint16_t remaining = length;
    uint8_t* current_data_buffer = data_buffer;
    uint16_t eeprom_addr_to_read = addr;
    uint8_t send_buffer[2] = { 0x00, 0x00};

    if(addr + length > _24LC256_EEPROM_SIZE)
    {
    	return E_BAD_PARAM;
    }

    while(remaining)
    {
		if(remaining <= I2C_MAX_READ_SIZE)
		{
		    send_buffer[0] = eeprom_addr_to_read >> 8;
		    send_buffer[1] = (eeprom_addr_to_read & 0x00FF);

			// Read
			err = i2c_read(&req, send_buffer, current_data_buffer, remaining);
			if (err != E_NO_ERROR) {
				return err;
			}else{
				remaining = 0;
			}
		}else
		{
		    send_buffer[0] = eeprom_addr_to_read >> 8;
		    send_buffer[1] = (eeprom_addr_to_read & 0x00FF);

			// Read
			err = i2c_read(&req, send_buffer, current_data_buffer, I2C_MAX_READ_SIZE);
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

/**
 * @brief   Writes a small chunk of data directly to the EEPROM. The written memory should be in the same page of EEPROM (1 page = 64 bytes)
 * @param   addr		Address we want to write to.
 * @param   data_buffer	Data buffer to write.
 * @param   length		Number of bytes to write.
 * @returns #E_NO_ERROR if write succeeded. non-zero if an error occurred.
 *
 */
static int eeprom_24LC256_write_chunk(uint16_t addr, uint8_t* data_buffer, uint16_t length)
{
    int err = E_NO_ERROR;
    int i = 0;
    uint8_t send_buffer[66];			// Max write length is equal to page size (64 bytes). So, max buffer size 64 + 2 bytes
    send_buffer[0] = addr >> 8;
    send_buffer[1] = (addr & 0x00FF);
    for(i = 0; i < length; i++)
    {
    	send_buffer[i+2] = data_buffer[i];
    }

	// Write
	err = i2c_write(&req, send_buffer, (length + 2));
	if (err != E_NO_ERROR) {
		return err;
	}
	return E_NO_ERROR;
}

/**
 * @brief   Writes data to the EEPROM
 * @param   addr		Address we want to write to.
 * @param   data_buffer	Data buffer to write.
 * @param   length		Number of bytes to write.
 * @returns #E_NO_ERROR if write succeeded. non-zero if an error occurred.
 *
 */
static int eeprom_24LC256_write(uint16_t addr, uint8_t* data_buffer, uint32_t length)
{
    int err = E_NO_ERROR;
    uint16_t remaining_data_length_to_write = length;
    uint16_t remaining_size_until_page_end = 0;
    uint8_t* current_data_buffer = data_buffer;
    uint16_t eeprom_addr_to_write = addr;
    if(addr + length > _24LC256_EEPROM_SIZE)
    {
    	return E_BAD_PARAM;
    }

    // EEPROM does not support writing to multiple pages simultaneously
    // In fist operation, if we are not writing to start of a page, we should write until the end of that page.
    if((addr % _24LC256_EEPROM_PAGE_SIZE) != 0)	// If starting address is not multiple of 64 bytes (not start of a page)
    {
    	remaining_size_until_page_end = _24LC256_EEPROM_PAGE_SIZE - (eeprom_addr_to_write & 0x3F);
        if(remaining_data_length_to_write <= remaining_size_until_page_end)		// if remaining data size is smaller than remaining page size
        {
			err = eeprom_24LC256_write_chunk(eeprom_addr_to_write, current_data_buffer, remaining_data_length_to_write);
			if (err != E_NO_ERROR) {
				return err;
			}else{
				remaining_data_length_to_write = 0;
			}
        }else
        {
			err = eeprom_24LC256_write_chunk(eeprom_addr_to_write, current_data_buffer, remaining_size_until_page_end);			// Write until the end of the page
			if (err != E_NO_ERROR) {
				return err;
			}else{
				remaining_data_length_to_write -= remaining_size_until_page_end;
				current_data_buffer += remaining_size_until_page_end;
				eeprom_addr_to_write += remaining_size_until_page_end;
				MXC_Delay(MXC_DELAY_MSEC(10));		// Wait for 10ms
			}
        }
    }

    while(remaining_data_length_to_write)
    {
		if(remaining_data_length_to_write <= _24LC256_EEPROM_PAGE_SIZE)
		{
			err = eeprom_24LC256_write_chunk(eeprom_addr_to_write, current_data_buffer, remaining_data_length_to_write);
			if (err != E_NO_ERROR) {
				return err;
			}
			return E_NO_ERROR;
		}
		else
		{
			err = eeprom_24LC256_write_chunk(eeprom_addr_to_write, current_data_buffer, _24LC256_EEPROM_PAGE_SIZE);
			if (err != E_NO_ERROR) {
				return err;
			}else{
				remaining_data_length_to_write -= _24LC256_EEPROM_PAGE_SIZE;
				current_data_buffer += _24LC256_EEPROM_PAGE_SIZE;
				eeprom_addr_to_write += _24LC256_EEPROM_PAGE_SIZE;
				MXC_Delay(MXC_DELAY_MSEC(10));		// Wait for 10ms
			}
		}
    }
	return E_NO_ERROR;
}

eeprom_24LC256_driver_t eeprom_24LC256_Open(void)
{
	eeprom_24LC256_driver_t SD;
    SD.init = eeprom_24LC256_init;
    SD.read = eeprom_24LC256_read;
    SD.write_chunk = eeprom_24LC256_write_chunk;
    SD.write = eeprom_24LC256_write;

    return SD;
}
