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

#include "include/eeprom.h"

#include <string.h>
#include "cache.h"
#include "mxc_delay.h"
#include "mxc_errors.h"
#include "nvic_table.h"

/***** Type Definitions *****/
typedef struct {
    mxc_i2c_regs_t *i2c; //Pointer to I2C instance used in EEPROM operations
    uint16_t write_addr; //Address to start storing data at
    uint16_t read_addr; //Address to read data from
    bool write_op; //Read/write operation state variable
    int num_rx; //Number of characters in rx_buf
    uint8_t rx_buf[EEPROM_RX_BUF_SIZE]; //Buffer to store received characters
    bool overflow; //RX Buffer overflowed during transaction
    mxc_gpio_cfg_t rdy_pin; //Ready status of the EEPROM (HIGH = Ready for TXN, LOW = Not ready)
} eeprom_t;

/***** Global Variables *****/
volatile bool eeprom_txn_done = false;
static eeprom_t eeprom;
static cache_t cache;

/***** Functions *****/
int eeprom_handler(mxc_i2c_regs_t *i2c, mxc_i2c_slave_event_t evt, void *retVal);
void eeprom_send_data(void);
void eeprom_receive_data(void);
void eeprom_store_data(void);
void eeprom_cleanup(int err);

static void I2C_Handler(void)
{
    MXC_I2C_AsyncHandler(eeprom.i2c);
}

int eeprom_init(mxc_i2c_regs_t *eeprom_i2c, mxc_gpio_cfg_t rdy_pin)
{
    int err;

    // Check for bad params
    if (eeprom_i2c == NULL) {
        return E_NULL_PTR;
    }
    eeprom.i2c = eeprom_i2c;

    // Initialize I2C Slave
    err = MXC_I2C_Init(eeprom.i2c, 0, EEPROM_ADDR);
    if (err != E_NO_ERROR) {
        printf("Failed to initialize I2C slave.\n");
        return err;
    }
    MXC_I2C_SetFrequency(eeprom.i2c, EEPROM_I2C_FREQ);

    // Initialize cache
    err = cache_init(&cache, EEPROM_BASE_ADDR);
    if (err != E_NO_ERROR) {
        printf("Failed to initialize cache.\n");
        return err;
    }

    // Initialize pin used for ready signal
    eeprom.rdy_pin = rdy_pin;
    eeprom.rdy_pin.func = MXC_GPIO_FUNC_OUT;
    eeprom.rdy_pin.pad = MXC_GPIO_PAD_NONE;
    eeprom.rdy_pin.vssel = MXC_GPIO_VSSEL_VDDIOH;

    err = MXC_GPIO_Config(&eeprom.rdy_pin);
    if (err != E_NO_ERROR) {
        printf("Failed to initialize ready signal.");
        return err;
    }

    MXC_GPIO_OutClr(eeprom.rdy_pin.port, eeprom.rdy_pin.mask);

    // Enable I2C Interrupt
    MXC_NVIC_SetVector(EEPROM_I2C_IRQN(eeprom.i2c), I2C_Handler);
    NVIC_EnableIRQ(EEPROM_I2C_IRQN(eeprom.i2c));

    return E_NO_ERROR;
}

void eeprom_prep_for_txn(void)
{
    // De-assert transaction status variable
    eeprom_txn_done = false;

    // Prep I2C for transaction with master
    MXC_I2C_SlaveTransactionAsync(eeprom.i2c, eeprom_handler);

    // Assert the ready signal
    MXC_GPIO_OutSet(eeprom.rdy_pin.port, eeprom.rdy_pin.mask);
}

int eeprom_handler(mxc_i2c_regs_t *i2c, mxc_i2c_slave_event_t evt, void *retVal)
{
    int err = *((int *)retVal);

    // Check for valid params
    if (i2c != eeprom.i2c) {
        return E_INVALID;
    }

    switch (evt) {
    case MXC_I2C_EVT_MASTER_WR: //Prepare to receive data from the master
        // Transaction started --> de-assert ready signal
        MXC_GPIO_OutClr(eeprom.rdy_pin.port, eeprom.rdy_pin.mask);

        // Reset EEPROM state variables
        eeprom.write_op = true;
        eeprom.overflow = false;
        eeprom.num_rx = 0;
        break;

    case MXC_I2C_EVT_MASTER_RD:
        // Transaction (re)started --> de-assert ready signal
        MXC_GPIO_OutClr(eeprom.rdy_pin.port, eeprom.rdy_pin.mask);

        // Check if a read address was received
        if (eeprom.write_op == true) {
            // Read all bytes from RXFIFO
            if (MXC_I2C_GetRXFIFOAvailable(eeprom.i2c) != 0) {
                eeprom_receive_data();
            }

            // Make sure we received a valid number of bytes to reset the Read Addr
            if (eeprom.num_rx >= EEPROM_ADDR_SIZE) {
                eeprom.read_addr = eeprom.rx_buf[0] << 8 | eeprom.rx_buf[1];
            }
        }

        eeprom.write_op = false;
        break;

    case MXC_I2C_EVT_UNDERFLOW:
    case MXC_I2C_EVT_TX_THRESH:
        //(Re)fill I2C TX FIFO
        eeprom_send_data();
        break;

    case MXC_I2C_EVT_OVERFLOW:
    case MXC_I2C_EVT_RX_THRESH:
        //Read data from I2C RX FIFO
        eeprom_receive_data();
        break;

    case MXC_I2C_EVT_TRANS_COMP:
        //TXN complete --> Reset EEPROM state and process data as necessary
        eeprom_txn_done = true;
        eeprom_cleanup(err);
        break;

    default:
        return E_BAD_PARAM;
    }

    return E_NO_ERROR;
}

void eeprom_send_data(void)
{
    int tx_avail;
    int num_written;

    // Check whether the read address is located in the current cache page
    if (EEPROM_RAW_ADDR(eeprom.read_addr) < cache.start_addr ||
        EEPROM_RAW_ADDR(eeprom.read_addr) > cache.end_addr) {
        cache_refresh(&cache, EEPROM_RAW_ADDR(eeprom.read_addr));
    }

    // Get the number of bytes available in the I2C TX FIFO
    tx_avail = MXC_I2C_GetTXFIFOAvailable(eeprom.i2c);

    // Check whether there are enough bytes in the cache to fill the FIFO
    if (EEPROM_RAW_ADDR(eeprom.read_addr) + tx_avail >= cache.end_addr) {
        // Not enough bytes in cache, write remaining bytes in cache to FIFO
        num_written = MXC_I2C_WriteTXFIFO(eeprom.i2c, &cache.cache[CACHE_IDX(eeprom.read_addr)],
                                          cache.end_addr - EEPROM_RAW_ADDR(eeprom.read_addr));

        // Update read address and available bytes in I2C TX FIFO
        tx_avail -= num_written;
        eeprom.read_addr += num_written;

        // Reset read_addr if it has reached the end of EEPROM address space
        if (eeprom.read_addr == EEPROM_FLASH_SZ) {
            eeprom.read_addr = 0;
        }

        // Update the cache with new flash page
        cache_refresh(&cache, EEPROM_RAW_ADDR(eeprom.read_addr));
    }

    // Write remaining bytes to I2C TX FIFO
    eeprom.read_addr +=
        MXC_I2C_WriteTXFIFO(eeprom.i2c, &cache.cache[CACHE_IDX(eeprom.read_addr)], tx_avail);
}

void eeprom_receive_data(void)
{
    int rx_avail;
    rx_avail = MXC_I2C_GetRXFIFOAvailable(eeprom.i2c);

    // Check whether receive buffer will overflow
    if ((rx_avail + eeprom.num_rx) > EEPROM_RX_BUF_SIZE) {
        eeprom.overflow = true;
        rx_avail -= MXC_I2C_ReadRXFIFO(eeprom.i2c, &eeprom.rx_buf[eeprom.num_rx],
                                       EEPROM_RX_BUF_SIZE - eeprom.num_rx);
        eeprom.num_rx = EEPROM_ADDR_SIZE;
    }

    // Read remaining characters in FIFO
    eeprom.num_rx += MXC_I2C_ReadRXFIFO(eeprom.i2c, &eeprom.rx_buf[eeprom.num_rx], rx_avail);
}

void eeprom_store_data(void)
{
    uint32_t write_addr;
    uint32_t rx_idx = 2;

    // Check to see if there are enough bytes to process received data
    if (eeprom.num_rx < EEPROM_ADDR_SIZE) {
        return;
    }

    // Get number of data bytes received
    if (eeprom.overflow) {
        eeprom.num_rx = EEPROM_MAX_DATA_RX;
    } else {
        eeprom.num_rx -= EEPROM_ADDR_SIZE;
    }

    // Get write address
    write_addr = (eeprom.rx_buf[0] << 8) | eeprom.rx_buf[1];

    // Check write address
    if (write_addr == EEPROM_WRITE_BACK_CMD) {
        // Received write back command --> store cache contents
        cache_write_back(&cache);
        return;
    } else if (write_addr >= EEPROM_FLASH_SZ) {
        // Invalid write address
        return;
    }

    // Refresh cache if write address isn't located in the current cache page
    if (EEPROM_RAW_ADDR(write_addr) < cache.start_addr ||
        EEPROM_RAW_ADDR(write_addr) > cache.end_addr) {
        cache_refresh(&cache, EEPROM_RAW_ADDR(write_addr));
    }

    //
    if (eeprom.num_rx + EEPROM_RAW_ADDR(write_addr) >= cache.end_addr) {
        memcpy(&cache.cache[CACHE_IDX(write_addr)], &eeprom.rx_buf[rx_idx],
               cache.end_addr - EEPROM_RAW_ADDR(write_addr));
        cache.dirty = true;

        // Update index variables
        eeprom.num_rx -= cache.end_addr - EEPROM_RAW_ADDR(write_addr);
        rx_idx += cache.end_addr - EEPROM_RAW_ADDR(write_addr);
        write_addr += cache.end_addr - EEPROM_RAW_ADDR(write_addr);

        // Reset write address if it has reached the end of EEPROM address space
        if (write_addr == EEPROM_FLASH_SZ) {
            write_addr = 0;
        }

        // Refresh cache
        cache_refresh(&cache, EEPROM_RAW_ADDR(write_addr));
    }

    // Write remaining bytes to cache
    memcpy(&cache.cache[CACHE_IDX(write_addr)], &eeprom.rx_buf[rx_idx], eeprom.num_rx);
    cache.dirty = true;
}

void eeprom_cleanup(int err)
{
    int fifo_avail;

    if (err == E_NO_ERROR) {
        if (eeprom.write_op) {
            // Write Operation
            // Read remaining characters if any remain in FIFO
            fifo_avail = MXC_I2C_GetRXFIFOAvailable(eeprom.i2c);
            if (fifo_avail) {
                eeprom_receive_data();
            }

            // Store received data in flash
            eeprom_store_data();
        } else {
            // Read Operation
            // Decrement read counter if there are unsent data bytes
            eeprom.read_addr -= EEPROM_FIFO_DEPTH - MXC_I2C_GetTXFIFOAvailable(eeprom.i2c);

            while (eeprom.i2c->status & MXC_F_I2C_STATUS_BUSY) {}
        }
    }

    // Reset EEPROM State
    memset(eeprom.rx_buf, 0x00, sizeof(eeprom.rx_buf));
    eeprom.write_addr = 0;
    eeprom.write_op = false;
    eeprom.num_rx = 0;
    eeprom.overflow = false;
}
