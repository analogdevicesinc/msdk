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

#include "max31889_driver.h"

/******************************* MAX31889 Register addresses *******************************/
#define MAX31889_STATUS_REG 0x00 //Status register
#define MAX31889_INTERRUPT_REG 0x01 //Interrupt register
#define MAX31889_FIFO_WR_PTR_REG 0x04 //FIFO write pointer register
#define MAX31889_FIFO_RD_PTR_REG 0x05 //FIFO read pointer register
#define MAX31889_FIFO_OF_CTR_REG 0x06 //FIFO overflow counter register
#define MAX31889_FIFO_DATA_CTR_REG 0x07 //FIFO data counter register
#define MAX31889_FIFO_DATA_REG 0x08 //FIFO data register
#define MAX31889_FIFO_CONF1_REG 0x09 //FIFO configuration register 1
#define MAX31889_FIFO_CONF2_REG 0x0A //FIFO configuration register 2
#define MAX31889_SYS_CTRL_REG 0x0C //System control register
#define MAX31889_ALARM_HI_MSB_REG 0x10 //Alarm temperature high threshold MSB register
#define MAX31889_ALARM_HI_LSB_REG 0x11 //Alarm temperature high threshold LSB register
#define MAX31889_ALARM_LO_MSB_REG 0x12 //Alarm temperature low threshold MSB register
#define MAX31889_ALARM_LO_LSB_REG 0x13 //Alarm temperature low threshold LSB register
#define MAX31889_SENSOR_SETUP_REG 0x14 //Temperature sensor setup register
#define MAX31889_GPIO_SETUP_REG 0x20 //GPIO setup register
#define MAX31889_GPIO_CTRL_REG 0x21 //GPIO control register
#define MAX31889_ROM_ID1_REG 0x31 //Factory ROM1 register
#define MAX31889_ROM_ID2_REG 0x32 //Factory ROM2 register
#define MAX31889_ROM_ID3_REG 0x33 //Factory ROM3 register
#define MAX31889_ROM_ID4_REG 0x34 //Factory ROM4 register
#define MAX31889_ROM_ID5_REG 0x35 //Factory ROM5 register
#define MAX31889_ROM_ID6_REG 0x36 //Factory ROM6 register
#define MAX31889_PART_ID_REG 0xFF //Part identifier register

/******************************* Bit definitions *******************************/
/* System Control Bits */
#define MAX31889_RESET_BIT 0

/* Interrrupt Registers*/
#define MAX31889_TEMP_RDY_BIT 0
#define MAX31889_TEMP_HI_BIT 1
#define MAX31889_TEMP_LO_BIT 2
#define MAX31889_A_FULL_BIT 7

/* Sensor Setup Register */
#define MAX31889_CONV_START_BIT 0

/* FIFO Bit Mask */
#define MAX31889_FIFO_MASK 0x1F

/* FIFO Configuartion Register 2 */
#define MAX31889_FIFO_RO_BIT 1
#define MAX31889_A_FULL_TYPE_BIT 2
#define MAX31889_FIFO_STAT_CLR_BIT 3
#define MAX31889_FLUSH_FIFO_BIT 4

/* GPIO Setup Register */
#define MAX31889_GPIO0_MODE0_BIT 0
#define MAX31889_GPIO0_MODE1_BIT 1
#define MAX31889_GPIO1_MODE0_BIT 6
#define MAX31889_GPIO1_MODE1_BIT 7

/* GPIO Control Register */
#define MAX31889_GPIO_LL_BIT 0
#define MAX31889_GPI1_LL_BIT 3

/******************************* Constants *******************************/
#define MAX31889_PART_ID_VALUE 0x30

/******************************* Useful Definitions *******************************/
#define SET_BIT(cmd, bit) cmd |= (1 << bit)
#define GET_BIT(cmd, bit) cmd &(1 << bit)
#define CLEAR_BIT(cmd, bit) cmd &= ~(1 << bit)

/******************************* Type Definitions *******************************/
typedef enum {
    MAX31889_GPIO_Mode_HiZ = 0, ///< Digital input (HiZ)
    MAX31889_GPIO_Mode_OD = 1, ///< Digital output (open-drain)
    MAX31889_GPIO_Mode_IPD = 2, ///< Digital Input with 1MOhm pulldown.
    MAX31889_GPIO_Mode_INT_Conv =
        3 ///< INTB (GPIO0) open-drain output or Convert Temperature Input (GPIO1)
} MAX31889_GPIO_Mode_t;

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
    return i2c_transfer(req, txData, 1, rxData, rxSize); // Create I2C read request
}

static uint8_t max31889_set_gpio_mode(MAX31889_GPIO_Mode_t gpio0_mode,
                                      MAX31889_GPIO_Mode_t gpio1_mode)
{
    uint8_t gpio_setup_reg = 0;
    if (gpio0_mode & 0x01) {
        SET_BIT(gpio_setup_reg, MAX31889_GPIO0_MODE0_BIT);
    } else {
        CLEAR_BIT(gpio_setup_reg, MAX31889_GPIO0_MODE0_BIT);
    }

    if (gpio0_mode & 0x02) {
        SET_BIT(gpio_setup_reg, MAX31889_GPIO0_MODE1_BIT);
    } else {
        CLEAR_BIT(gpio_setup_reg, MAX31889_GPIO0_MODE1_BIT);
    }

    if (gpio1_mode & 0x01) {
        SET_BIT(gpio_setup_reg, MAX31889_GPIO1_MODE0_BIT);
    } else {
        CLEAR_BIT(gpio_setup_reg, MAX31889_GPIO1_MODE0_BIT);
    }

    if (gpio1_mode & 0x02) {
        SET_BIT(gpio_setup_reg, MAX31889_GPIO1_MODE1_BIT);
    } else {
        CLEAR_BIT(gpio_setup_reg, MAX31889_GPIO1_MODE1_BIT);
    }

    return gpio_setup_reg;
}

static float max31889_convert_to_temp(uint8_t *reading)
{
    return 0.005f * (reading[1] + ((uint16_t)reading[0] << 8));
}

static int max31889_conversion(void)
{
    int ret_code = E_NO_ERROR;
    uint8_t setup_reg = 0xC0; // Bits 6 and 7 are reserved, should always be written as 1

    // Set conversion start bit
    SET_BIT(setup_reg, MAX31889_CONV_START_BIT);

    // Set MAX31889 sensor setup register
    uint8_t reg[] = { MAX31889_SENSOR_SETUP_REG, setup_reg };
    ret_code = i2c_write(&req, reg, 2);
    if (ret_code != E_NO_ERROR) {
        return ret_code;
    }

    //wait for conversion to finish
    uint32_t timeout = 1000; //maximum conversion time from datasheet
    do {
        ret_code = i2c_read(&req, &reg[0], &reg[1], 1);
        if (ret_code == E_NO_ERROR) {
            //Check if conversion bit is cleared by sensor
            if ((GET_BIT(reg[1], MAX31889_CONV_START_BIT)) == 0) {
                return E_NO_ERROR;
            }
        }
        // Wait for 1ms
        MXC_Delay(MXC_DELAY_MSEC(1));
    } while (timeout-- > 0);

    return E_TIME_OUT;
}

static int max318889_read_temperature(float *temp)
{
    int ret_code = E_NO_ERROR;
    uint8_t reg[2];
    uint8_t tr[2];
    uint8_t status, ov_cnt;

    // Read status
    reg[0] = MAX31889_STATUS_REG;

    ret_code = i2c_read(&req, &reg[0], &status, 1);
    if (ret_code != E_NO_ERROR) {
        return ret_code;
    }

    // Wait
    MXC_Delay(100);

    // Check FIFO status
    if (GET_BIT(status, MAX31889_A_FULL_BIT)) {
        // FIFO overflow occurred. Read the lost readings count
        reg[0] = MAX31889_FIFO_OF_CTR_REG;
        ret_code = i2c_read(&req, &reg[0], &ov_cnt, 1);
        if (ret_code != E_NO_ERROR) {
            return ret_code;
        }

        // Clear unused bits
        ov_cnt &= MAX31889_FIFO_MASK;
        printf("\nFIFO overflowed by %i words", ov_cnt);

        //Flush FIFO
        reg[0] = MAX31889_FIFO_CONF2_REG;
        SET_BIT(reg[1], MAX31889_FLUSH_FIFO_BIT);
        ret_code = i2c_write(&req, reg, 2);
        if (ret_code != E_NO_ERROR) {
            return ret_code;
        }
        printf("\nFIFO flushed");
    }

    // Wait
    MXC_Delay(100);

    // Check whether conversion is finished
    if (GET_BIT(status, MAX31889_TEMP_RDY_BIT)) {
        // Temperature reading is 16 bit (2-byte) value
        reg[0] = MAX31889_FIFO_DATA_REG;
        ret_code = i2c_read(&req, &reg[0], tr, 2);
        if (ret_code != E_NO_ERROR) {
            return ret_code;
        }

        //Convert the reading to temperature
        *temp = max31889_convert_to_temp(tr);

    } else {
        return E_BUSY;
    }

    return ret_code;
}

static int max31889_init(mxc_i2c_regs_t *i2c, uint8_t addr)
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
    uint8_t reg = MAX31889_PART_ID_REG;
    uint8_t id = 0;
    err = i2c_read(&req, &reg, &id, 1);
    if (err != E_NO_ERROR) {
        return err;
    }

    // Verify sensor ID
    if (id != MAX31889_PART_ID_VALUE) {
        printf("\nWrong sensor ID");
        return E_NO_DEVICE;
    }

    // Configure MAX31889 GPIO0 and GPIO1 as inputs
    uint8_t gpio_reg = max31889_set_gpio_mode(MAX31889_GPIO_Mode_IPD, MAX31889_GPIO_Mode_IPD);

    // Wait
    MXC_Delay(100);

    // Set MAX31889 gpio_register
    uint8_t gpio_setup_reg[] = { MAX31889_GPIO_SETUP_REG, gpio_reg };
    err = i2c_write(&req, gpio_setup_reg, 2);
    if (err != E_NO_ERROR) {
        printf("I2C sensor configure failed\n");
        return err;
    }

    return E_NO_ERROR;
}

static int max31889_read(void *buff)
{
    int err = E_NO_ERROR;

    err = max31889_conversion();
    if (err != E_NO_ERROR) {
        printf("\nSensor write error: %i", err);
        return err;
    }

    // Read conversion result
    return max318889_read_temperature((float *)buff);
}

max31889_driver_t MAX31889_Open(void)
{
    max31889_driver_t SD;
    SD.init = max31889_init;
    SD.read = max31889_read;
    return SD;
}
