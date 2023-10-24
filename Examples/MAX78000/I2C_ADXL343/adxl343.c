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

/*
  adxl343.c

  Analog Devices ADXL343 driver.
*/

#include <stdint.h>
#include <stdlib.h>
#include "mxc.h"
#include "adxl343.h"

// I2C address with pin ALT_ADDRESS/SDO pulled low
#define ADXL343_ADDR 0x53

#define DEVID 0xE5

#define DEVID_REG 0x00 // Device ID
#define THRESH_TAP_REG 0x1D // Tap threshold
#define OFSX_REG 0x1E // X-axis offset
#define OFSY_REG 0x1F // Y-axis offset
#define OFSZ_REG 0x20 // Z-axis offset
#define DUR_REG 0x21 // Tap duration
#define LATENT_REG 0x22 // Tap latency
#define WINDOW_REG 0x23 // Tap window
#define THRESH_ACT_REG 0x24 // Activity threshold
#define THRESH_INACT_REG 0x25 // Inactivity threshold
#define TIME_INACT_REG 0x26 // Inactivity time
#define ACT_INACT_CTL_REG 0x27 // Axis enable control for activity and inactivity detection
#define THRESH_FF_REG 0x28 // Free-fall threshold
#define TIME_FF_REG 0x29 // Free-fall time
#define TAP_AXES_REG 0x2A // Axis control for single tap/double tap
#define ACT_TAP_STATUS_REG 0x2B // Source of single tap/double tap
#define BW_RATE_REG 0x2C // Data rate and power mode control
#define POWER_CTL_REG 0x2D // Power-saving features control
#define INT_ENABLE_REG 0x2E // Interrupt enable control
#define INT_MAP_REG 0x2F // Interrupt mapping control
#define INT_SOURCE_REG 0x30 // Source of interrupts
#define DATA_FORMAT_REG 0x31 // Data format control
#define DATAX0_REG 0x32 // X-Axis Data 0
#define DATAX1_REG 0x33 // X-Axis Data 1
#define DATAY0_REG 0x34 // Y-Axis Data 0
#define DATAY1_REG 0x35 // Y-Axis Data 1
#define DATAZ0_REG 0x36 // Z-Axis Data 0
#define DATAZ1_REG 0x37 // Z-Axis Data 1
#define FIFO_CTL_REG 0x38 // FIFO control
#define FIFO_STATUS_REG 0x39 // FIFO status

#define RATE_MASK 0x0F
#define LP_MASK 0x10
#define RANGE_MASK 0x03
#define MEASURE_MASK 0x08
#define FIFO_MODE_MASK 0xC0

static mxc_i2c_req_t i2c_req;

static inline int reg_write(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };

    i2c_req.tx_buf = buf;
    i2c_req.tx_len = sizeof(buf);
    i2c_req.rx_len = 0;

    return MXC_I2C_MasterTransaction(&i2c_req);
}

static inline int reg_read(uint8_t reg, uint8_t *dat)
{
    uint8_t buf[1] = { reg };

    i2c_req.tx_buf = buf;
    i2c_req.tx_len = sizeof(buf);
    i2c_req.rx_buf = dat;
    i2c_req.rx_len = 1;

    return MXC_I2C_MasterTransaction(&i2c_req);
}

int adxl343_get_axis_data(int16_t *ptr)
{
    uint8_t buf[1] = { DATAX0_REG };

    i2c_req.tx_buf = buf;
    i2c_req.tx_len = sizeof(buf);
    i2c_req.rx_buf = (uint8_t *)ptr;
    i2c_req.rx_len = 6;

    return MXC_I2C_MasterTransaction(&i2c_req);
}

int adxl343_set_power_mode(uint8_t mode)
{
    int result;
    uint8_t reg;

    if ((result = reg_read(BW_RATE_REG, &reg)) != E_NO_ERROR)
        return result;
    reg &= ~LP_MASK;
    reg |= mode;
    return reg_write(BW_RATE_REG, reg);
}

int adxl343_set_data_rate(uint8_t rate)
{
    int result;
    uint8_t reg;

    if ((result = reg_read(BW_RATE_REG, &reg)) != E_NO_ERROR)
        return result;
    reg &= ~RATE_MASK;
    reg |= rate;
    return reg_write(BW_RATE_REG, reg);
}

int adxl343_set_fifo_mode(uint8_t mode)
{
    int result;
    uint8_t reg;

    if ((result = reg_read(FIFO_CTL_REG, &reg)) != E_NO_ERROR)
        return result;
    reg &= ~FIFO_MODE_MASK;
    reg |= mode;
    return reg_write(FIFO_CTL_REG, reg);
}

int adxl343_set_range(uint8_t range)
{
    int result;
    uint8_t reg;

    if ((result = reg_read(DATA_FORMAT_REG, &reg)) != E_NO_ERROR)
        return result;
    reg &= ~RANGE_MASK;
    reg |= range;
    return reg_write(DATA_FORMAT_REG, reg);
}

int adxl343_set_power_control(uint8_t pwr)
{
    int result;
    uint8_t reg;

    if ((result = reg_read(POWER_CTL_REG, &reg)) != E_NO_ERROR)
        return result;
    reg &= ~MEASURE_MASK;
    reg |= pwr;
    return reg_write(POWER_CTL_REG, reg);
}

int adxl343_set_int_enable(uint8_t srcs)
{
    return reg_write(INT_ENABLE_REG, srcs);
}

int adxl343_set_int_map(uint8_t map)
{
    return reg_write(INT_MAP_REG, map);
}

int adxl343_get_int_source(uint8_t *srcs)
{
    return reg_read(INT_SOURCE_REG, srcs);
}

int adxl343_set_offsets(const int8_t *offs)
{
    uint8_t buf[4] = { OFSX_REG, offs[0], offs[1], offs[2] };

    i2c_req.tx_buf = buf;
    i2c_req.tx_len = sizeof(buf);
    i2c_req.rx_len = 0;

    return MXC_I2C_MasterTransaction(&i2c_req);
}

int adxl343_init(mxc_i2c_regs_t *i2c_inst)
{
    int result;
    uint8_t id;

    if (!i2c_inst)
        return E_NULL_PTR;

    i2c_req.i2c = i2c_inst;
    i2c_req.addr = ADXL343_ADDR;
    i2c_req.restart = 0;
    i2c_req.callback = NULL;

    if ((result = reg_read(DEVID_REG, &id)) != E_NO_ERROR)
        return result;
    if (id != DEVID)
        return E_NOT_SUPPORTED;

    return E_NO_ERROR;
}
