/*******************************************************************************
 * Copyright (C) Maxim Integrated Products, Inc., All Rights Reserved.
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
#include <stdint.h>
#include <stddef.h>
#include <mxc_device.h>
#include <mxc_delay.h>
#include "i2c.h"
#include "i2c_regs.h"
#include "max20303.h"

// Device I2C address
#define MAX20303_ADDR 0x50

// I2C clock rate
#define I2C_FREQ 100000

// Device ID register value
#define HARDWARE_ID 0x02

// Device register addresses
#define REG_HARDWARE_ID 0x00
#define REG_FIRMWARE_REV 0x01
#define REG_INT0 0x03
#define REG_INT1 0x04
#define REG_INT2 0x05
#define REG_STATUS0 0x06
#define REG_STATUS1 0x07
#define REG_STATUS2 0x08
#define REG_STATUS3 0x09
#define REG_SYSTEM_ERROR 0x0B
#define REG_INT_MASK0 0x0C
#define REG_INT_MASK1 0x0D
#define REG_INT_MASK2 0x0E
#define REG_AP_DATOUT0 0x0F
#define REG_AP_DATOUT1 0x10
#define REG_AP_DATOUT2 0x11
#define REG_AP_DATOUT3 0x12
#define REG_AP_DATOUT4 0x13
#define REG_AP_DATOUT5 0x14
#define REG_AP_DATOUT6 0x15
#define REG_AP_CMDOUT 0x17
#define REG_AP_RESPONSE 0x18
#define REG_AP_DATAIN0 0x19
#define REG_AP_DATAIN1 0x1A
#define REG_AP_DATAIN2 0x1B
#define REG_AP_DATAIN3 0x1C
#define REG_AP_DATAIN4 0x1D
#define REG_AP_DATAIN5 0x1E
#define REG_LDO_DIRECT 0x20
#define REG_MPC_DIRECTWRITE 0x21
#define REG_MPC_DIRECTRED 0x22
#define REG_HPT_RAM_ADDR 0x28
#define REG_HPT_RAM_DATA_H 0x29
#define REG_HPT_RAM_DATA_M 0x2A
#define REG_HPT_RAM_DATA_L 0x2B
#define REG_LED_STEP_DIRECT 0x2C
#define REG_LED0_DIRECT 0x2D
#define REG_LED1_DIRECT 0x2E
#define REG_LED2_DIRECT 0x2F
#define REG_HPT_DIRECT0 0x30
#define REG_HPT_DIRECT1 0x31
#define REG_HPT_RTI2C_AMP 0x32
#define REG_HPT_PAT_RAM_ADDR 0x33

// Device commands
#define CMD_GPIO_CONFIG_WRITE 0x01
#define CMD_GPIO_CONTROL_WRITE 0x03
#define CMD_GPIO_CONTROL_READ 0x04

#define CMD_LDO1_CONFIG_WRITE 0x40
#define CMD_LDO1_CONFIG_READ 0x41
#define CMD_LDO2_CONFIG_WRITE 0x42
#define CMD_LDO2_CONFIG_READ 0x43

// LDO configuration fields
#define LDOn_PASSIVE_DISCHARGE 0x10
#define LDOn_ACTIVE_DISCHARGE 0x08
#define LDOn_LDO_MODE 0x00
#define LDOn_SWITCH_MODE 0x04
#define LDOn_DISABLED 0x00
#define LDOn_ENABLED 0x01
#define LDOn_MPC_CONTROLLED 0x02
#define LDOn_LDO_DIRECT_CONTROLLED 0x03

#define LDO1_VSET_LIMITED_BY_INPUT 0x3A
#define LDO2_VSET_2V8 0x13

// Interrupt flags that signal an error
#define MXC_I2C_ERROR                                                                      \
    (MXC_F_I2C_INTFL0_ARB_ERR | MXC_F_I2C_INTFL0_TO_ERR | MXC_F_I2C_INTFL0_ADDR_NACK_ERR | \
     MXC_F_I2C_INTFL0_DATA_ERR | MXC_F_I2C_INTFL0_DNR_ERR | MXC_F_I2C_INTFL0_START_ERR |   \
     MXC_F_I2C_INTFL0_STOP_ERR)

static mxc_i2c_regs_t *i2c = NULL;

/* The native driver seems paranoid calling this function often. Follow suit
   until determined otherwise. */
static inline void i2c_flush(void)
{
    i2c->intfl0 = i2c->intfl0;
    i2c->intfl1 = i2c->intfl1;
    i2c->txctrl0 |= MXC_F_I2C_TXCTRL0_FLUSH;
    while (i2c->txctrl0 & MXC_F_I2C_TXCTRL0_FLUSH) {}
    i2c->rxctrl0 |= MXC_F_I2C_RXCTRL0_FLUSH;
    while (i2c->rxctrl0 & MXC_F_I2C_RXCTRL0_FLUSH) {}
}

/* Write val to reg */
static int reg_write(uint8_t reg, uint8_t val)
{
    i2c_flush();

    i2c->fifo = MAX20303_ADDR;
    i2c->mstctrl |= MXC_F_I2C_MSTCTRL_START;
    i2c->fifo = reg;
    i2c->fifo = val;
    i2c->mstctrl |= MXC_F_I2C_MSTCTRL_STOP;

    while (!(i2c->intfl0 & MXC_F_I2C_INTFL0_STOP)) {}
    i2c->intfl0 = MXC_F_I2C_INTFL0_STOP;

    return (i2c->intfl0 & MXC_I2C_ERROR) ? E_COMM_ERR : E_NO_ERROR;
}

/* Write len sequential bytes from buf starting at reg */
static int reg_write_buf(uint8_t reg, uint8_t *buf, int len)
{
    /* Limit write length to one FIFOs worth of data.
       This module in its current state would not benefit from the
       additional complexity. */
    if (len > (MXC_I2C_FIFO_DEPTH - 2)) {
        return E_BAD_PARAM;
    }

    i2c_flush();

    i2c->fifo = MAX20303_ADDR;
    i2c->mstctrl |= MXC_F_I2C_MSTCTRL_START;
    i2c->fifo = reg;

    while (len--) {
        i2c->fifo = *buf++;
    }

    i2c->mstctrl |= MXC_F_I2C_MSTCTRL_STOP;

    while (!(i2c->intfl0 & MXC_F_I2C_INTFL0_STOP)) {}
    i2c->intfl0 = MXC_F_I2C_INTFL0_STOP;

    return (i2c->intfl0 & MXC_I2C_ERROR) ? E_COMM_ERR : E_NO_ERROR;
}

/* Place data read from reg at buf */
static int reg_read(uint8_t reg, uint8_t *buf)
{
    i2c_flush();

    i2c->fifo = MAX20303_ADDR;
    i2c->mstctrl = MXC_F_I2C_MSTCTRL_START;
    i2c->fifo = reg;

    i2c->rxctrl1 = 1;
    i2c->mstctrl = MXC_F_I2C_MSTCTRL_RESTART;
    while (i2c->mstctrl & MXC_F_I2C_MSTCTRL_RESTART) {}
    i2c->fifo = MAX20303_ADDR | 0x01;

    while (!(i2c->intfl0 & MXC_F_I2C_INTFL0_RX_THD)) {}
    *buf = i2c->fifo;
    i2c->intfl0 = MXC_F_I2C_INTFL0_RX_THD;

    i2c->mstctrl = MXC_F_I2C_MSTCTRL_STOP;
    while (!(i2c->intfl0 & MXC_F_I2C_INTFL0_STOP)) {}
    i2c->intfl0 = MXC_F_I2C_INTFL0_STOP;

    return (i2c->intfl0 & MXC_I2C_ERROR) ? E_COMM_ERR : E_NO_ERROR;
}

int max20303_led(int led, int on)
{
    if (!i2c) {
        return E_UNINITIALIZED;
    }

    switch (led) {
    case 0:
    case 1:
    case 2:
        break;
    default:
        return E_NO_DEVICE;
    }

    return reg_write(REG_LED0_DIRECT + led, (on) ? 0x20 : 0x00);
}

int max20303_led_red(int on)
{
    if (!i2c) {
        return E_UNINITIALIZED;
    }

    return reg_write(REG_LED1_DIRECT, (on) ? 0x21 : 0x01);
}

int max20303_led_green(int on)
{
    if (!i2c) {
        return E_UNINITIALIZED;
    }

    return reg_write(REG_LED2_DIRECT, (on) ? 0x21 : 0x01);
}

int max20303_led_blue(int on)
{
    if (!i2c) {
        return E_UNINITIALIZED;
    }

    return reg_write(REG_LED0_DIRECT, (on) ? 0x21 : 0x01);
}

/*
    Microphone power is controlled by LDO1 configured as a load switch, ie LDO1 input is regulated 1V8.
*/
int max20303_mic_power(int on)
{
    int err;
    uint8_t buf[2];

    if (!i2c) {
        return E_UNINITIALIZED;
    }

    buf[0] = (on) ? (LDOn_ENABLED | LDOn_SWITCH_MODE) : (LDOn_DISABLED | LDOn_SWITCH_MODE);
    buf[1] = LDO1_VSET_LIMITED_BY_INPUT;

    if ((err = reg_write_buf(REG_AP_DATOUT0, buf, 2)) != E_NO_ERROR) {
        return err;
    }

    if ((err = reg_write(REG_AP_CMDOUT, CMD_LDO1_CONFIG_WRITE)) != E_NO_ERROR) {
        return err;
    }

    MXC_Delay(MXC_DELAY_MSEC(10));

    if ((err = reg_read(REG_AP_RESPONSE, buf)) != E_NO_ERROR) {
        return err;
    }

    if (buf[0] != CMD_LDO1_CONFIG_WRITE) {
        return E_NO_RESPONSE;
    }

    return E_NO_ERROR;
}

/*
    Camera power is supplied by LDO2 @ 2V8.
*/
int max20303_camera_power(int on)
{
    int err;
    uint8_t buf[2];

    if (!i2c) {
        return E_UNINITIALIZED;
    }

    buf[0] = (on) ? LDOn_ENABLED : LDOn_DISABLED;
    buf[1] = LDO2_VSET_2V8;

    if ((err = reg_write_buf(REG_AP_DATOUT0, buf, 2)) != E_NO_ERROR) {
        return err;
    }

    if ((err = reg_write(REG_AP_CMDOUT, CMD_LDO2_CONFIG_WRITE)) != E_NO_ERROR) {
        return err;
    }

    /* The datasheet indicates there is a 5ms (typ), 9ms (max) latency
       associated with setting commands. */
    MXC_Delay(MXC_DELAY_MSEC(10));

    /* The datasheet indicates reading the data in APResponse provides
       verification of the successful execution of an opcode.
       So let's do just that. */
    if ((err = reg_read(REG_AP_RESPONSE, buf)) != E_NO_ERROR) {
        return err;
    }

    if (buf[0] != CMD_LDO2_CONFIG_WRITE) {
        return E_NO_RESPONSE;
    }

    return E_NO_ERROR;
}

int max20303_sd_power(int on)
{
    int err;
    uint8_t buf[5];

    if (!i2c) {
        return E_UNINITIALIZED;
    }

    buf[0] = 4; // GPIO0 in push-pull output mode controlled with AP commands
    buf[1] = 4; // GPIO1 in push-pull output mode controlled with AP commands
    buf[2] = 4; // GPIO2 in push-pull output mode controlled with AP commands
    buf[3] = 4; // GPIO3 in push-pull output mode controlled with AP commands
    buf[4] = 4; // GPIO4 in push-pull output mode controlled with AP commands

    if ((err = reg_write_buf(REG_AP_DATOUT0, buf, 5)) != E_NO_ERROR) {
        return err;
    }

    if ((err = reg_write(REG_AP_CMDOUT, CMD_GPIO_CONFIG_WRITE)) != E_NO_ERROR) {
        return err;
    }

    /* The datasheet indicates there is a 5ms (typ), 9ms (max) latency
       associated with setting commands. */
    MXC_Delay(MXC_DELAY_MSEC(10));

    /* The datasheet indicates reading the data in APResponse provides
       verification of the successful execution of an opcode.
       So let's do just that. */
    if ((err = reg_read(REG_AP_RESPONSE, buf)) != E_NO_ERROR) {
        return err;
    }

    if (buf[0] != CMD_GPIO_CONFIG_WRITE) {
        return E_NO_RESPONSE;
    }

    // Read the current gpio status and set the SD card power bit only (first bit)
    if ((err = reg_write(REG_AP_CMDOUT, CMD_GPIO_CONTROL_READ)) != E_NO_ERROR) {
        return err;
    }

    /* The datasheet indicates there is a 5ms (typ), 9ms (max) latency
       associated with setting commands. */
    MXC_Delay(MXC_DELAY_MSEC(10));

    /* The datasheet indicates reading the data in APResponse provides
       verification of the successful execution of an opcode.
       So let's do just that. */
    if ((err = reg_read(REG_AP_RESPONSE, buf)) != E_NO_ERROR) {
        return err;
    }

    if (buf[0] != CMD_GPIO_CONTROL_READ) {
        return E_NO_RESPONSE;
    }

    // read the gpio values
    if ((err = reg_read(REG_AP_DATAIN0, buf)) != E_NO_ERROR) {
        return err;
    }

    if (on) {
        buf[0] &= ~(1); // First GPIO low, keep current state on others
    } else {
        buf[0] |= 1; // First GPIO high, keep current state on others
    }

    if ((err = reg_write_buf(REG_AP_DATOUT0, buf, 1)) != E_NO_ERROR) {
        return err;
    }

    if ((err = reg_write(REG_AP_CMDOUT, CMD_GPIO_CONTROL_WRITE)) != E_NO_ERROR) {
        return err;
    }

    /* The datasheet indicates there is a 5ms (typ), 9ms (max) latency
       associated with setting commands. */
    MXC_Delay(MXC_DELAY_MSEC(10));

    /* The datasheet indicates reading the data in APResponse provides
       verification of the successful execution of an opcode.
       So let's do just that. */
    if ((err = reg_read(REG_AP_RESPONSE, buf)) != E_NO_ERROR) {
        return err;
    }

    if (buf[0] != CMD_GPIO_CONTROL_WRITE) {
        return E_NO_RESPONSE;
    }

    return E_NO_ERROR;
}

int max20303_gpio(int gpio, int on)
{
    int err;
    uint8_t buf[5];

    if (!i2c) {
        return E_UNINITIALIZED;
    }

    switch (gpio) {
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
        break;
    default:
        return E_NO_DEVICE;
    }

    buf[0] = 4; // GPIO0 in push-pull output mode controlled with AP commands
    buf[1] = 4; // GPIO1 in push-pull output mode controlled with AP commands
    buf[2] = 4; // GPIO2 in push-pull output mode controlled with AP commands
    buf[3] = 4; // GPIO3 in push-pull output mode controlled with AP commands
    buf[4] = 4; // GPIO4 in push-pull output mode controlled with AP commands

    if ((err = reg_write_buf(REG_AP_DATOUT0, buf, 5)) != E_NO_ERROR) {
        return err;
    }

    if ((err = reg_write(REG_AP_CMDOUT, CMD_GPIO_CONFIG_WRITE)) != E_NO_ERROR) {
        return err;
    }

    /* The datasheet indicates there is a 5ms (typ), 9ms (max) latency
       associated with setting commands. */
    MXC_Delay(MXC_DELAY_MSEC(10));

    /* The datasheet indicates reading the data in APResponse provides
       verification of the successful execution of an opcode.
       So let's do just that. */
    if ((err = reg_read(REG_AP_RESPONSE, buf)) != E_NO_ERROR) {
        return err;
    }

    if (buf[0] != CMD_GPIO_CONFIG_WRITE) {
        return E_NO_RESPONSE;
    }

    // Read the current gpio status and set the gpio bit only
    if ((err = reg_write(REG_AP_CMDOUT, CMD_GPIO_CONTROL_READ)) != E_NO_ERROR) {
        return err;
    }

    /* The datasheet indicates there is a 5ms (typ), 9ms (max) latency
       associated with setting commands. */
    MXC_Delay(MXC_DELAY_MSEC(10));

    /* The datasheet indicates reading the data in APResponse provides
       verification of the successful execution of an opcode.
       So let's do just that. */
    if ((err = reg_read(REG_AP_RESPONSE, buf)) != E_NO_ERROR) {
        return err;
    }

    if (buf[0] != CMD_GPIO_CONTROL_READ) {
        return E_NO_RESPONSE;
    }

    // read the gpio values
    if ((err = reg_read(REG_AP_DATAIN0, buf)) != E_NO_ERROR) {
        return err;
    }

    if (on) {
        buf[0] |= 1 << gpio;
    } else {
        buf[0] &= ~(1 << gpio);
    }

    if ((err = reg_write_buf(REG_AP_DATOUT0, buf, 1)) != E_NO_ERROR) {
        return err;
    }

    if ((err = reg_write(REG_AP_CMDOUT, CMD_GPIO_CONTROL_WRITE)) != E_NO_ERROR) {
        return err;
    }

    /* The datasheet indicates there is a 5ms (typ), 9ms (max) latency
       associated with setting commands. */
    MXC_Delay(MXC_DELAY_MSEC(10));

    /* The datasheet indicates reading the data in APResponse provides
       verification of the successful execution of an opcode.
       So let's do just that. */
    if ((err = reg_read(REG_AP_RESPONSE, buf)) != E_NO_ERROR) {
        return err;
    }

    if (buf[0] != CMD_GPIO_CONTROL_WRITE) {
        return E_NO_RESPONSE;
    }

    return E_NO_ERROR;
}

int max20303_init(mxc_i2c_regs_t *i2c_inst)
{
    int err;
    uint8_t hw_id;

    if (i2c != NULL) {
        return E_NO_ERROR;
    }

    i2c = i2c_inst;

    /* Return if peripheral is already enabled and in master mode */
    if ((i2c->ctrl & (MXC_F_I2C_CTRL_EN | MXC_F_I2C_CTRL_MST_MODE)) ==
        (MXC_F_I2C_CTRL_EN | MXC_F_I2C_CTRL_MST_MODE)) {
        return E_NO_ERROR;
    } else {
        if ((err = MXC_I2C_Init(i2c, TRUE, 0)) != E_NO_ERROR) {
            return err;
        }
        MXC_I2C_SetFrequency(i2c, I2C_FREQ);
        MXC_I2C_SetTXThreshold(i2c, 1);
        MXC_I2C_SetRXThreshold(i2c, 1);
    }

    /* Test connectivity by reading hardware ID */
    if ((err = reg_read(REG_HARDWARE_ID, &hw_id)) != E_NO_ERROR) {
        return err;
    }

    if (hw_id != HARDWARE_ID) {
        return E_NOT_SUPPORTED;
    }

    return E_NO_ERROR;
}
