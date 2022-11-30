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
#include <stdio.h>
#include <stdint.h>
#include "board.h"
#include "sccb.h"
#include "mxc_delay.h"
#include "gpio.h"
#include "mxc_device.h"

// clang-format off
// serial clock
#define SCL_PORT    SCCB_SCL_PORT
#define SCL_PIN     SCCB_SCL_PIN

// serial data
#define SDA_PORT    SCCB_SDA_PORT
#define SDA_PIN     SCCB_SDA_PIN

#define GPIO_SET(port, mask) (port)->out_set = mask
#define GPIO_CLR(port, mask) (port)->out_clr = mask
#define GPIO_GET(port, mask) (mask & (port)->in)

#define GPIO_IN(port, mask)  (port)->outen_clr = mask
#define GPIO_OUT(port, mask) (port)->outen_set = mask

// SCL
#define SCL_HIGH()  GPIO_SET(SCL_PORT, SCL_PIN)
#define SCL_LOW()   GPIO_CLR(SCL_PORT, SCL_PIN)

// SDA
#define SDA_HIGH()  GPIO_SET(SDA_PORT, SDA_PIN)
#define SDA_LOW()   GPIO_CLR(SDA_PORT, SDA_PIN)
#define SDA_IN()    GPIO_IN(SDA_PORT, SDA_PIN)
#define SDA_OUT()   GPIO_OUT(SDA_PORT, SDA_PIN)
#define SDA_GET()   GPIO_GET(SDA_PORT, SDA_PIN)

#define WAIT_US 4
#define DELAY_US(us) MXC_Delay(us)
// clang-format on

static const mxc_gpio_cfg_t gpio_cfg_scl = { SCL_PORT, SCL_PIN, MXC_GPIO_FUNC_OUT,
                                             MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO };
static const mxc_gpio_cfg_t gpio_cfg_sda = { SDA_PORT, SDA_PIN, MXC_GPIO_FUNC_OUT,
                                             MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO };

/******************************** Static Functions ***************************/
static void start(void)
{
    SDA_HIGH();
    DELAY_US(WAIT_US);
    SCL_HIGH();
    DELAY_US(WAIT_US);
    SDA_LOW();
    DELAY_US(WAIT_US);
    SCL_LOW();
    DELAY_US(WAIT_US);
}

static void stop(void)
{
    SDA_LOW();
    DELAY_US(WAIT_US);
    SCL_HIGH();
    DELAY_US(WAIT_US);
    SDA_HIGH();
    DELAY_US(WAIT_US);
}

static void send_NACK(void)
{
    SDA_HIGH();
    DELAY_US(WAIT_US);
    SCL_HIGH();
    DELAY_US(WAIT_US);
    SCL_LOW();
    DELAY_US(WAIT_US);
    SDA_LOW();
    DELAY_US(WAIT_US);
}

static uint8_t send_byte(uint8_t byt)
{
    uint8_t i;
    uint8_t res = 0;

    for (i = 0; i < 8; i++) {
        SCL_LOW();
        if ((byt << i) & 0x80) {
            SDA_HIGH();
        } else {
            SDA_LOW();
        }
        DELAY_US(WAIT_US);

        SCL_HIGH();
        DELAY_US(WAIT_US);
    }

    SCL_LOW();
    SDA_IN();
    DELAY_US(WAIT_US);
    SCL_HIGH();
    DELAY_US(WAIT_US);

    if (SDA_GET()) {
        res = 1; // means nack
    } else {
        res = 0; // means ack
    }

    SCL_LOW();
    DELAY_US(WAIT_US);
    SDA_OUT();

    return res;
}

static uint8_t get_byte(void)
{
    uint8_t byt = 0;
    uint8_t j;

    SDA_IN();
    DELAY_US(WAIT_US);

    for (j = 8; j > 0; j--) {
        SCL_HIGH();
        DELAY_US(WAIT_US);

        byt = byt << 1;

        if (SDA_GET()) {
            byt++;
        }

        SCL_LOW();
        DELAY_US(WAIT_US);
    }

    SDA_OUT();

    return byt;
}

/******************************** Public Functions ***************************/
int sccb_init(void)
{
    int ret = 0;

    MXC_GPIO_Config(&gpio_cfg_scl);
    MXC_GPIO_Config(&gpio_cfg_sda);

    SDA_HIGH();
    SCL_HIGH();
    return ret;
}

int sccb_vssel(mxc_gpio_vssel_t vssel)
{
    int sda_state;
    int scl_state;
    int ret = 0;

    // Preserve current state then set lines LOW before updating VSSEL.
    sda_state = MXC_GPIO_OutGet(SDA_PORT, SDA_PIN);
    scl_state = MXC_GPIO_OutGet(SCL_PORT, SCL_PIN);

    SDA_LOW();
    SCL_LOW();

    ret |= MXC_GPIO_SetVSSEL(SDA_PORT, vssel, SDA_PIN);
    ret |= MXC_GPIO_SetVSSEL(SCL_PORT, vssel, SCL_PIN);

    if (sda_state) {
        SDA_HIGH();
    }

    if (scl_state) {
        SCL_HIGH();
    }

    return ret;
}

int sccb_scan(void)
{
    int ret = 0;
    int slv_addr = 0;

    while (slv_addr < 0xFF) {
        start();
        ret = send_byte(slv_addr << 1); // address
        stop();

        if (ret == 0) {
            return slv_addr;
        }

        slv_addr++;
    }

    return -1; // not found
}

int sccb_read_byt(uint8_t slv_addr, uint8_t reg, uint8_t *byt)
{
    int ret = 0;

    start();

    if (ret == 0) {
        ret = send_byte(slv_addr << 1); // address
    }

    if (ret == 0) {
        ret = send_byte(reg);
    }

    stop();

    if (ret == 0) {
        DELAY_US(WAIT_US);

        start();
        ret = send_byte((slv_addr << 1) + 1); // +1 means read

        if (ret == 0) {
            *byt = get_byte(); //
            send_NACK();
        }

        stop();
    }

    return ret;
}

int sccb_write_byt(uint8_t slv_addr, uint8_t reg, uint8_t val)
{
    int ret = 0;

    start();

    if (ret == 0) {
        ret = send_byte(slv_addr << 1); // address
    }

    DELAY_US(100);

    if (ret == 0) {
        ret = send_byte(reg); //
    }

    DELAY_US(100);

    if (ret == 0) {
        ret = send_byte(val); //
    }

    DELAY_US(100);
    stop();

    return ret;
}

int sccb_read_reg16(uint8_t slv_addr, uint16_t reg, uint8_t *byte)
{
    int ret = 0;

    start();

    if (ret == 0) {
        ret = send_byte(slv_addr << 1); // address
    }

    if (ret == 0) {
        ret = send_byte(reg >> 8);
    }

    if (ret == 0) {
        ret = send_byte(reg);
    }

    stop();

    if (ret == 0) {
        DELAY_US(WAIT_US);

        start();
        ret = send_byte((slv_addr << 1) + 1); // +1 means read

        if (ret == 0) {
            *byte = get_byte(); //
            send_NACK();
        }

        stop();
    }

    return ret;
}

int sccb_write_reg16(uint8_t slv_addr, uint16_t reg, uint8_t val)
{
    int ret = 0;

    start();

    if (ret == 0) {
        ret = send_byte(slv_addr << 1); // address
    }

    if (ret == 0) {
        ret = send_byte(reg >> 8);
    }

    if (ret == 0) {
        ret = send_byte(reg);
    }

    if (ret == 0) {
        ret = send_byte(val); //
    }

    stop();

    return ret;
}
