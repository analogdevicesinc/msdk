/******************************************************************************
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
 *
 ******************************************************************************/

/*******************************      INCLUDES    ****************************/
#include <stdio.h>
#include <stdint.h>

#include "terminal.h"
#include "platform.h"

#include "mxc_delay.h"
#include "i2c.h"
#include "spi.h"

#include "nvic_table.h"

/*******************************      DEFINES     ****************************/
// I2C
#define I2C_MASTER     MXC_I2C1
#define I2C_SLAVE_ADDR 0x55
#define I2C_FREQ       400000

//
#define SPI       MXC_SPI0
#define SPI_IRQ   SPI0_IRQn
#define SPI_SPEED 1000000

/******************************* Type Definitions ****************************/

/******************************* 	Variables 	  ****************************/

/******************************* Static Functions ****************************/

/******************************* Public Functions ****************************/
/*
 * 	I2C
 */
int plt_i2c_init(void)
{
    int ret = 0;

    MXC_I2C_Init(I2C_MASTER, 1, 0);
    MXC_I2C_SetFrequency(I2C_MASTER, I2C_FREQ);

    return ret;
}

int plt_i2c_write(unsigned char* src, unsigned int len)
{
    int ret = 0;
    mxc_i2c_req_t reqMaster;

    reqMaster.i2c      = I2C_MASTER;
    reqMaster.addr     = I2C_SLAVE_ADDR;
    reqMaster.tx_buf   = src;
    reqMaster.tx_len   = len;
    reqMaster.rx_buf   = NULL;
    reqMaster.rx_len   = 0;
    reqMaster.restart  = 0;
    reqMaster.callback = NULL;

    ret = MXC_I2C_MasterTransaction(&reqMaster);

    return ret;
}

int plt_i2c_read(unsigned char* dst, unsigned int len)
{
    int ret = 0;
    mxc_i2c_req_t reqMaster;

    reqMaster.i2c      = I2C_MASTER;
    reqMaster.addr     = I2C_SLAVE_ADDR;
    reqMaster.tx_buf   = NULL;
    reqMaster.tx_len   = 0;
    reqMaster.rx_buf   = dst;
    reqMaster.rx_len   = len;
    reqMaster.restart  = 0;
    reqMaster.callback = NULL;

    ret = MXC_I2C_MasterTransaction(&reqMaster);

    return ret;
}

/*
 * 	SPI
 */
int plt_spi_init(void)
{
    int ret = 0;

    // Configure the peripheral
    mxc_spi_pins_t spiPins;
    spiPins.clock = TRUE;
    spiPins.ss0   = TRUE;
    spiPins.miso  = TRUE;
    spiPins.mosi  = TRUE;
    spiPins.vssel = TRUE;
    spiPins.map_a = TRUE;
    MXC_SPI_Init(SPI, 1, 0, 1, 0, SPI_SPEED, spiPins);

    //
    MXC_SPI_SetDataSize(SPI, 8);
    MXC_SPI_SetWidth(SPI, SPI_WIDTH_STANDARD);

    return ret;
}

int plt_spi_write(unsigned char* src, unsigned int len)
{
    int ret = 0;
    mxc_spi_req_t req;

    //SPI Request
    req.spi        = SPI;
    req.ssIdx      = 0;
    req.ssDeassert = 1;
    req.completeCB = NULL;

    /* Bootloader SPI protocol on target expects command first,
	   The target does not able to handle full command on first transaction,
	   so send command first then the rest of it.
	   Instead handling this differentiation in bootloader.c file
	   it is preferred to handle here to make bootloader.c file generic
	*/
    if (len > 2) {
        req.txData = src;
        req.rxData = NULL;
        req.txLen  = 2;
        req.rxLen  = 0;
        req.txCnt  = 0;
        req.rxCnt  = 0;
        //
        ret = MXC_SPI_MasterTransaction(&req);

        if (ret == 0) {
            req.txData = &src[2];
            req.rxData = NULL;
            req.txLen  = len - 2;
            req.rxLen  = 0;
            req.txCnt  = 0;
            req.rxCnt  = 0;
            //
            ret = MXC_SPI_MasterTransaction(&req);
        }
    } else {
        req.txData = src;
        req.rxData = NULL;
        req.txLen  = len;
        req.rxLen  = 0;
        req.txCnt  = 0;
        req.rxCnt  = 0;
        //
        ret = MXC_SPI_MasterTransaction(&req);
    }

    return ret;
}

int plt_spi_read(unsigned char* dst, unsigned int len)
{
    int ret = 0;
    mxc_spi_req_t req;

    //SPI Request
    req.spi        = SPI;
    req.txData     = NULL;
    req.rxData     = dst;
    req.txLen      = 0;
    req.rxLen      = len;
    req.ssIdx      = 0;
    req.ssDeassert = 1;
    req.txCnt      = 0;
    req.rxCnt      = 0;
    req.completeCB = NULL;

    ret = MXC_SPI_MasterTransaction(&req);

    return ret;
}

/*
 *  GPIO
 */
static const mxc_gpio_cfg_t bl_pins[] = {
    {MXC_GPIO0, MXC_GPIO_PIN_8, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH},
    {MXC_GPIO0, MXC_GPIO_PIN_9, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH},
};

int plt_gpio_init(void)
{
    MXC_GPIO_Config(&bl_pins[0]);
    MXC_GPIO_Config(&bl_pins[1]);
    return 0;
}

void plt_gpio_set(unsigned int idx, int state)
{
    if (state) {
        MXC_GPIO_OutSet(bl_pins[idx].port, bl_pins[idx].mask);
    } else {
        MXC_GPIO_OutClr(bl_pins[idx].port, bl_pins[idx].mask);
    }
}

int plt_gpio_get(unsigned int idx)
{
    return !MXC_GPIO_InGet(bl_pins[idx].port, bl_pins[idx].mask);
}

/*
 *	Delay
 */
void plt_delay_ms(unsigned int ms)
{
    MXC_Delay(ms * 1000UL);
}
