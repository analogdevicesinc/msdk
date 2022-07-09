/******************************************************************************
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
 *
 ******************************************************************************/
#include <stdio.h>
#include <stdint.h>
#include "sccb.h"
#include "mxc_delay.h"
#include "gpio.h"
#include "max32570.h"

// serial clock
#define SCL_PORT    MXC_GPIO2
#define SCL_PIN     MXC_GPIO_PIN_18
// serial data
#define SDA_PORT    MXC_GPIO2
#define SDA_PIN     MXC_GPIO_PIN_19

//
#define GPIO_SET(port, mask)    (port)->out_set = mask
#define GPIO_CLR(port, mask)    (port)->out_clr = mask
#define GPIO_GET(port, mask)    ((port)->in & mask)

//
#define GPIO_IN(port,  mask)    (port)->out_en_clr = mask
#define GPIO_OUT(port, mask)    (port)->out_en_set = mask

// SCL
#define SCL_HIGH()      GPIO_SET(SCL_PORT, SCL_PIN)
#define SCL_LOW()       GPIO_CLR(SCL_PORT, SCL_PIN)
// SDA
#define SDA_HIGH()      GPIO_SET(SDA_PORT, SDA_PIN)
#define SDA_LOW()       GPIO_CLR(SDA_PORT, SDA_PIN)
#define SDA_IN()        GPIO_IN (SDA_PORT, SDA_PIN)
#define SDA_OUT()       GPIO_OUT(SDA_PORT, SDA_PIN)
#define SDA_GET()       GPIO_GET(SDA_PORT, SDA_PIN)

#define WAIT_US         50
#define DELAY_US(us)    MXC_Delay(us)

static const mxc_gpio_cfg_t gpio_cfg_scl =   { SCL_PORT, SCL_PIN, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH};
static const mxc_gpio_cfg_t gpio_cfg_sda =   { SDA_PORT, SDA_PIN, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH};

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
        if ((byt << i) & 0x80)    {
            SDA_HIGH();
        }
        else                      {
            SDA_LOW();
        }
        
        DELAY_US(WAIT_US);
        
        SCL_HIGH();
        DELAY_US(WAIT_US);
        SCL_LOW();
        DELAY_US(WAIT_US);
    }
    
    SDA_IN();
    DELAY_US(WAIT_US);
    SCL_HIGH();
    DELAY_US(WAIT_US);
    
    if (SDA_GET()) {
        res = 1;    // means nack
    }
    else {
        res = 0;    // means ack
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

int sccb_read_byt(uint8_t slv_addr, uint8_t reg, uint8_t* byt)
{
    int ret = 0;
    
    start();
    
    if (ret == 0) {
        ret = send_byte(slv_addr << 1);   // address
    }
    
    if (ret == 0) {
        ret = send_byte(reg);
    }
    
    stop();
    
    if (ret == 0) {
        DELAY_US(WAIT_US);
        
        start();
        ret = send_byte((slv_addr << 1) + 1); // +1 means read
        
        if (ret == 0)   {
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
        ret = send_byte(slv_addr << 1);   // address
    }
    
    if (ret == 0) {
        ret = send_byte(reg);     //
    }
    
    if (ret == 0) {
        ret = send_byte(val);     //
    }
    
    stop();
    
    return ret;
}
