/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. (now owned by 
 * Analog Devices, Inc.),
 * Copyright (C) 2023-2024 Analog Devices, Inc.
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

#include <stdio.h>
#include "mxc_sys.h"
#include "mxc_errors.h"
#include "mxc_assert.h"
#include "board.h"
#include "uart.h"
#include "gpio.h"
#include "mxc_pins.h"
#include "led.h"
#include "pb.h"
#include "spixf.h"
#include "i2c.h"
#include "Ext_Flash.h"

/* **** Global Variables **** */
mxc_uart_regs_t *ConsoleUart = MXC_UART_GET_UART(CONSOLE_UART);
extern uint32_t SystemCoreClock;
extern uint8_t ChipRevision;

// clang-format off
const mxc_gpio_cfg_t pb_pin[] = {
    { MXC_GPIO2, MXC_GPIO_PIN_28, MXC_GPIO_FUNC_IN, MXC_GPIO_PAD_WEAK_PULL_UP, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 },
    { MXC_GPIO2, MXC_GPIO_PIN_30, MXC_GPIO_FUNC_IN, MXC_GPIO_PAD_WEAK_PULL_UP, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 },
};
const unsigned int num_pbs = (sizeof(pb_pin) / sizeof(mxc_gpio_cfg_t));

const mxc_gpio_cfg_t led_pin[] = {
    { MXC_GPIO2, MXC_GPIO_PIN_25, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 },
    { MXC_GPIO2, MXC_GPIO_PIN_26, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 },
};
const unsigned int num_leds = (sizeof(led_pin) / sizeof(mxc_gpio_cfg_t));
// clang-format on

const mxc_spixf_cfg_t mx25_spixc_cfg = {
    0, //mode
    0, //ssel_pol
    1000000 //baud
};

/* **** Functions **** */

/******************************************************************************/
/** 
 * NOTE: This weak definition is included to support Push Button interrupts in
 *       case the user does not define this interrupt handler in their application.
 **/
#if !defined(__ARMCC_VERSION) && !defined(__ICCARM__)
__weak
#endif
void GPIO2_IRQHandler(void)
{
    MXC_GPIO_Handler(MXC_GPIO_GET_IDX(MXC_GPIO2));
}

/******************************************************************************/
static int ext_flash_board_init(void)
{
    return MXC_SPIXF_Init(0, EXT_FLASH_BAUD);
}

/******************************************************************************/
static int ext_flash_board_read(uint8_t *read, unsigned len, unsigned deassert,
                                Ext_Flash_DataLine_t width)
{
    mxc_spixf_req_t req = { deassert, 0, NULL, read, (mxc_spixf_width_t)width, len, 0, 0, NULL };

    if (MXC_SPIXF_Transaction(&req) != len) {
        return E_COMM_ERR;
    }
    return E_NO_ERROR;
}

/******************************************************************************/
static int ext_flash_board_write(const uint8_t *write, unsigned len, unsigned deassert,
                                 Ext_Flash_DataLine_t width)
{
    mxc_spixf_req_t req = { deassert, 0, write, NULL, (mxc_spixf_width_t)width, len, 0, 0, NULL };

    if (MXC_SPIXF_Transaction(&req) != len) {
        return E_COMM_ERR;
    }
    return E_NO_ERROR;
}

/******************************************************************************/
static int ext_flash_clock(unsigned len, unsigned deassert)
{
    return MXC_SPIXF_Clocks(len, deassert);
}

/* ************************************************************************** */
void mxc_assert(const char *expr, const char *file, int line)
{
    printf("MXC_ASSERT %s #%d: (%s)\n", file, line, expr);
    while (1) {}
}

/* ************************************************************************** */
int Board_Init(void)
{
    int err;
    Ext_Flash_Config_t exf_cfg = { .init = ext_flash_board_init,
                                   .read = ext_flash_board_read,
                                   .write = ext_flash_board_write,
                                   .clock = ext_flash_clock };

    if ((err = Ext_Flash_Configure(&exf_cfg)) != E_NO_ERROR) {
        return err;
    }

    // Commented out on purpose.  Uncomment for Rev 1,2,3 boards since pmic is installed.
    // if ((err = MAX77650_Init()) != E_NO_ERROR) {
    //     MXC_ASSERT_FAIL();
    //     return err;
    // }

    if ((err = Console_Init()) != E_NO_ERROR) {
        return err;
    }

    if ((err = PB_Init()) != E_NO_ERROR) {
        MXC_ASSERT_FAIL();
        return err;
    }

    if ((err = LED_Init()) != E_NO_ERROR) {
        MXC_ASSERT_FAIL();
        return err;
    }

    return E_NO_ERROR;
}

/* ************************************************************************** */
int Console_Init(void)
{
    int err;

    if ((err = MXC_UART_Init(ConsoleUart, CONSOLE_BAUD)) != E_NO_ERROR) {
        return err;
    }

    return E_NO_ERROR;
}


#if !defined(__ARMCC_VERSION) && !defined(__ICCARM__)
/* ************************************************************************** */
void NMI_Handler(void)
{
    __NOP();
}
#endif /* __GNUC__ */

/* ************************************************************************** */
int MAX77650_Init(void)
{
    uint8_t data[2];
    int err;
    mxc_i2c_req_t i2c_req;

    if ((err = MXC_I2C_Init(MXC_I2C0, 1, 0)) != E_NO_ERROR) {
        return err;
    }

    MXC_I2C_SetFrequency(MXC_I2C0, MXC_I2C_FAST_SPEED);

    i2c_req.i2c = MXC_I2C0;
    i2c_req.addr = 0x90;
    i2c_req.tx_buf = data;
    i2c_req.tx_len = 2;
    i2c_req.rx_buf = NULL;
    i2c_req.rx_len = 0;
    i2c_req.restart = 0;
    /* Command PMIC to set VRTC voltage to 1.8V. */
    data[0] = 0x29;
    data[1] = 0x28;
    if ((err = MXC_I2C_MasterTransaction(&i2c_req)) != E_NO_ERROR) {
        return E_COMM_ERR;
    }

    /* Command PMIC to set VCORE voltage to 1.1V. */
    data[0] = 0x2B;
    data[1] = 0xD8;
    if ((err = MXC_I2C_MasterTransaction(&i2c_req)) != E_NO_ERROR) {
        return E_COMM_ERR;
    }

    /* Command PMIC to set VDDIOH voltage to 3.3V. */
    data[0] = 0x2D;
    data[1] = 0x32;
    if ((err = MXC_I2C_MasterTransaction(&i2c_req)) != E_NO_ERROR) {
        return E_COMM_ERR;
    }

    /* Command PMIC to set LDO voltage to 1.8V. */
    data[0] = 0x38;
    data[1] = 0x24;
    if ((err = MXC_I2C_MasterTransaction(&i2c_req)) != E_NO_ERROR) {
        return E_COMM_ERR;
    }

    if ((err = MXC_I2C_Shutdown(MXC_I2C0)) != E_NO_ERROR) {
        return err;
    }

    return E_NO_ERROR;
}

/******************************************************************************
 *
 *  These functions are defined multiple times in IAR and Keil startup files.
 *  Similar to the NMI and HardFault handlers, the GPIOn Handler functions
 *  that aren't used by the push button library will be defined here for the
 *  IAR and Keil.
 * 
 */
#if defined(__ARMCC_VERSION) || defined(__ICCARM__)
void GPIO0_IRQHandler(void)
{
    MXC_GPIO_Handler(MXC_GPIO_GET_IDX(MXC_GPIO0));
}

void GPIO1_IRQHandler(void)
{
    MXC_GPIO_Handler(MXC_GPIO_GET_IDX(MXC_GPIO1));
}

void GPIO3_IRQHandler(void)
{
    MXC_GPIO_Handler(MXC_GPIO_GET_IDX(MXC_GPIO3));
}
#endif
