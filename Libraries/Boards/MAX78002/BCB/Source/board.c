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
#include "mxc_device.h"
#include "mxc_sys.h"
#include "mxc_assert.h"
#include "board.h"
#include "uart.h"
#include "gpio.h"
#include "mxc_pins.h"
#include "led.h"
#include "pb.h"
#include "mxc_sys.h"

/***** Global Variables *****/
mxc_uart_regs_t *ConsoleUart = MXC_UART_GET_UART(CONSOLE_UART);
extern uint32_t SystemCoreClock;

// clang-format off
const mxc_gpio_cfg_t pb_pin[] = {
    { MXC_GPIO0, MXC_GPIO_PIN_16, MXC_GPIO_FUNC_IN, MXC_GPIO_PAD_PULL_UP, MXC_GPIO_VSSEL_VDDIOH, MXC_GPIO_DRVSTR_0},
    { MXC_GPIO0, MXC_GPIO_PIN_17, MXC_GPIO_FUNC_IN, MXC_GPIO_PAD_PULL_UP, MXC_GPIO_VSSEL_VDDIOH, MXC_GPIO_DRVSTR_0},
};
const unsigned int num_pbs = (sizeof(pb_pin) / sizeof(mxc_gpio_cfg_t));

const mxc_gpio_cfg_t led_pin[] = {
    { MXC_GPIO0, MXC_GPIO_PIN_18, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH, MXC_GPIO_DRVSTR_0},
    { MXC_GPIO0, MXC_GPIO_PIN_19, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH, MXC_GPIO_DRVSTR_0},
};
const unsigned int num_leds = (sizeof(led_pin) / sizeof(mxc_gpio_cfg_t));
// clang-format on

/***** File Scope Variables *****/
// const uart_cfg_t uart_cfg = {
//     UART_PARITY_DISABLE,
//     UART_DATA_SIZE_8_BITS,
//     UART_STOP_1,
//     UART_FLOW_CTRL_DIS,
//     UART_FLOW_POL_DIS,
//     CONSOLE_BAUD
// };

// const sys_cfg_uart_t uart_sys_cfg = {MAP_A,Enable};    // There is no special system configuration parameters for UART on MAX32650
// const sys_cfg_i2c_t i2c_sys_cfg = NULL;     // There is no special system configuration parameters for I2C on MAX32650
// const sys_cfg_spixc_t spixc_sys_cfg = NULL;   // There is no special system configuration parameters for SPIXC on MAX32650

// const spixc_cfg_t mx25_spixc_cfg = {
//     0, //mode
//     0, //ssel_pol
//     1000000 //baud
// };

/******************************************************************************/
void mxc_assert(const char *expr, const char *file, int line)
{
    printf("MXC_ASSERT %s #%d: (%s)\n", file, line, expr);

    while (1) {}
}

/******************************************************************************/
/** 
 * NOTE: This weak definition is included to support Push Button interrupts in
 *       case the user does not define this interrupt handler in their application.
 **/
__weak void GPIO0_IRQHandler(void)
{
    MXC_GPIO_Handler(MXC_GPIO_GET_IDX(MXC_GPIO0));
}

/******************************************************************************/
int Board_Init(void)
{
#ifndef __riscv
    int err;

    // Set SWDCLK and SWDIO pads to 3.3V
    MXC_GPIO0->vssel |= (3 << 28);

    // Enable GPIO
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_GPIO0);
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_GPIO1);

    if ((err = Console_Init()) < E_NO_ERROR) {
        // Cannot use this macro, sind it uses printf which won't work
        // MXC_ASSERT_FAIL();

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

#endif // __riscv

    return E_NO_ERROR;
}

/******************************************************************************/
int Console_Init(void)
{
    int err;

    if ((err = MXC_UART_Init(ConsoleUart, CONSOLE_BAUD, MXC_UART_IBRO_CLK)) != E_NO_ERROR) {
        return err;
    }

    return E_NO_ERROR;
}

/******************************************************************************/
int Console_Shutdown(void)
{
    int err;

    if ((err = MXC_UART_Shutdown(ConsoleUart)) != E_NO_ERROR) {
        return err;
    }

    return E_NO_ERROR;
}

/******************************************************************************/
void NMI_Handler(void)
{
    __NOP();
}

#ifdef __riscv
/******************************************************************************/
int Debug_Init(void)
{
    // Set up RISCV JTAG pins (P1[0..3] AF2)
    MXC_GPIO1->en0_clr = 0x0f;
    MXC_GPIO1->en1_set = 0x0f;
    MXC_GPIO1->en2_clr = 0x0f;

    return E_NO_ERROR;
}
#endif // __riscv
