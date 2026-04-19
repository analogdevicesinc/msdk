/******************************************************************************
 *
 * Copyright (C) 2025 Analog Devices, Inc.
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

#include "max32660.h"
#include "mxc_sys.h"
#include "wdt_regs.h"

/* 
 * This function is called during boot up.
 */
void max32xx_system_init(void)
{
    MXC_WDT0->ctrl &=
        ~MXC_F_WDT_CTRL_WDT_EN; /* Turn off watchdog. Application can re-enable as needed. */

    /* Disable clocks to peripherals by default to reduce power */
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_DMA);
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_SPI0);
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_SPI1);
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_UART0);
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_UART1);
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_I2C0);
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_TMR0);
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_TMR1);
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_TMR2);
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_I2C1);
}
