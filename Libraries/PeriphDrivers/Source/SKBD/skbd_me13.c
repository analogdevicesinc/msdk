/* *****************************************************************************
 * Copyright(C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files(the "Software"),
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
 **************************************************************************** */

//#include "smoh.h"
#include "skbd.h"
#include "skbd_reva.h"
#include "gpio.h"
#include "mxc_assert.h"
#include "mxc_pins.h"
#include "mxc_sys.h"
#include "mxc_lock.h"

int MXC_SKBD_PreInit(void)
{
    return MXC_SKBD_RevA_PreInit();
}

int MXC_SKBD_Init(mxc_skbd_config_t config)
{
    /* Enable the SKBD clock i.e. just in case clock is disabled */
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_CTB);
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_KBD);

    // MXC_GPIO_Config(&gpio_cfg_kbd_P0);
    // MXC_GPIO_Config(&gpio_cfg_kbd_P1);
    MXC_GPIO_Config(&gpio_cfg_kbd_P2);

    return MXC_SKBD_RevA_Init((mxc_skbd_reva_regs_t*)MXC_SKBD, config);
}

int MXC_SKBD_EnableInterruptEvents(unsigned int events)
{
    return MXC_SKBD_RevA_EnableInterruptEvents((mxc_skbd_reva_regs_t*)MXC_SKBD, events);
}

int MXC_SKBD_DisableInterruptEvents(unsigned int events)
{
    return MXC_SKBD_RevA_DisableInterruptEvents((mxc_skbd_reva_regs_t*)MXC_SKBD, events);
}

int MXC_SKBD_ClearInterruptStatus(unsigned int status)
{
    return MXC_SKBD_RevA_ClearInterruptStatus((mxc_skbd_reva_regs_t*)MXC_SKBD, status);
}

int MXC_SKBD_InterruptStatus(unsigned int* status)
{
    return MXC_SKBD_RevA_InterruptStatus((mxc_skbd_reva_regs_t*)MXC_SKBD, status);
}

int MXC_SKBD_ReadKeys(mxc_skbd_keys_t* keys)
{
    return MXC_SKBD_RevA_ReadKeys((mxc_skbd_reva_regs_t*)MXC_SKBD, (mxc_skbd_reva_keys_t*)keys);
}

int MXC_SKBD_Close(void)
{
    /* Reset the SKBD controller */
    MXC_SYS_Reset_Periph(MXC_SYS_RESET1_KBD);

    /* Wait until SKBD reset completes */
    while (MXC_F_GCR_RST1_KBD & MXC_GCR->rst1)
        ;

    return MXC_SKBD_RevA_Close();
}

const char* MXC_SKBD_GetVersion(void)
{
    return MXC_SKBD_REVA_VERSION_STRING;
}
