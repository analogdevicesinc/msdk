/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc., All Rights Reserved.
 * (now owned by Analog Devices, Inc.)
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
 ******************************************************************************
 *
 * Copyright 2023 Analog Devices, Inc.
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
#include <stddef.h>
#include <stdint.h>
#include "mxc_device.h"
#include "mxc_assert.h"
#include "mxc_sys.h"
#include "sfe.h"
#include "sfe_reva.h"

/* **** Functions **** */

int MXC_SFE_Init(void)
{
    MXC_GPIO_Config(&gpio_cfg_sfe);
    MXC_GPIO_Config(&gpio_cfg_sfe_ss0);
    MXC_GPIO_Config(&gpio_cfg_sfe_ss1);

    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_SFES);

    return MXC_SFE_RevA_Init();
}

int MXC_SFE_Shutdown(void)
{
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_SFES);
    return MXC_SFE_RevA_Shutdown();
}

int MXC_SFE_ReadEnable(void)
{
    return MXC_SFE_RevA_ReadEnable((mxc_sfe_reva_regs_t *)MXC_SFE);
}

int MXC_SFE_WriteEnable(void)
{
    return MXC_SFE_RevA_WriteEnable((mxc_sfe_reva_regs_t *)MXC_SFE);
}

int MXC_SFE_SetFlashAddress(uint32_t baseAdd, uint32_t topAdd)
{
    return MXC_SFE_RevA_SetFlashAddress((mxc_sfe_reva_regs_t *)MXC_SFE, baseAdd, topAdd);
}

int MXC_SFE_SetRAMAddress(uint32_t baseAdd, uint32_t topAdd)
{
    return MXC_SFE_RevA_SetRAMAddress((mxc_sfe_reva_regs_t *)MXC_SFE, baseAdd, topAdd);
}

int MXC_SFE_SetHostAddress(uint32_t RAMAdd, uint32_t FLASHAdd)
{
    return MXC_SFE_RevA_SetHostAddress((mxc_sfe_reva_regs_t *)MXC_SFE, RAMAdd, FLASHAdd);
}
