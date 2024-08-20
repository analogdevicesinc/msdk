/******************************************************************************
 *
 * Copyright (C) 2024 Analog Devices, Inc.
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
#include "Debug.h"

void DebugInitialize()
{
    mxc_gpio_cfg_t pinCfg;

    pinCfg.func = MXC_GPIO_FUNC_OUT;
    pinCfg.mask = DBG_IRQ_PIN;
    pinCfg.port = DBG_IRQ_PORT;
    pinCfg.vssel = MXC_GPIO_VSSEL_VDDIOH;
    pinCfg.pad = MXC_GPIO_PAD_NONE;
    pinCfg.drvstr = MXC_GPIO_DRVSTR_0;
    MXC_GPIO_OutClr(DBG_IRQ_PORT, DBG_IRQ_PIN);
    MXC_GPIO_Config(&pinCfg);

    pinCfg.func = MXC_GPIO_FUNC_OUT;
    pinCfg.mask = DBG_SQ_PIN;
    pinCfg.port = DBG_SQ_PORT;
    pinCfg.vssel = MXC_GPIO_VSSEL_VDDIOH;
    pinCfg.pad = MXC_GPIO_PAD_NONE;
    pinCfg.drvstr = MXC_GPIO_DRVSTR_0;
    MXC_GPIO_OutClr(DBG_SQ_PORT, DBG_SQ_PIN);
    MXC_GPIO_Config(&pinCfg);
}
