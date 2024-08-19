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
#include "UsageTrace.h"

/**
 * Initializes the UsageTrace functionality. Needs to be called before Start and
 * End are used
*/
void UsageTraceInit()
{
#ifdef USAGE_GPIO_OUT
    mxc_gpio_cfg_t gpioCfg;
    gpioCfg.port = USAGE_GPIO_PORT;
    gpioCfg.mask = USAGE_GPIO_PIN;
    gpioCfg.drvstr = MXC_GPIO_DRVSTR_0;
    gpioCfg.func = MXC_GPIO_FUNC_OUT;
    gpioCfg.pad = MXC_GPIO_PAD_NONE;
    gpioCfg.vssel = MXC_GPIO_VSSEL_VDDIO;
    USAGE_GPIO_PORT->out_clr = USAGE_GPIO_PIN;
    MXC_GPIO_Config(&gpioCfg);
#endif

#ifdef USAGE_TMR
    mxc_tmr_cfg_t tmrCfg;
    MXC_TMR_Shutdown(USAGE_TMR_INST);
    tmrCfg.pres = MXC_TMR_PRES_1;
    tmrCfg.mode = TMR_MODE_CONTINUOUS;
    tmrCfg.cmp_cnt = 0xFFFFFFFF;
    tmrCfg.pol = 0;
    MXC_TMR_Init(USAGE_TMR_INST, &tmrCfg);
    MXC_TMR_Start(USAGE_TMR_INST);
#endif
}
