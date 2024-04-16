/******************************************************************************
 *
 * Copyright (C) 2023 Analog Devices, Inc.
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

#include "max32690.h"
#include "icc.h"

/* 
 * This function is called during boot up.
 */
void max32xx_system_init(void)
{
    *(volatile uint32_t *)0x40000c00 = 1; // Enable test mode
    *(volatile uint32_t *)0x4000040c = (1 << 6); // Disable cache read buffer
    *(volatile uint32_t *)0x40000c00 = 0; // Disable test mode

    // Enable then disable ICC to clear the cache
    MXC_ICC_Enable(MXC_ICC0);
    MXC_ICC_Disable(MXC_ICC0);
    MXC_ICC_Enable(MXC_ICC0);
}
