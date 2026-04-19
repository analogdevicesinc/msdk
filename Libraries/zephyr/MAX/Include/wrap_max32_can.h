/******************************************************************************
 *
 * Copyright (C) 2024-2025 Analog Devices, Inc.
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

#ifndef LIBRARIES_ZEPHYR_MAX_INCLUDE_WRAP_MAX32_CAN_H_
#define LIBRARIES_ZEPHYR_MAX_INCLUDE_WRAP_MAX32_CAN_H_

/***** Includes *****/
#include <can.h>

#ifdef __cplusplus
extern "C" {
#endif

static inline int Wrap_MXC_CAN_Init(uint32_t can_idx, mxc_can_obj_cfg_t cfg,
                                    mxc_can_unit_event_cb_t unit_cb,
                                    mxc_can_object_event_cb_t obj_cb)
{
    /* The only API difference between the MAX32662 and MAX32690 relevant to
     * implementing the max32 CAN driver in Zephyr is that the former requires a
     * GPIO mapping parameter, which is not used anyway (MSDK_NO_GPIO_CLK_INIT).
     */

    /* MAX32662 related mapping */
#if defined(CONFIG_SOC_MAX32662)
    /* A value of -1 is invalid, will configure no GPIOs, but still pass through
     * to MXC_CAN_RevA_Init, even in case MSDK_NO_GPIO_CLK_INIT wasn't set.
     */
    return MXC_CAN_Init(can_idx, cfg, unit_cb, obj_cb, -1);

    /* MAX32690 related mapping */
#elif defined(CONFIG_SOC_MAX32690)
    return MXC_CAN_Init(can_idx, cfg, unit_cb, obj_cb);

#endif // part number
}

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_ZEPHYR_MAX_INCLUDE_WRAP_MAX32_CAN_H_
