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

#ifndef LIBRARIES_ZEPHYR_MAX_INCLUDE_WRAP_MAX32XXX_H_
#define LIBRARIES_ZEPHYR_MAX_INCLUDE_WRAP_MAX32XXX_H_

#ifdef __cplusplus
extern "C" {
#endif

#if defined(CONFIG_SOC_MAX32520)
#include <max32520.h>
#elif defined(CONFIG_SOC_MAX32570)
#include <max32570.h>
#elif defined(CONFIG_SOC_MAX32572)
#include <max32572.h>
#elif defined(CONFIG_SOC_MAX32650)
#include <max32650.h>
#elif defined(CONFIG_SOC_MAX32651)
#include <max32650.h>
#elif defined(CONFIG_SOC_MAX32655)
#include <max32655.h>
#elif defined(CONFIG_SOC_MAX32660)
#include <max32660.h>
#elif defined(CONFIG_SOC_MAX32662)
#include <max32662.h>
#elif defined(CONFIG_SOC_MAX32665)
#include <max326665.h>
#elif defined(CONFIG_SOC_MAX32666)
#include <max32665.h>
#elif defined(CONFIG_SOC_MAX32670)
#include <max32670.h>
#elif defined(CONFIG_SOC_MAX32672)
#include <max32672.h>
#elif defined(CONFIG_SOC_MAX32675)
#include <max32675.h>
#elif defined(CONFIG_SOC_MAX32680)
#include <max32680.h>
#elif defined(CONFIG_SOC_MAX32690)
#include <max32690.h>
#elif defined(CONFIG_SOC_MAX78000)
#include <max78000.h>
#elif defined(CONFIG_SOC_MAX78002)
#include <max78002.h>
#elif defined(CONFIG_SOC_MAX32657)
#include <max32657.h>
#endif

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_ZEPHYR_MAX_INCLUDE_WRAP_MAX32XXX_H_
