/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. All Rights Reserved.
 * (now owned by Analog Devices, Inc.),
 * Copyright (C) 2023 Analog Devices, Inc. All Rights Reserved. This software
 * is proprietary to Analog Devices, Inc. and its licensors.
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

#ifndef EXAMPLES_MAX78000_TFT_DEMO_EXAMPLE_CONFIG_H_
#define EXAMPLES_MAX78000_TFT_DEMO_EXAMPLE_CONFIG_H_

#include "board.h"

// Enable TFT display
#define ENABLE_TFT

// Board specific options...
// ---
#ifdef BOARD_EVKIT_V1
#include "tft_ssd2119.h"
#include "bitmap.h"
#define ENABLE_TS
#endif

#ifdef BOARD_FTHR_REVA
#include "tft_ili9341.h"
#endif
// ---

#endif // EXAMPLES_MAX78000_TFT_DEMO_EXAMPLE_CONFIG_H_
