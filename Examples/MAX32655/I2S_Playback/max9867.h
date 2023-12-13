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

#ifndef EXAMPLES_MAX32655_I2S_PLAYBACK_MAX9867_H_
#define EXAMPLES_MAX32655_I2S_PLAYBACK_MAX9867_H_

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------
#include <i2c_regs.h>

//-----------------------------------------------------------------------------
// Defines
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Function declarations
//-----------------------------------------------------------------------------
int max9867_init(mxc_i2c_regs_t *i2c_inst, int mclk, int lrclk);
int max9867_shutdown(void);
int max9867_status(void);

#endif // EXAMPLES_MAX32655_I2S_PLAYBACK_MAX9867_H_
