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
#ifndef EXAMPLES_MAX78000_IMGCAPTURE_INCLUDE_EXAMPLE_CONFIG_H_
#define EXAMPLES_MAX78000_IMGCAPTURE_INCLUDE_EXAMPLE_CONFIG_H_

// Configuration options
// ------------------------

#define CAMERA_FREQ 10000000
// ^ Set the camera frequency

#if defined(CAMERA_HM0360_MONO) || defined(CAMERA_HM0360_COLOR) || defined(CAMERA_PAG7920) || \
    defined(CAMERA_OV5642)
// These camera modules default to a higher resolution.  The HM0360 modules _only_ support a few
// resolutions 320x240, 160x120, etc.

#ifdef CAMERA_HM0360_MONO
#define IMAGE_XRES 320
#define IMAGE_YRES 240
#else
#define IMAGE_XRES 160
#define IMAGE_YRES 120
#endif

#else
#define IMAGE_XRES 64
#define IMAGE_YRES 64
#endif

#endif // EXAMPLES_MAX78000_IMGCAPTURE_INCLUDE_EXAMPLE_CONFIG_H_
