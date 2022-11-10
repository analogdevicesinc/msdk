/******************************************************************************
 * Copyright (C) 2022 Maxim Integrated Products, Inc., All Rights Reserved.
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
