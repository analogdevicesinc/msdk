/*******************************************************************************
* Copyright (C) 2022 Maxim Integrated Products, Inc., All rights Reserved.
*
* This software is protected by copyright laws of the United States and
* of foreign countries. This material may also be protected by patent laws
* and technology transfer regulations of the United States and of foreign
* countries. This software is furnished under a license agreement and/or a
* nondisclosure agreement and may only be used or reproduced in accordance
* with the terms of those agreements. Dissemination of this information to
* any party or parties not specified in the license agreement and/or
* nondisclosure agreement is expressly prohibited.
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
*******************************************************************************/
#ifndef __CAMERA_UTIL_H__
#define __CAMERA_UTIL_H__

#define CAMERA_FREQ (10 * 1000 * 1000)
#define BYTE_PER_PIXEL 2

#if defined(CAMERA_HM01B0)
#define IMAGE_XRES 324 / 2
#define IMAGE_YRES 244 / 2
#define CAMERA_MONO
//#define STREAM_ENABLE
#endif

#if defined(CAMERA_HM0360)
#define IMAGE_XRES 320
#define IMAGE_YRES 240
#define CAMERA_MONO
//#define STREAM_ENABLE
#endif

#if defined(CAMERA_OV7692) || defined(CAMERA_OV5642)

#define IMAGE_XRES 352
#define IMAGE_YRES 352

#define STREAM_ENABLE // If enabled, camera is setup in streaming mode to send the image line by line
// to TFT, or serial port as they are captured. Otherwise, it buffers the entire image first and
// then sends to TFT or serial port.
// With serial port set at 900kbps, it can stream for up to 80x80 with OV5642 camera in stream mode.
// or 176x144 when stream mode is disabled.
// It can display on TFT up to 176x144 if stream mode is disabled, or 320x240 if enabled.
#endif

#define X_START 0
#define Y_START 0

#define TFT_W 320
#define TFT_H 240

//#define PATTERN_GEN //  Replaces captured image with internally generated pattern

void process_img(void);
int initialize_camera(void);
void load_input_camera(void);
void run_camera(void);
void dump_cnn(void);
void display_camera(void);

#endif
