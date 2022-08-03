#ifndef EXAMPLE_CONFIG_H
#define EXAMPLE_CONFIG_H

// Configuration options
// ------------------------
//#define ENABLE_TFT // Comment out to disable TFT and send image to serial port instead.

#ifdef ENABLE_TFT
// TFT configuration options.
#define STREAM_ENABLE
/* If enabled, camera is setup in streaming mode to send the image
line by line to TFT, or serial port as they are captured. Otherwise, it buffers the entire
image first and then sends to TFT or serial port.
With serial port set at 900kbps, it can stream for up to 80x80 with OV5642 camera in 
stream mode, or 176x144 when stream mode is disabled.  It can display on TFT up to 176x144 
if stream mode is disabled, or 320x240 if enabled
*/
// #define BUTTON
/*
If BUTTON is defined, you'll need to push PB1 to capture an image frame.  Otherwise, images
will be captured continuously.
*/
#endif
// ------------------------

/*
Compiler definitions...  These configure TFT and camera settings based on the options above
*/
#ifdef ENABLE_TFT

#ifdef BOARD_EVKIT_V1
#include "tft_ssd2119.h"
#endif

#ifdef BOARD_FTHR_REVA
#include "tft_ili9341.h"
#endif

#endif

#define CAMERA_FREQ 10000000

#if defined(CAMERA_HM01B0)
#define CAMERA_MONO

#ifdef STREAM_ENABLE
#define IMAGE_XRES 324 / 2
#define IMAGE_YRES 244 / 2

#else
#define IMAGE_XRES 80
#define IMAGE_YRES 80

#endif
#endif

#if defined(CAMERA_HM0360)
#define CAMERA_MONO

#ifdef STREAM_ENABLE
#define IMAGE_XRES 320
#define IMAGE_YRES 240

#else
#define IMAGE_XRES 80
#define IMAGE_YRES 80

#endif
#endif

#if defined(CAMERA_OV7692) || defined(CAMERA_OV5642)

#ifdef ENABLE_TFT
#ifdef STREAM_ENABLE
#define IMAGE_XRES 320
#define IMAGE_YRES 240

#else
#define IMAGE_XRES 176
#define IMAGE_YRES 144
#endif

#else
#ifdef STREAM_ENABLE
#define IMAGE_XRES 80
#define IMAGE_YRES 80
#else
#define IMAGE_XRES 176
#define IMAGE_YRES 144
#endif

#endif
#endif

#define X_START 0
#define Y_START 0

#endif