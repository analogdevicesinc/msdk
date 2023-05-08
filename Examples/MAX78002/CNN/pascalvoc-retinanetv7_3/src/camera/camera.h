#ifndef CAMERA_H
#define CAMERA_H

#include "mxc_errors.h"
#include "mipi_camera.h"

#define IMAGE_WIDTH 320
#define IMAGE_HEIGHT 256
#define PIXEL_FORMAT PIXEL_FORMAT_RGB565
#define PIXEL_ORDER PIXEL_ORDER_RGB565_RGB

#define SRAM_ADDRESS 0x0
// ^ This is the address in SRAM that captured images will be stored at.

// Update for future cameras
#if defined(CAMERA_OV5640)
#define CAMERA_ID 0x5640
#else // Default
#define CAMERA_ID 0x5640
#endif

/***** Globals *****/

/***** Functions *****/

bool camera_init();
void camera_capture(void);
void camera_capture_and_load_cnn(void);
void camera_display_last_image(void);

#endif
