#ifndef EXAMPLE_CONFIG_H
#define EXAMPLE_CONFIG_H

// Configuration options
// ------------------------

#define CONSOLE
// ^ Enables the serial console interface.

// #define SD
// ^ Enables SD card functionality.  An additional set of serial commands
// is added to the console (if it's enabled).  Additionally, pushbutton 0 can
// be pressed to snap an image to the SD card.

#define CAMERA_FREQ 10000000
// ^ Set the camera frequency

#if defined(CAMERA_HM0360_MONO) || defined(CAMERA_HM0360_COLOR) || defined(CAMERA_PAG7920) || defined(CAMERA_OV5642)
// These camera modules default to a higher resolution.  The HM0360 modules _only_ support a few 
// resolutions 320x240, 160x120, etc.
#define IMAGE_XRES 320
#define IMAGE_YRES 240
#else
#define IMAGE_XRES 64
#define IMAGE_YRES 64
#endif

#endif