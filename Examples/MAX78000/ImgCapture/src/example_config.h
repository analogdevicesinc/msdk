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

#endif