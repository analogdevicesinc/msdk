#ifndef EXAMPLE_CONFIG_H
#define EXAMPLE_CONFIG_H

// Use RGB565 in camera setup, otherwise use RGB888
#define RGB565

#ifdef BOARD_EVKIT_V1
// Enable TFT by default on EVKIT
#define TFT_ENABLE
#endif

#ifdef BOARD_FTHR_REVA
// Disable TFT by default on FTHR
#define TFT_ENABLE
#endif

//#define USE_SAMPLEDATA
// ^ Uncomment this to use static sample data.
// ^ Comment this out to use live camera data.

#define CAMERA_FREQ (10 * 1000 * 1000)
#define TFT_BUFF_SIZE 50 // TFT buffer size

#endif
