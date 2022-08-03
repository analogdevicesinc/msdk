#ifndef EXAMPLE_CONFIG_H
#define EXAMPLE_CONFIG_H

// Configuration options
// ------------------------

#define CAMERA_FREQ 10000000

#if defined(CAMERA_HM01B0) || defined(CAMERA_HM0360)
#define CAMERA_MONO
#endif

#endif