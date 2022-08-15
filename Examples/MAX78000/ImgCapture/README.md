# ImgCapture

![Example Image Capture in VSCode](img/Example-Capture.png)
(Above:  352x352 capture of a test pattern using the MAX78000FTHR's OV7692 camera)

## Description

This example demonstrates how to capture an image using the PCIF peripheral and Camera drivers.  It also demonstrates a method for collecting high resolution images and sending them to any arbitrary output destination, such as a UART interface or SD card.  

Collecting images can be a challenge when dealing with low-power CMOS camera sensors, which usually don't have memory buffers of any significant size.  Most will internally buffer a single row and then immediately stream it out to a high-speed Parallel Camera Interface (PCIF).  As a result, exposure time is proportional to the clock speed of the camera.  Therefore, the host microcontroller must be able to process, forward, and/or store the incoming image data within the strict timing limitations imposed by the camera module.  The limited general purpose SRAM size of microcontrollers make alternate output destinations (host PC, external memory, SD card, etc.) attractive, but these often have much lower bandwidth than the PCIF peripheral.  Slowing the PCIF clock speed can work for these, but is not an ideal solution because of the relationship between clock speed and exposure time.

In order to collect useful high resolution images, the 512KB of data SRAM in the MAX78000's CNN accelerator is "stiched"" into the primary buffer for image data.  The data can then be forwarded on to any secondary destination by the host firmware.  The CNN accelerator's data SRAM is non-contiguous, so the implementation of this method requires custom pointer manipulation and byte-packing that must also fall within the strict timing requirements mentioned above.  The implementation can be found in [src/cnn_memutils.h](src/cnn_memutils.h).  High-level usage examples for supported output destinations. can be found in [src/main.c](src/main.c).

The example can perform standard blocking captures up to the memory limits of the device, and streaming DMA captures (using cnn_memutils) up to 352x352, including QVGA (320x240) at max camera clock speeds.  Image captures are controlled from a host PC running [utils/console.py](utils/console.py).

It supports the following output destinations:

* Transmit image to a host PC.

* Save image to SD card.

## Building, Flashing, & Debugging

For instructions on how to build, flash, and debug this example see [.vscode/readme.md](.vscode/readme.md).

## Console Interface & Utilites

For more details on running the serial console, converting images, and excercising the example, see [utils/README.md](utils/README.md)

## Build Notes

### Enabling Firmware Features

Firmware features can be toggled in [src/example_config.h](src/example_config.h).

* The console is enabled by default.  It will wait for a valid connection from [utils/console.py](utils/console.py) on startup.

* SD card functionality is also enabled by default.  It will attempt to mount the SD card on startup using the FAT32 format.  If it detects a blank card has been inserted (the drive name is empty) then it will attempt to format the card to FAT32.

After making any changes, fully clean the project and then rebuild.

### Setting BOARD Correctly

Before you build, ensure you've set the `BOARD` value to match your evaluation platform.

For the MAX78000EVKIT, use `EvKit_V1`.
For the MAX78000FTHR, use `FTHR_RevA`.

Where exactly you set this value will depend on your development environment.

* If you're developing on the command-line, set this value in [project.mk](project.mk).

* If you're developing with Visual Studio Code, set `"board"` in [.vscode/settings.json](.vscode/settings.json).  See the VSCode-Maxim [readme](.vscode/readme.md) for more details.

* If you're developing with Eclipse, set the `"BOARD"` environment variable.  Right click project -> Properties -> C/C++ Build -> Environment -> `"BOARD"`.  Apply, clean, and rebuild.