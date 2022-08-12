## Description

This example demonstrates how to capture an image using the PCIF peripheral and Camera drivers.  It also demonstrates a method for collecting high resolution images and sending them to any arbitrary output destination, which is a challenge when dealing with the high datarate of the PCIF peripheral.

It can perform standard blocking captures up to the memory limits of the device, and streaming DMA captures up to 352x352, including QVGA (320x240) at max camera clock speeds.  Standard blocking captures are the simplest to perform, while the streaming DMA captures should be used for higher resolutions.

Currently, the following output destinations are supported:
* Transmit image to a host PC running a serial console [utils/console.py](utils/console.py).
* Save image to SD card.

## Enabling Firmware Features

Firmware features can be toggled in [src/example_config.h](src/example_config.h).
* The console is enabled by default.  It will wait for a valid connection from [utils/console.py](utils/console.py) on startup.
* SD card functionality is also enabled by default.  It will attempt to mount the SD card on startup using the FAT32 format.  If it detects a blank card has been inserted (the drive name is empty) then it will attempt to format the card to FAT32.

After making any changes, fully clean the project and then rebuild.

## Setting BOARD Correctly

Before you build, ensure you've set the `BOARD` value to match your evaluation platform.

For the MAX78000EVKIT, use `EvKit_V1`.
For the MAX78000FTHR, use `FTHR_RevA`.

* If you're developing on the command-line, set this value in [project.mk](project.mk).

* If you're developing with Visual Studio Code, set `"board"` in [.vscode/settings.json](.vscode/settings.json).  See the VSCode-Maxim [readme](.vscode/readme.md) for more details.

* If you're developing with Eclipse, set the `"BOARD"` environment variable.  Right click project -> Properties -> C/C++ Build -> Environment -> `"BOARD"`.  Apply, clean, and rebuild.