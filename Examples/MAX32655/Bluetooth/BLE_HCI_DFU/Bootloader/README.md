# Description

Simple Bootloader that reloads the main flash image. Using an external memory device is preferable,
the update speed is limited by the erase/write time of the internal flash.

A 32 bit CRC and legnth value is appended to the beggning of the update flash image. 
CRC32 is used to verify the integrity of the update image. If a valid update image is identified,
the main flash section is erased and replaced with the update image. If no valid update image
is identified, the Bootloader will boot the exiting image in the main flash space.

__0x10000000__: Bootloader  
__0x10004000__: Main flash space  


## Setup

This `Bootloader` application needs to be loaded to the first two flash pages. The main application `BLT`
will run on top of this application. The linker file for the main application must coincide 
with the memory sections defined in this application. The main application is responsible 
for updating the update internal/external flash space.
The `project.mk` in this `Bootloader` application in conjunction with `project.mk` in `BLT` determine
where the expected file is stored and read from.
Default configuration in this example is to use an internal flash to store the transferd file.
Alternatively by changing `USE_INTERNAL_FLASH ?=1` to `USE_INTERNAL_FLASH ?=0` the transfered file
is stored in the external update flash space. Please remember, the `project.mk` file in `BLT` and `Second_App` should have the same configuration. 


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analogdevicesinc.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

* This project comes pre-configured for the MAX32655EVKIT.  See [Board Support Packages](https://analogdevicesinc.github.io/msdk/USERGUIDE/#board-support-packages) in the MSDK User Guide for instructions on changing the target board.

