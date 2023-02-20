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

This `Bootloader` application needs to be loaded to the first two flash pages. The main application `BLE_otas`
will run on top of this application. The linker file for the main application must coincide 
with the memory sections defined in this application. The main application is responsible 
for updating the update internal/external flash space.
The `project.mk` in this `Bootloader` application in conjunction with `project.mk` in BLE_otas determine
where the expected file is stored and read from.
Default configuration is to use an external flash to store the transferd file before
writing it to internal flash space during the update.
Alternatively by changing `USE_INTERNAL_FLASH ?=0` to `USE_INTERNAL_FLASH ?=1` the transfered file
is stored in the internal update flash space, see `ota_internal_mem.ld` `FLASH_UP`.   
## Expected Output

The green LED will blink when an update image has successfully been applied to the main image space.

The red LED will blink in the error case.
