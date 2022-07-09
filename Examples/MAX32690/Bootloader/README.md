# Description

Simple Bootloader that reloads the main flash image. Using an external memory device is preferable,
the update speed is limited by the erase/write time of the internal flash.

A 32 bit CRC value is appended to the end of the update flash image. 
CRC32 is used to verify the integrity of the update image. If a valid update image is identified,
the main flash section is erased and replaced with the update image. If no valid update image
is identified, the Bootloader will boot the exiting image in the main flash space.

__0x10000000__: Bootloader  
__0x10004000__: Main flash space  
__0x10300000__: Update flash space

## Setup

This Bootloader application needs to be loaded to the first two flash pages. The main application
will run on top of this application. The linker file for the main application must coincide 
with the memory sections defined in this application. The main application is responsible 
for updating the update flash space.

 
## Expected Output

The green LED will blink when an update image has successfully been applied to the main image space.

The red LED will blink in the error case.
