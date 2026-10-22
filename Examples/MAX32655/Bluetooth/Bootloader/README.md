# Description

Simple Bootloader that reloads the main flash image. Using an external memory device is preferable,
the update speed is limited by the erase/write time of the internal flash.

A 32 bit CRC and legnth value is appended to the beggning of the update flash image. 
CRC32 is used to verify the integrity of the update image. If a valid update image is identified,
the main flash section is erased and replaced with the update image. If no valid update image
is identified, the Bootloader will boot the exiting image in the main flash space.

__0x10000000__: Bootloader  
__0x10004000__: Main flash space  

# Notes:
* Before build the project, in the subfolder "...\msdk\Libraries\BlePhy\MAX32655":
* For MAX32655 chip revision A1 (The laser marking ends with "...A1", indicating an A1 chip revision). Please backup files for chip B1 revision: libphy.a, libphy_hard.a and libphy_riscv.a (Rename to "libphy_b1.a, libphy_b1_hard.a and libphy_b1_riscv.a"). Use files for A1 chip revision (Rename files "libphy_a1.a, libphy_a1_hard.a and libphy_a1_riscv.a" to "libphy.a, libphy_hard.a and libphy_riscv.a").
* For MAX32655 chip revision B1 (The laser marking ends with "...B1", indicating an B1 chip revision). Please backup files for chip A1 revision: libphy.a, libphy_hard.a and libphy_riscv.a (Rename to "libphy_a1.a, libphy_a1_hard.a and libphy_a1_riscv.a"). Use files for b1 chip revision (Rename files "libphy_b1.a, libphy_b1_hard.a and libphy_b1_riscv.a" to "libphy.a, libphy_hard.a and libphy_riscv.a").

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


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analogdevicesinc.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

* This project comes pre-configured for the MAX32655EVKIT.  See [Board Support Packages](https://analogdevicesinc.github.io/msdk/USERGUIDE/#board-support-packages) in the MSDK User Guide for instructions on changing the target board.

