## Description

This example demonstrates how to use the External_Flash library to communicate with the W25 external flash chip on the EvKit.

In this example, a portion of the external flash memory is erased (and verified), then the string "Analog Devices" is loaded into the flash and then the data string is read back to verify the write was successful.

The user may select between Quad and Single SPI interface modes by setting the value of the EXT_FLASH_SPIXFC_WIDTH definition at the top of main.c to either Ext_Flash_DataLine_Quad or Ext_Flash_DataLine_Single. 

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

* NOTE: This example is only supported by the MAX32655EVKIT.

## Required Connections
If using the MAX32655EVKIT (EvKit\_V1):
-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
********************* External_Flash Example *********************
This example communicates with an W25 flash over SPI (Quad mode).

SPI Clock: 5000000 Hz

External flash Initialized.

External flash ID: 0xef7018

Erasing first 64k sector
Erased

Quad mode enabled

Programming function (15 bytes @ 0x20000000) into external flash
Programmed
Written Data:Analog Devices

Verifying external flash
Verified
Read Data:Analog Devices

Example Succeeded

```

