## Description

This example communicates with the W25 flash on the EvKit. It loads "Analog Devices" array onto it and then reads it back.


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

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

