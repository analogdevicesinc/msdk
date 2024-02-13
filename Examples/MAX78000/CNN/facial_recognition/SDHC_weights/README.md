## Description

This example writes CNN weights for the FaceID model onto an SD card as a binary file.  It can be used to load the model onto an SD card if an adapter is unable to do so from a host PC.

Alternatively, if the host PC has a microSD card adapter the [weights_2.bin](weights_2.bin) file can be loaded into the root directory of the card from the PC.

Once loaded, the SD card is ready for use with the [facial_recognition](../README.md) demo.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

* This project is only supported for the MAX78000FTHR board.

## Setup

-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector on the MAX78000FTHR.
-   Insert a microSDHC card into the SD card slot on the bottom of the board.
-   Open a terminal application on the PC and connect to the FTHR's console UART at 115200, 8-N-1.
-   Flash and run the project.

## Expected Output

```
***** MAX78000 Writes CNN weights to SD card *****
Card inserted
SD card mounted.
SD Card Opened
Opened file 'weights_2.bin'
371364 total bytes to write
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
3000 bytes written to file
2364 bytes written to file
371364 total bytes written to file
File Closed
SD Unmonted
```