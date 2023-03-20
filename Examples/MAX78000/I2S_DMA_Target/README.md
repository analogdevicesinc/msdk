## Description

This application demonstrates simultaneous CODEC recording and playback using DMA.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

* This project comes pre-configured for the MAX78000FTHR.  It is only compatible with that board.

## Setup

Using the MAX78000 Feather board:
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect a line-in source to J5 connector.
-   Connect a line-in monitor to J7 connector.
-   Optionally open a terminal application on the PC and connect to the EV feather's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
***** MAX9867 CODEC DMA Loopback Example *****
Waiting...
Running...
```
