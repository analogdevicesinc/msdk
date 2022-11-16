## Description

This application demonstrates simultaneous CODEC recording and playback using DMA.

## Setup

This application runs on the MAX78000 Feather platform.

##### Required Connections:

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
