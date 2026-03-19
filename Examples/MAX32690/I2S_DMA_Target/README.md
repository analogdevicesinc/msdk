## Description

This application demonstrates simultaneous CODEC recording and playback using DMA.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analogdevicesinc.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Setup

Using the MAX32690 Ev kit board:
-   Connect a USB cable between the PC and the CN2 (USB/PWR - UART) connector.
-   Connect a line-in source to J5 connector.
-   Connect a headphone to J6 connector.
-   Optionally open a terminal application on the PC and connect to the Ev kit's console UART at 115200, 8-N-1.
-   Change JP10 jumper to SCL
-   Change JP9 jumper to SDA

## Expected Output

The Console UART of the device will output these messages:

```
***** MAX9867 CODEC DMA Loopback Example *****
Waiting...
Running...
I2C initialized successfully
Codec initialized successfully
I2S initialized successfully
```
