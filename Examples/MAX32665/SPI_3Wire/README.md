## Description

This example sends data between two SPI peripherals operating in 3-wire SPI mode.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

This example is supported on MAX32666FTHR and MAX32666FTHR2 boards. It is not supported on the MAX32666EVKit board.

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the board's (FTHR or FTHR2) console UART at 115200, 8-N-1.
-   You must make following connections:
-   SPI Clock Connection: Connect P0.27 - P0.19
-   SPI SS Connection   : Connect P0.7(GPIO) - P0.16
-   SPI Data Connection : Connect P0.25 - P0.17
	
## Expected Output

The Console UART of the device will output these messages:

```

******************* 3 Wire SPI Example for MAX32665/MAX32666 *******************
This example sends data between two SPI peripherals operating in 3-wire SPI mode.
SPI2 is configured as master and SPI1 is configured as slave.

Connections:
SPI Clock Connection: Connect P0.27 - P0.19
SPI SS Connection   : Connect P0.7(GPIO) - P0.16
SPI Data Connection : Connect P0.25 - P0.17

Master send / slave receive operation successful
Master receive / slave send operation successful

Master send / slave receive operation successful
Master receive / slave send operation successful


```