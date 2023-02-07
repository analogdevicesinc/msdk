## Description

This example uses the I2C Master to find the addresses of any I2C Slave devices connected to the same bus as I2C1 (MAX78000FTHR) or I2C2 (MAX78000EVKIT).

## Setup
##### Building Firmware:
Before building firmware you must select the correct value for _BOARD_  in "project.mk", either "EvKit\_V1" or "FTHR\_RevA", depending on the EV kit you are using to run the example.

After doing so, navigate to the directory where the example is located using a terminal window. Enter the following comand to build all of the files needed to run the example.

```
$ make
```

##### Required Connections:

If using the standard (EvKit_V1):
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect pins 1 and 2 (P0_1) of the JH1 (UART 0 EN) header.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Close jumper JP1 (LED1 EN).
-   Close jumper JP2 (LED2 EN).
-   Connect I2C2 (SCL - P0.30, SDA - P0.31) to I2C bus. These pins are connected to camera header J4 (SDA - Pin 6, SCL - Pin 8).

If using the MAX78000FTHR (FTHR_RevA)
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Connect I2C1 (SCL - P0.16, SDA - P0.17) to I2C bus. These pins are connected to header J4 (SCL - Pin 11, SDA - Pin 12).

## Expected Output

The Console UART of the device will output these messages:

```
******** I2C SLAVE ADDRESS SCANNER *********

This example finds the addresses of any I2C Slave devices connected to the
same bus as I2C2 (SCL - P0.30, SDA - P0.31).

If desired you may connect I2C2 to an external bus through pins 6 (SDA)
and 8 (SCL) on the camera header J4.

-->I2C Master Initialization Complete
-->Scanning started
..................................................................................................
Found slave ID 105; 0x69
..............
-->Scan finished. 1 devices found
```
