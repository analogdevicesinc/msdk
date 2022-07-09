## Description
This example uses the I2C Master to find the addresses of any I2C Slave devices connected to the same bus as I2C0.

## Required Connections
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Install headers JP7(RX\_EN) and JP8(TX\_EN).
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Select the proper pullup voltage for I2C1 using jumper JP11.
-   Enable I2C1 pullup resistors by installing jumpers JP2 and JP4.
-   Connect I2C1 (SDA - P0.9, SCL - P0.6) to I2C bus.

## Expected Output

The Console UART of the device will output these messages:

```
******** I2C SLAVE ADDRESS SCANNER *********

This example finds the addresses of any I2C Slave devices connected to the
same bus as I2C1 (SDA - P0.9, SCL - P0.6). Select the proper voltage for
the I2C1 pullup resistors using jumper JP11 and enable them by installing
jumpers JP2 and JP4.

-->I2C Master Initialization Complete
-->Scanning started
.........................................................................
Found slave ID 080; 0x50
.......................................
-->Scan finished. 1 devices found
```
