## Description
This example uses the I2C Master to find the addresses of any I2C Slave devices connected to the same bus as I2C1.

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Connect the pull-up jumpers (JP4 and JP6) to the proper I/O voltage.
-   Connect I2C1 (SDA - P2.17, SCL - P2.18) to I2C bus.

## Expected Output

The Console UART of the device will output these messages:

```
******** I2C SLAVE ADDRESS SCANNER *********

This example finds the addresses of any I2C Slave devices connected to the
same bus as I2C1 (SDA - P2.17, SCL - P2.18). You must connect the pull-up
jumpers (JP4 and JP6) to the proper I/O voltage.
-->I2C Master Initialization Complete
-->Scanning started
.........................................................................
Found slave ID 080; 0x50
.......................................
-->Scan finished. 1 devices found
```
