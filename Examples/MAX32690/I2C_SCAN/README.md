## Description
This example uses the I2C Master to find the addresses of any I2C Slave devices connected to the same bus as I2C0.

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Install JP7(RX_EN) and JP8(TX_EN) headers.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Close jumper JP5 (LED1 EN).
-   Close jumper JP6 (LED2 EN).
-   Select the proper pullup voltage for I2C0 using jumper JP2.
-   Enable I2C0 pullup resistors by installing jumpers JP3 and JP4.
-   Connect I2C0 (SDA - P2.7, SCL - P2.8) to I2C bus.

## Expected Output

The Console UART of the device will output these messages:

```
******** I2C SLAVE ADDRESS SCANNER *********

This example finds the addresses of any I2C Slave devices connected to the
same bus as I2C0 (SDA - P2.7, SCL - P2.8). Select the proper voltage for
the I2C0 pullup resistors using jumper JP2 and enable them by installing
jumpers JP3 and JP4.

-->I2C Master Initialization Complete
-->Scanning started
.........................................................................
Found slave ID 080; 0x50
.......................................
-->Scan finished. 1 devices found
```
