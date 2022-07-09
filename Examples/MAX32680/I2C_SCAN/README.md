## Description
This example uses the I2C Master to find the addresses of any I2C Slave devices connected to the same bus as I2C0.

## Required Connections

-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect pins JP22(RX_SEL) and JP23(TX_SEL) to UART1 header.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Select the proper pullup voltage for I2C0 using jumper JP8.
-   Connect I2C0 (SCL - P0.10, SDA - P0.11) to I2C bus.

## Expected Output

The Console UART of the device will output these messages:

```
******** I2C SLAVE ADDRESS SCANNER *********

This example finds the addresses of any I2C Slave devices connected to the
same bus as I2C0 (SCL - P0.10, SDA - P0.11). Select the proper voltage for
the I2C0 pullup resistors using jumper JP8.
-->I2C Master Initialization Complete
-->Scanning started
.........................................................................
Found slave ID 080; 0x50
.......................................
-->Scan finished. 1 devices found
```
