## Description

This example uses the I2C Master to find the addresses of any I2C Slave devices connected to the same bus as I2C1.

## Required Connections
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect pins JP4(RX_SEL) and JP5(TX_SEL) to RX0 and TX0  header.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   You must connect I2C slave to P0.16 (SCL) and P0.17 (SDA).
-   You must also connect the pull-up jumpers (JP21 and JP22) to the proper I/O voltage.

## Expected Output

The Console UART of the device will output these messages:

```
******** I2C SLAVE ADDRESS SCANNER *********

This example finds the addresses of any I2C Slave devices connected to the
same bus as I2C1 (SCL - P0.16, SDA - P0.17). You must connect the pull-up
jumpers (JP21 and JP22) to the proper I/O voltage.
-->I2C Master Initialization Complete
-->I2C Scanning started
......................
Found slave ID 029; 0x1D
..........................................................................................
Found slave ID 119; 0x77

-->Scan finished. 2 devices found
```
