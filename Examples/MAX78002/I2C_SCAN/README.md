## Description

This example uses the I2C Master to find the addresses of any I2C Slave devices connected to the same bus as I2C1.

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Connect the 5V power cable at (5V IN).
-   Close jumper (RX - P0.0) and (TX - P0.1) at Headers JP23 (UART 0 EN).
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Close jumper JP1 (LED1 EN).
-   Close jumper JP2 (LED2 EN).
-   Install jumpers JP19 and JP20 to enable I2C1 pullup resistors.
-   Connect I2C1 (SCL - P0.16, SDA - P0.17) to I2C bus.

## Expected Output

The Console UART of the device will output these messages:

```
******** I2C SLAVE ADDRESS SCANNER *********

This example finds the addresses of any I2C Slave devices connected to the
same bus as I2C1 (SCL - P0.16, SDA - P0.17). Install jumpers JP19 (SDA)
and JP20 (SCL) to enable I2C1 pullup resistors.
-->I2C Master Initialization Complete
-->Scanning started
.................
Found slave ID 024; 0x18
...............................................................................................
-->Scan finished. 1 devices found
```
