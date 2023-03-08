## Description

This example uses the I2C Master to find the addresses of any I2C Slave devices connected to the same bus as I2C0.


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect pins JP10(RX_SEL) and JP11(TX_SEL) to RX0 and TX0  header.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Close jumpers JP4 and JP5 (I2C0_PU).
-   Connect I2C0 (SCL - P0.6, SDA - P0.7) to I2C bus.

## Expected Output

The Console UART of the device will output these messages:

```
******** I2C SLAVE ADDRESS SCANNER *********

This example finds the addresses of any I2C Slave devices connected to the
same bus as I2C0 (SCL - P0.6, SDA - P0.7). Enable I2C0 pullup resistors
by connecting jumpers JP4 and JP5.
-->I2C Master Initialization Complete
-->Scanning started
.........................................................................
Found slave ID 080; 0x50
.......................................
-->Scan finished. 1 devices found
```
