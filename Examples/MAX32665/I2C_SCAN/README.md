## Description

This example uses the I2C Master to find the addresses of any I2C Slave devices connected to the same bus as I2C1.


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1..
-   Close jumper JP2 (I2C1_PU).
-   If you have:
    -	EvKit_V1 board: Connect I2C1 (SCL - P0.14, SDA - P0.15) to I2C bus, and close jumper JP2 (I2C1_PU). 
    -	FTHR2 board: 	Connect I2C1 (SCL - P0.14, SDA - P0.15) to I2C bus.
    -	FTHR board: 	Connect I2C0 (SCL - P0.6,  SDA - P0.7)  to I2C bus.

## Expected Output

The Console UART of the device will output these messages:

```
******** I2C SLAVE ADDRESS SCANNER *********

This example finds the addresses of any I2C Slave devices connected to the
same bus as I2C1 (SCL - P0.14, SDA - P0.15). Enable I2C1 pullup resistors
by connecting jumper JP2 (I2C1_PU).
-->I2C Master Initialization Complete
-->Scanning started
.........................................................................
Found slave ID 080; 0x50
.......................................
-->Scan finished. 1 devices found
```
