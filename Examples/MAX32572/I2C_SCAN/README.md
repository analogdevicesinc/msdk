## Description

This example uses the I2C Master to find the addresses of any I2C Slave devices connected to the same bus as I2C0.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the J4 (USB to UART0) connector.
-   Install P1.8 (UART0 RX EN) and P1.9 (UART0 TX EN) on header JP8.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Close jumper JP6 (LED0 EN).
-   Close jumper JP7 (LED1 EN).
-   Connect I2C0 (SCL - P0.0, SDA - P0.1) to I2C bus.

## Expected Output

The Console UART of the device will output these messages:

```
**************** I2C SLAVE ADDRESS SCANNER *****************

This example finds the addresses of any I2C Slave devices
connected to the same bus as I2C0 (SCL - P0.0, SDA - P0.1).
-->I2C Master Initialization Complete
-->Scanning started
...............................................
Found slave ID 054; 0x36
..................
Found slave ID 072; 0x48
...............................................
-->Scan finished. 2 devices found
```
