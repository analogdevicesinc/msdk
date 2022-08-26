## Description

This example demonstrate how 1-Wire master can be configured and read slave ROM ID.

## Required Connections

-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect a USB cable between the PC and the CN1 (USB/UART0) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Connect 1-Wire pin to the 1-Wire slave
-   Connect VCC and GND to the slave

## Expected Output

```
***** 1-Wire ROM (DS2401) Example *****
This example reads ROM ID of 1-Wire slave device
Connect 1-Wire pin, VCC and GND to the target
   - If you have FTHR or FTHR2 board 1-Wire pin: P0.12
   - If you have EvKit_V1 board      1-Wire pin: P0.24
For more information please take a look at your board schematic


ROM ID: 3B D0 15 00 00 00 00 C8
Example Succeeded
```

