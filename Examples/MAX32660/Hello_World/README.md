## Description

A basic getting started program.

This version of Hello_World prints an incrementing count to the console UART and toggles a GPIO (P1.14 - LED1) once every 500 ms.

## Required Connections
-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
***********Hello World!***********

LED1 on P1.14 toggles every 500 ms

Count = 0
Count = 1
Count = 2
Count = 3
```

