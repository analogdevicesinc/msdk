## Description

A basic getting started program.

This version of Hello_World prints an incrementing count to the console UART and toggles a GPIO (P2.25 - LED0) once every 500 ms.

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
**********Hello World Example**********
count = 0
count = 1
count = 2
count = 3
count = 4
count = 5
...
```

