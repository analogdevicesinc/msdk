## Description

A basic getting started program.

This version of Hello_World prints an incrementing count to the console UART and toggles a GPIO (LED1) once every 500 ms.

## Required Connections

-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Select RX SEL and TX SEL on headers JP7 and JP8.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Close jumper JP5 (LED0 EN).
-   Close jumper JP6 (LED1 EN).

## Expected Output

The Console UART of the device will output these messages:

```
Hello World!
count : 0
count : 1
count : 2
count : 3
```

You will also observe LED1 blinking at a rate of 2Hz.
