## Description

This example demonstrates the use of the LCD Display on the MAX32650 EV Kit. This is accomplished by showing and clearing the Maxim Intergrated on the display continuously in a loop.

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
********** CLCD Example **********
```

The LCD Display will show the following patter in a loop: the Maxim Integrated logo for 3 seconds, logo fades to blank, white screen, white screen holds for 3 seconds.
