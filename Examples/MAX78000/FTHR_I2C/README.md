## Description

This example uses I2C to cycle through the 8 colors of the RGB LED connected to the on-board MAX20303 Power Management IC. 

*** NOTE ***: This example is not supported by the MAX78000's Standard EV Kit (EvKit_V1).

## Setup

##### Required Connections
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Behavior
The LED should change colors once a second, cycling through the 8 colors (off, blue, red, purple, green, light blue, yellow, white).

The Console UART of the device will output these messages:

```
******** Featherboard I2C Demo *********

This demo uses I2C to change the state of the
on-board Power Management IC's RGB LED every second.

You should observe the LED cycling through it's
eight color states.
```
