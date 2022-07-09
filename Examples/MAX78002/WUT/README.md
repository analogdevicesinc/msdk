## Description

This example shows how the wake up timer is used. Press SW4 to sleep or wake up the device.

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Connect the 5V power cable at (5V IN).
-   Close jumper (RX - P0.0) and (TX - P0.1) at Headers JP23 (UART 0 EN).
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

```
/************** Wakeup timer example ********************/
This example is to show how the Wakeup timer is used and configured.
Press SW4 to sleep or wake up the device.

Entering SLEEP mode.
Waking up from SLEEP mode.
Entering SLEEP mode.
Waking up from SLEEP mode.

```

