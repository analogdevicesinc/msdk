## Description
This example shows how to wake up a device with the wakeup timer. Pressing SW2 will put the device into sleep mode and enable the wakeup timer to trigger a wakeup event after MILLISECONDS\_WUT number of ms (defined at the top of _main.c_). 

## Setup
-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Install headers JP7(RX\_EN) and JP8(TX\_EN).
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

```
/*********** Wakeup timer example *************/
This example demonstrates how to use the Wakeup Timer.

Pressing SW2 to will put the chip to sleep and enable the
wakeup timer to wake the device in 5000 Miliseconds.

Entering SLEEP mode.
Waking up from SLEEP mode.
Entering SLEEP mode.
Waking up from SLEEP mode.
...
```
