## Description

This example cycles through the various power states on the MAX32675. 

To move to the next power state push button 0 (SW1).

## Required Connections

-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
****Low Power Mode Example****

This code cycles through the MAX32675 power modes, using a push button 0 (SW1) to exit from each mode and enter the next.

Running in ACTIVE mode.
All unused RAMs placed in LIGHT SLEEP mode.
All unused RAMs shutdown.
Entering SLEEP mode.
Waking up from SLEEP mode.
Entering DEEPSLEEP mode.
Waking up from DEEPSLEEP mode.

```
