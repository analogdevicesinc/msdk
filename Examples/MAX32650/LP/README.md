## Description

This example showcases the various settings which can be configured to put the device into lower power states. Namely, putting the device RAMs in light sleep and shutdown modes, and cycling through the device sleep modes.

The wakeup source can be configured to be either the RTC clock or the push button based on the selection of the "USE_BUTTON" and "USE_ALARM" macros. Additionally, sleep modes which are cycled through can be enabled and disabled with the "DO_SLEEP", "DO_DEEPSLEEP", "DO_BACKGROUND" and "DO_BACKUP" macros.

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
****Low Power Mode Example****

This code cycles through the MAX32650 power modes, using a push button (SW2) to exit from each mode and enter the next.

Running in ACTIVE mode.
All unused RAMs placed in LIGHT SLEEP mode.
All unused RAMs shutdown.
Entering SLEEP mode.
Entering DEEPSLEEP mode.
Entering SLEEP mode.
```
