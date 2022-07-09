## Description

This example demonstrates entering and exiting from the various operating modes. The operating modes which are used can be enabled or disabled by setting the DO_SLEEP, DO_LPM, DO_UPM, DO_BACKUP, and DO_STANDBY defines to 1 or 0 respectively. The mecahanism to switch to the next operating mode, either the RTC or the push button, is selected with the USE_ALARM and USE_BUTTON defines, only one may be selected at a time.

## Setup
-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Install headers JP7(RX_EN) and JP8(TX_EN).
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages if USE_ALARM is defined:

```
****Low Power Mode Example****

This code cycles through the MAX32690 power modes, using the RTC alarm
to exit from each mode.  The modes will change every 2 seconds.

Running in ACTIVE mode.
Entering SLEEP mode.
Waking up from SLEEP mode.
Entering LPM mode.
Waking up from LPM mode.
Entering SLEEP mode.
...
```

The Console UART of the device will output these messages if USE_BUTTON is defined:

```
****Low Power Mode Example****

This code cycles through the MAX32690 power modes. Use push button (SW2)
to exit from each power mode and enter the next.

Running in ACTIVE mode.
Entering SLEEP mode.
Waking up from SLEEP mode.
Entering LPM mode.
Waking up from LPM mode.
Entering SLEEP mode.
...
```
