## Description

This example demonstrates the various low power modes on the MAX32672.

The device is capable of entering the Sleep, Deep Sleep, Backup and Storage low power modes. The user may select which low power modes to enter in this example by setting the DO\_SLEEP, DO\_DEEPSLEEP, DO\_BACKUP, and DO\_STORAGE defines at the top of main.

Either the push button or the RTC may be used as a wakeup source in this example. Select which one is used by setting either the USE\_BUTTON or USE\_ALARM defines at the top of main.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

This example uses a special linkerfile (either lp-nonsecure.ld or lp-sla.ld depending on whether Secure Boot Tools are enabled) that limits the SRAM usage to SRAM0 and SRAM1. This prevents a hardfault that would otherwise occur when the other SRAMs are shutdown in the example.


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Select RX0 and TX0 on Headers JP10 and JP11 (UART 0).
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
************ Low Power Mode Example ************

This code cycles through the MAX32672 power modes, using a push button (SW3) to exit from each mode and enter the next.

Running in ACTIVE mode.
All unused RAMs placed in LIGHT SLEEP mode.
All unused RAMs shutdown.
Entering SLEEP mode.
Waking up from SLEEP mode.
Entering DEEPSLEEP mode.
Waking up from DEEPSLEEP mode.
```
