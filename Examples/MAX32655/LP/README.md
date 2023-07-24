## Description

This example demonstrates the steps to enter and exit the various low-power modes supported by the MAX32655.

Users may decide which low power modes they wish to enter by setting or clearing the DO_SLEEP, DO_LPM, DO_UPM, DO_BACKUP, and DO_STANDBY macros at the top of main.

Users may also select whether they want to use a GPIO or RTC wakeup source. To use a GPIO wakeup source set the USE_BUTTON macro to 1; this will configure the GPIO connected to SW3 to send a wakeup signal each time the button is pressed. To use the RTC wakeup source set the USE_ALARM macro to 1; this will configure an RTC alarm to wake the device every DELAY_IN_SEC seconds.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

* This project comes pre-configured for the MAX32655EVKIT.  See [Board Support Packages](https://analog-devices-msdk.github.io/msdk/USERGUIDE/#board-support-packages) in the MSDK User Guide for instructions on changing the target board.

## Required Connections
If using the MAX32655EVKIT (EvKit\_V1):
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect pins JP4(RX_SEL) and JP5(TX_SEL) to RX0 and TX0  header.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

If using the MAX32655FTHR (FTHR\_Apps\_P1):
-   Connect a USB cable between the PC and the J4 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the board's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
****Low Power Mode Example****

This code cycles through the MAX32655 power modes, using a push button (PB1) to exit from each mode and enter the next.

Running in ACTIVE mode.
Entering SLEEP mode.
Waking up from SLEEP mode.
Entering LPM mode.
Waking up from LPM mode.

```
