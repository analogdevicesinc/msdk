## Description

TBD<!--TBD-->


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

* This project comes pre-configured for the MAX32655EVKIT.  See [Board Support Packages](https://analog-devices-msdk.github.io/msdk/USERGUIDE/#board-support-packages) in the MSDK User Guide for instructions on changing the target board.

## Setup

### Required Connections
If using the Standard EV Kit (EvKit_V1):
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect pins JP4(RX_SEL) and JP5(TX_SEL) to RX0 and TX0  header.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

If using the Featherboard (FTHR\_Apps\_P1):
-   Connect a USB cable between the PC and the J4 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the board's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
****Low Power Mode Example****

This code cycles through the MAX32655 power modes, using a push button (PB1) to exit from each mode and enter the next.

Running in ACTIVE mode.
All unused RAMs shutdown.
Entering SLEEP mode.
Waking up from SLEEP mode.
Entering DEEPSLEEP mode.
Waking up from DEEPSLEEP mode.

```
