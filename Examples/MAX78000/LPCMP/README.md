## Description

This example demonstrates the use of the Analog Comparator to wake up the device from sleep mode.

The example is configured to use two analog channels as the negative and positive input to the low-power comparator. A wakeup event is triggered when the comparator output transitions from low to high (positive input needs to transition from a lower voltage than the negative input to a higher voltage than the negative input).

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

* This project comes pre-configured for the MAX78000EVKIT.  See [Board Support Packages](https://analog-devices-msdk.github.io/msdk/USERGUIDE/#board-support-packages) in the UG for instructions on changing the target board.

## Setup

If using the MAX78000EVKIT (EvKit_V1):
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect pins 1 and 2 (P0_1) of the JH1 (UART 0 EN) header.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Apply the negative comparator input to AIN0/AIN0N (JH3).
-	Apply the positive comparator input to AIN1/AIN0P (JH3).

If using the MAX78000FTHR (FTHR_RevA)
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Apply the negative comparator input to AIN6/AIN3N (RX Pin on header J8).
-	Apply the positive comparator input to AIN7/AIN3P (TX Pin on header J8).

## Expected Output

The Console UART of the device will output these messages:

```
******************** Comparator Example ********************

Connect the analog signal used as the positive comparator input
to analog pin 1 (AIN1/AIN0P).
Connect the analog signal used as the negative comparator input
to analog pin 0 (AIN0/AIN0N).

The device will be placed in sleep mode and requires a rising
edge of the comparator output to wakeup.

Press SW2 to begin.

Entering sleep mode.
Waking up.

Entering sleep mode.
Waking up.
    .
    .
    .
```
