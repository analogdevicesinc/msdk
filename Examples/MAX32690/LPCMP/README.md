## Description

This example demonstrates the use of the Analog Comparator to wake up the device from sleep mode. 

The example is configured to use analog channels 0 and 1 as the negative and positive comparator inputs respectively. A wakeup event is triggered when the comparator output transitions from low to high (analog 1 needs to transition from a voltage lower than analog 0 to voltage higher than analog 0).

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

If using the MAX32690EVKIT:
-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Install JP7(RX_EN) and JP8(TX_EN) headers.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Apply the negative comparator input to the pin labeled 0 of the JH6 (Analog) header.
-   Apply the positive comparator input to the pin labeled 1 of the JH6 (Analog) header.

If using the MAX32690FTHR:
-   Connect a USB cable between the PC and the J5 (USB/PWR) connector.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Apply the negative comparator input to the pin labeled AIN0 of the J2 header.
-   Apply the positive comparator input to the pin labeled AIN1 of the J2 header.

## Expected Output

The Console UART of the device will output these messages:

```
********** Comparator Example **********

Connect the analog signal used as the positive comparator input to analog pin 1.
Connect the analog signal used as the negative comparator input to analog pin 0.

The device will be placed in sleep mode and requires a rising edge of the
comparator output to wakeup.

Press SW2 to begin.

Entering sleep mode.
Waking up.

Entering sleep mode.
Waking up.

Entering sleep mode.
...
```
