## Description

This example demonstrates the use of the Analog Comparator to wake up the device from sleep mode. 

The example is configured to use analog channels 0 and 1 as the negative and positive comparator inputs respectively. A wakeup event is triggered when the comparator output transitions from low to high (analog 1 needs to transition from a voltage lower than analog 0 to voltage higher than analog 0).

## Setup

##### Required Connections
-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Install headers JP7(RX\_EN) and JP8(TX\_EN).
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Apply the negative comparator input to the pin labeled 0 of the JH6 (Analog) header.
-	Apply the positive comparator input to the pin labeled 1 of the JH6 (Analog) header.

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
