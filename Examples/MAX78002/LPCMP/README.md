## Description

This example demonstrates the use of the Analog Comparator to wake up the device from sleep mode. 

The example is configured to use analog channels 0 (P2.0) and 1 (P2.1) as the negative and positive comparator inputs respectively. A wakeup event is triggered when the comparator output transitions from low to high (analog 1 needs to transition from a voltage lower than analog 0 to voltage higher than analog 0).

## Setup

##### Required Connections
-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Connect the 5V power cable at (5V IN).
-   Close jumper (RX - P0.0) and (TX - P0.1) at Headers JP23 (UART 0 EN).
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Close jumper JP1 (LED1 EN).
-   Close jumper JP2 (LED2 EN).
-   Apply the negative comparator input to P2.0 (AIN0/AIN0N).
-   Apply the positive comparator input to P2.1 (AIN1/AIN0P).

## Expected Output

The Console UART of the device will output these messages:

```
********** Comparator Example **********

Connect the analog signal used as the positive comparator input to P2.1 (AIN1/AIN0P).
Connect the analog signal used as the negative comparator input to P2.0 (AIN0/AIN0N).

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
