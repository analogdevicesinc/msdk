## Description

This example demonstrates the use of the Analog Comparator to wake up the device from deep sleep. 

The example is configured to use analog channels 3 and 7 as the negative and positive comparator inputs respectively. A wakeup event is triggered when the comparator output transitions from low to high (analog 7 needs to transition from a voltage lower than analog 3 to voltage higher than analog 3).


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Setup

##### Required Connections
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect pins JP10(RX_SEL) and JP11(TX_SEL) to RX0 and TX0  header.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Apply the negative comparator input to the pin labeled 3 of the JH1 (Analog) header.
-	Apply the positive comparator input to the pin labeled 7 of the JH2 (Analog) header.

## Expected Output

The Console UART of the device will output these messages:

```
********** Comparator Example **********

Connect the analog signal used as the positive comparator input to analog pin 7.
Connect the analog signal used as the negative comparator input to analog pin 3.

The device will be placed in Deep Sleep and requires an edge transition of
the comparator output to wakeup.

Press SW3 to begin.

Entering slee mode.
Waking up.

Entering slee mode.
Waking up.

...
```
