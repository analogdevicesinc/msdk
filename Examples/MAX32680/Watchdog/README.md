## Description

A demonstration of the windowed features of the watchdog timer.

When the application begins, it initializes and starts the watchdog timer.  You can select between two tests: OVERFLOW and UNDERFLOW. Use SW1 (P0.26) to create a watchdog interrupt and reset.


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Setup

##### Required Connections
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect pins JP23(RX_SEL) and JP22(TX_SEL) to UART1 header.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
************** Watchdog Timer Demo ****************
Watchdog timer is configured in Windowed mode. You can
select between two tests: Timer Overflow and Underflow.

Press a button to create watchdog interrupt and reset:
SW1 (P0.26) = timeout and reset program

```

