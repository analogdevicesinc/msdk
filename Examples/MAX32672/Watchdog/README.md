## Description

A demonstration of the windowed features of the watchdog timer.

When the application begins, it initializes and starts the watchdog timer.  The application then begins to reset the watchdog within the allowed window.  Use SW3 on the evaluation kit to control if and when the application attempts to reset the timer.

-Watchdog timer is configured in Windowed mode. You can select between two tests: Timer Overflow and Underflow. Press button SW3 to create watchdog interrupt and reset


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
************** Watchdog Timer Demo ****************
Watchdog timer is configured in Windowed mode. You can
select between two tests: Timer Overflow and Underflow.

Press a button to create watchdog interrupt and reset:
SW3 (P0.18)= timeout and reset program


Enabling Timeout Interrupt...

TIMEOUT!

Watchdog Reset occured too late (OVERFLOW)
```

