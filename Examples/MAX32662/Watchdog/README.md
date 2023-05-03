## Description

A demonstration of the windowed features of the watchdog timer.

When the application begins, it initializes and starts the watchdog timer. The application then begins to feed the watchdog within the approriate  window.  Pushing SW2 will trigger a watchdog reset.

After pressing SW2:
- If "OVERFLOW" is defined, execution will enter an infinite loop which blocks the watchdog from being fed causing the watchdog count to exceed the upper limit on the window and the device to be reset. 
- If "UNDERFLOW" is defined, two consecutive watchdog feeds are executed. Performing two consecutive feeds guarantees that one of the feeds will occur before the watchdog window. If the watchdiog count is not within the window when the first feed is executed this will cause the watchdog reset, otherwise the watchdog count will simply reset. If execution reaches the second feed, the watchdog count is guaranteed to be outside the window since the watchdog was just fed immediately prior to that point thus the watchdog reset will be triggered.



## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Install headers JP7(RX\_EN) and JP8(TX\_EN).
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
************** Watchdog Timer Demo ****************
Watchdog timer is configured in Windowed mode. This example can be compiled
for two tests: Timer Overflow and Underflow. It's currently compiled for timer Overflow.

It should be noted that triggering the watchdog reset
will reset the microcontroller.  As such, this
example runs better without a debugger attached.

Press SW2 (PB0) to create watchdog interrupt and reset.

Feeding watchdog...
Feeding watchdog...
Feeding watchdog...
Feeding watchdog...

Holding to trigger overflow condition...

WATCHDOG INTERRUPT TRIGGERED!

Recovering from watchdog reset...
Watchdog Reset occured too late (OVERFLOW)

************** Watchdog Timer Demo ****************
Watchdog timer is configured in Windowed mode. This example can be compiled
for two tests: Timer Overflow and Underflow. It's currently compiled for timer Overflow.

...
```

