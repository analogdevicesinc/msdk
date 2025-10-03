## Description

A demonstration of the windowed features of the watchdog timer.

When the application begins, it initializes and starts the watchdog timer.  The application then begins to reset the watchdog within the allowed window.  Use SW2 when the application attempts to reset the timer.

- If "OVERFLOW" is defined, pressing SW2 will intentionally stall the micro until the reset window has passed, triggering a "too-late" watchdog reset.
- If "UNDERFLOW is defined, pressing SW2 will reset the watchdog timer before the reset window, triggering a "too-early" watchdog reset.


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analogdevicesinc.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections
-   Connect a USB cable between the PC and the J1 (PWR-OBD-UART0) connector.
-   Connect pins JP19 (OBD VCOM EN) RX and TX header.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
************** Watchdog Timer Demo ****************
Watchdog timer is configured in Windowed mode. You can
select between two tests: Timer Overflow and Underflow.

Press a button to create watchdog interrupt and reset:
PB0 (P0.12)= timeout and reset program


Enabling Timeout Interrupt...

```

