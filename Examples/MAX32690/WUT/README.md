## Description
This example shows how to wake up a device with the wakeup timer. Pressing SW2 (SW3 on feather board) will put the device into sleep mode and enable the wakeup timer to trigger a wakeup event after MILLISECONDS\_WUT number of ms (defined at the top of _main.c_). 

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

If using the MAX32690FTHR:
-   Connect a USB cable between the PC and the J5 (USB/PWR) connector.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

```
/*********** Wakeup timer example *************/
This example demonstrates how to use the Wakeup Timer.

Pressing SW2 to will put the chip to sleep and enable the
wakeup timer to wake the device in 5000 Miliseconds.

Entering SLEEP mode.
Waking up from SLEEP mode.
Entering SLEEP mode.
Waking up from SLEEP mode.
...
```
