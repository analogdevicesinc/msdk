## Description

This example demonstrates using the UART (console UART) as a wake-up source from a low power state.


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analogdevicesinc.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the USB connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
******************** LP Serial Character Wake Up Example *******************

This example demonstrates how to send a serial character to wake up the device.
Each string sent will be echoed to the terminal. Strings sent to wake up the
device will appear as garbage due to wake-up latency, each of the following
strings will be processed correctly. Sending "sleep" will put the device
back in deep sleep and sending "quit" will end the example.

To measure wake-up latency, probe pins P0.13 (LED) and P0.11 (UART RX Pin).

**NOTE**: Each string sent to the device must end in a "\r" character for the
strings to be processed correctly.

Press PB1 to begin the demo.
Now entering deep sleep mode. Send any character string to wake up the device.

String Received: sleep
Going back to deep sleep.
String Received: quit

Stopping application.

```

