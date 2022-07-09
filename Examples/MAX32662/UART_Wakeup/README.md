## Description

This application uses two serial ports to send and receive data.  One serial port transmits data while the other receives it. The UART transaction can be executed using blocking or DMA methods, you may select between the two by commenting/uncommenting the DMA define at the top of main.

## Required Connections
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Install headers JP7(RX\_EN) and JP8(TX\_EN).
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

To measure wake-up latency, probe pins P0.14 (LED) and P0.11 (UART RX Pin).

**NOTE**: Each string sent to the device must end in a "\r" character for the
strings to be processed correctly.

Press PB1 to begin the demo.
Now entering sleep mode. Send any character string to wake up the device.

String Received: sleep
Going back to deep sleep.
String Received: quit
Stopping example...

```

