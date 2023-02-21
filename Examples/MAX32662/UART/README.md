## Description

This application uses two serial ports to send and receive data.  One serial port transmits data while the other receives it. The UART transaction can be executed using blocking or DMA methods, you may select between the two by commenting/uncommenting the DMA define at the top of main.

## Required Connections
-   Connect a USB cable between the PC and the USB/PWR (CN1) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   You must connect P0.2 (UART1 TX) to P0.11/AIN2 (UART0 RX).
-   Disconnect RX0A_EN (JP7) jumper.

## Expected Output

The Console UART of the device will output these messages:

```

**************** UART Example ******************
This example shows a loopback test between the 2 UARTs on the MAX32662.

Connect UART0 to UART1 (P0.11 (AIN2) -> P0.2) for this example.
The LEDs are used to indicate the success of the test.
Blinking->Success, Solid->Failure

-->UART Baud    : 115200 Hz

-->Test Length  : 1024 bytes

-->Initializing UARTS

-->Data verified

-->Example Succeeded
```

