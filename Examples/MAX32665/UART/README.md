## Description

This application uses two serial ports to send and receive data.  One serial port transmits data while the other receives it.

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
     - For EvKit
        -   Connect the jumper (JP7) to UART0.
        -   Connect the jumper (JP9) to RX0 and the jumper (JP10) to TX1.
-   Connect RX1 to TX0 with a wire.

## Expected Output

The Console UART of the device will output these messages:

```
**************** UART Example ******************
This example sends data from one UART to another

You will need to connect RX1 of UART1 and TX0 of UART0,
located to the right of the LCD.

Because UART1 is used to receive data,
not all printf statements will be printed to the terminal.

To indicate a successful UART transfer, the green LED (P1_15) will illuminate.
The red LED (P1_14) will illuminate when the transaction fails.

Push SW2 to continue

UART Baud       : 115200 Hz
Test Length     : 512 -->Data verified

-->EXAMPLE SUCCEEDED
```

The green LED (P1_15) will illuminate for successful transaction.
The red LED (P1_14) will illuminate if transaction failed.
