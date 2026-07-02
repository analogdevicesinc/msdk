## Description

This application uses two serial ports to send and receive data.  One serial port transmits data while the other receives it.


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analogdevicesinc.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Connect P0.8(UART0_RX) to P1.9(UART2_TX).

If using MAX32675 RevA:
-   Select TX0 on Headers JP6.
-	DE-select RX0 on Header JP5.

If using MAX32675 RevB:
-   Select TX0 on Headers JP3.
-	DE-select RX0 on Header JP4.

If using MAX32675 RevD:
-   Select TX0 on Headers JP6.
-	DE-select RX0 on Header JP5.

## Expected Output

The Console UART of the device will output these messages:

```
**************** UART Example ******************
This example sends data from one UART to another

The green LED1 will illuminate for successful transaction.
The red LED0 will illuminate if transaction failed.

Unplug the Jumper at (JP5 - RX0_EN) above the Port 0 headers.


Connect UART0 to UART2 for this example.
P0.8(UART0_RX) -> P1.9(UART2_TX)


-->UART Baud    : 115200 Hz

-->Test Length  : 256 bytes

-->UART Initialized

-->Data verified

-->Example Succeeded
```

The green LED1 will illuminate for successful transaction.
The red LED0 will illuminate if transaction failed.
