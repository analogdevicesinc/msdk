## Description

To demonstrate the use of the UART peripheral with DMA, data is sent between one UART port on the MAX32690.


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analogdevicesinc.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Connect P2.11 (UART 0 RX) and P2.12 (UART 0 TX).

## Expected Output

The Console UART of the device will output these messages:

```
**************** UART Example ******************
This example sends data from one UART to another

Connect RX(Px.xx) of UARTx and TX(Px.xx) of UARTx.

To indicate a successful UART transfer, LED1 will illuminate.

Push SW2 to continue

UART Baud       : 115200 Hz
Test Length     : 512 bytes

-->UART Initialized

-->Data verified

-->Example Succeeded

```

or

```
**************** UART Example ******************
This example sends data from one UART to another

Connect RX(Px.xx) of UARTx and TX(Px.xx) of UARTx.

To indicate a successful UART transfer, LED1 will illuminate.

Push SW2 to continue

UART Baud       : 115200 Hz
Test Length     : 512 bytes

Acquired DMA channel 0 for RX transaction
Acquired DMA channel 1 for TX transaction

-->UART Initialized

-->Data verified

-->Example Succeeded

```