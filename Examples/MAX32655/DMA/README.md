## Description

A simple DMA transaction is shown that copies a block of memory from one location to another.

A second more complex memory-to-memory DMA transaction is then shown that chains two transfers together.  A callback function is used to notify the application once both transfers have completed.

## Setup

##### Board Selection
Before building firmware you must select the correct value for _BOARD_  in "Makefile", either "EvKit\_V1" or "FTHR\_Apps\_P1", depending on the EV kit you are using to run the example.

##### Required Connections
If using the Standard EV Kit (EvKit\_V1):
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect pins JP4(RX_SEL) and JP5(TX_SEL) to RX0 and TX0  header.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

If using the featherboard (FTHR\_Apps\_P1):
-   Connect a USB cable between the PC and the J4 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the board's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
***** DMA Example *****
Transfer from memory to memory.
Data verified.

Transfer with Reload and Callback.
Data verified.

Example Succeeded
```


