## Description

A simple DMA transaction is shown that copies a block of memory from one location to another.

A second more complex memory-to-memory DMA transaction is then shown that chains two transfers together.  A callback function is used to notify the application once both transfers have completed.

## Setup
-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Install pins JP7(RX_EN) and JP8(TX_EN) headers.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

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


