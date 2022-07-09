## Description

This example demonstrates a couple of simple DMA transfers in which data is moved from one memory location to another. (Other examples show peripheral to memory (or vice versa) transfers.)

The first example does just as described above, a single DMA transaction that copies a block of memory from one location to another. The second, more complex memory-to-memory DMA transaction chains two transfers together.  A callback function is used to notify the application when each transaction has completed.

## Required Connections
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Install headers JP7(RX\_EN) and JP8(TX\_EN).
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

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


