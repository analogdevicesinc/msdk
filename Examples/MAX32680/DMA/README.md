## Description

A simple DMA transaction is shown that copies a block of memory from one location to another.

A second more complex memory-to-memory DMA transaction is then shown that chains two transfers together.  A callback function is used to notify the application once both transfers have completed.


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Setup

##### Required Connections
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect pins JP22(RX_SEL) and JP23(TX_SEL) to UART1 header.
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


