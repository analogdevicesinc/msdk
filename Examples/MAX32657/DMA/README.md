## Description

A simple DMA transaction is shown that copies a block of memory from one location to another.

A second more complex DMA transaction is then shown that chains two memory-to-memory transfers together.  In the second example a callback function is used to notify the application once both transfers have completed.


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analogdevicesinc.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections
-   Connect a USB cable between the PC and the J1 (PWR-OBD-UART0) connector.
-   Connect pins JP19 (OBD VCOM EN) RX and TX header.
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


