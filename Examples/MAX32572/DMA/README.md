## Description

A simple DMA transaction is shown that copies a block of memory from one location to another.

A second more complex memory-to-memory DMA transaction is then shown that chains two transfers together.  A callback function is used to notify the application once both transfers have completed.


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the J4 (USB to UART0) connector.
-   Install P1.8 (UART0 RX EN) and P1.9 (UART0 TX EN) on header JP8.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Close jumper JP6 (LED0 EN).
-   Close jumper JP7 (LED1 EN).

## Expected Output

The Console UART of the device will output these messages:

```
********** DMA Example **********
Transfer from memory to memory.
Starting transfer
Data verified.

Transfer with Reload and Callback.
Data verified.

Example Succeeded
```


