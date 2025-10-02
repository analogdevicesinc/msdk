## Description

A simple DMA transaction is shown that copies a block of memory from one location to another.

A second more complex DMA transaction is then shown that chains two memory-to-memory transfers together.  In the second example a callback function is used to notify the application once both transfers have completed.


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analogdevicesinc.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

* This project comes pre-configured for the MAX32655EVKIT.  See [Board Support Packages](https://analogdevicesinc.github.io/msdk/USERGUIDE/#board-support-packages) in the MSDK User Guide for instructions on changing the target board.

## Required Connections
If using the MAX32657EVKIT (EvKit\_V1):
-   Connect a USB cable between the PC and the J1 (PWR-OBD_UART0) connector.
-   Connect jumper to RX and TX pins at JP19 (OBD VCOM EN).
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


