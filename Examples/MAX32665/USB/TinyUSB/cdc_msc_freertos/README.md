## Description

The example demonstrates the use of USB composite device with Mass Storage driver class and CDC-ACM driver class using the TinyUSB stack
and FreeRTOS operating system. After doing the required connections given below, run the program and two new devices appear in the device manager, a Portable device and COM Port. The README.TXT file on the portable device can be read. Open a terminal application on the PC at 115200 baud rate, and echo the characters on the terminal.


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analogdevicesinc.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

tusb_config.h by default has logging disabled via the CFG_TUSB_DEBUG definition.
For TinyUSB console logging, set the debug level to 1, 2, or 3, with a higher value
indicating more verbose logging.

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR/UARTS) connector.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector. Make sure JP6 (2-3) is connected to UART.
-   Default EV kit hardware configuration.

## Expected Output

By default, the console UART does not output any message.  With CFG_TUSB_DEBUG set
to '2', the Console UART of the device will output these messages:

```
USBD init on controller 0, Highspeed = 1024
sizeof(usbd_device_t) = 69
sizeof(dcd_event_t) = 12
sizeof(tu_fifo_t) = 20
sizeof(tu_edpt_stream_t) = 112
CDC init
MSC init
USBD Bus Reset : High Speed
```
