## Description

The example demonstrates the use of USB Mass Storage driver class with TinyUSB.
The storage driver is connected to the External Flash device on the AD-APARD32690-SL board.


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analogdevicesinc.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

tusb_config.h by default has logging disabled via the CFG_TUSB_DEBUG definition.
For TinyUSB console logging, set the debug level to 1, 2, or 3, with a higher value
indicating more verbose logging.

## Required Connections


If using the AD-APARD32690-SL:
-   Connect a USB cable between the PC and the P10 (USB-C) connector.
-   Connect a MAXPICO Debug adapter to P9 (SWD Connector)
-   Open a terminal application on the PC and connect to the MAXPICO's console UART at 115200, 8-N-1.

## Expected Output

By default, the console UART does not output any message.  With CFG_TUSB_DEBUG set
to '2', the Console UART of the device will output these messages:

```
USBD init on controller 0, Highspeed = 1024
sizeof(usbd_device_t) = 69
sizeof(dcd_event_t) = 12
sizeof(tu_fifo_t) = 12
sizeof(tu_edpt_stream_t) = 24
MSC init
```
