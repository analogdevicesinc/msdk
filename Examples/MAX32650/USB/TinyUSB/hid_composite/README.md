## Description

The example demonstrates the use of USB composite HID driver class. After doing the required connections given below, run the program and a new composite
HID device appears in the device manager, with keyboard, joystick, mouse and consumer control support. Pressing the switch SW2 on the EV Kit will result
in the 'a' key being pressed by the keyboard, the mouse moving down and right, joystick button A, and volume down.


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analogdevicesinc.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

tusb_config.h by default has logging disabled via the CFG_TUSB_DEBUG definition.
For TinyUSB console logging, set the debug level to 1, 2, or 3, with a higher value
indicating more verbose logging.

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Connect a USB cable between the PC and the CN1 (USB/UART0) connector. Make sure JP11 (2-3) is connected to UART.
-   Default EV kit hardware configuration.

## Expected Output

By default, the console UART does not output any message.  With CFG_TUSB_DEBUG set
to '2', the Console UART of the device will output these messages:

```
USBD init on controller 0, Highspeed = 1024
sizeof(usbd_device_t) = 69
sizeof(dcd_event_t) = 12
sizeof(tu_fifo_t) = 12
sizeof(tu_edpt_stream_t) = 24
HID init
```
