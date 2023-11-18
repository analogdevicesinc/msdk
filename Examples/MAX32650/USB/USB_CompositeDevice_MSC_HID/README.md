## Description

The example demonstartes the use of USB composite device with Mass Storage driver class and HID driver class. After doing the required connections given below, run the program and two new devices appear in the device manager, a Portable device and a HID keyboard device. The portable device can be read and written to. Open any text editor in the PC, pressing the switch SW1 on the evkit wil print out 'Maxin Integrated' in the editor one letter at a time.


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Connect a USB cable between the PC and the CN1 (USB/UART0) connector. Make sure JP11 (2-3) is connected to UART.
-   Default EV kit hardware configuration.

## Expected Output

The Console UART of the device will output these messages:

```
***** MAX32650 USB Composite Device (Keyboard and Mass Storage) Example *****
Waiting for VBUS...
VBUS Connect
Suspended
Bus Reset
Bus Reset
Enumeration complete. Press SW2 to send character.
```
