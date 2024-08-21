## Description

The example demonstartes the use of USB composite device with Mass Storage driver class and CDC-SCM driver class. After doing the required connections given below, run the program and two new devices appear in the device manager, a Portable device and COM Port. The portable device can be read and written to. Open a terminal application on the PC at 115200 baud rate, and echo the characters on the terminal.


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR/UARTS) connector.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector. Make sure JP6 (2-3) is connected to UART.
-   Default EV kit hardware configuration.

## Expected Output

The Console UART of the device will output these messages:

```
****** USB Composite Device (CDCACM and Mass Storage) Example ******
Waiting for VBUS...
VBUS Connect
Suspended
Bus Reset
Bus Reset Done: High speed
Enumeration complete...
```
