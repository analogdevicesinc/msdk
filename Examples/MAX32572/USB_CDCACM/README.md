## Description

The example demonstartes the use of USB CDC-ACM driver class. After doing the required connections given below, run the program and a new serial terminal (COM port) appears in the device manager. Open a terminal application on the PC at 9600 baud rate, and echo the characters on the terminal.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

(TBD - until we receive EV Kits).

## Expected Output

The Console UART of the device will output these messages:

```
***** MAX32572 USB CDC-ACM Example *****
Waiting for VBUS...
VBUS Connect
Suspended
Bus Reset
Bus Reset Done: High speed
Enumeration complete...
```
