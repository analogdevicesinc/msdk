## Description

A basic getting started program.

This version of Hello_World prints an incrementing count to the console UART and toggles a LED1 every 500 ms.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analogdevicesinc.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

Connect a MAXPICO or other supported Debug adapter to the SWD Connector.
-   Note: Debug adapters other than the MAXPICO may not route the UART signals to the SWD connector. On MAX32690FTHR and AD-APARD32690-SL boards, this may limit your ability to access to serial port.

If using the MAX32690EVKIT:
-   Connect a USB cable between the PC and the CN2 (USB/PWR - UART) connector.
-   Install JP7(RX_EN) and JP8(TX_EN) headers.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Close jumper JP5 (LED1 EN).
-   Close jumper JP6 (LED2 EN).

If using the MAX32690FTHR:
-   Connect a USB cable between the PC and the J5 (USB/PWR) connector.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

If using the AD-APARD32690-SL:
-   Connect a USB cable between the PC and the P10 (USB-C) connector.
-   Open a terminal application on the PC and connect to the MAXPICO's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
Hello World!
count : 0
count : 1
count : 2
count : 3
```


You will also observe LED1 blinking at a rate of 1Hz.
