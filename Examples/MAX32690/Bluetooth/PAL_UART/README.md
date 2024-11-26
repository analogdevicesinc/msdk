# Description

A minimal program for testing serial echoing by utilizing Cordio's PAL UART implementation.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analogdevicesinc.github.io/msdk/USERGUIDE/)**.

### Required Connections

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

### Project-Specific Build Notes
* Setting `TRACE=1` in [**project.mk**](project.mk) initializes the on-board USB-to-UART adapter for
viewing the trace messages and interacting with the application. Port uses settings:
    - Baud            : 115200
    - Char size       : 8
    - Parity          : None
    - Stop bits       : 1
    - HW Flow Control : No
    - SW Flow Control : No
* Setting `SBT=1` in [**project.mk**](project.mk) enables Secure Boot Tools for the project
* The HCI serial port is used for HCI communication with BLE controller. Require
external USB-to-UART adapter configured to the following settings:
    - Baud            : 115200
    - Char size       : 8
    - Parity          : None
    - Stop bits       : 1
    - HW Flow Control : No
    - SW Flow Control : No
* HCI commands can be send to Controller by BLE tester, or from PC as illustrated in
Tools/Bluetooth/BLE_hci.py. The script has built in help options to describe the usage.
