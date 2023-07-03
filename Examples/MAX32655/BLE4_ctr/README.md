# BLE4_ctr

Bluetooth version 4.2 controller, accepts HCI commands via Serial Port.

Refer to the [BLE4_ctr](../../../Libraries/Cordio/docs/Applications/BLE4_ctr.md) documentation in the Cordio Library.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Required Connections

-   Connect a USB cable between the PC and the (USB/PWR - UART) connector.
-   Use an external USB-to-UART adapter to access HCI UART. Connect a USB cable between the PC or BLE Tester
    and USB side of the adapter. Connect UART side of the adapter to board TX,RX and GND header pins.
-   Optionally you can reconfigure the UART definitions in board.h to use the on-board USB to UART 
    adapter for the HCI UART.

### Project-Specific Build Notes
* Setting `TRACE=1` in [**project.mk**](project.mk) initializes the on-board USB-to-UART adapter for
viewing the trace messages and interacting with the application. Port uses settings:
    - Baud            : 115200  
    - Char size       : 8  
    - Parity          : None  
    - Stop bits       : 1  
    - HW Flow Control : No  
    - SW Flow Control : No  
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