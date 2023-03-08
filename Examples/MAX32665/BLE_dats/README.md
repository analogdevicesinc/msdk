# BLE_dats

Refer to the [BLE_datc_dats](../../../Libraries/Cordio/docs/Applications/BLE_datc_dats.md) documentation in the Cordio Library.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Required Connections
* Connect a USB cable between the PC and the (USB/PWR - UART) connector.

### Project-Specific Build Notes
* Setting `TRACE=1` in **project.mk** initializes the on-board USB-to-UART adapter for
viewing the trace messages and interacting with the application. Port uses settings:
    - Baud            : 115200  
    - Char size       : 8  
    - Parity          : None  
    - Stop bits       : 1  
    - HW Flow Control : No  
    - SW Flow Control : No  