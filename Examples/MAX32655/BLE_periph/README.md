# BLE_periph
Refer to the [BLE_periph](../../../Libraries/Cordio/docs/Applications/BLE_periph.md) documentation in the Cordio Library.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Required Connections
* Connect a USB cable between the PC and the (USB/PWR - UART) connector. An anteanna or wired connection can be used if SMA is available on the board. 

### Project-Specific Build Notes
 Setting `TRACE=1` in **project.mk** initializes the on-board USB-to-UART adapter for
viewing the trace messages and interacting with the application. Port uses settings:
    - Baud            : 115200  
    - Char size       : 8  
    - Parity          : None  
    - Stop bits       : 1  
    - HW Flow Control : No  
    - SW Flow Control : No  