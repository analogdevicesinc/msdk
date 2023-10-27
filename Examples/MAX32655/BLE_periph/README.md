# BLE_periph
Refer to the [BLE_periph](../../../Libraries/Cordio/docs/Applications/BLE_periph.md) documentation in the Cordio Library.

# NOTE: BLE_periph is a bare bones example with no security and is not guaranteed nor intended to work with Windows, IOS, Android, etc. The example is only designed to operate with other embedded devices with loose security requirements. For a more fully featured application, please checkout BLE_dat(c/s)

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Required Connections
* Connect a USB cable between the PC and the (USB/PWR - UART) connector. An anteanna or wired connection can be used if SMA is available on the board. 

### Project-Specific Build Notes
 Setting `TRACE=1` in [**project.mk**](project.mk) initializes the on-board USB-to-UART adapter for
viewing the trace messages and interacting with the application. Port uses settings:
    - Baud            : 115200  
    - Char size       : 8  
    - Parity          : None  
    - Stop bits       : 1  
    - HW Flow Control : No  
    - SW Flow Control : No  
