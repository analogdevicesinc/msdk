# BLE_otac

Bluetooth data client that scans for and connects to advertisers with the name of "OTAS".

The Wireless Data Exchange profile is used to transfer files from the client to the server. 
A CRC32 value is used to check the integrity of the transferred file.

Refer to the [BLE_otac_otas](../../../Libraries/Cordio/docs/Applications/BLE_otac_otas.md) documentation in the Cordio Library.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Required Connections
* Connect a USB cable between the PC and the (USB/PWR - UART) connector.

### Project-Specific Build Notes
* The application to use for the firmware update can be selected using the `FW_UPDATE_DIR` option in [project.mk](project.mk).  Whichever application is selected by this option must be configured to run from the appropriate memory section, as defined by the Bootloader (see the `Bootloader` example for more details).
* Setting `TRACE=1` in **project.mk** initializes the on-board USB-to-UART adapter for
viewing the trace messages and interacting with the application. Port uses settings:
    - Baud            : 115200  
    - Char size       : 8  
    - Parity          : None  
    - Stop bits       : 1  
    - HW Flow Control : No  
    - SW Flow Control : No  
