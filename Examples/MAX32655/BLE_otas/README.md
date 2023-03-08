# Description

Bluetooth data server that advertises as "OTAS" and accepts connection requests.

The Wireless Data Exchange profile is used to transfer files from the client to the server.
A CRC32 value is used to check the integrity of the transferred file.

Refer to the [BLE_otac_otas](../../../Libraries/Cordio/docs/Applications/BLE_otac_otas.md) documentation in the Cordio Library.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

## Required Connections
* Connect a USB cable between the PC and the (USB/PWR - UART) connector.


### Project-Specific Build Notes
* The Bootloader application needs to be loaded prior to loading this application. This application
will run on top of the Bootloader. The [ota.ld](ota.ld) linker file included with this application must be used
to properly setup the memory sections to coincide with the Bootloader.
* Setting `TRACE=1` in **project.mk** initializes the on-board USB-to-UART adapter for
viewing the trace messages and interacting with the application. Port uses settings:
    - Baud            : 115200  
    - Char size       : 8  
    - Parity          : None  
    - Stop bits       : 1  
    - HW Flow Control : No  
    - SW Flow Control : No  
* Setting `SBT=1` in **project.mk** enables Secure Boot Tools for the project
* Setting `USE_INTERNAL_FLASH=1` in **project.mk** tells the Bootloader to look for firmware updates in internal flash
* Setting `USE_INTERNAL_FLASH=0` in **project.mk** tells the Bootloader to look for firmware updates in external flash
    - This is the default behavior
