# Description

Bluetooth version 4.2 controller, accepts HCI commands via Serial Port.

# Usage

## LEDs

The red LED will indicate that an error assertion has occurred.  

The green LED indicates CPU activity. When the LED is on, the CPU is active, when the LED
is off, the CPU is in sleep mode.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the (USB/PWR - UART) connector.
-   Use an external USB-to-UART adapter to access HCI UART. Connect a USB cable between the PC or BLE Tester
    and USB side of the adapter. Connect UART side of the adapter to board TX,RX and GND header pins.
-   Optionally you can reconfigure the UART definitions in board.h to use the on-board USB to UART 
    adapter for the HCI UART.

## Trace Serial Port
When TRACE is enabled in the [project.mk](project.mk), the on-board USB-to-UART adapter can
be used to view the trace messages. Open a serial port terminal with
the following settings.

Baud            : 115200  
Char size       : 8  
Parity          : None  
Stop bits       : 1  
HW Flow Control : No  
SW Flow Control : No  

## HCI Serial Port
The HCI serial port is used for HCI communication with BLE controller. Require
external USB-to-UART adapter configured to the following settings.

Baud            : 115200  
Char size       : 8  
Parity          : None  
Stop bits       : 1
HW Flow Control : No
SW Flow Control : No

Commands can be send to Controller by BLE tester, or from PC as illustrated in
Tools/Bluetooth/BLE_hci.py. The script has built in help options to describe the usage.
