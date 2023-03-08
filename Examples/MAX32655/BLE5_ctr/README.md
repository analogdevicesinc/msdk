# Description

Bluetooth version 5.2 controller, accepts HCI commands via Serial Port.

# Usage

## LEDs

The red LED will indicate that an error assertion has occurred.  

The green LED indicates CPU activity. When the LED is on, the CPU is active, when the LED
is off, the CPU is in sleep mode.

## Setup

### Board Selection

Before building firmware you must select the correct value for BOARD in project.mk, e.g. "EvKit_V1".

### Required Connections
-   Connect a USB cable between the PC and the (USB/PWR - UART) connector.
-   Use an external USB-to-UART adapter to access HCI UART. Connect a USB cable between the PC or BLE Tester
    and USB side of the adapter. Connect UART side of the adapter to board TX,RX and GND header pins.
-   Optionally you can reconfigure the UART definitions in board.h to use the on-board USB to UART 
    adapter for the HCI UART.

## Trace Serial Port
When TRACE is enabled in the project.mk, the on-board USB-to-UART adapter can
be used to view the trace messages. Open a serial port terminal with
the following settings.

Baud            : 115200  
Char size       : 8  
Parity          : None  
Stop bits       : 1  
HW Flow Control : No  
SW Flow Control : No  

### Expected Output

On startup:
```
    RAM: 4 x 752 bytes -- connection context
    RAM: 16 x 719 bytes -- Tx buffer descriptors
    RAM: 2 x 2296 bytes -- advertising set context
    RAM: 2 x 704 bytes -- CIS context
    RAM: 2 x 124 bytes -- CIG context
    RAM: 2 x 208 bytes -- BIS context
    RAM: 2 x 584 bytes -- BIG context
    RAM: 8 x 120 bytes -- Tx buffer descriptors
LlHandlerInit: LL initialization completed
    opModeFlags = 0x005F5C40
### LlApi ###  LlSetBdAddr
Static BDA[5:3]=00:18:80
       BDA[2:0]=F9:69:42
### LlApi ###  LlSetAdvTxPower, advTxPwr=0
```

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

### Expected Output

```
python3 BLE_hci.py /dev/ttyUSB0
Bluetooth Low Energy HCI tool
Serial port: /dev/ttyUSB0
8N1 115200

>>> reset
2022-03-08 09:03:26.602167 > 01030C00
2022-03-08 09:03:26.616613 < 040E0401030C00
>>> 

```
This shows HCI_Reset command sent to controller along with response event
from controller. If command successful then the terminal shows
the following trace messages.

```
### LlApi ###  LlReset
lctrMstScanExecuteSm: state=0, event=0
lctrMstInitExecuteSm: state=0, event=0
lctrSlvAdvExecuteSm: state=0, event=0
lctrMstExtScanExecuteSm: phy=0, state=0, event=0
lctrMstExtScanExecuteCommonSm: numScanEnabled=0, scanMode=0, event=RESET
lctrMstExtInitExecuteSm: state=0, event=0
lctrMstExtInitExecuteCommonSm: scanMode=0, event=RESET
lctrMstCreateSyncExecuteSm: state=0, event=0
lctrMstTransferSyncExecuteSm: state=0, event=0                                                                         
lctrMstPerScanExecuteSm: state=0, event=0                                                                              
lctrMstPerScanExecuteSm: state=0, event=0                                                                              
lctrMstPerScanExecuteSm: state=0, event=0                                                                              
lctrMstPerScanExecuteSm: state=0, event=0                                                                              
lctrMstPerScanExecuteSm: state=0, event=0                                                                              
lctrMstPerScanExecuteSm: state=0, event=0                                                                              
lctrSlvAcadDisp: handle=4, event=0                                                                                     
### LlEvent ###  LL_RESET_CNF, status=LL_SUCCESS                                                                       
```


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

