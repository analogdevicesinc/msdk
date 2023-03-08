# Description

Bluetooth version 5.2 controller, accepts HCI commands via Serial Port.

# Usage

## LEDs

The red LED will indicate that an error assertion has occurred.  

The green LED indicates CPU activity. When the LED is on, the CPU is active, when the LED
is off, the CPU is in sleep mode.

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

HCI:
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
