# BLE_LR_Central

This project is a demo for long range scanning. The coded PHY (s=8) is used. It works with the BLE_LR_Peripheral project.  

The project is modified from the BLE_datc.  

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.  

### Required Connections
* Connect a USB cable between the PC and the (USB/PWR - UART) connector.  

### Project-Specific Build Notes
* Setting `TRACE=1` in [**project.mk**](project.mk) initializes the on-board USB-to-UART adapter for
viewing the trace messages and interacting with the application. Port uses settings:
    - Baud            : 115200  
    - Char size       : 8  
    - Parity          : None  
    - Stop bits       : 1  
    - HW Flow Control : No  
    - SW Flow Control : No  
* Upon server discovery the user will be prompted to enter a passkey.
An arbitrary pin can be entered in the format `pin (connId) passkey` 
    * Example: `pin 1 123456`
    * The server is expected to enter the same connId and passkey to establish a secure connection
    and share bonding information
    * ***Note***: *Either the client or server can enter the passkey first. The peer device must then match.*

### Logs
```
terminal: init␍␊  
==========================================␍␊  
Long distance scanner demo (CODED PHY S=8)␍␊  
BT_VER=9␍␊  
==========================================␍␊  
DatcHandlerInit␍␊  
MAC Addr: 00:18:80:00:44:B1␍␊  
>>> Reset complete <<<␍␊  
Database hash updated␍␊  
dmDevPassEvtToDevPriv: event: 12, param: 74, advHandle: 0␍␊  
␍␊  
00:18:80:00:a2:22 BF 02 03 ... 30 31 BF␍␊  
00:18:80:00:a2:22 C0 02 03 ... 30 31 C0␍␊  
00:18:80:00:a2:22 C1 02 03 ... 30 31 C1␍␊  
00:18:80:00:a2:22 C2 02 03 ... 30 31 C2␍␊  
00:18:80:00:a2:22 C3 02 03 ... 30 31 C3␍␊  
00:18:80:00:a2:22 C4 02 03 ... 30 31 C4␍␊  
00:18:80:00:a2:22 C5 02 03 ... 30 31 C5␍␊  
00:18:80:00:a2:22 C6 02 03 ... 30 31 C6␍␊  
00:18:80:00:a2:22 C7 02 03 ... 30 31 C7␍␊  
00:18:80:00:a2:22 C8 02 03 ... 30 31 C8␍␊  
00:18:80:00:a2:22 C9 02 03 ... 30 31 C9␍␊  
00:18:80:00:a2:22 CA 02 03 ... 30 31 CA␍␊  
00:18:80:00:a2:22 CB 02 03 ... 30 31 CB␍␊  
00:18:80:00:a2:22 CC 02 03 ... 30 31 CC␍␊  
00:18:80:00:a2:22 CD 02 03 ... 30 31 CD␍␊  
00:18:80:00:a2:22 CE 02 03 ... 30 31 CE␍␊  
00:18:80:00:a2:22 CF 02 03 ... 30 31 CF␍␊  
00:18:80:00:a2:22 D0 02 03 ... 30 31 D0␍␊  
00:18:80:00:a2:22 D1 02 03 ... 30 31 D1␍␊  
00:18:80:00:a2:22 D2 02 03 ... 30 31 D2␍␊  
00:18:80:00:a2:22 D3 02 03 ... 30 31 D3␍␊  
00:18:80:00:a2:22 D4 02 03 ... 30 31 D4␍␊  
00:18:80:00:a2:22 D5 02 03 ... 30 31 D5␍␊  
00:18:80:00:a2:22 D6 02 03 ... 30 31 D6␍␊  
00:18:80:00:a2:22 D7 02 03 ... 30 31 D7␍␊  
00:18:80:00:a2:22 D8 02 03 ... 30 31 D8␍␊  
00:18:80:00:a2:22 D9 02 03 ... 30 31 D9␍␊  
...  
```