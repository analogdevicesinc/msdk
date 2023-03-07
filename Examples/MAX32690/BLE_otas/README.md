# Description

Bluetooth data server that advertises as "OTAS" and accepts connection requests.

The Wireless Data Exchange profile is used to transfer files from the client to the server.
A CRC32 value is used to check the integrity of the transferred file.

# Usage

## LEDs

The red LED will indicate that an error assertion has occurred.  

The green LED indicates CPU activity. When the LED is on, the CPU is active, when the LED
is off, the CPU is in sleep mode.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

* The Bootloader application needs to be loaded prior to loading this application. This application
will run on top of the Bootloader. The [ota.ld] linker file included with this application must be used
to properly setup the memory sections to coincide with the Bootloader.

## Required Connections

-   Connect a USB cable between the PC and the (USB/PWR - UART) connector.

## Trace Serial Port
When TRACE is enabled in the project.mk, the on-board USB-to-UART adapter can
be used to view the trace messages and interact with the application. Open a serial port terminal with
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
terminal: init
32kHz trimmed to 0xE
DatsHandlerInit
WDXS: WdxsHandlerInit
FW_VERSION: 1.0
File Hdl: 1
Dats got evt 32
>>> Reset complete <<<
dmAdvActConfig: state: 0
dmAdvActSetData: state: 0
dmAdvActSetData: state: 0
dmAdvActStart: state: 0
HCI_LE_ADV_ENABLE_CMD_CMPL_CBACK_EVT: state: 3
dmDevPassEvtToDevPriv: event: 12, param: 33, advHandle: 0
Dats got evt 33
>>> Advertising started <<<

```

When a connection has been made.
```
dmConnIdByBdAddr not found
dmConnCcbAlloc 1
dmConnSmExecute event=28 state=0
dmAdvConnected: state: 1
dmDevPassEvtToDevPriv: event: 13, param: 34, advHandle: 0
smpDbGetRecord: connId: 1 type: 0
smpDbAddDevice
SmpDbGetFailureCount: connId: 1 count: 0
smpDbGetRecord: connId: 1 type: 0
smpDbAddDevice
SmpDbGetPairingDisabledTime: connId: 1 period: 0 attemptMult: 0
Dats got evt 39
>>> Connection opened <<<
Dats got evt 65
Dats got evt 87
connId=1 idleMask=0x0004
connId=1 idleMask=0x0004
connId=1 idleMask=0x0004
connId=1 idleMask=0x0004
connId=1 idleMask=0x0004
connId=1 idleMask=0x0004
connId=1 idleMask=0x0004
connId=1 idleMask=0x0004
connId=1 idleMask=0x0004
connId=1 idleMask=0x0004
connId=1 idleMask=0x0004


```

Upon reception of File Discovery `btn 2 s` command
```
WDXS: FTC Write: len=12
WDXS: FTC Write: op=1 handle=0
WDXS: FTC GetReq handle=0 len=9
WDXS: FTC SendRsp op=2 handle=0 status=0
WDXS: Task Handler Evt=1
WDXS: FTC Send
WDXS: AttHook handle=581 event=18
WDXS: Task Handler Evt=1
WDXS: FTC SendRsp op=10 handle=0 status=0
WDXS: AttHook handle=584 event=18
WDXS: Task Handler Evt=1
WDXS: FTC Send
WDXS: AttHook handle=581 event=18
WDXS: Task Handler Evt=1

```

Upon reception of Initiate Transfer `btn 2 m` command
```
WDXS: FTC Write: len=16
WDXS: FTC Write: op=3 handle=1
WDXS: FTC PutReq handle=1 offset=0, len=209768
>>> Initiating erase of 26 pages of internal flash <<<
WDXS: FTC PutReq handle=1 status=0
>>> Erasing address 0x10302000 in internal flash <<<
>>> Erasing address 0x10304000 in internal flash <<<
>>> Erasing address 0x10306000 in internal flash <<<
>>> Erasing address 0x10308000 in internal flash <<<
>>> Erasing address 0x1030A000 in internal flash <<<
...
...
>>> Internal flash erase complete <<<
WDXS: FTC SendRsp op=4 handle=1 status=0
WDXS: Task Handler Evt=1
WDXS: FTC Send
WDXS: AttHook handle=581 event=18
WDXS: Task Handler Evt=1
Int. Flash: Wrote 224 bytes @ 0x10300000
Int. Flash: Wrote 224 bytes @ 0x103000E0
Int. Flash: Wrote 224 bytes @ 0x103001C0
Int. Flash: Wrote 224 bytes @ 0x103002A0
Int. Flash: Wrote 224 bytes @ 0x10300380

```

Upon reception of Verify transfer `btn 2 l` command 
```
DXS: FTC Write: len=3
WDXS: FTC Write: op=7 handle=1
WDXS: FTC VerifyReq: handle=1
CRC start addr: 0x10300000 Len: 0x00033368
CRC From File : 0x8871C78B
CRC Calculated: 0x8871C78B
WDXS: FTC SendRsp op=8 handle=1 status=0
WDXS: Task Handler Evt=1
WDXS: FTC Send
WDXS: AttHook handle=581 event=18
WDXS: Task Handler Evt=1

```

Upon reception of Request Reset `btn 2 x` command 
```
dmConnSmExecute event=25 state=3
dmConnSmExecute event=29 state=4
dmConnCcbDealloc 1
smpDbGetRecord: connId: 1 type: 0
smpDbAddDevice
SmpDbSetFailureCount: connId: 1 count: 0
smpSmExecute event=10 state=0
Dats got evt 40
Reseting!

```

On successful update the device resets and connects once again.


### Commands
Type the desired command and parameter (if applicable) and press enter to execute the command.  

__help__  Displays the available commands.  
__echo__ (on|off) Enables or disables the input echo. On by default.  
__btn__ (ID) (s|m|l|x) Simulates button presses. Example: "btn 1 s" for a short button press on button 1.  
__pin__ (ConnID) (Pin Code) Used to input the pairing pin code.  

## Push buttons
Push buttons can be used to interact with the application.

__short__ press is less than 200 ms  
__medium__ press is between 200 and 500 ms  
__long__ press is between 500 and 1000 ms  
__extra long__ press is greater than 1000 ms  

### When connected
1. Button 2 short press: Toggle PHY 
2. Button 2 medium press : Display firmware version
### When disconnected
1. Button 1 short press: On/Off advertising
2. Button 1 medium press: Cycle through the connection index
3. Button 1 long press: Clear all bonding info
4. Button 1 extra long press: Add RPAO characteristic to GAP service -- needed only when DM Privacy enabled
5. Button 2 medium press : Display firmware version
6. Button 2 extra long press: Enable device privacy -- start generating local RPAs every 15 minutes
