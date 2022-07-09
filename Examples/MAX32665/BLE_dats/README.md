# Description

Bluetooth data server that advertises as "DATS" and accepts connection requests.

# Usage

## LEDs

The red LED will indicate that an error assertion has occurred.  

The green LED indicates CPU activity. When the LED is on, the CPU is active, when the LED
is off, the CPU is in sleep mode.

## Setup

### Board Selection

Before building firmware you must select the correct value for BOARD in Makefile, e.g. "EvKit_V1".

### Required Connections
-   Connect a USB cable between the PC and the (USB/PWR - UART) connector.

## Trace Serial Port
When TRACE is enabled in the Makefile, the on-board USB-to-UART adapter can
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
32kHz trimed to 0x17
DatsHandlerInit
Calculating database hash
Dats got evt 119
Dats got evt 32
>>> Reset complete <<<
Dats got evt 58
Database hash calculation complete
dmAdvActConfig: state: 0
dmAdvActSetData: state: 0
dmAdvActSetData: state: 0
dmAdvActStart: state: 0
Dats got evt 21
Database hash updated
dmDevPassEvtToDevPriv: event: 13, param: 1, advHandle: 0
HCI_LE_ADV_ENABLE_CMD_CMPL_CBACK_EVT: state: 3
dmDevPassEvtToDevPriv: event: 12, param: 33, advHandle: 0
Dats got evt 63
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
AttsCccInitTable connId=1
smpDbGetRecord: connId: 1 type: 0
smpDbAddDevice
SmpDbGetFailureCount: connId: 1 count: 0
smpDbGetRecord: connId: 1 type: 0
smpDbAddDevice
SmpDbGetPairingDisabledTime: connId: 1 period: 0 attemptMult: 0
Dats got evt 39
>>> Connection opened <<<
smpSmExecute event=5 state=0
hciCoreTxAclStart len=6
Dats got evt 65
smpSmExecute event=6 state=1
connId=1 idleMask=0x0001
Dats got evt 49
smpSmExecute event=3 state=2
hciCoreTxAclStart len=6
connId=1 idleMask=0x0000
Dats got evt 43
>>> Pairing failed <<<
hciCoreTxAclStart len=24
connId=1 idleMask=0x0004
hciCoreTxAclStart len=9
Dats got evt 87
hciCoreTxAclStart len=27
hciCoreTxAclStart len=13
hciCoreTxAclStart len=9
connId=1 idleMask=0x0004
hciCoreTxAclStart len=10
connId=1 idleMask=0x0004
hciCoreTxAclStart len=9
hciCoreTxAclStart len=27
hciCoreTxAclStart len=9
connId=1 idleMask=0x0004
hciCoreTxAclStart len=9
hciCoreTxAclStart len=27
hciCoreTxAclStart len=9
connId=1 idleMask=0x0004
hciCoreTxAclStart len=10
connId=1 idleMask=0x0004
hciCoreTxAclStart len=9
attsCccMainCback connId=1 handle=19
hciCoreTxAclStart len=5
connId 1 updated csf to 0x01
hciCoreTxAclStart len=5
attsCccMainCback connId=1 handle=515
hciCoreTxAclStart len=5
App got evt 16
App got evt 16
App got evt 16
```

When PHY change performed
```
> btn 2 s
Short Button 2 Press
2 MBit TX and RX PHY Requested
Dats got evt 70
DM_PHY_UPDATE_IND - RX: 2, TX: 2
Dats got evt 65
App got evt 16
App got evt 16
connId=1 idleMask=0x0000
> btn 2 s
Short Button 2 Press
LE Coded S2 TX and RX PHY Requested
Dats got evt 70
DM_PHY_UPDATE_IND - RX: 3, TX: 3
Dats got evt 65
App got evt 16
App got evt 16
dmConnUpdExecute event=113 state=3
dmConnSmExecute event=30 state=3
Dats got evt 41
> btn 2 s
Short Button 2 Press
LE Coded S8 TX and RX PHY Requested
Dats got evt 70
DM_PHY_UPDATE_IND - RX: 3, TX: 3
> btn 2 s
Short Button 2 Press
1 MBit TX and RX PHY Requested
Dats got evt 70
DM_PHY_UPDATE_IND - RX: 1, TX: 1
Dats got evt 65
```

When message received from peer
```
hello world
hciCoreTxAclStart len=18
Dats got evt 18
```

### Commands
Type the desired command and parameter (if applicable) and press enter to execute the command.  

__help__  Displays the available commands.  
__echo (on|off)__  Enables or disables the input echo. On by default.  
__btn (ID) (s|m|l|x)__  Simulates button presses. Example: "btn 1 s" for a short button press on button 1.  
__pin (ConnID) (Pin Code)__  Used to input the pairing pin code.  

## Push buttons
Push buttons can be used to interact with the application.

__short__ press is less than 200 ms  
__medium__ press is between 200 and 500 ms  
__long__ press is between 500 and 1000 ms  
__extra long__ press is greater than 1000 ms  

### When disconnected
1. Button 1 short press: Start advertising
2. Button 1 medium press: Enter bondable mode
3. Button 1 long press: Clear all bonding info
4. Button 1 extra long press: Show version info
5. Button 2 short press: Stop advertising

### When connected
1. Button 2 short press: Change PHY (1M-2M-Coded_S2-Coded_S8)
