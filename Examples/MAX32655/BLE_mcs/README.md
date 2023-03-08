## Description

Maxim custom Bluetooth profile and service that advertises as "MCS" and accepts
connection requests.


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

# Usage

## LEDs

Each LED is controlled by a characteristic. Write the characteristic to 1 to turn on the LED,
0 to turn off the LED. Boards without 3 LEDs have the characteristics combined. 

Red LED Characteristic   : 0x85FC567F31D9418587C6339924D1C5BE  
Green LED Characteristic : 0x85FC568031D9418587C6339924D1C5BE  
Blue LED Characteristic  : 0x85FC568131D9418587C6339924D1C5BE  

## Setup

### Board Selection

Before building firmware you must select the correct value for BOARD in project.mk, e.g. "EvKit_V1".

### Required Connections
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
McsAppHandlerInit
Calculating database hash
McsApp got evt 119
McsApp got evt 32
>>> Reset complete <<<
dmAdvActConfig: state: 0
dmAdvActSetData: state: 0
dmAdvActSetData: state: 0
dmAdvActStart: state: 0
HCI_LE_ADV_ENABLE_CMD_CMPL_CBACK_EVT: state: 3
dmDevPassEvtToDevPriv: event: 12, param: 33, advHandle: 0
McsApp got evt 33
>>> Advertising started <<<
Database hash calculation complete
McsApp got evt 21
Database hash updated
```

When a connection has been made.
```
HCI_LE_ADV_ENABLE_CMD_CMPL_CBACK_EVT: state: 5
dmDevPassEvtToDevPriv: event: 13, param: 34, advHandle: 0
McsApp got evt 34
>>> Advertising stopped <<<
dmAdvActConfig: state: 0
dmAdvActStart: state: 0
HCI_LE_ADV_ENABLE_CMD_CMPL_CBACK_EVT: state: 3
dmDevPassEvtToDevPriv: event: 12, param: 33, advHandle: 0
McsApp got evt 33
>>> Advertising started <<<
dmConnIdByBdAddr not found
dmConnCcbAlloc 1
dmConnSmExecute event=28 state=0
dmAdvConnected: state: 1
dmDevPassEvtToDevPriv: event: 13, param: 34, advHandle: 0
AttsCccInitTable connId=1
smpDbGetRecord: connId: 1 type: 1
smpDbAddDevice
SmpDbGetFailureCount: connId: 1 count: 0
smpDbGetRecord: connId: 1 type: 1
smpDbAddDevice
SmpDbGetPairingDisabledTime: connId: 1 period: 0 attemptMult: 0
McsApp got evt 39
>>> Connection opened <<<
McsApp got evt 65
McsApp got evt 87
attsProcMtuReq features 0x00
hciCoreTxAclStart len=7
McsApp got evt 22
connId=1 idleMask=0x0004
hciCoreTxAclStart len=18
connId=1 idleMask=0x0004
hciCoreTxAclStart len=26
hciCoreTxAclStart len=34
connId=1 idleMask=0x0004
hciCoreTxAclStart len=34
attsCccMainCback connId=1 handle=19
hciCoreTxAclStart len=5
McsApp got evt 20
ccc state ind value:2 handle:19 idx:0
hciCoreTxAclStart len=90
hciCoreTxAclStart len=9
connId=1 idleMask=0x0004
hciCoreTxAclStart len=10
connId=1 idleMask=0x0004
hciCoreTxAclStart len=9
attsCccMainCback connId=1 handle=5379
hciCoreTxAclStart len=7
hciCoreTxAclStart len=14
```

When push buttons pressed
```
btn 1 s
Short Button 1 Press
mcsAppBtnCback; 2
> hciCoreTxAclStart len=8
McsApp got evt 18
btn 1 m
Medium Button 1 Press
mcsAppBtnCback; 3
> hciCoreTxAclStart len=8
McsApp got evt 18
btn 1 l
Long Button 1 Press
mcsAppBtnCback; 4
> hciCoreTxAclStart len=8
McsApp got evt 18
btn 1 x
XL Button 1 Press
mcsAppBtnCback; 5
> hciCoreTxAclStart len=8
McsApp got evt 18
btn 2 s
Short Button 2 Press
mcsAppBtnCback; 7
> hciCoreTxAclStart len=8
McsApp got evt 18
btn 2 m
Medium Button 2 Press
mcsAppBtnCback; 8
> hciCoreTxAclStart len=8
McsApp got evt 18
btn 2 l
Long Button 2 Press
mcsAppBtnCback; 9
> hciCoreTxAclStart len=8
McsApp got evt 18
btn 2 x
XL Button 2 Press
mcsAppBtnCback; 10
> hciCoreTxAclStart len=8
McsApp got evt 18
```


### Commands
Type the desired command and parameter (if applicable) and press enter to execute the command.  

__help__  Displays the available commands.  
__echo (on|off)__  Enables or disables the input echo. On by default.  
__btn (ID) (s|m|l|x)__  Simulates button presses. Example: "btn 1 s" for a short button press on button 1.  
__pin (ConnID) (Pin Code)__  Used to input the pairing pin code.  

## Push buttons
Push buttons can be used to set McsButton characteristic.
The value can be read via BLE.

__short__ press is less than 200 ms  
__medium__ press is between 200 and 500 ms  
__long__ press is between 500 and 1000 ms  
__extra long__ press is greater than 1000 ms  

### When connected
1. Button 1 short press:      set McsButton = 0x02
2. Button 1 medium press:     set McsButton = 0x03
3. Button 1 long press:       set McsButton = 0x04
4. Button 1 extra long press: set McsButton = 0x05
5. Button 2 short press:      set McsButton = 0x07
6. Button 2 medium press:     set McsButton = 0x08
7. Button 2 long press:       set McsButton = 0x09
8. Button 2 extra long press: set McsButton = 0x0A
