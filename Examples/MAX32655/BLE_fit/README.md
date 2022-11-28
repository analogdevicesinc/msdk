# Description

Bluetooth fitness device. Showcases heart rate, battery level, running speed and cadence.

# Usage

## LEDs

The red LED will indicate that an error assertion has occurred.  

The green LED indicates CPU activity. When the LED is on, the CPU is active, when the LED
is off, the CPU is in sleep mode.

## Setup

### Board Selection

Before building firmware you must select the correct value for _BOARD_  in "project.mk", e.g. "EvKit\_V1".

### Required Connections
If using the Standard EV Kit board (EvKit\_V1):
-   Connect a USB cable between the PC and the CN2 (USB/PWR - UART) connector.
-   Close jumpers JP7 (RX_EN) and JP8 (TX_EN).
-   Close jumpers JP5 (LED1 EN) and JP6 (LED2 EN).


### Serial Port
When TRACE is enabled in the project.mk, the on-board USB-to-UART adapter can be used to view
the trace messages as well as interact with the demo. Open a serial port terminal
with the following settings.

Baud:             115200
Char size:        8
Parity:           None
Stop bits:        1
HW Flow Control:  No


## Expected Output

On startup:
```
terminal: init
32kHz trimmed to 0xF
FitHandlerInit
Calculating database hash
Fit got evt 32
>>> Reset complete <<<
dmAdvActConfig: state: 0
dmAdvActSetData: state: 0
dmAdvActSetData: state: 0
dmAdvActStart: state: 0
HCI_LE_ADV_ENABLE_CMD_CMPL_CBACK_EVT: state: 3
dmDevPassEvtToDevPriv: event: 12, param: 33, advHandle: 0
Fit got evt 33
>>> Advertising started <<<
Database hash calculation complete
Fit got evt 21
Database hash updated
Fit got evt 52
```

When a connection has been made.
```
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
Fit got evt 39
>>> Connection opened <<<
Fit got evt 65
Fit got evt 87
attsProcMtuReq features 0x00
hciCoreTxAclStart len=7
connId=1 idleMask=0x0004
hciCoreTxAclStart len=24
connId=1 idleMask=0x0004
hciCoreTxAclStart len=24
hciCoreTxAclStart len=27
hciCoreTxAclStart len=13
connId=1 idleMask=0x0004
hciCoreTxAclStart len=26
connId=1 idleMask=0x0004
hciCoreTxAclStart len=14
attsCccMainCback connId=1 handle=19
hciCoreTxAclStart len=5
Fit got evt 20
ccc state ind value:2 handle:19 idx:0
hciCoreTxAclStart len=27
hciCoreTxAclStart len=27
hciCoreTxAclStart len=27
hciCoreTxAclStart len=27
hciCoreTxAclStart len=13
hciCoreTxAclStart len=27
hciCoreTxAclStart len=9
connId=1 idleMask=0x0004
hciCoreTxAclStart len=10
hciCoreTxAclStart len=13
hciCoreTxAclStart len=13
hciCoreTxAclStart len=26
hciCoreTxAclStart len=27
hciCoreTxAclStart len=5
hciCoreTxAclStart len=23
hciCoreTxAclStart len=23
hciCoreTxAclStart len=23
hciCoreTxAclStart len=11
hciCoreTxAclStart len=12
connId=1 idleMask=0x0004
hciCoreTxAclStart len=10
hciCoreTxAclStart len=6
connId=1 idleMask=0x0004
hciCoreTxAclStart len=10
connId=1 idleMask=0x0004
hciCoreTxAclStart len=9
attsCccMainCback connId=1 handle=35
hciCoreTxAclStart len=7
attsCccMainCback connId=1 handle=99
hciCoreTxAclStart len=7
attsCccMainCback connId=1 handle=1189
hciCoreTxAclStart len=7
hciCoreTxAclStart len=14
App got evt 16
```

### Commands
Type the desired command and parameter (if applicable) and press enter to execute the command.  

__help__  Displays the available commands.  
__echo (on|off)__ Enables or disables the input echo. On by default.  
__btn (ID) (s|m|l|x)__ Simulates button presses. Example: "btn 1 s" for a short button press on button 1.  
__pin (ConnID) (Pin Code)__ Used to input the pairing pin code.  

## Push buttons
Push buttons can be used to interact with the application.

__short__ press is less than 200 ms  
__medium__ press is between 200 and 500 ms  
__long__ press is between 500 and 1000 ms  
__extra long__ press is greater than 1000 ms  

### When disconnected
1. Button 1 short: Start/Stop scanning
2. Button 1 medium: Enter discoverable and bondable mode
3. Button 1 long: Clear all bonding info
4. Button 2 short: Toggle HRM flag for 8 and 16 bit values

### When connected
1. Button 1 short: Increment the heart rate
2. Button 1 long: Close the connection
3. Button 2 short: Decrement the heart rate
4. Button 2 medium: Toggle HRM DET flags
5. Button 2 long: Toggle HRM RR Interval feature flag


## Use ARM core and RISC-V core for splitted HCI
In the project.mk, changing USE_DUAL_CORE to 1 will enable using both ARM core and RISC-V core for the splitted HCI.

