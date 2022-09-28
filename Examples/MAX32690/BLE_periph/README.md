## Description

Bluetooth peripheral that advertises as "Periph" and accepts connection requests.


# Usage

## LEDs

The red LED will indicate that an error assertion has occurred.  

The green LED indicates CPU activity. When the LED is on, the CPU is active, when the LED
is off, the CPU is in sleep mode.

## Setup

### Board Selection

Before building firmware you must select the correct value for BOARD in [project.mk](project.mk), e.g. "EvKit_V1".

### Required Connections
-   Connect a USB cable between the PC and the (USB/PWR - UART) connector.

## Trace Serial Port
When TRACE is enabled in the [project.mk](project.mk), the on-board USB-to-UART adapter can
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
PeriphHandlerInit
Calculating database hash
Periph got evt 119
Periph got evt 32
>>> Reset complete <<<
dmAdvActConfig: state: 0
dmAdvActSetData: state: 0
dmAdvActStart: state: 0
HCI_LE_ADV_ENABLE_CMD_CMPL_CBACK_EVT: state: 3
dmDevPassEvtToDevPriv: event: 12, param: 33, advHandle: 0
Periph got evt 33
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
smpDbGetRecord: connId: 1 type: 1
smpDbAddDevice
SmpDbGetFailureCount: connId: 1 count: 0
smpDbGetRecord: connId: 1 type: 1
smpDbAddDevice
SmpDbGetPairingDisabledTime: connId: 1 period: 0 attemptMult: 0
Periph got evt 39
>>> Connection opened <<<
Periph got evt 65
Periph got evt 87
attsProcMtuReq features 0x00
hciCoreTxAclStart len=7
Periph got evt 22
connId=1 idleMask=0x0004
hciCoreTxAclStart len=18
connId=1 idleMask=0x0004
hciCoreTxAclStart len=26
hciCoreTxAclStart len=34                                                                                              
connId=1 idleMask=0x0004                                                                                              
hciCoreTxAclStart len=34                                                                                              
attsCccMainCback connId=1 handle=19                                                                                   
hciCoreTxAclStart len=5                                                                                               
hciCoreTxAclStart len=27                                                                                              
hciCoreTxAclStart len=9                                                                                               
connId=1 idleMask=0x0004                                                                                              
hciCoreTxAclStart len=10                                                                                              
connId=1 idleMask=0x0004                                                                                              
hciCoreTxAclStart len=9                                                                                               
attsCccMainCback connId=1 handle=515                                                                                  
hciCoreTxAclStart len=7                                                                                               
hciCoreTxAclStart len=14
```


### Commands
Type the desired command and parameter (if applicable) and press enter to execute the command.  

__help__  Displays the available commands.  
__echo (on|off)__  Enables or disables the input echo. On by default.  
__btn (ID) (s|m|l|x)__  Simulates button presses. Example: "btn 1 s" for a short button press on button 1.  
__pin (ConnID) (Pin Code)__  Used to input the pairing pin code.  

## Push buttons
Push buttons are not implemented in this example.

