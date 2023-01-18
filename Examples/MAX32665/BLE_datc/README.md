# Description

Bluetooth data client that scans for and connects to advertisers with the name of "DATS".

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
32kHz trimmed to 0xF
DatcHandlerInit
Calculating database hash
Datc got evt 32
>>> Reset complete <<<
Datc got evt 58
Database hash calculation complete
Datc got evt 21
Database hash updated
dmDevPassEvtToDevPriv: event: 13, param: 1, advHandle: 0
Datc got evt 63
Datc got evt 153
dmDevPassEvtToDevPriv: event: 12, param: 36, advHandle: 0
Datc got evt 36
>>> Scanning started <<<
```

When a scan report has been received:
```
Scan Report:                                                                  
  55:CF:67:1F:6F:27                                                           
Scan Report:                                                                  
  00:05:8B:44:12:02                                                           
  Name: Fit
```

When a connection has been made:
```
Scan Report:                                                                                                           
  00:18:80:55:F6:AE                                                                                                    
  Name: DATS                                                                                                           
dmDevPassEvtToDevPriv: event: 13, param: 37, advHandle: 0                                                              
Datc got evt 37                                                                                                        
Scan results: 10                                                                                                       
dmConnIdByBdAddr not found                                                                                             
dmConnCcbAlloc 1                                                                                                       
>>> Scanning stopped <<<                                                                                               
dmConnSmExecute event=24 state=0                                                                                       
dmDevPassEvtToDevPriv: event: 14, param: 0, advHandle: 0                                                               
dmConnSmExecute event=28 state=1                                                                                       
dmDevPassEvtToDevPriv: event: 14, param: 1, advHandle: 0                                                               
dmDevPassEvtToDevPriv: event: 12, param: 39, advHandle: 0                                                              
smpDbGetRecord: connId: 1 type: 0                                                                                      
smpDbAddDevice                                                                                                         
SmpDbGetFailureCount: connId: 1 count: 0                                                                               
smpDbGetRecord: connId: 1 type: 0                                                                                      
smpDbAddDevice                                                                                                         
SmpDbGetPairingDisabledTime: connId: 1 period: 0 attemptMult: 0                                                        
Datc got evt 39                                                                                                        
>>> Connection opened <<<                                                                                              
Datc got evt 65                                                                                                        
Datc got evt 22                                                                                                        
Datc got evt 87                                                                                                        
Datc got evt 4                                                                                                         
connId=1 idleMask=0x0008                                                                                               
Datc got evt 3                                                                                                         
AttcDiscServiceCmpl status 0x00                                                                                        
Datc got evt 4                                                                                                         
AttcDiscCharCmpl status 0x79                                                                                           
Datc got evt 4                                                                                                         
AttcDiscCharCmpl status 0x79                                                                                           
Datc got evt 2                                                                                                         
AttcDiscCharCmpl status 0x00                                                                                           
connId=1 idleMask=0x0008                                                                                               
Datc got evt 3                                                                                                         
AttcDiscServiceCmpl status 0x00                                                                                        
Datc got evt 4                                                                                                         
AttcDiscCharCmpl status 0x79                                                                                           
Datc got evt 4                                                                                                         
AttcDiscCharCmpl status 0x00                                                                                           
connId=1 idleMask=0x0008                                                                                               
Datc got evt 3                                                                                                         
AttcDiscServiceCmpl status 0x00                                                                                        
Datc got evt 4                                                                                                         
AttcDiscCharCmpl status 0x79                                                                                           
Datc got evt 4                                                                                                         
AttcDiscCharCmpl status 0x79                                                                                           
Datc got evt 2                                                                                                         
AttcDiscCharCmpl status 0x79                                                                                           
Datc got evt 2                                                                                                         
AttcDiscCharCmpl status 0x00                                                                                           
connId=1 idleMask=0x0000                                                                                               
AppDiscComplete connId:1 status:0x04                                                                                   
connId=1 idleMask=0x0008                                                                                               
Datc got evt 9                                                                                                         
AttcDiscConfigCmpl status 0x79                                                                                         
Datc got evt 9                                                                                                         
AttcDiscConfigCmpl status 0x79                                                                                         
Datc got evt 9                                                                                                         
AttcDiscConfigCmpl status 0x00                                                                                         
connId=1 idleMask=0x0000                                                                                               
AppDiscComplete connId:1 status:0x08                                                                                   
```

Simple message passing to peer:
```
btn 2 l
Long Button 2 Press
> Datc got evt 10
Datc got evt 13
hello back
```

On PHY change request:
```
btn 1 x
XL Button 1 Press
2 MBit TX and RX PHY Requested
> Datc got evt 70
DM_PHY_UPDATE_IND - RX: 2, TX: 2
Datc got evt 65
btn 1 x
XL Button 1 Press
LE Coded S2 TX and RX PHY Requested
> Datc got evt 70
DM_PHY_UPDATE_IND - RX: 3, TX: 3
Datc got evt 65
btn 1 x
XL Button 1 Press
LE Coded S8 TX and RX PHY Requested
> Datc got evt 70
DM_PHY_UPDATE_IND - RX: 3, TX: 3
btn 1 x
XL Button 1 Press
1 MBit TX and RX PHY Requested
> Datc got evt 70
DM_PHY_UPDATE_IND - RX: 1, TX: 1
Datc got evt 65
```

Data Transfer Speed test (1M PHY):
```
btn 2 x
XL Button 2 Press
Starting speed test
...
flowDisabled=1 handle=0
flowDisabled=0 handle=0
9520000 bits transferred in 18823249 us
505757 bps
```

Data Transfer Speed test (2M PHY):
```
btn 2 x
XL Button 2 Press
Starting speed test
...
flowDisabled=1 handle=0
flowDisabled=0 handle=0
9520000 bits transferred in 9413977 us
1011262 bps
```

### Commands
Type the desired command and parameter (if applicable) and press enter to execute the command.  

__help__  Displays the available commands.  
__echo (on|off)__ Enables or disables the input echo. On by default.  
__btn (ID) (s|m|l|x)__ Simulates button presses. Example: "btn 1 s" for a short button press on button 1.  
__pin (ConnID) (Pin Code)__ Used to input the pairing pin code.  

## Push buttons
Push buttons can be used to interact with the application.

__short__       : press is less than 200 ms  
__medium__      : press is between 200 and 500 ms  
__long__        : press is between 500 and 1000 ms  
__extra long__  : press is greater than 1000 ms  

### When disconnected
1. Button 1 short: Start/Stop scanning
2. Button 1 medium: Cycle through the connection index (select connection)
3. Button 1 long: Clear all bonding info
4. Button 1 extra long: Add RPAO characteristic to GAP service -- needed only when DM Privacy enabled
5. Button 2 extra long: Enable device privacy -- start generating local RPAs every 15 minutes

### When connected
1. Button 1 short: Start/Stop scanning
2. Button 1 medium: Cycle through connection index (select connection)
3. Button 1 long: Close selected connection  
4. Button 2 short: Request PHY change (1M-2M-S2-S8) Only for BLE5 version.
5. Button 2 long: Send short message to peer  
6. Button 2 extra long: Start data transfer speed test
