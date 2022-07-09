# Description

Bluetooth data client that scans for and connects to advertisers with the name of "OTAS".

The Wireless Data Exchange profile is used to transfer files from the client to the server. 
A CRC32 value is used to check the integrity of the transferred file. 

# Usage

## LEDs

The red LED will indicate that an error assertion has occurred.  

The green LED indicates CPU activity. When the LED is on, the CPU is active, when the LED
is off, the CPU is in sleep mode.

## Setup
The Makefile can be edited to select the appropriate application directory for the update
image. Change FW_UPDATE_DIR to modify which application is used for the update. Whichever
application is selected must be setup to run from the appropriate memory section, as defined
by the Bootloader.

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
32kHz trimmed to 0xD                                                           
DatcHandlerInit                                                               
Update File CRC: 0xB4811A68                                                   
WDXC: WdxcHandlerInit                                                         
>>> Reset complete <<<                                                        
Database hash updated                                                         
dmDevPassEvtToDevPriv: event: 12, param: 36, advHandle: 0                     
>>> Scanning started <<<                                                      
```

When a connection has been made.
```
dmDevPassEvtToDevPriv: event: 13, param: 37, advHandle: 0                                                             
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
attcMsgCback: msg: x slot: x                                                                                          
hciCoreTxAclStart len=7                                                                                               
>>> Connection opened <<<                                                                                             
attcMsgCback: msg: x slot: x                                                                                          
hciCoreTxAclStart len=11                                                                                              
connId=1 idleMask=0x0008                                                                                              
attcMsgCback: msg: x slot: x                                                                                          
hciCoreTxAclStart len=13                                                                                              
found service startHdl=0x10 endHdl=0x19                                                                               
AttcDiscServiceCmpl status 0x00                                                                                       
attcMsgCback: msg: x slot: x                                                                                          
hciCoreTxAclStart len=11                                                                                              
hciCoreTxAclStart len=11                                                                                              
characteristic found handle:0x12                                                                                      
characteristic found handle:0x15                                                                                      
AttcDiscCharCmpl status 0x79                                                                                          
AttcDiscCharCmpl status 0x79                                                                                          
attcMsgCback: msg: x slot: x                                                                                          
hciCoreTxAclStart len=9                                                                                               
descriptor found handle:0x13                                                                                          
AttcDiscCharCmpl status 0x00                                                                                          
connId=1 idleMask=0x0008                                                                                              
attcMsgCback: msg: x slot: x                                                                                          
hciCoreTxAclStart len=13                                                                                              
found service startHdl=0x1 endHdl=0x7                                                                                 
AttcDiscServiceCmpl status 0x00                                                                                       
attcMsgCback: msg: x slot: x                                                                                          
hciCoreTxAclStart len=11                                                                                              
hciCoreTxAclStart len=11                                                                                              
characteristic found handle:0x7                                                                                       
AttcDiscCharCmpl status 0x79                                                                                          
AttcDiscCharCmpl status 0x00                                                                                          
connId=1 idleMask=0x0008                                                                                              
attcMsgCback: msg: x slot: x                                                                                          
hciCoreTxAclStart len=27                                                                                              
found service startHdl=0x200 endHdl=0x203                                                                             
AttcDiscServiceCmpl status 0x00                                                                                       
attcMsgCback: msg: x slot: x                                                                                          
hciCoreTxAclStart len=11                                                                                              
hciCoreTxAclStart len=11                                                                                              
characteristic found handle:0x202                                                                                     
AttcDiscCharCmpl status 0x79                                                                                          
AttcDiscCharCmpl status 0x79                                                                                          
attcMsgCback: msg: x slot: x                                                                                          
hciCoreTxAclStart len=9                                                                                               
descriptor found handle:0x203                                                                                         
AttcDiscCharCmpl status 0x00                                                                                          
connId=1 idleMask=0x0008                                                                                              
attcMsgCback: msg: x slot: x                                                                                          
hciCoreTxAclStart len=13                                                                                              
found service startHdl=0x240 endHdl=0xFFFF                                                                            
AttcDiscServiceCmpl status 0x00                                                                                       
attcMsgCback: msg: x slot: x                                                                                          
hciCoreTxAclStart len=11                                                                                              
hciCoreTxAclStart len=11                                                                                              
characteristic found handle:0x242                                                                                     
characteristic found handle:0x245                                                                                     
characteristic found handle:0x248                                                                                     
characteristic found handle:0x24B                                                                                     
AttcDiscCharCmpl status 0x79                                                                                          
AttcDiscCharCmpl status 0x79                                                                                          
attcMsgCback: msg: x slot: x                                                                                          
hciCoreTxAclStart len=9                                                                                               
descriptor found handle:0x243                                                                                         
AttcDiscCharCmpl status 0x79                                                                                          
attcMsgCback: msg: x slot: x                                                                                          
hciCoreTxAclStart len=9                                                                                               
descriptor found handle:0x246                                                                                         
AttcDiscCharCmpl status 0x79                                                                                          
attcMsgCback: msg: x slot: x                                                                                          
hciCoreTxAclStart len=9                                                                                               
descriptor found handle:0x249                                                                                         
AttcDiscCharCmpl status 0x79                                                                                          
attcMsgCback: msg: x slot: x                                                                                          
hciCoreTxAclStart len=9                                                                                               
hciCoreTxAclStart len=9                                                                                               
descriptor found handle:0x24C                                                                                         
AttcDiscCharCmpl status 0x79                                                                                          
AttcDiscCharCmpl status 0x00                                                                                          
connId=1 idleMask=0x0000                                                                                              
AppDiscComplete connId:1 status:0x04                                                                                  
connId=1 idleMask=0x0008                                                                                              
attcMsgCback: msg: x slot: x                                                                                          
hciCoreTxAclStart len=9                                                                                               
AttcDiscConfigCmpl status 0x79                                                                                        
attcMsgCback: msg: x slot: x                                                                                          
hciCoreTxAclStart len=8                                                                                               
AttcDiscConfigCmpl status 0x79                                                                                        
attcMsgCback: msg: x slot: x                                                                                          
hciCoreTxAclStart len=9                                                                                               
AttcDiscConfigCmpl status 0x79                                                                                        
attcMsgCback: msg: x slot: x                                                                                          
hciCoreTxAclStart len=9                                                                                               
AttcDiscConfigCmpl status 0x79                                                                                        
attcMsgCback: msg: x slot: x                                                                                          
hciCoreTxAclStart len=9                                                                                               
AttcDiscConfigCmpl status 0x79                                                                                        
attcMsgCback: msg: x slot: x                                                                                          
hciCoreTxAclStart len=9                                                                                               
AttcDiscConfigCmpl status 0x79                                                                                        
attcMsgCback: msg: x slot: x                                                                                          
hciCoreTxAclStart len=9                                                                                               
AttcDiscConfigCmpl status 0x00                                                                                        
connId=1 idleMask=0x0000                                                                                              
AppDiscComplete connId:1 status:0x08                                                                                  
```

OTA procedure
```
btn 2 s                                                                       
Short Button 2 Press                                                          
> WDXC file transfer control.                                                 
FTC op: 2 status: 0                                                           
WDXC file transfer control.                                                   
FTC op: 10 status: 0                                                          
>>> File discovery complete <<<                                               
                                                                              
btn 2 m                                                                       
Medium Button 2 Press                                                         
> WDXC file transfer control.                                                 
FTC op: 4 status: 0                                                           
>>> Starting file transfer <<<
...                                                     
WDXC file transfer control.                                                   
FTC op: 10 status: 0                                                          
>>> File transfer complete 3445340 us <<<                                     
                                                                              
file_size = 183300 usec = 3445340 Bps = 53207                                 
                                                                              
flowDisabled=0 handle=0                                                       
btn 2 l                                                                       
Long Button 2 Press                                                           
> WDXC file transfer control.                                                 
FTC op: 8 status: 0                                                           
>>> Verify complete status: 0 <<< 

btn 2 x                                                                       
XL Button 2 Press                                                             
> dmConnSmExecute event=29 state=3                                            
dmConnCcbDealloc 1                                                            
dmDevPassEvtToDevPriv: event: 13, param: 40, advHandle: 0                     
smpDbGetRecord: connId: 1 type: 0                                             
smpDbAddDevice                                                                
SmpDbSetFailureCount: connId: 1 count: 0                                      
smpSmExecute event=10 state=0                                                 
Connection closed status 0x0, reason 0x13                                     
 REMOTE TERM                                                                  
>>> Connection closed <<<
```

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
1. Button 1 short: On/Off scanning  
2. Button 1 medium: Cycle through the connection index  
3. Button 1 long: Drop selected connection  
4. Button 1 extra long: Toggle PHY 
5. Button 2 short: Discover file space on the peer device.
6. Button 2 medium: Start the update transfer.
7. Button 2 long: Verify the transfer.
8. Button 2 extra long: Command the peer to disconnect and reset.

### When disconnected
1. Button 1 short press: On/Off scanning
2. Button 1 medium press: Cycle through the connection index
3. Button 1 long press: Clear all bonding info
4. Button 1 extra long press: Add RPAO characteristic to GAP service -- needed only when DM Privacy enabled
5. Button 2 extra long press: Enable device privacy -- start generating local RPAs every 15 minutes
