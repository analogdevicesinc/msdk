# Description

Bluetooth data server that advertises as "DATS" and accepts connection requests.

BLE roles are split into pre-connection and post-connection. In pre-connection, at startup a device is either a peripheral (like a Fitbit or smart watch) or a central (like a smartphone or PC). In post-connection (after a BLE connection has been established), devices can be either a client or a server.

![BLE roels](../../../Documentation/Images/BLE_roles.PNG)

In this application, the dats plays a slave role of a proprietary data tansfer application. After it starts it will begin advertising. When dats receives a data message from the peer device (see example BLE_datc), it will automatically send a fixed data message back.

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
32kHz trimmed to 0x16
DatsHandlerInit
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
Dats got evt 21
Database hash updated

```

When a connection attempting to connect to the client.
```
Dats got evt 21
Database hash updated
Dats got evt 153
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
Dats got evt 22
smpSmExecute event=6 state=0
connId=1 idleMask=0x0001
Dats got evt 49
smpSmExecute event=2 state=2
smpSmExecute event=17 state=3
smpSmExecute event=4 state=5
smpSmExecute event=6 state=4
smpSmExecute event=20 state=6
connId=1 idleMask=0x0001
Dats got evt 46
>>> Prompt user to enter passkey <<<
Dats got evt 87
connId=1 idleMask=0x0005
connId=1 idleMask=0x0005
connId=1 idleMask=0x0005
connId=1 idleMask=0x0005
connId=1 idleMask=0x0005
connId=1 idleMask=0x0005
connId=1 idleMask=0x0005
connId=1 idleMask=0x0005

```
When matching pins have been entered
```
> smpSmExecute event=4 state=13                                          
Rand Nb                                                                  
[52730a1e 080e2b3f 6d010f4b 0d29b8ac]                                    
smpSmExecute event=28 state=15                                           
Cbi                                                                      
[520aab85 e23b56d5 70598231 64553242]                                    
connId=1 idleMask=0x0001                                                 
smpSmExecute event=6 state=16                                            
smpSmExecute event=28 state=17                                           
Ca                                                                       
[634082f9 ae97841d 408a0609 d3149a0f]                                    
Ca Peer                                                                  
[634082f9 ae97841d 408a0609 d3149a0f]                                    
connId=1 idleMask=0x0001                                                 
smpSmExecute event=26 state=18                                           
smpSmExecute event=6 state=14                                            
Rand Nb                                                                  
[2b4f65e1 41465986 1732c49b 0779c049]                                    
smpSmExecute event=28 state=15                                           
Cbi                                                                      
[34b63e15 0a0b1d8a fbaac17d ae94c891]                                    
connId=1 idleMask=0x0001                                                 
smpSmExecute event=6 state=16                                            
smpSmExecute event=28 state=17                                           
Ca                                                                       
[e7b74ec3 7bb660e0 fd3ffb69 a70fc1b5]                                    
Ca Peer                                                                  
[e7b74ec3 7bb660e0 fd3ffb69 a70fc1b5]                                    
connId=1 idleMask=0x0001                                                 
smpSmExecute event=26 state=18                                           
smpSmExecute event=6 state=14                                            
Rand Nb                                                                  
[cb42c521 d4edb9cd fe2a5777 526d153d]                                    
smpSmExecute event=28 state=15                                           
Cbi                                                                      
[58d9a8c4 c4710a1a bd41ed81 2e666953]                                    
connId=1 idleMask=0x0001                                                 
smpSmExecute event=6 state=16                                            
smpSmExecute event=28 state=17                                           
Ca                                                                       
[d100648f ae87b3e0 850e96ef 3c2c9a47]                                    
Ca Peer                                                                  
[d100648f ae87b3e0 850e96ef 3c2c9a47]                                    
connId=1 idleMask=0x0001                                                 
smpSmExecute event=26 state=18                                           
smpSmExecute event=6 state=14                                            
Rand Nb                                                                  
[21a3cd36 613f1834 c6e3d6fa 337e3f6c]                                    
smpSmExecute event=28 state=15                                           
Cbi                                                                      
[314e5afe c5799362 0c3c7e01 54a62b7d]                                    
connId=1 idleMask=0x0001                                                 
smpSmExecute event=6 state=16                                            
smpSmExecute event=28 state=17                                           
Ca                                                                       
[bc03c14d cf73fbab c64c8de8 65c9bd3f]                                    
Ca Peer                                                                  
[bc03c14d cf73fbab c64c8de8 65c9bd3f]                                    
connId=1 idleMask=0x0001                                                 
smpSmExecute event=26 state=18                                           
smpSmExecute event=6 state=14                                            
Rand Nb                                                                  
[3e0cb037 9edea6c2 dfdd238f eee47a18]                                    
smpSmExecute event=28 state=15                                           
Cbi                                                                      
[223876ac 32cfa06e 5764766f e655cfda]                                    
connId=1 idleMask=0x0001                                                 
smpSmExecute event=6 state=16                                            
smpSmExecute event=28 state=17                                           
Ca                                                                       
[3a1dceac 4ed11dc5 60384042 5f625042]                                    
Ca Peer                                                                  
[3a1dceac 4ed11dc5 60384042 5f625042]                                    
connId=1 idleMask=0x0001                                                 
smpSmExecute event=26 state=18                                           
smpSmExecute event=6 state=14                                            
Rand Nb                                                                  
[23ca1caf 70a3ba5b fd40e6ca db907405]                                    
smpSmExecute event=28 state=15                                           
Cbi                                                                      
[0f969efc f159e4c6 588c68ae 5d1f9851]                                    
connId=1 idleMask=0x0001                                                 
smpSmExecute event=6 state=16                                            
smpSmExecute event=28 state=17                                           
Ca                                                                       
[c34f8c33 3e16ee08 3cac6f79 1eea831f]                                    
Ca Peer                                                                  
[c34f8c33 3e16ee08 3cac6f79 1eea831f]                                    
connId=1 idleMask=0x0001                                                 
smpSmExecute event=26 state=18                                           
smpSmExecute event=6 state=14                                            
Rand Nb                                                                  
[840f704c 59845fc2 47d5463a 9c68159b]                                    
smpSmExecute event=28 state=15                                           
Cbi                                                                      
[f5a84dfe 36bdfc2d 410e133d 0c474039]                                    
connId=1 idleMask=0x0001                                                 
smpSmExecute event=6 state=16                                            
smpSmExecute event=28 state=17                                           
Ca                                                                       
[e27af695 a9227a18 74c2ba3d 57ed5254]                                    
Ca Peer                                                                  
[e27af695 a9227a18 74c2ba3d 57ed5254]                                    
connId=1 idleMask=0x0001                                                 
smpSmExecute event=26 state=18                                           
smpSmExecute event=6 state=14                                            
Rand Nb                                                                  
[5558d557 ba639669 226f7665 98ba1755]                                    
smpSmExecute event=28 state=15                                           
Cbi                                                                      
[c8158453 c50a9448 236c7bd5 fe249a0d]                                    
connId=1 idleMask=0x0001                                                 
smpSmExecute event=6 state=16                                            
smpSmExecute event=28 state=17                                           
Ca                                                                       
[530da1cb 834ec9fa 6076fd05 037dce54]                                    
Ca Peer                                                                  
[530da1cb 834ec9fa 6076fd05 037dce54]                                    
connId=1 idleMask=0x0001                                                 
smpSmExecute event=26 state=18                                           
smpSmExecute event=6 state=14                                            
Rand Nb                                                                  
[9f57fda8 897aac72 810475a4 eda95e4c]                                    
smpSmExecute event=28 state=15                                           
Cbi                                                                      
[12494fdf 83718797 5efa3ff7 4e230a9d]                                    
connId=1 idleMask=0x0001                                                 
smpSmExecute event=6 state=16                                            
smpSmExecute event=28 state=17                                           
Ca                                                                       
[b9c97dbe 7a725ab4 20c59148 10821669]                                    
Ca Peer                                                                  
[b9c97dbe 7a725ab4 20c59148 10821669]                                    
connId=1 idleMask=0x0001                                                 
smpSmExecute event=26 state=18                                           
smpSmExecute event=6 state=14                                            
Rand Nb                                                                  
[564e5d0e 2605341f 4d5b6913 0c89c1aa]                                    
smpSmExecute event=28 state=15                                           
Cbi                                                                      
[a8b37963 eafdcaee ecbf62ab a23f686a]                                    
connId=1 idleMask=0x0001                                                 
smpSmExecute event=6 state=16                                            
smpSmExecute event=28 state=17                                           
Ca                                                                       
[b1387cd1 054a3616 3a73b06d 085e74b7]                                    
Ca Peer                                                                  
[b1387cd1 054a3616 3a73b06d 085e74b7]                                    
connId=1 idleMask=0x0001                                                 
smpSmExecute event=26 state=18                                           
smpSmExecute event=6 state=14                                            
Rand Nb                                                                  
[fc4d0a4e 2e6ca8f1 65f242a8 88876d0e]                                    
smpSmExecute event=28 state=15                                           
Cbi                                                                      
[623c1256 7e20dba1 f5257b86 11b54ece]                                    
connId=1 idleMask=0x0001                                                 
smpSmExecute event=6 state=16                                            
smpSmExecute event=28 state=17                                           
Ca                                                                       
[4540e3cd 77e8ac57 8024607c 0de42ce2]                                    
Ca Peer                                                                  
[4540e3cd 77e8ac57 8024607c 0de42ce2]                                    
connId=1 idleMask=0x0001                                                 
smpSmExecute event=26 state=18                                           
smpSmExecute event=6 state=14                                            
Rand Nb                                                                  
[144e1412 64996fa0 d686011f 7913327d]                                    
smpSmExecute event=28 state=15                                           
Cbi                                                                      
[b354740a 3d097abc e33af952 cffc0e8a]                                    
connId=1 idleMask=0x0001                                                 
smpSmExecute event=6 state=16                                            
smpSmExecute event=28 state=17                                           
Ca                                                                       
[ba1e5505 2e488dda b984bd08 2d07fdeb]                                    
Ca Peer                                                                  
[ba1e5505 2e488dda b984bd08 2d07fdeb]                                    
connId=1 idleMask=0x0001                                                 
smpSmExecute event=26 state=18                                           
smpSmExecute event=6 state=14                                            
Rand Nb                                                                  
[2596e6cd 44929e11 9791ba02 62e3feef]                                    
smpSmExecute event=28 state=15                                           
Cbi                                                                      
[3790e2f7 96ca63b6 826bbf5c 077c3f31]                                    
connId=1 idleMask=0x0001                                                 
smpSmExecute event=6 state=16                                            
smpSmExecute event=28 state=17                                           
Ca                                                                       
[fe05bd6a 1f277dee 124de8b0 02c03343]                                    
Ca Peer                                                                  
[fe05bd6a 1f277dee 124de8b0 02c03343]                                    
connId=1 idleMask=0x0001                                                 
smpSmExecute event=26 state=18                                           
smpSmExecute event=6 state=14                                            
Rand Nb                                                                  
[06175016 3aabb9f3 f3aa5925 c9b8a43d]                                    
smpSmExecute event=28 state=15                                           
Cbi                                                                      
[57d2399c 89fbc8f0 3ee71020 f66b45d5]                                    
connId=1 idleMask=0x0001                                                 
smpSmExecute event=6 state=16                                            
smpSmExecute event=28 state=17                                           
Ca                                                                       
[c2dff69b 7ae8eba8 e92f3b57 ff3978c7]                                    
Ca Peer                                                                  
[c2dff69b 7ae8eba8 e92f3b57 ff3978c7]                                    
connId=1 idleMask=0x0001                                                 
smpSmExecute event=26 state=18                                           
smpSmExecute event=6 state=14                                            
Rand Nb                                                                  
[5c70daab 66fe0495 07d0e27d 824d9865]                                    
smpSmExecute event=28 state=15                                           
Cbi                                                                      
[f03928aa 2324a2ca 8c3c1fd8 9d589afe]                                    
connId=1 idleMask=0x0001                                                 
smpSmExecute event=6 state=16                                            
smpSmExecute event=28 state=17                                           
Ca                                                                       
[9f8dec26 018dcea5 0cc56156 32b9107b]                                    
Ca Peer                                                                  
[9f8dec26 018dcea5 0cc56156 32b9107b]                                    
connId=1 idleMask=0x0001                                                 
smpSmExecute event=26 state=18                                           
smpSmExecute event=6 state=14                                            
Rand Nb                                                                  
[fd89b81d e2b3fdaf f22cf4c4 b39da763]                                    
smpSmExecute event=28 state=15                                           
Cbi                                                                      
[c005af08 39334b4a ac68cf0a ca5acc00]                                    
connId=1 idleMask=0x0001                                                 
smpSmExecute event=6 state=16                                            
smpSmExecute event=28 state=17                                           
Ca                                                                       
[94681382 aa6a4d27 1b8eb879 a5ed8043]                                    
Ca Peer                                                                  
[94681382 aa6a4d27 1b8eb879 a5ed8043]                                    
connId=1 idleMask=0x0001                                                 
smpSmExecute event=26 state=18                                           
smpSmExecute event=6 state=14                                            
Rand Nb                                                                  
[5b0789ba 4bd1a9f8 10e15f9d 0dbd4ac2]                                    
smpSmExecute event=28 state=15                                           
Cbi                                                                      
[b719fe1d 02e6cc10 8776a1b4 0bc0fa6b]                                    
connId=1 idleMask=0x0001                                                 
smpSmExecute event=6 state=16                                            
smpSmExecute event=28 state=17                                           
Ca                                                                       
[3f8a85da fad78d81 0c8e0733 876df6fb]                                    
Ca Peer                                                                  
[3f8a85da fad78d81 0c8e0733 876df6fb]                                    
connId=1 idleMask=0x0001                                                 
smpSmExecute event=26 state=18                                           
smpSmExecute event=6 state=14                                            
Rand Nb                                                                  
[c0c90b30 03659e86 a143abe4 ec289e73]                                    
smpSmExecute event=28 state=15                                           
Cbi                                                                      
[d02d6808 35888e79 55e04238 f0930c07]                                    
connId=1 idleMask=0x0001                                                 
smpSmExecute event=6 state=16                                            
smpSmExecute event=28 state=17                                           
Ca                                                                       
[4efc0b02 bcd8bc6a ac93b70c effeaee1]                                    
Ca Peer                                                                  
[4efc0b02 bcd8bc6a ac93b70c effeaee1]                                    
connId=1 idleMask=0x0001                                                 
smpSmExecute event=26 state=18                                           
smpSmExecute event=6 state=14                                            
Rand Nb                                                                  
[96aab51d 5e627668 ba927ed6 d1192554]                                    
smpSmExecute event=28 state=15                                           
Cbi                                                                      
[49660e21 5b3e82cd 32e3ac9a d839ad2d]                                    
connId=1 idleMask=0x0001                                                 
smpSmExecute event=6 state=16                                            
smpSmExecute event=28 state=17                                           
Ca                                                                       
[2db916d0 32b02469 1711edf5 812ae090]                                    
Ca Peer                                                                  
[2db916d0 32b02469 1711edf5 812ae090]                                    
connId=1 idleMask=0x0001                                                 
smpSmExecute event=26 state=18                                           
smpSmExecute event=6 state=14                                            
Rand Nb                                                                  
[d9e874e4 9b02cb3f 9aac021d 7ad59561]                                    
smpSmExecute event=28 state=15                                           
Cbi                                                                      
[837b81cf 6c409caa c29a550b 649b2c46]                                    
connId=1 idleMask=0x0001                                                 
smpSmExecute event=6 state=16                                            
smpSmExecute event=28 state=17                                           
Ca                                                                       
[f3d0b1b5 76097bfb 14418e57 6d3c11ba]                                    
Ca Peer                                                                  
[f3d0b1b5 76097bfb 14418e57 6d3c11ba]                                    
smpSmExecute event=27 state=18                                           
connId=1 idleMask=0x0001                                                 
smpSmExecute event=6 state=21                                            
smpSmExecute event=25 state=22                                           
DHKey (Shared Secret)                                                    
[bf891259 97f62954 0f8d8934 5bcd585f]                                    
[48fc2eb4 4c6f5c19 b8f817a9 907571b3]                                    
smpSmExecute event=28 state=23                                           
T                                                                        
[72c2ed8a fe1ae569 8237053f b5af654c]                                    
smpSmExecute event=28 state=24                                           
MAC                                                                      
[273eaeb4 163166a1 362c8773 888811aa]                                    
smpSmExecute event=28 state=25                                           
LTK                                                                      
[7eacbbfe 776de847 1b78effc 32e84ba5]                                    
smpSmExecute event=28 state=26                                           
DHKey Ea                                                                 
[e619e7cf 7d654f17 f621cb5c f6dd720b]                                    
smpSmExecute event=28 state=27                                           
DHKey Eb                                                                 
[1e72f16f dcab7a66 2787c1ce ee93d5ec]                                    
connId=1 idleMask=0x0001                                                 
connId=1 idleMask=0x0001                                                 
smpSmExecute event=8 state=36                                            
Dats got evt 44                                                          
>>> Connection encrypted <<<                                             
Dats got evt 47                                                          
smpSmExecute event=12 state=37                                           
smpSmExecute event=12 state=37                                           
smpSmExecute event=6 state=37                                            
smpSmExecute event=6 state=37                                            
smpSmExecute event=14 state=37                                           
connId=1 idleMask=0x0000                                                 
Dats got evt 47                                                          
Dats got evt 42                                                          
>>> Pairing completed successfully <<<                                   
dmDevPassEvtToDevPriv: event: 13, param: 1, advHandle: 0                 
Dats got evt 58                                                          
Dats got evt 63                                                          
Dats got evt 153                       

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
When secure message recevied from peer
```
>> Received secure data <<                                               
Secret number is 0x42                                                    
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

Table: Button and operations
| Connection Status | Button Press        | Operation                            |
| ----------------- | ------------------- | ------------------------------------ |
| Disconnected      | Button 1 short      | Start advertising                    |
|                   | Button 1 medium     | Enter bondable mode                  |
|                   | Button 1 long       | Clear all bonding info               |
|                   | Button 1 extra long | Show version info                    |
|                   | Button 2 short      | Stop advertising                     |
| Connected         | Button 2 short      | Change PHY (1M-2M-Coded_S2-Coded_S8) |