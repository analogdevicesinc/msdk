# BLE_datc_dats
BLE Data Client and Data Server. These examples demonstrate a simple data client and server. The intent here is to show simple unformatted data exchange between two devices.

The Bluetooth specification does not define a simple data exchange protocol, similar to UART. We use a proprietary GATT service to offer this feature.

The data server will advertise with a devices name of "DATS". The data client will scan for this device and create a connection. Once the conneciton has been created and the client completes the discovery, it will enable notificaitons and indiciations on the proprietary data serivce. This will allow the server to transmit data to the client with notifications to the proprietary data characteristic. The client can transmit data to the server by writing to the proprietary data characteristic.

<p align="center">
  <img width="300" src="../pics/ADI_Attach_DATS.jpg">
</p>

<p align = "center">
ADI Attach connected to DATS
</p>

## ARM Proprietary data service 

**UUID:** E0262760-08C2-11E1-9073-0E8AC72E1001

This is a proprietary serivce is used for simple unformatted data exchange between the server and the client.

## Secure data service

**UUID:** 42FC367E-32D9-4285-87C6-339924D135BE

This is another proprietary service used for secure data exchange that requires elevated levels of encryption and authentication. Devices must be paired with LE Secure Connections to enable data transfer with this service.

## Passkey input
Upon server discovery the user will be prompted to enter a passkey.
An arbitrary pin can be entered in the following format 
``` 
pin (connId) passkey
Eg: 
pin 1 123456

```
Next the server is expected to enter the same connId and passkey
to establish a secure connection and share bonding information
which for demonstration purposes,is echoed via the trace mechanism

Note that either the client or server can enter the passkey first.
The peer device must then match.

## LEDs

The red LED will indicate that an error assertion has occurred.  

The green LED indicates CPU activity. When the LED is on, the CPU is active, when the LED
is off, the CPU is in sleep mode.

## Board Setup

### Board Selection

Before building firmware you must select the correct value for BOARD in project.mk, e.g. "EvKit_V1".

### Required Connections
-   Connect a USB cable between the PC and the (USB/PWR - UART) connector.

### Trace Serial Port
When TRACE is enabled in the project.mk, the on-board USB-to-UART adapter can
be used to view the trace messages and interact with the application. Open a serial port terminal with
the following settings.

Baud            : 115200  
Char size       : 8  
Parity          : None  
Stop bits       : 1  
HW Flow Control : No  
SW Flow Control : No  


# BLE_Dats

Bluetooth data server that advertises as "DATS" and accepts connection requests. This exmaples contains two proprietary services that are used to show simple message passing to and from the data client. 

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

### When disconnected
1. Button 1 short press: Start advertising
2. Button 1 medium press: Enter bondable mode
3. Button 1 long press: Clear all bonding info
4. Button 1 extra long press: Show version info
5. Button 2 short press: Stop advertising

### When connected
1. Button 2 short press: Change PHY (1M-2M-Coded_S2-Coded_S8)

# BLE_Datc

Bluetooth data client that scans and connections to devices advertising with the device name "DATS". The client will automatically discover the profiles and services on the device, subscribing to notifications and indications to enable the unformatted data exchange with the proprietary data service. 

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

When server has been found:
```
Scan Report:
  00:18:80:04:52:1F
  Name: DATS
dmDevPassEvtToDevPriv: event: 13, param: 37, advHandle: 0
Datc got evt 37
Scan results: 4
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
smpSmExecute event=1 state=0
connId=1 idleMask=0x0001
Datc got evt 65
Datc got evt 22
smpSmExecute event=6 state=1
smpSmExecute event=17 state=2
smpSmExecute event=4 state=3
connId=1 idleMask=0x0001
Datc got evt 87
Datc got evt 4
connId=1 idleMask=0x0009
smpSmExecute event=6 state=4
smpSmExecute event=20 state=5
Datc got evt 46
>>> Prompt user to enter passkey <<<
Datc got evt 3
AttcDiscServiceCmpl status 0x00
Datc got evt 4
AttcDiscCharCmpl status 0x79
Datc got evt 4
AttcDiscCharCmpl status 0x79
Datc got evt 2
AttcDiscCharCmpl status 0x00
connId=1 idleMask=0x0009
Datc got evt 3
AttcDiscServiceCmpl status 0x00
Datc got evt 4
AttcDiscCharCmpl status 0x79
Datc got evt 4
AttcDiscCharCmpl status 0x00
connId=1 idleMask=0x0009
Datc got evt 3
AttcDiscServiceCmpl status 0x00
Datc got evt 4
AttcDiscCharCmpl status 0x79
Datc got evt 4
AttcDiscCharCmpl status 0x79
Datc got evt 2
AttcDiscCharCmpl status 0x00
connId=1 idleMask=0x0009
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
connId=1 idleMask=0x0001
AppDiscComplete connId:1 status:0x04
connId=1 idleMask=0x0009
Datc got evt 9
AttcDiscConfigCmpl status 0x79
Datc got evt 9
AttcDiscConfigCmpl status 0x79
Datc got evt 9
AttcDiscConfigCmpl status 0x79
Datc got evt 9
AttcDiscConfigCmpl status 0x00
connId=1 idleMask=0x0001
AppDiscComplete connId:1 status:0x08                                                                               
```
After entering a pin
```
pin 1 12345
> smpSmExecute event=4 state=11
Rand Na
[5fa96af9 f434b4de 3e452813 fef4f5eb]
smpSmExecute event=28 state=12
Cai
[57964640 e18b1aa1 8ea7d8b2 2eb274c3]
connId=1 idleMask=0x0001

```

Once server enters a matching pin and encrypted connection is established
```
smpSmExecute event=6 state=14
smpSmExecute event=28 state=15
Cbi
[7e2523a4 be6a8923 8ef900bf d4eaa458]
smpSmExecute event=26 state=16
Rand Na
[efb856be 61bd11a8 2dff39c0 1f5d4d91]
smpSmExecute event=28 state=12
Cai
[65f7c942 8c216010 cc73fdc9 bd21120e]
connId=1 idleMask=0x0001
smpSmExecute event=6 state=13
connId=1 idleMask=0x0001
smpSmExecute event=6 state=14
smpSmExecute event=28 state=15
Cbi
[db4858f8 33a964c1 e35ba27c 1f07e7db]
smpSmExecute event=26 state=16
Rand Na
[9904fded 40ea2bb1 dfabe4be b9bcba45]
smpSmExecute event=28 state=12
Cai
[1362d0db 074d32ea 32d0e79c 1155a20b]
connId=1 idleMask=0x0001
smpSmExecute event=6 state=13
connId=1 idleMask=0x0001
smpSmExecute event=6 state=14
smpSmExecute event=28 state=15
Cbi
[efd27c04 96ff72e0 c893bf54 9c524f0a]
smpSmExecute event=26 state=16
Rand Na
[5ba16640 fd3af1ae afc6d035 d8eb1fa5]
smpSmExecute event=28 state=12
Cai
[79603efa ad87d2cd bdc4c83b 17dcffe6]
connId=1 idleMask=0x0001
smpSmExecute event=6 state=13
connId=1 idleMask=0x0001
smpSmExecute event=6 state=14
smpSmExecute event=28 state=15
Cbi
[c1221276 73df3bf7 90e8df64 741909c1]
smpSmExecute event=26 state=16
Rand Na
[59ea06d0 760407f7 87415144 fe234a1e]
smpSmExecute event=28 state=12
Cai
[40b9915c af4a8712 c81e98f9 f197953a]
connId=1 idleMask=0x0001
smpSmExecute event=6 state=13
connId=1 idleMask=0x0001
smpSmExecute event=6 state=14
smpSmExecute event=28 state=15
Cbi
[325fe15b 5a6862e1 f4de459d 6c91c220]
smpSmExecute event=26 state=16
Rand Na
[4c56f9f9 b1d91236 8dfb81f8 b0629bb7]
smpSmExecute event=28 state=12
Cai
[c2fb6b18 63cb9311 d53c3333 0a2924cf]
connId=1 idleMask=0x0001
smpSmExecute event=6 state=13
connId=1 idleMask=0x0001
smpSmExecute event=6 state=14
smpSmExecute event=28 state=15
Cbi
[7725d42d 35f8e465 b3b6cfed 9bfe125a]
smpSmExecute event=26 state=16
Rand Na
[3909e384 45979c24 c85d36d3 fc2bd4bf]
smpSmExecute event=28 state=12
Cai
[9e570480 195412a0 07c84616 36cd6a06]
connId=1 idleMask=0x0001
smpSmExecute event=6 state=13
connId=1 idleMask=0x0001
smpSmExecute event=6 state=14
smpSmExecute event=28 state=15
Cbi
[1bce9472 21f7d6c4 4f8fe62b 91ac2271]
smpSmExecute event=26 state=16
Rand Na
[fe57e223 78ca04e3 cd13bd82 ad43289c]
smpSmExecute event=28 state=12
Cai
[42ba2b2e bd35ebe6 56c59b28 957639d2]
connId=1 idleMask=0x0001
smpSmExecute event=6 state=13
connId=1 idleMask=0x0001
smpSmExecute event=6 state=14
smpSmExecute event=28 state=15
Cbi
[a35acba5 00eba332 31442e10 aea9dacf]
smpSmExecute event=26 state=16
Rand Na
[71aa44ad abe2d668 07b79f02 d2a974dc]
smpSmExecute event=28 state=12
Cai
[5b02fce8 f92549ca 54bea290 c9fdfd1b]
connId=1 idleMask=0x0001
smpSmExecute event=6 state=13
connId=1 idleMask=0x0001
smpSmExecute event=6 state=14
smpSmExecute event=28 state=15
Cbi
[4a52c87c a504c8cf 49d9e8f0 1a69aca2]
smpSmExecute event=26 state=16
Rand Na
[0f87eb54 83f4f78b 754d2f74 1b406c0d]
smpSmExecute event=28 state=12
Cai
[16a9ba43 8c8b39d2 912f1f85 3f738170]
connId=1 idleMask=0x0001
smpSmExecute event=6 state=13
connId=1 idleMask=0x0001
smpSmExecute event=6 state=14
smpSmExecute event=28 state=15
Cbi
[b2aa1f53 802fa4ad 3e415844 4953c931]
smpSmExecute event=26 state=16
Rand Na
[466db705 9fd2d031 35759a3f 45869950]
smpSmExecute event=28 state=12
Cai
[004c8d5f 4413efb9 f60c10c2 8af82058]
connId=1 idleMask=0x0001
smpSmExecute event=6 state=13
connId=1 idleMask=0x0001
smpSmExecute event=6 state=14
smpSmExecute event=28 state=15
Cbi
[6b93dfa8 91d6bc27 fbb7142a 778d6eab]
smpSmExecute event=26 state=16
Rand Na
[4d0cfaee 277f0959 ee9ad6b5 bf89e129]
smpSmExecute event=28 state=12
Cai
[b977b373 4fdeb62d 4b9fc6dd 0cd58ccb]
connId=1 idleMask=0x0001
smpSmExecute event=6 state=13
connId=1 idleMask=0x0001
smpSmExecute event=6 state=14
smpSmExecute event=28 state=15
Cbi
[7d734f74 37d7575f 9a2176fe 9e0d7793]
smpSmExecute event=26 state=16
Rand Na
[aa704717 22c5a505 922ea504 c8f32c21]
smpSmExecute event=28 state=12
Cai
[b8db800a c2cfb643 0ad2127e 154b27e3]
connId=1 idleMask=0x0001
smpSmExecute event=6 state=13
connId=1 idleMask=0x0001
smpSmExecute event=6 state=14
smpSmExecute event=28 state=15
Cbi
[fb8bfb47 1db075d6 c10dfe20 275580a6]
smpSmExecute event=26 state=16
Rand Na
[6715c20d 284d6526 c826990b c0266d4d]
smpSmExecute event=28 state=12
Cai
[9a07bded 4652b3b3 04e58dd3 ddc41f5b]
connId=1 idleMask=0x0001
smpSmExecute event=6 state=13
connId=1 idleMask=0x0001
smpSmExecute event=6 state=14
smpSmExecute event=28 state=15
Cbi
[9b11572d 6ac46706 2a42ddf7 fa3f7eb6]
smpSmExecute event=26 state=16
Rand Na
[276b1950 0160195c 2ca2749e 04cc0bba]
smpSmExecute event=28 state=12
Cai
[7811b835 1ee6cd01 b35e0068 b0e995a2]
connId=1 idleMask=0x0001
smpSmExecute event=6 state=13
connId=1 idleMask=0x0001
smpSmExecute event=6 state=14
smpSmExecute event=28 state=15
Cbi
[febf4094 ccf93969 adc4d7e3 c3f26246]
smpSmExecute event=26 state=16
Rand Na
[31c8d021 62bd5eb6 470c008d 0f9a8f69]
smpSmExecute event=28 state=12
Cai
[6cc99436 0835e6bf b7ccd2a8 81eb7c2d]
connId=1 idleMask=0x0001
smpSmExecute event=6 state=13
connId=1 idleMask=0x0001
smpSmExecute event=6 state=14
smpSmExecute event=28 state=15
Cbi
[314f5edd e73e8b9e 865c3924 c438e032]
smpSmExecute event=26 state=16
Rand Na
[4f46b9ce 57474f8d 8d00a000 74bdea15]
smpSmExecute event=28 state=12
Cai
[baea7468 fabb299a 837bbbfb c47572e2]
connId=1 idleMask=0x0001
smpSmExecute event=6 state=13
connId=1 idleMask=0x0001
smpSmExecute event=6 state=14
smpSmExecute event=28 state=15
Cbi
[f8a46358 201aab0a ae9aae2c 0c32fb92]
smpSmExecute event=26 state=16
Rand Na
[b80a6511 3c82e7e6 055243e6 a45202a6]
smpSmExecute event=28 state=12
Cai
[24c084e8 e92b8e03 1a37f096 f4d5c31f]
connId=1 idleMask=0x0001
smpSmExecute event=6 state=13
connId=1 idleMask=0x0001
smpSmExecute event=6 state=14
smpSmExecute event=28 state=15
Cbi
[04aa43e8 dac7aa2a 490e6154 11b81966]
smpSmExecute event=26 state=16
Rand Na
[16bc0b9f a93b2745 729a08b9 85682b0d]
smpSmExecute event=28 state=12
Cai
[a5caa3a1 b7b66eaa f113972c 88645db6]
connId=1 idleMask=0x0001
smpSmExecute event=6 state=13
connId=1 idleMask=0x0001
smpSmExecute event=6 state=14
smpSmExecute event=28 state=15
Cbi
[831b8daf 7ae674d8 e2c5fbc3 9fc27be3]
smpSmExecute event=26 state=16
Rand Na
[3a8e02cf 001e84b3 08d9a24e 19ff9b18]
smpSmExecute event=28 state=12
Cai
[bf5345f1 299bca3f 0c99eac6 3ea96052]
connId=1 idleMask=0x0001
smpSmExecute event=6 state=13
connId=1 idleMask=0x0001
smpSmExecute event=6 state=14
smpSmExecute event=28 state=15
Cbi
[0a4dec2e 6aebc804 7bb1a21b aadc18c1]
smpSmExecute event=27 state=16
smpSmExecute event=25 state=19
DHKey (Shared Secret)
[7cf8b651 6c0f5bff 7d66bbf6 d8d85abd]
[0ef6bf2d 85a594d2 f55dccc3 6f98dd1d]
smpSmExecute event=28 state=20
T
[a02f2c9d 9a153f28 0bde8cb2 ab2a7b84]
smpSmExecute event=28 state=21
MAC
[66ebd01a c4818e8b 1cec71bf f0644ed9]
smpSmExecute event=28 state=22
LTK
[b4cad222 8324e2fb 71bbca0f 04bfc828]
smpSmExecute event=28 state=23
DHKey Ea
[a2925a68 0b71e9c3 065a959d 39079b01]
smpSmExecute event=28 state=24
DHKey Eb
[3ed3c373 5b99fc5e 16577603 7a1ba181]
connId=1 idleMask=0x0001
smpSmExecute event=6 state=25
connId=1 idleMask=0x0001
smpSmExecute event=8 state=34
Datc got evt 44
>>> Connection encrypted <<<
smpSmExecute event=6 state=35
smpSmExecute event=6 state=35
smpSmExecute event=12 state=35
Datc got evt 47
Datc got evt 47
smpSmExecute event=12 state=35
smpSmExecute event=12 state=35
smpSmExecute event=14 state=35
connId=1 idleMask=0x0000
Datc got evt 42
>>> Pairing completed successfully <<<
dmDevPassEvtToDevPriv: event: 13, param: 1, advHandle: 0
Datc got evt 58
Datc got evt 63
```
Simple message passing to peer:
```
btn 2 l
Long Button 2 Press
> Datc got evt 10
Datc got evt 13
hello back
```
Secure message passing to peer:
```
btn 2 m
Medium Button 2 Press
> Datc got evt 10
Datc got evt 13
>> Notification from secure data service <<<
Secure data received!

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
5. Button 2 medium : Send secure message to peer
6. Button 2 long: Send short message to peer  
7. Button 2 extra long: Start data transfer speed test

