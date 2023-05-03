# Description

Simple Bootloader that reloads the main flash image. Using an external memory device is preferable,
the update speed is limited by the erase/write time of the internal flash.

A 32 bit CRC value is appended to the end of the update flash image. 
CRC32 is used to verify the integrity of the update image. If a valid update image is identified,
the main flash section is erased and replaced with the update image. If no valid update image
is identified, the Bootloader will boot the exiting image in the main flash space.

__0x10000000__: Bootloader  
__0x10004000__: Main flash space  
__0x10300000__: Update flash space

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

This Bootloader application needs to be loaded to the first two flash pages. This is accomplished with the [bootloader.ld](bootloader.ld) linker file.  The main application
will run on top of this application. The linker file for the main application must coincide 
with the memory sections defined in this application (see the `BLE_otas` for an example of an application that does this). The main application is responsible for updating the update flash space.

 
## Expected Output

On startup:
```
terminal: init
32kHz trimmed to 0x16
DatcHandlerInit
File addr: 10025950 file size: 00033368
Update File CRC: 0x8871C78B
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
>>> Connection opened <<<
connId=1 idleMask=0x0008
AttcDiscServiceCmpl status 0x00
AttcDiscCharCmpl status 0x79
AttcDiscCharCmpl status 0x79
AttcDiscCharCmpl status 0x00
connId=1 idleMask=0x0008
AttcDiscServiceCmpl status 0x00
AttcDiscCharCmpl status 0x79
AttcDiscCharCmpl status 0x00
connId=1 idleMask=0x0008
AttcDiscServiceCmpl status 0x00
AttcDiscCharCmpl status 0x79
AttcDiscCharCmpl status 0x79
AttcDiscCharCmpl status 0x00
connId=1 idleMask=0x0008
AttcDiscServiceCmpl status 0x00
AttcDiscCharCmpl status 0x79
AttcDiscCharCmpl status 0x79
AttcDiscCharCmpl status 0x79
AttcDiscCharCmpl status 0x79
AttcDiscCharCmpl status 0x79
AttcDiscCharCmpl status 0x79
AttcDiscCharCmpl status 0x00
connId=1 idleMask=0x0000
AppDiscComplete connId:1 status:0x04
connId=1 idleMask=0x0008
AttcDiscConfigCmpl status 0x79
AttcDiscConfigCmpl status 0x79
AttcDiscConfigCmpl status 0x79
AttcDiscConfigCmpl status 0x79
AttcDiscConfigCmpl status 0x79
AttcDiscConfigCmpl status 0x79
AttcDiscConfigCmpl status 0x00
connId=1 idleMask=0x0000
AppDiscComplete connId:1 status:0x08
                                                                    
```

OTA procedure
```
btn 2 s
Short Button 2 Press
> 
WDXC file transfer control.
FTC op: 2 status: 0

WDXC file transfer control.
FTC op: 10 status: 0
>>> File discovery complete <<<

>>> Current fw version: 1.0 <<<                                              
                                                                              
btn 2 m                                                                       
Medium Button 2 Press                                                         
> WDXC file transfer control.                                                 
FTC op: 4 status: 0                                                           
>>> Starting file transfer <<<
... 
WDXC file transfer control.
FTC op: 10 status: 0
>>> File transfer complete 3547207 us <<<
file_size = 209768 usec = 3547207 bps = 473112
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

