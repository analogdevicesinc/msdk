---
title: "Build SCP Session - User Guide"
subtitle: "UG98S09"
author: [Maxim Integrated]
date: "2018-09-10"
keywords: [Markdown, Example]
titlepage: true
titlepage-text-color: "00B2A9"
listings-disable-line-numbers: true
...

# Description

The build_scp_session tool computes offline the SCP frames corresponding to the provided parameters.

# Usage


```
build_scp_session [OPTION] [PARAMETERS]... [ FOLDER [EXTRA_PARAM]... ]
```

---------------- -----------------------------------------------------------
 `OPTION`         See General Options
 `PARAMETERS`     See Parameters
 `FOLDER`         Output folder to store SCP Packet
 `EXTRA_PARAM`    See Extra Parameters
---------------- -----------------------------------------------------------


# General Options

## c - chip part number

` -c CHIP_NAME ` - Use default configuration of CHIP_NAME.

## Help

` -h	` - Print this help and quit.

## Version

`  -v	  `  - 			output software and libraries versions and quit.

## Debug

`  -d		` -	Activate debug output.

# Parameters

Parameters are used by priority in the following order :

1. Command line
2. Configuration file " INIFILE " in the current folder
3. Chip default parameters selected by the -c option or the MAXIM_SBT_DEVICE env variable.
4. Software default values.


## SCP Script file 

```
script_file=script.txt
```
`script.txt` - text file containing SCP operation to perform. For more information see [SCP Script Commands](#scp-script-commands)

## Output filename prefix 

```
output_file=nameprefix
```
` nameprefix  ` - filename prefix of SCP generated file (packets and logfile). Default value : `session.txt`

## Output folder 

```
output_dir=dir
```
` dir ` - folder where SCP files will be saved. If this folder does not exist it will be created.

## Key file

```
key_file=file.key
```
UCL format private key file for SCP packet signing. For more information see *UCL Key Format* documentation.


## Session Mode

```
session_mode=mode
```
` mode ` - SCP session mode to be used for the communication with SBL. Please refers to CHIP documentation to select the corect one. Available mode are :

- `SCP_FLORA_RSA`
- `MSP_MAXQ1852_ECDSA`
- `SCP_ECDSA`
- `SCP_LITE_ECDSA`
- `SCP_PAOLA`



## Protection Profile

```
pp=PP
```

` PP ` - SCP Protection profile to be used for the communication with SBL. Please refers to CHIP documentation to select the corect one. Available protection profile are :

- `RSA_2048`
- `RSA_4096`
- `ECDSA`

## Verbose 

```
verbose=level
```
verbose level (0-5)

## Chunk Size

```
chunk_size=size
```

` size ` - maximum data size for one SCP packet (in bytes), this value have to be set according the CHIP used.

## Maximum Flash Size

```
flash_size_mb=size
```
` size ` - maximum flash size in Mo, this define the memory allocated when reading a data file (S19, HEX or SBIN)
	
## USN - Unique serial Number
	
```
usn=USN
```
` USN ` - Unique Serial Number of the device you want to personnalized the SCP session for (i.e. kill-chip command).
	
## Transaction ID (MAXQ1852 only)	

```
transaction_id=trid
```
` trid ` - User Selected transaction ID when using MSP_MAXQ1852_ECDSA.
	
## Transaction ID (MAXQ1852 only)	

```
addr_offset=address
```
` address` - address offset added when reading S19 files and base address when reading SBIN files.


# HSM Parameter

This application can use a Thales(R) HSM for key storage and cryptographics operation. By default the application use it's builtin cryptographics functions.

## HSM 

```
hsm=yes
hsm=no
```

Use or not an HSM to manage key and perform cryptographics operations

## HSM Key Name

```
hsm_key_name=name
```
` name ` - name of the key to use stored inside the HSM.

## HSM Thales DLL Location

```
hsm_thales_dll=dll_path
```

` dll_path ` - path to the Thales cknfast DLL.

## HSM SLot Number

```
hsm_slot_nb=nb
```

` nb ` - number of the HSM slot to use (usually : 1).


# Extra Parameters

In order to make *SCP scripts* more modular extra parmaters can be used. There are **TAGS** used in the script that will be replace at the excution with parameters provided in the commandline.
 
The format of the at is the following **%PARAM_N%**. With N from 0 to 9.

## Example

Let the following script :

```
wrtie-otp 08bc %PARM_0% 
write-otp 09bc %PARM_1% 
write-file %PARM_2% 

```

With the following command line call

```
build_scp_session scp_folder 02654212 ED45830A firmware.sbin
```

 will become :

```
wrtie-otp 08bc 02654212 
write-otp 09bc ED45830A 
write-file firmware.sbin 

```

> Note: When Using Extra parameters the folder **MUST** be specified. 

# SCP Script Commands



## Write File

```
write-file filename [address]
```

This command send the binary data contains in the provided file to the SBL for writing using the `WRITE DATA` SCP Command. It also erase the target memory are using the `ERASE DATA` SCP Command.


` filename `   S19 or sbin file containing the data to be send for writing to the SBL.
-------------- -----------------------------------------------------------
` address `    Start address to where writing data from sbin file or offset adddress to add to S19 addresses.
                 Optionnal whith S19 files but mandatory with sbin files.


## Write Only

```
write-only filename [address]
```


This command send the binary data contains in the provided file to the SBL for writing using the `WRITE DATA` SCP Command. 

` filename `   S19 or sbin file containing the data to be send for writing to the SBL.
-------------- -----------------------------------------------------------
` address `    Start address to where writing data from sbin file or offset adddress to add to S19 addresses.
                 Optionnal whith S19 files but mandatory with sbin files.


## Verify file

```
verify-file filename [address [dump]]
```


This command send the binary data contains in the provided file to the SBL for verification against the content of the memory writing using the `COMPARE DATA` SCP Command. 

-------------- -----------------------------------------------------------
` filename `   S19 or sbin file containing the data to be send for writing to the SBL.
` address `    Start address to where start data for verification from sbin file or offset adddress to add to S19 addresses.
                Optionnal whith S19 files but mandatory with sbin files.
` dump `    	 yes/no Add a dummy dump packet for the SCP sender
-------------- -----------------------------------------------------------

## Write CRK

```
write-crk filename
```

This command send `WRITE-CRK` SCP Command. It send the CRK with it's signature by the MRK. 

--------------- -----------------------------------------------------------
` filename `     File containing the CRK sign by the MRK
--------------- -----------------------------------------------------------




## Rewrite CRK

```
rewrite-crk old_crk_filename new_crk_filename
```

This command send `REWRITE-CRK` SCP Command. It send the *old* CRK and the *new* CRK with it's signature by the MRK. 


` old_crk_filename `     File containing the old CRK sign by the MRK
------------------------ -----------------------------------------------------------
` new_crk_filename `     File containing the new CRK sign by the MRK


## Echo

```
echo
```

This command check the communication with the SBL by sending an `ECHO` SCP command.


## Write OTP

```
write-otp offset data
```

This command write data inside the CHIP OTP using the `WRITE-OTP` SCP Command. 


` offset `      Address offset inside the OTP memory.
-------------- -----------------------------------------------------------
` data `   	 Data to write at the offset specified.


## Write Time-out

```
write-timeout target value
```

This command write the timeout configuration for the different SCP bus using the `WRITE-TIMEOUT` SCP Command.

--------------  --------------------------------------------------------------------------
` target `   	 Bus for which the timeout will be written. Possible value are :
				 `0` - for UART
				 `V` - for VBUS
				 `U` - for USB
				 `E` - for Ethernet
				 `S` - for SPI					
` Value `   	 Value of the Timeout in ms.
--------------  --------------------------------------------------------------------------


## Write Parameter

```
write-param target value
```

This command write the parameter configuration for the different SCP bus using the `WRITE-PARAM` SCP Command.

--------------  --------------------------------------------------------------------------
` target `   	 Bus for which the parameter will be written. Possible value are :
				 `0` - for UART
				 `V` - for VBUS
				 `U` - for USB
				 `E` - for Ethernet
				 `S` - for SPI					
` Value `   	 Value of the parameter.
--------------  --------------------------------------------------------------------------


## Write Stimulus

```
write-stim target value
```

This command write the stimulus configuration for the different SCP bus using the `WRITE-STIM` SCP Command.

--------------  --------------------------------------------------------------------------
` target `   	 Bus for which the stimulus will be written. Possible value are :
				 `0` - for UART
				 `V` - for VBUS
				 `U` - for USB
				 `E` - for Ethernet
				 `S` - for SPI					
` Value `   	 Value of the stimulus.
--------------  --------------------------------------------------------------------------


## Write Deactivation

```
write-deact target
```

This command deactivate the different SCP bus using the `WRITE-DEACT` SCP Command.

--------------  --------------------------------------------------------------------------
` target `   	 Bus for which the stimulus will be written. Possible value are :
				 `0` - for UART
				 `V` - for VBUS
				 `U` - for USB
				 `E` - for Ethernet
				 `S` - for SPI					
--------------  --------------------------------------------------------------------------

## Kill Chip

```
kill-chip
```

This command send the `KILL-CHIP` SCP command to SBL.


## Kill Chip USN

```
kill-chip2
```
This command send the `KILL-CHIP2` SCP command to SBL with the Chip Unique Serial Number (USN) provided with the corresponding option.


## Execute Code / Register Applet

```
execute-code address 
```
This command send the `EXECUTE-CODE` SCP command to SBL. This will register an applet if the adress point to an applet header or will launch an application if the address point to an application header.

-------------- -----------------------------------------------------------
` address `    Address of the previously loaded SCP applet or Application.
-------------- -----------------------------------------------------------

## Write Minimum Application Version


```
write-app-ver version
```
This command send the `WRITE_APP_VER` SCP command to SBL. This will setup the minimum requiered version for the customer application.

-------------- -----------------------------------------------------------
` version `    Minimum version required for the application 0xMMmmrrrr(i.e 0x01021240 for version 1.2.4672)
-------------- -----------------------------------------------------------

# Glossary

------ --------------------------------
 SBL     Secure Boot Loader           
 SCP     Secure Communication Protocol
 USN     Unique Serial Number         
 OTP     One Time Programmable Memory 
 CRK     Customer Root Key            
 MRK     Maxim Root Key			    
------ --------------------------------

