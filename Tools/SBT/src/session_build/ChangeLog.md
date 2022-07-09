---
title: "SCP session build ChangeLog"
subtitle: "UG98SXX"
author: [Maxim Integrated]
date: "2018-09-10"
keywords: [Markdown, Example]
titlepage: false
titlepage-text-color: "00B2A9"
listings-disable-line-numbers: true
...

# SCP session build ChangeLog
 this tool creates scp sessions packets
 for max325xx, maxq1852 and max32653

## 1.0.0

Initial version

### 1.1.0:

- Modification on hello reply and challenge including protection profile
- Modification for display bin (wb instead of w)

### 1.1.1

- Modification on adding response length in generic response
- Modification on chlg combination with pp (on rn0 and not rn15)

This version contains:
	- support for IC400D
	- RCS commands management
	
### 1.1.2

- packet for binaries frames extension

### 1.1.3

- Transaction id tr_id incremented after each response

### 1.1.4

- crypto_aes_cmac embedded to avoid dependance to libicrypto.a

### 1.1.5
	
- CHUNK_SIZE increased to 3KB
	
### 1.1.6 

- Decimal field in file name extended to 7 digits
- session size in write_mem_payload and verify_data_payload on 2 bytes..

### 1.1.7 

- verify_file added in script

### 1.1.8 
	 
- mem_mapping_payload was not called by mem_mapping, but read_conf...
- CHUNK_SIZE=15KB
- Fix two bugs in write_file: 
		- the chunk_len was incremented too early (before using chunk_len=0), so the 1st byte was always zero
		- the internal loop was performed without checking i vs data_len, which leads to an error on end of file

### 1.1.9 
	
- Fix bug 1601 & 1597: hello reply corrected.
- disconnection_request & reply used with incorrect seq number (not incremented from previous command)

### 1.2.0 
	 
- Management of scripts
	 
### 1.2.1 

- Only one process_script function; fixed-random-number mode supported.

### 1.2.2 
	 
- Fix bug #1623 (bad transaction-id) corrected
- Fix bug #1622 incorrect blpk (and fak!) values

### 1.2.3 

- Fix Bug : transaction id (tr_id) was never incremented (and not checked by the bootloader)

### 1.2.4 

- file size normalized to multiple of 16

### 1.2.5 

- MAX_SCP_COMMAND increased (buffer overflow)

### 1.2.6
 
- MAX_SCP COMMAND increased (buffer overflow: bug 1749); compilation warnings removed

### 1.2.7 

- Issue on parameters process (config file loading fgets_correction not used every time)

### 1.3.0 

- Fix bug #1757 correction; support s20 format

### 1.3.0 

- correction s20 nbbytes, seq is modulo 16

### 1.3.1 

- verbosity reduced
- #1748 correction

### 1.3.2 

- bug #1767 correction: supported flash is now only limited to 1023MB (because of 32-bit int).
	 
### 1.3.3 

- implements feature #1560: DEL-MEM and WRITE-MEM have same level of admin, so codes have been changed.


## 2.0.0 

Implements SCP for flora (specific commands; compliant with SPEC22T02 rev 0.2.0); two modes: SCP__FLORA_RSA and SCP__FLORA_AES

### 2.1.0 

- implements modified write-timeout, following SPEC22T02 rev 0.2.1 modification

### 2.1.1 

- EXECUTE CODE command added for FLORA profile

### 2.1.2

- chunk_size=4KB

### 2.1.3 

- pp=RSA forced when SCP__FLORA_RSA

### 2.1.4 

- write-crk len corrected (2 extrabytes)

### 2.1.5 

- Errors returns codes from commands processing exploited

### 2.2.0 

- Support of write-file-only and erase-file

### 2.2.1 

- Support of flora 13-byte USN

### 2.2.2 

- Control of RSA lengths to avoid bug #2031

### 2.2.3 

- Hardcoded value of hello-reply configuration was wrong (0x01); it has been changed into jtag+rwk enabled: 0xC0

### 2.2.4 

- write-otp command format help was wrong; offset set as 1st parameter

### 2.3.0 
 
 - Fix bug 2061: SCP bootloader for PCI-Linux shall use SHA256 and not SHA1

### 2.3.1 

- Fix bug 2064: timeout uart target issue

### 2.3.2 

- Add warning about secure keys handling

### 2.3.3 

- Fix bug2128

### 2.3.4 

- Fix bugs 2130, #2203, #feature 2212; timeout packet length+1 (write-timeout-payload)
- Added ptr_address_offset for an optional use of address_offset. but not finished !
- CAUTION: write-mem-config is not implemented

### 2.3.5 

 - SCP_FLORA_RSA full support

### 2.3.6 

- Fix bug 2252 (incorrect file size for verify-file); also corrected on write-file and write-only

### 2.3.7 

- The chunk_size for write-data corresponds to the whole packet length and not the data payload only


## 3.0.0 

Implements secure protocol for MAXQ1852 (ECDSA256)

### 3.1.0 

- Added transaction id field for 1852 (msp specs rev 0.93)

### 3.2.0 
	
- Added verify-crk, activate-crk commands, generate-application-startup-certificate, verify-application-startup-certificate, renamed load crk, change response ASP into AGP (msp specs rev 1.0)
	renamed SCP_MAXQ1852 into MSP_MAXQ1852

### 3.3.0
 
- MSP_MAXQ1852: SCOFFSET removed from  host packets (incorrect); size recomputed accurately (incl. ecdsa signature size) and handled for the signature.
- ipayload reduced by 3 as not including the header.

### 3.4.0 

- SCOFFSET removed from target packets (misunderstanding of the spec); signature is include lsB first in payload for MSP.

### 3.5.0 
	 
- xq,yq is lsB in load-crk and in verify-crk (it is preferred to reverse it within the application and to keep it msB in the script file).

### 3.5.1

- correction of bug #2982; ecdsafile and rsa parameters renamed in ecdsa_file and rsa_file; 
- rsamod, public_exponent and private_exponent retrieved from the possible parameters list (as the rsa key is loaded now via the rsa_file parameter).

### 3.5.2 

- Support of load-file hex for maxq1852

### 3.7.0 

- Add support for hsm

### 3.7.1 

- Add support for (lighthouse) angela SCP for test keys.

### 3.7.2

- rsa file format frozen to modulus|privexp|pubexp (consistent with crk_sign 1.2.1 and ca_sign 1.2.9).
- ecdsa signature storage in payload corrected.

### 3.7.3 
	 
- read crk file was not supporting ecdsa format (angela ecdsa).

### 3.7.4 

- ANGELA ECDSA PROFILE inserted in the HELLO-REQ.

### 3.7.5 

- Data len in write-crk ecdsa angela incorrect.

### 3.7.6. 

- Generic response for angela ecdsa includes an incorrect aes cmac

### 3.7.7 

- MAXQ1852 load-file improved

### 3.7.8 

- Fix Bug #3747: rewrite-crk added for angela and lhassa

### 3.7.9 

- Fix #3780: add vbus detect support
	 
### 3.7.10

- Printf(hello) removed in ecdsa_sign

### 3.7.11 

- Fix bug #3946: rewrite-crk including previous crk information					 

### 3.7.12

Date : 31.03.2015

- Fix Bug #1704 UCL dynamic load conflict wehn using MAX32590

### 3.7.13 

- Fix bug #4041 max packet size shall be default value, 
- Fix bug #4023 missing file shall stop execution
- Fix bug #4287 correction made in 2.3.7 for write-file applied to verify-file

### 3.7.14 

- Fix bug #4186 blank lines support in .ini.
- Fix bug #4288 extended-address management was wrong for maxq1852.
- Fix bug #4306 parameter name string in parameter value.

### 3.7.15 

- support of tiroc ( SCP-LITE ) protocol

### 3.7.16 

- TIROC - synchro pattern fixed to 2 Bytes
- TIROC - digital signature added after payload

### 3.7.17 

- Fix bug #4393: incorrect extended address management on 1852 hex file.
	
### 3.7.18 
	
- echo & dump commands added

### 3.7.19 

- Fix bug #4427 "==session_mode" missing in test for SCP_LITE_ECDSA

### 3.7.20 

- Fix #4428
- Implement Dump function in erase-data and verify-file

### 3.8.0: 
	
- support of MAX32653 (ROM code named PAOLA): SCP_PAOLA and SCP_PP_PAOLA: protocol equivalent to SCP_ANGELA_ECDSA except the RSA instead of the ECDSA; support only RSA 4096.

### 3.8.1: 

- Support PAOLA_4096 and PAOLA_2048

### 3.8.2: 
- hello request byte changed from 0x03 to 0x02 (because RSA)

### 3.8.3: 

- rewrite-crk paola mode corrected for crk1 and crk2 not having the same length

### 3.8.4: 

- kill-chip2 is previous kill-chip with USN as argument. It's different command than kill-chip.

### 3.9.0: 

- Reorganize Code
- Update HSM functions

### 3.10.0

- Fix issue length S19 and del_mem in scp
- Add OS differentiation in lib handling

### 3.10.1

- Fix hsm slot number parameter parsing in config file
- Improve HSM login sequence
- Improve build parameters

### 3.10.2

- Fix parsing of config file last line 
	
## 4.0.0

**Date:** December 2018

- Remove USIP compatibilities 
- Rework INI file loading
- Rework argument parsing
- Relayered SCP protocoles
- Cleanup and Improve User Output
  * Add Color
  * Sort information by level (DEBUG, WARN, ERROR....)
  * Reformulate messages to be more explicit
- Add new option for getting information from env variable
- Uniformized common function with other software ( rsa, ecdsa, readfiles,.....)
- Remove global Variable
- Fix various bugs
- Parse and check SCP script before processing
- Improve error handling
- Remove dead code
- Add inline help
- Add user Guide
- Add unytary tests
- Update copyrigth year
- Add new SCP commands:
  * write-deactivation
  * write-stimulus
  * write-params
  * Add new SCP target Ethernet and SPI
- Correct Static check error and warning
- Add file check before starting process
- Add doxygen documentation in header files
  



	
	

