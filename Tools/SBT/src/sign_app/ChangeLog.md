---
title: "Sign App ChangeLog"
author: [Maxim Integrated]
date: "2018-09-10"
keywords: [Markdown, Example]
titlepage: false
titlepage-text-color: "00B2A9"
listings-disable-line-numbers: true
...

# Sign App ChangeLog


## 1.0.0

Initial version, from crk_sign

### 1.1.0

- FA header new options added

### 1.2.0
	
- FA header UCI options finalized

### 1.2.1

- SR_PRFSH reduced to 2 bytes (16bits) as in the spec

### 1.2.2

- Fix allocation size error for payload (forgotten strlen(arguments)), which led to output wrong bytes (dhb report)

### 1.2.3

- sdr_* parameters renamed into sdr_*

### 1.2.4

- Remove warnings 
- Add file dump under verbose=true only

### 1.2.5

- Fix bug #2262 version (4bytes) and SR_PEXT (1byte) fields added.

### 1.2.6
	
- Dynamic memory slot parameters names and lengths changed to match with flora wiki binary header format

### 1.2.7
	
- Add UCI encryption and integrity support, including source information (#2282 and flora wiki binary header format)

### 1.2.8

- ky1r removed

## 1.3.0

- build 4 display RSA key name in case of ncipher session
- Fix bug in displaying RSA key name in case of ncipher session

### 1.4.0

- correct the way to read binary file and get its size (fscanf_s shall be avoided use fread is better). now ca_sign_build.exe (with hsm) and ncipher_ca_sign_build.exe (without hsm) give the same result.
	 Adding code to manage both rsa key file format:
  -1- modulus
  -2- public exponent
  -3- private exponent
  or
  -1- modulus
  -2- private exponent
  -3- public exponent
  correct the way to read binary file and get its size (fscanf_s shall be avoided use fread is better). 
  now ca_sign_build.exe (with hsm) and ncipher_ca_sign_build.exe (without hsm) give the same result.
  resource file: version string
  add a cast for c++ build 

### 1.5.0

- support of angela/ecdsa

### 1.5.1

- minor changes

### 1.5.2

- Add support for 64-bit

### 1.5.3

- HSM modification call to UCL lib

### 1.5.4

- Add new feature #4539 : .sig generation (new parameter added in the .ini: signonly=yes|no

### 1.5.5

- secure header generation becomes an option managed by the header parameter

### 1.6.0

- Add PAOLA support for rsa 2048 and 4096 (only)
- re-indenting

### 1.6.1

- bug fix when generating PAOLA header..6.1: bug fix when generating PAOLA header.

### 1.6.2

- add "boot_method" parameter to indicate ROM Code to boot either 'cobra' or 'cmsis' ways

### 1.6.3

- Display RSA signature bug fix.

### 1.7.0

- Add "sha256", "crc32" and "none" algorithms.

### 1.7.1

- "crc32" algorithm fixed.

### 1.8.0

- Rework HSM support

### 1.8.2

- Fix element in algorithm list : typo "ras_paola" instead of "rsa_paola"

### 1.8.3 

- Specific byte in header for CRC has been changed from 0x48 to 0x44. 0x48 is for ECDSA256.
	
### 1.8.4

## 2.0.0

**Date:** December 2018

- Remove USIP compatibilities 
- Rework INI file loading
- Rework argument parsing
- Cleanup and Improve User Output
  * Add Color
  * Sort information by level (DEBUG, WARN, ERROR....)
  * Reformulate messages to be more explicit
- Add new option for getting information from env variable
- Uniformized common function with other software ( rsa, ecdsa, readfiles,.....)
- Remove global Variable
- Fix various bugs
- Improve error handling
- Remove dead code
- Add inline help
- Add user Guide
- Update copyrigth year
- Correct Static check error and warning
- Add doxygen documentation in header files

### 2.0.1

**Date:** December 2018

- Fix parse Option output message. Only signal when paramter is unknown in debug mode. 

