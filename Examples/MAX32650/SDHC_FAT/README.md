## Description

This example demonstrates the SDHC FAT Filesystem. The terminal prompts with a list of user-selectable tasks to run on the inserted Micro SD Card.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analogdevicesinc.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-	Insert MicroSD card into SDHC slot on the EV Kit.

## Expected Output

```
***** MAX32650 SDHC FAT Filesystem Example *****
Card inserted.
Card Initialized.
Card type: SDHC
SDHC Ready!
CLI Initialized! Enter 'help' to see a list of available commands.

$ help
help

size:
  Usage: size
  Description: Find the Size of the SD Card and Free Space


format:
  Usage: format
  Description: Format the Card


mount:
  Usage: mount
  Description: Manually Mount Card


ls:
  Usage: ls
  Description: list the contents of the current directory


mkdir:
  Usage: mkdir <directory name>
  Description: Create a directory


file_create:
  Usage: file_create <file name> <number of bytes to add>
  Description: Create a file of random data


cd:
  Usage: cd <directory name>
  Description: Move into a directory


add_data:
  Usage: add_data <file name> <number of bytes to add>
  Description: Add random Data to an Existing File


del:
  Usage: del <file name>
  Description: Delete a file


fatfs:
  Usage: fatfs
  Description: Format Card and Run Example of FatFS Operations


unmount:
  Usage: unmount
  Description: Unmount card


$ format
format


*****THE DRIVE WILL BE FORMATTED IN 5 SECONDS*****
**************PRESS ANY KEY TO ABORT**************

FORMATTING DRIVE
Drive formatted.
SD card mounted.
SD card unmounted.

$ size
size
SD card mounted.
Disk Size: 31162880 bytes
Available: 31162848 bytes

$ mount
mount
SD card mounted.

$ mkdir Analog_devices
mkdir Analog_devices
Creating directory...
Directory Analog_devices created.

$ cd Analog_Devices
cd Analog_Devices
Changed to Analog_Devices

$ file_create ADI 30
file_create ADI 30
Creating file ADI with length 30
File opened!
30 bytes written to file!
File Closed!

$ add_data ADI 30
add_data ADI 30
File opened!
30 bytes written to file
File closed.

$ ls
ls
Listing Contents of /Analog_devices -
/Analog_devices/ADI

Finished listing contents

$ del ADI
del ADI
Deleted file ADI

$ fatfs
fatfs


*****THE DRIVE WILL BE FORMATTED IN 5 SECONDS*****
**************PRESS ANY KEY TO ABORT**************

FORMATTING DRIVE
Drive formatted.
SD card mounted.
SD card unmounted.
SD card mounted.
SD Card Opened!
File opened!
256 bytes written to file!
File Closed!
Creating Directory...
Renaming File...
Attempting to read back file...
Read Back 256 bytes
Message: dwfl-owoXGcTZ,5z,Sy8lfsNqDGrzio'O6vntRMoWODcIKP!C'y7tF.'W88ZjR81BpiibPhokQfa3w'cvmnr0EgE1MNDIhXKfBJGP6b?0tvHEPK-WNc7yuPdFNL6FPq10',Q,GSf3jSyY?MU0wv'FToTI!ct.E6Q4nbVuavg6h'48D5sR5mcepxf1l!MesddI7aZ9s?KIVnybRwZ.UBJpX1b?5oXP9wLKZcgW-k,gZ5HMIMwAcy!n9S?E57m0zvI
File Closed!

$ unmount
unmount
SD card unmounted.

$
```

