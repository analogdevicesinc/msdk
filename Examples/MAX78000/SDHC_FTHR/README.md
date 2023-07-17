## Description

This example demonstrates the SDHC FAT Filesystem. The terminal prompts with a list of user-selectable tasks to run on the inserted Micro SD Card.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

* This project comes pre-configured for the MAX78000FTHR.  See [Board Support Packages](https://analog-devices-msdk.github.io/msdk/USERGUIDE/#board-support-packages) in the UG for instructions on changing the target board.

## Required Connections

-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Insert the SD card into the Micro SD Card Connector.

## Expected Output

```
***** MAX78000 SDHC FAT Filesystem Example *****
Card inserted.
$ Help

Size --> Find the Size of the SD Card and Free Space
Format --> Format the Card
Mount --> Manually Mount Card
ls --> list the contents of the current directory
mkdir --> Create a directory
file_create --> Create a file of random data
cd --> Move into a directory
add_data --> Add random Data to an Existing File
Del --> Delete a file
FatFs --> Format Card and Run Example of FatFS Operations
Unmount --> Unmount card
Help --> Prints a help message with info about all of the supported commands.
$ SIZE
SD card mounted.
Disk Size: 31163072 bytes
Available: 31163008 bytes
$ Format


*****THE DRIVE WILL BE FORMATTED IN 5 SECONDS*****
**************PRESS ANY KEY TO ABORT**************

FORMATTING DRIVE
Drive formatted.
SD card mounted.
SD card unmounted.
$ mount
SD card mounted.
$ mkdir Analog_Devices
Creating directory...
Directory Analog_Devices created.
$ cd Analog_Devices
Changed to Analog_Devices
$ file_create Maxim 30
Creating file Maxim with length 30
File opened!
30 bytes written to file!
File Closed!
$ add_data Maxim 25
File opened!
25 bytes written to file
File closed.
$ ls
Listing Contents of 0:/Analog_Devices -
0:/Analog_Devices/Maxim

Finished listing contents
$ Del Maxim
Deleted file Maxim
$ fatfs


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
Message: StiUH7-HxP!fffB,q.DqMnI2nvSnchst.ZaLTpG'w8I7Tg0,l6VVkEsehp#IHSMZN94WDc'N#-0qkBlAil,'#DvMJZ!zzf,j?Lm,H5cbfFfVNHUlPF9GsTWbrop0EG7VV57qZZzjdvzJH5Xh2'82'fz9t#R,kzoaqYBJVRhrlD5W1mZItggYqyIICUvWOOppJQIVvt.BR0Vy4#YwqiNYj'jYnX8j7ePtuzJO?t-sTCGvibwYn81?Sutq'Q0s7udL
File Closed!
$ unmount
SD card unmounted.
```

