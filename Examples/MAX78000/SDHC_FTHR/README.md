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

Choose one of the following options:
0. Find the Size of the SD Card and Free Space
1. Format the Card
2. Manually Mount Card
3. List Contents of Current Directory
4. Create a Directory
5. Move into a Directory (cd)
6. Create a File of Random Data
7. Add Random Data to an Existing File
8. Delete a File
9. Format Card and Run Exmaple of FatFS Operations
10. Unmount Card and Quit
>>0
SD card mounted.
Disk Size: 7849984 bytes
Available: 7849920 bytes
Function Returned with code: FR_OK

Choose one of the following options:
0. Find the Size of the SD Card and Free Space
1. Format the Card
2. Manually Mount Card
3. List Contents of Current Directory
4. Create a Directory
5. Move into a Directory (cd)
6. Create a File of Random Data
7. Add Random Data to an Existing File
8. Delete a File
9. Format Card and Run Exmaple of FatFS Operations
10. Unmount Card and Quit
/>>1


*****THE DRIVE WILL BE FORMATTED IN 5 SECONDS*****
**************PRESS ANY KEY TO ABORT**************

FORMATTING DRIVE
Drive formatted.
SD card mounted.
SD card unmounted.
Function Returned with code: FR_OK

Choose one of the following options:
0. Find the Size of the SD Card and Free Space
1. Format the Card
2. Manually Mount Card
3. List Contents of Current Directory
4. Create a Directory
5. Move into a Directory (cd)
6. Create a File of Random Data
7. Add Random Data to an Existing File
8. Delete a File
9. Format Card and Run Exmaple of FatFS Operations
10. Unmount Card and Quit
>>2
SD card mounted.
Function Returned with code: FR_OK

Choose one of the following options:
0. Find the Size of the SD Card and Free Space
1. Format the Card
2. Manually Mount Card
3. List Contents of Current Directory
4. Create a Directory
5. Move into a Directory (cd)
6. Create a File of Random Data
7. Add Random Data to an Existing File
8. Delete a File
9. Format Card and Run Exmaple of FatFS Operations
10. Unmount Card and Quit
/>>4
Enter directory name:
Creating directory...
Directory Analog-Devices created.
Function Returned with code: FR_OK

Choose one of the following options:
0. Find the Size of the SD Card and Free Space
1. Format the Card
2. Manually Mount Card
3. List Contents of Current Directory
4. Create a Directory
5. Move into a Directory (cd)
6. Create a File of Random Data
7. Add Random Data to an Existing File
8. Delete a File
9. Format Card and Run Exmaple of FatFS Operations
10. Unmount Card and Quit
/>>5
Directory to change into:
Changed to Analog-Devices
Function Returned with code: FR_OK

Choose one of the following options:
0. Find the Size of the SD Card and Free Space
1. Format the Card
2. Manually Mount Card
3. List Contents of Current Directory
4. Create a Directory
5. Move into a Directory (cd)
6. Create a File of Random Data
7. Add Random Data to an Existing File
8. Delete a File
9. Format Card and Run Exmaple of FatFS Operations
10. Unmount Card and Quit
/Analog-Devices>>6
Enter the name of the text file:
Enter the length of the file: (256 max)
Creating file Maxim with length 200
File opened!
200 bytes written to file!
File Closed!
Function Returned with code: FR_OK

Choose one of the following options:
0. Find the Size of the SD Card and Free Space
1. Format the Card
2. Manually Mount Card
3. List Contents of Current Directory
4. Create a Directory
5. Move into a Directory (cd)
6. Create a File of Random Data
7. Add Random Data to an Existing File
8. Delete a File
9. Format Card and Run Exmaple of FatFS Operations
10. Unmount Card and Quit
/Analog-Devices>>7
Enter name of file to append:
Enter length of random data to append: (256 max)
File opened!
50 bytes written to file
File closed.
Function Returned with code: FR_OK

Choose one of the following options:
0. Find the Size of the SD Card and Free Space
1. Format the Card
2. Manually Mount Card
3. List Contents of Current Directory
4. Create a Directory
5. Move into a Directory (cd)
6. Create a File of Random Data
7. Add Random Data to an Existing File
8. Delete a File
9. Format Card and Run Exmaple of FatFS Operations
10. Unmount Card and Quit
/Analog-Devices>>3
Listing Contents of /Analog-Devices -
/Analog-Devices/Maxim

Finished listing contents
Function Returned with code: FR_OK

Choose one of the following options:
0. Find the Size of the SD Card and Free Space
1. Format the Card
2. Manually Mount Card
3. List Contents of Current Directory
4. Create a Directory
5. Move into a Directory (cd)
6. Create a File of Random Data
7. Add Random Data to an Existing File
8. Delete a File
9. Format Card and Run Exmaple of FatFS Operations
10. Unmount Card and Quit
/Analog-Devices>>8
File or directory to delete (always recursive!)
Deleted file Maxim
Function Returned with code: FR_OK

Choose one of the following options:
0. Find the Size of the SD Card and Free Space
1. Format the Card
2. Manually Mount Card
3. List Contents of Current Directory
4. Create a Directory
5. Move into a Directory (cd)
6. Create a File of Random Data
7. Add Random Data to an Existing File
8. Delete a File
9. Format Card and Run Exmaple of FatFS Operations
10. Unmount Card and Quit
/Analog-Devices>>9


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
Message: Vvt.BR0Vy4#YwqiNYj'jYnX8j7ePtuzJO?t-sTCGvibwYn81?Sutq'Q0s7udLTie5QZ-3d0mo6Hk5wL1z3!8GQFLbTYGQ!xtNwGI,eWiCJMbdqf7Ko?3T4dd7?Di4''4Cf2w.I?UPiSa'xfuMi5PV8Tn!ZJc83MccYv7BtHJ'VWcw#qbSh8rsqgi!EjFX3jbXhx8--6ZY6jvd1DmG5iAN,etVm3vtRVhr2Mgml3jJ?d.a0tjwx,lXT7.e,WC!wJ4
File Closed!
Function Returned with code: FR_OK

Choose one of the following options:
0. Find the Size of the SD Card and Free Space
1. Format the Card
2. Manually Mount Card
3. List Contents of Current Directory
4. Create a Directory
5. Move into a Directory (cd)
6. Create a File of Random Data
7. Add Random Data to an Existing File
8. Delete a File
9. Format Card and Run Exmaple of FatFS Operations
10. Unmount Card and Quit
>>10
SD card unmounted.
Function Returned with code: FR_OK
End of example, please try to read the card.
```

