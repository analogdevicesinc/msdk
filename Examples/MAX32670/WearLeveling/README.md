## Description

In this example, the MAX32670 accepts commands from the user which demonstrate various features of the LittleFS library, including flash wear leveling. User's may send commands to the device with their terminal application.

Below is a list of the supported commands:
* help: Prints out the list of available commands and describes how each command is used.
* stop: Ends the example.
* read: Reads data from a file and prints it to the terminal.
* write: Writes a characterstring to a file
* swl: Stands for "show wear leveling". This command performs a specified number of writes (passed as an argument on the command line) to a test file and prints out the number of times each filesystem block was written to. Users should see the writes distributed somewhat evenly across most filesystem blocks. 
	
Enter "help" in the command line to see more details on the usage of each of the commands including what arguments/options need to be specified to successfully execute each command.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analogdevicesinc.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

* This project comes pre-configured for the MAX32670EVKIT.  See [Board Support Packages](https://analogdevicesinc.github.io/msdk/USERGUIDE/#board-support-packages) in the UG for instructions on changing the target board.

## Required Connections

-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Select RX0 and TX0 on Headers JP1 and JP3 (UART 0).
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Close jumper JP1 (LED1 EN).
-   Close jumper JP2 (LED2 EN).

## Expected Output

The Console UART of the device will output these messages:

```
*************** Wear Leveling Example ***************
Mounting the file system...
littlefs/lfs.c:1224:error: Corrupted dir pair at {0x0, 0x1}
Filesystem is invalid, formatting...
File system is mounted!

CLI Initialized! Enter 'help' to see a list of available commands.

$ help
help

stop:
  Usage: stop
  Description: Ends the example

read:
  Usage: read <filename> <number of bytes> <location>
  Description: Reads data from a specific location within a file.

write:
  Usage: write <filename> <character string> <location>
  Description: Writes a character string to a specific location within a file.

swl:
  Usage: swl <number of writes>
  Description: Stands for "show wear leveling." This command writes to a file
    the specified number of times. Once all writes have completed, the number
    of times each flash page (filesystem block) was written to is printed to
    the terminal. (Writes should be distributed somewhat evenly across many
    filesystem blocks.) This command may take a while to complete. LED0 is
    used as a heartbeat while the command is executing.

$ write demo_file thisisanexampledatastringtowritetodemofile 0
write demo_file thisisanexampledatastringtowritetodemofile 0
42 bytes were written to demo_file in filesystem block 6.

$ read demo_file 42 0
read demo_file 42 0
42 bytes were read from demo_file in filesystem block 6.
The following string was read from file demo_file:
thisisanexampledatastringtowritetodemofile

$ swl 1000
swl 1000
All writes have completed. Here are the results:
Block 0 was written to 0 times.
Block 1 was written to 0 times.
Block 2 was written to 83 times.
Block 3 was written to 83 times.
Block 4 was written to 42 times.
Block 5 was written to 42 times.
Block 6 was written to 0 times.
Block 7 was written to 84 times.
Block 8 was written to 84 times.
Block 9 was written to 84 times.
Block 10 was written to 83 times.
Block 11 was written to 83 times.
Block 12 was written to 83 times.
Block 13 was written to 83 times.
Block 14 was written to 83 times.
Block 15 was written to 83 times.


$ stop
stop

$
Filesystem resources released.
Example complete!
```

