## Description

In this example, the MAX78002 accepts commands from the user which demonstrate various features of the LittleFS library, including flash wear leveling. User's may send commands to the device with their terminal application.

Below is a list of the supported commands:
* help: Prints out the list of available commands and describes how each command is used.
* stop: Ends the example.
* read: Reads data from a file and prints it to the terminal.
* write: Writes data to a file and can optionally create the file to write to if it does not already exist.
* swl: Stands for "show wear leveling". This command performs a specified number of writes (passed as an argument on the command line) to a test file and prints out the number of times each filesystem block was written to. Users should see the writes occur somewhat evenly across most filesystem blocks. 
	
Enter "help" in the command line to see more details on the usage of each of the commands including what arguments/options need to be specified to successfully execute each command.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

* This project comes pre-configured for the MAX78002EVKIT.  See [Board Support Packages](https://analog-devices-msdk.github.io/msdk/USERGUIDE/#board-support-packages) in the UG for instructions on changing the target board.

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Connect the 5V power cable at (5V IN).
-   Close jumper (RX - P0.0) and (TX - P0.1) at Headers JP23 (UART 0 EN).
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Close jumper JP1 (LED1 EN).
-   Close jumper JP2 (LED2 EN).

## Expected Output

The Console UART of the device will output these messages:

```
********** Wear Leveling Example **********
Mounting the filesystem...
C:/MaximSDK/Libraries/littlefs/lfs.c:1224:error: Corrupted dir pair at {0x0, 0x1}
Filesystem is invalid, formatting...
Filesystem is mounted! Ready for commands.

cmd> help

The available commands are:
  * help
       Description: Prints out list of available commands.
       Usage: help

  * stop
       Description: Ends the example.
       Usage: stop

  * read
       Description: Reads data from a specific location within a file. If
                    the read is successful, the data read is printed to the
                    terminal.
       Usage: read <filename> <number of bytes to read> <location>

  * write
       Description: Writes a character string to a specific location within
                    a file.
       Usage: write (--create) <filename> <character string> <location>
       Options:
          --create: Creates file <filename> if it does not already exist.
  * swl
       DDescription: Stands for "show wear leveling". Writes to a file the
                     specified number of times. Once all writes have completed,
                     the number of times each flash page (filesystem block)
                     was written to is printed to the terminal. This command may
                     take a while to complete. LED0 is used as a heartbeat while
                     the command is executing.
       Usage: swl <number of writes>


cmd> write --create demo_file thisisanexampledatastringtowritetodemofile 0
42 bytes were written to demo_file in filesystem block 6.

cmd> read demo_file 42 0
42 bytes were read from demo_file in filesystem block 6.
The following string was read from file demo_file:
thisisanexampledatastringtowritetodemofile

cmd> swl 1000
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


cmd> stop

Filesystem resources released.
Example complete!
```

