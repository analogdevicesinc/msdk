## Description

This application demonstrates receiving data from the microphone on the MAX78000 EV Kits using the I2S and DMA modules. Once valid data is received, a status message is printed confirming the example succeeded. 

## Setup
##### Building Firmware:
Before building firmware you must select the correct value for _BOARD_  in "Makefile", either "EvKit\_V1" or "FTHR\_RevA", depending on the EV kit you are using to run the example.

After doing so, navigate to the directory where the example is located using a terminal window. Enter the following comand to build all of the files needed to run the example.

```
$ make
```

##### Required Connections:
If using the standard EV Kit (EvKit_V1):
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect pins 1 and 2 (P0_1) of the JH1 (UART 0 EN) header.
-	Select "INT" with jumper J20 to supply the clock signal to the I2S module.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

If using the Featherboard (FTHR_RevA):
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
***** I2S Receiver Example *****

Microphone enabled!
Receiving microphone data!
```

