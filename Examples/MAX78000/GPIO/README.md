## Description

Basic GPIO input, output, and interrupts are demonstrated in this example.

##### Standard EV Kit (EvKit_V1):
	
P2.6 (PB1) is continuously scanned and whatever value is read on that pin is then output to P0.2 (LED1).  An interrupt is set up on P2.7 (PB2). P0.3 (LED2) toggles when that interrupt occurs.

##### Featherboard (FTHR_RevA):
	
P1.7 (SW2) is continuously scanned and whatever value is read on that pin is then output to P2.0 (Red LED in RGB LED). An interrupt is set up on P0.2 (SW1). P0.9 (the SDIO3 pin on header J4) toggles when that interrupt occurs.

## Setup
##### Building Firmware:
Before building firmware you must select the correct value for _BOARD_  in "Makefile", either "EvKit\_V1" or "FTHR\_RevA", depending on the EV kit you are using to run the example.

After doing so, navigate to the directory where the example is located using a terminal window. Enter the following comand to build all of the files needed to run the example.

```
$ make
```

##### Required Connections:

If using the standard (EvKit_V1):
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect pins 1 and 2 (P0_1) of the JH1 (UART 0 EN) header.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Close jumper JP1 (LED1 EN).
-   Close jumper JP2 (LED2 EN).

If using the featherboard (FTHR_RevA):
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Connect a logic analyzer to the SDIO3 pin on header J4 to observe the pin toggling on each SW1 button press.

## Expected Output

The Console UART of the device will output these messages:

```
***** GPIO Example *****

1. This example reads P2.6 (PB1 input) and outputs the same state onto
   P0.2 (LED1).
2. An interrupt is set up on P2.7 (PB2 input). P0.3 (LED2) toggles when
   that interrupt occurs.
```

You will also observe the LED behavior given in the Description section above.