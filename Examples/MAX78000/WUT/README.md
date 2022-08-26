## Description

This example shows how to wake up a device after it is asleep with a wake up timer.  After a defined number of seconds it will wake up after going to sleep.

## Setup

##### Building Firmware:
Before building firmware you must select the correct value for _BOARD_  in "project.mk", either "EvKit\_V1" or "FTHR\_RevA", depending on the EV kit you are using to run the example.

After doing so, navigate to the directory where the example is located using a terminal window. Enter the following comand to build all of the files needed to run the example.

```
$ make
```

##### Required Connections:

If using Standard EV Kit (EvKit_V1):
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect pins 1 and 2 (P0_1) of the JH1 (UART 0 EN) header.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

If using the Featherboard (FTHR_RevA):
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

```
************** Wakeup timer example ********************
This example is to show how the Wakeup timer is used and configured.
Press SW1 to continue.
Entering SLEEP mode.
Waking up from SLEEP mode.

Press the button again to trigger another sleep-wake cycle.
Entering SLEEP mode.
Waking up from SLEEP mode.

```

