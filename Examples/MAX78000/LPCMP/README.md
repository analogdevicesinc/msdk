## Description

This example demonstrates the use of the Analog Comparator to wake up the device from sleep mode. 

The example is configured to use two analog channels as the negative and positive input to the low-power comparator. A wakeup event is triggered when the comparator output transitions from low to high (positive input needs to transition from a lower voltage than the negative input to a higher voltage than the negative input).

## Setup

##### Building Firmware:
Before building firmware you must select the correct value for _BOARD_  in "project.mk", either "EvKit\_V1" or "FTHR\_RevA", depending on the EV kit you are using to run the example.

After doing so, navigate to the directory where the example is located using a terminal window. Enter the following comand to build all of the files needed to run the example.

```
$ make
```

##### Required Connections:
If using the MAX78000EVKIT (EvKit_V1):
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect pins 1 and 2 (P0_1) of the JH1 (UART 0 EN) header.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Apply the negative comparator input to AIN0/AIN0N (JH3).
-	Apply the positive comparator input to AIN1/AIN0P (JH3).

If using the MAX78000FTHR (FTHR_RevA)
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Apply the negative comparator input to AIN6/AIN3N (RX Pin on header J8).
-	Apply the positive comparator input to AIN7/AIN3P (TX Pin on header J8).

## Expected Output

The Console UART of the device will output these messages:

```
******************** Comparator Example ********************

Connect the analog signal used as the positive comparator input
to analog pin 1 (AIN1/AIN0P).
Connect the analog signal used as the negative comparator input
to analog pin 0 (AIN0/AIN0N).

The device will be placed in sleep mode and requires a rising
edge of the comparator output to wakeup.

Press SW2 to begin.

Entering sleep mode.
Waking up.

Entering sleep mode.
Waking up.
    .
    .
    .
```
