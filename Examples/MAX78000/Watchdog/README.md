## Description

A demonstration of the windowed features of the watchdog timer.

When the application begins, it initializes and starts the watchdog timer.  The application then begins to reset the watchdog within the allowed window.  Use SW2 and SW3 on the Standard EV Kit or SW1 and SW2 on the Featherboard to control if and when the application attempts to reset the timer.

- Push SW1 (Featherboard) or SW2 (Standard EV Kit) to trigger a "too-late" watchdog reset. This will stop reseting the watchdog timer until it generates the "too-late" interrupt.  After that it will reset the watchdog timer only once, allowing it to pass the reset timeout period.
- Push SW2 (Featherboard) or SW3 (Standard EV Kit) to reset the watchdog timer in the "too-early" period.

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
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

If using the MAX78000FTHR (FTHR_RevA)
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages if SW1 (Featherboard) or SW2 (Standard EV Kit) is pressed:

```
************** Watchdog Timer Demo ****************
SW1: Push SW1 to trigger a "too-late" watchdog reset. This will stop resetting
     the watchdog timer until it generates the "too-late" interrupt.  After that
     it will reset the watchdog timer only once, allowing it to pass the reset
     timeout period.

SW2: Push SW2 to reset the watchdog timer in the "too-early" period.

Enabling Timeout Interrupt...

Watchdog has tripped!
This is the first time, so we'll go ahead and reset it
once it is within the proper window.

Watchdog has tripped!
This is the not the first time.  What happens if we
do not reset it?

Watchdog Reset occured too late (OVERFLOW)
```

The Console UART of the device will output these messages if SW2 (Featherboard) or SW3 (Standard EV Kit) is pressed:

```
************** Watchdog Timer Demo ****************
SW1: Push SW1 to trigger a "too-late" watchdog reset. This will stop resetting
     the watchdog timer until it generates the "too-late" interrupt.  After that
     it will reset the watchdog timer only once, allowing it to pass the reset
     timeout period.

SW2: Push SW2 to reset the watchdog timer in the "too-early" period.
What happens if the watchdog is reset too early?

Watchdog Reset occured too soon (UNDERFLOW)
```

