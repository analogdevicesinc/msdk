## Description

This application cycles through the various low power modes. Switching between the different modes can be triggered by either pressing a button (PB1/SW1) or automatically with an RTC alarm. This is configured with the define statements at the top of the application.

Following modes can be tested:

 *            Active mode power with all clocks
 *            Active mode power with peripheral clocks disabled (if USE_CONSOLE is 0)
 *            SLEEP mode
 *            LPM mode
 *            UPM mode
 *            STANDBY mode
 *            BACKUP mode
 *            Power Down mode

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

If using the MAX78000FTHR (FTHR_RevA)
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

```
****Low Power Mode Example****

This code cycles through the MAX78000 power modes, using the RTC alarm to exit from each mode.  The modes will change every 4 seconds.

Running in ACTIVE mode.
Entering SLEEP mode.
Waking up from SLEEP mode.
Entering Low power mode.
Waking up from Low power mode.
Entering Micro power mode.
Waking up from Micro power mode.
Entering STANDBY mode.
Waking up from STANDBY mode.
Entering BACKUP mode.

```

