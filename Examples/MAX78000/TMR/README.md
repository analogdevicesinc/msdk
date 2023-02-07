## Description

Three timers are used to demonstrate three different modes of the general purpose timers.

1. A oneshot mode timer, Timer 5 (low-power timer) is used to create an interrupt at a freq of 1 Hz. If the example is being run on the standard EV Kit, LED2 will toggle when the interrupt occurs.
2. Timer 4 is used to output a PWM signal on Port 2.5 (AIN1 pin on featherboard). The PWM frequency is 1000 Hz and the duty cycle is 50%.
3. Timer 1 is configured as 16-bit timer used in continuous mode which is used to create an interrupt at freq of 2 Hz. LED1 will toggle when the interrupt occurs. 

Push PB1/SW1 to start the PWM and continuous timers and PB2/SW2 to start the oneshot timer.

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
-   Close jumper JP1 (LED1 EN).
-   Close jumper JP2 (LED2 EN).
-   Connect Logic Analyzer or Oscilloscope to P2.4 on header JH6 to view PWM signal.

If using the MAX78000FTHR (FTHR_RevA)
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Connect Logic Analyzer or Oscilloscope to P2.4, labeled "AIN1" on header J8, to view PWM signal.

## Expected Output

The Console UART of the device will output these messages:

```
************************** Timer Example **************************

1. A oneshot mode timer, Timer 5 (low-power timer) is used to create an
   interrupt at a freq of 1 Hz. If running the example on the MAX78000EVKIT,
   LED2 will toggle when the interrupt occurs.

2. Timer 4 is used to output a PWM signal on P2.4 (labeled "AIN1" on
   MAX78000FTHR). The PWM frequency is 10 Hz and the duty cycle is 50%.

3. Timer 1 is configured as a 16-bit timer used in continuous mode.
   It creates an interrupt at freq of 2 Hz. LED1 will toggle when
   the interrupt occurs.

Push PB1 to start the PWM and continuous timers and PB2 to start the
oneshot timer.

PWM started.

Continuous timer started.

Oneshot timer started.

Oneshot timer expired!

```

You will also observe the LED behavior given in the Description section above.

