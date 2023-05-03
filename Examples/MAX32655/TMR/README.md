## Description

Three timers are used to demonstrate three different modes of the general purpose timers.

1. A oneshot mode timer, Timer 4 (low-power timer) is used to create an interrupt at a freq of 1 Hz. LED 2 will toggle when the interrupt occurs.
2. Timer 3 is used to output a PWM signal on Port 1.6. The PWM frequency is 1000 Hz and the duty cycle is 50%.
3. Timer 1 is configured as 16-bit timer used in continuous mode which is used to create an interrupt at freq of 2 Hz. LED 1 will toggle when the interrupt occurs. 

Push PB1 to start the PWM and continuous timer and PB2 to start the oneshot timer.

On the standard EV Kit:
-    PB1: P0.18/SW3
-    PB2: P0.19/SW4
-    LED 1: P0.24/LED0
-    LED 2: P0.25/LED1

On the Featherboard:
-    PB1: P0.2/SW2
-    PB2: P0.3/SW3
-    LED 1: P0.18/Red LED
-    LED 2: P0.19/Green LED


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Setup

##### Board Selection
Before building firmware you must select the correct value for _BOARD_  in "project.mk", either "EvKit\_V1" or "FTHR\_Apps\_P1", depending on the board version you are using to run the example.

##### Required Connections
If using the Standard EV Kit (EvKit_V1):
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect pins JP4(RX_SEL) and JP5(TX_SEL) to RX0 and TX0  header.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Close jumper JP1 (LED0 EN).
-   Close jumper JP2 (LED1 EN).

If using the Featherboard (FTHR\_Apps\_P1):
-   Connect a USB cable between the PC and the J4 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the board's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
************************** Timer Example **************************

1. A oneshot mode timer, Timer 4 (lptimer) is used to create an
   interrupt at a freq of 1 Hz. LED 2 (P0.25) will toggle when the
   interrupt occurs.

2. Timer 3 is used to output a PWM signal on Port 1.6.
   The PWM frequency is 1000 Hz and the duty cycle is 50%.

3. Timer 1 is configured as 16-bit timer used in continuous mode
   which is used to create an interrupt at freq of 2 Hz.
   LED 1 (P0.24) will toggle when the interrupt occurs.

Push PB1 to start the PWM and continuous timer and PB2 to start the
   oneshot timer.

PWM started.

Continuous timer started.
```

You will also observe the LED behavior given in the Description section above.

