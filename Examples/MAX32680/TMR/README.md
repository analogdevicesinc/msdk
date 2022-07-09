## Description

Three timers are used to demonstrate three different modes of the general purpose timers.

1. A oneshot mode timer, Timer 4 (low-power timer) is used to create an interrupt at a freq of 1 Hz. LED 2 will toggle when the interrupt occurs.
2. Timer 3 is used to output a PWM signal on Port 1.6. The PWM frequency is 1000 Hz and the duty cycle is 50%.
3. Timer 1 is configured as 16-bit timer used in continuous mode which is used to create an interrupt at freq of 2 Hz. LED 1 will toggle when the interrupt occurs. 

Push PB1 to start the PWM and continuous timer and PB2 to start the oneshot timer.

-    PB1: P0.26/SW1
-    PB2: P0.27/SW2
-    LED 1: P0.24/LED0
-    LED 2: P0.25/LED1

## Setup

##### Required Connections
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect pins JP22(RX_SEL) and JP23(TX_SEL) to UART1 header.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Close jumper JP10 (LED0 EN).
-   Close jumper JP11 (LED1 EN).

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

