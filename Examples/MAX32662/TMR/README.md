## Description

Three timers are used to demonstrate three different modes of the general purpose timers.

1. A oneshot mode timer, Timer 2 is used to create an interrupt at a freq of 1 Hz. It prints the status of when it is started and when it expires. Additionally P0.7 is toggled immediately before the timer is started and immediately after. 
2. Timer 0 is used to output a PWM signal on P0.3. The PWM frequency is 10 Hz and the duty cycle is 25%.
3. Timer 2 is configured as a 16-bit timer used in continuous mode which generates an interrupt at a frequency of 2 Hz. LED0 (P0.14) will toggle when the interrupt occurs. Additionally the timer output is viewable on P0.5.

Push SW2 to start the oneshot timer.

## Required Connections
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Install headers JP7(RX\_EN) and JP8(TX\_EN).
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
************************** Timer Example **************************

This example demonstrates the following timer modes:
1. Continuous timer: this timer is used to generate an interrupt at a
   frequency of 2 Hz. In the continuous timer ISR LED0 is toggled.
   Additionally, the continuous timer output signal is enabled and is
   viewable on pin P0.5.

2. PWM Timer:  this timer is used to output a PWM signal on pin P0.3.
   The PWM frequency is 10 Hz and the duty cycle is 25%.

3. Oneshot Timer:  this timer is configured in oneshot mode. Pressing
   the pushbutton (SW2) will start the timer and P0.7 will be set high.
   After the oneshot timer expires a message will be printed to the
   console and P0.7 will be cleared.

Continuous timer started.

PWM started.

Oneshot timer started.

Oneshot timer expired!
```

