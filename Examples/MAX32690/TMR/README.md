## Description

Three timers are used to demonstrate three different modes of the general purpose timers.

1. A oneshot mode timer, Timer 4 (low-power timer) is used to create an interrupt at a freq of 1 Hz. The device will wake up and LED2 will toggle when the interrupt occurs.
2. Timer 0 is used to output a PWM signal on Port 0.7. The PWM frequency is 1000 Hz and the duty cycle is 50%.
3. Timer 1 is configured as 32-bit timer used in continuous mode to create an interrupt at a freq of 2 Hz. LED1 will toggle when the interrupt occurs.

Each of the frequencies, clock sources, and timer instances mentioned above can be changed using the defines at the top of _main.c_.

Push SW2 to start the PWM and continuous timers initially. After the PWM and continuous timers are started, SW2 is used to run the oneshot timer.

## Setup
-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Install headers JP7(RX\_EN) and JP8(TX\_EN).
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Close jumper JP5 (LED1\_EN).
-   Close jumper JP6 (LED2\_EN).

## Expected Output

The Console UART of the device will output these messages:

```
**************************Timer Example **************************

1. A oneshot mode timer, Timer 4 (lptimer) is used to create
   an interrupt at a freq of 1 Hz. LED 2 (Port 2.12) will toggle
   when the interrupt occurs.

2. Timer 0 is used to output a PWM signal on Port 0.7.
   The PWM frequency is 1000 Hz and the duty cycle is 20%.

3. Timer 1 is configured as a 32-bit timer used in continuous
   mode which is used to create an interrupt at a freq of 2 Hz.
   LED 1 (Port 0.14) will toggle each time the oneshot timer is
   finished running.

Push SW2 to start the PWM and continuous timers initially. Then
use SW2 to run the lptimer in oneshot mode thereafter.

PWM started.

Continuous timer started.

Oneshot timer started.
```

You will also observe the LED behavior given in the Description section above.

