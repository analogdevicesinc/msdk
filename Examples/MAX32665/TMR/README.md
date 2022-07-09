## Description

Two timers are used to demonstrate two different modes of the general purpose timers.

1. A continuous mode timer is used to create an interrupt at freq of 1 Hz. LED0 will toggle each time the interrupt occurs.

2. Timer 2 is used to output a PWM signal on Port 0.14. The PWM frequency is 200 Hz and the duty cycle is 75%.

3. A one shot mode timer is used to turn on LED1 after 3 sec.

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
************************** Timer Example **************************

1. A continuous mode timer is used to create an interrupt every 1 sec.
   LED0 will toggle each time the interrupt occurs.

2. Timer 2 is used to output a PWM signal on Port 0.14.
   The PWM frequency is 200 Hz and the duty cycle is 75%.

3. A one shot mode timer is used to turn on LED1 after 3 sec.

PWM started.

Continuous timer started.

Oneshot timer started.
```

