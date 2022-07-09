## Description

Two timers are used to demonstrate two different modes of the general purpose timers.

1. A continuous mode timer is used to create an interrupt at freq of 1 Hz. LED0 (Port 1.06) will toggle each time the interrupt occurs.

2. Timer 0 is used to output a PWM signal on Port 1.0. The PWM frequency is 200 Hz and the duty cycle is 75%.

## Required Connections

-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect a USB cable between the PC and the CN1 (USB/UART0) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
************************** Timer Example **************************

1. A continuous mode timer is used to create an interrupt every 1 sec.
   LED0 (Port 3.05) will toggle each time the interrupt occurs.

2. Timer 0 is used to output a PWM signal on Port 1.14.
   The PWM frequency is 200 Hz and the duty cycle is 75%.

PWM started.

Continuous timer started.
```

