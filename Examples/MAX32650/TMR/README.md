## Description

Three timers are used to demonstrate three different modes of the general purpose timers.

1. Timer 0 is used in continuous mode to create an interrupt at a frequency of 1Hz. LED0 will toggle each time the interrupt occurs.

2. Timer 1 is used to output a PWM signal on Port 3.7. The PWM frequency is 200 Hz and the duty cycle is 75%.

3. Timer 4 is used in one shot mode to illuminate LED1 after 3 sec.


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-	To see the PWM output, connect to P3.7.

## Expected Output

The Console UART of the device will output these messages:

```
************************** Timer Example **************************

1. A continuous mode timer is used to create an interrupt every 1 sec.
   LED0 (P2.25) will toggle each time the interrupt occurs.

2. Timer 1 is used to output a PWM signal on Port 3.7.
   The PWM frequency is 200 Hz and the duty cycle is 75%.

3. A one shot mode timer is used to turn on LED1 (Port 2.26) after 3 sec.

PWM started.

Continuous timer started.

Oneshot timer started.

```

