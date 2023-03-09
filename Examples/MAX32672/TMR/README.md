## Description

Three timers are used to demonstrate three different modes of the general purpose timers.

1. Timer 3 is used in continuous mode to generate an interrupt at a frequency of 4 Hz. LED0 will toggle each time the interrupt
   is triggered.
2. Timer 2 is used to output a PWM signal on Port 0.5. The PWM frequency is 1000 Hz and the duty cycle is 75%.
3. Timer 1 is configured as a 16-bit timer in oneshot mode that generates an interrupt at a frequency of 1 Hz. LED1 will toggle when the oneshot interrupt is triggered.

LED pins on EvKit board:  LED0:P0.22  LED1:P0.23
LED pins on FTHR  board:  LED0:P0.2   LED1:P0.3

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Select RX0 and TX0 on Headers JP10 and JP11 (UART 0).
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
************************** Timer Example **************************

1. Timer 3 is used in continuous mode to generate an interrupt at
   a frequency of 4 Hz. LED0 will toggle each time the interrupt
   is triggered.

2. Timer 2 is used to output a PWM signal on Port 0.5. The PWM
   frequency is 1000 Hz and the duty cycle is 75%.

3. Timer 1 is configured as a 16-bit timer in oneshot mode that
   generates an interrupt at a frequency of 1 Hz. LED1 will toggle
   when the oneshot interrupt is triggered.

PWM started.

Continuous timer started.

Oneshot timer started.
```

You will also observe the LED behavior given in the Description section above.

