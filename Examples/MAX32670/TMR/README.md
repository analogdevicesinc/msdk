## Description

Three timers are used to demonstrate three different modes of the general purpose timers.

1. A continuous mode timer is used to create an interrupt at freq of 4 Hz. LED0 (Port 0.22) will toggle each time the interrupt occurs.

2. Timer 0 is used to output a PWM signal on Port 0.5. The PWM frequency is 1000 Hz and the duty cycle is 50%.

3. Timer 1 is configured as 16-bit timer used in oneshot mode which is used to create an interrupt at freq of 1 Hz. LED1 (Port 0.23) will toggle when the interrupt occurs.

Push PB1 to start the timers.


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Select RX0 and TX0 on Headers JP1 and JP3 (UART 0).
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Close jumper JP1 (LED0 EN).
-   Close jumper JP2 (LED1 EN).

## Expected Output

The Console UART of the device will output these messages:

```
************************** Timer Example **************************

1. A continuous mode timer is used to create an interrupt at freq of 4 Hz.
   LED0 (Port 0.22) will toggle each time the interrupt occurs.

2. Timer 0 is used to output a PWM signal on Port 0.5.
   The PWM frequency is 1000 Hz and the duty cycle is 50%.

3. Timer 1 is configured as 16-bit timer used in oneshot mode
   which is used to create an interrupt at freq of 1 Hz.
   LED1 (Port 0.23) will toggle when the interrupt occurs.

PWM started.

Continuous timer started.

Oneshot timer started.
```

You will also observe the LED behavior given in the Description section above.

