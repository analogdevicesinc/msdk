## Description

Three timers are used to demonstrate three different modes of the general purpose timers.

1. A oneshot mode timer, Timer 4 (low-power timer) is used to create an interrupt at a freq of 1 Hz. LED 2 will toggle when the interrupt occurs.
2. Timer 3 is used to output a PWM signal on Port 1.6. The PWM frequency is 1000 Hz and the duty cycle is 50%.
3. Timer 1 is configured as 16-bit timer used in continuous mode which is used to create an interrupt at freq of 2 Hz. LED 1 will toggle when the interrupt occurs. 

Push PB1 to start the PWM and continuous timer and PB2 to start the oneshot timer.


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analogdevicesinc.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections
-   Connect a USB cable between the PC and the J1 (PWR-OBD-UART0) connector.
-   Connect pins JP19 (OBD VCOM EN) RX and TX header.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

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

