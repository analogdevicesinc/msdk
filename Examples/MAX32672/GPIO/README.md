## Description

Basic GPIO input, output, and interrupts are demonstrated in this example.

An interrupt is set up to trigger when the button (SW3, P0.18) is pressed. When that interrupt occurs LED1 (D2, P0.23) will toggle. Additionally, the button state is constantly being polled, whenever the button is pressed and held LED0 (D2, P0.22) will illuminate.



## Required Connections

-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Select RX0 and TX0 on Headers JP10 and JP11 (UART 0).

## Expected Output

The Console UART of the device will output these messages:

```
************************* GPIO Example ***********************

This example uses the push button (SW3, P0.18) to demonstrate some of the
   functions of the GPIO peripheral. An interrupt is set up to trigger
   when the button is pressed. When that interrupt occurs LED1 (D2, P0.23)
   will toggle. Additionally, the button state is constantly being polled
   whenever the button is pressed and held LED0 (D2, P0.22) will illuminate.
```

You will also observe the LED behavior given in the Description section above.