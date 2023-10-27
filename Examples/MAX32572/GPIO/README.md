## Description

Basic GPIO input, output, and interrupts are demonstrated in this example.

P0.31 (PB0) is continuously scanned and whatever value is read on that pin is then output to P1.30 (LED0). An interrupt is set up on P1.29 (PB1). P1.31 (LED1) toggles when that interrupt occurs.


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the J4 (USB to UART0) connector.
-   Install P1.8 (UART0 RX EN) and P1.9 (UART0 TX EN) on header JP8.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Close jumper JP6 (LED0 EN).
-   Close jumper JP7 (LED1 EN).

## Expected Output

The Console UART of the device will output these messages:

```
************************* GPIO Example ***********************

1. This example reads P0.31 (PB0) and outputs the same state onto
   P1.30 (LED0).
2. An interrupt is set up on P0.18 (PB1). P0.19 (LED1) is toggled
   when that interrupt occurs.
```

You will also observe the LED behavior given in the Description section above.