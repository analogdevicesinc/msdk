## Description

Basic GPIO input, output, and interrupts are demonstrated in this example.

In this example, PB1 is continuously polled and the value read on its pin is output to LED1.  As well, an interrupt is set up on PB2; each time PB2 is pressed, LED2 is toggled.

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

The Console UART of the device will output these messages (pin numbers will may be different depending on which board the example is built for):

```
************************* GPIO Example ***********************

1. This example reads P0.18 and outputs the same state onto P0.24.
2. An interrupt is set up on P0.19. P0.25 toggles when that
   interrupt occurs.
```

You will also observe the LED behavior given in the Description section above.