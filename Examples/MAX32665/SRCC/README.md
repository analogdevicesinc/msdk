## Description

The example compares the execution times required to read from MX25 RAM with the data cache enable and disabled.

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Connect the jumper (JP7) to SPIRAM.

## Expected Output

The Console UART of the device will output these messages:

```
***** SRCC Example *****
Connect the jumper (JP7) to SPIRAM.

Running test reads with data cache enabled.   Time elapsed: 0.3587
Running test reads with data cache disabled.  Time elapsed: 4.2092
Example Succeeded.
```
