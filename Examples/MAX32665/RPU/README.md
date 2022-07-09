## Description

TBD<!--TBD-->

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
********* Resource Protection Unit Example **********
This example uses the resource protection unit to prevent
code running in Core 0 from accessing the timer in use by Core 1



Hard Fault reached
Press reset to run the example again
Example Complete
```