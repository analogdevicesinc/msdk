## Description

Demonstrates the use of the ADC by continuously monitoring ADC input channel 0.  Vary the voltage on the AIN0 input (0 to 1.22V) to observe different readings from the ADC.  

High and low limits are set arbitrarily to demonstrate the detection of overvoltage and undervoltage conditions respectively.  If the ADC reading exceeds 0x300, the example will report that the high limit has been reached.  If the ADC reading falls below 0x25, the example will report the low limit has been reached.

Any reading that exceeds the full-scale value of the ADC will have an '*' appended to the value.

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Connect the analog voltage input to the AIN0 pin.

## Expected Output

The Console UART of the device will output these messages:

```
***** ADC Example *****
High Limit on AIN0
0: 0x0398

Low Limit on AIN0
0: 0x0001


0: 0x01b1

High Limit on AIN0
0: 0x03ff*
...
```
