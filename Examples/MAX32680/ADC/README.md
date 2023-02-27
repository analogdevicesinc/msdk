## Description

Demonstrates the use of the ADC by continuously monitoring ADC input channel 4.  Vary the voltage on the AIN12 input (0 to 1.8V) to observe different readings from the ADC.

The example can be configured to either use a polling or interrupt driven ADC Conversion by commenting or uncommenting the "USE_INTERRUPTS" define respectively. 

Any reading that exceeds the full-scale value of the ADC will have an '*' appended to the value.

## Setup

##### Required Connections
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect pins JP22(RX_SEL) and JP23(TX_SEL) to UART1 header.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Apply an input voltage between 0 and 1.8V to pin labeled 4 of the JH5 (Analog) header.

## Expected Output

The Console UART of the device will output these messages:

```
ADC Example
 Low Limit on AIN12
12: 0x0009

 Low Limit on AIN12
12: 0x001b

 Low Limit on AIN12
12: 0x000c

 Low Limit on AIN12
12: 0x001b

 Low Limit on AIN12
12: 0x0242


12: 0x01da

 Low Limit on AIN12
12: 0x03ff

 High Limit on AIN12
12: 0x03ff

 High Limit on AIN12
12: 0x03ff
```
