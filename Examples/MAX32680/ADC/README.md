## Description

Demonstrates the use of the ADC by continuously monitoring ADC input channel 0.  Vary the voltage on the AIN0 input (0 to 1.8V) to observe different readings from the ADC.

The example can be configured to either use a polling or interrupt driven ADC Conversion by commenting or uncommenting the "USE_INTERRUPTS" define respectively. 

Any reading that exceeds the full-scale value of the ADC will have an '*' appended to the value.

## Setup

##### Required Connections
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect pins JP22(RX_SEL) and JP23(TX_SEL) to UART1 header.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Apply an input voltage between 0 and 0.9V to pin labeled 0 of the JH11 (Analog) header.

## Expected Output

The Console UART of the device will output these messages:

```
ADC Example
0: 0x0001


0: 0x003f


0: 0x01ad


0: 0x028d


0: 0x0289


0: 0x006a
```
