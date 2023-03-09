## Description

Demonstrates the use of the ADC by continuously monitoring ADC input channel 2.  Vary the voltage on the AIN2 input (0 to 1.25V) to observe different readings from the ADC.

The example can be configured to either use a polling, interrupt, or DMA driven ADC Conversions by commenting or uncommenting the "POLLING", "INTERRUPT" or "DMA" defines respectively. 

Any reading that is invalid will have an '*' appended to the value.


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Setup

##### Required Connections
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect pins JP10(RX_SEL) and JP11(TX_SEL) to RX0 and TX0  header.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Apply an input voltage between 0 and 1.25V to pin labeled 2 of the JH1 (Analog) header.

## Expected Output

The Console UART of the device will output these messages:

```
********** ADC Example **********

The voltage applied to analog pin 2 is continuously
measured and the result is printed to the terminal.

The example can be configured to take the measurements
by polling, using interrupts, or using DMA.

2: 0x0001


2: 0x003f


2: 0x01ad


2: 0x028d


2: 0x0289


2: 0x006a
```
