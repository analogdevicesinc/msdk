## Description

Demonstrates the use of the ADC by continuously monitoring ADC channels in single and multi read configurations. Vary the voltage on the AIN0 input (0 to 2.048V) to observe different readings from the ADC.

The example can be configured to either use polling, interrupt, or DMA driven ADC Conversions by commenting or uncommenting the "POLLING", "INTERRUPT" or "DMA" defines respectively.

Additionally the trigger source that starts the ADC conversion can be selected as either a software or hardware trigger using the "SOFTWARE" and "HARDWARE" defines. If the hardware trigger is selected, timer 1 is set up to trigger an ADC reading every 2 seconds.


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Install headers JP7(RX\_EN) and JP8(TX\_EN).
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Apply an input voltage between 0 and 2.048V to AIN1 of the JH2 (Analog) header.

## Expected Output

The Console UART of the device will output these messages:

```
********** ADC Example **********

The voltage applied to analog pin continuously
measured and the result is printed to the terminal.

The example can be configured to take the measurements
by polling, using interrupts, or using DMA.

Note: AIN2 and AIN3 pins are shared with Console UART.

CH : Data : Voltage
01 : 1599 : 0.800V


Running Single Channel Example
CH : Data : Voltage
01 : 1518 : 0.759V


Running Multi Channel Example
CH : Data : Voltage
00 : 4095 : 2.048V
01 : 1933 : 0.967V


```
