## Description

This example cycles through three typical ADC examples: a single channel conversion, temperature sensor reading, and a multichannel conversion.

The single channel conversion measures the voltage on ADC channel 3.

Unlike the single and multi channel conversions, the temperature sensor conversion uses the temperature sensor ready signal as a hardware trigger to initiate the ADC conversion. The raw ADC value mesaured in the temperature sensor conversion is printed to the terminal each time a reading is taken; on every 16th temperature reading the previous 16 temperature readings are averaged, converted to degrees celsius, and printed to the terminal.

The multi-channel ADC conversion takes readings from analog channel 3-10. 

The example can be configured to either use a polling, interrupt, or DMA driven ADC Conversions by commenting or uncommenting the "POLLING", "INTERRUPT" or "DMA" defines respectively. 

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect pins JP10(RX_SEL) and JP11(TX_SEL) to RX0 and TX0  header.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Apply input voltages between 0V and 2.048V to analog channels 3-10 on headers JH1, JH2, and JH3.

## Expected Output

The Console UART of the device will output these messages:

```
********** ADC Example **********

The example cycles trhough three typical ADC use cases:
a single channel conversion (on CH3), an internal temp
sensor reading and a multi-channel conversion (on CH3-CH10).

The example can be configured to take the measurements
by polling, using interrupts, or using DMA.

Running Single Channel Example
CH : Data
03 : 022


Running Temperature Sensor Example
CH : Data
0C : CDA
0D : 477
0E : 8AC
Average = 30.51C



Running Multi Channel Example
CH : Data
03 : 022
04 : 023
05 : 023
06 : 022
07 : 023
08 : 022
09 : 023
0A : 022
```
