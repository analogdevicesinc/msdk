## Description

Demonstrates the use of MAX11261 24-bit Analog-to-Digital converter. By default the program will monitor the analog voltage value of the AIN0 input. A voltage between -Vref (-2.5V) and +Vref (2.5V) can be applied to the input pins.

Conversion results outside the convertor range will be clipped to the minimum or maximum value. The ADC can also detect if the input signal is outside reference voltage boundaries.

Any reading that exceeds the full-scale value of the ADC will have an '*' appended to the value.

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Connect the analog voltage input to the AIN0 pin. For a quick test you can short RSTN (1.8V) to AIN0.

## Expected Output

The Console UART of the device will output the voltage values in millivolts.

```
******************** MAX11261 ADC Example ********************
Demonstrates various features of MAX11261 ADC.

An input voltage between -Vref and +Vref can be applied to AIN
inputs. Conversion results for any input voltage outside this
range will be clipped to the minimum or maximum level.


    1753 mV at 21360 us
...
```
