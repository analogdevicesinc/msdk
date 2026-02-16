## Description

This example demonstrate how 1-Wire master can be configured and read slave ROM ID.


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analogdevicesinc.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the board connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Connect GND from EVK to the GND of DS2401
-   Connect V_AUX via a 1k Pull Up Resistor to the DATA (DQ) of DS2401
-   Connect 1-Wire pin in EVK (P0.8 for MAX32690 EVK) to the DATA (DQ) of DS2401

## Expected Output

```
***** 1-Wire ROM (DS2401) Example *****
This example reads ROM ID of 1-Wire slave device
Place the jumper to select V_AUX to 3.3V (JP11 in MAX32690 EVK)
Connect GND from EVK to the GND of DS2401
Connect V_AUX via a 1k Pull Up Resistor to the DATA (DQ) of DS2401
Connect 1-Wire pin in EVK (P0.8 for MAX32690 EVK) to the DATA (DQ) of DS2401


ROM ID: 01 C7 9E 14 14 00 00 D0 
Example Succeeded
```