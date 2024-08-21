## Description

This example demonstrates the use of the quadrature decoder interface and outputs the position & the direction of a quadrature encoder. 

The example configures the QDEC with the Compare and Reset on MAXCNT functions. The range of the position is currently set to 0 - 255. A Compare interrupt event will occur when the position matches the the compare value (128 in the example's case). The position counter resets after reaching the maximum count (255).

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Select RX0 and TX0 on Headers JP10 and JP11 (UART 0).
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-	Connect your encoder signal pins to QEA (P0_6) and QEB (P0_7).

## Expected Output

The Console UART of the device will output these messages:

```
******** QDEC Example ********
This example shows the Compare function with Reset on
MAXCNT enabled. An interrupt is set up when a COMPARE
event and a MAXCNT event occurs.

Configure the settings to meet the specs of your application.
Connect your encoder signal pins to QEA (P0_6) and QEB (P0_7).

LED1 (P0_22) will toggle when a MAXCNT event occurs.
LED2 (P0_23) will toggle when a COMPARE event occurs.

0 - Counter-Clockwise
1 - Clockwise

Position: 0  - Direction: 0
Position: 0  - Direction: 0
Position: 0  - Direction: 0
Position: 0  - Direction: 0


```

