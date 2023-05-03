## Description

TBD<!--TBD-->


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Close jumper JP1 (LED0 EN).

## Expected Output

```
MAX32680 HART UART Example

This example transmits a basic welcome message via the HART uart
Then it enters a HART loop back mode, waiting to receive a message.
This examples requires use of another HART modem such as the HCF-Tool-35
From the HART Physical Layer test kit.
The modems may be connected across a 250Ohm load resistor.
For basic operation of the other HART modem, a terminal emulator is required.
it should be able to toggle RTS manually.  This example was tested with
Roger Meier's CoolTerm.

Setup the terminal at 1200 baud, with 1 stop bit, ODD parity, and connect.
Upon execution of this example a hello message is sent via HART then loopback.
To send via CoolTerm SET RTS via mouse click, type message, then CLEAR RTS via mouse
NOTE: Set is indicated by the RTS button color as light green, or lit.
The message will be returned with "Echo from MAX32675" appended.


Sending HART Banner Now...

```
