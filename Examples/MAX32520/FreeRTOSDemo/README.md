## Description

TBD<!--TBD-->


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Select RX SEL and TX SEL on headers JP7 and JP8.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
-=- MAX32520 FreeRTOS (V10.2.0) Demo -=-
Tickless idle is configured. Type 'tickless 1' to enable.
Starting scheduler.
Uptime is 0x00000000 (0 seconds), tickless-idle is disabled

Enter 'help' to view a list of available commands.
cmd> help

help:
 Lists all the registered commands


ps:
 Displays a table showing the state of each FreeRTOS task


uptime:
 Displays the uptime of the FreeRTOS system


tickless <0/1>:
 Disable (0) or enable (1) tick-less operation


echo_3_parameters <param1> <param2> <param3>:
 Expects three parameters, echos each in turn


echo_parameters <...>:
 Take variable number of parameters, echos each in turn

cmd>
```