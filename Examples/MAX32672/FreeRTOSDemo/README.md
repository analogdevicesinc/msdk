## Description

This example demonstrates thye basic functionality of FreeRTOS. There are four main tasks running: Task0, Task1, TickTockTask, and CmdLineTask.

Task0 and Task1 each flash an LED at a rate of 1Hz. Task0 controls LED0 and Task1 controls LED1. To ensure there is not a conflict when the tasks modify the GPIO registers, they each must first gain control over a shared semaphore before making any changes to those registers.

The TickTockTask simply prints the FreeRTOS tick count to the console once per minute.

And finally, the CmdLineTask demostrates FreeRTOS's command line interface feature. The available commands are:
- ps: Displays a table showing the state of each FreeRTOS task
- uptime: Displays the uptime of the FreeRTOS system
- tickless: Enables or disables tick-less operation when configUSE_TICKLESS_IDLE is set to 1
- echo_3_parameters: Expects three parameters and echos each if command is valid
- echo_parameters: Take variable number of parameters and echos each if command is valid

For more information on how to input these commands enter "help" into the serial terminal.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

Tickless idle may configured by uncommenting this line in *FreeRTOSConfig.h.*

```
#define configUSE_TICKLESS_IDLE     1
```

## Required Connections

-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Select RX0 and TX0 on Headers JP10 and JP11 (UART 0).
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
-=- MAX32672 FreeRTOS (V10.2.0) Demo -=-
Starting scheduler.
Uptime is 0x00000000 (0 seconds), tickless-idle is disabled

Enter 'help' to view a list of available commands.
cmd>
```
