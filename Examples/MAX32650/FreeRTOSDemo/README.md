## Description

This example demonstrates a simple application of FreeRTOS. Three main tasks are supported: Task0, Task1, and vCmdLineTask. 

Task0 and Task1 operate using task delays and work with each other to blink an LED. Task0 sets a global variable to the desired state of LED0 and Task1 reads that state and sets LED0 accordingly. To protect the LED0 state global variable it is guarded with a mutex. 

vCmdLineTask takes command inputs from the terminal and executes the command (the list of available commands will be printed to the terminal by sending the command "help"). The task will wait to run until the UART interrupt handler is triggered by an incoming command. When the interrupt is triggered, the task will be notified and will process the characters received.


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
-=- 32650 FreeRTOS (V10.2.0) Demo -=-
Tickless idle is configured. Type 'tickless 1' to enable.
Starting scheduler.
Uptime is 0x00000000 (0 seconds), tickless-idle is disabled

Enter 'help' to view a list of available commands.
help
hl

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
 Take variable number of parameters,
```


