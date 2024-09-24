## Description

A basic getting started program for the RISCV, run from the ARM core.

RV_ARM_Loader runs on the ARM core to load the RISCV code space, setup the RISCV debugger pins, and start the RISCV core with the specified application.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analogdevicesinc.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

* The application to load into the RISC-V core can be selected using the `RISCV_APP` option in [project.mk](project.mk).  By default, the `Hello_World` example is used.

## Required Connections

-   Connect a MAXPICO or other supported Debug adapter to the SWD Connector.
-   Connect a USB cable between the PC and the CN2 (USB/UART) connector.
-   Connect the 5V power cable at (5V IN).
-   Close jumper (RX - P0.0) and (TX - P0.1) at Headers JP23 (UART 0 EN).
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Close jumper JP1 (LED1 EN).
-   Close jumper JP2 (LED2 EN).

## Expected Output

The Console UART of the device will output whatever the expected output for the `RISCV_APP`-selected project is.  By default, that's the `Hello_World` output:

```
Hello World!
count : 0
count : 1
count : 2
count : 3
```
