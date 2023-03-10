## Description

A basic getting started program for the RISCV, run from the ARM core.

RV_ARM_Loader runs on the ARM core to load the RISCV code space, setup the RISCV debugger pins, and start the RISCV core with the specified application.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

* The application to load into the RISC-V core can be selected using the `RISCV_APP` option in [project.mk](project.mk).  By default, the `Hello_World` example is used.

## Expected Output

The Console UART of the device will output whatever the expected output for the `RISCV_APP`-selected project is.  By default, that's the `Hello_World` output:

```
Hello World!
count : 0
count : 1
count : 2
count : 3
```

