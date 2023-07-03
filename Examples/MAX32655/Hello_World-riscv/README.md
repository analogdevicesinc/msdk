## Description

This example is the introduction to using the RISC-V core.

The RISC-V core runs the same code as is found in the Hello_World example, with the addition of initializing the RISC-V pins and enabling the RISC-V core's instruction cache.

The ARM core initializes the RISC-V core before relinquishing control of execution to it.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

* **This combined RISC-V project contains a unique set of split Makefiles without a project.mk file.**  However, the same build configuration variables are accessible from your IDE as a standard example (see the MSDK User Guide for more details), and the example should function as a standard example.

##### Required Connections:

If using the standard EV Kit (EvKit_V1):
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

If using the Featherboard (FTHR_RevA):
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
Waiting...
Hello World!
count = 0
count = 1
count = 2
count = 3
count = 4
count = 5
...
```

