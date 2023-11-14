## Description

A basic getting started program for RISC-V core.

This version of Hello_World prints an incrementing count to the console UART and toggles a GPIO 
once every 500 ms. The same application runs simultaneously on ARM core and on RISC-V core.
Both cores use the same console UART but different LEDs.
ARM core will blink LED0 (GPIO pin P1.30), RISC-V core will blink LED1 (GPIO P1.31).

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the J4 (USB TO UART0/PWR) connector on EV KIT board. 
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

There might be delay for up to 18 sec between powering up and starting code execution,
due to secure boot verification process.
The Console UART of the device will output these messages:
```

ARM: ***********Hello World!***********
ARM: LED0 on P1.30 toggles every 500 ms

ARM: Starting RISC-V at 0x20068000, Nov 10 2023 12:22:59

RISCV: ***********Hello World!***********
RISCV: LED1 on P1.31 toggles every 500 ms

ARM: Counter = 0                                                                                                
RISCV: Counter = 0                                                                                             
ARM: Counter = 1                                                                                                
RISCV: Counter = 1                                                                                             
ARM: Counter = 2                                                                                               
RISCV: Counter = 2                                                                                             
ARM: Counter = 3                                                                                               
RISCV: Counter = 3
...
```
You will also observe LED0 and LED1 blinking at a rate of about 1Hz.