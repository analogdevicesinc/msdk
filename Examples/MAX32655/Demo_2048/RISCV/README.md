## Description

This example demonstrates running the 2048 game on both ARM and RISC-V cores on the MAX32655.

The RISC-V core, running at 60MHz (ISO), handles the controller (keyboard) user inputs and the main 2048 game logic.

The ARM core, running at 100MHz (IPO), keeps track of the timer (RTC) and handles the display graphics after the RISC-V core finishes handling the main game logic.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analogdevicesinc.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

This project only supports the MAX32655EVKIT as it has the 320x240 TFT Display.

## Required Connections

If using the MAX32655EVKIT (EvKit\_V1):
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect pins JP4(RX_SEL) and JP5(TX_SEL) to RX0 and TX0  header.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 2000000, 8-N-1.
-   Close jumper JP2 (LED0 EN).
-   Close jumper JP3 (LED1 EN).

## Expected Output

```
*******************************************************************************
ARM: Starting ARM Initialization.

ARM: Semaphore is not busy - with previous value: 1

ARM: Starting RISC-V core and handing off major UART0 Control to RISC-V.

RISC-V: Starting RISC-V Initialization.

RISC-V: Semaphore is not busy - with previous value: 1

RISC-V: Finished startup. Main UART0 control is handled by RISC-V now.

RISC-V: Starting Controller and Game










        |        |        |
        |        |        |
        |        |        |
-----------------------------------
        |        |        |
        |        |        |
        |        |        |
-----------------------------------
        |        |        |
        |        |  0002  |
        |        |        |
-----------------------------------
        |        |        |
        |        |        |
        |        |        |
```
