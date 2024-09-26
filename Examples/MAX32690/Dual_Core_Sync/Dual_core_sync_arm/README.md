## Description

Projects Dual_core_sync_arm and Dual_core_sync_riscv demonstrate loading the RISC-V core program from the ARM core and synchronizing these two cores by hardware semaphores.

Dual_core_sync_arm runs on the ARM core (CM4) to load the RISC-V core (RV32) code space, setup the RISC-V debugger pins, and start the RISC-V core.

The Dual_core_sync_riscv example runs on the the RISC-V core.

Please refer to the App Note [The MAX32655: Why Two Cores Are Better Than One](https://www.maximintegrated.com/en/design/technical-documents/app-notes/7/7336.html) for more information.


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analogdevicesinc.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

* This project comes pre-configured for the MAX32655EVKIT. See [Board Support Packages](https://analogdevicesinc.github.io/msdk/USERGUIDE/#board-support-packages) in the MSDK User Guide for instructions on changing the target board.

* This project will build and load the `Dual_core_sync_riscv` example into the RISC-V core.  See the [project.mk](project.mk) file and [Build Variables for RISC-V Cores](https://analogdevicesinc.github.io/msdk//USERGUIDE/#build-variables-for-risc-v-cores) documentation.

## Required Connections

Connect a MAXPICO or other supported Debug adapter to the SWD Connector.
-   Note: Debug adapters other than the MAXPICO may not route the UART signals to the SWD connector. On MAX32690FTHR and AD-APARD32690-SL boards, this may limit your ability to access to serial port.

If using the MAX32690EVKIT:
-   Connect a USB cable between the PC and the CN2 (USB/PWR - UART) connector.
-   Install JP7(RX_EN) and JP8(TX_EN) headers.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Close jumper JP5 (LED1 EN).
-   Close jumper JP6 (LED2 EN).

If using the MAX32690FTHR:
-   Connect a USB cable between the PC and the J5 (USB/PWR) connector.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

If using the AD-APARD32690-SL:
-   Connect a USB cable between the PC and the P10 (USB-C) connector.
-   Open a terminal application on the PC and connect to the MAXPICO's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
-----------------------------------
ARM   : Start.
ARM   : After init, CheckSema(0) returned NOT BUSY.
ARM   : GetSema returned NOT BUSY with previous semaphore value 1.
ARM   : Wait 2 secs then start the RISC-V core.

RISC-V: Start.
RISC-V: After init, CheckSema(1) returned NOT BUSY.
RISC-V: GetSema returned NOT BUSY with previous semaphore value 1.
RISC-V: Do initialization works here.
RISC-V: Signal ARM.
ARM   : Do initialization works here.
ARM   : Signal RISC-V.
RISC-V: cnt=0
ARM   : cnt=1
RISC-V: cnt=2
ARM   : cnt=3
RISC-V: cnt=4
ARM   : cnt=5
```
