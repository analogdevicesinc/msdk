## Description

This version of Dual_core_sync_riscv prints an incrementing count to the console UART and toggles the LED0 every 500 ms.

This project demonstrates the synchronization between the ARM core (CM4) and the RISC-V core (RV32). Please refer to the App Note [The MAX32655: Why Two Cores Are Better Than One](https://www.maximintegrated.com/en/design/technical-documents/app-notes/7/7336.html) for more information. Note, that doc is for the MAX32655.

README file of project `MAX32690/Dual_Core_Sync/Dual_core_sync_arm` will present more details on the dual core synchronization.


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analogdevicesinc.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

* This project comes pre-configured for the MAX32690EVKIT.  See [Board Support Packages](https://analogdevicesinc.github.io/msdk/USERGUIDE/#board-support-packages) in the MSDK User Guide for instructions on changing the target board.

* **This project is not typically compiled on its own**.  Instead, the `Dual_core_sync_arm` project should be used to build and load it.

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
