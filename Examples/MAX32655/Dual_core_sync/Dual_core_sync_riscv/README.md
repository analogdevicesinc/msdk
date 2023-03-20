## Description

This version of Dual_core_sync_riscv prints an incrementing count to the console UART and toggles the LED0 every 500 ms.

This project demonstrates the synchronization between the ARM core (CM4) and the RISC-V core (RV32). Please refer to the App Note [The MAX32655: Why Two Cores Are Better Than One](https://www.maximintegrated.com/en/design/technical-documents/app-notes/7/7336.html) for more information.

README file of project MAX32655/Dual_core_sync_arm will present more details on the dual core synchronization.


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Setup

##### Board Selection

Before building firmware you must select the correct value for _BOARD_  in "Makefile", either "EvKit\_V1" or "FTHR\_Apps\_P1", depending on the EV kit you are using to run the example.

##### Required Connections
If using the Standard EV Kit (EvKit\_V1):
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect pins JP4(RX_SEL) and JP5(TX_SEL) to RX0 and TX0  header.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Close jumper JP2 (LED0 EN).
-   Close jumper JP3 (LED1 EN).

If using the Featherboard (FTHR\_Apps\_P1):
-   Connect a USB cable between the PC and the J4 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the board's console UART at 115200, 8-N-1.
