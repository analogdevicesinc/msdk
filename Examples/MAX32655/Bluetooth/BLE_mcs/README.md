# BLE_mcs

Maxim custom Bluetooth profile and service that advertises as "MCS" and accepts
connection requests.
Refer to the [BLE_mcs](../../../../Libraries/Cordio/docs/Applications/BLE_mcs.md) documentation in the Cordio Library.

# Notes:
* Before build the project, in the subfolder "...\msdk\Libraries\BlePhy\MAX32655":
* For MAX32655 chip revision A1 (The laser marking ends with "...A1", indicating an A1 chip revision). Please backup files for chip B1 revision: libphy.a, libphy_hard.a and libphy_riscv.a (Rename to "libphy_b1.a, libphy_b1_hard.a and libphy_b1_riscv.a"). Use files for A1 chip revision (Rename files "libphy_a1.a, libphy_a1_hard.a and libphy_a1_riscv.a" to "libphy.a, libphy_hard.a and libphy_riscv.a").
* For MAX32655 chip revision B1 (The laser marking ends with "...B1", indicating an B1 chip revision). Please backup files for chip A1 revision: libphy.a, libphy_hard.a and libphy_riscv.a (Rename to "libphy_a1.a, libphy_a1_hard.a and libphy_a1_riscv.a"). Use files for b1 chip revision (Rename files "libphy_b1.a, libphy_b1_hard.a and libphy_b1_riscv.a" to "libphy.a, libphy_hard.a and libphy_riscv.a").

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analogdevicesinc.github.io/msdk/USERGUIDE/)**.

### Required Connections
* Connect a USB cable between the PC and the (USB/PWR - UART) connector.


### Project-Specific Build Notes
* Setting `TRACE=1` in [**project.mk**](project.mk) initializes the on-board USB-to-UART adapter for
viewing the trace messages and interacting with the application. Port uses settings:
    - Baud            : 115200  
    - Char size       : 8  
    - Parity          : None  
    - Stop bits       : 1  
    - HW Flow Control : No  
    - SW Flow Control : No  
