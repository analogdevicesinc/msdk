# Description

Bluetooth fitness device. Showcases heart rate, battery level, running speed and cadence.

Please refer to the [Examples/MAX32655/BLE_fit/README.md](../BLE_fit/README.md) for details.

This demo project modifies the [Examples/MAX32655/BLE_fit/](../BLE_fit/) project and implements the application in FreeRTOS.

In the serial console, two new commands "cmd ps s" and "cmd ps l" are added to display the FreeRTOS task related information.
```
cmd ps s  
Task            Run time cnt    Run time percentage  
***************************************************  
CordioH        	73		<1  
IDLE           	13641		99  
Tmr Svc        	0		<1  
CordioM        	0		<1  
```

```
cmd ps l  
Task          State  Priority  Stack    #  
**************************************  
CordioH        	X	2	3654	2  
IDLE           	R	0	117	3  
Tmr Svc        	B	2	90	4  
CordioM        	S	3	3982	1  
```

# Notes:
* Before build the project, in the subfolder "...\msdk\Libraries\BlePhy\MAX32655":
* For MAX32655 chip revision A1 (The laser marking ends with "...A1", indicating an A1 chip revision). Please backup files for chip B1 revision: libphy.a, libphy_hard.a and libphy_riscv.a (Rename to "libphy_b1.a, libphy_b1_hard.a and libphy_b1_riscv.a"). Use files for A1 chip revision (Rename files "libphy_a1.a, libphy_a1_hard.a and libphy_a1_riscv.a" to "libphy.a, libphy_hard.a and libphy_riscv.a").
* For MAX32655 chip revision B1 (The laser marking ends with "...B1", indicating an B1 chip revision). Please backup files for chip A1 revision: libphy.a, libphy_hard.a and libphy_riscv.a (Rename to "libphy_a1.a, libphy_a1_hard.a and libphy_a1_riscv.a"). Use files for b1 chip revision (Rename files "libphy_b1.a, libphy_b1_hard.a and libphy_b1_riscv.a" to "libphy.a, libphy_hard.a and libphy_riscv.a").

# Use ARM core and RISC-V core for split HCI
In the project.mk, changing USE_DUAL_CORE to 1 will enable using both ARM core and RISC-V core for the split HCI.


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analogdevicesinc.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

* This project comes pre-configured for the MAX32655EVKIT.  See [Board Support Packages](https://analogdevicesinc.github.io/msdk/USERGUIDE/#board-support-packages) in the MSDK User Guide for instructions on changing the target board.

