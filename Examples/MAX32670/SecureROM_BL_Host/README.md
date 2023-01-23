## Description

The Analog Devices Inc. (ADI) secure microcontroller has special bootloader, it is called as "Secure ROM Bootloader".
This bootloaders provide firmware update function by secure way. The security related staff is handle by SDK and SDK tools.
This example is a reference that demonstrate communication with ADI Secure ROM bootloaders.

In this example host will be MAX32670 and target will be MAX32520. 
There are two blinkled examples that converted to C array by using ./scripts/scp_to_c_array.py script.

After apply related connection between MAX32670-EvKit and MAX32520-EvKit the blinkled example can be programmed by MAX32670.

This example can be port on any platform, in case of need update terminal.c and platform_MAX32670.c file
depend on the target platform.

Please check:
- Secure ROM section in [MAX32520 UG](https://www.analog.com/media/en/technical-documentation/user-guides/max32520-users-guide.pdf)
- [MAX32670-EVKIT board schematic](https://www.analog.com/media/en/technical-documentation/data-sheets/MAX32670EVKIT.pdf)
- [MAX32520-KIT schematic](https://www.analog.com/media/en/technical-documentation/data-sheets/MAX32520-KIT.pdf)

## Required Connections

- Connect a USB cable between the PC and MAX32670 EVKit board.
- MAX32670 UART1 will be used as debug port, connect it to the PC
     - On PC side open a serial port monitor application, 115200 8-N-1
- MAX32670 UART0 will be used to communicate with MAX32520/MAX32651, apply below connection
     - Connect MAX32670 P0.28 (RX) to the target UART TX
     - Connect MAX32670 P0.29 (TX) to the target UART RX
     - Connect MAX32670 P0.30  to the target RSTN
     - (If stimulus pin requires) Connect MAX32670 P0.27  to the target Stimulus PIN
	 set PIN active state by changing STIMULUS_PIN_ACTIVE_STATE in bootloader/bootloader.h file
	 - Connect GND line between MAX32670 - target boards
	
 
## Expected Output

MAX32670 output message will be (Debug port is UART1):

```
***********Secure ROM Bootloader Host Example***********
This example demonstrate how image can be load on secure microcontroller
by using second microcontroller as host
In this example host micro will be MAX32670 and target will be MAX32520/MAX32651
Blinkled images will be load to the target device by MAX32670 host

HW Connection:
MAX32670 UART1 will be used as debug port, connect it to the PC
    On PC side open a serial port monitor application, 115200 8-N-1
MAX32670 UART0 will be used to communicate with MAX32520/MAX32651, apply below connection
    Connect MAX32670 P0.28 (RX) to the target UART TX
    Connect MAX32670 P0.29 (TX) to the target UART RX
    Connect MAX32670 P0.30  to the target RSTN
    (If stimulus pin requires) Connect MAX32670 P0.27  to the target Stimulus PIN
    Connect GND line between MAX32670 - target boards

Note:
1-This example can be ported on any platform
    If you would like to port it on other platform
    you need to update terminal.c and platform_MAX32670.c files
2- To convert SCP images to C array, check ./scripts/scp_to_c_array.py script

Main Menu
---------------------------------------------------------
1  - MAX32520-EvKit load blinkled-1 (P1.6)
2  - MAX32520-EvKit load blinkled-2 (P1.7)
3  - MAX32651-EvKit load blinkled-1 (P2.25)

Please select: 1
Trying to connect: 0
...
...
Trying to connect: 4
Trying to connect: 5

---------- Connected  ----------
Please wait loading 03/33...
--------------------------------
ROM Version: 0x01010002
Phase: 0x04
Rework: Allowed
JTAG: Enabled
CRK: Exist
USN: A26C891108091B30030918EE17
--------------------------------
Please wait loading 06/33...
Please wait loading 07/33...
Please wait loading 10/33...
Please wait loading 11/33...
Please wait loading 14/33...
Please wait loading 15/33...
Please wait loading 18/33...
Please wait loading 19/33...
Please wait loading 22/33...
Please wait loading 23/33...
Please wait loading 26/33...
Please wait loading 27/33...
Please wait loading 30/33...
Please wait loading 31/33...
Loading done.

MAX32520-EvKit load blinkled-1 (P1.6): SUCCESS



Main Menu
---------------------------------------------------------
1  - MAX32520-EvKit load blinkled-1 (P1.6)
2  - MAX32520-EvKit load blinkled-2 (P1.7)
3  - MAX32651-EvKit load blinkled-1 (P2.25)

Please select:

```

