# Description

Bluetooth data client that scans for and connects to advertisers with the name of "OTAS".

The Wireless Data Exchange profile is used to transfer files from the client to the server. 
A CRC32 value is used to check the integrity of the transferred file. 

# Usage

## LEDs

The red LED will indicate that an error assertion has occurred.  

The green LED indicates CPU activity. When the LED is on, the CPU is active, when the LED
is off, the CPU is in sleep mode.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

* The application to use for the firmware update can be selected using the `FW_UPDATE_DIR` option in [project.mk](project.mk).  Whichever application is selected by this option must be configured to run from the appropriate memory section, as defined by the Bootloader (see the `Bootloader` example for more details).

## Required Connections

-   Connect a USB cable between the PC and the (USB/PWR - UART) connector.

## Trace Serial Port
When TRACE is enabled in the project.mk, the on-board USB-to-UART adapter can
be used to view the trace messages and interact with the application. Open a serial port terminal with
the following settings.

Baud            : 115200  
Char size       : 8  
Parity          : None  
Stop bits       : 1  
HW Flow Control : No  
SW Flow Control : No  

## Commands 
Type the desired command and parameter (if applicable) and press enter to execute the command.  

__help__  Displays the available commands.  
__echo__ (on|off) Enables or disables the input echo. On by default.  
__btn__ (ID) (s|m|l|x) Simulates button presses. Example: "btn 1 s" for a short button press on button 1.  
__pin__ (ConnID) (Pin Code) Used to input the pairing pin code.  

## Push buttons
Push buttons can be used to interact with the application. 

### When connected
1. Button 1 short: On/Off scanning  
2. Button 1 medium: Cycle through the connection index  
3. Button 1 long: Drop selected connection  
4. Button 1 extra long: Toggle PHY 
5. Button 2 short: Discover file space on the peer device.
6. Button 2 medium: Start the update transfer.
7. Button 2 long: Verify the transfer.
8. Button 2 extra long: Command the peer to disconnect and reset.

### When disconnected
1. Button 1 short press: On/Off scanning
2. Button 1 medium press: Cycle through the connection index
3. Button 1 long press: Clear all bonding info
4. Button 1 extra long press: Add RPAO characteristic to GAP service -- needed only when DM Privacy enabled
5. Button 2 extra long press: Enable device privacy -- start generating local RPAs every 15 minutes
