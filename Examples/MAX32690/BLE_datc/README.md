# BLE_datc

Refer to the [BLE_datc_dats](../../../Libraries/Cordio/docs/Applications/BLE_datc_dats.md) documentation in the Cordio Library.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

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

## Passkey input
Upon server dsicovery the user will be prompted to enter a passkey.
An arbitrary pin can be entered in the following format 
``` 
pin (connId) passkey
Eg: 
pin 1 123456

```
Next the server is expected to enter the same connId and passkey
to establish a secure connection and share bonding information
which for demonstration purposes,is echoed via the trace mechanism

Note that either the client or server can enter the passkey first.
The peer device must then match.

### Commands
Type the desired command and parameter (if applicable) and press enter to execute the command.  

__help__  Displays the available commands.  
__echo (on|off)__ Enables or disables the input echo. On by default.  
__btn (ID) (s|m|l|x)__ Simulates button presses. Example: "btn 1 s" for a short button press on button 1.  
__pin (ConnID) (Pin Code)__ Used to input the pairing pin code.  

## Push buttons
Push buttons can be used to interact with the application.

### When disconnected
1. Button 1 short: Start/Stop scanning
2. Button 1 medium: Cycle through the connection index (select connection)
3. Button 1 long: Clear all bonding info
4. Button 1 extra long: Add RPAO characteristic to GAP service -- needed only when DM Privacy enabled
5. Button 2 extra long: Enable device privacy -- start generating local RPAs every 15 minutes

### When connected
1. Button 1 short: Start/Stop scanning
2. Button 1 medium: Cycle through connection index (select connection)
3. Button 1 long: Close selected connection  
4. Button 2 short: Request PHY change (1M-2M-S2-S8) Only for BLE5 version.
5. Button 2 medium : Send secure message to peer
6. Button 2 long: Send short message to peer  
7. Button 2 extra long: Start data transfer speed test
