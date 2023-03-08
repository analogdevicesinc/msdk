# Description

Bluetooth fitness device. Showcases heart rate, battery level, running speed and cadence.

# Usage

## LEDs

The red LED will indicate that an error assertion has occurred.  

The green LED indicates CPU activity. When the LED is on, the CPU is active, when the LED
is off, the CPU is in sleep mode.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

If using the Standard EV Kit board (EvKit\_V1):
-   Connect a USB cable between the PC and the CN2 (USB/PWR - UART) connector.
-   Close jumpers JP7 (RX_EN) and JP8 (TX_EN).
-   Close jumpers JP5 (LED1 EN) and JP6 (LED2 EN).


### Serial Port
When TRACE is enabled in the project.mk, the on-board USB-to-UART adapter can be used to view
the trace messages as well as interact with the demo. Open a serial port terminal
with the following settings.

Baud:             115200
Char size:        8
Parity:           None
Stop bits:        1
HW Flow Control:  No

## Commands
Type the desired command and parameter (if applicable) and press enter to execute the command.  

__help__  Displays the available commands.  
__echo (on|off)__ Enables or disables the input echo. On by default.  
__btn (ID) (s|m|l|x)__ Simulates button presses. Example: "btn 1 s" for a short button press on button 1.  
__pin (ConnID) (Pin Code)__ Used to input the pairing pin code.  

## Push buttons
Push buttons can be used to interact with the application. 

### When disconnected
1. Button 1 short: Start/Stop scanning
2. Button 1 medium: Enter discoverable and bondable mode
3. Button 1 long: Clear all bonding info
4. Button 2 short: Toggle HRM flag for 8 and 16 bit values

### When connected
1. Button 1 short: Increment the heart rate
2. Button 1 long: Close the connection
3. Button 2 short: Decrement the heart rate
4. Button 2 medium: Toggle HRM DET flags
5. Button 2 long: Toggle HRM RR Interval feature flag


