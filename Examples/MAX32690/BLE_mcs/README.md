## Description

Maxim custom Bluetooth profile and service that advertises as "MCS" and accepts
connection requests.

# Usage

## LEDs

Each LED is controlled by a characteristic. Write the characteristic to 1 to turn on the LED,
0 to turn off the LED. Boards without 3 LEDs have the characteristics combined. 

Red LED Characteristic   : 0x85FC567F31D9418587C6339924D1C5BE  
Green LED Characteristic : 0x85FC568031D9418587C6339924D1C5BE  
Blue LED Characteristic  : 0x85FC568131D9418587C6339924D1C5BE  

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

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
__echo (on|off)__  Enables or disables the input echo. On by default.  
__btn (ID) (s|m|l|x)__  Simulates button presses. Example: "btn 1 s" for a short button press on button 1.  
__pin (ConnID) (Pin Code)__  Used to input the pairing pin code.  

## Push buttons
Push buttons can be used to set McsButton characteristic.
The value can be read via BLE. 

### When connected
1. Button 1 short press:      set McsButton = 0x02
2. Button 1 medium press:     set McsButton = 0x03
3. Button 1 long press:       set McsButton = 0x04
4. Button 1 extra long press: set McsButton = 0x05
5. Button 2 short press:      set McsButton = 0x07
6. Button 2 medium press:     set McsButton = 0x08
7. Button 2 long press:       set McsButton = 0x09
8. Button 2 extra long press: set McsButton = 0x0A
