## Description

The bootloader is a special firmware that loads inside the main microcontroller, 
and generally the purpose of the bootloaders is to load the main application 
and provide firmware update functionality. 

For some microcontrollers, Maxim provides the bootloader firmware that gives 
target microcontroller firmware update functionality and security. 
For more detail about Maxim bootloader solution please contact with
Maxim representatives.

There are two types of Maxim bootloaders:
    - ROM based bootloaders
    - Flash based bootloaders (MAX32660, MAX32670, MAX32655, MAX78000...)

This example able to communicate with flash based bootloaders.

The purpose of this example is to provide a reference to how you communicate with 
flash based bootloader. It is designed to easily be ported on any micro.


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and MAX32660 EVKit board.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Connect I2C and SPI pin with target bootloader.
Then driver example over PC console.

## Expected Output

The Console UART of the device will output these messages:

```
***********Bootloader Host Example***********
The purpose of this example is:
   1- Demonstrate how bootloader device can be program
   2- Provide platform independent bootloader protocol (files under bootloader folder)

This example can be ported on any platform
If you would like to port it on other platform
you need to update terminal.c and platform_max32660.c files

    HW Pins
    I2C:    SCL(P0.2),   SDA(P0.3) (Note: I2C requires pullup resistor)
    SPI:    MISO(P0.4),  MOSI(P0.5),  SCK(P0.6),  SS(P0.7)
    Target: RESET(P0.8), MFIO(P0.9)
    
UART1 is used as terminal comport

Main Menu
-----------------------------------
1- Select Interface
2- Bootloader Test Menu
3- Load MSBL: Non-secure MAX32660 blinkled fast P0.13
4- Load MSBL: Non-secure MAX32660 blinkled slow P0.13
5- Load MSBL: MAX32660 blinkled fast P0.13 (development key)
6- Load MSBL: MAX32660 blinkled slow P0.13 (development key)


```

