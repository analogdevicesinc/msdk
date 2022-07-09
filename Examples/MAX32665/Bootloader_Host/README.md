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

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Connect I2C and SPI pin with target bootloader.
For more information please check: "Bootloader Host Example" section on the below page
https://pdfserv.maximintegrated.com/en/an/ug7510-maxim-bootloader-tools.pdf 

## Expected Output

The Console UART of the device will output these messages:

```
***********Bootloader Host Example***********
The purpose of this example is to demonstrate how bootloader device can be program
Depend on your target device you should apply I2C/SPI connection
Pin mapping of this example is listed below

    HW Pins
    I2C:    SCL(P0.6),    SDA(P0.7)
    SPI:    MOSI(P0.17),  MISO(P0.18),  SCK(P0.19),  SS(P0.16)
    Target: RESET(P0.14), MFIO(P0.15)

UART1 is used as terminal comport

Main Menu
-----------------------------------
1- Select Interface
2- Bootloader Test Menu
3- Load MSBL: Non-secure MAX32660 blinkled fast P0.13
4- Load MSBL: Non-secure MAX32660 blinkled slow P0.13
5- Load MSBL: MAX32660 blinkled fast P0.13 (development key)
6- Load MSBL: MAX32660 blinkled slow P0.13 (development key)
7- Load MSBL: MAX32670 blinkled P0.22 (development key)
8- Load MSBL: MAX32670 blinkled P0.23 (development key)


```

