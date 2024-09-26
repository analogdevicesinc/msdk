# Description

This is an example to show how to update the firmware through HCI



## Required Connections
-   Connect a USB cable between the PC and the (USB/PWR - UART) connector.
-   Use an external USB-to-UART adapter to access HCI UART. Connect a USB cable between the PC or BLE Tester
    and USB side of the adapter. Connect UART side of the adapter to board TX,RX and GND header pins.
-   Optionally you can reconfigure the UART definitions in board.h to use the on-board USB to UART 
    adapter for the HCI UART.


### Project-Specific Build Notes
The `project.mk` in `Bootloader` application in conjunction with `project.mk` in `BLT` and `Second_App` to determine
where the expected file is stored and read from.

The flag `USE_INTERNAL_FLASH ` is used to determine which memory to store the firmware (internal or external). This example only is implemented using internal flash memory. 
So, you should set the `USE_INTERNAL_FLASH ` to 1 to run the example. 


You should completely clean the bin folder inside the `Bootloader  ` by type `make distclean` to make sure that it will link the correct version of Bootloader. Then you compile your first program inside the `BLT` folder and flash it from `BLT` folder. 

### Result:
The original program is BLE_CTR5, if you look at the `main.c` in `BLT`, this is exactly the same as the `main.c` in `BLT_ctr5`. We run this program first so that it can communicate through HCI. 

After flashing the program, the result should look like this: 
```
    RAM: 1 x 752 bytes -- connection context
    RAM: 4 x 386 bytes -- Tx buffer descriptors
    RAM: 2 x 2296 bytes -- advertising set context
LlHandlerInit: LL initialization completed
    opModeFlags = 0x005F5C40
### LlApi ###  LlSetBdAddr
Static BDA[5:3]=00:18:80
       BDA[2:0]=03:AE:F6
### LlApi ###  LlSetAdvTxPower, advTxPwr=0

```

**Attention**: To update the firmware, you have to erase the second memory bank first! The flash memory won't allow you to write until you erase that memory region first. 

you can do it by using HCI through CLI by typing `erase 10:04:00:00`, the result should look like this:
```
Erase flash memory at address: 10040000
Done
```

**Attention**: `erase 10:04:00:00`: the parameter `10:04:00:00` is the starting address of second memory bank (0x10040000) which is used to store the updated firmware. This command will only erase **one page** of internal memory starting from 10:04:00:00. To upload the second application successfully, you may need to erase mutiple pages according to the size of the second application.

If you want to erase multipages at once, you can use python script to erase the memory bank. The code is shown in `MAX-BLE-HCI/examples/firmware_update/firmware_update.py`

Then you can update the program by typing `update 10:04:00:00 hello_world.bin`. `hello_world.bin` is file for new application. This file is in repository `MAX-BLE-HCI/examples/firmware_update`. You can also doing this by writing a script. 

Finally you can reset the program to enable the updated firmware. You can reset the program by pressing the reset button or type `sysreset` through HCI. Then you will get the following result:
```
Second Application: Hello World! 
```


You can also run this example through HCI script. The example is located in `MAX-BLE-HCI/examples/firmware_update/firmware_update.py`.

### Configuration:
You can change the starting address and size of the memory in linker script in both `BLT` folder and `Bootloader` folder.
The setting starting address and size in this example is for MAX32655.