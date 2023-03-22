## Description

Simple program that demonstrates how to link an application to static libraries.

The demo application reads the S2/SW2 push button input and toggles the red LED. The GPIO functions are built 
into a static library that can be found under Library_Generate project. Refer to project.mk to see how to 
link your application to a prebuilt static library.

Prior to building this project, you should first build the Library_Generate project to generate the necessary 
".a" static library file. Then, open the project.mk file and set the LIB_LOCATION variable to point to the 
location of the Library_Generate project. If you have changed the default build directory, you should also 
update LIB_BUILD_DIR so that the build system can find the ".a" file during linkage.

## Required Connections

### MAX32650 Evaluation Kit
-   Connect a USB cable between the PC and the CN2 (USB TO UART) connector.
-   Connect a debugger to J3 or J4 (JTAG/SWD).
-   Select RX SEL and TX SEL on headers JP12 and JP13.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

### MAX32650FTHR Evaluation Kit
-   Connect a USB cable between the PC and the J1 connector.
-   Connect a debugger to J5.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
********************** Static Library Example **********************

This example uses static library functions to read the state of S2/SW2 and
switch the red LED on as long as the push button is pressed.

...
```
