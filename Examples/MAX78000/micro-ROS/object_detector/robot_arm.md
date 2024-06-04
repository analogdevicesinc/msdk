# Visual Servoing Robot Arm Demo Guide

## Overview

This demo show real-time visual servoing enabled by the MAX78000CAM02 custom camera module.  A custom single-shot neural network has been trained to detect QR codes using a synthetic dataset, and has been deployed to the MAX78000 AI microcontroller.  "micro-ROS" has also been ported to the MAX78000 to allow it to integrate directly with ROS (Robot Operating System) networks.  The MAX78000CAM02 captures a 160x120 RGB image and sends it to the CNN accelerator as input.  The inference results are a bounding box defining the 4 corners of the QR code.  These results are then packaged as a standard ROS message and sent to a Raspberry Pi over a UART serial connection.  The Raspberry Pi computes a position-based feedback loop based on the relative size and position of the bounding box in the image frame, and sends motor control commands to a Robotis OpenManipulator-X robotic arm.

## Demo Components

There are 3 primary components to the demo:

1. MAX78000CAM02 module
2. Raspberry Pi 4B
3. Robotis OpenManipulator-X robot arm

Each component requires some assembly and associated hardware connections.  Additionally, each component requires software and firmware to link them together into the complete demo.  The components are discussed briefly in more detail here to provide a high-level overview, followed by a detailed setup guide for the entire demo.

![Block Diagram](res/Asimov-Block-Diagram.jpg)

## Hardware Setup

**(!) Begin all assembly with components powered off and disconnected from power supplies until otherwise noted.**

1. Unpack all components from the Pelican case.  It should be noted that the case has two "levels".

2. Open the Robotis OpenManipulator-X box and locate the primary tools.  These consist of:

    - 3 silver allen (hex) keys
    - A small philips head screwdriver

3. Take the screen out of the ESD bag and turn it over.  The Raspberry Pi is attached to the back of it.  Place a pink thermal pad on top of the CPU.

![Img 1](res/1.jpg)

4. Using the silver allen key, install the cooling fan for the Raspberry Pi.  You will need to place 4 white spacers in each corner, and use 4 silver M3 screws.  These can be found together in a bag inside the OpenManipulator-X box.  The silver fan bracket will only fit in one orientation as follows: 

![Img 2](res/2.jpg)

5. Connect the fan supply pins to the GPIO header on the Raspberry Pi.

![Img 3](res/3.jpg)

6. Using the small Philips head screwdriver, install the feet for the TFT screen.

![Img 4](res/4.jpg)

![Img 5](res/5.jpg)

7. Now, locate the bottom motor on the robot arm.  It's labelled "11".  Install 4 black spacers into the screw holes on the bottom of the motor.

![Img 6](res/6.jpg)

8. Prepare the robot arm to be attached to the silver aluminum plate.  THe easiest way to do this is to fold it onto itself like below:

![Img 7](res/7.jpg)

9. Using the long silver screws, attach the robot arm to the aluminum plate.

![Img 8](res/8.jpg)

![Img 9](res/9.jpg)

10. Now, locate the 3D printed attachment for the MAX78000CAM02 board, and insert the board as follows.  Note that the camera module should be removed before inserting into the attachment.

![Img 10](res/10.jpg)

![Img 11](res/11.jpg)

11. Now, insert the camera module into the PCIF slot.  The orientation should be as follows.  (Note the small white text on the "top" edge).

![Img 12](res/12.jpg)

![Img 13](res/13.jpg)

12. Attach the PICO debugger board to the MAX78000CAM02 module using the SWD cable.  Note the orientation of the cable (defined by the red line) must match pin 1 of the SWD header.  This connection "locks" the MAX78000CAM02 board in place and prevents it from slipping as well.

![Img 14](res/14.jpg)

13. Now, install the MAX78000CAM02 attachment to the underside of the robot arm gripper using 2 silver screws.  Take care not to overtighten - snug but not loose is sufficient.

- **Red** : Keyboard/mouse connection
- **Yellow**: OpenCR1.0 connection
- **Blue**: MAX78000CAM02 PICO debugger connection

![Img 15](res/15.jpg)

14. Connect the small power cable that is threaded through the robot arm to to the QWIIC connector.

![Img 16](res/16.jpg)

![Img 27](res/27.jpg)

15. Now, locate the OpenCR1.0 board.  Ensure the main power switch is set to the OFF position, then connect its power supply to the barrel jack.

![Img 17](res/17.jpg)

16. Connect the TTL cable from the bottom motor of the robot arm to the OpenCR1.0 controller.  Ensure that the connection matches the image below.

![Img 18](res/18.jpg)

17. Attach the female end of the MAX78000CAM02 supply cable to the 3.3V supply pins of the Raspberry Pi's GPIO header.

![Img 29](res/29.jpg)

![Img 28](res/28.jpg)

18. Check the supply pins again from step 17.  Getting the orientation backwards will destroy the MAX78000CAM02.

19. Ensure the USB-C power supply switch is set to OFF, then connect it to the Raspberry Pi.

![Img 20](res/20.jpg)

20. Pop open the back of the keyboard and find the wireless USB dongle.

![Img 21](res/21.jpg)

21. Make the USB connections to the Raspberry Pi.  **Ensure that all connections match the exact ports specified**.  Otherwise, power over-draw issues/brown-outs may occur.  The USB ports can only supply 1.2A each, and the screen has proved to draw close to this limit.  The keyboard should be installed onto the same USB pair as the screen.

![Img 23](res/23.jpg)

![Img 22](res/22.jpg)

![Img 24](res/24.jpg)

22. Hardware setup is now complete.  Proceed to the software setup.

## Software Setup

After completing hardware setup, follow these instructions to setup the software and launch the demo.

**(!) Begin all assembly with components powered off and disconnected from power supplies until otherwise noted.**

1. Power on the OpenCR1.0 board.

![Img 26](res/26.jpg)

2. Power on the Raspberry Pi.

3. Log in to the Pi as the "adi" user.  The credentials are:

    - Username: `adi-user`
    - Password: `Adi9442!`

    (Credentials are for an RPi user account completely local to the board and not used for any other ADI logins or systems)

4. Open a terminal.

5. Navigate the MSDK repository.  It's located in the `adi-user` home directory at `/home/adi-user/repos/msdk`.

    Note the `ranger` command can be used to easily navigate around.  

    - `SHIFT + S` to open the selected directory
    - `:q` to quit ranger

6. Navigate to the demo project folder.  Inside the MSDK, the demo project folder is located at `Examples/MAX78000/micro-ROS/object_detector`

7. Now that your terminal is opened in the project folder, the demo is ready for operation.

## Demo Operation

### Launching

To launch the demo, run the following command from inside the project folder (`Examples/MAX78000/micro-ROS/object_detector`):

```shell
python3 arm_feedback.py
```

A terminal UI will launch showing statuses of various demo components, as well as a console that logs debug messages.

After successfully connecting to the OpenCR1.0 and MAX78000CAM02 boards, the arm will slowly move to its home position after a 3-second countdown.

### Running

After moving to its home position, the demo is in full operation.  The robot arm will wait for a cube marked with a QR code to enter its field of view.  Consider this the "home state".  

When the arm sees a cube marked with a QR code, it will move to pick up the cube.  Consider this "grab mode".

While in "grab mode", the cube can be moved around and the robot arm will readjust is pathing on the fly to correct for any changes.  If the cube is taken out of the field of view of the camera, the arm will enter a simple search mode.  The search mode will move back and forth in the last known direction, while slowly drifting back towards the home position.  The arm will only stop moving after it has successfully picked up and dropped off a cube - at which point it enters its initial "home" state again, waiting for another cube to enter its initial field of view.

Note that while the operation and real-time correction is relatively robust, it's still possible to "over-extend" the arm and put it into impossible scenarios or broken states.  

An impossible pick-up scenario usually occurs when the arm is overextended to the extreme edge of the plate and simultaneously gets too low to the ground plane.  This can usually be fixed by letting the arm iterate back towards home through its search mode.

Additionally, note that the arm only has 5 degrees-of-freedom (DoF).  As a result, the gripper cannot rotate.  The demo runner should ensure that the arm does not attempt to pick up an overly rotated cube.

More rarely, a completely broken state can occur when joint limits have been exceeded.  This will cause the arm to freeze entirely (for its own protection) and can only be fixed by shutting down and restarting the demo.  

### Shutting Down

To shut down the demo, press `CTRL + C` on the keyboard.  **(!!!) Note the that this will shut down the motor controller and cause the arm to drop into freefall from wherever it is!**  Place your hand underneath the gripper and hold the robot arm upright before shutting it down to avoid any damage to the demo.

## Detailed Demo Components

### MAX78000CAM02

The MAX78000CAM02 module is a tiny custom camera board built around the MAX78000 and a GalaxyCore GC0308 low-power CMOS camera module.  It can be powered from a 3.3V input and offers an I2C or UART communication interface exposed from a QWIIC connector.

The firmware for the MAX78000CAM02 is a hybrid application combining the pre-trained QR code detection CNN model with camera drivers and acquisition code.  Additionally, the firmware leverages micro-ROS, which is in turn built on top of a FreeRTOS layer.  Overall, the firmware initializes the board-level hardware (including the camera), then initializes FreeRTOS and micro-ROS.  The main application loop is implemented inside a single FreeRTOS task.

Status LEDs on the MAX78000CAM02 indicate the overall application state, which falls into two categories:  initialization and operation.

#### MAX78000CAM02 Initialization

After receiving power, the MAX78000CAM02 will immediately boot up.  After a 2-second delay, the red LED on the board will flash once to indicate that the board is powered on and the firmware is launching.  Next, FreeRTOS will be initialized and the main application task will be launched.  If any errors occur during this initial boot-up phase (FreeRTOS doesn't have enough memory, fails to launch the main application task, etc.) the red LED on the board will slowly blink once every 2 seconds.  These errors can be considered critical and usually require re-development of the core drivers and port layer for micro-ROS.  They should not be seen in a validated demo setup.

After booting up, the micro-ROS library will be initialized from inside the main application task.  If any initialization errors occur during this phase, the red status LED on the MAX78000CAM02 will blink once every second.

After micro-ROS is initialized, the firmware will attempt to establish a connection with a special "bridge" program running on the Raspberry Pi called the "micro-ROS Agent".  The firmware will attempt to establish the connection for up to 10 seconds.  If the connection is not established, the red status LED will blink once every second.  The setup of the micro-ROS Agent itself is discussed in more detail below, but it's important to note that the firmware will attempt to establish this connection on startup.

Once the connection to the micro-ROS Agent is established, the firmware will enter its main loop.  The red status LED will blink 4 times every second, and when a QR is detected in the frame the green status LED will turn on.  It should be noted that sometimes initialization errors occur between the establishment of the micro-ROS agent and the main loop that cause the module to "freeze".  If the micro-ROS Agent connection is established but the red status LED is not blinking rapidly, power cycle the MAX78000CAM02 module until the firmware enters the main loop successfully.  

### Raspberry Pi

