## Description

This project demonstrates image-based object detection (QR codes) using the MAX78000's CNN accelerator and micro-ROS integration.

RGB images are captured at 160x120 and fed to the CNN accelerator.  The CNN runs a pre-trained model trained on a synthetic dataset to detect QR codes.  The model draws a bounding box around the QR code, and can track rotated and skewed objects.  The QR codes are used as visual tags only - the QR codes themselves are not resolved or classified.  Any QR code will be detected.

The CNN results are published as ROS messages to the following topics:

- `/microROS/roi` ([sensor_msgs/RegionOfInterest](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/RegionOfInterest.msg))
  - A RegionOfInterest message describing a bounding box around the QR code.  This box is not rotated.
- `/microROS/polygon` ([geometry_msgs/PolygonStamped](https://github.com/ros2/common_interfaces/blob/humble/geometry_msgs/msg/PolygonStamped.msg))
  - A PolygonStamped message describing a bounding box around the QR code that is rotated and/or skewed to match the corners of the QR code as closely as possible.

Optionally, the raw input images to the CNN can also be published as a [`sensor_msgs/Image`](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/Image.msg) message to the `/microROS/image` topic.  A [`utils/image_viewer`](utils/image_viewer.py) script is included that can be used to view and save the published images and detected boxes.  Note that the raw image transmission can take a significant amount of time (~10s) for each.

## Robot Arm Demo

This project is used as a component in the Robotis OpenManipulator-X demo.  The demo setup and usage is documented in [`robot_arm.md`](./robot_arm.md)

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

* This project comes pre-configured for the MAX78000CAM02 board.  See [Board Support Packages](https://analog-devices-msdk.github.io/msdk/USERGUIDE/#board-support-packages) in the UG for instructions on changing the target board.

* To enable the publication of images, define `PUBLISH_IMAGE` in [app_config.h](app_config.h)

## Required Connections

If using the MAX78000EVKIT (EvKit_V1):
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect pins 1 and 2 (P0_1) of the JH1 (UART 0 EN) header.
-   Close jumper JP1 (LED1 EN).
-   Close jumper JP2 (LED2 EN).
-   Connect the micro-ROS Agent to the serial port presented by CN1 at a baud rate of 115200.
-   Reset the board

If using the MAX78000FTHR (FTHR_RevA)
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect the micro-ROS Agent to the serial port presented by CN1 at a baud rate of 115200.
-   Reset the board

If using the CAM02_RevA (CAM02_RevA)
-   Connect the QWIIC connector to a 3.3V supply to power the CAM02 board.
-   Connect the PICO debugger to the SWD debug port on the CAM02, and connect the PICO's micro-USB port to the host PC.
-   Connect the micro-ROS Agent to the serial port presented by the PICO debugger at a baud rate of 115200.
-   Reset the board

For more details on micro-ROS and connecting the agent, see the [MSDK's micro-ROS documentation](https://github.com/Jake-Carter/msdk/blob/dev/micro-ros/Libraries/microROS/MICROROS.md) 

## Expected Output

micro-ROS Agent output:

```shell
[1720834520.368300] info     | TermiosAgentLinux.cpp | init                     | running...             | fd: 3
[1720834520.368464] info     | Root.cpp           | set_verbose_level        | logger setup           | verbose_level: 4
[1720834559.849440] info     | Root.cpp           | create_client            | create                 | client_key: 0x4AC6DCFD, session_id: 0x81
[1720834559.849494] info     | SessionManager.hpp | establish_session        | session established    | client_key: 0x4AC6DCFD, address: 0
[1720834559.889856] info     | ProxyClient.cpp    | create_participant       | participant created    | client_key: 0x4AC6DCFD, participant_id: 0x000(1)
[1720834559.924232] info     | ProxyClient.cpp    | create_topic             | topic created          | client_key: 0x4AC6DCFD, topic_id: 0x000(2), participant_id: 0x000(1)
[1720834559.975378] info     | ProxyClient.cpp    | create_publisher         | publisher created      | client_key: 0x4AC6DCFD, publisher_id: 0x000(3), participant_id: 0x000(1)
[1720834560.028476] info     | ProxyClient.cpp    | create_datawriter        | datawriter created     | client_key: 0x4AC6DCFD, datawriter_id: 0x000(5), publisher_id: 0x000(3)
[1720834560.085345] info     | ProxyClient.cpp    | create_topic             | topic created          | client_key: 0x4AC6DCFD, topic_id: 0x001(2), participant_id: 0x000(1)
[1720834560.137301] info     | ProxyClient.cpp    | create_publisher         | publisher created      | client_key: 0x4AC6DCFD, publisher_id: 0x001(3), participant_id: 0x000(1)
[1720834560.189532] info     | ProxyClient.cpp    | create_datawriter        | datawriter created     | client_key: 0x4AC6DCFD, datawriter_id: 0x001(5), publisher_id: 0x001(3)
[1720834560.245412] info     | ProxyClient.cpp    | create_topic             | topic created          | client_key: 0x4AC6DCFD, topic_id: 0x002(2), participant_id: 0x000(1)
[1720834560.320391] info     | ProxyClient.cpp    | create_publisher         | publisher created      | client_key: 0x4AC6DCFD, publisher_id: 0x002(3), participant_id: 0x000(1)
[1720834560.373549] info     | ProxyClient.cpp    | create_datawriter        | datawriter created     | client_key: 0x4AC6DCFD, datawriter_id: 0x002(5), publisher_id: 0x002(3)
```