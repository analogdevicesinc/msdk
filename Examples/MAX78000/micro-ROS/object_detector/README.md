## Description

This project demonstrates image-based object detection using the MAX78000's CNN accelerator.  Additionally, it publishes the bounding box of the detected object over micro-ROS to a [`sensor_msgs/RegionOfInterest`](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/RegionOfInterest.msg) message to the `/microROS/roi` topic.  The project comes with a pre-trained model to detect QR codes, and takes a 160x120 RGB image as input.

Optionally, the input images to the CNN can also be published as a [`sensor_msgs/Image`](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/Image.msg) message to the `/microROS/image` topic.  A [`utils/image_viewer`](utils/image_viewer.py) script is included that can be used to view and save the published images and detected boxes.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

* This project comes pre-configured for the MAX78000EVKIT.  See [Board Support Packages](https://analog-devices-msdk.github.io/msdk/USERGUIDE/#board-support-packages) in the UG for instructions on changing the target board.

* To enable the publication of images, define `PUBLISH_IMAGE` in [app_config.h](app_config.h)

## Required Connections

If using the MAX78000EVKIT (EvKit_V1):
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect pins 1 and 2 (P0_1) of the JH1 (UART 0 EN) header.
-   Close jumper JP1 (LED1 EN).
-   Close jumper JP2 (LED2 EN).
-   Connect the micro-ROS Agent to the serial port presented by CN1 at a baud rate of 115200.

If using the MAX78000FTHR (FTHR_RevA)
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect the micro-ROS Agent to the serial port presented by CN1 at a baud rate of 115200.

If using the CAM02_RevA (CAM02_RevA)
-   Connect the QWIIC connector to a 3.3V supply to power the CAM02 board.
-   Connect the PICO debugger to the SWD debug port on the CAM02, and connect the PICO's micro-USB port to the host PC.
-   Connect the micro-ROS Agent to the serial port presented by the PICO debugger at a baud rate of 115200.

## Expected Output

micro-ROS Agent output:

```shell
[1705623743.199386] info     | TermiosAgentLinux.cpp | init                     | running...             | fd: 3
[1705623743.199555] info     | Root.cpp           | set_verbose_level        | logger setup           | verbose_level: 4
[1705623758.944199] info     | Root.cpp           | create_client            | create                 | client_key: 0x74FC2C2D, session_id: 0x81
[1705623758.944252] info     | SessionManager.hpp | establish_session        | session established    | client_key: 0x74FC2C2D, address: 0
[1705623758.959727] info     | ProxyClient.cpp    | create_participant       | participant created    | client_key: 0x74FC2C2D, participant_id: 0x000(1)
[1705623758.974120] info     | ProxyClient.cpp    | create_topic             | topic created          | client_key: 0x74FC2C2D, topic_id: 0x000(2), participant_id: 0x000(1)
[1705623758.984117] info     | ProxyClient.cpp    | create_publisher         | publisher created      | client_key: 0x74FC2C2D, publisher_id: 0x000(3), participant_id: 0x000(1)
[1705623758.995288] info     | ProxyClient.cpp    | create_datawriter        | datawriter created     | client_key: 0x74FC2C2D, datawriter_id: 0x000(5), publisher_id: 0x000(3)
```

Bounding boxes can be subscribed to on the `/microROS/roi` topic:

```shell
--- # No box in view
x_offset: 0
y_offset: 0
height: 0
width: 0
do_rectify: false
--- # Box detected
x_offset: 71
y_offset: 57
height: 38
width: 30
do_rectify: false
---
x_offset: 65
y_offset: 58
height: 39
width: 30
do_rectify: false
---
x_offset: 68
y_offset: 56
height: 42
width: 30
do_rectify: false
--- # ...
```

[`utils/image_viewer.py`](utils/image_viewer.py) output without image publication

```shell
/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/qos.py:307: UserWarning: HistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST is deprecated. Use HistoryPolicy.KEEP_LAST instead.
  warnings.warn(
[INFO] [1705625065.372411354] [image_subscriber]: Initialized image subscriber.
[INFO] [1705625065.374541960] [roi_subscriber]: Initialized box subscriber,
[INFO] [1705625065.437383682] [roi_subscriber]: Received bounding box:
[INFO] [1705625065.437758749] [roi_subscriber]:         x_offset: 0
[INFO] [1705625065.438069814] [roi_subscriber]:         y offset: 0
[INFO] [1705625065.438316915] [roi_subscriber]:         width: 0
[INFO] [1705625065.438561216] [roi_subscriber]:         height: 0
[INFO] [1705625065.438812137] [roi_subscriber]:         middle: 0.0,0.0
[INFO] [1705625068.004519162] [roi_subscriber]: Received bounding box:
[INFO] [1705625068.004924880] [roi_subscriber]:         x_offset: 52
[INFO] [1705625068.005262556] [roi_subscriber]:         y offset: 73
[INFO] [1705625068.005512517] [roi_subscriber]:         width: 37
[INFO] [1705625068.005758648] [roi_subscriber]:         height: 43
[INFO] [1705625068.006012459] [roi_subscriber]:         middle: 70.5,94.5
[INFO] [1705625068.516594212] [roi_subscriber]: Received bounding box:
[INFO] [1705625068.517004511] [roi_subscriber]:         x_offset: 65
[INFO] [1705625068.517316045] [roi_subscriber]:         y offset: 80
[INFO] [1705625068.517561076] [roi_subscriber]:         width: 37
[INFO] [1705625068.517803867] [roi_subscriber]:         height: 41
[INFO] [1705625068.518053819] [roi_subscriber]:         middle: 83.5,100.5
```

[`utils/image_viewer.py`](utils/image_viewer.py) output with image publication
```shell
/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/qos.py:307: UserWarning: HistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST is deprecated. Use HistoryPolicy.KEEP_LAST instead.
  warnings.warn(
[INFO] [1705625650.163090925] [image_subscriber]: Initialized image subscriber.
[INFO] [1705625650.486086260] [roi_subscriber]: Initialized box subscriber,
[INFO] [1705625654.042360467] [image_subscriber]: Received image
[INFO] [1705625666.985965346] [roi_subscriber]: Received bounding box: # No box detected
[INFO] [1705625666.986351433] [roi_subscriber]:         x_offset: 0
[INFO] [1705625666.986606595] [roi_subscriber]:         y offset: 0
[INFO] [1705625666.986852406] [roi_subscriber]:         width: 0
[INFO] [1705625666.987098187] [roi_subscriber]:         height: 0
[INFO] [1705625666.987346918] [roi_subscriber]:         middle: 0.0,0.0
[INFO] [1705625672.822121449] [image_subscriber]: Received image
[INFO] [1705625674.059276902] [roi_subscriber]: Received bounding box: # Box detected
[INFO] [1705625674.059579156] [roi_subscriber]:         x_offset: 64
[INFO] [1705625674.059828327] [roi_subscriber]:         y offset: 53
[INFO] [1705625674.060073439] [roi_subscriber]:         width: 19
[INFO] [1705625674.060317650] [roi_subscriber]:         height: 24
[INFO] [1705625674.060570051] [roi_subscriber]:         middle: 73.5,65.0
[INFO] [1705625674.069790178] [roi_subscriber]: Saved image to test.png # Save image
[INFO] [1705625681.834765515] [roi_subscriber]: Received bounding box:
[INFO] [1705625681.835081999] [roi_subscriber]:         x_offset: 75
[INFO] [1705625681.835348741] [roi_subscriber]:         y offset: 58
[INFO] [1705625681.835614654] [roi_subscriber]:         width: 15
[INFO] [1705625681.835874615] [roi_subscriber]:         height: 30
[INFO] [1705625681.836142058] [roi_subscriber]:         middle: 82.5,73.0
```