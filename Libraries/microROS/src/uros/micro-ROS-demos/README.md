# micro-ROS Demos

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

## Overview

The primary purpose for this repository is to organise all packages for the [Micro-ROS project](https://microros.github.io/micro-ROS/) functionalities demonstrations.
All packages contained in this repository are a part of the Micro-ROS project stack.

## Previous step

To run all the demonstrations, you need to set up the ROS2 environment and build all the required packages.
Click [here](https://github.com/microROS/micro-ROS-doc) to read further about how to do this previous step.

## Package clusters

The repository contains the below packages clusters:

### Simple message demonstration

#### Packages

##### Int32_publisher

The purpose of the package is to publish one of the most basic ROS2 messages and demonstrate how Micro-ROS layers (rcl, typesupport and rmw) handle it.
For each publication, the message value increases in one unit order to see in the subscriber side the message variations.

##### Int32_subscriber

The purpose of the package is to subscribe to one of the most basic ROS2 messages and demonstrate how Micro-ROS layers (rcl, typesupport and rmw) handle it.

#### Run demonstration (Linux)

Run the micro-ROS Agent.
For the micro-ROS Agent to find the XML reference file, the execution must be done from the executable folder.

```bash
cd ~/agent_ws/install/uros_agent/lib/uros_agent/
./uros_agent udp 8888
```

You may prefer to run the Agent in the background and discard all outputs to keep using the same terminal for the next step.

```bash
cd ~/agent_ws/install/uros_agent/lib/uros_agent/
./uros_agent udp 8888 > /dev/null &
```

Run the publisher.

```bash
~/client_ws/install/int32_publisher_c/lib/int32_publisher_c/./int32_publisher_c
```

You may prefer to run the publisher in the background and discard all outputs to keep using the terminal for the next step.

```bash
 ~/client_ws/install/int32_publisher_c/lib/int32_publisher_c/./int32_publisher_c > /dev/null &
```

Run the subscriber.

```bash
~/client_ws/install/int32_subscriber_c/lib/int32_subscriber_c/./int32_subscriber_c
```

#### Run demonstration (Windows)

Run the micro-ROS Agent.
For the micro-ROS Agent to find the XML reference file, the execution must be done from the executable folder.

```bash
cd C:\A\install\Lib\uros_agent\
uros_agent.exe udp 8888
```

Run the publisher.

```bash
cd C:\C\install\Lib\int32_publisher_c\
int32_publisher_c.exe
```

Run the subscriber.

```bash
cd C:\C\install\Lib\int32_subscriber_c\
int32_subscriber_c.exe
```


### String message demonstration

#### String packages

##### String_publisher

The purpose of the package is to publish a simple string ROS2 message and demonstrate how Micro-ROS layers (rcl, typesupport and rmw) handle it.
For each publication, the message string number increases in one unit order to see in the subscriber side the message variations.

##### String_subscriber

The purpose of the package is to subscribe to a simple string ROS2 message and demonstrate how Micro-ROS layers (rcl, typesupport and rmw) handle it.

#### Run string demonstration (Linux)

Run the micro-ROS Agent.
For the micro-ROS Agent to find the XML reference file, the execution must be done from the executable folder.

```bash
cd ~/agent_ws/install/uros_agent/lib/uros_agent/
./uros_agent udp 8888
```

You may prefer to run the Agent in the background and discard all outputs to keep using the same terminal for the next step.

```bash
cd ~/agent_ws/install/uros_agent/lib/uros_agent/
./uros_agent udp 8888 > /dev/null &
```

Run the publisher.

```bash
 ~/client_ws/install/string_publisher_c/lib/string_publisher_c/./string_publisher_c
```

You may prefer to run the publisher in the background and discard all outputs in order to keep using the terminal for the next step.

```bash
 ~/client_ws/install/string_publisher_c/lib/string_publisher_c/./string_publisher_c > /dev/null &
```

Run the subscriber.

```bash
~/client_ws/install/string_subscriber_c/lib/string_subscriber_c/./string_subscriber_c
```

#### Run string demonstration (Windows)

Run the micro-ROS Agent.
For the micro-ROS Agent to find the XML reference file, the execution must be done from the executable folder.

```bash
cd C:\A\install\Lib\uros_agent\
uros_agent.exe udp 8888
```

Run the publisher.

```bash
cd C:\C\install\Lib\string_publisher_c\
string_publisher_c.exe
```

Run the subscriber.

```bash
cd C:\C\install\Lib\string_subscriber_c\
string_subscriber_c.exe
```

### Complex message demonstration

#### Complex packages

##### complex_msg

One of the purposes of the package is to demonstrate how typesupport code is generated for a complex message.
Also, the generation of a complex ROS2 structure message is used to demonstrate how the different layers (rcl, typesupport and rmw) handle it.
The message structure contains the following types:

- All primitive data types.
- Nested message data.
- Unbonded string data.

##### Complex_msg_publisher

The purpose of the package is to publish a complex ROS2 message and demonstrate how Micro-ROS layers (rcl, typesupport and rmw) handle it.
For each publication, the message values increases in one unit order to see in the subscriber side the message variations.

##### Complex_msg_subscriber

The purpose of the package is to subscribe to a complex ROS2 message and demonstrate how Micro-ROS layers (rcl, typesupport and rmw) handle it.

#### Run complex demonstration (Linux)


Run the micro-ROS Agent

```bash
cd ~/agent_ws/install/uros_agent/lib/uros_agent/
./uros_agent udp 8888
```

You may prefer to run the Agent in the background and discard all outputs to keep using the same terminal for the next step.

```bash
cd ~/agent_ws/install/uros_agent/lib/uros_agent/
./uros_agent udp 8888 > /dev/null &
```

Run the publisher.

```bash
 ~/client_ws/install/complex_msg_publisher_c/lib/complex_msg_publisher_c/./complex_msg_publisher_c
```

You may prefer to run the Agent in the background and discard all outputs to keep using the same terminal for the next step.

```bash
 ~/client_ws/install/complex_msg_publisher_c/lib/complex_msg_publisher_c/./complex_msg_publisher_c > /dev/null &
```

Run the subscriber.

```bash
~/client_ws/install/complex_msg_subscriber_c/lib/complex_msg_subscriber_c/./complex_msg_subscriber_c
```

#### Run complex demonstration (Windows)

Run the micro-ROS Agent.
For the micro-ROS Agent to find the XML reference file, the execution must be done from the executable folder.

```bash
cd C:\A\install\Lib\uros_agent\
uros_agent.exe udp 8888
```

Run the publisher.

```bash
cd C:\C\install\Lib\complex_msg_publisher_c\
complex_msg_publisher_c.exe
```

Run the subscriber.

```bash
cd C:\C\install\Lib\complex_msg_subscriber_c\
complex_msg_subscriber_c.exe
```


### Real application demonstration

This purpose of the packages is to demonstrate Micro-ROS stack can be used in a real application scenario.
In this demonstration, an altitude control system is simulated.
The primary purpose of this is to demonstrate how Micro-ROS communicates with ROS2 nodes.

#### Real application packages

##### rad0_actuator

The mission of this node is to simulate a dummy engine power actuator.
It receives power increments and publishes the total power amount as a DDS topic.

The node is built using the Micro-ROS middleware packages (rmw_micro_xrcedds and rosidl_typesupport_microxrcedds).

It is meant to be running in a microcontroller processor, but for this demonstration, the node runs on the host PC.
The node is connected to the DDS world through a Micro XRCE-DDS Agent.

##### rad0_altitude_sensor

The mission of this node is to simulate a dummy altitude sensor.
It publishes the altitude variations as a DDS topic.

The node is built using the Micro-ROS middleware packages (rmw_micro_xrcedds and rosidl_typesupport_microxrcedds).

It is meant to be running in a microcontroller processor, but for this demonstration, the node runs on the host PC.
The node is connected to the DDS world through a Micro XRCE-DDS Agent.

##### rad0_control

The mission of this node is to read altitude values and send to the actuator engine variations.
It also publishes the status (OK, WARNING or FAILURE) as a DDS topic.
The status depends on the altitude value.

The node is built using the ROS2 middleware packages (rmw_fastrtps and rosidl_typesupport_fastrtps).

It is meant to be running in on a regular PC, and it is directly connected to de DDS world.

##### rad0_display

The mission of this node is to simulate one LCD screen that prints the critical parameters.
It subscribes to the altitude, power and status messages available as a DDS topic.

The node is built using the Micro-ROS middleware packages (rmw_micro_xrcedds and rosidl_typesupport_microxrcedds).

It is meant to be running in a microcontroller processor, but for this demonstration, the node runs on the host PC.
The node is connected to the DDS world through a Micro XRCE-DDS Agent.

#### Run real application demonstration (Linux)

##### Micro-ROS nodes

Run the micro-ROS Agent.
For the micro-ROS Agent to find the XML reference file, the execution must be done from the executable folder.

```bash
cd ~/uros_WS/install/uros_agent/lib/uros_agent/
./uros_agent udp 8888
```

You may prefer to run the Agent in the background and discard all outputs to keep using the same terminal for the next step.

```bash
cd ~/uros_WS/install/uros_agent/lib/uros_agent/
./uros_agent udp 8888 > /dev/null &
```

Run the altitude_sensor node.

```bash
~/client_ws/install/rad0_altitude_sensor_c/lib/rad0_altitude_sensor_c/./rad0_altitude_sensor_c
```

You may prefer to run the publisher in the background and discard all outputs to keep using the terminal for the next steps.

```bash
~/client_ws/install/rad0_altitude_sensor_c/lib/rad0_altitude_sensor_c/./rad0_altitude_sensor_c > /dev/null &
```

Run the actuator node.

```bash
 ~/client_ws/install/rad0_actuator_c/lib/rad0_actuator_c/./rad0_actuator_c
```

You may prefer to run the publisher in the background and discard all outputs to keep using the terminal for the next steps.

```bash
 ~/client_ws/install/rad0_actuator_c/lib/rad0_actuator_c/./rad0_actuator_c > /dev/null &
```

Run the display node.

```bash
~/client_ws/install/rad0_display_c/lib/rad0_display_c/./rad0_display_c
```

##### ROS2 nodes

```bash
~/agent_ws/install/rad0_display_c/lib/rad0_display_c/./rad0_display_c
```

#### Run real application demonstration (Windows)

##### Micro-ROS nodes

Run the micro-ROS Agent.
For the micro-ROS Agent to find the XML reference file, the execution must be done from the executable folder.

```bash
cd C:\A\install\Lib\uros_agent\
uros_agent.exe udp 8888
```

Run the altitude_sensor node.

```bash
cd C:\C\install\Lib\rad0_altitude_sensor_c
rad0_altitude_sensor_c.exe
```

Run the actuator node.

```bash
cd C:\C\install\Lib\rad0_actuator_c
rad0_actuator_c.exe
```

Run the display node.

```bash
cd C:\C\install\Lib\rad0_display_c\
rad0_display_c.exe
```

##### ROS2 nodes

```bash
cd C:\A\install\Lib\rad0_control_cpp\
rad0_control_cpp.exe
```


## Purpose of the Project

This software is not ready for production use. It has neither been developed nor
tested for a specific use case. However, the license conditions of the
applicable Open Source licenses allow you to adapt the software to your needs.
Before using it in a safety relevant setting, make sure that the software
fulfills your requirements and adjust it according to any applicable safety
standards, e.g., ISO 26262.

## License

This repository is open-sourced under the Apache-2.0 license. See the [LICENSE](LICENSE) file for details.


For a list of other open-source components included in this repository,
see the file [3rd-party-licenses.txt](3rd-party-licenses.txt).

## Known Issues/Limitations

There are no known limitations.
