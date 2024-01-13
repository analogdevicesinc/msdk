# micro-ROS (beta)

[micro-ROS](https://micro.ros.org/) is a library that puts [ROS 2](https://www.ros.org/) onto microcontrollers.  The Robot Operating System (ROS) is a set of software libraries and tools for building robot applications.

The MSDK maintains support for micro-ROS so that users can integrate microcontrollers directly into the larger ROS ecosystem.  This reduces complexity and system integration overhead while simultaneously speeding up application development.  Additionally, the ROS ecosystem offers a huge set of libraries to enable integrated robotic applications and products.

**The MSDK's micro-ROS support is currently in beta**.  It is not officially released or ready for production applications, but it is ready for beta testing and internal usage.

## Supported Parts

- MAX78000

## Supported ROS Distributions

- [humble](https://docs.ros.org/en/humble/index.html)

## Overview

Fundamentally, ROS enables the creation of publisher/subscriber networks.  Each "node" in the network can publish messages that any other node can subscribe to.  Additionally, a node can provide "services" to provide responses on-demand.

In general, the most popular network transport layer for ROS is USB/Serial.  Other transport layers are possible, but USB/Serial is currently the only transport layer supported by the MSDK.  As a result, a ROS network typically consists of multiple devices connected to a host machine over USB.

For more details on basic ROS concepts see the official [ROS Concepts Documentation](https://docs.ros.org/en/humble/Concepts.html)

![ROS Network](https://docs.ros.org/en/humble/_images/Nodes-TopicandService.gif)

([Image source](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html))

### Why ROS?

Robotics applications can be intricate, involving multiple systems written in different languages.  On top of its network topology, ROS offers a compelling advantage with its standardization of the [interfaces](https://docs.ros.org/en/humble/Concepts/Basic/About-Interfaces.html#) between nodes. ROS provides pre-defined message types that packages and applications can leverage, enabling the modularization of systems and libraries.

For example, the [sensor_msgs](https://index.ros.org/p/sensor_msgs/) messages establish standard "packets" for various data types, such as images, IMU data, temperature measurements, and battery information commonly emitted by low-level sensors. Consider a scenario where a robot needs to query its orientation by polling an IMU, which involves a microcontroller.  ROS facilitates the microcontroller in filling and publishing a standardized message packet, streamlining and decoupling robot development from any custom IMU interface code that might otherwise be necessary.

Additionally, ROS defines datatypes and interfaces comprehensively, supporting most common [field types](https://docs.ros.org/en/humble/Concepts/Basic/About-Interfaces.html#field-types) with built-in type conversions. ROS also offers robust support for defining custom messages, and messages can be transmitted seamlessly across C, C++, and Python code.  No more manual string scraping!

So - ROS is an attractive option for building robots, but historically required a full Linux distribution running on a powerful host machine.  With the development of [micro-ROS](https://micro.ros.org/docs/overview/features/), a ROS client implementation now exists for extremely resource constrained microcontrollers.  It builds on an RTOS layer, and the MSDK provides an implementation that builds on FreeRTOS with future plans for Zephyr support.

## Installation and Setup

One of the disadvantages of ROS is that it can be difficult to set up.  To address this, the MSDK provides an installation script that can streamline these difficulties.

### Requirements

ROS development requires a linux environment with Ubuntu 22.04.  Windows 10 is technically supported by ROS, but the installation is non-trivial and is untested by the MSDK developers.

A native Linux machine is ideal, but development can also be achieved from a virtual machine (recommend [VirtualBox](https://www.virtualbox.org/)), or on Windows via [WSL2](https://learn.microsoft.com/en-us/windows/wsl/).  If developing from a virtual machine or WSL2, it should be noted that some additional steps are required to pass through USB devices.

### install.py

The MSDK offers an installation and setup script [`install.py`](./install.py) that will install everything you need to get started with micro-ROS development.  It will install ROS2 and micro-ROS onto your system, and will also setup and build the micro-ROS Agent - a special "bridge" program that connects micro-ROS applications running on embedded devices to the main ROS network.

`install.py` can be run with Python (version 3+).  Ubuntu 22.04 comes with a [Python](https://www.python.org/) 3 installation by default.  It's accessible as `python3`.  The presence of Python on your system can be validated with the following command:

```shell
$ python3 --version
Python 3.10.12
```

To run the micro-ROS installation, run the following commands where `$MAXIM_PATH` is the root directory of the MSDK.

```shell
$ cd $MAXIM_PATH/Libraries/microROS
$ python3 ./install.py
```

### Manual Installation

If the automatic installer doesn't work, or manual setup is preferred, official documentation can be found below.

1. Follow the [official ROS2 humble installation instructions](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)

2. Follow the official [micro-ROS installation tutorials](https://micro.ros.org/docs/tutorials/core/overview/) and build the [micro-ROS Agent](https://github.com/micro-ROS/micro-ROS-Agent).

## Getting Started with Micro-ROS

### Validating ROS

First, it's a good idea to verify that your ROS installation is working properly.  You can do so with the "Talker-listener" demo.

In a terminal, source the ROS setup file and then run the C++ talker.

```shell
$ source /opt/ros/humble/setup.bash
$ ros2 run demo_nodes_cpp talker
```

You should see the `talker` application publishing "Hello World" messages.

```shell
~/repos/msdk/Libraries/microROS (dev/micro-ros) » ros2 run demo_nodes_cpp talker
[INFO] [1705098027.367870917] [talker]: Publishing: 'Hello World: 1'
[INFO] [1705098028.367865147] [talker]: Publishing: 'Hello World: 2'
[INFO] [1705098029.367871096] [talker]: Publishing: 'Hello World: 3'
[INFO] [1705098030.367898335] [talker]: Publishing: 'Hello World: 4'
[INFO] [1705098031.367917003] [talker]: Publishing: 'Hello World: 5'
```

In a _new_ terminal, source the ROS setup file again.  This time, run a Python listener.

```shell
$ source /opt/ros/humble/setup.bash
$ ros2 run demo_nodes_py listener
```

The listener should start logging the published messages from the talker.

```shell
~/repos/msdk (dev/micro-ros) » ros2 run demo_nodes_py listener
[INFO] [1705098640.330092707] [listener]: I heard: [Hello World: 15]
[INFO] [1705098641.322672227] [listener]: I heard: [Hello World: 16]
[INFO] [1705098642.322685286] [listener]: I heard: [Hello World: 17]
[INFO] [1705098643.322713053] [listener]: I heard: [Hello World: 18]
[INFO] [1705098644.322746222] [listener]: I heard: [Hello World: 19]
```

Having validated the basic publisher/subscriber functionality of the ROS library across its C++ and Python packages, the micro-ROS library is ready for use.

### First micro-ROS Application

The MSDK includes a pre-built micro-ROS library and some micro-ROS examples.  The best way to get started is with the ["ping-pong" application](../../Examples/MAX78000/micro-ROS/ping_pong/).  This will validate that the micro-ROS agent and any micro-ROS application code can communicate correctly, and that the messages from our microcontroller firmware are accessible in the ROS network.  It's a good application to get familiar with the basic concepts and tools.

This section will use VS Code and the MAX78000FTHR to demonstrate the application's functionality.

1. Open the ["ping-pong" application](../../Examples/MAX78000/micro-ROS/ping_pong/).

2. Set the [Board Support Package](https://analog-devices-msdk.github.io/msdk/USERGUIDE/#board-support-packages) correctly for your evaluation platform.

3. Connect your board to your PC.  Take note of the serial port it presents.

4. Build the firmware, but don't flash or run it yet.  ([how to build projects](https://analog-devices-msdk.github.io/msdk/USERGUIDE/#build-tasks))

5. In a new terminal, source the ROS setup script _and_ the micro-ROS setup script.  Run the following commands, where `$MAXIM_PATH` is the root directory of the MSDK.

```shell
$ source /opt/ros/humble/setup.bash
$ source $MAXIM_PATH/Libraries/microROS/install/local_setup.zsh
```

6. Now, connect the micro-ROS agent to the serial port presented by your evaluation platform.  It may vary, but in the example below the MAX78000FTHR has presented `/dev/ttyACM0`.

```shell
$ ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 --baud 115200
```

The output should look something like this.

```shell
[1705101610.297841] info     | TermiosAgentLinux.cpp | init                     | running...             | fd: 3
[1705101610.298004] info     | Root.cpp           | set_verbose_level        | logger setup           | verbose_level: 4
```

The micro-ROS Agent has opened the serial port and is now listening for any connections from our micro-ROS firmware.

7. While keeping the micro-ROS Agent open, [Flash and run](https://analog-devices-msdk.github.io/msdk/USERGUIDE/#flash-run) the "ping-pong" application.

After 2 seconds, you should see the micro-ROS agent register the application firmware and its ping-pong publishers/subscribers.  The agent log should look something like this:

```shell
~/repos/msdk/Examples/MAX78000/micro-ROS/ping_pong (dev/micro-ros) » ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 --baud 115200

[1705101610.297841] info     | TermiosAgentLinux.cpp | init                     | running...             | fd: 3
[1705101610.298004] info     | Root.cpp           | set_verbose_level        | logger setup           | verbose_level: 4
[1705101781.408763] info     | Root.cpp           | create_client            | create                 | client_key: 0x185B87C0, session_id: 0x81
[1705101781.408812] info     | SessionManager.hpp | establish_session        | session established    | client_key: 0x185B87C0, address: 0
[1705101781.424575] info     | ProxyClient.cpp    | create_participant       | participant created    | client_key: 0x185B87C0, participant_id: 0x000(1)
[1705101781.437705] info     | ProxyClient.cpp    | create_topic             | topic created          | client_key: 0x185B87C0, topic_id: 0x000(2), participant_id: 0x000(1)
[1705101781.447689] info     | ProxyClient.cpp    | create_publisher         | publisher created      | client_key: 0x185B87C0, publisher_id: 0x000(3), participant_id: 0x000(1)
[1705101781.458791] info     | ProxyClient.cpp    | create_datawriter        | datawriter created     | client_key: 0x185B87C0, datawriter_id: 0x000(5), publisher_id: 0x000(3)
[1705101781.474715] info     | ProxyClient.cpp    | create_topic             | topic created          | client_key: 0x185B87C0, topic_id: 0x001(2), participant_id: 0x000(1)
[1705101781.484684] info     | ProxyClient.cpp    | create_publisher         | publisher created      | client_key: 0x185B87C0, publisher_id: 0x001(3), participant_id: 0x000(1)
[1705101781.495768] info     | ProxyClient.cpp    | create_datawriter        | datawriter created     | client_key: 0x185B87C0, datawriter_id: 0x001(5), publisher_id: 0x001(3)
[1705101781.510728] info     | ProxyClient.cpp    | create_topic             | topic created          | client_key: 0x185B87C0, topic_id: 0x002(2), participant_id: 0x000(1)
[1705101781.521716] info     | ProxyClient.cpp    | create_subscriber        | subscriber created     | client_key: 0x185B87C0, subscriber_id: 0x000(4), participant_id: 0x000(1)
[1705101781.532838] info     | ProxyClient.cpp    | create_datareader        | datareader created     | client_key: 0x185B87C0, datareader_id: 0x000(6), subscriber_id: 0x000(4)
[1705101781.550777] info     | ProxyClient.cpp    | create_topic             | topic created          | client_key: 0x185B87C0, topic_id: 0x003(2), participant_id: 0x000(1)
[1705101781.560689] info     | ProxyClient.cpp    | create_subscriber        | subscriber created     | client_key: 0x185B87C0, subscriber_id: 0x001(4), participant_id: 0x000(1)
[1705101781.571767] info     | ProxyClient.cpp    | create_datareader        | datareader created     | client_key: 0x185B87C0, datareader_id: 0x001(6), subscriber_id: 0x001(4)
```

You will also notice an LED on the board blinking every 2 seconds.  This indicates that the application firmware is publishing "pings".  Next, we will examine those pings using ROS.

8. In a _new_ terminal, source the ROS setup script again.  Then, echo the `/microROS/ping` topic with the command below.

```shell
$ source /opt/ros/humble/setup.bash
$ ros2 topic echo /microROS/ping
```

You should start seeing the "ping" messages being logged to the terminal, and the time stamp for each one.

```shell
~/repos/msdk/Examples/MAX78000/micro-ROS/ping_pong (dev/micro-ros*) » ros2 topic echo /microROS/ping

stamp:
  sec: 177
  nanosec: 178955072
frame_id: '1654288741_1122375324'
---
stamp:
  sec: 179
  nanosec: 178955072
frame_id: '1239141039_1122375324'
---
stamp:
  sec: 181
  nanosec: 178955072
frame_id: '279575609_1122375324'
---
stamp:
  sec: 183
  nanosec: 178955072
frame_id: '1601257771_1122375324'
---
stamp:
  sec: 185
  nanosec: 178955072
frame_id: '858013871_1122375324'
```

9. Now, we will test the "pong" functionality of the application by manually publishing a message into the ROS network.  First, open a _new_ terminal and source the ROS setup script.  Echo the `/microROS/pong` topic.

```shell
$ source /opt/ros/humble/setup.bash
$ ros2 topic echo /microROS/pong
```

10. In (yet another) _new_ terminal, source the ROS setup script.  Then, run the command below to publish a ping message into the network.

```shell
$ source /opt/ros/humble/setup.bash
$ ros2 topic pub --once /microROS/ping std_msgs/msg/Header '{frame_id: "fake_ping"}'
```

The output should look something like this.

```shell
~/repos/msdk/Examples/MAX78000/micro-ROS/ping_pong (dev/micro-ros*) » ros2 topic pub --once /microROS/ping std_msgs/msg/Header '{frame_id: "fake_ping"}' 
publisher: beginning loop
publishing #1: std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=0, nanosec=0), frame_id='fake_ping')
```

In the terminal that's echoing the `/microROS/pong` topic, you should now see a "pong" stamped with the "fake_ping" `frame_id`.

```shell
~/repos/msdk/Examples/MAX78000/micro-ROS/ping_pong (dev/micro-ros) » ros2 topic echo /microROS/pong
stamp:
  sec: 0
  nanosec: 0
frame_id: fake_ping
---
```

In the terminal that's echoing the `/microROS/ping` topic, you should also see the "ping" of the "fake_ping" message inserted as it was being published into the network.

```shell
---
stamp:
  sec: 240
  nanosec: 616210944
frame_id: '1266118970_1122375324'
---
stamp:
  sec: 0
  nanosec: 0
frame_id: fake_ping
---
stamp:
  sec: 241
  nanosec: 226318352
frame_id: '1279849170_1122375324'
```

Congratulations!  You have successfully published and subscribed to your first messages through the ROS network, and validated that the MSDK's micro-ROS library and setup are working.

11. When you are finished, use `CTRL+C` to kill any active terminals.

### Automatically Sourcing Setup Scripts

ROS development demands a lot of active terminals.  Manually `source`-ing the setup scripts for each one can be cumbersome.  The setup scripts can be automatically sourced by modifying your shell's startup script.

#### Bash

For Bash, edit `~/.bashrc` to append the following lines.

```shell
```bash
# Set MAXIM_PATH (if you haven't already)
export MAXIM_PATH=$HOME/repos/msdk

# ROS/micro-ROS setup
source /opt/ros/humble/setup.bash
source $MAXIM_PATH/Libraries/microROS/install/local_setup.bash
```

Reload your active shell with `source ~/.bashrc`

#### Zsh

For Zsh, edit `~/.zshrc` to append the following lines

```bash
# Set MAXIM_PATH (if you haven't already)
export MAXIM_PATH=$HOME/repos/msdk

# ROS/micro-ROS setup
source /opt/ros/humble/setup.zsh
source $MAXIM_PATH/Libraries/microROS/install/local_setup.zsh
```

Reload your active shell with `source ~/.zshrc`
