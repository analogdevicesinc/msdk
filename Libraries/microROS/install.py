###############################################################################
 #
 # Copyright (C) 2024 Analog Devices, Inc. All Rights Reserved. This software
 # is proprietary to Analog Devices, Inc. and its licensors.
 #
 # Licensed under the Apache License, Version 2.0 (the "License");
 # you may not use this file except in compliance with the License.
 # You may obtain a copy of the License at
 #
 #     http://www.apache.org/licenses/LICENSE-2.0
 #
 # Unless required by applicable law or agreed to in writing, software
 # distributed under the License is distributed on an "AS IS" BASIS,
 # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 # See the License for the specific language governing permissions and
 # limitations under the License.
 #
 ##############################################################################

import os
import sys
from os import environ
from subprocess import run
from pathlib import Path

def _validate(returncode:int, fail_msg:str):
    if returncode != 0:
        print(fail_msg)
        exit(returncode)

def install_ros():
    result = run("sudo apt install software-properties-common -y", shell=True)
    _validate(result.returncode, "Failed to add Ubuntu Universe repo!")
    result = run("sudo add-apt-repository universe -y", shell=True)
    _validate(result.returncode, "Failed to add Ubuntu Universe repo!")

    result = run("sudo apt update && sudo apt install curl -y", shell=True)
    _validate(result.returncode, "Failed to install curl!")
    result = run("sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg", shell=True)
    _validate(result.returncode, "Failed to add ROS2 GPG key!")

    result = run('echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null', shell=True)
    _validate(result.returncode, "Failed to add ROS repository!")

    result = run("sudo apt update && sudo apt upgrade", shell=True)
    _validate(result.returncode, "Failed to update apt packages!")

    result = run("sudo apt install ros-humble-desktop -y", shell=True)
    _validate(result.returncode, "Failed to install ros!")

    print("Successfully installed ros humble.")

def install_microros():
    result = run("colcon --help", shell=True, capture_output=True)
    if result.returncode != 0:
        print("Installing colcon")
        result = run("sudo apt update && sudo apt install python3-colcon-common-extensions -y", shell=True)
        _validate(result.returncode, "Failed to install colcon!")

    else:
        print(f"Found colcon")
        result = run("colcon version-check", shell=True)
        _validate(result.returncode, "Failed to retrieve colcon version list!")

    result = run("rosdep --version", shell=True, capture_output=True)
    if result.returncode != 0:
        result = run("sudo apt install python3-rosdep -y", shell=True)
        _validate(result.returncode, "Failed to install rosdep!")

        result = run("sudo rosdep init", shell=True)
        _validate(result.returncode, "Failed to initialize rosdep!")
    
    result = run("rosdep update", shell=True)
    _validate(result.returncode, "Failed to update rosdep!")

    result = run("rosdep install --from-paths src/micro_ros_setup --ignore-src -y", cwd=_cwd, shell=True)
    _validate(result.returncode, "Failed to install micro_ros_setup dependencies!")
    
    result = run("colcon build --paths src/micro_ros_setup", cwd=_cwd, shell=True)
    _validate(result.returncode, "Failed to build micro_ros_setup!")

    result = run("colcon build --paths src/micro_ros_setup", cwd=_cwd, shell=True)
    _validate(result.returncode, "Failed to build micro_ros_setup!")

    local_source = f"source install/local_setup.bash"
    result = run(f"{local_source} && ros2 run micro_ros_setup create_agent_ws.sh", shell=True, executable="/bin/bash", cwd=_cwd)
    _validate(result.returncode, "Failed to create agent workspace!")

    result = run(f"{local_source} && ros2 run micro_ros_setup build_agent.sh", shell=True, executable="/bin/bash", cwd=_cwd)
    _validate(result.returncode, "Failed to build agent!")

if __name__ == "__main__":
    _cwd = Path(__file__).parent.resolve()

    if not environ.get("ROS_DISTRO"):
        if Path(f"/opt/ros/humble/setup.bash").exists():
            print("ROS humble detected!")
            print("run 'source /opt/ros/humble/setup.bash' before running this script!")
            exit(2)
        else:
            print("No ROS distro detected on your system.")
            confirm = input("Would you like this script to auto-install ROS humble now? [y/n]")
            if confirm.lower() == "y":
                install_ros()
                run(f"source /opt/ros/humble/setup.bash && {sys.executable} install.py", shell=True, executable="/bin/bash", cwd=_cwd)
                exit(0)
            else:
                print("Aborting.")
                exit(3)
    else:
        print(f"Found ROS {environ['ROS_DISTRO']}")

    install_microros()

    print("Success!")
