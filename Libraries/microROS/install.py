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
import subprocess
from subprocess import run, CompletedProcess, Popen, check_call
from pathlib import Path
import logging
import traceback
import platform

_cwd = Path(__file__).parent.resolve()

# Set up logger to stream to file and console simultaneously.  Any debug messages will
# go to the file only
log_file = Path(f"{_cwd}/log/install.log")
logging.basicConfig(filename=log_file, encoding="utf-8", level=logging.DEBUG)
console = logging.StreamHandler(sys.stdout)
console.setLevel(logging.INFO)
formatter = logging.Formatter("%(levelname)-8s: %(message)s")
console.setFormatter(formatter)
logging.getLogger().addHandler(console)

def _validate(process_result:CompletedProcess, fail_msg:str):
    if process_result.returncode != 0:
        logging.error(f"{fail_msg}\n\nPlease report to https://github.com/Analog-Devices-MSDK/msdk/issues\nAttach {log_file} to your ticket")
        exit(process_result.returncode)

def log_cmd(cmd, **kwargs):
    logging.debug(f"Running command: {cmd}")
    _env = environ
    _env["PYTHONUNBUFFERED"] = "1"
    stdout = ""
    with Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, encoding="utf-8", env=_env, **kwargs) as process:  
        for line in process.stdout:
            print(line, end="")
            stdout += line

        return_code=process.wait()

    logging.debug(f"Output\n{stdout}")

    return CompletedProcess(cmd, return_code, stdout, process.stderr)

def install_ros():
    logging.info("Installing ROS2 'humble'")
    result = log_cmd("sudo apt install software-properties-common -y", shell=True)
    _validate(result, "Failed to add Ubuntu Universe repo!")
    result = log_cmd("sudo add-apt-repository universe -y", shell=True)
    _validate(result, "Failed to add Ubuntu Universe repo!")

    result = log_cmd("sudo apt update && sudo apt install curl -y", shell=True)
    _validate(result, "Failed to install curl!")
    result = log_cmd("sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg", shell=True)
    _validate(result, "Failed to add ROS2 GPG key!")

    result = log_cmd('echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null', shell=True)
    _validate(result, "Failed to add ROS repository!")

    result = log_cmd("sudo apt update && sudo apt upgrade", shell=True)
    _validate(result, "Failed to update apt packages!")

    result = log_cmd("sudo apt install ros-humble-desktop -y", shell=True)
    _validate(result, "Failed to install ros!")

    logging.info("Successfully installed ROS2 'humble'!")

def install_microros():
    result = run("colcon --help", shell=True, capture_output=True)
    if result.returncode != 0:
        logging.info("Installing colcon")
        result = run("sudo apt update && sudo apt install python3-colcon-common-extensions -y", shell=True, stdout=PIPE, stderr=PIPE)
        _validate(result, "Failed to install colcon!")

    else:
        logging.info(f"Found colcon")
        result = log_cmd("colcon version-check", shell=True)
        _validate(result, "Failed to retrieve colcon version list!")

    result = run("rosdep --version", shell=True, capture_output=True)    
    if result.returncode != 0:
        result = log_cmd("sudo apt install python3-rosdep -y", shell=True)
        _validate(result, "Failed to install rosdep!")

        result = log_cmd("sudo rosdep init", shell=True)
        _validate(result, "Failed to initialize rosdep!")
    
    result = log_cmd("rosdep update", shell=True)
    _validate(result, "Failed to update rosdep!")

    result = log_cmd("rosdep install --from-paths src/micro_ros_setup --ignore-src -y", cwd=_cwd, shell=True)
    _validate(result, "Failed to install micro_ros_setup dependencies!")
    
    result = log_cmd("colcon build --paths src/micro_ros_setup", cwd=_cwd, shell=True)
    _validate(result, "Failed to build micro_ros_setup!")

    result = log_cmd("colcon build --paths src/micro_ros_setup", cwd=_cwd, shell=True)
    _validate(result, "Failed to build micro_ros_setup!")

    local_source = f"source install/local_setup.bash"
    result = log_cmd(f"{local_source} && ros2 run micro_ros_setup create_agent_ws.sh", shell=True, executable="/bin/bash", cwd=_cwd)
    _validate(result, "Failed to create agent workspace!")

    result = log_cmd(f"{local_source} && ros2 run micro_ros_setup build_agent.sh", shell=True, executable="/bin/bash", cwd=_cwd)
    _validate(result, "Failed to build agent!")

def main():
    logging.info(f"Running MSDK micro-ROS install.py for {platform.platform()}")
    logging.info(f"Using Python {sys.version}")

    if not environ.get("ROS_DISTRO"):
        if Path(f"/opt/ros/humble/setup.bash").exists():
            logging.info("Detected ROS 'humble'")
            logging.error("run 'source /opt/ros/humble/setup.bash' before running this script!")
            exit(2)
        else:
            logging.warning("No ROS distro detected on your system.")
            confirm = input("Would you like this script to auto-install ROS humble now? [y/n]")
            if confirm.lower() == "y":
                install_ros()
                logging.debug("Source-ing ROS setup and re-running script")
                run(f"source /opt/ros/humble/setup.bash && {sys.executable} install.py", shell=True, executable="/bin/bash", cwd=_cwd)
                exit(0)
            else:
                logging.info("Aborting.")
                exit(3)
    else:
        logging.info(f"Found ROS '{environ['ROS_DISTRO']}'")

    if not (sys.version_info.major == 3 and sys.version_info.minor == 10 and sys.version_info.minor == 12):
        logging.warning("ROS and micro-ROS depend on Python 3.10.12!  Your current python version does not match.")
        logging.warning("Untested Python version detected...  Failures may occur.  You have been warned!...")
        response = input("Would you like to continue?... [y/n]")
        if response.lower() != "y":
            logging.info("Aborting.")
            exit(3)

    install_microros()

    logging.info("Success!  micro-ROS is now ready to use with the MSDK.")
    logging.info(f"You can now source the {_cwd / 'install/local_setup.bash'} script when you want to run the micro-ROS agent.")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        # This will get triggered on CTRL+C, so we handle it separately to cleanly exit
        # without traceback
        exit(1)
    except SystemExit as e:
        # Pass through system exits
        exit(e.code)
    except:
        logging.error(f"There was a problem with the 'install.py' script.\n{traceback.format_exc()}")
        print("Please report this issue to https://github.com/Analog-Devices-MSDK/msdk/issues")
        exit(1)
