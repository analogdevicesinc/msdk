import os
from os import environ
from subprocess import run
from pathlib import Path

def _validate(returncode:int, fail_msg:str):
    if returncode != 0:
        print(fail_msg)
        exit(returncode)

if __name__ == "__main__":
    _cwd = Path(__file__).parent.resolve()

    if not environ.get("ROS_DISTRO"):
        if Path(f"/opt/ros/humble/setup.bash").exists():
            print("run 'source /opt/ros/humble/setup.bash' before running this script!")
            exit(2)
        else:
            print("Install and set up ROS before running this script!")
            exit(3)
    
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
