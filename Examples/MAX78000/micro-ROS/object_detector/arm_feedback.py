import rclpy
from rclpy.node import Node
from sensor_msgs.msg import RegionOfInterest
from copy import deepcopy
from pathlib import Path

from vector import VectorObject2D as Vec2D
from vector import VectorObject3D as Vec3D

from time import sleep

from cv_bridge import CvBridge
import cv2

from PIL import ImageDraw as PILImageDraw
from PIL import Image as PILImage

from open_manipulator_msgs.msg import KinematicsPose, OpenManipulatorState
from open_manipulator_msgs.srv import SetJointPosition, SetKinematicsPose
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Image
from geometry_msgs.msg import PolygonStamped, Point32
# from rclpy.executors import Executor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
import os
import select
import sys
import math

from serial.tools import list_ports
from threading import Thread
import subprocess
import traceback

from console_ui import *
import datetime

global g_stopped
g_stopped = False

# home = [1.514991, -0.754719, 0.391165, 1.184233, -0.01]
# home = [1.495631, -1.156622, 0.302194, 1.572330, -0.01]
# home = [1.552389, -0.986350, 0.076699, 1.711923, -0.01]
home = [1.512505, -0.512350, -0.058291, 1.644427, 0.01]
home_xyz = Vec3D(x=0.02, y=0.124, z=0.152)
shutdown_pos = [1.613748, 0.090505, 0.391165, 1.092194, 0.01]

present_joint_angle = deepcopy(home)
goal_joint_angle = deepcopy(home)
prev_goal_joint_angle = [0.0, 0.0, 0.0, 0.0]
present_kinematics_pose = [0.009, 0.147, 0.162, 0.666, -0.221, 0.217, 0.679]
goal_kinematics_pose = [0.009, 0.147, 0.162, 0.666, -0.221, 0.217, 0.679]
prev_goal_kinematics_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
height_drop = 0
drop_angle = []
drop_off_point = [-0.11, 0.2]

debug = True
task_position_delta = (0.01 / 3) * 1.3  # meter
joint_angle_delta = (0.15 * 1.3)  # radian
global path_time
path_time = 0.2 # second
skew_tolerance = 0.3
dry_run = False

g_img: Image = None
valid_image = False

class OMX_Controller_Manager():
    port: str = None
    serial_thread : Thread = None
    stop = False
    console: ConsolePanel
    status: Status

    def __init__(self, port, console: ConsolePanel, status: Status):
        self.serial_thread = Thread(target=self._run, name="OMX_Controller")
        self.port = port
        self.console = console
        self.status = status
        self._status("[b][red]Uninitialized[/b][/red]")

    def start(self):
        self.serial_thread.start()

    def _print(self, *args, **kwargs):
        self.console.print(*args, **kwargs)

    def _status(self, status: str):
        self.status.update(f"OpenManipulator-X Controller (OpenCR1.0) [i]({self.port})[/i]: {status}")

    def _run(self):
        self._status("[yellow]Starting OMX controller[/yellow]")
        cmd = f"ros2 launch open_manipulator_x_controller open_manipulator_x_controller.launch.py usb_port:={self.port}"
        with subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, encoding="utf-8", bufsize=0, shell=True) as process:
            self._status(f"[b][green]Connected[/b][/green]")
            for line in process.stdout:
                self._print(line, end="")

        if process.returncode != 0:
            self._print(f"OMX controller quit with error code {process.returncode}")

        self._status("[b][red]Disconnected[/b][/red]")

class microROS_Agent_Manager():
    port: str = None
    serial_thread : Thread = None
    stop: bool = False
    baud: int
    console: ConsolePanel
    status: Status
    connected = False

    def __init__(self, port, console: ConsolePanel, status: Status, baud=115200):
        self.serial_thread = Thread(target=self._run, name="CAM02_microROS_Agent")
        self.port = port
        self.baud = baud
        self.console = console
        self.status = status
        self._status("[b][red]Uninitialized[/b][/red]")

    def start(self):
        self.serial_thread.start()

    def _print(self, *args, **kwargs):
        self.console.print(*args, **kwargs)

    def _status(self, status: str):
        self.status.update(f"microROS Agent[i] ({self.port})[/i]: {status}")

    def _reset_cam02(self):
        self._status("[yellow]Resetting MAX78000CAM02...[/yellow]")
        cmd = "openocd -s $MAXIM_PATH/Tools/OpenOCD/scripts -f interface/cmsis-dap.cfg -f target/max78000.cfg -c \"init;reset;exit\""
        with subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, encoding="utf-8", bufsize=0, shell=True) as process:
            for line in process.stdout:
                self._print(line, end="")

        if process.returncode != 0:
            self._status(f"[b][red]Failed to reset CAM02 (err {process.returncode})[/red][/b]")
            self._print("Check power to MAX78000CAM02")
            self._print("Check PICO connection")
            return False
        
        return True

    def _run(self):
        if not self._reset_cam02():
            return
        else:
            time.sleep(0.5)
        self._status("[yellow]Starting microROS Agent...[/yellow]")
        cmd = f"ros2 run micro_ros_agent micro_ros_agent serial --dev {self.port} -b {self.baud}"
        with subprocess.Popen(f"exec {cmd}", stdout=subprocess.PIPE, stderr=subprocess.STDOUT, encoding="utf-8", bufsize=0, shell=True) as process:
            self._status(f"[b][green]Establishing session...[/b][/green]")
            for line in process.stdout:
                self._print(line, end="")
                if "error" in line:
                    self._status("[b][red]Disconnected[/b][/red]")
                    return
                elif "session established" in line:
                    self._status("[b][green]Connected[/b][/green]")
                    self.connected = True

        if process.returncode != 0:
            self._print(f"microROS agent quit with error code {process.returncode}")

        self._status("[b][red]Disconnected[/b][/red]")
    

def convert_sensorsmsgs_image_to_pil(image:Image) -> PILImage:
    img = CvBridge().imgmsg_to_cv2(image)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = PILImage.fromarray(img)
    return img

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)
      
        self.subscription = self.create_subscription(
            Image,
            '/microROS/image',
            self.listener_callback, 
            qos_profile = qos_profile)
   
    def listener_callback(self, data: Image):
        self.get_logger().info(f'Received image {data.header.frame_id}')

        # Convert to PIL image so we can draw to it.
        global g_img
        g_img = data

        convert_sensorsmsgs_image_to_pil(data).save(f"{data.header.frame_id}.png")

class BoxSubscriber(Node):
  x: int
  y: int

  def __init__(self):
    # Initiate the Node class's constructor and give it a name
    super().__init__('box_subscriber')
      
    qos_profile = QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1)

    self.subscription = self.create_subscription(
        RegionOfInterest,
        '/microROS/roi', 
        self.listener_callback,
        qos_profile = qos_profile)
    
    self.x = 0
    self.y = 0
   
  def listener_callback(self, data:RegionOfInterest):
    """
    Callback function.
    """
    # Display the message on the console
    self.get_logger().info('Received bounding box:')
    self.get_logger().info(f"\tx_offset: {data.x_offset}")
    self.get_logger().info(f"\ty offset: {data.y_offset}")
    self.get_logger().info(f"\twidth: {data.width}")
    self.get_logger().info(f"\theight: {data.height}")
    self.x = data.x_offset + (data.width / 2)
    self.y = data.y_offset - (data.height / 2)
    self.get_logger().info(f"\tmiddle: {self.x},{self.y}")

    if self.x > 0 and self.y > 0:
        if (self.x < (160 / 2)):
           step_size = joint_angle_delta * (1 * ((80 - self.x)/80.0))
           goal_joint_angle[0] -= step_size
        elif (self.x > (160 / 2)):
           step_size = joint_angle_delta * (1 * ((self.x - 80)/80.0))
           goal_joint_angle[0] += step_size

        if (self.y < (120 / 2)):
           step_size = (joint_angle_delta / 2) * (1 * ((60 - self.y)/60.0))
           goal_joint_angle[1] -= step_size
           goal_joint_angle[3] -= (step_size / 3)
        elif (self.y > (120 / 2)):
           step_size = (joint_angle_delta / 2) * (1 * ((self.y - 60)/60.0))
           goal_joint_angle[1] += step_size
           goal_joint_angle[3] += (step_size / 3)

        teleop_keyboard.send_goal_joint_space()

        self.x = 0
        self.y = 0
        
class PolygonSubscriber(Node):
    x: int
    y: int

    state = -1
    img = None

    last_left : Vec2D = None
    last_top : Vec2D = None
    last_right : Vec2D = None
    last_bottom : Vec2D = None

    box_to_process: bool = False
    last_timestamp: datetime.datetime = None
    search_direction = 0
    grabbability:float = 0.0

    def __init__(self, console: ConsolePanel, status: Status):
        super().__init__('polygon_subscriber')

        self.console = console
        self.status = status
        self.last_timestamp = datetime.datetime.now()

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)

        self.subscription = self.create_subscription(
          PolygonStamped,
          '/microROS/polygon', 
          self.listener_callback,
          qos_profile = qos_profile)

        self.x = 0
        self.y = 0

    def _print(self, *args, **kwargs):
        self.console.print(*args, **kwargs)

    def _status(self, status:str):
        self.status.update(f"[b]{status}[/b]\t(x={round(present_kinematics_pose[0],3)}, y={round(present_kinematics_pose[1],3)}, z={round(present_kinematics_pose[2],3)})")

    def correct_height(self):
        self._status("Correcting height")
        global goal_kinematics_pose
        global present_kinematics_pose
        goal_kinematics_pose = deepcopy(present_kinematics_pose)
        current_orientation = Vec3D(x=1.0, y=0.0, z=0.0).rotate_quaternion(
            u=goal_kinematics_pose[3], # w
            i=goal_kinematics_pose[4], # x
            j=goal_kinematics_pose[5], # y
            k=goal_kinematics_pose[6]  # z
        ).unit()
        self._print(current_orientation)
        goal_kinematics_pose[0] += current_orientation.x * task_position_delta
        goal_kinematics_pose[1] += current_orientation.y * task_position_delta
        goal_kinematics_pose[2] += current_orientation.z * task_position_delta
        
        self._print(f"--- AREA: {self.area}")

        teleop_keyboard.send_goal_task_space()
    
    def correct_skew(self):
        global goal_kinematics_pose
        global present_kinematics_pose
        goal_kinematics_pose = deepcopy(present_kinematics_pose)
        current_xy = Vec2D(x = goal_kinematics_pose[0], y = goal_kinematics_pose[1])
        delta_xy : Vec2D = current_xy.unit()
        if (self.skew < 0.9):
            step_size = task_position_delta * (1 * ((0.9 - self.skew)/0.9))
            goal_kinematics_pose[0] += (delta_xy.x * step_size)
            goal_kinematics_pose[1] += (delta_xy.y * step_size)
            goal_kinematics_pose[2] += (task_position_delta / 2)
        elif (self.skew > 1.1):
            step_size = task_position_delta * (1 * ((self.skew - 1.1)/1.1))
            goal_kinematics_pose[0] -= (delta_xy.x * step_size)
            goal_kinematics_pose[1] -= (delta_xy.y * step_size)
            goal_kinematics_pose[2] += (task_position_delta / 2)

        teleop_keyboard.send_goal_task_space()
       
    def correct_pos(self):
        self._status("Correcting position")
        global goal_joint_angle
        global present_joint_angle
        goal_joint_angle = deepcopy(present_joint_angle)
        if (self.x < (160 / 2)):
            step_size = joint_angle_delta * (1 * ((80 - self.x)/80.0))
            goal_joint_angle[0] -= step_size
        elif (self.x > (160 / 2)):
            step_size = joint_angle_delta * (1 * ((self.x - 80)/80.0))
            goal_joint_angle[0] += step_size

        if (self.y < (120 / 2)):
            step_size = ((joint_angle_delta / 2) * (1 * ((60 - self.y)/60.0))) * 2
            goal_joint_angle[1] -= (step_size * 0.3)
            goal_joint_angle[3] -= (step_size)
        elif (self.y > (120 / 2)):
            step_size = ((joint_angle_delta / 2) * (1 * ((self.y - 60)/60.0))) * 2
            goal_joint_angle[1] -= (step_size * 0.3)
            goal_joint_angle[3] += (step_size)

        teleop_keyboard.send_goal_joint_space()

    def grab_reposition(self):
        self._status("Repositioning grabber")
        global goal_kinematics_pose
        global present_kinematics_pose
        goal_kinematics_pose = deepcopy(present_kinematics_pose)
        xy = Vec2D(x = present_kinematics_pose[0], y = present_kinematics_pose[1])

        # The OMX gives us a quaternion vector representing the current rotation.  If we rotate the initial position (1,0,0) by
        # the quaternion we get an 3D carteisan orientation vector relative to the current position.
        # We will use this vector to move "towards" the cube.
        current_orientation = Vec3D(x=1.0, y=0.0, z=0.0).rotate_quaternion(
            u=goal_kinematics_pose[3], # w
            i=goal_kinematics_pose[4], # x
            j=goal_kinematics_pose[5], # y
            k=goal_kinematics_pose[6]  # z
        ).unit()

        # Now, we need to correct for the offset between the camera's position and the center point of the gripper.
        # Essentially we want to move "down", but the vector representing "down" will change relative to the current orientation.
 
        # Calculate a plane along the "center line" of the arm.  We do this by taking the (x,y) component of the current
        # orientation and crossing it with the Z axis.  This gives us a normal vector that defines the plane.
        center_plane_normal = Vec3D(x=current_orientation.x, y=current_orientation.y, z=0).cross(Vec3D(x=0, y=0, z=1)).unit()

        # Now we want to calculate a vector that is perpendicular to the current orientation vector *along* the axis of
        # the center-line plane.  The resulting vector will be perpendicular to *both* the center-line plane's normal
        # vector *and* the current orientation vector.  Therefore a cross-product between the two gives us the desired result.
        # The direction of the vector is desired as well (based on the right-hand rule)
        cube_offset_vector = current_orientation.cross(center_plane_normal).unit()

        correction = (current_orientation * 0.03) + (cube_offset_vector * 0.04) # Note the ratio between the correction factors is 50/50.  If the camera pos were different, the ratio here can be tuned.
        goal_kinematics_pose[0] += correction.x
        goal_kinematics_pose[1] += correction.y
        goal_kinematics_pose[2] += correction.z

        teleop_keyboard.send_goal_task_space()

    def grab(self):
        self._status("Grabbing object")
        goal_joint_angle[4] = 0.002
        teleop_keyboard.send_goal_tool_control()

    def search_forward(self):
        global goal_kinematics_pose
        global present_kinematics_pose
        xy = Vec2D(x = present_kinematics_pose[0], y = present_kinematics_pose[1])
        movement_vector = xy.unit() * (task_position_delta * 20)
        goal_kinematics_pose[0] += movement_vector.x
        goal_kinematics_pose[1] += movement_vector.y

        global path_time
        teleop_keyboard.send_goal_task_space(interruptable=True, time=5.0)

    def search_backward(self):
        global goal_kinematics_pose
        global present_kinematics_pose
        xy = Vec2D(x = present_kinematics_pose[0], y = present_kinematics_pose[1])
        movement_vector = xy.unit() * (task_position_delta * 20)
        goal_kinematics_pose[0] -= movement_vector.x
        goal_kinematics_pose[1] -= movement_vector.y

        global home_xyz
        home_drift = home_xyz - Vec3D(x = present_kinematics_pose[0], y = present_kinematics_pose[1], z = present_kinematics_pose[2])
        home_drift *= 0.2
        goal_kinematics_pose[0] += home_drift.x
        goal_kinematics_pose[1] += home_drift.y
        goal_kinematics_pose[2] += home_drift.z

        global path_time
        teleop_keyboard.send_goal_task_space(interruptable=True, time=5.0)

    def process_state(self):
        global path_time
        global present_joint_angle
        global goal_joint_angle
        global goal_kinematics_pose
        global drop_off_point
        global next_drop_off_point

        if self.state == 10: # Search
            self._status(f"[yellow]Searching[/yellow]... ({(datetime.datetime.now() - self.last_timestamp).seconds}s)")
            if self.x > 0 and self.y > 0:
                self._print("Found!")
                self.state = 0
                self.search_direction = 0 if self.search_direction == 1 else 0
                # ^ If we have a match, we generally want to keep moving in the same search direction
            elif teleop_keyboard.state != "IS_MOVING" and not teleop_keyboard.locked and not teleop_keyboard.wait_to_move:
                if self.search_direction == 0:
                    self.search_forward()
                    self.search_direction = 1
                elif self.search_direction == 1:
                    self.search_backward()
                    self.search_direction = 0

        elif self.state == -1: # Init
            if self.x > 0 and self.y > 0:
                self.state = 0

        elif (self.x == 0 and self.y == 0) and (datetime.datetime.now() - self.last_timestamp).seconds >= 1 and not self.state == 10:
            self.state = 10
            goal_kinematics_pose = deepcopy(present_kinematics_pose) # Copy current pose so that the current orientation is preserved for the search
            self._status("[yellow]Searching...[/yellow]")

        elif self.x > 0 and self.y > 0 and ((teleop_keyboard.state != "IS_MOVING" and not teleop_keyboard.locked and not teleop_keyboard.wait_to_move) or teleop_keyboard.interruptable):

            if self.state < 4:
                area_good = self.area >= 4400
                # print(f"*** AREA GOOD: {area_good}")
                skew_good = math.fabs(self.skew - 1.0) < 0.1
                # print(f"*** SKEW_GOOD: {skew_good}")
                center_good = math.fabs(self.x - 80) < 4 and math.fabs(self.y - 60) < 10
                # print(f"*** CENTER_GOOD: {center_good}")
                if area_good and center_good:
                    self.state = 4

            if self.state == 0: # Position
                self._print("Checking position")
                if not center_good: 
                    self.correct_pos()
                    self.x = 0
                    self.y = 0
                    self.state = 0
                else:
                    self.state = 2

            elif self.state == 2: # Height
                self._print("Checking height")
                if not area_good:
                    self.correct_height()
                    self.area = 0
                    self.x = 0
                    self.y = 0
                self.state = 0

            elif self.state == 4: # Grab Pos
                temp_path_time = path_time
                path_time = 3.0
                self.grab_reposition()
                self.state = 5
                path_time = temp_path_time

            elif self.state == 5:  # Close grabber
                temp_path_time = path_time
                path_time = 3.0
                self.grab()
                next_drop_off_point = deepcopy(present_kinematics_pose[0:2])
                self._print(f"Next drop off: {next_drop_off_point}")
                self.state = 6
                path_time = temp_path_time

            elif self.state == 6: # Move back to home
                global height_drop
                global drop_angle
                height_drop = (deepcopy(present_kinematics_pose[2]) * 1.2)
                drop_angle = deepcopy(present_kinematics_pose[3:6])
                goal_joint_angle = home

                temp_path_time = path_time
                path_time = 3.0

                teleop_keyboard.send_goal_joint_space()
                self.state = 7
                path_time = temp_path_time

            elif self.state == 7: # Move to drop-off point 
                temp_path_time = path_time
                path_time = 3.0
                goal_kinematics_pose = deepcopy(present_kinematics_pose)
                goal_kinematics_pose[0:1] = drop_off_point
                goal_kinematics_pose[2] = height_drop
                goal_kinematics_pose[3:6] = drop_angle
                self._print(f"DROP-OFF: {drop_off_point}")
                drop_off_point = next_drop_off_point
                teleop_keyboard.send_goal_task_space()
                self.state = 8
            
            elif self.state == 8:
                goal_joint_angle[4] = 0.010
                teleop_keyboard.send_goal_tool_control()
                goal_joint_angle = deepcopy(home)
                teleop_keyboard.send_goal_joint_space()

                path_time = 0.2
                self.state = 9

            elif self.state == 9:
                self.x = 0
                self.y = 0
                self.area = 0
                self.state = -1

    def listener_callback(self, data:PolygonStamped):        
        self.last_timestamp = datetime.datetime.now()

        points = [
            Vec2D(x = data.polygon.points[0].x, y = data.polygon.points[0].y),
            Vec2D(x = data.polygon.points[1].x, y = data.polygon.points[1].y),
            Vec2D(x = data.polygon.points[2].x, y = data.polygon.points[2].y),
            Vec2D(x = data.polygon.points[3].x, y = data.polygon.points[3].y)
        ]

        # Sort points to match perceived box orientation
        # + -----------------------> x
        # |
        # |     TL               TR
        # |        + --------- +
        # |        |           |
        # |        |           |
        # |        |           |
        # |        + --------- +
        # |      BL              BR
        # \/
        # y

        # Tuple comparison works directly
        # i.e. (1,0) > (0,0) returns True
        points = sorted(points, key=lambda p: (p.x + p.y, p.x - p.y)) # Sort from top-left -> bottom right with lower y values placed first

        bottom_left = points[1]
        top_left = points[0]
        top_right = points[2]
        bottom_right = points[3]

        left : Vec2D = bottom_left - top_left
        # if self.last_left is not None: left = (left + self.last_left) / 2
        top : Vec2D = top_right - top_left
        # if self.last_top is not None: top = (top + self.last_top) / 2
        right : Vec2D = bottom_right - top_right
        # if self.last_right is not None: right = (right + self.last_right) / 2
        bottom : Vec2D = bottom_right - bottom_left
        # if self.last_bottom is not None: bottom = (bottom + self.last_bottom) / 2

        self.area = ((left.rho * top.rho) / 2) + ((right.rho * bottom.rho) / 2)

        top_middle : Vec2D = (top_left + (top / 2)).x
        # print(f"Top middle: {top_middle}")
        bottom_middle : Vec2D = (bottom_left + (bottom / 2)).x
        # print(f"Bottom middle: {bottom_middle}")
        x = top_middle + ((bottom_middle - top_middle) / 2)
        self.x = x
        self.skew = ((left + right) / 2).rho / ((top + bottom) / 2).rho
        # print(f"Skew: {self.skew}")

        left_middle = top_left.y + ((bottom_left.y - top_left.y) / 2)
        right_middle = top_right.y + ((bottom_right.y - top_right.y) / 2)
        y = left_middle + ((right_middle - left_middle) / 2)
        self.y = y

        global g_img        
        matching_image: Image = None
        if g_img is not None:
            if g_img.header.frame_id == data.header.frame_id:
                matching_image = g_img
            elif Path(f"{data.header.frame_id}.png").exists():
                matching_image = PILImage.open(f"{data.header.frame_id}.png")
                # convert_sensorsmsgs_image_to_pil(g_img).save(f"{g_img.header.frame_id}.png")
        
        img: PILImage = None
        if matching_image is not None:
            # Convert ROS Image message to OpenCV image
            img = convert_sensorsmsgs_image_to_pil(matching_image)

            drawer = PILImageDraw.Draw(img)
            drawer.line(((top_left.x, top_left.y),(top_right.x, top_right.y)), fill="red", width=2)
            drawer.line(((top_right.x, top_right.y),(bottom_right.x, bottom_right.y)), fill="blue", width=2)
            drawer.line(((bottom_right.x, bottom_right.y),(bottom_left.x, bottom_left.y)), fill="green", width=2)
            drawer.line(((bottom_left.x, bottom_left.y),(top_left.x, top_left.y)), fill="yellow", width=2)
            drawer.point((top_left.x, top_left.y), fill="red")
            drawer.point((top_right.x, top_right.y), fill="blue")
            drawer.point((bottom_left.x, bottom_left.y), fill="green")
            drawer.point((bottom_right.x, bottom_right.y), fill="yellow")
            img.save(f"{data.header.frame_id}.png")

        self.console.log(f"Received box ({round(self.x,2)}, {round(self.y,2)})")

        xy = Vec2D(x = present_kinematics_pose[0], y = present_kinematics_pose[1])
        self.grabbability = 1.0 - xy.unit().dot(top.unit())
        self._print(self.grabbability)
        # self.last_left = left
        # self.last_top = top
        # self.last_right = right
        # self.last_bottom = bottom

class TeleopKeyboard(Node):#

    qos = QoSProfile(depth=10)
    state : str = ""
    locked = False
    console: ConsolePanel
    status: Status
    interruptable = False
    wait_to_move = True

    def __init__(self, console: ConsolePanel, status: Status):
        super().__init__('teleop_keyboard')
        key_value = ''
        self.console = console
        self.status = status

        self.max_x = 0.168
        self.min_x = -0.127
        self.max_y = 0.26
        self.min_y = 0.1

        # Create joint_states subscriber
        self.joint_state_subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            self.qos)
        self.joint_state_subscription

        # Create kinematics_pose subscriber
        self.kinematics_pose_subscription = self.create_subscription(
            KinematicsPose,
            'kinematics_pose',
            self.kinematics_pose_callback,
            self.qos)
        self.kinematics_pose_subscription

        # Create manipulator state subscriber
        self.open_manipulator_state_subscription = self.create_subscription(
            OpenManipulatorState,
            'states',
            self.open_manipulator_state_callback,
            self.qos)
        self.open_manipulator_state_subscription

        # Create Service Clients
        self.goal_joint_space = self.create_client(SetJointPosition, 'goal_joint_space_path')
        self.goal_task_space = self.create_client(SetKinematicsPose, 'goal_task_space_path')
        self.goal_tool_control = self.create_client(SetJointPosition, 'goal_tool_control')
        self.goal_joint_space_req = SetJointPosition.Request()
        self.goal_task_space_req = SetKinematicsPose.Request()
        self.goal_tool_control_req = SetJointPosition.Request()

    def _print(self, *args, **kwargs):
        self.console.print(*args, **kwargs)

    def lock(self, *args, **kwargs):
        self.locked = True

    def unlock(self, *args, **kwargs):
        self.locked = False

    def send_goal_task_space(self, interruptable=False, time=None):
        global path_time
        self.goal_task_space_req.end_effector_name = 'gripper'
        self.goal_task_space_req.kinematics_pose.pose.position.x = min(max(goal_kinematics_pose[0], self.min_x), self.max_x)
        self.goal_task_space_req.kinematics_pose.pose.position.y = min(max(goal_kinematics_pose[1], self.min_y), self.max_y)
        self.goal_task_space_req.kinematics_pose.pose.position.z = goal_kinematics_pose[2]
        self.goal_task_space_req.kinematics_pose.pose.orientation.w = goal_kinematics_pose[3]
        self.goal_task_space_req.kinematics_pose.pose.orientation.x = goal_kinematics_pose[4]
        self.goal_task_space_req.kinematics_pose.pose.orientation.y = goal_kinematics_pose[5]
        self.goal_task_space_req.kinematics_pose.pose.orientation.z = goal_kinematics_pose[6]
        self.goal_task_space_req.path_time = path_time if time is None else time

        self.interruptable = interruptable

        if not dry_run and not self.locked:
            # self._print("  ---GOALTASK")
            try:
                self.lock()
                self.wait_to_move = True
                send_goal_task = self.goal_task_space.call_async(self.goal_task_space_req)
                send_goal_task.add_done_callback(self.unlock)
            except Exception as e:
                self._print('Sending Goal Kinematic Pose failed %r' % (e,))

    def send_goal_joint_space(self, interruptable=False, time=None):
        self.state = ""
        self.goal_joint_space_req.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4']
        self.goal_joint_space_req.joint_position.position = [goal_joint_angle[0], goal_joint_angle[1], goal_joint_angle[2], goal_joint_angle[3]]
        global path_time
        self.goal_joint_space_req.path_time = path_time if time is None else time

        self.interruptable = interruptable

        if not dry_run and (not self.locked or self.interruptable):
            # self._print("  ---GOALJOINT")
            try:
                self.lock()
                self.wait_to_move = True
                send_goal_joint = self.goal_joint_space.call_async(self.goal_joint_space_req)
                send_goal_joint.add_done_callback(self.unlock)
            except Exception as e:
                self._print('Sending Goal Joint failed %r' % (e,))

    def send_goal_tool_control(self):
        self.goal_tool_control_req.joint_position.joint_name = ["gripper"]
        self.goal_tool_control_req.joint_position.position = [goal_joint_angle[4]]
        self.goal_tool_control_req.path_time = path_time
        try:
            send_goal_tool_control = self.goal_tool_control.call_async(self.goal_tool_control_req)
        except Exception as e:
            self._print('Sending Goal Joint failed %r' % (e,))

    def kinematics_pose_callback(self, msg):
        present_kinematics_pose[0] = msg.pose.position.x
        present_kinematics_pose[1] = msg.pose.position.y
        present_kinematics_pose[2] = msg.pose.position.z
        present_kinematics_pose[3] = msg.pose.orientation.w
        present_kinematics_pose[4] = msg.pose.orientation.x
        present_kinematics_pose[5] = msg.pose.orientation.y
        present_kinematics_pose[6] = msg.pose.orientation.z

    def joint_state_callback(self, msg):
        present_joint_angle[0] = msg.position[0]
        present_joint_angle[1] = msg.position[1]
        present_joint_angle[2] = msg.position[2]
        present_joint_angle[3] = msg.position[3]
        present_joint_angle[4] = msg.position[4]

    def open_manipulator_state_callback(self, msg):
        self.state = msg.open_manipulator_moving_state
        if self.wait_to_move and self.state == "IS_MOVING":
            self.wait_to_move = False
        # print(f"--- {self.state}")
            # for index in range(0, 7):
            #     goal_kinematics_pose[index] = present_kinematics_pose[index]
            # for index in range(0, 4):
            #     goal_joint_angle[index] = present_joint_angle[index]

def main(args=None):

    ui = ConsoleUI()

    def main_status(status: str):
        ui.main_console_status.update(f"Primary status: [b]{status}[/b]")

    with Live(ui.layout, refresh_per_second=2) as live:

        # Auto-locate serial ports
        # - Expected OpenCR1.0 VID:PID is 0x0483:0x5740
        # - Expected PICO VID:PID is 0x0D28:0x0204
        main_status("Searching serial ports...")
        omx_controller_port = None
        cam02_serial_port = None
        for port in list_ports.comports():
            ui.main_console.print(f"Checking {port}")
            if omx_controller_port is None and port.vid == 0x0483 and port.pid == 0x5740:
                omx_controller_port = port.device
                ui.main_console.print(f"Located OpenCR1.0 controller on {omx_controller_port}")

            if cam02_serial_port is None and port.vid == 0x0D28 and port.pid == 0x0204:
                cam02_serial_port = port.device
                ui.main_console.print(f"Located CAM02 serial port on {cam02_serial_port}")

        if omx_controller_port is None:
            main_status("[red]Failed to locate OpenCR1.0[/red]")

        if cam02_serial_port is None:
            main_status("[red]Failed to locate MAX78000CAM02[/red]")

        if omx_controller_port is None or cam02_serial_port is None:
            exit(1)

        main_status("Serial ports located!")

        microROS_Agent = microROS_Agent_Manager(cam02_serial_port, ui.microros_console, ui.microros_status)
        microROS_Agent.start()

        while not microROS_Agent.connected: pass

        omx_controller = OMX_Controller_Manager(omx_controller_port, ui.omx_console, ui.omx_status)
        omx_controller.start()

        

        main_status(f"[yellow]Creating primary node[/yellow]")
        rclpy.init(args=args)
        
        # Create the node    
        global teleop_keyboard
        teleop_keyboard = TeleopKeyboard(ui.main_console, ui.main_console_status)
        
        init_delay = 3
        for i in range(1, init_delay + 1):
            main_status(f"[yellow]Initializing ({i}/{init_delay}s)[/yellow]")
            rclpy.spin_once(teleop_keyboard, timeout_sec=0.1)
            sleep(1)


        main_status("Moving to home")
        global path_time
        temp_path_time = path_time
        path_time = 5.0
        teleop_keyboard.send_goal_tool_control()
        teleop_keyboard.send_goal_joint_space()
        while(teleop_keyboard.state != "IS_MOVING" and omx_controller.serial_thread.is_alive()):
            rclpy.spin_once(teleop_keyboard, timeout_sec=0.1)
        while(teleop_keyboard.state != "STOPPED" and omx_controller.serial_thread.is_alive()):
            rclpy.spin_once(teleop_keyboard, timeout_sec=0.1)

        main_status("Arrived home")
        path_time = temp_path_time

        main_status("Initializing nodes")
        polygon_subscriber = PolygonSubscriber(ui.main_console, ui.main_console_status)
        box_subscriber = BoxSubscriber()
        image_subscriber = ImageSubscriber()

        executor = SingleThreadedExecutor()
        executor.add_node(teleop_keyboard)
        executor.add_node(image_subscriber)
        executor.add_node(polygon_subscriber)

        main_status("[green]Ready[/green]")
        while(rclpy.ok()):
            if not omx_controller.serial_thread.is_alive():
                main_status("[red]Controller connection dropped[/red]")
                break
            executor.spin_once()
            polygon_subscriber.process_state()

        # Spin the node so the callback function is called.
        # while(rclpy.ok()):        
        #     # rclpy.spin_once(box_subscriber)
        #     rclpy.spin_once(teleop_keyboard, timeout_sec=0.01)
        #     rclpy.spin_once(image_subscriber, timeout_sec=0.01)
        #     rclpy.spin_once(polygon_subscriber, timeout_sec=0.01)
        #     polygon_subscriber.process_state()
        
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        box_subscriber.destroy_node()
        
        # Shutdown the ROS client library for Python
        rclpy.shutdown()

        main_status("[red]Shutdown[/red]")

if __name__ == "__main__":
    main()
