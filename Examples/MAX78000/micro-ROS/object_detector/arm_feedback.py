import rclpy
from rclpy.node import Node
from sensor_msgs.msg import RegionOfInterest
from copy import deepcopy

from vector import VectorObject2D as Vec2D

from time import sleep

from PIL import ImageDraw
from PIL import Image

from open_manipulator_msgs.msg import KinematicsPose, OpenManipulatorState
from open_manipulator_msgs.srv import SetJointPosition, SetKinematicsPose
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PolygonStamped, Point32
# from rclpy.executors import Executor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import os
import select
import sys
import math

global g_stopped
g_stopped = False

home = [1.514991, -0.754719, 0.391165, 1.184233, -0.010]

present_joint_angle = deepcopy(home)
goal_joint_angle = deepcopy(home)
prev_goal_joint_angle = [0.0, 0.0, 0.0, 0.0]
present_kinematics_pose = [0.009, 0.147, 0.162, 0.666, -0.221, 0.217, 0.679]
goal_kinematics_pose = [0.009, 0.147, 0.162, 0.666, -0.221, 0.217, 0.679]
prev_goal_kinematics_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
height_drop = 0
drop_angle = []

debug = True
task_position_delta = 0.01 / 3  # meter
joint_angle_delta = 0.2 / 2  # radian
global path_time
path_time = 0.2 # second
dry_run = False

class BoxSubscriber(Node):
  x: int
  y: int

  def __init__(self):
    # Initiate the Node class's constructor and give it a name
    super().__init__('box_subscriber')
      
    qos_profile = QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
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
           goal_joint_angle[3] -= (step_size / 4)
        elif (self.y > (120 / 2)):
           step_size = (joint_angle_delta / 2) * (1 * ((self.y - 60)/60.0))
           goal_joint_angle[1] += step_size
           goal_joint_angle[3] += (step_size / 4)

        teleop_keyboard.send_goal_joint_space()

        self.x = 0
        self.y = 0

        # for index in range(0, 7):
        #     prev_goal_kinematics_pose[index] = goal_kinematics_pose[index]
        # for index in range(0, 4):
        #     prev_goal_joint_angle[index] = goal_joint_angle[index]
        
class PolygonSubscriber(Node):
    x: int
    y: int

    state = 0
    img = None

    last_left : Vec2D = None
    last_top : Vec2D = None
    last_right : Vec2D = None
    last_bottom : Vec2D = None

    def __init__(self):
        super().__init__('polygon_subscriber')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1)

        self.subscription = self.create_subscription(
          PolygonStamped,
          '/microROS/polygon', 
          self.listener_callback,
          qos_profile = qos_profile)

        self.x = 0
        self.y = 0

    def correct_height(self):
        print("---HEIGHT")
        global goal_kinematics_pose
        global present_kinematics_pose
        goal_kinematics_pose = deepcopy(present_kinematics_pose)
        print(f"--- AREA: {self.area}")
        if self.area > 3000:
            goal_kinematics_pose[2] += task_position_delta
        
        if goal_kinematics_pose[2] < 0.1:
            print("*** FLOOR") 
            goal_kinematics_pose[2] = 0.1

        teleop_keyboard.send_goal_task_space()
        while(teleop_keyboard.state != "IS_MOVING"):
            rclpy.spin_once(teleop_keyboard, timeout_sec=0.01)
    
    def correct_skew(self):
        print("---SKEW")
        global goal_kinematics_pose
        global present_kinematics_pose
        goal_kinematics_pose = deepcopy(present_kinematics_pose)
        current_xy = Vec2D(x = goal_kinematics_pose[0], y = goal_kinematics_pose[1])
        delta_xy : Vec2D = current_xy.unit() * task_position_delta
        if (self.skew < 0.9):
            step_size = task_position_delta * (1 * ((0.9 - self.skew)/0.9))
            goal_kinematics_pose[1] += delta_xy.y
            goal_kinematics_pose[2] += (step_size * 1.1)
        elif (self.skew > 1.1):
            step_size = task_position_delta * (1 * ((self.skew - 1.1)/1.1))
            goal_kinematics_pose[1] -= delta_xy.y
            goal_kinematics_pose[2] -= (step_size * 1.1)

        teleop_keyboard.send_goal_task_space()
        while(teleop_keyboard.state != "IS_MOVING"):
            rclpy.spin_once(teleop_keyboard, timeout_sec=0.01)
       
    def correct_pos(self):
        print("---POS")
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
            # goal_joint_angle[1] -= step_size
            goal_joint_angle[3] -= (step_size)
        elif (self.y > (120 / 2)):
            step_size = ((joint_angle_delta / 2) * (1 * ((self.y - 60)/60.0))) * 2
            # goal_joint_angle[1] += step_size
            goal_joint_angle[3] += (step_size)

        teleop_keyboard.send_goal_joint_space()
        while(teleop_keyboard.state != "IS_MOVING"):
            rclpy.spin_once(teleop_keyboard, timeout_sec=0.01)

    def grab_reposition(self):
        print("--- GRAB REPOS")
        global goal_kinematics_pose
        global present_kinematics_pose        
        xy = Vec2D(x = present_kinematics_pose[0], y = present_kinematics_pose[1])
        correction = xy.unit() * (task_position_delta * -10)
        goal_kinematics_pose[0] += correction.x
        goal_kinematics_pose[1] += correction.y
        goal_kinematics_pose[2] -= task_position_delta * 15

        teleop_keyboard.send_goal_task_space()
        while(teleop_keyboard.state != "IS_MOVING"):
            rclpy.spin_once(teleop_keyboard, timeout_sec=0.01)
        while(teleop_keyboard.state != "STOPPED"):
            rclpy.spin_once(teleop_keyboard, timeout_sec=0.01)

        print("--- DONE")

    def grab(self):
        print("--- GRAB")
        goal_joint_angle[4] = -0.005729
        teleop_keyboard.send_goal_tool_control()

    def process_state(self):
        global path_time
        if self.x > 0 and self.y > 0 and teleop_keyboard.state != "IS_MOVING" and not teleop_keyboard.locked:
            if self.state == 0: # Position
                self.correct_pos()
                self.x = 0
                self.y = 0
                self.state = 1
            
            elif self.state == 1: # Skew
                self.correct_skew()
                self.state = 2
            
            elif self.state == 2: # Height
                self.correct_height()
                if self.area < 6700 and math.fabs(self.skew - 1.0) > 0.1:
                    self.state = 0
                else:
                    self.state = 4

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
                self.state = 6
                path_time = temp_path_time

            elif self.state == 6:
                global goal_joint_angle
                global height_drop
                global drop_angle
                height_drop = (deepcopy(present_kinematics_pose[2]) * 1.3)
                drop_angle = deepcopy(present_kinematics_pose[3:6])
                goal_joint_angle = home

                temp_path_time = path_time
                path_time = 3.0

                teleop_keyboard.send_goal_joint_space()
                while(teleop_keyboard.state != "IS_MOVING"):
                    rclpy.spin_once(teleop_keyboard, timeout_sec=0.01)
                while(teleop_keyboard.state != "STOPPED"):
                    rclpy.spin_once(teleop_keyboard, timeout_sec=0.01)
                self.state = 7
                path_time = temp_path_time

            elif self.state == 7:
                global goal_kinematics_pose
                temp_path_time = path_time
                path_time = 5.0
                goal_kinematics_pose = deepcopy(present_kinematics_pose)
                goal_kinematics_pose[0:2] = [-0.11, 0.2, height_drop]
                goal_kinematics_pose[3:6] = drop_angle
                teleop_keyboard.send_goal_task_space()
                while(teleop_keyboard.state != "IS_MOVING"):
                    rclpy.spin_once(teleop_keyboard, timeout_sec=0.01)
                while(teleop_keyboard.state != "STOPPED"):
                    rclpy.spin_once(teleop_keyboard, timeout_sec=0.01)

                goal_joint_angle[4] = -0.010
                teleop_keyboard.send_goal_tool_control()
                goal_joint_angle = deepcopy(home)
                teleop_keyboard.send_goal_joint_space()
                while(teleop_keyboard.state != "IS_MOVING"):
                    rclpy.spin_once(teleop_keyboard, timeout_sec=0.01)
                while(teleop_keyboard.state != "STOPPED"):
                    rclpy.spin_once(teleop_keyboard, timeout_sec=0.01)

                path_time = 0.2
                self.state = 0


    def listener_callback(self, data:PolygonStamped):
        # self.get_logger().info(f'Received polygon box:')

        points = [
            Vec2D(x = data.polygon.points[0].x, y = data.polygon.points[0].y),
            Vec2D(x = data.polygon.points[1].x, y = data.polygon.points[1].y),
            Vec2D(x = data.polygon.points[2].x, y = data.polygon.points[2].y),
            Vec2D(x = data.polygon.points[3].x, y = data.polygon.points[3].y)
        ]

        for i in range(len(points) - 1):
            j = i
            while((points[j].x > points[j + 1].x or points[j].y > points[j + 1].y) and j < len(points) - 2):
                tmp = points[j + 1]
                points[j + 1] = points[j]
                points[j] = tmp
                j += 1

        bottom_left = points[1]
        top_left = points[0]
        top_right = points[2]
        bottom_right = points[3]
        # print(f"BL: ({bottom_left.x},{bottom_left.y})")
        # print(f"TL: ({top_left.x},{top_left.y})")
        # print(f"TR: ({top_right.x},{top_right.y})")
        # print(f"BR: ({bottom_right.x},{bottom_right.y})")

        left : Vec2D = bottom_left - top_left
        if self.last_left is not None: left = (left + self.last_left) / 2
        top : Vec2D = top_right - top_left
        if self.last_top is not None: top = (top + self.last_top) / 2
        right : Vec2D = bottom_right - top_right
        if self.last_right is not None: right = (right + self.last_right) / 2
        bottom : Vec2D = bottom_right - bottom_left
        if self.last_bottom is not None: bottom = (bottom + self.last_bottom) / 2

        self.area = ((left.rho * top.rho) / 2) + ((right.rho * bottom.rho) / 2)
        print(f"*** AREA: {self.area}")

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

        self.img = Image.new("RGB", (160,120))
        drawer = ImageDraw.Draw(self.img)
        drawer.point((top_left.x, top_left.y), fill="red")
        drawer.point((top_right.x, top_right.y), fill="blue")
        drawer.point((bottom_left.x, bottom_left.y), fill="green")
        drawer.point((bottom_right.x, bottom_right.y), fill="yellow")
        self.img.save("test.png")

        self.last_left = left
        self.last_top = top
        self.last_right = right
        self.last_bottom = bottom

class TeleopKeyboard(Node):

    qos = QoSProfile(depth=10)
    state : str = ""
    locked = False

    def __init__(self):
        super().__init__('teleop_keyboard')
        key_value = ''

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

    def lock(self, *args, **kwargs):
        print("---> LOCK")
        self.locked = True

    def unlock(self, *args, **kwargs):
        print("<--- UNLOCK")
        self.locked = False

    def send_goal_task_space(self):
        self.goal_task_space_req.end_effector_name = 'gripper'
        self.goal_task_space_req.kinematics_pose.pose.position.x = goal_kinematics_pose[0]
        self.goal_task_space_req.kinematics_pose.pose.position.y = goal_kinematics_pose[1]
        self.goal_task_space_req.kinematics_pose.pose.position.z = goal_kinematics_pose[2]
        self.goal_task_space_req.kinematics_pose.pose.orientation.w = goal_kinematics_pose[3]
        self.goal_task_space_req.kinematics_pose.pose.orientation.x = goal_kinematics_pose[4]
        self.goal_task_space_req.kinematics_pose.pose.orientation.y = goal_kinematics_pose[5]
        self.goal_task_space_req.kinematics_pose.pose.orientation.z = goal_kinematics_pose[6]
        self.goal_task_space_req.path_time = path_time

        if not dry_run and not self.locked:
            print("  ---GOALTASK")
            try:
                self.lock()
                send_goal_task = self.goal_task_space.call_async(self.goal_task_space_req)
                send_goal_task.add_done_callback(self.unlock)
            except Exception as e:
                self.get_logger().info('Sending Goal Kinematic Pose failed %r' % (e,))

    def send_goal_joint_space(self):
        self.state = ""
        self.goal_joint_space_req.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4']
        self.goal_joint_space_req.joint_position.position = [goal_joint_angle[0], goal_joint_angle[1], goal_joint_angle[2], goal_joint_angle[3]]
        self.goal_joint_space_req.path_time = path_time

        if not dry_run and not self.locked:
            print("  ---GOALJOINT")
            try:
                self.lock()
                send_goal_joint = self.goal_joint_space.call_async(self.goal_joint_space_req)
                send_goal_joint.add_done_callback(self.unlock)
            except Exception as e:
                self.get_logger().info('Sending Goal Joint failed %r' % (e,))

    def send_goal_tool_control(self):
        self.goal_tool_control_req.joint_position.joint_name = ["gripper"]
        self.goal_tool_control_req.joint_position.position = [goal_joint_angle[4]]
        self.goal_tool_control_req.path_time = path_time
        try:
            send_goal_tool_control = self.goal_tool_control.call_async(self.goal_tool_control_req)
        except Exception as e:
            self.get_logger().info('Sending Goal Joint failed %r' % (e,))

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
        # print(f"--- {self.state}")
            # for index in range(0, 7):
            #     goal_kinematics_pose[index] = present_kinematics_pose[index]
            # for index in range(0, 4):
            #     goal_joint_angle[index] = present_joint_angle[index]

def main(args=None):

    # Initialize the rclpy library
    rclpy.init(args=args)
    
    # Create the node    
    global teleop_keyboard
    teleop_keyboard = TeleopKeyboard()
    
    global path_time
    temp_path_time = path_time
    path_time = 5.0
    teleop_keyboard.send_goal_tool_control()
    teleop_keyboard.send_goal_joint_space()
    while(teleop_keyboard.state != "IS_MOVING"):
        rclpy.spin_once(teleop_keyboard, timeout_sec=0.1)
    while(teleop_keyboard.state != "STOPPED"):
        rclpy.spin_once(teleop_keyboard, timeout_sec=0.1)    

    path_time = temp_path_time

    polygon_subscriber = PolygonSubscriber()
    box_subscriber = BoxSubscriber()

    # Spin the node so the callback function is called.
    while(rclpy.ok()):        
        # rclpy.spin_once(box_subscriber)
        rclpy.spin_once(teleop_keyboard, timeout_sec=0.01)
        rclpy.spin_once(polygon_subscriber, timeout_sec=0.01)
        polygon_subscriber.process_state()
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    box_subscriber.destroy_node()
    
    # Shutdown the ROS client library for Python
    rclpy.shutdown()

if __name__ == "__main__":
  main()