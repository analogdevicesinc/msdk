import rclpy
from rclpy.node import Node
from sensor_msgs.msg import RegionOfInterest

from open_manipulator_msgs.msg import KinematicsPose, OpenManipulatorState
from open_manipulator_msgs.srv import SetJointPosition, SetKinematicsPose
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import JointState
# from rclpy.executors import Executor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile
import os
import select
import sys

class BoxSubscriber(Node):
  x: int
  y: int

  def __init__(self):
    # Initiate the Node class's constructor and give it a name
    super().__init__('box_subscriber')
      
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.subscription = self.create_subscription(
      RegionOfInterest,
      '/microROS/box', 
      self.listener_callback, 
      1)
    self.subscription # prevent unused variable warning
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
    self.y = data.y_offset + (data.height / 2)
    self.get_logger().info(f"\tmiddle: {self.x},{self.y}")

    if self.x > 0 and self.y > 0:
        prev = goal_joint_angle[0]
        if (self.x < (160 / 2)):           
           step_size = joint_angle_delta * (1 * ((80 - self.x)/80.0))
           goal_joint_angle[0] = prev - step_size
           print(f"{prev}-{step_size} -> {goal_joint_angle[0]}")
           teleop_keyboard.send_goal_joint_space()
        elif (self.x > (160 / 2)):
           step_size = joint_angle_delta * (1 * ((self.x - 80)/80.0))
           goal_joint_angle[0] = goal_joint_angle[0] + step_size
           print(f"{prev}+{step_size} -> {goal_joint_angle[0]}")
           teleop_keyboard.send_goal_joint_space()
        print(goal_joint_angle[0])
        for index in range(0, 7):
            prev_goal_kinematics_pose[index] = goal_kinematics_pose[index]
        for index in range(0, 4):
            prev_goal_joint_angle[index] = goal_joint_angle[index]
        

present_joint_angle = [1.514991, -0.754719, 0.391165, 1.184233]
goal_joint_angle = [1.514991, -0.754719, 0.391165, 1.184233]
prev_goal_joint_angle = [0.0, 0.0, 0.0, 0.0]
present_kinematics_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
goal_kinematics_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
prev_goal_kinematics_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

debug = True
task_position_delta = 0.01  # meter
joint_angle_delta = 0.4  # radian
path_time = 0.5  # second

class TeleopKeyboard(Node):

    qos = QoSProfile(depth=10)

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
        self.goal_joint_space_req = SetJointPosition.Request()
        self.goal_task_space_req = SetKinematicsPose.Request()

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

        try:
            send_goal_task = self.goal_task_space.call_async(self.goal_task_space_req)
        except Exception as e:
            self.get_logger().info('Sending Goal Kinematic Pose failed %r' % (e,))

    def send_goal_joint_space(self):
        self.goal_joint_space_req.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4']
        self.goal_joint_space_req.joint_position.position = [goal_joint_angle[0], goal_joint_angle[1], goal_joint_angle[2], goal_joint_angle[3]]
        self.goal_joint_space_req.path_time = path_time

        try:
            send_goal_joint = self.goal_joint_space.call_async(self.goal_joint_space_req)
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

    def open_manipulator_state_callback(self, msg):
        if msg.open_manipulator_moving_state == 'STOPPED': pass
            # for index in range(0, 7):
            #     goal_kinematics_pose[index] = present_kinematics_pose[index]
            # for index in range(0, 4):
            #     goal_joint_angle[index] = present_joint_angle[index]

def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  box_subscriber = BoxSubscriber()
  global teleop_keyboard
  teleop_keyboard = TeleopKeyboard()  
  
  # Spin the node so the callback function is called.
  while(rclpy.ok()):
    rclpy.spin_once(box_subscriber)
    rclpy.spin_once(teleop_keyboard)
    #    teleop_keyboard.send_goal_joint_space()
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  box_subscriber.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()

if __name__ == "__main__":
  main()