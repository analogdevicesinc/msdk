from PIL import ImageDraw
from PIL.Image import fromarray, Image
from pathlib import Path

# Import the necessary libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, RegionOfInterest
from cv_bridge import CvBridge # Converts between ROS and OpenCV image formats
import cv2

g_img: Image
valid_image = False

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1)
      
        self.subscription = self.create_subscription(
            Image,
            '/microROS/image',
            self.listener_callback, 
            qos_profile = qos_profile)
          
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()
   
    def listener_callback(self, data):
        self.get_logger().info('Received image')
    
        # Convert ROS Image message to OpenCV image
        img = self.br.imgmsg_to_cv2(data)

        # Convert to PIL image so we can draw to it.
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        global g_img
        g_img = fromarray(img)
        global valid_image
        valid_image = True

class ROISubscriber(Node):
    x: int
    y: int

    def __init__(self):
        super().__init__('roi_subscriber')

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
        self.get_logger().info('Received bounding box:')
        self.get_logger().info(f"\tx_offset: {data.x_offset}")
        self.get_logger().info(f"\ty offset: {data.y_offset}")
        self.get_logger().info(f"\twidth: {data.width}")
        self.get_logger().info(f"\theight: {data.height}")
        self.x = data.x_offset + (data.width / 2)
        self.y = data.y_offset + (data.height / 2)
        self.get_logger().info(f"\tmiddle: {self.x},{self.y}")

        if (data.width > 0 and data.height > 0):
            global valid_image
            global g_img
            if valid_image:
                drawer = ImageDraw.Draw(g_img)
                drawer.rectangle((data.x_offset, data.y_offset, data.x_offset + data.width, data.y_offset + data.height), outline="red", width=3)
                drawer.point((self.x, self.y), fill="red")
                g_img.save("./test.png")
                self.get_logger().info("Saved image to test.png")
                valid_image = False
  
def main(args=None):
  
    # Initialize the rclpy library
    rclpy.init(args=args)
    
    # Create the node
    image_subscriber = ImageSubscriber()
    image_subscriber.get_logger().info("Initialized image subscriber.")

    box_subcriber = ROISubscriber()
    box_subcriber.get_logger().info("Initialized box subscriber,")
    
    # Spin the node so the callback function is called.
    while(rclpy.ok()):
      rclpy.spin_once(box_subcriber, timeout_sec=0.1)
      rclpy.spin_once(image_subscriber, timeout_sec=0.1)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_subscriber.destroy_node()
    
    # Shutdown the ROS client library for Python
    rclpy.shutdown()
  
if __name__ == '__main__':
    main()