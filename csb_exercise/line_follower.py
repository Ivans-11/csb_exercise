import rclpy
from sensor_msgs.msg import Image # Image message
from geometry_msgs.msg import Twist # Twist message
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

import numpy as np
import cv2
import cv_bridge

# Create a bridge between ROS and OpenCV
bridge = cv_bridge.CvBridge()

class FollowerNode(Node):
    """
    Line follower node
    """
    def __init__(self, name):
        super().__init__(name)

        # Define QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # Create subscriber to subscribe to image data
        self.sub = self.create_subscription(
            Image,
            'image',
            self.image_callback, # Callback function
            qos_profile
        )

        # Create publisher to publish velocity commands
        self.pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

    def image_callback(self, image_msg):
        """
        Callback function to receive and process image data
        :param image_msg: Image message
        """
        # Convert ROS Image message to OpenCV image
        image = bridge.imgmsg_to_cv2(image_msg,desired_encoding='bgr8')

        # Create Twist message object
        twist = Twist()

        # TODO: Implement line following logic

        # Publish velocity command
        self.pub.publish(twist)

        cv2.imshow("detect_line", image)
        cv2.waitKey(3)

def main(args=None):
    # Initialize ROS2 interface
    rclpy.init(args=args)

    # Create line follower node object
    node = FollowerNode("follower_node")

    # Loop and wait for callback function execution
    rclpy.spin(node)

    # Destroy node and shutdown ROS2 interface
    node.destroy_node()
    rclpy.shutdown()


