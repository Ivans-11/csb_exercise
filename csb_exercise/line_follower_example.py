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

        # Control parameters
        self.base_speed = 0.2
        self.Kp = 0.002
        self.gray_threshold = 100
        self.width_threshold = 140

    def image_callback(self, image_msg):
        """
        Callback function to receive and process image data
        :param image_msg: Image message
        """
        # Convert ROS Image message to OpenCV image
        img = bridge.imgmsg_to_cv2(image_msg,desired_encoding='bgr8')

        # Create Twist message object
        twist = Twist()
        twist.linear.x = self.base_speed
        twist.angular.z = 0.0

        # Convert to grayscale and apply binary threshold
        img_h, img_w = img.shape[:2]
        roi = img[img_h-80:img_h, :]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(gray, self.gray_threshold, 255, cv2.THRESH_BINARY_INV)

        # Calculate moments of binary image
        M = cv2.moments(mask)
        if M['m00'] != 0:
            cx = int(M['m10'] / M['m00'])
            error = cx - img_w // 2
            print('error:', error)

            # Set angular velocity proportional to the error
            twist.angular.z = self.Kp * error
            cv2.circle(img, (cx, img_h - 40), 10, (0, 0, 255), -1)
            
        
        # Detect T-junction(simple method)
        h = mask.shape[0]
        sample_rows = [int(h*0.6), int(h*0.7), int(h*0.8), int(h*0.9)]
        line_widths = [cv2.countNonZero(mask[row, :]) for row in sample_rows]
        line_width = max(line_widths)
        if line_width > self.width_threshold:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

            self.get_logger().info('T line detected')
            self.pub.publish(twist)

            # Exit
            self.destroy_node()
            rclpy.shutdown()
            return

        # Publish velocity command
        self.pub.publish(twist)

        cv2.imshow("detect_line", img)
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


