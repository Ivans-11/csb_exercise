import rclpy
from sensor_msgs.msg import LaserScan # LaserScan message
from geometry_msgs.msg import Twist # Twist message
from rclpy.node import Node

class AvoidNode(Node):
    """
    Obstacle avoidance node
    """
    def __init__(self, name):
        super().__init__(name)

        # Create subscriber to subscribe to laser scan data
        self.sub = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback, # Callback function
            10
        )

        # Create publisher to publish velocity commands
        self.pub = self.create_publisher(
            Twist,
            'cmd_vel',
            1
        )

    def laser_callback(self, scan_msg):
        """
        Callback function to receive and process laser scan data
        :param scan_msg: LaserScan message
        """
        # Create Twist message object
        twist = Twist()
        
        # TODO: Implement obstacle avoidance logic
        # This is just a simple example, can be modified according to actual needs
        front_distance = self.get_range_by_angle(scan_msg, 0.0)
        if front_distance < 0.5:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        else:
            twist.linear.x = 0.2
            twist.angular.z = 0.0

        # Publish velocity command
        self.pub.publish(twist)

        self.get_logger().info(f"Front: {front_distance:.2f}")
        self.get_logger().info(f"Published: linear.x={twist.linear.x}, angular.z={twist.angular.z}")
    

    def get_range_by_angle(self, scan_msg, angle):
        """
        Get the corresponding distance value by angle
        :param scan_msg: LaserScan message
        :param angle: Angle in radians
        :return: Corresponding distance value
        """
        # Convert angle to index
        index = int((angle - scan_msg.angle_min) / scan_msg.angle_increment)

        # Check if index is out of bounds
        if 0 <= index < len(scan_msg.ranges):
            # Return the corresponding distance value
            return scan_msg.ranges[index]
        else:
            # Return the minimum distance value
            return scan_msg.range_min

def main(args=None):
    # Initialize ROS2 interface
    rclpy.init(args=args)

    # Create obstacle avoidance node object
    node = AvoidNode("avoid_node")

    # Loop and wait for callback function execution
    rclpy.spin(node)

    # Destroy node and shutdown ROS2 interface
    node.destroy_node()
    rclpy.shutdown()


