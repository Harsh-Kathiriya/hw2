import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
import random

class SmartWalker(Node):
    def __init__(self):
        super().__init__('smart_walker')
        
        # Publisher to send velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber to LIDAR sensor data
        self.subscription = self.create_subscription(
            LaserScan,
            '/base_scan',
            self.sensor_callback,
            10)

        # --- Constants for Tuning ---
        self.FORWARD_SPEED = 0.25
        self.ROTATION_SPEED = 0.6
        self.FRONT_THRESHOLD = 0.6
        self.SIDE_THRESHOLD = 0.4

        # --- Robot Command Variables ---
        self.target_linear_velocity = 0.0
        self.target_angular_velocity = 0.0

        self.get_logger().info("SmartWalker (v3 - Random Turn Safety) started.")

    def sensor_callback(self, msg):
        """
        Process LIDAR data and choose action based on nearby obstacles.
        Adds controlled randomness when turning to avoid local traps.
        """
        # Split the LIDAR scan
        right_view = msg.ranges[10:60]
        front_view = msg.ranges[60:121]
        left_view = msg.ranges[121:171]

        # Get min distances
        min_dist_right = min([r for r in right_view if not math.isinf(r)] or [100])
        min_dist_front = min([r for r in front_view if not math.isinf(r)] or [100])
        min_dist_left = min([r for r in left_view if not math.isinf(r)] or [100])

        # --- Decision Logic ---
        if min_dist_front < self.FRONT_THRESHOLD:
            # Robot needs to turn — add randomness
            self.get_logger().info(f'FRONT BLOCKED ({min_dist_front:.2f}m). Choosing turn...')
            
            # Pick turn direction based on space, but with some randomness
            if min_dist_left > min_dist_right:
                base_turn = 1  # left
            elif min_dist_right > min_dist_left:
                base_turn = -1  # right
            else:
                base_turn = random.choice([-1, 1])  # random if equal

            # Randomly invert turn 20% of the time to escape loops
            if random.random() < 0.2:
                base_turn *= -1
                self.get_logger().info('Random direction flip for exploration.')

            # Randomize rotation speed slightly (but safe range)
            rot_speed = self.ROTATION_SPEED * random.uniform(0.8, 1.2)

            # Assign command
            self.target_linear_velocity = 0.0
            self.target_angular_velocity = rot_speed * base_turn

        elif min_dist_left < self.SIDE_THRESHOLD:
            self.get_logger().info(f'SIDE-LEFT BLOCKED ({min_dist_left:.2f}m). Steering right.')
            self.target_linear_velocity = self.FORWARD_SPEED * 0.5
            self.target_angular_velocity = -self.ROTATION_SPEED * 0.7

        elif min_dist_right < self.SIDE_THRESHOLD:
            self.get_logger().info(f'SIDE-RIGHT BLOCKED ({min_dist_right:.2f}m). Steering left.')
            self.target_linear_velocity = self.FORWARD_SPEED * 0.5
            self.target_angular_velocity = self.ROTATION_SPEED * 0.7

        else:
            self.get_logger().info('Path clear. Moving forward.')
            self.target_linear_velocity = self.FORWARD_SPEED
            self.target_angular_velocity = 0.0

        # Publish command immediately (no timer)
        self.publish_command()

    def publish_command(self):
        twist_msg = Twist()
        twist_msg.linear.x = self.target_linear_velocity
        twist_msg.angular.z = self.target_angular_velocity
        self.cmd_pub.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SmartWalker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
