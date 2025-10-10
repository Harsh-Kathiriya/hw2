import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
import random
import time

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
        self.WANDER_PROBABILITY = 0.1  # ← Adjust this for more/less random turns

        # --- Robot State Variables ---
        self.target_linear_velocity = 0.0
        self.target_angular_velocity = 0.0

        # Random wandering state
        self.random_turn_active = False
        self.random_turn_end_time = 0.0

        self.get_logger().info("SmartWalker (v5 - Tunable Random Wandering) started.")

    def sensor_callback(self, msg):
        """
        Process LIDAR data and choose action based on nearby obstacles.
        Adds gentle random direction changes when safe.
        """
        # Split LIDAR scan
        right_view = msg.ranges[10:60]
        front_view = msg.ranges[60:121]
        left_view = msg.ranges[121:171]

        # Get min distances
        min_dist_right = min([r for r in right_view if not math.isinf(r)] or [100])
        min_dist_front = min([r for r in front_view if not math.isinf(r)] or [100])
        min_dist_left = min([r for r in left_view if not math.isinf(r)] or [100])

        now = time.time()

        # --- Obstacle Avoidance Logic ---
        if min_dist_front < self.FRONT_THRESHOLD:
            self.get_logger().info(f'FRONT BLOCKED ({min_dist_front:.2f}m). Turning to avoid.')
            self.random_turn_active = False
            self.target_linear_velocity = 0.0
            if min_dist_left > min_dist_right:
                self.target_angular_velocity = self.ROTATION_SPEED
            else:
                self.target_angular_velocity = -self.ROTATION_SPEED

        elif min_dist_left < self.SIDE_THRESHOLD:
            self.get_logger().info(f'SIDE-LEFT BLOCKED ({min_dist_left:.2f}m). Steering right.')
            self.random_turn_active = False
            self.target_linear_velocity = self.FORWARD_SPEED * 0.5
            self.target_angular_velocity = -self.ROTATION_SPEED * 0.7

        elif min_dist_right < self.SIDE_THRESHOLD:
            self.get_logger().info(f'SIDE-RIGHT BLOCKED ({min_dist_right:.2f}m). Steering left.')
            self.random_turn_active = False
            self.target_linear_velocity = self.FORWARD_SPEED * 0.5
            self.target_angular_velocity = self.ROTATION_SPEED * 0.7

        # --- Wandering Behavior: Path is clear ---
        else:
            if not self.random_turn_active and random.random() < self.WANDER_PROBABILITY:
                # Begin a gentle random turn
                turn_dir = random.choice([-1, 1])
                duration = random.uniform(1.0, 2.5)
                self.random_turn_active = True
                self.random_turn_end_time = now + duration
                self.target_angular_velocity = self.ROTATION_SPEED * 0.3 * turn_dir
                self.get_logger().info(
                    f'Path clear. Starting gentle random turn ({turn_dir}, {duration:.1f}s).'
                )

            elif self.random_turn_active and now < self.random_turn_end_time:
                # Continue the random turn
                pass

            else:
                # End wandering and go straight
                if self.random_turn_active:
                    self.get_logger().info('Ending random turn. Moving straight.')
                self.random_turn_active = False
                self.target_angular_velocity = 0.0

            self.target_linear_velocity = self.FORWARD_SPEED

        # Publish immediately (no timer)
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
