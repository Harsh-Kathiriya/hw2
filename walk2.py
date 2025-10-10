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
        self.FORWARD_SPEED = 0.37
        self.ROTATION_SPEED = 0.6
        self.FRONT_THRESHOLD = 0.7
        self.SIDE_THRESHOLD = 0.5

        # --- Robot Command Variables ---
        self.target_linear_velocity = 0.0
        self.target_angular_velocity = 0.0

        self.get_logger().info("SmartWalker (v7 - Safe Random Turn on Obstacle) started.")

    def sensor_callback(self, msg):
        """
        Processes LIDAR data and determines the robot's next move.
        When facing an obstacle, it checks both sides and chooses
        a safe (possibly random) direction to turn.
        """
        # Divide the LIDAR scan
        right_view = msg.ranges[10:60]
        front_view = msg.ranges[60:121]
        left_view = msg.ranges[121:171]

        # Get minimum distances
        min_dist_right = min([r for r in right_view if not math.isinf(r)] or [100])
        min_dist_front = min([r for r in front_view if not math.isinf(r)] or [100])
        min_dist_left = min([r for r in left_view if not math.isinf(r)] or [100])

        # --- Decision Logic ---
        if min_dist_front < self.FRONT_THRESHOLD:
            # Obstacle directly ahead
            self.get_logger().info(
                f'FRONT BLOCKED ({min_dist_front:.2f}m). Deciding turn direction safely...'
            )

            self.target_linear_velocity = 0.0

            # Check which sides are safe enough to turn
            left_safe = min_dist_left > self.SIDE_THRESHOLD
            right_safe = min_dist_right > self.SIDE_THRESHOLD

            if left_safe and right_safe:
                # Both sides open — choose randomly
                turn_dir = random.choice([-1, 1])
                self.get_logger().info(
                    f'Both sides safe. Randomly turning {"left" if turn_dir == 1 else "right"}.'
                )
                self.target_angular_velocity = self.ROTATION_SPEED * turn_dir

            elif left_safe:
                # Only left side safe
                self.get_logger().info(f'Only LEFT side safe ({min_dist_left:.2f}m). Turning left.')
                self.target_angular_velocity = self.ROTATION_SPEED

            elif right_safe:
                # Only right side safe
                self.get_logger().info(f'Only RIGHT side safe ({min_dist_right:.2f}m). Turning right.')
                self.target_angular_velocity = -self.ROTATION_SPEED

            else:
                # Both sides blocked — turn toward side with slightly more space
                if min_dist_left > min_dist_right:
                    self.get_logger().info('Both sides tight. Turning left slowly.')
                    self.target_angular_velocity = self.ROTATION_SPEED * 0.4
                else:
                    self.get_logger().info('Both sides tight. Turning right slowly.')
                    self.target_angular_velocity = -self.ROTATION_SPEED * 0.4

        elif min_dist_left < self.SIDE_THRESHOLD:
            # Obstacle on left — steer away
            self.get_logger().info(f'SIDE-LEFT BLOCKED ({min_dist_left:.2f}m). Steering right.')
            self.target_linear_velocity = self.FORWARD_SPEED * 0.4
            self.target_angular_velocity = -self.ROTATION_SPEED * 0.7

        elif min_dist_right < self.SIDE_THRESHOLD:
            # Obstacle on right — steer away
            self.get_logger().info(f'SIDE-RIGHT BLOCKED ({min_dist_right:.2f}m). Steering left.')
            self.target_linear_velocity = self.FORWARD_SPEED * 0.4
            self.target_angular_velocity = self.ROTATION_SPEED * 0.7

        else:
            # Path clear — go straight
            self.get_logger().info('Path is clear. Moving forward.')
            self.target_linear_velocity = self.FORWARD_SPEED
            self.target_angular_velocity = 0.0

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
