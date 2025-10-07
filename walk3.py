# walk.py (v9 - Simplified with Constant Speed)
#
# DESCRIPTION:
# This version removes the adaptive speed control for simplicity.
# It uses a single, constant forward speed.
# It retains the corrected logic for:
#  - Turning if the distance to the start point decreases.
#  - A top-priority safety override to avoid crashing into obstacles.

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math

class SimplifiedWalker(Node):
    def __init__(self):
        super().__init__('simplified_walker')
        
        # Publishers and Subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/base_scan', self.sensor_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/ground_truth', self.odometry_callback, 10)
        self.timer = self.create_timer(0.1, self.run_logic_loop)

        # --- [MODIFIED] Simplified Constants ---
        self.TARGET_DISTANCE = 19.0      # Set for each starting position
        self.FORWARD_SPEED = 0.5         # A single, constant forward speed. Tune this value!
        self.ROTATION_SPEED = 0.8
        self.FRONT_THRESHOLD = 0.7       # Closest distance before a safety turn is triggered
        self.DISTANCE_DECREASE_TOLERANCE = 0.005 # How much distance must decrease to trigger a turn

        # State Variables
        self.target_linear_velocity = 0.0
        self.target_angular_velocity = 0.0
        self.start_pos = None
        self.current_distance = 0.0
        self.previous_distance = 0.0
        self.goal_reached = False
        self.latest_scan = None
        
        self.get_logger().info(f"SimplifiedWalker started. Target: {self.TARGET_DISTANCE}m")

    def odometry_callback(self, msg):
        if self.start_pos is None:
            self.start_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.current_distance = math.sqrt((msg.pose.pose.position.x - self.start_pos[0])**2 + (msg.pose.pose.position.y - self.start_pos[1])**2)
        if self.current_distance >= self.TARGET_DISTANCE and not self.goal_reached:
            self.goal_reached = True
            self.get_logger().info(f"SUCCESS: Goal of {self.TARGET_DISTANCE}m reached! Stopping.")

    def sensor_callback(self, msg):
        self.latest_scan = msg

    def run_logic_loop(self):
        if self.latest_scan is None or self.start_pos is None:
            return

        if self.goal_reached:
            self.target_linear_velocity = 0.0
            self.target_angular_velocity = 0.0
            self.publish_command()
            return
            
        min_dist_front = min([r for r in self.latest_scan.ranges[60:121] if not math.isinf(r)] or [100])
        min_dist_left = min([r for r in self.latest_scan.ranges[121:171] if not math.isinf(r)] or [100])
        min_dist_right = min([r for r in self.latest_scan.ranges[10:60] if not math.isinf(r)] or [100])

        # --- [MODIFIED] Simplified Logic Structure ---

        # 1. Default Intention: Go forward at a constant speed.
        self.target_linear_velocity = self.FORWARD_SPEED
        self.target_angular_velocity = 0.0
        self.get_logger().info(f"Moving forward. Current distance: {self.current_distance:.2f}m")

        # 2. Goal-Oriented Override: Check if we are losing progress.
        if self.current_distance < self.previous_distance - self.DISTANCE_DECREASE_TOLERANCE:
            self.get_logger().warn(f"Distance decreased! Forcing a turn.")
            self.target_linear_velocity = 0.0
            self.target_angular_velocity = self.ROTATION_SPEED # Force a turn left

        # 3. Final Safety Override: Highest priority is to not crash.
        if min_dist_front < self.FRONT_THRESHOLD:
            self.get_logger().info(f"SAFETY OVERRIDE: Obstacle at {min_dist_front:.2f}m. Must turn.")
            self.target_linear_velocity = 0.0
            if min_dist_left > min_dist_right:
                self.target_angular_velocity = self.ROTATION_SPEED # Turn Left
            else:
                self.target_angular_velocity = -self.ROTATION_SPEED # Turn Right
        
        # --- End of Logic ---

        self.publish_command()
        self.previous_distance = self.current_distance

    def publish_command(self):
        twist_msg = Twist()
        twist_msg.linear.x = self.target_linear_velocity
        twist_msg.angular.z = self.target_angular_velocity
        self.cmd_pub.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    walker_node = SimplifiedWalker()
    rclpy.spin(walker_node)
    walker_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
