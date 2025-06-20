#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import numpy as np

# Message types
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, Pose

def euler_from_quaternion(quaternion):
    """
    Converts a quaternion into Euler angles (roll, pitch, yaw).
    yaw is the rotation around the z-axis.
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z

class Controller(Node):
    """
    A ROS 2 node to control the robot's actions. For now, it follows a given path.
    """
    def __init__(self):
        super().__init__('controller')
        self.get_logger().info("Controller node has been started.")

        # --- Parameters ---
        self.declare_parameter('kp_linear', 1.5)      # Proportional gain for linear velocity
        self.declare_parameter('kp_angular', 2.0)     # Proportional gain for angular velocity
        self.declare_parameter('goal_tolerance', 0.1) # Distance in meters to consider a waypoint reached
        self.declare_parameter('max_linear_speed', 2.0) # Maximum linear speed (m/s)
        self.declare_parameter('max_angular_speed', 1.5) # Maximum angular speed (rad/s)

        # --- State Variables ---
        self.current_path = []
        self.target_waypoint_index = 0
        self.current_pose = None
        
        # --- Subscribers ---
        self.path_subscriber = self.create_subscription(
            Path,
            '/planned_path',
            self.path_callback,
            10)
        
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/mobile_base_controller/odom',
            self.odom_callback,
            10)
            
        # --- Publisher ---
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- Control Loop Timer ---
        self.control_timer = self.create_timer(0.1, self.control_loop) # 10 Hz loop

    def path_callback(self, msg: Path):
        """Receives a new path and resets the path-following logic."""
        self.get_logger().info(f"New path with {len(msg.poses)} waypoints received.")
        if len(msg.poses) > 0:
            self.current_path = msg.poses
            self.target_waypoint_index = 0
        else:
            self.current_path = []
            self.stop_robot()

    def odom_callback(self, msg: Odometry):
        """Updates the robot's current position and orientation."""
        self.current_pose = msg.pose.pose

    def control_loop(self):
        """The main logic for the controller (currently, path following)."""
        # --- Guard Clauses: Check if we are ready to follow a path ---
        if self.current_pose is None:
            self.get_logger().warn("Waiting for odometry data...", throttle_duration_sec=5)
            return

        if not self.current_path:
            return # No active path, so we do nothing.
            
        if self.target_waypoint_index >= len(self.current_path):
            self.get_logger().info("End of path reached.")
            self.stop_robot()
            self.current_path = [] # Clear the path so we don't restart
            return

        # --- Get the current target waypoint ---
        target_waypoint = self.current_path[self.target_waypoint_index]
        target_pos = target_waypoint.pose.position
        current_pos = self.current_pose.position
        
        # --- Calculate distance to the target waypoint ---
        dx = target_pos.x - current_pos.x
        dy = target_pos.y - current_pos.y
        distance_to_target = math.sqrt(dx**2 + dy**2)
        
        # --- Check if we have reached the waypoint ---
        goal_tolerance = self.get_parameter('goal_tolerance').get_parameter_value().double_value
        if distance_to_target < goal_tolerance:
            self.get_logger().info(f"Waypoint {self.target_waypoint_index} reached.")
            self.target_waypoint_index += 1
            return

        # --- Calculate required heading (angle) to the target ---
        angle_to_target = math.atan2(dy, dx)
        
        # --- Get the robot's current heading (yaw) ---
        _, _, current_yaw = euler_from_quaternion(self.current_pose.orientation)
        
        # --- Calculate the error in heading ---
        angle_error = angle_to_target - current_yaw
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error)) # Normalize

        # --- Calculate control commands using a Proportional (P) controller ---
        kp_linear = self.get_parameter('kp_linear').get_parameter_value().double_value
        kp_angular = self.get_parameter('kp_angular').get_parameter_value().double_value
        max_linear = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        max_angular = self.get_parameter('max_angular_speed').get_parameter_value().double_value

        angular_vel = kp_angular * angle_error
        linear_vel = kp_linear * distance_to_target if abs(angle_error) < math.pi / 4 else 0.0

        # --- Clamp velocities to their maximum values ---
        linear_vel = np.clip(linear_vel, -max_linear, max_linear)
        angular_vel = np.clip(angular_vel, -max_angular, max_angular)
        
        # --- Publish the velocity command ---
        twist_msg = Twist()
        twist_msg.linear.x = linear_vel
        twist_msg.angular.z = angular_vel
        self.velocity_publisher.publish(twist_msg)

    def stop_robot(self):
        """Publishes a zero-velocity command to stop the robot."""
        self.get_logger().info("Publishing stop command.")
        stop_msg = Twist()
        self.velocity_publisher.publish(stop_msg)

def main(args=None):
    rclpy.init(args=args)
    controller_node = Controller()
    try:
        rclpy.spin(controller_node)
    except KeyboardInterrupt:
        pass
    finally:
        controller_node.stop_robot()
        controller_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()