#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import math
import time
from geometry_msgs.msg import PoseStamped, Point, Quaternion as RosQuaternion, Twist # MODIFIED LINE

# Message types
from std_srvs.srv import Trigger
from nav_msgs.msg import Path
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Point, Quaternion as RosQuaternion
from action_msgs.msg import GoalStatus


def _yaw_to_quaternion(yaw_radians):
    """Helper function to convert yaw angle (in radians) to ROS Quaternion."""
    q = RosQuaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw_radians / 2.0)
    q.w = math.cos(yaw_radians / 2.0)
    return q


def _quaternion_to_yaw(quaternion):
    """Helper function to convert ROS Quaternion to yaw angle (in radians)."""
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y ** 2 + quaternion.z ** 2)
    )


def _compute_yaw_towards(target: PoseStamped, previous: PoseStamped = None):
    """Compute yaw pointing from previous pose to target. If no previous, point along +X."""
    if previous:
        dx = target.pose.position.x - previous.pose.position.x
        dy = target.pose.position.y - previous.pose.position.y
    else:
        dx, dy = 1.0, 0.0
    return math.atan2(dy, dx)


class SimpleNavigationController(Node):
    """
    A simple ROS 2 node that receives a path and navigates to each waypoint 
    using Nav2's NavigateToPose action, with improved error handling and recovery.
    """

    def __init__(self):
        super().__init__('simple_navigation_controller')
        self.get_logger().info("Simple Navigation Controller node has been started.")
        time.sleep(30.0)  # Wait for Gazebo to start up

        # --- Parameters ---
        self.declare_parameter('goal_tolerance', 0.1)
        self.declare_parameter('navigation_timeout', 120.0)  # seconds
        self.declare_parameter('goal_send_timeout', 5.0)  # seconds
        self.declare_parameter('max_retry_attempts', 3)  # max retries per waypoint
        self.declare_parameter('retry_delay', 2.0)  # seconds between retries

        # --- State Variables ---
        self.current_path = []
        self.target_waypoint_index = 0
        self.is_navigating = False
        self.current_goal_handle = None
        self.retry_count = 0
        self.max_retries = self.get_parameter('max_retry_attempts').value
        self.retry_delay = self.get_parameter('retry_delay').value
        self.goal_start_time = None

        # --- Nav2 Action Client ---
        self._nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info("NavigateToPose action client created.")
        # Give Gazebo time to start up (20+ seconds)
        if not self._check_nav_server_ready(timeout_sec=30.0):
            self.get_logger().warn("Nav2 server not ready at startup. Will wait for it when needed.")

        # --- Subscribers ---
        self.path_subscriber = self.create_subscription(
            Path,
            '/planned_path',
            self.path_callback,
            10)

        # Publishers
        
        self.path_planner_publisher = self.create_publisher(
            String, 'path_planner_status', 10)
        self.cmd_vel_publisher = self.create_publisher(
            Twist, 'cmd_vel', 10) # ADDED LINE

        # --- Service server for path commands ---
        self.srv = self.create_service(Trigger, '/controller/stop_trigger', self.stop_trigger_callback)

        # --- Single timer for managing navigation flow ---
        self.navigation_timer = self.create_timer(0.5, self.navigation_state_machine)

        # --- State machine states ---
        self.state = 'IDLE'  # IDLE, NAVIGATING, RETRY_WAIT, MOVING_TO_NEXT

    def stop_trigger_callback(self, request, response):
        """Service callback to stop the navigation."""
        self.get_logger().info("Stop trigger received.")
        
        # Always publish a zero velocity command to ensure immediate physical stop
        stop_cmd = Twist()
        self.cmd_vel_publisher.publish(stop_cmd)
        self.get_logger().info("Published zero velocity command to cmd_vel for immediate stop.")

        if self.is_navigating and self.current_goal_handle:
            self.get_logger().info("Active navigation detected, canceling goal.")
            self.current_goal_handle.cancel_goal_async()
            self.is_navigating = False
            self.current_goal_handle = None
            self.get_logger().info("Navigation goal cancellation requested.")
        else:
            self.get_logger().info("No active navigation goal to cancel (or goal not yet accepted).")

        # Regardless of active navigation, reset controller state
        self.state = 'IDLE'
        self.current_path = []  # Clear the path to prevent continuation
        self.target_waypoint_index = 0  # Reset waypoint index
        self.retry_count = 0  # Reset retry count
        self.get_logger().info("Controller state reset to IDLE.")

        response.success = True
        response.message = "Navigation stop command processed."
        
        # Always publish a "stopped" status
        self.path_planner_publisher.publish(String(data="stopped"))
        return response

    def _check_nav_server_ready(self, timeout_sec=5.0):
        """Checks if the Nav2 action server is ready, waits if necessary."""
        if not self._nav_action_client.server_is_ready():
            self.get_logger().warn("NavigateToPose action server not available. Waiting...")
            if not self._nav_action_client.wait_for_server(timeout_sec=timeout_sec):
                self.get_logger().error("NavigateToPose action server not available after timeout.")
                return False
            self.get_logger().info("NavigateToPose action server is ready.")
        return True

    def path_callback(self, msg: Path):
        """Receives a new path and prepares to start navigation to the first waypoint."""
        self.get_logger().info(f"New path with {len(msg.poses)} waypoints received.")

        # Always cancel any ongoing navigation first if a new path arrives
        if self.is_navigating and self.current_goal_handle:
            self.get_logger().info("Canceling current navigation goal due to new path.")
            self.current_goal_handle.cancel_goal_async()
            # It's crucial to immediately publish a stop command here too,
            # in case the robot is already moving from a previous goal
            stop_cmd = Twist()
            self.cmd_vel_publisher.publish(stop_cmd)
            self.get_logger().info("Published zero velocity command to cmd_vel (new path received).")

        # Reset state for the new path
        self.is_navigating = False
        self.current_goal_handle = None
        self.retry_count = 0

        if msg.poses:
            self.current_path = msg.poses
            self.target_waypoint_index = 0
            # FIX: Set state to MOVING_TO_NEXT to trigger navigation
            self.state = 'MOVING_TO_NEXT'
            self.get_logger().info("Path received, controller ready to start navigation.")
        else:
            self.current_path = []
            self.state = 'IDLE'
            self.get_logger().info("Empty path received, stopping navigation.")

    def navigation_state_machine(self):
        """State machine to handle navigation flow."""
        if self.state == 'IDLE':
            return

        if self.state == 'MOVING_TO_NEXT':
            if not self.current_path or self.target_waypoint_index >= len(self.current_path):
                self.get_logger().info("All waypoints completed or no valid waypoints.")
                self.state = 'IDLE'
                self.current_path = []
                return

            if self.start_navigation_to_current_waypoint():
                self.state = 'NAVIGATING'
                self.goal_start_time = time.time()
            else:
                self.handle_navigation_failure()

        elif self.state == 'NAVIGATING':
            timeout = self.get_parameter('navigation_timeout').value
            if self.goal_start_time and (time.time() - self.goal_start_time) > timeout:
                self.get_logger().warn("Navigation timeout reached, canceling goal.")
                if self.current_goal_handle:
                    self.current_goal_handle.cancel_goal_async()
                self.handle_navigation_failure()
        # RETRY_WAIT is timer-driven; no action here

    def start_navigation_to_current_waypoint(self):
        """Start navigation to the current target waypoint."""
        if not self.current_path or self.target_waypoint_index >= len(self.current_path):
            return False

        if not self._check_nav_server_ready(timeout_sec=10.0):
            self.get_logger().warn("Navigation server not ready, will retry later.")
            return False

        target_waypoint = self.current_path[self.target_waypoint_index]

        # Compute orientation to face toward next waypoint
        if self.target_waypoint_index > 0:
            prev_waypoint = self.current_path[self.target_waypoint_index - 1]
        else:
            prev_waypoint = None
        yaw = _compute_yaw_towards(target_waypoint, prev_waypoint)
        orientation = _yaw_to_quaternion(yaw)

        x = target_waypoint.pose.position.x
        y = target_waypoint.pose.position.y
        yaw_deg = math.degrees(yaw)
        retry_info = f" (retry {self.retry_count + 1}/{self.max_retries})" if self.retry_count > 0 else ""
        self.get_logger().info(
            f"Navigating to waypoint {self.target_waypoint_index + 1}/{len(self.current_path)}: "
            f"x={x:.2f}, y={y:.2f}, theta={yaw_deg:.2f}°{retry_info}"
        )

        goal_msg = NavigateToPose.Goal()
        goal_pose = PoseStamped()
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = target_waypoint.header.frame_id
        goal_pose.pose.position = Point(x=float(x), y=float(y), z=0.0)
        goal_pose.pose.orientation = orientation
        goal_msg.pose = goal_pose

        try:
            self.is_navigating = True
            send_goal_future = self._nav_action_client.send_goal_async(goal_msg)
            send_goal_future.add_done_callback(self.goal_response_callback)
            return True
        except Exception as e:
            self.get_logger().error(f"Error sending navigation goal: {e}")
            self.is_navigating = False
            return False

    def goal_response_callback(self, future):
        """Callback for when the goal is accepted or rejected."""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error('Navigation goal was rejected by the server.')
                self.is_navigating = False
                self.handle_navigation_failure()
                return

            self.get_logger().info('Navigation goal accepted by server.')
            self.current_goal_handle = goal_handle
            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(self.goal_result_callback)
        except Exception as e:
            self.get_logger().error(f"Error in goal response callback: {e}")
            self.is_navigating = False
            self.handle_navigation_failure()

    def goal_result_callback(self, future):
        """Callback for when navigation goal is completed."""
        try:
            result = future.result()
            status = result.status
            self.is_navigating = False
            self.current_goal_handle = None

            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(f'Navigation to waypoint {self.target_waypoint_index + 1} succeeded!')
                self.target_waypoint_index += 1
                self.retry_count = 0
                if self.target_waypoint_index < len(self.current_path):
                    self.get_logger().info("Moving to next waypoint...")
                    self.state = 'MOVING_TO_NEXT'
                else:
                    self.get_logger().info("All waypoints reached! Path navigation completed.")
                    # Publish "finished" message
                    msg = String()
                    msg.data = "finished"
                    self.path_planner_publisher.publish(msg)
                    self.get_logger().info("Published 'finished' status to path_planner_status.")
                    self.current_path = []
                    self.state = 'IDLE'
            elif status == GoalStatus.STATUS_CANCELED:
                self.get_logger().info('Navigation goal was canceled.')
                self.state = 'IDLE'
            elif status == GoalStatus.STATUS_ABORTED:
                self.get_logger().error('Navigation goal was aborted.')
                self.handle_navigation_failure()
            else:
                self.get_logger().error(f'Navigation goal failed with status: {status}')
                self.handle_navigation_failure()
        except Exception as e:
            self.get_logger().error(f"Error in goal result callback: {e}")
            self.is_navigating = False
            self.handle_navigation_failure()

    def handle_navigation_failure(self):
        """Handle navigation failure with retry logic."""
        if self.retry_count < self.max_retries:
            self.retry_count += 1
            self.get_logger().warn(
                f"Navigation failed, retrying in {self.retry_delay} seconds... (attempt {self.retry_count}/{self.max_retries})"
            )
            self.create_timer(self.retry_delay, self.retry_navigation, clock=self.get_clock())
            self.state = 'RETRY_WAIT'
        else:
            self.get_logger().error(
                f"Max retry attempts ({self.max_retries}) reached for waypoint {self.target_waypoint_index + 1}. Skipping to next waypoint."
            )
            self.target_waypoint_index += 1
            self.retry_count = 0
            if self.target_waypoint_index < len(self.current_path):
                self.state = 'MOVING_TO_NEXT'
            else:
                self.get_logger().info("No more waypoints, navigation completed with errors.")
                self.state = 'IDLE'
                self.current_path = []

    def retry_navigation(self):
        """Called after retry delay to attempt navigation again."""
        if self.state == 'RETRY_WAIT':
            self.state = 'MOVING_TO_NEXT'


def main(args=None):
    rclpy.init(args=args)
    controller_node = SimpleNavigationController()
    try:
        rclpy.spin(controller_node)
    except KeyboardInterrupt:
        controller_node.get_logger().info("KeyboardInterrupt received, shutting down.")
    finally:
        if controller_node.is_navigating and controller_node.current_goal_handle:
            controller_node.get_logger().info("Canceling navigation goal during shutdown.")
            controller_node.current_goal_handle.cancel_goal_async()
        controller_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
