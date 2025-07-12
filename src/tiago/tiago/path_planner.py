#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import os
import random
import math
import traceback  # Import the traceback module

# ROS standard message types
from nav_msgs.srv import GetPlan
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs

# This is the ROS 2 function to find installed package files
from ament_index_python.packages import get_package_share_directory

# This is the absolute import for your A* library
from tiago.lib.path.astar_path_planner import AStarPathPlanner
from tiago.srv import PathPlannerCommand


class PathPlannerService(Node):
    """
    ROS 2 Node that provides path planning services using an improved A* algorithm.
    The A* planner itself is modified to prioritize paths away from obstacles.
    """

    def __init__(self):
        super().__init__('path_planner_service')
        self.get_logger().info(f"Node '{self.get_name()}' has been started.")

        # --- Parameters ---
        # The 'safety_weight' is now the primary control for path safety.
        # It's passed to the A* algorithm to penalize paths close to obstacles.
        # Higher values create safer, more central paths.
        self.declare_parameter('safety_weight', 5.0)
        self.declare_parameter('smoothing_points', 40)
        self.declare_parameter('map_filename', 'my_map.yaml')

        # Publishers
        self.path_publisher = self.create_publisher(Path, 'planned_path', 10)
        self.status_publisher = self.create_publisher(String, 'path_planner_status', 10)

        # Service clients
        self.controller_stop_client = self.create_client(Trigger, '/controller/stop_trigger')

        # TF2 for robot position tracking
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.robot_frame = 'base_link'
        self.map_frame = 'map'

        # Load map and initialize planner
        try:
            package_share_dir = get_package_share_directory('tiago')
            map_filename = self.get_parameter('map_filename').get_parameter_value().string_value
            map_file_path = os.path.join(package_share_dir, 'maps', map_filename)

            self.get_logger().info(f"Loading map from: {map_file_path}")
            if not os.path.exists(map_file_path):
                raise FileNotFoundError(f"Map file not found at: {map_file_path}")

            self.planner = AStarPathPlanner(map_file_path)
            self.get_logger().info("A* planner initialized successfully.")

        except Exception as e:
            self.get_logger().error(f"Failed to initialize A* planner: {e}")
            raise e

        # Create services
        self.plan_service = self.create_service(
            GetPlan, 'plan_path', self.plan_path_callback)

        self.command_service = self.create_service(
            PathPlannerCommand, 'path_planner_command', self.handle_path_command)

        self.get_logger().info("Services '/plan_path' and 'path_planner_command' are ready.")

    def get_robot_position(self):
        """Get current robot position in map frame."""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, rclpy.time.Time())
            return [transform.transform.translation.x, transform.transform.translation.y]
        except Exception as e:
            self.get_logger().warn(f"Could not get robot position: {e}")
            return None

    def _plan_and_smooth_path(self, start_coords, goal_coords):
        """
        Helper function to run the A* planner and then smooth the result.
        This centralizes the logic and uses the new safety-aware planner.
        """
        safety_weight = self.get_parameter('safety_weight').get_parameter_value().double_value
        num_points = self.get_parameter('smoothing_points').get_parameter_value().integer_value

        self.get_logger().info(
            f"Planning path from {start_coords} to {goal_coords} with safety_weight={safety_weight}...")

        # Call the planner, which now internally handles obstacle avoidance
        raw_path = self.planner.find_path(start_coords, goal_coords, safety_weight=safety_weight)

        if not raw_path:
            self.get_logger().warn("A* planner failed to find a path.")
            return None

        # Smoothing is now mostly for aesthetics, so collision_threshold can be small.
        smoothed_path = self.planner.smooth_path(raw_path, num_points=num_points, collision_threshold=0.1)

        # Return the smoothed path if available, otherwise fall back to the raw (but safe) A* path
        return smoothed_path if smoothed_path else raw_path

    def generate_random_walk_path(self):
        """Generates a path to a single randomly selected waypoint using the safety-aware A* algorithm."""
        predefined_waypoints = [
            [0, -4.0, 0.0],    # Near entrance
            [0, 3.5, 0.0],    # Opposite to the entrance
            [-5.5, 0, 0.0],    # Left aisle
            [5.5, 0, 0.0],   # Right aisle
            [0, 0, 0.0],
        ]

        current_pos = self.get_robot_position()

        if current_pos is None:
            self.get_logger().warn("Could not get current robot position for random walk.")
            return None

        reachable_waypoints = [wp for wp in predefined_waypoints if self.planner._is_valid_cell(wp[0],wp[1])]

        if not reachable_waypoints:
            self.get_logger().warn("No valid waypoints for random walk.")
            return self.create_path_message([])

        random_target = random.choice(reachable_waypoints)

        final_path_points = self._plan_and_smooth_path(current_pos, random_target)

        if not final_path_points:
            self.get_logger().warn(f"Could not generate random walk path to {random_target}.")
            return self.create_path_message([])

        self.get_logger().info(f"Generated A* random walk path with {len(final_path_points)} points.")
        return self.create_path_message(final_path_points)

    def create_path_message(self, path_points):
        """Convert list of [x, y] points to nav_msgs/Path message."""
        ros_path = Path()
        ros_path.header.stamp = self.get_clock().now().to_msg()
        ros_path.header.frame_id = self.map_frame

        for point in path_points:
            pose = PoseStamped()
            pose.header = ros_path.header
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.orientation.w = 1.0
            ros_path.poses.append(pose)

        return ros_path

    def handle_path_command(self, request, response):
        """Handle PathPlannerCommand service requests."""
        command = request.command
        self.get_logger().info(f"Received path command: {command}")

        if command == "random_walk":
            try:
                random_path = self.generate_random_walk_path()
                if not random_path:
                    self.get_logger().warn("Failed to generate random walk path.")
                    response.success = False
                    return response
                if not random_path.poses:
                    raise ValueError("Failed to generate a valid random walk path.")

                self.path_publisher.publish(random_path)
                self.publish_status("started_random_walk")
                response.success = True
            except Exception as e:
                # Modified: Use traceback.format_exc() to include full traceback
                error_detail = traceback.format_exc()
                self.get_logger().error(f"Failed to generate random walk: {e}\n{error_detail}")
                response.success = False

        elif command == "go_to_location":
            try:
                current_pos = self.get_robot_position()
                if current_pos is None:
                    self.get_logger().warn("Could not get current robot position for go_to_location.")
                    response.success = False
                    return response
                target_pos = [request.location.x, request.location.y]

                final_path_points = self._plan_and_smooth_path(current_pos, target_pos)

                if final_path_points:
                    ros_path = self.create_path_message(final_path_points)
                    self.path_publisher.publish(ros_path)
                    self.publish_status("started_navigation")
                    response.success = True
                else:
                    response.success = False
            except Exception as e:
                # Modified: Use traceback.format_exc() to include full traceback
                error_detail = traceback.format_exc()
                self.get_logger().error(f"Failed to plan path to location: {e}\n{error_detail}")
                response.success = False

        elif command == "stop_movement":
            try:
                if self.controller_stop_client.service_is_ready():
                    self.controller_stop_client.call_async(Trigger.Request())
                    self.publish_status("stopped")
                    response.success = True
                else:
                    self.get_logger().warn("Controller stop service not available")
                    response.success = False
            except Exception as e:
                self.get_logger().error(f"Failed to stop movement: {e}")
                response.success = False

        else:
            response.success = False

        return response

    def publish_status(self, status):
        """Publish status message."""
        msg = String()
        msg.data = status
        self.status_publisher.publish(msg)
        self.get_logger().info(f"Published status: {status}")

    def plan_path_callback(self, request: GetPlan.Request, response: GetPlan.Response):
        """Callback for the GetPlan service."""
        start_coords = [request.start.pose.position.x, request.start.pose.position.y]
        goal_coords = [request.goal.pose.position.x, request.goal.pose.position.y]

        final_path_points = self._plan_and_smooth_path(start_coords, goal_coords)

        if final_path_points:
            response.plan = self.create_path_message(final_path_points)
            self.get_logger().info("Successfully planned and returned path.")
            self.path_publisher.publish(response.plan)
        else:
            self.get_logger().warn("Path planning failed. Returning empty path.")
            response.plan = Path()
            response.plan.header.stamp = self.get_clock().now().to_msg()
            response.plan.header.frame_id = self.map_frame

        return response


def main(args=None):
    rclpy.init(args=args)
    path_planner_service = None
    try:
        path_planner_service = PathPlannerService()
        rclpy.spin(path_planner_service)
    except Exception as e:
        if path_planner_service:
            # ORIGINAL LINE: path_planner_service.get_logger().fatal(f"A critical error occurred: {e}", exc_info=True)
            # FIX: Format the exception info into the message string
            error_message = f"A critical error occurred: {e}\n{traceback.format_exc()}"
            path_planner_service.get_logger().fatal(error_message)
        else:
            print(f"Critical error during node initialization: {e}")
            # Also add traceback for initial errors if path_planner_service is not yet initialized
            print(traceback.format_exc())
    finally:
        if path_planner_service:
            path_planner_service.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()