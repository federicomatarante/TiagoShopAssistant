#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import os
import random
import math

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
from tiago.srv import PathPlannerCommand  # You need to create this service definition

class PathPlannerService(Node):
    """
    ROS 2 Node that provides path planning services using the A* algorithm.
    Handles both GetPlan service calls and PathPlannerCommand service calls.
    """
    def __init__(self):
        super().__init__('path_planner_service')
        self.get_logger().info(f"Node '{self.get_name()}' has been started.")

        # Declare parameters
        self.declare_parameter('w_heuristic', 100.0)
        self.declare_parameter('smoothing_points', 40)
        self.declare_parameter('smoothing_collision_threshold', 0.3)
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
            return [0.0, 0.0, 0.0]  # Default position

    def generate_random_walk_path(self):
        """
        Generates a path to a single randomly selected waypoint using the A* algorithm.
        """
        # Define predefined waypoints for random walk (adjust based on your map)
        predefined_waypoints = [
            [0, -4.0, 0.0],    # Near entrance
            [0, 3.5, 0.0],    # Opposite to the entrance
            [-5.5, 0, 0.0],    # Left aisle
            [5.5, 0, 0.0],   # Right aisle
            [0, 0, 0.0],    # Center
        ]
        
        current_pos = self.get_robot_position()
        
        # Filter waypoints to find ones that are reachable from the current position
        reachable_waypoints = []
        for waypoint in predefined_waypoints:
            if self.planner.is_valid_point(waypoint):
                # Check if a path exists from the current position to the waypoint
                test_path = self.planner.find_path(current_pos, waypoint, w=1.0)
                if test_path:
                    reachable_waypoints.append(waypoint)
        
        if not reachable_waypoints:
            self.get_logger().warn("No reachable waypoints found for random walk from current position.")
            return self.create_path_message([]) # Return an empty path

        # Select one random target waypoint from the reachable ones
        random_target = random.choice(reachable_waypoints)
        self.get_logger().info(f"Generating A* random walk path to: {random_target}")

        # Use the A* planner to generate a full path to the target
        w = self.get_parameter('w_heuristic').get_parameter_value().double_value
        num_points = self.get_parameter('smoothing_points').get_parameter_value().integer_value
        threshold = self.get_parameter('smoothing_collision_threshold').get_parameter_value().double_value

        raw_path = self.planner.find_path(current_pos, random_target, w=w)

        if not raw_path:
            self.get_logger().warn(f"A* could not find a path to the random waypoint {random_target}, though it was deemed reachable.")
            return self.create_path_message([]) # Return empty path

        smoothed_path = self.planner.smooth_path(raw_path, num_points=num_points, collision_threshold=threshold)
        final_path_points = smoothed_path if smoothed_path else raw_path
        
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
                if not random_path.poses:
                    raise ValueError("Failed to generate a valid random walk path.")
                
                self.path_publisher.publish(random_path)
                self.publish_status("started_random_walk")
                response.success = True
                response.message = "Random walk path generated and published"
            except Exception as e:
                self.get_logger().error(f"Failed to generate random walk: {e}")
                response.success = False
                response.message = f"Failed to generate random walk: {e}"
        
        elif command == "go_to_location":
            try:
                current_pos = self.get_robot_position()
                target_pos = [request.location.x, request.location.y]
                
                # Use A* to plan path to target location
                w = self.get_parameter('w_heuristic').get_parameter_value().double_value
                raw_path = self.planner.find_path(current_pos, target_pos, w=w)
                
                if raw_path:
                    # Smooth the path
                    num_points = self.get_parameter('smoothing_points').get_parameter_value().integer_value
                    threshold = self.get_parameter('smoothing_collision_threshold').get_parameter_value().double_value
                    smoothed_path = self.planner.smooth_path(raw_path, num_points=num_points, collision_threshold=threshold)
                    final_path = smoothed_path if smoothed_path else raw_path
                    
                    ros_path = self.create_path_message(final_path)
                    self.path_publisher.publish(ros_path)
                    self.publish_status("started_navigation")
                    response.success = True
                    response.message = f"Path to location [{target_pos[0]:.2f}, {target_pos[1]:.2f}] generated"
                else:
                    response.success = False
                    response.message = "No path found to target location"
            except Exception as e:
                self.get_logger().error(f"Failed to plan path to location: {e}")
                response.success = False
                response.message = f"Failed to plan path: {e}"
        
        elif command == "stop_movement":
            try:
                # Call controller stop service
                if self.controller_stop_client.service_is_ready():
                    stop_request = Trigger.Request()
                    future = self.controller_stop_client.call_async(stop_request)
                    # Note: Not waiting for response in this simple implementation
                    self.publish_status("stopped")
                    response.success = True
                    response.message = "Stop command sent to controller"
                else:
                    self.get_logger().warn("Controller stop service not available")
                    response.success = False
                    response.message = "Controller stop service not available"
            except Exception as e:
                self.get_logger().error(f"Failed to stop movement: {e}")
                response.success = False
                response.message = f"Failed to stop movement: {e}"
        
        else:
            response.success = False
            response.message = f"Unknown command: {command}"
        
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

        self.get_logger().info(f"Received path request from {start_coords} to {goal_coords}")

        # Get planner settings from ROS parameters
        w = self.get_parameter('w_heuristic').get_parameter_value().double_value
        num_points = self.get_parameter('smoothing_points').get_parameter_value().integer_value
        threshold = self.get_parameter('smoothing_collision_threshold').get_parameter_value().double_value

        # Find and smooth path
        raw_path = self.planner.find_path(start_coords, goal_coords, w=w)

        if not raw_path:
            self.get_logger().warn("No path found by A* algorithm.")
            response.plan = Path()
            response.plan.header.stamp = self.get_clock().now().to_msg()
            response.plan.header.frame_id = self.map_frame
            return response

        smoothed_path = self.planner.smooth_path(raw_path, num_points=num_points, collision_threshold=threshold)
        final_path_points = smoothed_path if smoothed_path else raw_path
        
        response.plan = self.create_path_message(final_path_points)
        self.get_logger().info("Successfully planned and returned path.")
        
        # Also publish the path for the controller
        self.path_publisher.publish(response.plan)
        
        return response

def main(args=None):
    rclpy.init(args=args)
    path_planner_service = None
    try:
        path_planner_service = PathPlannerService()
        rclpy.spin(path_planner_service)
    except Exception as e:
        if path_planner_service:
            path_planner_service.get_logger().fatal(f"A critical error occurred: {e}")
        else:
            print(f"Critical error during node initialization: {e}")
    finally:
        if path_planner_service:
            path_planner_service.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()