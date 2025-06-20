#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import os

# ROS standard message types
from nav_msgs.srv import GetPlan
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

# This is the ROS 2 function to find installed package files
from ament_index_python.packages import get_package_share_directory

# This is the absolute import for your A* library
from tiago.lib.path.astar_path_planner import AStarPathPlanner

class PathPlannerService(Node):
    """
    ROS 2 Node that provides a path planning service using the A* algorithm.
    """
    def __init__(self):
        """
        Initializes the node, loads the map, and creates the service server.
        """
        super().__init__('path_planner_service')
        self.get_logger().info(f"Node '{self.get_name()}' has been started.")

        # Declare parameters for planner configuration
        self.declare_parameter('w_heuristic', 100.0)
        self.declare_parameter('smoothing_points', 40)
        self.declare_parameter('smoothing_collision_threshold', 0.15)
        self.declare_parameter('map_filename', 'my_map.yaml')

        self.path_publisher = self.create_publisher(Path, 'planned_path', 10)

        # Find and load the map file from the package share directory
        try:
            package_share_dir = get_package_share_directory('tiago')
            map_filename = self.get_parameter('map_filename').get_parameter_value().string_value
            
            # Construct the full path to the map file inside the 'share/tiago/maps' directory
            map_file_path = os.path.join(package_share_dir, 'maps', map_filename)
            
            self.get_logger().info(f"Attempting to load map from: {map_file_path}")
            if not os.path.exists(map_file_path):
                 raise FileNotFoundError(f"Map file not found at the installed path. Check your CMakeLists.txt install rule.")

            self.planner = AStarPathPlanner(map_file_path)
            self.get_logger().info("A* planner initialized successfully.")

        except Exception as e:
            self.get_logger().error(f"Failed to initialize A* planner: {e}")
            raise e

        # Create a service that uses the standard nav_msgs/GetPlan message type
        self.plan_service = self.create_service(
            GetPlan,
            'plan_path',
            self.plan_path_callback
        )
        self.get_logger().info("Service '/plan_path' is ready.")

    def plan_path_callback(self, request: GetPlan.Request, response: GetPlan.Response) -> GetPlan.Response:
        """
        Callback for the GetPlan service. Receives start and goal, returns a path.
        """
        start_coords = [request.start.pose.position.x, request.start.pose.position.y]
        goal_coords = [request.goal.pose.position.x, request.goal.pose.position.y]

        self.get_logger().info(f"Received path request from {start_coords} to {goal_coords}")

        # Get planner settings from ROS parameters
        w = self.get_parameter('w_heuristic').get_parameter_value().double_value
        num_points = self.get_parameter('smoothing_points').get_parameter_value().integer_value
        threshold = self.get_parameter('smoothing_collision_threshold').get_parameter_value().double_value

        # 1. Find the raw path using A*
        raw_path = self.planner.find_path(start_coords, goal_coords, w=w)

        if not raw_path:
            self.get_logger().warn("No path found by A* algorithm.")
            response.plan = Path() # Return an empty path
            response.plan.header.stamp = self.get_clock().now().to_msg()
            response.plan.header.frame_id = 'map'
            return response

        self.get_logger().info(f"Found raw A* path with {len(raw_path)} points.")

        # 2. Smooth the path
        smoothed_path = self.planner.smooth_path(raw_path, num_points=num_points, collision_threshold=threshold)
        final_path_points = smoothed_path if smoothed_path else raw_path
        
        self.get_logger().info(f"Path smoothed to {len(final_path_points)} points.")

        # 3. Convert the path to a nav_msgs/Path message
        ros_path = Path()
        ros_path.header.stamp = self.get_clock().now().to_msg()
        ros_path.header.frame_id = 'map' 

        for point in final_path_points:
            pose = PoseStamped()
            pose.header = ros_path.header
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.orientation.w = 1.0
            ros_path.poses.append(pose)
        
        response.plan = ros_path
        self.get_logger().info("Successfully planned and returned path.")

        self.path_publisher.publish(ros_path)


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