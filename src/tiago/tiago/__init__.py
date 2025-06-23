# Inside your PathPlannerService class

def __init__(self):
    """
    Initializes the node, loads the map, and creates the service server.
    """
    super().__init__('path_planner_service')
    self.get_logger().info(f"Node '{self.get_name()}' has been started.")

    self.declare_parameter('map_filename', 'my_map.yaml')

    try:
        # This is the robust ROS 2 way to find data files
        package_share_dir = get_package_share_directory('tiago')
        map_filename = self.get_parameter('map_filename').get_parameter_value().string_value
        
        # Construct the full path to the map file inside the installed 'share/tiago/maps' directory
        map_file_path = os.path.join(package_share_dir, 'maps', map_filename)

        self.get_logger().info(f"Attempting to load map from: {map_file_path}")
        if not os.path.exists(map_file_path):
             raise FileNotFoundError(f"Map file not found at the installed path. Check CMakeLists.txt install rule.")

        self.planner = AStarPathPlanner(map_file_path)
        self.get_logger().info("A* planner initialized successfully.")

    except Exception as e:
        self.get_logger().error(f"Failed to initialize A* planner: {e}")
        raise e

    # ... rest of your __init__ method ...