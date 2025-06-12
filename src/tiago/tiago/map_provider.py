#!/usr/bin/env python3
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import datetime
from typing import Optional

# ROS2 messages
from geometry_msgs.msg import Polygon as ROSPolygon
from geometry_msgs.msg import Point as RosPoint
from builtin_interfaces.msg import Time, Duration

# Your custom service definitions (replace 'your_package' with actual package name)
from tiago.srv import (
    GetMapInfo, GetProductLocation, GetStaffLocation,
    UpdateProductLocation, UpdateStaffLocation, GetAreaAtPoint,
    CheckOccupancy
)

# Import your Map class (adjust the import path as needed)
from tiago.lib.map.geometry import Point2D, Polygon
from tiago.lib.map.map import Map


class MapProviderNode(Node):
    """
    ROS2 Node that provides map and observation services.
    """

    def __init__(self, map_file_path: str = 'ground.mp',
                 cleanup_interval: float = 30.0,
                 max_observation_age: float = 300.0):
        super().__init__('map_service_node')

        # Initialize or load the map
        try:
            self.map = Map.from_image(map_file_path)
            self.get_logger().info(f"Loaded map from {map_file_path}")
        except FileNotFoundError as e:
            self.get_logger().warn(f"Map file {map_file_path} not found! Aborting node initialization.")
            raise e

        # Use ReentrantCallbackGroup to allow multiple services to be called simultaneously
        self.callback_group = ReentrantCallbackGroup()

        self.cleanup_interval = cleanup_interval  # seconds between cleanup runs
        self.max_observation_age = max_observation_age  # max age before observations are cleared
        # Create services
        self.create_services()
        # Setup periodic cleanup timer
        self.setup_cleanup_timer()

        self.get_logger().info("Map service node initialized with all services")
        self.get_logger().info(f"Cleanup interval: {cleanup_interval}s, Max observation age: {max_observation_age}s")

    def create_services(self):
        """Create all the service servers."""

        # Map information service
        self.map_info_service = self.create_service(
            GetMapInfo,
            'get_map_info',
            self.get_map_info_callback,
            callback_group=self.callback_group
        )

        # Product location services
        self.get_product_location_service = self.create_service(
            GetProductLocation,
            'get_product_location',
            self.get_product_location_callback,
            callback_group=self.callback_group
        )

        self.update_product_location_service = self.create_service(
            UpdateProductLocation,
            'update_product_location',
            self.update_product_location_callback,
            callback_group=self.callback_group
        )

        # Staff location services
        self.get_staff_location_service = self.create_service(
            GetStaffLocation,
            'get_staff_location',
            self.get_staff_location_callback,
            callback_group=self.callback_group
        )

        self.update_staff_location_service = self.create_service(
            UpdateStaffLocation,
            'update_staff_location',
            self.update_staff_location_callback,
            callback_group=self.callback_group
        )

        # Area services
        self.get_area_at_point_service = self.create_service(
            GetAreaAtPoint,
            'get_area_at_point',
            self.get_area_at_point_callback,
            callback_group=self.callback_group
        )

        # Occupancy service
        self.check_occupancy_service = self.create_service(
            CheckOccupancy,
            'check_occupancy',
            self.check_occupancy_callback,
            callback_group=self.callback_group
        )

    def setup_cleanup_timer(self):
        """Setup the periodic cleanup timer."""
        self.cleanup_timer = self.create_timer(
            self.cleanup_interval,
            self.periodic_cleanup_callback,
            callback_group=self.callback_group
        )
        self.get_logger().info(f"Periodic cleanup timer started (interval: {self.cleanup_interval}s)")

    def periodic_cleanup_callback(self):
        """Periodic callback to clean up expired observations."""
        try:
            # Create timedelta for max age
            max_age = datetime.timedelta(seconds=self.max_observation_age)

            # Count items before clearing
            products_before = len(self.map.observations._last_seen_products)
            staff_before = len(self.map.observations._last_seen_staff)

            # Clear expired observations
            self.map.observations.clear_expired(max_age)

            # Count items after clearing
            products_after = len(self.map.observations._last_seen_products)
            staff_after = len(self.map.observations._last_seen_staff)

            cleared_products = products_before - products_after
            cleared_staff = staff_before - staff_after

            # Only log if something was cleared
            if cleared_products > 0 or cleared_staff > 0:
                self.get_logger().info(
                    f"Periodic cleanup: cleared {cleared_products} products and {cleared_staff} staff"
                )
                # Save map to file if changes were made
                self.map.save()
            else:
                self.get_logger().debug("Periodic cleanup: no expired observations to clear")

        except Exception as e:
            self.get_logger().error(f"Error in periodic cleanup: {str(e)}")

    def get_map_info_callback(self, request, response):
        """Get basic map information including occupancy grid and areas."""
        try:
            response.width = self.map.width
            response.height = self.map.height

            # Flatten the 2D occupancy grid
            response.occupancy_grid = []
            for row in self.map.occupancy_grid:
                response.occupancy_grid.extend(row)

            # Convert named areas to ROS format
            response.area_names = list(self.map.area_names)
            response.areas = []

            for area_name in self.map.area_names:
                area_polygon = self.map.get_area_polygon(area_name)
                ros_polygon = ROSPolygon()
                for vertex in area_polygon.vertices:
                    ros_point = RosPoint()
                    ros_point.x = float(vertex.x)
                    ros_point.y = float(vertex.y)
                    ros_point.z = 0.0
                    ros_polygon.points.append(ros_point)
                response.areas.append(ros_polygon)

            self.get_logger().debug("Map info requested")

        except Exception as e:
            self.get_logger().error(f"Error in get_map_info: {str(e)}")

        return response

    def get_product_location_callback(self, request, response):
        """Get the last known location of a product."""
        try:
            location = self.map.observations.get_product_location(request.product_id)

            if location is not None:
                response.found = True
                response.position.x = float(location.x)
                response.position.y = float(location.y)
                response.position.z = 0.0

                # Get timestamp
                if request.product_id in self.map.observations._last_seen_products:
                    timestamp = self.map.observations._last_seen_products[request.product_id][1]
            else:
                response.found = False

            self.get_logger().debug(f"Product location requested for ID: {request.product_id}")

        except Exception as e:
            self.get_logger().error(f"Error in get_product_location: {str(e)}")
            response.found = False

        return response

    def update_product_location_callback(self, request, response):
        """Update the location of a product."""
        try:
            point2d = Point2D(request.position.x, request.position.y)
            self.map.observations.update_product_location(request.product_id, point2d)
            response.success = True

            # Save map to file
            self.map.save()

            self.get_logger().debug(f"Updated product location for ID: {request.product_id}")

        except Exception as e:
            self.get_logger().error(f"Error in update_product_location: {str(e)}")
            response.success = False

        return response

    def get_staff_location_callback(self, request, response):
        """Get the last known location of a staff member."""
        try:
            location = self.map.observations.get_staff_location(request.staff_id)

            if location is not None:
                response.found = True
                response.position.x = float(location.x)
                response.position.y = float(location.y)
                response.position.z = 0.0

            else:
                response.found = False

            self.get_logger().debug(f"Staff location requested for ID: {request.staff_id}")

        except Exception as e:
            self.get_logger().error(f"Error in get_staff_location: {str(e)}")
            response.found = False

        return response

    def update_staff_location_callback(self, request, response):
        """Update the location of a staff member."""
        try:
            point2d = Point2D(request.position.x, request.position.y)
            self.map.observations.update_staff_location(request.staff_id, point2d)
            response.success = True

            # Save map to file
            self.map.save()

            self.get_logger().debug(f"Updated staff location for ID: {request.staff_id}")

        except Exception as e:
            self.get_logger().error(f"Error in update_staff_location: {str(e)}")
            response.success = False

        return response

    def get_area_at_point_callback(self, request, response):
        """Get the area that contains a given point."""
        try:
            point2d = Point2D(request.point.x, request.point.y)
            area_info = self.map.get_area(point2d)

            if area_info is not None:
                area_name, area_polygon = area_info
                response.found = True
                response.area_name = area_name

                # Convert polygon to ROS format
                ros_polygon = ROSPolygon()
                for vertex in area_polygon.vertices:
                    point32 = RosPoint()
                    point32.x = float(vertex.x)
                    point32.y = float(vertex.y)
                    point32.z = 0.0
                    ros_polygon.points.append(point32)
                response.area = ros_polygon
            else:
                response.found = False

            self.get_logger().debug(f"Area requested for point: ({request.point.x}, {request.point.y})")

        except Exception as e:
            self.get_logger().error(f"Error in get_area_at_point: {str(e)}")
            response.found = False

        return response

    def check_occupancy_callback(self, request, response):
        """Check if a cell is occupied."""
        try:
            # Check if coordinates are valid
            if not (0 <= request.x < self.map.width and 0 <= request.y < self.map.height):
                response.valid_coordinates = False
                response.occupied = False
            else:
                response.valid_coordinates = True
                response.occupied = self.map.is_occupied(request.x, request.y)

            self.get_logger().debug(f"Occupancy check for ({request.x}, {request.y})")

        except Exception as e:
            self.get_logger().error(f"Error in check_occupancy: {str(e)}")
            response.valid_coordinates = False
            response.occupied = False

        return response


def main(args=None):
    rclpy.init(args=args)
    package_share = get_package_share_directory('tiago')
    resource_path = os.path.join(package_share, 'res', 'map.png')
    # Create the node
    map_service_node = MapProviderNode(
        map_file_path=resource_path,
        cleanup_interval=30.0,  # seconds between cleanup runs
        max_observation_age=600.0  # max age before observations are cleared
    )

    # Use MultiThreadedExecutor to handle multiple service calls
    executor = MultiThreadedExecutor()
    executor.add_node(map_service_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        map_service_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
