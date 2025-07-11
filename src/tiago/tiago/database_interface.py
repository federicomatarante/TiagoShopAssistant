#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import os
from ament_index_python.packages import get_package_share_directory
import datetime

from tiago.lib.database.database import Database
from tiago.lib.database.interface import DataInterface
from tiago.lib.map.map import Map, Point2D, Polygon  # Import Polygon as it's used in type hints
from tiago.lib.map.geometry import Point2D

# Import custom service and message types
from tiago.srv import (
    UpdateCustomerInformation,
    GetCustomerInformation,
    UpdateProductInformation,
    UpdateProductPosition,
    UpdateStaffPosition,
    UpdateStaffInformation,
    ExtractRelevantProducts,
    ExtractRelevantStaff,
    GetAreaPosition,
    GetProductLocation,
    GetStaffLocation,
)

from tiago.lib.ros_utils.msg_converters import (
    customer_to_info,
    product_to_info,
    staff_to_info,
    product_query_to_dict,
    staff_query_to_dict,
    customer_to_msg,
    msg_to_customer,
    msg_to_product,
    msg_to_staff,
    product_to_msg,
    staff_to_msg,
    point2d_to_msg  # Assuming you have this converter or will create it
)


class DatabaseInterface(Node):
    def __init__(self, data_interface: DataInterface, map_instance: Map):
        super().__init__('database_interface')
        self.get_logger().info("Node 'database_interface' has been started.")
        self.data_interface = data_interface
        self.map_instance = map_instance  # Store map instance
        self.create_services()
        self.start_observation_clearer(clear_interval_sec=5.0,
                                       max_age_sec=30.0)  # Clear observations every 5 seconds, max age 30 seconds

    def create_services(self):
        self.update_customer_information_service = self.create_service(
            UpdateCustomerInformation,
            'update_customer_information',
            self.update_customer_information_callback
        )

        self.get_customer_information_service = self.create_service(
            GetCustomerInformation,
            'get_customer_information',
            self.get_customer_information_callback
        )

        self.update_product_information_service = self.create_service(
            UpdateProductInformation,
            'update_product_information',
            self.update_product_information_callback
        )

        self.update_product_position_service = self.create_service(
            UpdateProductPosition,
            'update_product_position',
            self.update_product_position_callback
        )

        self.update_staff_position_service = self.create_service(
            UpdateStaffPosition,
            'update_staff_position',
            self.update_staff_position_callback
        )

        self.update_staff_information_service = self.create_service(
            UpdateStaffInformation,
            'update_staff_information',
            self.update_staff_information_callback
        )

        self.extract_relevant_products_service = self.create_service(
            ExtractRelevantProducts,
            'extract_relevant_products',
            self.extract_relevant_products_callback
        )

        self.extract_relevante_staff_service = self.create_service(
            ExtractRelevantStaff,
            'extract_relevant_staff',
            self.extract_relevant_staff_callback
        )

        # New services
        self.get_product_location_service = self.create_service(
            GetProductLocation,
            'get_product_location',
            self.get_product_location_callback
        )

        self.get_staff_location_service = self.create_service(
            GetStaffLocation,
            'get_staff_location',
            self.get_staff_location_callback
        )

        self.get_area_position_service = self.create_service(
            GetAreaPosition,
            'get_area_position',
            self.get_area_position_callback
        )

    def start_observation_clearer(self, clear_interval_sec: float, max_age_sec: float):
        """
        Starts a timer to periodically clear expired observations from the map.
        """
        self.clear_max_age = datetime.timedelta(seconds=max_age_sec)
        self.clear_timer = self.create_timer(clear_interval_sec, self.clear_observations_callback)
        self.get_logger().info(
            f"Observation clearer started with interval {clear_interval_sec}s and max age {max_age_sec}s.")

    def clear_observations_callback(self):
        """
        Callback function for the timer to clear expired observations.
        """
        self.map_instance.observations.clear_expired(self.clear_max_age)
        self.get_logger().debug("Expired observations cleared.")

    def update_customer_information_callback(self, request, response):
        self.get_logger().info(
            f"Received request to update customer info for customer_id: {request.customer.customer_id}")

        try:
            # Convert request message to customer info dictionary using converter
            customer_info = customer_to_info(request.customer, remove_null=True)  # Access request.customer directly

            # Update customer information through data interface
            updated_customer = self.data_interface.update_customer_information(request.customer.customer_id,
                                                                               customer_info)
            response.success = True
            self.get_logger().info(
                f"Customer information updated successfully for customer_id: {request.customer.customer_id}")
        except Exception as e:
            self.get_logger().error(f"Failed to update customer information for {request.customer.customer_id}: {e}")
            response.success = False
        return response

    def get_customer_information_callback(self, request, response):
        self.get_logger().info(f"Request to get customer info for customer_id: {request.customer_id}")

        try:
            # Get customer data from data interface
            customer_data = self.data_interface.get_customer_information(request.customer_id)

            # Convert customer entity to message using converter
            response.customer = customer_to_msg(customer_data)
            response.success = True
            self.get_logger().info(f"Successfully retrieved info for customer_id: {request.customer_id}")
        except Exception as e:
            self.get_logger().error(f"Failed to retrieve customer info for {request.customer_id}: {e}")
            response.success = False
        return response

    def update_product_information_callback(self, request, response):
        product = request.product
        self.get_logger().info(f"Request to update product info for product_id: {product.product_id}")

        try:
            product_dict = product_to_info(product)

            self.data_interface.update_product_information(product_dict, product.product_id)
            response.success = True
            self.get_logger().info(f"Product info updated for product_id: {product.product_id or 'new product'}")
        except Exception as e:
            self.get_logger().error(f"Failed to update product information: {e}")
            response.success = False
        return response

    def update_product_position_callback(self, request, response):
        self.get_logger().info(f"Request to update position for product_id: {request.product_id}")

        try:
            # Create position from request
            position = Point2D(x=request.position.x, y=request.position.y)

            # Update product position through data interface (which uses map_instance.observations)
            self.map_instance.observations.update_product_location(request.product_id, position)
            response.success = True
            self.get_logger().info(f"Successfully updated position for product_id: {request.product_id}")
        except Exception as e:
            self.get_logger().error(f"Failed to update product position for {request.product_id}: {e}")
            response.success = False
        return response

    def update_staff_position_callback(self, request, response):
        self.get_logger().info(f"Request to update position for staff_id: {request.staff_id}")

        try:
            # Create position from request
            position = Point2D(x=request.position.x, y=request.position.y)

            # Update staff position through data interface (which uses map_instance.observations)
            self.map_instance.observations.update_staff_location(request.staff_id, position)
            response.success = True
            self.get_logger().info(f"Successfully updated position for staff_id: {request.staff_id}")
        except Exception as e:
            self.get_logger().error(f"Failed to update staff position for {request.staff_id}: {e}")
            response.success = False
        return response

    def update_staff_information_callback(self, request, response):
        staff = request.staff
        self.get_logger().info(f"Request to update info for staff_id: {staff.staff_id}")

        try:
            staff_dict = staff_to_info(staff)

            # Update staff information through data interface
            self.data_interface.update_staff_information(staff_dict, staff.staff_id)
            response.success = True
            self.get_logger().info(f"Staff info updated for staff_id: {staff.staff_id or 'new staff'}")
        except Exception as e:
            self.get_logger().error(f"Failed to update staff information: {e}")
            response.success = False
        return response

    def extract_relevant_products_callback(self, request, response):
        self.get_logger().info("Request to extract relevant products.")

        query = product_query_to_dict(request.query)

        try:
            products, areas = self.data_interface.extract_relevant_products(query)
            response.products = [
                product_to_msg(product) for product in products
            ]
            response.areas = areas if areas else []
            response.success = True
            self.get_logger().info(f"Successfully extracted {len(response.products)} relevant products.")
        except Exception as e:
            self.get_logger().error(f"Failed to extract relevant products: {e}")
            response.success = False
        return response

    def extract_relevant_staff_callback(self, request, response):
        self.get_logger().info("Request to extract relevant staff.")

        query = staff_query_to_dict(request.query)

        try:
            staff_members, areas = self.data_interface.extract_relevant_staff(query)
            response.staff = [
                staff_to_msg(staff) for staff in staff_members
            ]
            response.areas = areas if areas else []
            response.success = True
            self.get_logger().info(f"Successfully extracted {len(response.staff)} relevant staff members.")
        except Exception as e:
            self.get_logger().error(f"Failed to extract relevant staff: {e}")
            response.success = False
        return response

    # New methods for getting location information
    def get_product_location_callback(self, request, response):
        self.get_logger().info(f"Request to get product location for product_id: {request.product_id}")
        try:
            location = self.map_instance.observations.get_product_location(request.product_id)
            if location:
                response.position = point2d_to_msg(location)
                response.success = True
                self.get_logger().info(
                    f"Successfully retrieved location for product_id {request.product_id}: {location.x}, {location.y}")
            else:
                response.success = False
                self.get_logger().warn(f"Product with ID {request.product_id} not found in observations.")
        except Exception as e:
            self.get_logger().error(f"Failed to get product location for {request.product_id}: {e}")
            response.success = False
        return response

    def get_staff_location_callback(self, request, response):
        self.get_logger().info(f"Request to get staff location for staff_id: {request.staff_id}")
        try:
            location = self.map_instance.observations.get_staff_location(request.staff_id)
            if location:
                response.position = point2d_to_msg(location)
                response.success = True
                self.get_logger().info(
                    f"Successfully retrieved location for staff_id {request.staff_id}: {location.x}, {location.y}")
            else:
                response.success = False
                self.get_logger().warn(f"Staff with ID {request.staff_id} not found in observations.")
        except Exception as e:
            self.get_logger().error(f"Failed to get staff location for {request.staff_id}: {e}")
            response.success = False
        return response

    def get_area_position_callback(self, request, response):
        self.get_logger().info(f"Request to get position for area_name: {request.area_name}")
        try:
            area_polygon = self.map_instance.get_area_polygon(request.area_name)
            if area_polygon:
                # For an area, returning the centroid is a common approach for "position"
                # If a different representation is needed (e.g., list of vertices), adjust accordingly.
                centroid = area_polygon.centroid
                response.position = point2d_to_msg(centroid)
                response.success = True
                self.get_logger().info(
                    f"Successfully retrieved position for area {request.area_name}: {centroid.x}, {centroid.y}")
            else:
                response.success = False
                self.get_logger().warn(f"Area with name {request.area_name} not found.")
        except Exception as e:
            self.get_logger().error(f"Failed to get area position for {request.area_name}: {e}")
            response.success = False
        return response


def main(args=None):
    rclpy.init(args=args)
    package_share = get_package_share_directory('tiago')
    map_path = os.path.join(package_share, 'res', 'map.map')
    db_path = os.path.join(package_share, 'res', 'database.db')

    database = Database(db_path)
    map_instance = Map.from_file(map_path)
    data_interface = DataInterface(database, map_instance)
    database_interface_node = DatabaseInterface(data_interface, map_instance)  # Pass map_instance to the node

    try:
        rclpy.spin(database_interface_node)
    except KeyboardInterrupt:
        pass
    finally:
        database_interface_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
