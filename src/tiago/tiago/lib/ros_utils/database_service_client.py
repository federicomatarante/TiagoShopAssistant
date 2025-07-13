from typing import Optional, Tuple, List, Dict, Any, Callable

import rclpy
from rclpy.node import Node

from tiago.srv import (
    GetCustomerInformation,
    UpdateCustomerInformation,
    ExtractRelevantProducts,
    ExtractRelevantStaff
)

from tiago.lib.ros_utils.msg_converters import (
    customer_to_msg,
    msg_to_customer,
    dict_to_product_query,
    dict_to_staff_query
)

# Assuming these imports are available in your project structure
from tiago.lib.database.entities import Customer
from tiago.lib.ros_utils.msg_converters import msg_to_product, msg_to_staff

class DatabaseServiceClient:
    """Auxiliary class to handle communication with DatabaseInterface services"""

    def __init__(self, node: Node):
        self.node = node
        self._setup_service_clients()
        self.node.get_logger().info("DatabaseServiceClient initialized")

    def _setup_service_clients(self):
        """Initialize service clients for database operations"""
        self.get_customer_info_client = self.node.create_client(
            GetCustomerInformation,
            'get_customer_information'
        )

        self.update_customer_info_client = self.node.create_client(
            UpdateCustomerInformation,
            'update_customer_information'
        )

        self.extract_products_client = self.node.create_client(
            ExtractRelevantProducts,
            'extract_relevant_products'
        )

        self.extract_staff_client = self.node.create_client(
            ExtractRelevantStaff,
            'extract_relevant_staff'
        )

        self.node.get_logger().info("Service clients set up")

    def wait_for_services(self, timeout_sec: float = 30.0) -> bool:
        """Wait for all required services to become available"""
        services = [
            self.get_customer_info_client,
            self.update_customer_info_client,
            self.extract_products_client,
            self.extract_staff_client
        ]

        for client in services:
            self.node.get_logger().info(f"Waiting for service: {client.srv_name}")
            if not client.wait_for_service(timeout_sec=timeout_sec):
                self.node.get_logger().error(f"Service {client.srv_name} not available after {timeout_sec} seconds.")
                return False
            self.node.get_logger().info(f"{client.srv_name} is available.")

        self.node.get_logger().info("All database services are available")
        return True

    def get_customer_information(self, customer_id: str, callback: Callable[[Optional[Dict[str, Any]]], None]):
        """
        Get customer information from database service.
        The result is passed to the provided callback function.
        """
        request = GetCustomerInformation.Request()
        request.customer_id = customer_id
        self.node.get_logger().info(f"Sending get_customer_information request for: {customer_id}")

        future = self.get_customer_info_client.call_async(request)
        future.add_done_callback(lambda future: self._get_customer_information_callback(future, customer_id, callback))

    def _get_customer_information_callback(self, future, customer_id: str, user_callback: Callable[[Optional[Dict[str, Any]]], None]):
        """Callback for get_customer_information service response."""
        try:
            result = future.result()
            self.node.get_logger().info(f"Response received for {customer_id}: {result}")

            if result and result.success:
                self.node.get_logger().info(f"Customer info retrieval successful for {customer_id}")
                customer_data = msg_to_customer(result.customer)
                user_callback(customer_data)
            else:
                self.node.get_logger().warn(f"Customer info retrieval failed for {customer_id}: {result.message if hasattr(result, 'message') else 'Unknown reason'}")
                user_callback(None)
        except Exception as e:
            self.node.get_logger().error(f"Error getting customer info for {customer_id}: {e}")
            user_callback(None)

    def update_customer_information(self, customer_id: str, customer_information: Dict[str, Any], callback: Optional[Callable[[bool], None]] = None) -> None:
        """
        Update customer information via database service.
        The success status is passed to the provided callback function.
        """
        request = UpdateCustomerInformation.Request()

        try:
            temp_customer = Customer(customer_id)

            for key, value in customer_information.items():
                if hasattr(temp_customer, key):
                    setattr(temp_customer, key, value)
                    # self.node.get_logger().info(f"Set {key} to {value} on temp_customer")

            request.customer = customer_to_msg(temp_customer)
            self.node.get_logger().info(f"Sending update_customer_information request for {customer_id}")

            future = self.update_customer_info_client.call_async(request)
            future.add_done_callback(lambda future: self._update_customer_information_callback(future, customer_id, callback))

        except Exception as e:
            self.node.get_logger().error(f"Error preparing update customer info request for {customer_id}: {e}")
            if callback:
                callback(False)

    def _update_customer_information_callback(self, future, customer_id: str, user_callback: Optional[Callable[[bool], None]]):
        """Callback for update_customer_information service response."""
        try:
            result = future.result()
            success = result.success if result else False
            self.node.get_logger().info(f"Update customer information for {customer_id} successful: {success}")
            if user_callback:
                user_callback(success)
        except Exception as e:
            self.node.get_logger().error(f"Error updating customer info for {customer_id}: {e}")
            if user_callback:
                user_callback(False)


    def extract_relevant_products(self, products_query: Dict[str, Any], callback: Callable[[List, Optional[List[str]]], None]):
        """
        Extract relevant products via database service.
        The result (products list, areas list) is passed to the provided callback function.
        """
        request = ExtractRelevantProducts.Request()
        request.query = dict_to_product_query(products_query)

        self.node.get_logger().info(f"Sending extract_relevant_products request: {request.query}")

        future = self.extract_products_client.call_async(request)
        future.add_done_callback(lambda future: self._extract_products_callback(future, callback))

    def _extract_products_callback(self, future, user_callback: Callable[[List, Optional[List[str]]], None]):
        """Callback for extract_relevant_products service response."""
        try:
            result = future.result()
            self.node.get_logger().info(f"Products extraction response: {result}")

            if result and result.success:
                products = [msg_to_product(prod_msg) for prod_msg in result.products]
                areas = result.areas if result.areas else None
                user_callback(products, areas)
            else:
                self.node.get_logger().warn("Products extraction failed.")
                user_callback([], None)
        except Exception as e:
            self.node.get_logger().error(f"Error extracting products: {e}")
            user_callback([], None)


    def extract_relevant_staff(self, staff_query: Dict[str, Any], callback: Callable[[List, Optional[List[str]]], None]):
        """
        Extract relevant staff via database service.
        The result (staff list, areas list) is passed to the provided callback function.
        """
        request = ExtractRelevantStaff.Request()
        request.query = dict_to_staff_query(staff_query)

        self.node.get_logger().info(f"Sending extract_relevant_staff request: {request.query}")

        future = self.extract_staff_client.call_async(request)
        future.add_done_callback(lambda future: self._extract_staff_callback(future, callback))

    def _extract_staff_callback(self, future, user_callback: Callable[[List, Optional[List[str]]], None]):
        """Callback for extract_relevant_staff service response."""
        try:
            result = future.result()
            self.node.get_logger().info(f"Staff extraction response: {result}")

            if result and result.success:
                staff = [msg_to_staff(staff_msg) for staff_msg in result.staff]
                areas = result.areas if result.areas else None
                user_callback(staff, areas)
            else:
                self.node.get_logger().warn("Staff extraction failed.")
                user_callback([], None)
        except Exception as e:
            self.node.get_logger().error(f"Error extracting staff: {e}")
            user_callback([], None)