#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tiago.srv import UpdateCustomerInformation
import uuid
from datetime import datetime


class Database(Node):
    def __init__(self):
        super().__init__('database')
        self.get_logger().info(f"Node 'database' (class: Database) has been started.")

        # Initialize customer database (in-memory storage)
        self.customers = {}

        # Create service server
        self.srv = self.create_service(
            UpdateCustomerInformation,
            'update_customer_information',
            self.update_customer_callback
        )

        self.get_logger().info("Database service 'update_customer_information' ready.")

    def update_customer_callback(self, request, response):
        """
        Handle customer information update requests.
        If customer_id is empty, generate a new one.
        If customer_id exists, update the existing record.
        """
        try:
            # Generate new customer ID if not provided or empty
            if not request.customer_id or request.customer_id.strip() == "":
                customer_id = self.generate_customer_id()
                self.get_logger().info(f"Generated new customer ID: {customer_id}")
            else:
                customer_id = request.customer_id
                self.get_logger().info(f"Updating existing customer ID: {customer_id}")

            # Create customer record
            customer_data = {
                'customer_id': customer_id,
                'name': request.name,
                'age_category': request.age_category,
                'gender': request.gender,
                'preferences': list(request.preferences),
                'size_info': request.size_info,
                'budget': request.budget,
                'products_of_interest': list(request.products_of_interest),
                'purchase_intent': request.purchase_intent,
                'follow_up_needed': request.follow_up_needed,
                'conversation_summary': request.conversation_summary,
                'last_updated': datetime.now().isoformat()
            }

            # Store in database
            self.customers[customer_id] = customer_data

            # Prepare response
            response.customer_id = customer_id
            response.name = request.name
            response.age_category = request.age_category
            response.gender = request.gender
            response.preferences = request.preferences
            response.size_info = request.size_info
            response.budget = request.budget
            response.products_of_interest = request.products_of_interest
            response.purchase_intent = request.purchase_intent
            response.follow_up_needed = request.follow_up_needed
            response.conversation_summary = request.conversation_summary

            self.get_logger().info(f"Successfully updated customer: {customer_id}")
            self.get_logger().info(f"Customer name: {request.name}")
            self.get_logger().info(f"Total customers in database: {len(self.customers)}")

            return response

        except Exception as e:
            self.get_logger().error(f"Error updating customer information: {str(e)}")
            # Return empty response on error
            response.customer_id = ""
            return response

    def generate_customer_id(self):
        """Generate a unique customer ID"""
        # Generate UUID-based ID with timestamp prefix
        timestamp = datetime.now().strftime("%Y%m%d")
        unique_id = str(uuid.uuid4())[:8]
        return f"CUST_{timestamp}_{unique_id}"

    def get_customer(self, customer_id):
        """Retrieve customer information by ID"""
        return self.customers.get(customer_id, None)

    def get_all_customers(self):
        """Get all customers (for debugging/admin purposes)"""
        return self.customers

    def print_database_stats(self):
        """Print database statistics"""
        self.get_logger().info(f"Database contains {len(self.customers)} customers")
        for customer_id, data in self.customers.items():
            self.get_logger().info(f"  - {customer_id}: {data['name']} (last updated: {data['last_updated']})")


def main(args=None):
    rclpy.init(args=args)
    node = Database()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Database node shutting down...")
        node.print_database_stats()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()