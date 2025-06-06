#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

# Import your custom service
from tiago.srv import AddThreeInts # Replace my_py_pkg with your package name

class AddThreeIntsServerNode(Node):
    def __init__(self):
        super().__init__('add_three_ints_server')
        # Create the service server
        self.srv = self.create_service(
            AddThreeInts,       # Service type
            'add_three_ints',   # Service name (how clients will call it)
            self.add_three_ints_callback) # Callback function to handle requests
        self.get_logger().info('AddThreeInts service server started and ready.')

    def add_three_ints_callback(self, request, response):
        # This function is called when a client makes a request
        # 'request' is an instance of AddThreeInts.Request
        # 'response' is an instance of AddThreeInts.Response

        response.sum = request.a + request.b + request.c

        self.get_logger().info(
            f'Incoming request: a={request.a}, b={request.b}, c={request.c}. Returning sum: {response.sum}'
        )

        return response # Return the populated response object

def main(args=None):
    rclpy.init(args=args)
    node = AddThreeIntsServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()