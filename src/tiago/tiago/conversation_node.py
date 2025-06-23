#!/usr/bin/env python3
import os
from typing import Optional, Tuple, List, Dict, Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
import sys
from tiago.lib.assistant import ShopAssistant

from tiago.msg import ConversationStatus
from tiago.lib.ros_utils.database_service_client import DatabaseServiceClient


class ConversationNode(Node):
    def __init__(self, model_name: str):
        super().__init__('conversation_node')

        # Conversation state
        self.current_assistant: Optional[ShopAssistant] = None
        self.model_name = model_name
        self.current_customer_id: Optional[str] = None
        self.conversation_active = False
        self.current_user_input: Optional[str] = None  # Store the input while async call is pending

        # Database service client
        self.db_client = DatabaseServiceClient(self)

        # ROS2 Publishers and Subscribers
        self._setup_ros_communication()

        # Wait for database services to be available
        if not self.db_client.wait_for_services():
            self.get_logger().error("Required database services not available. Shutting down.")
            raise RuntimeError("Database services not available")

        self.get_logger().info("ConversationNode initialized and ready")

    def _setup_ros_communication(self):
        """Setup ROS2 publishers and subscribers"""

        # Subscriber for controller commands (start/stop conversation)
        self.controller_subscriber = self.create_subscription(
            ConversationStatus,
            '/conversation/controller',
            self.controller_command_callback,
            10
        )

        # Subscriber for speech recognition input
        self.speech_subscriber = self.create_subscription(
            String,
            '/speech/recognized_text',
            self.speech_input_callback,
            10
        )

        # Publisher for TTS (text-to-speech) output
        self.tts_publisher = self.create_publisher(
            String,
            '/speech/tts_input',
            10
        )

        # Publisher for walk area commands
        self.walk_area_publisher = self.create_publisher(
            String,
            '/navigation/walk_to_area',
            10
        )

        # Service to check if node is online
        self.status_service = self.create_service(
            Trigger,
            '/conversation/status',
            self.status_service_callback
        )

    def status_service_callback(self, request, response):
        """Service callback to report if the node is online"""
        response.success = True
        response.message = "ConversationNode is online and ready"
        return response

    def controller_command_callback(self, msg: ConversationStatus):
        """Handle commands from the controller"""
        self.get_logger().info(f"Received controller command: {msg}")
        try:
            command = msg.command

            if command == 'start':
                customer_id = msg.customer_id
                if customer_id.lower() == 'none':
                    self.get_logger().warn("Received 'start' command with no customer ID")
                    return
                self._start_conversation(customer_id)
            elif command == 'stop':
                self._finalize_conversation()
            else:
                self.get_logger().warn(f"Unknown controller command!")

        except Exception as e:
            self.get_logger().error(f"Error processing controller command: {e}")

    def speech_input_callback(self, msg: String):
        """Handle speech recognition input"""
        if not self.conversation_active or not self.current_assistant:
            self.get_logger().warn("Received speech input but no active conversation")
            return

        user_input = msg.data.strip()
        if not user_input:
            return

        try:
            # Log the user input
            self.get_logger().info(f"User ({self.current_customer_id}): {user_input}")
            self.current_user_input = user_input  # Store for later use in callbacks

            # Process the input and potentially trigger async calls
            self._process_user_input_async(user_input)

        except Exception as e:
            self.get_logger().error(f"Error processing speech input: {e}")
            error_msg = "Sorry, I encountered an error. Could you please repeat that?"
            tts_msg = String()
            tts_msg.data = error_msg
            self.tts_publisher.publish(tts_msg)

    def _process_user_input_async(self, user_input: str):
        self.current_assistant.add_message(user_input)
        direction = self.current_assistant.get_conversation_direction()
        products_query = self.current_assistant.detect_products_query()
        staff_query = self.current_assistant.detect_staff_query()
        self.get_logger().info(f"Detected direction: {direction}")
        # Depending on the direction, call the appropriate async database client method
        # and provide a specific callback to handle its response.
        if direction == "walk_to_product":
            self.db_client.extract_relevant_products(
                products_query,
                lambda products, areas: self._handle_product_walk_response(products, areas, user_input)
            )
        elif direction == "walk_to_staff":
            self.db_client.extract_relevant_staff(
                staff_query,
                lambda staff, areas: self._handle_staff_walk_response(staff, areas, user_input)
            )
        elif direction == "product_info":
            self.db_client.extract_relevant_products(
                products_query,
                lambda products, areas: self._handle_product_info_response(products, areas, user_input)
            )
        elif direction == "staff_info":
            self.db_client.extract_relevant_staff(
                staff_query,
                lambda staff, areas: self._handle_staff_info_response(staff, areas, user_input)
            )
        elif direction == "finished":
            assistant_reply = self.current_assistant.answer(option="Say goodbye to the customer")
            self._publish_assistant_reply(assistant_reply)
            self._finalize_conversation()
        else:
            # For directions that don't require database interaction, respond immediately
            assistant_reply = self.current_assistant.answer()
            self._publish_assistant_reply(assistant_reply)

    def _publish_assistant_reply(self, reply: str):
        """Helper to publish TTS messages."""
        if reply:
            tts_msg = String()
            tts_msg.data = reply
            self.tts_publisher.publish(tts_msg)
            self.get_logger().info(f"Assistant: {reply}")

    def _publish_walk_area(self, areas: Optional[List[str]]):
        """Helper to publish walk area command."""
        if areas and len(areas) > 0:
            area_msg = String()
            area_msg.data = areas[0]  # Use the first area
            self.walk_area_publisher.publish(area_msg)
            self.get_logger().info(f"Published walk to area: {areas[0]}")

    # --- Callbacks for DatabaseServiceClient responses ---

    def _handle_product_walk_response(self, products: List, areas: Optional[List[str]], original_user_input: str):
        """Handles response for 'walk_to_product' query."""
        if len(products) > 0:
            product = products[0]
            self.get_logger().info(f"Found product for walk: {product.name}")
            results = {'products': [product.to_json(), ]}

            # Add areas to results if available
            if areas:
                results['areas'] = areas

            assistant_reply = self.current_assistant.answer_with_results(results,
                                                                         option="Say the customer you're going to walk him to the product")

            # Publish walk area command
            self._publish_walk_area(areas)
        else:
            self.get_logger().warn(f"Could not find product for walk.")
            assistant_reply = self.current_assistant.answer(
                option="Say the customer you couldn't find the product so you cannot walk him to it"
            )
        self._publish_assistant_reply(assistant_reply)

    def _handle_staff_walk_response(self, staff: List, areas: Optional[List[str]], original_user_input: str):
        """Handles response for 'walk_to_staff' query."""
        if len(staff) > 0:
            staff_member = staff[0]
            self.get_logger().info(f"Found staff member for walk: {staff_member.name}")
            results = {'staff': [staff_member.to_json(), ]}

            # Add areas to results if available
            if areas:
                results['areas'] = areas

            assistant_reply = self.current_assistant.answer_with_results(results,
                                                                         option="Say the customer you're going to walk him to the staff member")

            # Publish walk area command
            self._publish_walk_area(areas)
        else:
            self.get_logger().warn(f"Could not find staff member for walk.")
            assistant_reply = self.current_assistant.answer(
                option="Say the customer you couldn't find the staff member so you cannot walk him to it"
            )
        self._publish_assistant_reply(assistant_reply)

    def _handle_product_info_response(self, products: List, areas: Optional[List[str]], original_user_input: str):
        """Handles response for 'product_info' query."""
        if len(products) > 0:
            self.get_logger().info(f"Found product(s) for info: {[p.name for p in products]}")
            results = {'products': [product.to_json() for product in products]}

            # Add areas to results if available
            if areas:
                results['areas'] = areas

            assistant_reply = self.current_assistant.answer_with_results(results,
                                                                         option="Say the customer you found the product information")
        else:
            self.get_logger().warn(f"Could not find product for info.")
            assistant_reply = self.current_assistant.answer(
                option="Say the customer you couldn't find the product information"
            )
        self._publish_assistant_reply(assistant_reply)

    def _handle_staff_info_response(self, staff: List, areas: Optional[List[str]], original_user_input: str):
        """Handles response for 'staff_info' query."""
        if len(staff) > 0:
            self.get_logger().info(f"Found staff member(s) for info: {[s.name for s in staff]}")
            results = {'staff': [staffer.to_json() for staffer in staff]}

            # Add areas to results if available
            if areas:
                results['areas'] = areas

            assistant_reply = self.current_assistant.answer_with_results(results,
                                                                         option="Say the customer you found the staff information")
        else:
            self.get_logger().warn(f"Could not find staff member for info.")
            assistant_reply = self.current_assistant.answer(
                option="Say the customer you couldn't find the staff information"
            )
        self._publish_assistant_reply(assistant_reply)

    def _start_conversation(self, customer_id: str):
        """Start a new conversation with a customer"""
        try:
            self.current_assistant = ShopAssistant(
                model=self.model_name,
                customer_id=customer_id,
            )
            self.current_customer_id = customer_id
            self.conversation_active = True

            self.get_logger().info(f"Attempting to start conversation with customer {customer_id}")
            # Asynchronously get customer information
            self.db_client.get_customer_information(customer_id, self._on_customer_info_received)

        except Exception as e:
            self.get_logger().error(f"Failed to initiate conversation: {e}")

    def _on_customer_info_received(self, customer_information: Optional[Dict[str, Any]]):
        """Callback executed when customer information is received."""
        if self.current_assistant and self.current_customer_id:
            if customer_information:
                self.current_assistant.set_customer_information(customer_information)
                self.get_logger().info(f"Customer information loaded for {self.current_customer_id}")
            else:
                self.get_logger().warn(
                    f"No customer information found or retrieval failed for {self.current_customer_id}. Starting conversation without it.")

            initial_greeting = self.current_assistant.answer(option="Say a greeting to the customer")
            self._publish_assistant_reply(initial_greeting)
        else:
            self.get_logger().error("Callback received, but conversation state is invalid or customer ID is missing.")

    def _finalize_conversation(self):
        """Finalize and clean up the conversation"""
        if self.current_customer_id and self.current_assistant:
            self.get_logger().info(f"Ending conversation with customer {self.current_customer_id}")

            knowledge = self.current_assistant.extract_knowledge()
            if knowledge:
                # Asynchronously update customer information
                self.db_client.update_customer_information(
                    self.current_customer_id,
                    knowledge,
                    self._on_customer_knowledge_saved  # Optional: Add a callback to know if save succeeded
                )
                self.get_logger().info("Customer knowledge extraction initiated.")
            else:
                self.get_logger().warn("No customer knowledge to save.")

        self.current_assistant = None
        self.current_customer_id = None
        self.conversation_active = False
        self.current_user_input = None

    def _on_customer_knowledge_saved(self, success: bool):
        """Callback for when customer knowledge update completes."""
        if success:
            self.get_logger().info("Customer knowledge successfully saved to database.")
        else:
            self.get_logger().error("Failed to save customer knowledge to database.")


def main(args=None):
    rclpy.init(args=args)

    try:
        node = ConversationNode(model_name="llama3")
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error starting conversation node: {e}", file=sys.stderr)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()