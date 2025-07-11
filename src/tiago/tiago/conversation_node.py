#!/usr/bin/env python3
import os
from typing import Optional, Tuple, List, Dict, Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from tiago.srv import HRICommand  # Use the HRICommand service
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
        """Setup ROS2 publishers and subscribers and services."""

        # HRI Command service server (replaces controller_subscriber)
        self.hri_command_service = self.create_service(
            HRICommand,
            'hri_command',  # This node will provide the HRICommand service
            self.hri_command_callback
        )

        # Publisher for HRI conversation status (to ReasoningNode)
        self.hri_status_publisher = self.create_publisher(
            ConversationStatus,
            'hri_conversation_status',
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

    def hri_command_callback(self, request: HRICommand.Request, response: HRICommand.Response):
        """Handle HRI commands from the ReasoningNode."""
        self.get_logger().info(f"Received HRI command: {request.command}")
        try:
            command = request.command

            if command == 'start_conversation':
                customer_id = request.person_id
                if not customer_id:
                    self.get_logger().warn("Received 'start_conversation' command with no person ID.")
                    response.success = False
                    response.message = "No person ID provided."
                    return response
                self._start_conversation(customer_id)
                response.success = True
                response.message = "Conversation initiated."
            elif command == 'stop_conversation':
                self._finalize_conversation()
                response.success = True
                response.message = "Conversation stopped."
            elif command == 'pause_conversation':
                # For now, pausing simply means stopping speech input processing
                # More sophisticated pausing might involve saving LLM state.
                self.conversation_active = False
                response.success = True
                response.message = "Conversation paused."
            elif command == 'resume_conversation':
                # Resume conversation after movement
                self.conversation_active = True
                # Publish a general prompt for the user to continue the conversation
                self._publish_assistant_reply("Hello again! How can I help you now?")
                response.success = True
                response.message = "Conversation resumed."
            else:
                self.get_logger().warn(f"Unknown HRI command: {command}")
                response.success = False
                response.message = f"Unknown command: {command}"

        except Exception as e:
            self.get_logger().error(f"Error processing HRI command: {e}")
            response.success = False
            response.message = f"Error processing command: {e}"
        return response

    def speech_input_callback(self, msg: String):
        """Handle speech recognition input."""
        if not self.conversation_active or not self.current_assistant:
            self.get_logger().warn("Received speech input but no active conversation.")
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
            self._publish_assistant_reply(error_msg)

    def _process_user_input_async(self, user_input: str):
        self.current_assistant.add_message(user_input)
        direction = self.current_assistant.get_conversation_direction()
        products_query = self.current_assistant.detect_products_query()
        staff_query = self.current_assistant.detect_staff_query()
        self.get_logger().info(f"Detected direction: {direction}")

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
            self._publish_hri_status("finished") # Notify ReasoningNode that conversation is finished
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

    def _publish_hri_status(self, status: str, area: str = ""):
        """Helper to publish HRI conversation status."""
        status_msg = ConversationStatus()
        status_msg.status = status
        status_msg.area = area
        self.hri_status_publisher.publish(status_msg)
        self.get_logger().info(f"Published HRI status: {status_msg.status} (Area: {status_msg.area})")

    # --- Callbacks for DatabaseServiceClient responses ---

    def _handle_product_walk_response(self, products: List, areas: Optional[List[str]], original_user_input: str):
        """Handles response for 'walk_to_product' query."""
        if len(products) > 0 and areas and len(areas) > 0:
            product = products[0]
            self.get_logger().info(f"Found product for walk: {product.name}")
            results = {'products': [product.to_json(), ]}
            results['areas'] = areas # Always add areas if available

            assistant_reply = self.current_assistant.answer_with_results(
                results,
                option="Say the customer you're going to walk him to the product"
            )
            self._publish_assistant_reply(assistant_reply)
            # Notify ReasoningNode to walk to the area
            self._publish_hri_status("walk_to_area", areas[0])
        else:
            self.get_logger().warn(f"Could not find product or area for walk.")
            assistant_reply = self.current_assistant.answer(
                option="Say the customer you couldn't find the product so you cannot walk him to it"
            )
            self._publish_assistant_reply(assistant_reply)

    def _handle_staff_walk_response(self, staff: List, areas: Optional[List[str]], original_user_input: str):
        """Handles response for 'walk_to_staff' query."""
        if len(staff) > 0 and areas and len(areas) > 0:
            staff_member = staff[0]
            self.get_logger().info(f"Found staff member for walk: {staff_member.name}")
            results = {'staff': [staff_member.to_json(), ]}
            results['areas'] = areas

            assistant_reply = self.current_assistant.answer_with_results(
                results,
                option="Say the customer you're going to walk him to the staff member"
            )
            self._publish_assistant_reply(assistant_reply)
            # Notify ReasoningNode to walk to the area
            self._publish_hri_status("walk_to_area", areas[0])
        else:
            self.get_logger().warn(f"Could not find staff member or area for walk.")
            assistant_reply = self.current_assistant.answer(
                option="Say the customer you couldn't find the staff member so you cannot walk him to it"
            )
            self._publish_assistant_reply(assistant_reply)

    def _handle_product_info_response(self, products: List, areas: Optional[List[str]], original_user_input: str):
        """Handles response for 'product_info' query."""
        if len(products) > 0:
            self.get_logger().info(f"Found product(s) for info: {[p.name for p in products]}")
            results = {'products': [product.to_json() for product in products]}
            if areas: # Include areas in results if available
                results['areas'] = areas

            assistant_reply = self.current_assistant.answer_with_results(
                results,
                option="Say the customer you found the product information"
            )
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
            if areas: # Include areas in results if available
                results['areas'] = areas

            assistant_reply = self.current_assistant.answer_with_results(
                results,
                option="Say the customer you found the staff information"
            )
        else:
            self.get_logger().warn(f"Could not find staff member for info.")
            assistant_reply = self.current_assistant.answer(
                option="Say the customer you couldn't find the staff information"
            )
        self._publish_assistant_reply(assistant_reply)

    def _start_conversation(self, customer_id: str):
        """Start a new conversation with a customer."""
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
        """Finalize and clean up the conversation."""
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