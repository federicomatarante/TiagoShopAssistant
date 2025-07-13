#!/usr/bin/env python3
import os
import sys
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tiago.msg import ConversationStatus  # Import ConversationStatus message for monitoring
from tiago.srv import HRICommand  # Import HRICommand service for sending commands


class ConversationSimulator(Node):
    def __init__(self):
        super().__init__('conversation_simulator')

        # State tracking for the simulator's internal representation
        self.conversation_active = False
        self.current_customer_id: Optional[str] = None

        # Setup ROS communication
        self._setup_ros_communication()

        # Start input thread to handle user commands and speech input
        self.input_thread = threading.Thread(target=self._input_loop, daemon=True)
        self.input_thread.start()

        self.get_logger().info("Conversation Simulator initialized and ready.")
        self._print_help()

    def _setup_ros_communication(self):
        """Sets up all ROS 2 publishers, subscribers, and service clients."""

        # Publisher for simulated speech recognition input (to ConversationNode)
        self.speech_publisher = self.create_publisher(
            String,
            '/speech/recognized_text',
            10
        )

        # Service client to send control commands to ConversationNode
        # The ConversationNode expects HRICommand service calls on this topic.
        self.controller_client = self.create_client(
            HRICommand,
            '/conversation/controller'
        )

        # Subscriber to receive TTS output from ConversationNode (assistant responses)
        self.tts_subscriber = self.create_subscription(
            String,
            '/speech/tts_input',
            self._tts_callback,
            10
        )

        # Subscriber to monitor walk area commands from ConversationNode
        # Note: ConversationNode now publishes ConversationStatus with area field for walk commands.
        # This subscriber needs to be updated to listen to /conversation/status and extract the area.
        self.conversation_status_subscriber = self.create_subscription(
            ConversationStatus,
            '/conversation/status',
            self._conversation_status_callback,
            10
        )


        # Service client to check if ConversationNode is online
        self.status_client = self.create_client(Trigger, '/conversation/status')


    def _tts_callback(self, msg: String):
        """Handles text-to-speech output received from the ConversationNode."""
        print(f"\n🤖 Assistant: {msg.data}")
        if self.conversation_active:
            print("You: ", end="", flush=True) # Prompt for next user input

    def _conversation_status_callback(self, msg: ConversationStatus):
        """Handles conversation status updates, including walk commands."""
        if msg.status == "walk_to_area" and msg.area:
            print(f"\n🚶 Walk Command: Navigate to area '{msg.area}'")
        elif msg.status == "finished":
            print("\n✅ ConversationNode reported conversation finished.")
            self.conversation_active = False
            self.current_customer_id = None
            print("Command: ", end="", flush=True) # Return to command prompt
        else:
            self.get_logger().info(f"Received ConversationNode status: {msg.status} (Area: {msg.area})")

        if self.conversation_active:
            print("You: ", end="", flush=True) # Prompt for next user input


    def _print_help(self):
        """Displays the list of available commands and monitoring information."""
        print("\n" + "=" * 70)
        print("🎯 CONVERSATION SIMULATOR (Compatible with your ConversationNode)")
        print("=" * 70)
        print("Commands:")
        print("  /start <customer_id> [category] - Start a new conversation. Category defaults to 'customer'.")
        print("  /stop                          - Stop the current conversation.")
        print("  /pause                         - Pause the current conversation.")
        print("  /resume                        - Resume a paused conversation.")
        print("  /status                        - Check if ConversationNode is online.")
        print("  /monitor                       - Show current monitoring information.")
        print("  /help                          - Display this help message.")
        print("  /quit                          - Exit the simulator.")
        print("\nOnce a conversation is started, simply type your messages to chat!")
        print("\n📡 Monitoring (Simulator receives):")
        print("  • TTS responses:         /speech/tts_input")
        print("  • Conversation Status:   /conversation/status (includes walk commands)")
        print("\n💬 Commands (Simulator sends):")
        print("  • Speech Input:          /speech/recognized_text")
        print("\n⚙️ Services (Simulator uses to send commands):")
        print("  • Control Commands:      /conversation/controller (HRICommand service)")
        print("  • Conversation Status:   /conversation/status (Trigger service)")
        print("=" * 70)

    def _input_loop(self):
        """Manages user input in a separate thread, handling commands or speech."""
        while True:
            try:
                # Prompt depends on whether a conversation is active
                if self.conversation_active:
                    user_input = input("You: ").strip()
                else:
                    user_input = input("Command: ").strip()

                if not user_input:
                    continue # Ignore empty inputs

                # Check if the input is a simulator command (starts with '/')
                if user_input.startswith('/'):
                    self._handle_simulator_command(user_input)
                # Otherwise, if a conversation is active, send it as speech input
                elif self.conversation_active:
                    self._send_speech_input(user_input)
                # If not a command and no conversation is active, inform the user
                else:
                    print("❌ No active conversation. Use /start <customer_id> to begin.")

            except (EOFError, KeyboardInterrupt):
                print("\n👋 Exiting Conversation Simulator...")
                os._exit(0) # Force exit cleanly for thread safety

    def _handle_simulator_command(self, command_string: str):
        """Processes internal simulator commands."""
        parts = command_string.split()
        cmd = parts[0].lower()

        if cmd == '/help':
            self._print_help()

        elif cmd == '/quit':
            print("👋 Goodbye!")
            os._exit(0)

        elif cmd == '/start':
            if len(parts) < 2:
                print("❌ Usage: /start <customer_id> [category]")
                return
            customer_id = parts[1]
            category = parts[2] if len(parts) > 2 else "customer" # Default category
            self._send_control_command('start_conversation', customer_id, category)

        elif cmd == '/stop':
            self._send_control_command('stop_conversation')

        elif cmd == '/pause':
            self._send_control_command('pause_conversation')

        elif cmd == '/resume':
            self._send_control_command('resume_conversation')

        elif cmd == '/status':
            self._check_conversation_node_status()

        elif cmd == '/monitor':
            self._show_monitoring_info()

        else:
            print(f"❌ Unknown command: {cmd}. Use /help to see available commands.")

    def _send_control_command(self, command_type: str, person_id: str = "none", category: str = "none"):
        """
        Sends a control command (start/stop/pause/resume) to the ConversationNode via the
        /conversation/controller service using an HRICommand message.
        """
        if not self.controller_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f"ConversationNode's /conversation/controller service not available for command '{command_type}'.")
            print(f"❌ ConversationNode's /conversation/controller service not available. Is the node running?")
            return

        request = HRICommand.Request()
        request.command = command_type
        request.person_id = person_id
        request.category = category

        self.get_logger().info(f"Sending control command: '{command_type}' for person '{person_id}' (category: '{category}')")
        future = self.controller_client.call_async(request)

        # Spin until the future is complete (blocking the input thread briefly)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            response = future.result()
            if response.success:
                print(f"✅ Command '{command_type}' sent successfully to ConversationNode.")
                # Update simulator's internal state based on the command sent
                if command_type == 'start_conversation':
                    self.conversation_active = True
                    self.current_customer_id = person_id
                    print(f"💬 Conversation started with customer: {person_id}")
                    print("   Now, type your messages to chat!")
                elif command_type == 'stop_conversation':
                    self.conversation_active = False
                    self.current_customer_id = None
                    print("💬 Conversation stop command sent. Simulator conversation ended.")
                    print("Command: ", end="", flush=True) # Return to command prompt
                elif command_type == 'pause_conversation':
                    print("⏸️ Conversation pause command sent.")
                elif command_type == 'resume_conversation':
                    print("▶️ Conversation resume command sent.")
            else:
                print(f"❌ ConversationNode failed to execute command '{command_type}': {response.message}")
        else:
            print(f"❌ Service call for '{command_type}' failed: No response received from ConversationNode.")


    def _check_conversation_node_status(self):
        """Calls the /conversation/status service on ConversationNode."""
        print("🔍 Checking ConversationNode status via /conversation/status service...")

        # Wait for the service to be available
        if not self.status_client.wait_for_service(timeout_sec=2.0):
            print("❌ ConversationNode's /conversation/status service not available. Is the node running?")
            return

        try:
            request = Trigger.Request()
            future = self.status_client.call_async(request)

            # Spin until the future is complete (blocking the input thread briefly)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

            if future.result() is not None:
                response = future.result()
                if response.success:
                    print(f"✅ ConversationNode Status: {response.message}")
                else:
                    print(f"⚠️  ConversationNode reported an issue: {response.message}")
            else:
                print("❌ Service call failed: No response received from ConversationNode.")

        except Exception as e:
            self.get_logger().error(f"Error calling /conversation/status service: {e}")
            print(f"❌ An error occurred while checking status: {e}")

    def _show_monitoring_info(self):
        """Displays current monitoring and communication setup information."""
        print("\n" + "=" * 50)
        print("📡 MONITORING & COMMUNICATION SETUP")
        print("=" * 50)
        print("Topics this Simulator listens to (Receives from ConversationNode):")
        print("  • TTS Output:            /speech/tts_input (std_msgs/String)")
        print("  • Conversation Status:   /conversation/status (tiago/ConversationStatus) - includes walk commands")
        print("\nTopics this Simulator publishes to (Sends to ConversationNode):")
        print("  • Speech Input:          /speech/recognized_text (std_msgs/String)")
        print("\nServices this Simulator calls (on ConversationNode):")
        print("  • Control Commands:      /conversation/controller (tiago/HRICommand service)")
        print("  • Node Status Check:     /conversation/status (std_srvs/Trigger service)")
        print("\nSimulator's Current Internal State:")
        print(f"  • Conversation Active:   {self.conversation_active}")
        print(f"  • Current Customer ID:   {self.current_customer_id or 'None'}")
        print("=" * 50)

    def _send_speech_input(self, text: str):
        """Publishes user-typed text as simulated speech input."""
        try:
            msg = String()
            msg.data = text
            self.speech_publisher.publish(msg)
            # self.get_logger().debug(f"Sent speech input: {text}") # Uncomment for verbose logging

        except Exception as e:
            self.get_logger().error(f"Failed to send speech input: {e}")
            print(f"❌ Failed to send message: {e}")


def main(args=None):
    rclpy.init(args=args)

    try:
        simulator = ConversationSimulator()
        rclpy.spin(simulator) # Keep the node spinning to process callbacks

    except KeyboardInterrupt:
        print("\n👋 Shutting down Conversation Simulator...")
    except Exception as e:
        print(f"❌ An unexpected error occurred: {e}", file=sys.stderr)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()