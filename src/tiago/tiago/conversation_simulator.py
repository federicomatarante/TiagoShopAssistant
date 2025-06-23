#!/usr/bin/env python3
import os
import sys
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tiago.msg import ConversationStatus


class ConversationSimulator(Node):
    def __init__(self):
        super().__init__('conversation_simulator')

        # State tracking
        self.conversation_active = False
        self.current_customer_id: Optional[str] = None

        # Setup ROS communication
        self._setup_ros_communication()

        # Start input thread
        self.input_thread = threading.Thread(target=self._input_loop, daemon=True)
        self.input_thread.start()

        self.get_logger().info("Conversation Simulator initialized")
        self._print_help()

    def _setup_ros_communication(self):
        """Setup ROS2 publishers and subscribers"""

        # Publisher to send controller commands
        self.controller_publisher = self.create_publisher(
            ConversationStatus,
            '/conversation/controller',
            10
        )

        # Publisher to simulate speech recognition input
        self.speech_publisher = self.create_publisher(
            String,
            '/speech/recognized_text',
            10
        )

        # Subscriber to receive TTS output (assistant responses)
        self.tts_subscriber = self.create_subscription(
            String,
            '/speech/tts_input',
            self.tts_callback,
            10
        )

        # NEW: Subscriber to monitor walk area commands
        self.walk_area_subscriber = self.create_subscription(
            String,
            '/navigation/walk_to_area',
            self.walk_area_callback,
            10
        )

        # NEW: Service client to test status service
        self.status_client = self.create_client(Trigger, '/conversation/status')

    def tts_callback(self, msg: String):
        """Handle TTS output from the conversation node"""
        print(f"\n🤖 Assistant: {msg.data}")
        if self.conversation_active:
            print("You: ", end="", flush=True)

    def walk_area_callback(self, msg: String):
        """Handle walk area commands from the conversation node"""
        print(f"\n🚶 Walk Command: Navigate to area '{msg.data}'")
        if self.conversation_active:
            print("You: ", end="", flush=True)

    def _print_help(self):
        """Print available commands"""
        print("\n" + "=" * 70)
        print("🎯 CONVERSATION SIMULATOR WITH MONITORING")
        print("=" * 70)
        print("Commands:")
        print("  /start <customer_id>  - Start conversation with customer")
        print("  /stop                 - Stop current conversation")
        print("  /status               - Check conversation node status")
        print("  /monitor              - Show monitoring info")
        print("  /help                 - Show this help")
        print("  /quit                 - Exit simulator")
        print("\nOnce conversation is started, type messages to chat!")
        print("\n📡 Monitoring:")
        print("  • TTS responses: /speech/tts_input")
        print("  • Walk commands: /navigation/walk_to_area")
        print("  • Status service: /conversation/status")
        print("=" * 70)

    def _input_loop(self):
        """Handle user input in a separate thread"""
        while True:
            try:
                if self.conversation_active:
                    user_input = input("You: ").strip()
                else:
                    user_input = input("Command: ").strip()

                if not user_input:
                    continue

                # Handle commands
                if user_input.startswith('/'):
                    self._handle_command(user_input)
                elif self.conversation_active:
                    # Send as speech input
                    self._send_speech_input(user_input)
                else:
                    print("❌ No active conversation. Use /start <customer_id> to begin.")

            except (EOFError, KeyboardInterrupt):
                print("\n👋 Exiting...")
                os._exit(0)

    def _handle_command(self, command: str):
        """Handle simulator commands"""
        parts = command.split()
        cmd = parts[0].lower()

        if cmd == '/help':
            self._print_help()

        elif cmd == '/quit':
            print("👋 Goodbye!")
            os._exit(0)

        elif cmd == '/start':
            if len(parts) < 2:
                print("❌ Usage: /start <customer_id>")
                return

            customer_id = parts[1]
            self._start_conversation(customer_id)

        elif cmd == '/stop':
            self._stop_conversation()

        elif cmd == '/status':
            self._check_status()

        elif cmd == '/monitor':
            self._show_monitoring_info()

        else:
            print(f"❌ Unknown command: {cmd}")
            print("Use /help to see available commands")

    def _check_status(self):
        """Check the status of the conversation node"""
        print("🔍 Checking conversation node status...")

        if not self.status_client.wait_for_service(timeout_sec=2.0):
            print("❌ Status service not available - conversation node may not be running")
            return

        try:
            request = Trigger.Request()
            future = self.status_client.call_async(request)

            # Wait for response (blocking in this thread is ok since it's the input thread)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

            if future.result() is not None:
                response = future.result()
                if response.success:
                    print(f"✅ {response.message}")
                else:
                    print(f"⚠️  Service responded but with failure: {response.message}")
            else:
                print("❌ Service call failed - no response received")

        except Exception as e:
            print(f"❌ Error calling status service: {e}")

    def _show_monitoring_info(self):
        """Show current monitoring information"""
        print("\n" + "=" * 50)
        print("📡 MONITORING INFO")
        print("=" * 50)
        print("Topics being monitored:")
        print("  🎤 Speech Input:     /speech/recognized_text")
        print("  🗣️  TTS Output:       /speech/tts_input")
        print("  🚶 Walk Commands:    /navigation/walk_to_area")
        print("  🎛️  Controller:       /conversation/controller")
        print("\nServices available:")
        print("  📊 Status Check:     /conversation/status")
        print("\nConversation State:")
        print(f"  Active: {self.conversation_active}")
        print(f"  Customer ID: {self.current_customer_id or 'None'}")
        print("=" * 50)

    def _start_conversation(self, customer_id: str):
        """Start a conversation with the given customer ID"""
        if self.conversation_active:
            print(f"❌ Conversation already active with customer {self.current_customer_id}")
            print("Use /stop to end current conversation first")
            return

        try:
            # Send start command
            msg = ConversationStatus()
            msg.command = 'start'
            msg.customer_id = customer_id

            self.controller_publisher.publish(msg)

            self.conversation_active = True
            self.current_customer_id = customer_id

            print(f"✅ Starting conversation with customer: {customer_id}")
            print("💬 You can now chat! Type your messages below.")
            print("   Use /stop to end the conversation")
            print("📡 Monitoring walk commands and TTS responses...")

        except Exception as e:
            self.get_logger().error(f"Failed to start conversation: {e}")
            print(f"❌ Failed to start conversation: {e}")

    def _stop_conversation(self):
        """Stop the current conversation"""
        if not self.conversation_active:
            print("❌ No active conversation to stop")
            return

        try:
            # Send stop command
            msg = ConversationStatus()
            msg.command = 'stop'
            msg.customer_id = self.current_customer_id or ''

            self.controller_publisher.publish(msg)

            print(f"✅ Stopping conversation with customer: {self.current_customer_id}")

            self.conversation_active = False
            self.current_customer_id = None

        except Exception as e:
            self.get_logger().error(f"Failed to stop conversation: {e}")
            print(f"❌ Failed to stop conversation: {e}")

    def _send_speech_input(self, text: str):
        """Send text as simulated speech input"""
        try:
            msg = String()
            msg.data = text
            self.speech_publisher.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Failed to send speech input: {e}")
            print(f"❌ Failed to send message: {e}")


def main(args=None):
    rclpy.init(args=args)

    try:
        simulator = ConversationSimulator()

        # Keep the node spinning
        rclpy.spin(simulator)

    except KeyboardInterrupt:
        print("\n👋 Shutting down...")
    except Exception as e:
        print(f"❌ Error running simulator: {e}", file=sys.stderr)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()