#!/usr/bin/env python3
import time
from enum import Enum
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs

from geometry_msgs.msg import Point
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from tiago.srv import PathPlannerCommand, HRICommand
from tiago.msg import VisionPersonDetection, ConversationStatus


class TiagoState(Enum):
    IDLE = "IDLE"
    CONVERSATION = "CONVERSATION"
    WALKING_TO_AREA = "WALKING_TO_AREA"
    APPROACHING_PERSON = "APPROACHING_PERSON"  # New state for approaching medium distance person


class ReasoningNode(Node):
    def __init__(self):
        super().__init__('reasoning_node')
        self.state = TiagoState.IDLE  # Start in IDLE state
        self.current_person_id = None
        self.robot_position = Point()
        self.current_path_status = None  # Track current path planner status
        self.approaching_person_id = None  # Track which person we're approaching
        
        # Add retry logic for failed random walks
        self.random_walk_retry_timer = None
        self.random_walk_retry_count = 0
        self.max_random_walk_retries = 10
        self.waiting_for_path_completion = False  # Prevent multiple simultaneous path commands

        self.get_logger().info("Starting TiaGo Reasoning Node in IDLE state")

        # TF2 for robot position tracking
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.robot_frame = 'base_link'
        self.map_frame = 'map'

        # Timer to update robot position
        self.position_timer = self.create_timer(0.5, self.update_robot_position)

        # QoS profiles
        self.qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Service clients
        self.path_client = self.create_client(PathPlannerCommand, 'path_planner_command')
        self.hri_client = self.create_client(HRICommand, 'hri_command')

        # Wait for services
        while not self.path_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for path planner service...')
        # while not self.hri_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Waiting for HRI service...')

        # Subscribers
        self.path_status_sub = self.create_subscription(
            String, 'path_planner_status', self.on_path_status, self.qos_reliable)

        self.vision_sub = self.create_subscription(
            VisionPersonDetection, '/person_detection', self.on_person_detected, self.qos_sensor)

        self.hri_status_sub = self.create_subscription(
            ConversationStatus, '/hri_conversation_status', self.on_hri_status, self.qos_reliable)

        # Alternative: Subscribe to odometry if TF is not available
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.on_odometry, self.qos_sensor)

        # Start random walk behavior in IDLE state (with delay to let services initialize)
        self.create_timer(2.0, self.start_idle_behavior)

    def start_idle_behavior(self):
        """Start random walk behavior when in IDLE state."""
        if self.state == TiagoState.IDLE and not self.waiting_for_path_completion:
            self.get_logger().info("Starting random walk behavior in IDLE state")
            self.send_path_command_with_retry("random_walk")

    def send_path_command_with_retry(self, command, location=None):
        """Send path command with retry logic for random walks."""
        # Check if we're already waiting for a path completion
        if self.waiting_for_path_completion:
            # self.get_logger().info("Already waiting for path completion, skipping duplicate request")
            return
            
        # Cancel any existing retry timer
        if self.random_walk_retry_timer:
            self.random_walk_retry_timer.cancel()
            self.random_walk_retry_timer = None
        
        self.waiting_for_path_completion = True  # Mark that we're waiting
        
        future = self.send_path_command(command, location)
        
        def handle_response(future):
            try:
                response = future.result()
                if response is not None and response.success:
                    self.get_logger().info(f"Path command '{command}' sent successfully")
                    self.random_walk_retry_count = 0  # Reset retry count on success
                    # Don't reset waiting_for_path_completion here - wait for path status callback
                else:
                    self.get_logger().error(f"Path command '{command}' failed - response: {response}")
                    self.waiting_for_path_completion = False  # Reset waiting flag
                    self.handle_random_walk_failure()
            except Exception as e:
                self.get_logger().error(f"Path command '{command}' service call failed: {e}")
                self.waiting_for_path_completion = False  # Reset waiting flag
                self.handle_random_walk_failure()
        
        future.add_done_callback(handle_response)

    def handle_random_walk_failure(self):
        """Handle random walk failure with retry logic."""
        if self.state != TiagoState.IDLE:
            return
            
        self.random_walk_retry_count += 1
        
        if self.random_walk_retry_count <= self.max_random_walk_retries:
            retry_delay = min(5.0 * self.random_walk_retry_count, 30.0)  # Exponential backoff, max 30s
            self.get_logger().info(f"Retrying random walk in {retry_delay}s (attempt {self.random_walk_retry_count}/{self.max_random_walk_retries})")
            
            # Cancel existing timer if any
            if self.random_walk_retry_timer:
                self.random_walk_retry_timer.cancel()
            
            # Create new retry timer
            self.random_walk_retry_timer = self.create_timer(retry_delay, self.retry_random_walk)
        else:
            self.get_logger().error("Max random walk retries reached. Stopping retry attempts.")
            self.random_walk_retry_count = 0

    def retry_random_walk(self):
        """Retry random walk (single-shot timer callback)."""
        if self.random_walk_retry_timer:
            self.random_walk_retry_timer.cancel()
            self.random_walk_retry_timer = None
            
        if self.state == TiagoState.IDLE:
            self.get_logger().info("Retrying random walk...")
            self.send_path_command_with_retry("random_walk")

    def update_robot_position(self):
        """Update robot position using TF2."""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, rclpy.time.Time())
            
            self.robot_position.x = transform.transform.translation.x
            self.robot_position.y = transform.transform.translation.y
            self.robot_position.z = transform.transform.translation.z
            
        except Exception as e:
            # Don't spam the logs with TF errors
            pass

    def on_odometry(self, msg):
        """Fallback: update robot position from odometry if TF is not available."""
        if self.robot_position.x == 0.0 and self.robot_position.y == 0.0:
            # Only use odometry if TF is not working
            self.robot_position.x = msg.pose.pose.position.x
            self.robot_position.y = msg.pose.pose.position.y
            self.robot_position.z = msg.pose.pose.position.z

    def approach_person(self, person_position, person_id):
        """Approach a person at medium distance."""
        self.get_logger().info(f"Approaching person {person_id} at position ({person_position.x:.2f}, {person_position.y:.2f})")
        
        # First stop current movement
        stop_future = self.send_path_command("stop_movement")
        
        def handle_stop_response(future):
            try:
                stop_response = future.result()
                if stop_response and stop_response.success:
                    self.get_logger().info("Movement stopped, now approaching person")
                    # Now send go_to_location command
                    approach_future = self.send_path_command("go_to_location", person_position)
                    
                    def handle_approach_response(future):
                        try:
                            approach_response = future.result()
                            if approach_response and approach_response.success:
                                self.get_logger().info("Successfully started approaching person")
                                self.state = TiagoState.APPROACHING_PERSON
                                self.approaching_person_id = person_id
                            else:
                                self.get_logger().error("Failed to start approaching person")
                                # Return to idle state and resume random walk
                                self.state = TiagoState.IDLE
                                self.send_path_command_with_retry("random_walk")
                        except Exception as e:
                            self.get_logger().error(f"Approach command failed: {e}")
                            self.state = TiagoState.IDLE
                            self.send_path_command_with_retry("random_walk")
                    
                    approach_future.add_done_callback(handle_approach_response)
                else:
                    self.get_logger().error("Failed to stop movement")
                    # Continue with current behavior
            except Exception as e:
                self.get_logger().error(f"Stop command failed: {e}")
                
        stop_future.add_done_callback(handle_stop_response)

    # Callbacks
    def on_path_status(self, msg):
        # Track current path status
        self.current_path_status = msg.data
        
        if msg.data == "finished":
            self.waiting_for_path_completion = False  # Reset waiting flag
            
            if self.state == TiagoState.IDLE:
                # When a random walk finishes in IDLE state, start another one
                self.get_logger().info("Random walk completed, starting new random walk")
                # Add small delay before starting next random walk
                self.create_timer(2.0, self.start_next_random_walk)
            elif self.state == TiagoState.WALKING_TO_AREA:
                self.get_logger().info("Reached area - resuming conversation")
                self.send_hri_command("resume_conversation")
                self.state = TiagoState.CONVERSATION
            elif self.state == TiagoState.APPROACHING_PERSON:
                self.get_logger().info("Reached person - checking if still close")
                # The person detection callback will handle transition to conversation
                # if the person is still close, or back to idle if they moved away
                pass

    def start_next_random_walk(self):
        """Start the next random walk (single-shot timer callback)."""
        if self.state == TiagoState.IDLE:
            self.get_logger().info("Starting next random walk")
            self.send_path_command_with_retry("random_walk")

    def is_person(self, person_id):
        """Check if the detected entity is a person (staff or customer) and not an object."""
        if not person_id:
            return False
        person_id_lower = person_id.lower()
        return "staff" in person_id_lower or "customer" in person_id_lower

    def on_person_detected(self, msg):
        # Only process if this is actually a person (not an object)
        if not self.is_person(msg.person_id):
            return
            
        # Calculate distance for all states
        distance = self.get_distance_class(msg.position)
        
        # Calculate actual numerical distance
        dx = msg.position.x - self.robot_position.x
        dy = msg.position.y - self.robot_position.y
        dz = msg.position.z - self.robot_position.z
        actual_distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        
        # Print distance and ID for all detected people, regardless of state
        # self.get_logger().info(f"Person detected - ID: {msg.person_id}, Distance: {distance} ({actual_distance:.2f}m), State: {self.state.value}")

        if self.state == TiagoState.IDLE:
            if distance == "close":
                self.get_logger().info(f"Starting conversation with person {msg.person_id}")
                self.send_path_command("stop_movement")
                self.send_hri_command("start_conversation", msg.person_id, msg.cls)
                self.state = TiagoState.CONVERSATION
                self.current_person_id = msg.person_id
            elif distance == "medium":
                self.get_logger().info(f"Person {msg.person_id} detected at medium distance, stopping random walk and approaching...")
                self.approach_person(msg.position, msg.person_id)

        elif self.state == TiagoState.CONVERSATION and distance == "far":
            if msg.person_id == self.current_person_id:
                self.get_logger().info("Person moved away - ending conversation")
                self.go_to_idle()
                
        elif self.state == TiagoState.APPROACHING_PERSON:
            if msg.person_id == self.approaching_person_id:
                if distance == "close":
                    self.get_logger().info(f"Reached person {msg.person_id}, starting conversation")
                    self.send_path_command("stop_movement")
                    self.send_hri_command("start_conversation", msg.person_id, msg.cls)
                    self.state = TiagoState.CONVERSATION
                    self.current_person_id = msg.person_id
                    self.approaching_person_id = None
                elif distance == "far":
                    self.get_logger().info(f"Person {msg.person_id} moved away while approaching, returning to idle")
                    self.go_to_idle()
                    self.approaching_person_id = None

    def on_hri_status(self, msg):
        if self.state != TiagoState.CONVERSATION:
            return

        if msg.status == "finished":
            self.get_logger().info("Conversation finished")
            self.go_to_idle()

        elif msg.status == "walk_to_area":
            self.get_logger().info(f"Walking to area: {msg.area}")
            self.send_hri_command("pause_conversation")
            location = self.get_area_location(msg.area)
            self.send_path_command("go_to_location", location)
            self.state = TiagoState.WALKING_TO_AREA

    # Helper methods
    def go_to_idle(self):
        self.send_hri_command("stop_conversation")
        self.send_path_command_with_retry("random_walk")
        self.state = TiagoState.IDLE
        self.current_person_id = None

    def get_distance_class(self, position):
        # Now uses updated robot position
        dx = position.x - self.robot_position.x
        dy = position.y - self.robot_position.y
        dz = position.z - self.robot_position.z
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)

        if distance < 1.5:
            return "close"
        elif distance < 3.0:
            return "medium"
        else:
            return "far"

    def get_area_location(self, area):
        # Simple area mapping
        areas = {
            "customer_service_desk": Point(x=5.0, y=2.0, z=0.0),
            "information_booth": Point(x=10.0, y=5.0, z=0.0),
            "exit": Point(x=0.0, y=0.0, z=0.0)
        }
        return areas.get(area, Point())

    # Service calls (simple async)
    def send_path_command(self, command, location=None):
        request = PathPlannerCommand.Request()
        request.command = command
        request.location = location or Point()
        future = self.path_client.call_async(request)
        return future

    def send_hri_command(self, command, person_id="", category=""):
        request = HRICommand.Request()
        request.command = command
        request.person_id = person_id
        request.category = category
        future = self.hri_client.call_async(request)
        return future


def main(args=None):
    rclpy.init(args=args)
    node = ReasoningNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()