#!/usr/bin/env python3
import time
from datetime import datetime
from enum import Enum
import math
from typing import Dict

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from tf2_ros import Buffer, TransformListener

from geometry_msgs.msg import Point
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from tiago.srv import PathPlannerCommand, HRICommand, GetAreaPosition, UpdateProductPosition
from tiago.msg import VisionPersonDetection, VisionObjectDetection, ConversationStatus

# Assuming these are correct and provide necessary conversions
from tiago.lib.ros_utils.coordinates_system import bottom_left_to_center, center_to_bottom_left


class TiagoState(Enum):
    IDLE = "IDLE"
    CONVERSATION = "CONVERSATION"
    WALKING_TO_AREA = "WALKING_TO_AREA"
    APPROACHING_PERSON = "APPROACHING_PERSON"  # New state for clarity


class ReasoningNode(Node):
    def __init__(self):
        super().__init__('reasoning_node')
        self.state = TiagoState.IDLE
        self.current_person_id = None
        self.robot_position = Point()

        # Add retry logic for failed random walks
        self.random_walk_retry_timer = None
        self.random_walk_retry_count = 0
        self.max_random_walk_retries = 10
        self.waiting_for_path_completion = False  # Prevent multiple simultaneous path commands

        # Add listening behavior parameters
        self.listening_duration = 5.0  # seconds to listen before starting random walk
        self.listening_timer = None
        self.is_listening = False

        self.get_logger().info("Starting TiaGo Reasoning Node in IDLE state")

        # TF2 for robot position tracking
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.robot_frame = 'base_link'
        self.map_frame = 'map'

        self.map_width = 1069.0
        self.map_height = 726.0

        self.updated_objects: Dict[str, datetime] = {}  # Track updated objects with timestamps

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
        self.hri_client = self.create_client(HRICommand, '/conversation/controller')
        self.area_position_client = self.create_client(GetAreaPosition, 'get_area_position')
        self.update_product_position_client = self.create_client(
            UpdateProductPosition, 'update_product_position')

        # Wait for services - Blocking here is acceptable as it's during initialization
        self.get_logger().info("Waiting for required services...")
        while not self.path_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for path planner service...')
        while not self.hri_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for HRI service...')
        while not self.area_position_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for area position service...')
        while not self.update_product_position_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for update product position service...')

        self.get_logger().info("ALL SERVICES CONNECTED")

        # Subscribers
        self.path_status_sub = self.create_subscription(
            String, '/path_planner_status', self.on_path_status, self.qos_reliable)

        self.vision_person_sub = self.create_subscription(
            VisionPersonDetection, '/person_detection', self.on_person_detected, self.qos_sensor)

        self.vision_object_sub = self.create_subscription(
            VisionObjectDetection, '/object_detection', self.on_object_detected, self.qos_sensor)

        self.hri_status_sub = self.create_subscription(
            ConversationStatus, '/conversation/status', self.on_hri_status, self.qos_reliable)

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.on_odometry, self.qos_sensor)
        
        time.sleep(25)
        # Start initial idle behavior
        self.start_idle_behavior()

    def start_idle_behavior(self):
        """Start listening behavior when in IDLE state."""
        if self.state == TiagoState.IDLE and not self.waiting_for_path_completion:
            self.get_logger().info("Starting listening behavior in IDLE state")
            self.start_listening()

    def start_listening(self):
        """Start listening to surroundings for a specified duration."""
        if self.state != TiagoState.IDLE:
            return

        self.is_listening = True
        self.get_logger().info(f"Robot is now listening to surroundings for {self.listening_duration} seconds...")

        if self.listening_timer:
            self.listening_timer.cancel()

        self.listening_timer = self.create_timer(self.listening_duration, self.end_listening)

    def end_listening(self):
        """End listening period and start random walk if no person was detected."""
        if self.listening_timer:
            self.listening_timer.cancel()
            self.listening_timer = None

        self.is_listening = False

        if self.state == TiagoState.IDLE:
            self.get_logger().info(
                "Listening period ended, no person detected at medium distance. Starting random walk.")
            self.send_path_command_with_retry("random_walk")

    def send_path_command_with_retry(self, command, location=None):
        """Send path command with retry logic for random walks."""
        if self.waiting_for_path_completion:
            return

        if self.random_walk_retry_timer:
            self.random_walk_retry_timer.cancel()
            self.random_walk_retry_timer = None

        self.waiting_for_path_completion = True

        future = self._send_path_command_async(command, location)  # Use the async internal helper
        future.add_done_callback(
            lambda f: self._handle_path_command_sent_response(f, command, location)
        )

    def _handle_path_command_sent_response(self, future, command, location):
        """Handles the response from the PathPlannerCommand service call."""
        try:
            response = future.result()
            if response is not None and response.success:
                self.get_logger().info(f"Path command '{command}' sent successfully.")
                self.random_walk_retry_count = 0  # Reset retry count on success
                # self.waiting_for_path_completion remains True until /path_planner_status 'finished'
            else:
                self.get_logger().error(f"Path command '{command}' failed - response: {response}")
                self.waiting_for_path_completion = False
                self.handle_random_walk_failure()
        except Exception as e:
            self.get_logger().error(f"Path command '{command}' service call failed: {e}")
            self.waiting_for_path_completion = False
            self.handle_random_walk_failure()

    def handle_random_walk_failure(self):
        """Handle random walk failure with retry logic."""
        if self.state != TiagoState.IDLE:
            return

        self.random_walk_retry_count += 1

        if self.random_walk_retry_count <= self.max_random_walk_retries:
            retry_delay = min(5.0 * self.random_walk_retry_count, 30.0)
            self.get_logger().info(
                f"Retrying random walk in {retry_delay:.1f}s (attempt {self.random_walk_retry_count}/{self.max_random_walk_retries})")

            if self.random_walk_retry_timer:
                self.random_walk_retry_timer.cancel()

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

        except Exception:
            # Silence TF errors for brevity in logs if TF is not critical for every cycle
            pass

    def on_odometry(self, msg):
        """Fallback: update robot position from odometry if TF is not available."""
        if self.robot_position.x == 0.0 and self.robot_position.y == 0.0:
            self.robot_position.x = msg.pose.pose.position.x
            self.robot_position.y = msg.pose.pose.position.y
            self.robot_position.z = msg.pose.pose.position.z

    # Callbacks
    def on_path_status(self, msg):
        self.get_logger().info(f"Path status received: {msg.data}")
        if msg.data == "finished":
            self.waiting_for_path_completion = False

            if self.state == TiagoState.IDLE:
                self.get_logger().info("Random walk completed, starting listening behavior")
                self.create_timer(2.0, self.start_next_idle_cycle)
            elif self.state == TiagoState.WALKING_TO_AREA:
                self.get_logger().info("Reached area - resuming conversation")
                self._send_hri_command_async("resume_conversation")
                self.state = TiagoState.CONVERSATION
            elif self.state == TiagoState.APPROACHING_PERSON:
                self.get_logger().info("Reached approaching point - starting conversation")
                self._send_hri_command_async("start_conversation", self.current_person_id,
                                             "unknown")  # Assuming category is unknown
                self.state = TiagoState.CONVERSATION

    def start_next_idle_cycle(self):
        """Start the next idle cycle with listening (single-shot timer callback)."""
        if self.state == TiagoState.IDLE:
            self.get_logger().info("Starting next idle cycle with listening")
            self.start_listening()

    def on_person_detected(self, msg):
        distance = self.get_distance_class(msg.position)
        dx = msg.position.x - self.robot_position.x
        dy = msg.position.y - self.robot_position.y
        dz = msg.position.z - self.robot_position.z
        actual_distance = math.sqrt(dx * dx + dy * dy + dz * dz)

        self.get_logger().info(
            f"Person detected - ID: {msg.person_id}, Distance: {distance} ({actual_distance:.2f}m), "
            f"State: {self.state.value}, X:{msg.position.x}, Y:{msg.position.y}")

        if self.state == TiagoState.IDLE and distance == "medium":
            if self.is_listening:
                self.cancel_listening()

            self.get_logger().info(f"Person {msg.person_id} detected at medium distance, approaching.")

            if self.random_walk_retry_timer:
                self.random_walk_retry_timer.cancel()
                self.random_walk_retry_timer = None
                self.get_logger().info("Canceled random walk retry timer.")

            self.waiting_for_path_completion = False  # Ensure we can send new path commands

            # First, stop any current movement
            stop_future = self._send_path_command_async("stop_movement")
            # We don't strictly need to wait for stop_movement to complete here,
            # as the next path command should naturally override it.
            # If a strict sequence is needed, a done_callback could chain the next action.

            # Calculate approaching point
            location = self._get_approaching_point(msg.position)
            if location is None:
                self.get_logger().error("Failed to calculate approaching point, cannot proceed.")
                self.go_to_idle()
                return

            self.get_logger().info(f"Calculated approaching point: {location}")

            # Send go_to_location command
            go_to_future = self._send_path_command_async("go_to_location", location)
            go_to_future.add_done_callback(
                lambda f: self._handle_approach_person_sent_response(f, msg.person_id)
            )
            self.state = TiagoState.APPROACHING_PERSON  # Set new state
            self.current_person_id = msg.person_id  # Store the person being approached
            self.get_logger().info(f"Robot state changed to {self.state.value} to approach person {msg.person_id}.")


        elif self.state == TiagoState.IDLE and distance == "close":
            if self.is_listening:
                self.cancel_listening()

            self.get_logger().info(f"Person {msg.person_id} detected at close distance, starting conversation.")
            self._send_path_command_async("stop_movement")  # Stop any movement
            self._send_hri_command_async("start_conversation", msg.person_id, msg.cls)
            self.state = TiagoState.CONVERSATION
            self.current_person_id = msg.person_id

        elif self.state == TiagoState.CONVERSATION and distance == "far":
            if msg.person_id == self.current_person_id:  # Check if it's the person we are conversing with
                self.get_logger().info("Conversing person moved away - ending conversation.")
                self.go_to_idle()

    def _handle_approach_person_sent_response(self, future, person_id):
        """Handle the response after sending go_to_location for approaching a person."""
        try:
            response = future.result()
            if response is not None and response.success:
                self.get_logger().info(f"Go-to-location command sent successfully for approaching person {person_id}.")
                # The on_path_status callback will handle the transition to CONVERSATION when path completes
            else:
                self.get_logger().error(
                    f"Failed to send go-to-location command for approaching person {person_id}. Response: {response}")
                self.go_to_idle()  # Go back to idle if path command failed
        except Exception as e:
            self.get_logger().error(
                f"Error checking go-to-location command response for approaching person {person_id}: {e}")
            self.go_to_idle()

    def cancel_listening(self):
        """Cancel the listening behavior."""
        if self.listening_timer:
            self.listening_timer.cancel()
            self.listening_timer = None
        self.is_listening = False
        self.get_logger().info("Listening behavior canceled.")

    def on_object_detected(self, msg):
        if self.state == TiagoState.CONVERSATION:
            return

        MIN_TIME_BETWEEN_UPDATES = 5  # seconds

        if msg.object in self.updated_objects:
            last_update = self.updated_objects[msg.object]
            if (datetime.now() - last_update).total_seconds() < MIN_TIME_BETWEEN_UPDATES:
                return

        self.get_logger().info(
            f"Object detected - ID: {msg.object}, Position: ({msg.position.x}, {msg.position.y}, {msg.position.z}). "
            f"Updating position in database.")

        request = UpdateProductPosition.Request()
        request.product_id = msg.object
        request.position = msg.position

        future = self.update_product_position_client.call_async(request)
        future.add_done_callback(
            lambda f: self._handle_update_product_position_response(f, msg.object)
        )

    def _handle_update_product_position_response(self, future, object_id):
        """Callback to handle the response from UpdateProductPosition service."""
        try:
            result = future.result()
            if result is not None and result.success:
                self.get_logger().info(f"Object {object_id} position updated successfully.")
                self.updated_objects[object_id] = datetime.now()
            else:
                self.get_logger().error(f"Failed to update position for object {object_id}. Result: {result}")
        except Exception as e:
            self.get_logger().error(f"Service call for UpdateProductPosition failed: {e}")

    def on_hri_status(self, msg):
        self.get_logger().info(f"Current state: {self.state}")
        if self.state != TiagoState.CONVERSATION:
            return

        self.get_logger().info(f"Received conversation status: {msg.status}")

        if msg.status == "finished":
            self.get_logger().info("Conversation finished.")
            self.go_to_idle()

        elif msg.status == "walk_to_area":
            area_name_to_walk = msg.area
            self.get_logger().info(f"Received command to walk to area: {area_name_to_walk}.")

            future = self.area_position_client.call_async(GetAreaPosition.Request(area_name=area_name_to_walk))
            future.add_done_callback(
                lambda f: self._handle_get_area_location_response(f, area_name_to_walk)
            )
            # Temporarily pause conversation until we know if we can walk
            self._send_hri_command_async("pause_conversation")

    def _handle_get_area_location_response(self, future, area_name):
        """Callback to process the response from the GetAreaPosition service."""
        try:
            result = future.result()
            self.get_logger().info(f"Area position result for '{area_name}': {result}")

            if result is not None and result.success:
                location = result.position
                # Ensure coordinate conversion is correct for your map/robot setup
                # bottom_left_to_center takes (x, y) from bottom-left origin of map_width x map_height image
                # and converts to center-origin (ROS typically uses center-origin for map coordinates)
                new_x, new_y = bottom_left_to_center(location.x, location.y, self.map_width, self.map_height)
                target_point = Point(x=new_x, y=new_y, z=location.z)  # Keep Z if relevant

                self.get_logger().info(f"Target point for area '{area_name}': X={target_point.x}, Y={target_point.y}")

                self._send_hri_command_async("walking_to_area")

                path_future = self._send_path_command_async("go_to_location", target_point)
                path_future.add_done_callback(
                    lambda f: self._handle_go_to_area_sent_response(f, area_name)
                )
                self.state = TiagoState.WALKING_TO_AREA  # Update state
                self.get_logger().info(f"Robot state changed to {self.state.value} to walk to area {area_name}.")
            else:
                self.get_logger().error(
                    f"Failed to get location for area '{area_name}' or service returned success=False. Result: {result}")
                self._send_hri_command_async("unknown_location")
                self.go_to_idle()  # Revert to idle if area lookup fails

        except Exception as e:
            self.get_logger().error(f"Service call for GetAreaPosition failed during callback: {e}")
            self._send_hri_command_async("unknown_location")
            self.go_to_idle()

    def _handle_go_to_area_sent_response(self, future, area_name):
        """Callback to handle the response after sending go_to_location for an area."""
        try:
            response = future.result()
            if response is not None and response.success:
                self.get_logger().info(f"Go-to-location command sent successfully for area {area_name}.")
                # The on_path_status callback will handle the state transition when path completes
            else:
                self.get_logger().error(
                    f"Failed to send go-to-location command for area {area_name}. Response: {response}")
                self.go_to_idle()  # Go back to idle if path command failed
        except Exception as e:
            self.get_logger().error(f"Error checking go-to-location command response for area {area_name}: {e}")
            self.go_to_idle()

    # Helper methods
    def go_to_idle(self):
        """Transitions the robot to the IDLE state and starts listening."""
        self.get_logger().info("Transitioning to IDLE state.")
        self.cancel_listening()
        self._send_hri_command_async("stop_conversation")
        self.state = TiagoState.IDLE
        self.current_person_id = None
        self.start_listening()

    def get_distance_class(self, position):
        """Calculates distance from robot to a given position and returns a class."""
        dx = position.x - self.robot_position.x
        dy = position.y - self.robot_position.y
        dz = position.z - self.robot_position.z  # Include Z if meaningful for distance
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)

        if distance < 2.0:  # Close range
            return "close"
        elif distance < 3.0:  # Medium range (for approaching)
            return "medium"
        else:  # Far range
            return "far"

    def _get_approaching_point(self, person_position):
        """
        Calculates a point 1 meter in front of the person, relative to the robot.
        Assumes person_position and robot_position are in the same coordinate frame.
        """
        # Vector from robot to person
        vec_x = person_position.x - self.robot_position.x
        vec_y = person_position.y - self.robot_position.y
        vec_z = person_position.z - self.robot_position.z

        norm = math.sqrt(vec_x * vec_x + vec_y * vec_y + vec_z * vec_z)
        if norm == 0:
            self.get_logger().warn("Cannot calculate approaching point: Person at robot's exact location.")
            return None

        # Normalize the vector
        unit_vec_x = vec_x / norm
        unit_vec_y = vec_y / norm
        unit_vec_z = vec_z / norm

        # Move 1 meter *towards* the person from the robot's current position,
        # or *away* from the person if you want to stand 1m in front of them from their perspective.
        # The original code did: new_x = position.x - dx * approach_distance
        # This implies moving *away* from the person's current position by `approach_distance`
        # in the direction *from* robot *to* person. Let's stick to that for now.
        # If you want to move the robot *to* a point 1m in front of the person:
        # new_x = person_position.x - unit_vec_x * 1.0
        # new_y = person_position.y - unit_vec_y * 1.0
        # new_z = person_position.z - unit_vec_z * 1.0 # Or keep person's Z for a 2D approach

        # Following original logic: 1 meter in front of the person, relative to robot's perspective.
        # This means robot should go to (person_pos - (normalized_robot_to_person_vec * approach_distance))
        approach_distance = 1.0
        new_x = person_position.x - unit_vec_x * approach_distance
        new_y = person_position.y - unit_vec_y * approach_distance
        new_z = person_position.z - unit_vec_z * approach_distance

        return Point(x=new_x, y=new_y, z=new_z)

    # Internal helpers for sending service commands asynchronously
    # These are now private methods to ensure all calls go through the add_done_callback pattern
    def _send_path_command_async(self, command, location=None):
        request = PathPlannerCommand.Request()
        request.command = command
        request.location = location or Point()
        self.get_logger().info(f"Sending path command: '{command}' with location {location}")
        return self.path_client.call_async(request)