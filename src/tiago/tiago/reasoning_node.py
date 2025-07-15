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

from tiago.lib.ros_utils.coordinates_system import bottom_left_to_center, center_to_bottom_left


class TiagoState(Enum):
    IDLE = "IDLE"
    CONVERSATION = "CONVERSATION"
    WALKING_TO_AREA = "WALKING_TO_AREA"


class ReasoningNode(Node):
    def __init__(self):
        super().__init__('reasoning_node')
        self.state = TiagoState.IDLE  # Start in IDLE state
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

        self.updated_objects: Dict[str,datetime] = {}  # Track updated objects with timestamps

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
        # Wait for services
        while not self.path_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for path planner service...')

        while not self.hri_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for HRI service...')

        while not self.area_position_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for area position service...')

        self.get_logger().info("ALL SERVICES TU MADRE")

        # Subscribers
        self.path_status_sub = self.create_subscription(
            String, '/path_planner_status', self.on_path_status, self.qos_reliable)

        self.vision_person_sub = self.create_subscription(
            VisionPersonDetection, '/person_detection', self.on_person_detected, self.qos_sensor)

        self.vision_object_sub = self.create_subscription(
            VisionObjectDetection, '/object_detection', self.on_object_detected, self.qos_sensor)

        self.hri_status_sub = self.create_subscription(
            ConversationStatus, '/conversation/status', self.on_hri_status, self.qos_reliable)

        # Alternative: Subscribe to odometry if TF is not available
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.on_odometry, self.qos_sensor)

        # Start listening behavior in IDLE state (with delay to let services initialize)
        time.sleep(25)
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
        
        # Cancel any existing listening timer
        if self.listening_timer:
            self.listening_timer.cancel()
            
        # Set timer to end listening period
        self.listening_timer = self.create_timer(self.listening_duration, self.end_listening)

    def end_listening(self):
        """End listening period and start random walk if no person was detected."""
        if self.listening_timer:
            self.listening_timer.cancel()
            self.listening_timer = None
            
        self.is_listening = False
        
        if self.state == TiagoState.IDLE:
            self.get_logger().info("Listening period ended, no person detected at medium distance. Starting random walk.")
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
            self.get_logger().info(
                f"Retrying random walk in {retry_delay}s (attempt {self.random_walk_retry_count}/{self.max_random_walk_retries})")

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
        """Update robot position using TF2.
            The robot position should be relative to the map frame, with the origin at the image center.
        """
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
        """Fallback: update robot position from odometry if TF is not available.
            The odometry position should also be relative to the map frame, with the origin at the image center.
        """
        if self.robot_position.x == 0.0 and self.robot_position.y == 0.0:
            # Only use odometry if TF is not working
            self.robot_position.x = msg.pose.pose.position.x
            self.robot_position.y = msg.pose.pose.position.y
            self.robot_position.z = msg.pose.pose.position.z

    # Callbacks
    def on_path_status(self, msg):
        self.get_logger().info(f"Path status received:{msg.data}")
        if msg.data == "finished":
            self.waiting_for_path_completion = False  # Reset waiting flag

            if self.state == TiagoState.IDLE:
                # When a random walk finishes in IDLE state, start listening again
                self.get_logger().info("Random walk completed, starting listening behavior")
                # Add small delay before starting listening
                self.create_timer(2.0, self.start_next_idle_cycle)
            elif self.state == TiagoState.WALKING_TO_AREA:
                self.get_logger().info("Reached area - resuming conversation")
                self.send_hri_command("resume_conversation")
                self.state = TiagoState.CONVERSATION

    def start_next_idle_cycle(self):
        """Start the next idle cycle with listening (single-shot timer callback)."""
        if self.state == TiagoState.IDLE:
            self.get_logger().info("Starting next idle cycle with listening")
            self.start_listening()

    def on_person_detected(self, msg):
        # Calculate distance for all states
        # TODO converting msg.position to right coordinate system?

        distance = self.get_distance_class(msg.position)

        # Calculate actual numerical distance
        dx = msg.position.x - self.robot_position.x
        dy = msg.position.y - self.robot_position.y
        dz = msg.position.z - self.robot_position.z
        actual_distance = math.sqrt(dx * dx + dy * dy + dz * dz)

        # Print distance and ID for all detected people, regardless of state
        self.get_logger().info(
            f"Person detected - ID: {msg.person_id}, Distance: {distance} ({actual_distance:.2f}m), State: {self.state.value}, X:{msg.position.x}, Y:{msg.position.y}")

            
        if self.state == TiagoState.IDLE and distance == "medium":
            # Cancel listening if active
            if self.is_listening:
                self.cancel_listening()
                
            self.get_logger().info(f"Person {msg.person_id} detected at medium distance during {'listening' if self.is_listening else 'idle'}, approaching")
            
            # 1. Immediately stop any current random walk retries
            if self.random_walk_retry_timer:
                self.random_walk_retry_timer.cancel()
                self.random_walk_retry_timer = None
                self.get_logger().info("Canceled random walk retry timer.")
            
            # 2. Reset the waiting_for_path_completion flag
            self.waiting_for_path_completion = False 
            self.get_logger().info("Reset waiting_for_path_completion flag.")

            # 3. Send stop movement command
            stop_result = self.send_path_command("stop_movement", sync=False)
            # if not stop_result.success:
            #     self.get_logger().error("Failed to stop movement, cannot proceed with approaching.")
            #     return

            # self.get_logger().info("Stop movement command successful.")

            # 4. Calculate approaching point
            location = self._get_approaching_point(msg.position)
            if location is None:
                self.get_logger().error("Failed to calculate approaching point, cannot proceed.")
                return
            
            # 5. Send go_to_location command
            go_to_result = self.send_path_command("go_to_location", location, sync=False)
            rclpy.spin_until_future_complete(self, go_to_result, timeout_sec=10.0)
            # Check if the command was successful
            if go_to_result.result() is None:
                self.get_logger().error("Failed to get result for go_to_location command.")
                return
            if go_to_result.result().success:
                self.get_logger().info(f"Approaching person {msg.person_id} at {location}")
                self.state = TiagoState.WALKING_TO_AREA
            else:
                self.get_logger().error("Failed to send go_to_location command.")
                self.go_to_idle()


        elif self.state == TiagoState.IDLE and distance == "close":
            # Cancel listening if active
            if self.is_listening:
                self.cancel_listening()
                
            self.get_logger().info(f"Starting conversation with person {msg.person_id}")
            self.send_path_command("stop_movement",sync=False)
            self.send_hri_command("start_conversation", msg.person_id, msg.cls)
            self.state = TiagoState.CONVERSATION
            self.current_person_id = msg.person_id

        elif self.state == TiagoState.CONVERSATION and distance == "far":
            if msg.person_id == self.current_person_id:
                self.get_logger().info("Person moved away - ending conversation")
                self.go_to_idle()

    def cancel_listening(self):
        """Cancel the listening behavior."""
        if self.listening_timer:
            self.listening_timer.cancel()
            self.listening_timer = None
        self.is_listening = False
        self.get_logger().info("Listening behavior canceled")

    def on_object_detected(self, msg):
        if self.state == TiagoState.CONVERSATION:
            return

        MIN_TIME_BETWEEN_UPDATES = 5  # seconds

        if msg.object in self.updated_objects:
            last_update = self.updated_objects[msg.object]
            if (datetime.now() - last_update).total_seconds() < MIN_TIME_BETWEEN_UPDATES:
                return

        # TODO convert msg.position to right coordinate system?
        self.get_logger().info(
            f"Object detected - ID: {msg.object}, Position: ({msg.position.x}, {msg.position.y}, {msg.position.z}). "
            f"Updating position in database.")
        request = UpdateProductPosition.Request()
        request.product_id = msg.object
        request.position = msg.position
        future = self.update_product_position_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        if result is not None and result.success:
            self.get_logger().info(f"Object {msg.object} position updated successfully.")
            self.updated_objects[msg.object] = datetime.now()  # Update last update time
        else:
            self.get_logger().error(f"Failed to update position for object {msg.object}.")

    def on_hri_status(self, msg):
        self.get_logger().info(f"Current state: {self.state}") 
        if self.state != TiagoState.CONVERSATION:
            return

        self.get_logger().info(f"Conversation status: {msg.status}")

        if msg.status == "finished":
            self.get_logger().info("Conversation finished")
            self.go_to_idle()

        elif msg.status == "walk_to_area":
            location = self.get_area_location(msg.area)
            if location is None:
                self.get_logger().error(f"Failed to get location for area '{msg.area}'")
                self.send_hri_command("unknown_location")
                return
            self.send_hri_command("walking_to_area")
            self.send_hri_command("pause_conversation")
            self.get_logger().info(f"Walking to area: {msg.area}")
            self.send_path_command("go_to_location", location)
            self.state = TiagoState.WALKING_TO_AREA

    # Helper methods
    def go_to_idle(self):
        # Cancel any listening behavior
        self.cancel_listening()
        
        self.send_hri_command("stop_conversation")
        self.state = TiagoState.IDLE
        self.current_person_id = None
        
        # Start listening behavior instead of immediately starting random walk
        self.start_listening()

    def get_distance_class(self, position):
        # Now uses updated robot position
        dx = position.x - self.robot_position.x
        dy = position.y - self.robot_position.y
        dz = position.z - self.robot_position.z
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)

        if distance < 2.0:
            return "close"
        elif distance < 3.0:
            return "medium"
        else:
            return "far"

    def get_area_location(self, area):
        # Use service to get area position
        request = GetAreaPosition.Request()
        request.area_name = area
        future = self.area_position_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        if result is not None and result.success:
            location = result.position
            new_x, new_y = bottom_left_to_center(location.x, location.y, self.map_width, self.map_height)
            return Point(x=new_x, y=new_y)
        else:
            self.get_logger().error(f"Failed to get position for area '{area}'")
            return None

    def _get_approaching_point(self, position):
        """Calculate the point in front of the person to approach. 1 mt in front of the person."""
        dx = position.x - self.robot_position.x
        dy = position.y - self.robot_position.y
        dz = position.z - self.robot_position.z

        # Normalize the direction vector
        norm = math.sqrt(dx * dx + dy * dy + dz * dz)
        if norm == 0:
            return None
        dx /= norm
        dy /= norm
        dz /= norm
        # Move 1 meter in front of the person
        approach_distance = 1.0
        new_x = position.x - dx * approach_distance
        new_y = position.y - dy * approach_distance
        new_z = position.z - dz * approach_distance
        return Point(x=new_x, y=new_y, z=new_z)

    # Service calls (simple async)
    def send_path_command(self, command, location=None, sync=False):
        request = PathPlannerCommand.Request()
        request.command = command
        request.location = location or Point()
        if sync:
            future = self.path_client.call(request)
            rclpy.spin_until_future_complete(self, future)
            return future.result()
        else:
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