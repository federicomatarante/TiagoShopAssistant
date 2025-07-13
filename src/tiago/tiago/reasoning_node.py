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
    AUTONOMOUS_SEQUENCE = "AUTONOMOUS_SEQUENCE"  # New state for autonomous behavior


class ReasoningNode(Node):
    def __init__(self):
        super().__init__('reasoning_node')
        self.state = TiagoState.AUTONOMOUS_SEQUENCE  # Start in autonomous mode
        self.current_person_id = None
        self.robot_position = Point()

        # Autonomous sequence tracking
        self.random_walk_count = 0
        self.max_random_walks = 2
        self.final_position = Point(x=-4.3, y=-3.0, z=0.0)
        self.sequence_completed = False
        self.waiting_for_path_completion = False  # NEW: Track if we're waiting for path completion
        self.retry_timer = None  # NEW: Track retry timer to avoid overlaps

        self.get_logger().info("Starting TiaGo Reasoning Node with autonomous sequence")

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
            VisionPersonDetection, 'vision_person_detection', self.on_person_detected, self.qos_sensor)

        self.hri_status_sub = self.create_subscription(
            ConversationStatus, 'hri_conversation_status', self.on_hri_status, self.qos_reliable)

        # Alternative: Subscribe to odometry if TF is not available
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.on_odometry, self.qos_sensor)

        # Start autonomous sequence
        self.start_autonomous_sequence()

    def start_autonomous_sequence(self):
        """Start the autonomous sequence with random walks followed by fixed position."""
        self.get_logger().info("Starting autonomous sequence: 2 random walks + final position")
        self.execute_next_random_walk()

    def execute_next_random_walk(self):
        """Execute the next random walk in the sequence."""
        # NEW: Check if we're already waiting for a path completion
        if self.waiting_for_path_completion:
            self.get_logger().info("Already waiting for path completion, skipping duplicate request")
            return
            
        # NEW: Cancel any existing retry timer
        if self.retry_timer:
            self.retry_timer.cancel()
            self.retry_timer = None
        
        if self.random_walk_count < self.max_random_walks:
            self.random_walk_count += 1
            self.waiting_for_path_completion = True  # NEW: Mark that we're waiting
            
            self.get_logger().info(f"Starting random walk {self.random_walk_count}/{self.max_random_walks}")
            
            # Send command and wait for completion via path status callback
            future = self.send_path_command("random_walk")
            
            # Handle the response
            def handle_random_walk_response(future):
                try:
                    response = future.result()
                    if response is not None and response.success:
                        self.get_logger().info(f"Random walk {self.random_walk_count} command successfully sent")
                        # Don't set waiting_for_path_completion = False here, 
                        # wait for path status callback
                    else:
                        self.get_logger().error(f"Random walk {self.random_walk_count} command failed")
                        self.waiting_for_path_completion = False  # NEW: Reset waiting flag
                        self.handle_random_walk_failure()
                except Exception as e:
                    self.get_logger().error(f"Random walk service call failed: {e}")
                    self.waiting_for_path_completion = False  # NEW: Reset waiting flag
                    self.handle_random_walk_failure()
            
            future.add_done_callback(handle_random_walk_response)
        else:
            self.execute_final_position()

    def handle_random_walk_failure(self):
        """Handle random walk failure with retry after delay."""
        if self.retry_timer:
            self.retry_timer.cancel()
            
        self.get_logger().info(f"Retrying random walk {self.random_walk_count} in 5 seconds...")
        self.random_walk_count -= 1  # Decrement to retry the same walk
        
        # NEW: Create single-shot timer for retry
        self.retry_timer = self.create_timer(5.0, self.retry_random_walk_callback)

    def retry_random_walk_callback(self):
        """Single-shot callback for retry timer."""
        if self.retry_timer:
            self.retry_timer.cancel()
            self.retry_timer = None
        
        self.get_logger().info("Retry timer expired, attempting random walk again")
        self.execute_next_random_walk()

    def execute_final_position(self):
        """Navigate to the final fixed position."""
        # NEW: Check if we're already waiting for a path completion
        if self.waiting_for_path_completion:
            self.get_logger().info("Already waiting for path completion, skipping duplicate request")
            return
            
        # NEW: Cancel any existing retry timer
        if self.retry_timer:
            self.retry_timer.cancel()
            self.retry_timer = None
        
        self.waiting_for_path_completion = True  # NEW: Mark that we're waiting
        
        self.get_logger().info(f"All random walks completed. Going to final position: [{self.final_position.x}, {self.final_position.y}, {self.final_position.z}]")
        
        future = self.send_path_command("go_to_location", self.final_position)
        
        def handle_final_position_response(future):
            try:
                response = future.result()
                if response is not None and response.success:
                    self.get_logger().info("Final position command successfully sent")
                    # Don't set waiting_for_path_completion = False here,
                    # wait for path status callback
                else:
                    self.get_logger().error("Final position command failed")
                    self.waiting_for_path_completion = False  # NEW: Reset waiting flag
                    self.handle_final_position_failure()
            except Exception as e:
                self.get_logger().error(f"Final position service call failed: {e}")
                self.waiting_for_path_completion = False  # NEW: Reset waiting flag
                self.handle_final_position_failure()
        
        future.add_done_callback(handle_final_position_response)

    def handle_final_position_failure(self):
        """Handle final position failure with retry after delay."""
        if self.retry_timer:
            self.retry_timer.cancel()
            
        self.get_logger().info("Retrying final position navigation in 5 seconds...")
        
        # NEW: Create single-shot timer for retry
        self.retry_timer = self.create_timer(5.0, self.retry_final_position_callback)

    def retry_final_position_callback(self):
        """Single-shot callback for final position retry timer."""
        if self.retry_timer:
            self.retry_timer.cancel()
            self.retry_timer = None
        
        self.get_logger().info("Final position retry timer expired, attempting navigation again")
        self.execute_final_position()

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

    # Callbacks
    def on_path_status(self, msg):
        if msg.data == "finished":
            self.waiting_for_path_completion = False  # NEW: Reset waiting flag
            
            if self.state == TiagoState.AUTONOMOUS_SEQUENCE:
                if self.random_walk_count < self.max_random_walks:
                    self.get_logger().info(f"Random walk {self.random_walk_count} completed")
                    # NEW: Add delay before next random walk to avoid rapid succession
                    self.create_timer(2.0, self.execute_next_random_walk)
                elif not self.sequence_completed:
                    self.get_logger().info("Final position reached! Autonomous sequence completed.")
                    self.sequence_completed = True
                    self.state = TiagoState.IDLE
                    self.get_logger().info("Switching to normal operation mode (IDLE)")
            elif self.state == TiagoState.WALKING_TO_AREA:
                self.get_logger().info("Reached area - resuming conversation")
                self.send_hri_command("resume_conversation")
                self.state = TiagoState.CONVERSATION

    def on_person_detected(self, msg):
        # Only respond to person detection if not in autonomous sequence
        if self.state == TiagoState.AUTONOMOUS_SEQUENCE:
            return
            
        distance = self.get_distance_class(msg.position)

        if self.state == TiagoState.IDLE and distance == "close":
            self.get_logger().info(f"Starting conversation with person {msg.person_id}")
            self.send_path_command("stop_movement")
            self.send_hri_command("start_conversation", msg.person_id, msg.cls)
            self.state = TiagoState.CONVERSATION
            self.current_person_id = msg.person_id

        elif self.state == TiagoState.CONVERSATION and distance == "far":
            if msg.person_id == self.current_person_id:
                self.get_logger().info("Person moved away - ending conversation")
                self.go_to_idle()

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
        self.send_path_command("random_walk")
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