#!/usr/bin/env python3
from enum import Enum
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import Point
from std_msgs.msg import String
from tiago.srv import PathPlannerCommand, HRICommand
from tiago.msg import VisionPersonDetection, ConversationStatus


class TiagoState(Enum):
    IDLE = "IDLE"
    CONVERSATION = "CONVERSATION"
    WALKING_TO_AREA = "WALKING_TO_AREA"


class ReasoningNode(Node):
    def __init__(self):
        super().__init__('reasoning_node')
        self.state = TiagoState.IDLE
        self.current_person_id = None
        self.robot_position = Point()

        self.get_logger().info("Starting TiaGo Reasoning Node")

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
        while not self.hri_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for HRI service...')

        # Subscribers
        self.path_status_sub = self.create_subscription(
            String, 'path_planner_status', self.on_path_status, self.qos_reliable)

        self.vision_sub = self.create_subscription(
            VisionPersonDetection, 'vision_person_detection', self.on_person_detected, self.qos_sensor)

        self.hri_status_sub = self.create_subscription(
            ConversationStatus, 'hri_conversation_status', self.on_hri_status, self.qos_reliable)

        # Start random walk
        self.send_path_command("random_walk")
        self.get_logger().info("Node initialized - starting random walk")

    # Callbacks
    def on_path_status(self, msg):
        if msg.data == "finished" and self.state == TiagoState.WALKING_TO_AREA:
            self.get_logger().info("Reached area - resuming conversation")
            self.send_hri_command("resume_conversation")
            self.state = TiagoState.CONVERSATION

    def on_person_detected(self, msg):
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
        # Simple distance from robot position
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