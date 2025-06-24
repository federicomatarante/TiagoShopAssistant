#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import Point, PointStamped
from tf2_ros import TransformListener, Buffer
import tf2_geometry_msgs
from cv_bridge import CvBridge
import json
import numpy as np
import cv2
import apriltag
import time
from collections import defaultdict

class VisionController(Node):
    def __init__(self):
        super().__init__('vision_controller')
        
        # Initialize CV bridge
        self.bridge = CvBridge()

        self.log_counter = 0
        self.current_depth_image = None

        # Camera intrinsics (will be updated from camera_info)
        self.camera_intrinsics = None
        self.camera_frame = 'head_front_camera_rgb_optical_frame'
        self.world_frame = 'map'

        # TF2 setup for coordinate transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Person tracking storage
        self.tracked_persons = {}  # person_id -> PersonTracker
        self.next_person_id = 1
        self.person_timeout = 5.0  # seconds
        
        # Subscribe to TIAGo's camera
        self.image_subscription = self.create_subscription(
            Image,
            '/head_front_camera/rgb/image_raw',
            self.image_callback,
            10)

        # Subscription to TIAGo's depth camera
        self.depth_subscription = self.create_subscription(
            Image,
            '/head_front_camera/depth/image_raw',
            self.depth_callback,
            10)
        
        # Subscription to camera info for intrinsics
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            '/head_front_camera/rgb/camera_info',
            self.camera_info_callback,
            10)
            
        # Publisher for detection results
        self.result_publisher = self.create_publisher(
            String,
            '/person_detection',
            10)
        
        # Try to initialize face cascade (with error handling)
        try:
            cascade_path = '/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml'
            self.face_cascade = cv2.CascadeClassifier(cascade_path)
            if self.face_cascade.empty():
                self.get_logger().warn('Face cascade failed to load, disabling face detection')
                self.face_cascade = None
            else:
                self.get_logger().info('Face cascade loaded successfully')
        except Exception as e:
            self.get_logger().warn(f'Could not load face cascade: {e}')
            self.face_cascade = None

        # Person detection
        self.person_detector = self.init_person_detector()

        # AprilTag detection
        self.apriltag_detector = apriltag.Detector()
        
        # Tag ID mapping
        self.tag_mapping = {
            1: {'type': 'staff', 'name': 'Staff Member'},
            2: {'type': 'customer', 'name': 'Customer'}
        }
        
        self.get_logger().info('AprilTag detector initialized')

        # Debug timer
        self.timer = self.create_timer(10.0, self.timer_callback)
        
        self.get_logger().info('Vision Controller initialized')

    def init_person_detector(self):
        """Initialize person detection using HOG or simple blob detection"""
        try:
            # TODO: Do we need more complex option?
            hog = cv2.HOGDescriptor()
            hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
            self.get_logger().info('HOG person detector initialized')
            return hog
        except Exception as e:
            self.get_logger().error(f'Error initializing person detector: {e}')
            return None

    def depth_callback(self, msg):
        """Process incoming depth images"""
        try:
            # Convert ROS Image message to OpenCV format
            # Depth images can be in different formats (16UC1, 32FC1)
            if msg.encoding == '16UC1':
                # 16-bit unsigned integer (millimeters)
                depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
                # Convert to meters
                depth_image = depth_image.astype(np.float32) / 1000.0
            elif msg.encoding == '32FC1':
                # 32-bit float (meters)
                depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            else:
                self.get_logger().error(f'Unsupported depth image encoding: {msg.encoding}')
                return

            # Store the depth image for use in RGB processing
            self.current_depth_image = depth_image

        except Exception as e:
            self.get_logger().error(f'Error in depth callback: {e}')

    def camera_info_callback(self, msg):
        """Process camera intrinsics parameters"""
        if self.camera_intrinsics is None:
            # Extract camera intrinsics from CameraInfo message
            self.camera_intrinsics = {
                'fx': msg.k[0],  # Focal length x
                'fy': msg.k[4],  # Focal length y
                'cx': msg.k[2],  # Principal point x
                'cy': msg.k[5],  # Principal point y
                'width': msg.width,
                'height': msg.height
            }
            self.get_logger().info(f'Camera intrinsics loaded: fx={self.camera_intrinsics["fx"]:.1f}, fy={self.camera_intrinsics["fy"]:.1f}')

            # Unsubscribe after getting intrinsics (they are static )
            self.destroy_subscription(self.camera_info_subscription)
 
    def pixel_to_3d_camera(self, u, v, depth):
        """Convert pixel coordinates + depth to 3D camera coordinates"""
        if self.camera_intrinsics is None or depth is None or depth <= 0:
            return None

        # Convert pixel coordinates to 3D camera coordinates
        # Using pinhole camera model: X = (u - cx) * Z / fx, Y = (v - cy) * Z / fy
        fx = self.camera_intrinsics['fx']
        fy = self.camera_intrinsics['fy']
        cx = self.camera_intrinsics['cx']
        cy = self.camera_intrinsics['cy']

        # 3D point in camera coordinate frame
        x_cam = (u - cx) * depth / fx
        y_cam = (v - cy) * depth / fy
        z_cam = depth 

        return [x_cam, y_cam, z_cam]

    def transform_to_world_coordinates(self, camera_point, timestamp):
        """Transform 3D camera coordinates to world coordinates"""
        if camera_point is None:
            return None
        try:
            # Create a PointStamped in camera frame
            point_camera = PointStamped()
            point_camera.header.frame_id = self.camera_frame
            point_camera.header.stamp = timestamp
            point_camera.point.x = camera_point[0]
            point_camera.point.y = camera_point[1]
            point_camera.point.z = camera_point[2]

            # Transform to world coordinates
            point_world = self.tf_buffer.transform(point_camera, self.world_frame)

            return [point_world.point.x, point_world.point.y, point_world.point.z]
        
        except Exception as e:
            self.get_logger().error(f'Error transforming to world coordinates: {e}')
            return None

    def get_person_depth(self, person_rect, depth_image):
        """Extract reliable depth measurement for a detected person"""
        if depth_image is None:
            return None

        x, y, w, h = person_rect

        # Define sampling region - focus on person's torso area
        # Avoid edges where background might leak in
        margin_x = max(1, w // 6) # 16% margin from left/right edges
        margin_y = max(1, h // 8) # 12% margin from top/bottom edges

        # Sample region: center torso area
        sample_x1 = x + margin_x
        sample_x2 = x + w - margin_x
        sample_y1 = y + margin_y + h // 4  # Start at 25% height - upper torso
        sample_y2 = y + h - margin_y - h // 4 # End at 75% height - lower torso

        # Ensure bounds are valid
        sample_x1 = max(0, min(sample_x1, depth_image.shape[1] - 1))
        sample_x2 = max(sample_x1 + 1, min(sample_x2, depth_image.shape[1]))
        sample_y1 = max(0, min(sample_y1, depth_image.shape[0] - 1))
        sample_y2 = max(sample_y1 + 1, min(sample_y2, depth_image.shape[0]))

        # Extract depth values from the sampling region
        depth_region = depth_image[sample_y1:sample_y2, sample_x1:sample_x2]
        
        # Filter out invalid depth values
        valid_depths = depth_region[
          (depth_region > 0.1) &  # Minimum 10cm
          (depth_region < 10.0) &  # Maximum 10m
          (~np.isnan(depth_region)) &  # Not NaN
          (~np.isinf(depth_region))   # Not infinite
        ]

        if len(valid_depths) == 0:
            return None

        # Use median depth for robustness against outliers
        person_depth = np.median(valid_depths)

        return float(person_depth) 
        

    def detect_persons(self, image):
        """Detect persons in the given image"""
        if self.person_detector is None:
            return []

        try:
            (rects, weights) = self.person_detector.detectMultiScale(
                image, 
                winStride=(4, 4), 
                padding=(8, 8), 
                scale=1.05
            )
            return rects  # CHANGED: return rectangles
        except Exception as e:
            self.get_logger().error(f'Error during person detection: {e}')
            return []

    def detect_apriltags(self, image):
        """Detect AprilTags in the image"""
        try:
            # Convert to grayscale for AprilTag detection
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            
            # Detect tags
            tags = self.apriltag_detector.detect(gray)
            
            detected_tags = []
            for tag in tags:
                # Extract tag information
                tag_info = {
                    'id': tag.tag_id,
                    'center': tag.center,
                    'corners': tag.corners,
                    'confidence': tag.decision_margin,
                    'hamming': tag.hamming
                }
                detected_tags.append(tag_info)
                
                self.get_logger().debug(f'Detected AprilTag ID: {tag.tag_id}, confidence: {tag.decision_margin:.2f}')
            
            return detected_tags
            
        except Exception as e:
            self.get_logger().error(f'Error during AprilTag detection: {e}')
            return []

    def match_tags_to_persons(self, person_rects, detected_tags):
        """Match AprilTags to person detections based on proximity"""
        matches = []
        
        for i, person_rect in enumerate(person_rects):
            x, y, w, h = person_rect
            person_center = (x + w//2, y + h//2)
            
            best_tag = None
            min_distance = float('inf')
            
            for tag in detected_tags:
                tag_center = tag['center']
                distance = np.sqrt((person_center[0] - tag_center[0])**2 + 
                                (person_center[1] - tag_center[1])**2)
                
                expanded_margin = max(w, h) * 0.5
                within_x = (x - expanded_margin) <= tag_center[0] <= (x + w + expanded_margin)
                within_y = (y - expanded_margin) <= tag_center[1] <= (y + h + expanded_margin)
                
                if (within_x and within_y) or distance < min(w, h):
                    if distance < min_distance:
                        min_distance = distance
                        best_tag = tag
            
            matches.append({
                'person_rect': person_rect,
                'tag': best_tag,
                'distance': min_distance if best_tag else None
            })
        
        return matches

    # Badge detection method
    def detect_badge_type(self, image, person_rect):
        """Detect badge type in person chest area"""
        try:
            x, y, w, h = person_rect
            # Focus on chest area where badge is located
            chest_y = y + int(h * 0.3)  # Upper 30% of person
            chest_h = int(h * 0.4)      # Next 40% of person height
            chest_roi = image[chest_y:chest_y+chest_h, x:x+w]
            
            # Convert to HSV for better color detection
            hsv = cv2.cvtColor(chest_roi, cv2.COLOR_BGR2HSV)
            
            # Define color ranges for badges
            # Dark/black badge (staff)
            lower_dark = np.array([0, 0, 0])
            upper_dark = np.array([180, 255, 50])
            dark_mask = cv2.inRange(hsv, lower_dark, upper_dark)
            
            # Gray badge (customer)  
            lower_gray = np.array([0, 0, 50])
            upper_gray = np.array([180, 30, 200])
            gray_mask = cv2.inRange(hsv, lower_gray, upper_gray)
            
            dark_pixels = cv2.countNonZero(dark_mask)
            gray_pixels = cv2.countNonZero(gray_mask)
            
            if dark_pixels > gray_pixels and dark_pixels > 50:
                return 'staff'
            elif gray_pixels > 50:
                return 'customer'
            else:
                return 'unknown'
                
        except Exception as e:
            self.get_logger().error(f'Error in badge detection: {e}')
            return 'unknown'

    def classify_person_with_tag(self, image, person_rect, tag_info=None, timestamp=None):
        """Complete pipeline: person -> AprilTag detection -> classification"""
        
        x, y, w, h = person_rect
        
        if tag_info and tag_info['id'] in self.tag_mapping:
            # AprilTag detected and recognized
            tag_data = self.tag_mapping[tag_info['id']]
            confidence = min(0.95, max(0.5, tag_info['confidence'] / 50.0))  # Normalize confidence
            
            # Calculate world coordinates
            person_center_u = x + w // 2
            person_center_v = y + h // 2
            person_depth = self.get_person_depth(person_rect, self.current_depth_image)

            world_coords = None
            if person_depth is not None:
                camera_coords = self.pixel_to_3d_camera(person_center_u, person_center_v, person_depth)
                if camera_coords is not None and timestamp is not None:
                    world_coords = self.transform_to_world_coordinates(camera_coords, timestamp)

            return {
                'person_type': tag_data['type'],
                'name': tag_data['name'],
                'tag_id': tag_info['id'],
                'confidence': confidence,
                'pixel_position': f"x:{x}, y:{y}, w:{w}, h:{h}",
                'world_position': world_coords,
                'depth': person_depth,
                'tag_center': tag_info['center'].tolist(),
                'tag_confidence': tag_info['confidence'],
                'detection_method': 'apriltag'
            }
        elif tag_info:
            # AprilTag detected but unknown ID
            # Calculate world coordinates
            person_center_u = x + w // 2
            person_center_v = y + h // 2
            person_depth = self.get_person_depth(person_rect, self.current_depth_image)

            world_coords = None
            if person_depth is not None:
                camera_coords = self.pixel_to_3d_camera(person_center_u, person_center_v, person_depth)
                if camera_coords is not None and timestamp is not None:
                    world_coords = self.transform_to_world_coordinates(camera_coords, timestamp)
            return {
                'person_type': 'unknown',
                'name': f'Unknown Tag ID {tag_info["id"]}',
                'tag_id': tag_info['id'],
                'confidence': 0.8,
                'pixel_position': f"x:{x}, y:{y}, w:{w}, h:{h}",
                'world_position': world_coords,
                'depth': person_depth,
                'tag_center': tag_info['center'].tolist(),
                'detection_method': 'apriltag_unknown'
            }
        else:
            # No AprilTag detected
            # Calculate world coordinates
            person_center_u = x + w // 2
            person_center_v = y + h // 2
            person_depth = self.get_person_depth(person_rect, self.current_depth_image)

            world_coords = None
            if person_depth is not None:
                camera_coords = self.pixel_to_3d_camera(person_center_u, person_center_v, person_depth)
                if camera_coords is not None and timestamp is not None:
                    world_coords = self.transform_to_world_coordinates(camera_coords, timestamp)
            
            return {
                'person_type': 'unknown',
                'name': 'Person without badge',
                'confidence': 0.3,
                'pixel_position': f"x:{x}, y:{y}, w:{w}, h:{h}",
                'world_position': world_coords,
                'depth': person_depth,
                'detection_method': 'no_apriltag'
            }
    
    def timer_callback(self):
        self.get_logger().info("Timer callback - node is alive")
    
    def image_callback(self, msg):
        """Process incoming camera images - AprilTag Recognition Pipeline"""
        try:            
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Less frequent logging
            self.log_counter += 1
            should_log = (self.log_counter % 10 == 0)
            
            # Step 1: Detect persons
            person_rects = self.detect_persons(cv_image)
            
            # Step 2: Detect AprilTags
            detected_tags = self.detect_apriltags(cv_image)

            if len(person_rects) > 0:
                # Always log detection counts if should_log
                if should_log:
                    self.get_logger().info(f'Detected {len(person_rects)} person(s) and {len(detected_tags)} AprilTag(s)')

                # Step 3: Match tags to persons (ALWAYS)
                matches = self.match_tags_to_persons(person_rects, detected_tags)
                
                # Step 4: Classify each person (ALWAYS)
                for match in matches:
                    result = self.classify_person_with_tag(
                        cv_image,
                        match['person_rect'],
                        match['tag'],
                        msg.header.stamp
                    )
                    
                    # Publish result (ALWAYS)
                    result_msg = String()
                    result_msg.data = json.dumps(result)
                    self.result_publisher.publish(result_msg)
                    
                    # Log result only when should_log
                    if should_log:
                        person_type = result['person_type']
                        name = result.get('name', 'Unknown')
                        method = result.get('detection_method', 'unknown')
                        confidence = result.get('confidence', 0.0)
                        tag_id = result.get('tag_id', 'None')
                        self.get_logger().info(f'Classified {person_type}: {name} (method: {method}, tag_id: {tag_id}, confidence: {confidence:.2f})')

            elif len(detected_tags) > 0 and should_log:
                self.get_logger().info(f'No persons detected, but found {len(detected_tags)} AprilTag(s)')
            elif should_log:
                self.get_logger().debug(f'No persons or tags detected')
                
        except Exception as e:
            self.get_logger().error(f'Error in image callback: {e}')
                        

def main(args=None):
    rclpy.init(args=args)
    vision_controller = VisionController()
    
    try:
        rclpy.spin(vision_controller)
    except KeyboardInterrupt:
        pass
    finally:
        vision_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()