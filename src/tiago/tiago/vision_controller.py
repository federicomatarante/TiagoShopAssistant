#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import Point, PointStamped
from tiago.msg import VisionPersonDetection
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
        self.camera_frame = 'head_front_camera_color_optical_frame'
        self.world_frame = 'odom'

        # TF2 setup for coordinate transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Person tracking storage
        self.tracked_persons = {}  # person_id -> PersonTracker
        self.next_person_id = 1
        self.person_timeout = 5.0  # seconds
        
        # AprilTag-based unique person tracking
        self.apriltag_to_person = {}  # apriltag_id -> unique_person_id
        self.unique_persons = {}  # unique_person_id -> person_data
        
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
            VisionPersonDetection,
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
        
        # Tag ID mapping - unique person IDs
        self.tag_mapping = {
            1: {'type': 'staff', 'name': 'staff_leonardo'},
            2: {'type': 'staff', 'name': 'staff_lorenzo'},
            3: {'type': 'staff', 'name': 'staff_federico'},
            4: {'type': 'customer', 'name': 'customer_emanuele'},
            5: {'type': 'customer', 'name': 'customer_niccolo'},
            6: {'type': 'customer', 'name': 'customer_antonello'}
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

        # 3D point in camera optical coordinate frame
        # In optical frame: X=right, Y=down, Z=forward
        x_cam = (u - cx) * depth / fx
        y_cam = (v - cy) * depth / fy
        z_cam = depth 

        return [x_cam, y_cam, z_cam]

    def transform_to_world_coordinates(self, camera_point, timestamp):
        """Transform 3D camera coordinates to world coordinates with robust TF handling"""
        if camera_point is None:
            return None
        try:
            # Try multiple approaches for robust transformation
            
            # Approach 1: Use image timestamp with tolerance
            try:
                point_camera = PointStamped()
                point_camera.header.frame_id = self.camera_frame
                point_camera.header.stamp = timestamp
                point_camera.point.x = camera_point[0]
                point_camera.point.y = camera_point[1]
                point_camera.point.z = camera_point[2]
                
                # Transform with timeout for timestamp matching
                from rclpy.duration import Duration
                timeout = Duration(seconds=0.1)
                point_world = self.tf_buffer.transform(point_camera, self.world_frame, timeout)
                
                # Debug: Log successful transform method
                self.get_logger().debug(f'Transform success: Using image timestamp approach')
                return [point_world.point.x, point_world.point.y, point_world.point.z]
                
            except Exception as e1:
                self.get_logger().debug(f'Image timestamp transform failed: {e1}')
                
                # Approach 2: Use latest common time
                try:
                    latest_time = self.tf_buffer.get_latest_common_time(self.camera_frame, self.world_frame)
                    
                    point_camera.header.stamp = latest_time.to_msg()
                    point_world = self.tf_buffer.transform(point_camera, self.world_frame)
                    return [point_world.point.x, point_world.point.y, point_world.point.z]
                    
                except Exception as e2:
                    self.get_logger().debug(f'Latest time transform failed: {e2}')
                    
                    # Approach 3: Use zero timestamp (latest available)
                    try:
                        from rclpy.time import Time
                        point_camera.header.stamp = Time().to_msg()  # Zero time = latest
                        point_world = self.tf_buffer.transform(point_camera, self.world_frame)
                        return [point_world.point.x, point_world.point.y, point_world.point.z]
                        
                    except Exception as e3:
                        raise Exception(f'All transform approaches failed: {e1}, {e2}, {e3}')
        
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

    def get_or_create_unique_person(self, apriltag_id, person_data):
        """Get or create unique person identity based on AprilTag ID"""
        if apriltag_id in self.apriltag_to_person:
            # Update existing person
            unique_id = self.apriltag_to_person[apriltag_id]
            self.unique_persons[unique_id].update({
                'last_seen': time.time(),
                'position': person_data.get('world_position'),
                'pixel_position': person_data.get('pixel_position'),
                'depth': person_data.get('depth'),
                'confidence': person_data.get('confidence')
            })
            return unique_id, False  # False = not new
        else:
            # Create new unique person
            unique_id = f"person_{apriltag_id}"
            self.apriltag_to_person[apriltag_id] = unique_id
            self.unique_persons[unique_id] = {
                'person_id': person_data.get('name', unique_id),
                'cls': person_data.get('person_type', 'unknown'),
                'apriltag_id': apriltag_id,
                'first_seen': time.time(),
                'last_seen': time.time(),
                'position': person_data.get('world_position'),
                'pixel_position': person_data.get('pixel_position'),
                'depth': person_data.get('depth'),
                'confidence': person_data.get('confidence')
            }
            return unique_id, True  # True = is new

    def should_publish_person_update(self, unique_id, is_new_person):
        """Determine if person update should be published (avoid spam)"""
        person = self.unique_persons.get(unique_id)
        if not person:
            return False
            
        # Always publish new persons immediately
        if is_new_person:
            person['last_publish'] = time.time()
            return True
            
        # For existing persons, publish every 2 seconds
        current_time = time.time()
        last_publish = person.get('last_publish', 0)
        
        if current_time - last_publish > 2.0:  # 2 second interval
            person['last_publish'] = current_time
            return True
            
        return False

    def get_depth_at_point(self, u, v, depth_image):
        """Get depth value at specific pixel coordinates"""
        if depth_image is None:
            return None
        
        # Ensure coordinates are within image bounds
        h, w = depth_image.shape
        u = max(0, min(u, w - 1))
        v = max(0, min(v, h - 1))
        
        # Sample a small region around the point for robustness
        sample_size = 3
        u_start = max(0, u - sample_size // 2)
        u_end = min(w, u + sample_size // 2 + 1)
        v_start = max(0, v - sample_size // 2)
        v_end = min(h, v + sample_size // 2 + 1)
        
        depth_region = depth_image[v_start:v_end, u_start:u_end]
        
        # Filter valid depths
        valid_depths = depth_region[
            (depth_region > 0.1) &
            (depth_region < 10.0) &
            (~np.isnan(depth_region)) &
            (~np.isinf(depth_region))
        ]
        
        if len(valid_depths) == 0:
            return None
            
        return float(np.median(valid_depths))

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
            
            # Calculate world coordinates using AprilTag center for better precision
            tag_center_u = int(tag_info['center'][0])
            tag_center_v = int(tag_info['center'][1])
            # Get depth at AprilTag location instead of person center
            person_depth = self.get_depth_at_point(tag_center_u, tag_center_v, self.current_depth_image)

            world_coords = None
            if person_depth is not None:
                camera_coords = self.pixel_to_3d_camera(tag_center_u, tag_center_v, person_depth)
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
            # Calculate world coordinates using AprilTag center for better precision
            tag_center_u = int(tag_info['center'][0])
            tag_center_v = int(tag_info['center'][1])
            # Get depth at AprilTag location instead of person center
            person_depth = self.get_depth_at_point(tag_center_u, tag_center_v, self.current_depth_image)

            world_coords = None
            if person_depth is not None:
                camera_coords = self.pixel_to_3d_camera(tag_center_u, tag_center_v, person_depth)
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
                    
                    # Only track and publish persons with AprilTags (for unique identity)
                    if match['tag'] and 'tag_id' in result:
                        apriltag_id = result['tag_id']
                        unique_id, is_new = self.get_or_create_unique_person(apriltag_id, result)
                        
                        # Only publish if it's a new person or if enough time has passed
                        if self.should_publish_person_update(unique_id, is_new):
                            person_data = self.unique_persons[unique_id]
                            
                            result_msg = VisionPersonDetection()
                            result_msg.person_id = person_data['person_id']
                            result_msg.cls = person_data['cls']
                            
                            # Set 3D position
                            result_msg.position = Point()
                            world_pos = person_data.get('position')
                            if world_pos and len(world_pos) >= 3:
                                result_msg.position.x = float(world_pos[0])
                                result_msg.position.y = float(world_pos[1])
                                result_msg.position.z = float(world_pos[2])
                            else:
                                result_msg.position.x = 0.0
                                result_msg.position.y = 0.0
                                result_msg.position.z = 0.0
                            
                            self.result_publisher.publish(result_msg)
                            
                            if should_log:
                                status = "NEW" if is_new else "UPDATE"
                                self.get_logger().info(f'{status} Person: {person_data["person_id"]} (AprilTag {apriltag_id}) at [{world_pos[0]:.3f}, {world_pos[1]:.3f}, {world_pos[2]:.3f}]' if world_pos else f'{status} Person: {person_data["person_id"]} (AprilTag {apriltag_id}) - no position')
                    else:
                        # Handle persons without AprilTags (don't publish to avoid duplicates)
                        if should_log:
                            self.get_logger().info(f'Person detected without AprilTag - not publishing to avoid duplicates')

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