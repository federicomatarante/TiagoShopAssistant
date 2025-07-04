#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import GetEntityState, GetModelList
from tiago.msg import VisionPersonDetection, VisionObjectDetection
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
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

        # Gazebo model states for position lookup
        self.model_states = None
        
        # Gazebo service clients (reliable method)
        self.gazebo_get_entity_client = self.create_client(GetEntityState, 'get_entity_state')
        self.gazebo_get_model_list_client = self.create_client(GetModelList, 'get_model_list')
        
        # Cache for model list to avoid repeated service calls
        self.cached_model_list = None
        self.model_list_cache_time = 0

        # Person tracking storage
        self.tracked_persons = {}  # person_id -> PersonTracker
        self.next_person_id = 1
        self.person_timeout = 5.0  # seconds
        
        # AprilTag-based unique person tracking
        self.apriltag_to_person = {}  # apriltag_id -> unique_person_id
        self.unique_persons = {}  # unique_person_id -> person_data
        
        # AprilTag-based object tracking
        self.apriltag_to_object = {}  # apriltag_id -> unique_object_id
        self.unique_objects = {}  # unique_object_id -> object_data
        
        # Subscribe to TIAGo's camera
        self.image_subscription = self.create_subscription(
            Image,
            '/head_front_camera/rgb/image_raw',
            self.image_callback,
            10)

        # Subscribe to Gazebo model states for perfect position data
        self.gazebo_subscription = self.create_subscription(
            ModelStates,
            '/model_states',
            self.gazebo_callback,
            10)
            
        # Publisher for detection results
        self.result_publisher = self.create_publisher(
            VisionPersonDetection,
            '/person_detection',
            10)
        
        # Publisher for object detection results
        self.object_publisher = self.create_publisher(
            VisionObjectDetection,
            '/object_detection',
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
        
        # Tag ID mapping - maps AprilTag IDs to Gazebo model names
        self.tag_mapping = {
            1: {'type': 'staff', 'name': 'staff_leonardo', 'gazebo_model': 'staff_leonardo'},
            2: {'type': 'staff', 'name': 'staff_lorenzo', 'gazebo_model': 'staff_lorenzo'},
            3: {'type': 'staff', 'name': 'staff_federico', 'gazebo_model': 'staff_federico'},
            4: {'type': 'customer', 'name': 'customer_emanuele', 'gazebo_model': 'customer_emanuele'},
            5: {'type': 'customer', 'name': 'customer_niccolo', 'gazebo_model': 'customer_niccolo'},
            6: {'type': 'customer', 'name': 'customer_antonello', 'gazebo_model': 'customer_antonello'},
            # Object tags (sport equipment on red boxes) - 8 total objects, 2 different per sport
            8: {'type': 'object', 'class': 'baseball', 'object': 'baseball bat', 'gazebo_model': 'red_box_baseball_bat'},
            9: {'type': 'object', 'class': 'baseball', 'object': 'baseball', 'gazebo_model': 'red_box_baseball'},
            10: {'type': 'object', 'class': 'soccer', 'object': 'soccer ball', 'gazebo_model': 'red_box_soccer_ball'},
            11: {'type': 'object', 'class': 'soccer', 'object': 'soccer cleats', 'gazebo_model': 'red_box_soccer_cleats'},
            12: {'type': 'object', 'class': 'basketball', 'object': 'basketball', 'gazebo_model': 'red_box_basketball'},
            13: {'type': 'object', 'class': 'basketball', 'object': 'basketball hoop', 'gazebo_model': 'red_box_basketball_hoop'},
            14: {'type': 'object', 'class': 'tennis', 'object': 'tennis racket', 'gazebo_model': 'red_box_tennis_racket'},
            15: {'type': 'object', 'class': 'tennis', 'object': 'tennis ball', 'gazebo_model': 'red_box_tennis_ball'}
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

    def gazebo_callback(self, msg):
        """Process Gazebo model states for perfect position data"""
        self.model_states = msg

    def get_model_list_from_gazebo_cached(self):
        """Get cached model list from Gazebo (refresh every 10 seconds)"""
        current_time = time.time()
        
        # Use cached list if recent (models don't change often)
        if (self.cached_model_list is not None and 
            current_time - self.model_list_cache_time < 10.0):
            return self.cached_model_list
            
        # Refresh cache
        try:
            self.get_logger().info('🔍 DEBUG: Checking if get_model_list service is ready...')
            if not self.gazebo_get_model_list_client.service_is_ready():
                self.get_logger().error('❌ Gazebo get_model_list service not ready')
                return self.cached_model_list  # Return old cache if available
            
            self.get_logger().info('✅ DEBUG: Service is ready, making call...')
                
            request = GetModelList.Request()
            future = self.gazebo_get_model_list_client.call_async(request)
            
            # Use reasonable timeout for model list
            start_time = time.time()
            self.get_logger().info('🔄 DEBUG: Starting service call wait loop...')
            while not future.done() and (time.time() - start_time) < 2.0:
                rclpy.spin_once(self, timeout_sec=0.01)
            
            elapsed = time.time() - start_time
            self.get_logger().info(f'⏱️  DEBUG: Wait loop finished. Elapsed: {elapsed:.3f}s, Done: {future.done()}')
            
            if future.done() and future.result() is not None:
                response = future.result()
                if response.success:
                    self.cached_model_list = response.model_names
                    self.model_list_cache_time = current_time
                    return self.cached_model_list
                    
            self.get_logger().warn('⚠️  Model list service call failed/timeout')
            return self.cached_model_list  # Return old cache if available
                
        except Exception as e:
            self.get_logger().error(f'💥 Error getting model list: {e}')
            return self.cached_model_list

    def get_position_from_gazebo_service(self, model_name):
        """Get position using Gazebo service call (non-blocking method)"""
        try:
            # Check if model exists (using cached list)
            model_list = self.get_model_list_from_gazebo_cached()
            if model_list and model_name not in model_list:
                self.get_logger().debug(f'Model {model_name} not found in cached list')
                return None
            
            # Get entity state - but don't wait if service isn't ready
            if not self.gazebo_get_entity_client.service_is_ready():
                self.get_logger().debug('get_entity_state service not ready')
                return None
                
            request = GetEntityState.Request()
            request.name = model_name
            request.reference_frame = 'world'
            
            future = self.gazebo_get_entity_client.call_async(request)
            
            # Reasonable timeout for service calls
            start_time = time.time()
            while not future.done() and (time.time() - start_time) < 2.0:
                rclpy.spin_once(self, timeout_sec=0.01)
            
            if future.done() and future.result() is not None:
                response = future.result()
                if response.success:
                    pos = response.state.pose.position
                    position = [pos.x, pos.y, pos.z]
                    self.get_logger().info(f'✅ GAZEBO SERVICE: {model_name} at [{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}]')
                    return position
                else:
                    self.get_logger().debug(f'Service call failed for {model_name}')
                    return None
            else:
                self.get_logger().debug(f'Service timeout for {model_name}')
                return None
                
        except Exception as e:
            self.get_logger().debug(f'Service error for {model_name}: {e}')
            return None

    def get_position_from_gazebo(self, model_name):
        """Get exact position of a model from Gazebo using model_states topic"""
        
        if self.model_states is not None:
            try:
                if model_name in self.model_states.name:
                    index = self.model_states.name.index(model_name)
                    pose = self.model_states.pose[index]
                    position = [pose.position.x, pose.position.y, pose.position.z]
                    return position
                else:
                    self.get_logger().debug(f'Model {model_name} not found in Gazebo')
                    return None
            except Exception as e:
                self.get_logger().error(f'Error getting position for {model_name}: {e}')
                return None
        else:
            return None
 

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

    def get_or_create_unique_object(self, apriltag_id, object_data):
        """Get or create unique object identity based on AprilTag ID"""
        if apriltag_id in self.apriltag_to_object:
            # Update existing object (but keep original position to avoid duplicates)
            unique_id = self.apriltag_to_object[apriltag_id]
            self.unique_objects[unique_id].update({
                'last_seen': time.time(),
                'tag_center': object_data.get('tag_center'),
                'confidence': object_data.get('confidence')
                # NOTE: Position is NOT updated to prevent duplicate detections from multiple faces
            })
            return unique_id, False  # False = not new
        else:
            # Create new unique object with initial position
            unique_id = f"object_{apriltag_id}"
            self.apriltag_to_object[apriltag_id] = unique_id
            self.unique_objects[unique_id] = {
                'class': object_data.get('class', 'unknown'),
                'object': object_data.get('name', 'unknown'),  # Use 'name' field consistently
                'apriltag_id': apriltag_id,
                'first_seen': time.time(),
                'last_seen': time.time(),
                'position': object_data.get('world_position'),  # Store position only once
                'tag_center': object_data.get('tag_center'),
                'confidence': object_data.get('confidence'),
                'position_locked': True  # Flag to indicate position is set and should not change
            }
            return unique_id, True  # True = is new

    def should_publish_object_update(self, unique_id, is_new_object):
        """Determine if object update should be published"""
        obj = self.unique_objects.get(unique_id)
        if not obj:
            return False
            
        # Always publish new objects immediately
        if is_new_object:
            obj['last_publish'] = time.time()
            return True
            
        # For existing objects, publish every 2 seconds
        current_time = time.time()
        last_publish = obj.get('last_publish', 0)
        
        if current_time - last_publish > 2.0:  # 2 second interval
            obj['last_publish'] = current_time
            return True
            
        return False

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

    def detect_red_regions(self, image):
        """Simple red color detection for confidence boost"""
        try:
            # Convert to HSV for better color detection
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            
            # Define red color range (two ranges for red hue wrap-around)
            lower_red1 = np.array([0, 50, 50])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([170, 50, 50])
            upper_red2 = np.array([180, 255, 255])
            
            # Create masks for red regions
            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            red_mask = cv2.bitwise_or(mask1, mask2)
            
            # Find contours
            contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            red_regions = []
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 500:  # Minimum area threshold
                    x, y, w, h = cv2.boundingRect(contour)
                    red_regions.append((x, y, w, h, area))
            
            return red_regions
            
        except Exception as e:
            self.get_logger().error(f'Error in red detection: {e}')
            return []

    def is_tag_on_red_box(self, tag_info, red_regions):
        """Check if AprilTag is on a red region"""
        if not red_regions:
            return False
            
        tag_center = tag_info['center']
        
        for x, y, w, h, area in red_regions:
            # Check if tag center is within red region
            if x <= tag_center[0] <= x + w and y <= tag_center[1] <= y + h:
                return True
        
        return False

    def classify_person_with_tag(self, image, person_rect, tag_info=None, timestamp=None):
        """Simplified pipeline: AprilTag -> Classification -> Gazebo Position Lookup"""
        
        x, y, w, h = person_rect
        
        if tag_info and tag_info['id'] in self.tag_mapping:
            # AprilTag detected and recognized
            tag_data = self.tag_mapping[tag_info['id']]
            confidence = min(0.95, max(0.5, tag_info['confidence'] / 50.0))  # Normalize confidence
            
            # Get exact position from Gazebo using model name
            gazebo_model_name = tag_data['gazebo_model']
            world_coords = self.get_position_from_gazebo(gazebo_model_name)
            
            if not world_coords:
                self.get_logger().debug(f'No position data available for {gazebo_model_name}')

            # Handle different field names for person vs object tags
            if tag_data['type'] == 'object':
                name = tag_data['object']  # Objects use 'object' field
            else:
                name = tag_data['name']    # Persons use 'name' field
                
            return {
                'person_type': tag_data['type'],
                'name': name,
                'tag_id': tag_info['id'],
                'confidence': confidence,
                'pixel_position': f"x:{x}, y:{y}, w:{w}, h:{h}",
                'world_position': world_coords,
                'tag_center': tag_info['center'].tolist(),
                'tag_confidence': tag_info['confidence'],
                'detection_method': 'gazebo_lookup'
            }
        elif tag_info:
            # AprilTag detected but unknown ID
            return {
                'person_type': 'unknown',
                'name': f'Unknown Tag ID {tag_info["id"]}',
                'tag_id': tag_info['id'],
                'confidence': 0.8,
                'pixel_position': f"x:{x}, y:{y}, w:{w}, h:{h}",
                'world_position': None,  # No Gazebo model for unknown tags
                'tag_center': tag_info['center'].tolist(),
                'detection_method': 'apriltag_unknown'
            }
        else:
            # No AprilTag detected - cannot get position without tag
            return {
                'person_type': 'unknown',
                'name': 'Person without AprilTag',
                'confidence': 0.3,
                'pixel_position': f"x:{x}, y:{y}, w:{w}, h:{h}",
                'world_position': None,  # Cannot determine position without tag
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
            
            # Step 3: Detect red regions (for confidence boost)
            red_regions = self.detect_red_regions(cv_image)

            if len(person_rects) > 0:
                # Always log detection counts if should_log
                if should_log:
                    self.get_logger().info(f'Detected {len(person_rects)} person(s), {len(detected_tags)} AprilTag(s), and {len(red_regions)} red region(s)')

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
                                if world_pos:
                                    self.get_logger().info(f'{"="*80}')
                                    self.get_logger().info(f'{status} PERSON DETECTED: {person_data["person_id"]} (AprilTag {apriltag_id})')
                                    self.get_logger().info(f'WORLD POSITION: [{world_pos[0]:.3f}, {world_pos[1]:.3f}, {world_pos[2]:.3f}]')
                                    self.get_logger().info(f'{"="*80}')
                                else:
                                    self.get_logger().info(f'{"="*80}')
                                    self.get_logger().info(f'{status} PERSON: {person_data["person_id"]} (AprilTag {apriltag_id}) - NO POSITION')
                                    self.get_logger().info(f'{"="*80}')
                    else:
                        # Handle persons without AprilTags (don't publish to avoid duplicates)
                        if should_log:
                            self.get_logger().info(f'Person detected without AprilTag - not publishing to avoid duplicates')
            
            # Handle object detection for AprilTags not matched to persons
            if len(detected_tags) > 0:
                # Find tags that are objects (ID 8+) and not matched to persons
                object_tags = [tag for tag in detected_tags if tag['id'] >= 8 and tag['id'] in self.tag_mapping]
                
                for tag in object_tags:
                    tag_id = tag['id']
                    tag_data = self.tag_mapping[tag_id]
                    
                    # Get position from Gazebo
                    gazebo_model_name = tag_data['gazebo_model']
                    world_coords = self.get_position_from_gazebo(gazebo_model_name)
                    
                    # Check if tag is on red box for confidence boost
                    is_on_red = self.is_tag_on_red_box(tag, red_regions)
                    base_confidence = min(0.95, max(0.5, tag['confidence'] / 50.0))
                    confidence_boost = 0.1 if is_on_red else 0.0
                    
                    # Create object data
                    object_data = {
                        'class': tag_data['class'],
                        'name': tag_data['object'],  # Use 'name' to match get_or_create_unique_object
                        'world_position': world_coords,
                        'tag_center': tag['center'].tolist(),
                        'confidence': min(0.99, base_confidence + confidence_boost),
                        'on_red_box': is_on_red
                    }
                    
                    # Track and publish object
                    unique_id, is_new = self.get_or_create_unique_object(tag_id, object_data)
                    
                    if self.should_publish_object_update(unique_id, is_new):
                        obj_data = self.unique_objects[unique_id]
                        
                        result_msg = VisionObjectDetection()
                        result_msg.sport_class = obj_data['class']
                        result_msg.object = obj_data['object']
                        
                        # Set 3D position
                        result_msg.position = Point()
                        world_pos = obj_data.get('position')
                        if world_pos and len(world_pos) >= 3:
                            result_msg.position.x = float(world_pos[0])
                            result_msg.position.y = float(world_pos[1])
                            result_msg.position.z = float(world_pos[2])
                        else:
                            result_msg.position.x = 0.0
                            result_msg.position.y = 0.0
                            result_msg.position.z = 0.0
                        
                        self.object_publisher.publish(result_msg)
                        
                        if should_log:
                            status = "NEW" if is_new else "UPDATE"
                            if world_pos:
                                self.get_logger().info(f'{"="*60}')
                                self.get_logger().info(f'{status} OBJECT: {obj_data["object"]} ({obj_data["class"]})')
                                self.get_logger().info(f'POSITION: [{world_pos[0]:.3f}, {world_pos[1]:.3f}, {world_pos[2]:.3f}]')
                                self.get_logger().info(f'{"="*60}')
                            else:
                                self.get_logger().info(f'{status} OBJECT: {obj_data["object"]} - NO POSITION')

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