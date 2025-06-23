#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import json
import numpy as np
import cv2
import apriltag

class VisionController(Node):
    def __init__(self):
        super().__init__('vision_controller')
        
        # Initialize CV bridge
        self.bridge = CvBridge()

        self.log_counter = 0
        
        # Subscribe to TIAGo's camera
        self.image_subscription = self.create_subscription(
            Image,
            '/head_front_camera/rgb/image_raw',
            self.image_callback,
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

    # # Face detection in person region
    # def detect_faces_in_person(self, image, person_rect):
    #     """Detect faces within a person's bounding box"""
    #     if self.face_cascade is None:
    #         return []

    #     try:
    #         x, y, w, h = person_rect
    #         person_roi = image[y:y+h, x:x+w]
    #         gray_roi = cv2.cvtColor(person_roi, cv2.COLOR_BGR2GRAY)
            
    #         faces = self.face_cascade.detectMultiScale(gray_roi, 1.1, 4)
            
    #         # Convert face coordinates back to full image coordinates
    #         adjusted_faces = []
    #         for (fx, fy, fw, fh) in faces:
    #             adjusted_faces.append((x + fx, y + fy, fw, fh))
            
    #         return adjusted_faces
    #     except Exception as e:
    #         self.get_logger().error(f'Error during face detection: {e}')
    #         return []

    # Face recognition placeholder
    # def recognize_face(self, image, face_rect):
    #     """Recognize face and return staff info if found"""
    #     # Implement actual face recognition with embeddings
    #     # For now: placeholder logic
        
    #     # Extract face region for future embedding comparison
    #     x, y, w, h = face_rect
    #     face_roi = image[y:y+h, x:x+w]
        
    #     # Placeholder: random staff recognition for demo
    #     # In real implementation: compare face_roi embedding with staff_database
        
    #     return None  # No recognition yet - placeholder

    # Badge color detection method
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

    def classify_person_with_tag(self, image, person_rect, tag_info=None):
        """Complete pipeline: person -> AprilTag detection -> classification"""
        
        x, y, w, h = person_rect
        
        if tag_info and tag_info['id'] in self.tag_mapping:
            # AprilTag detected and recognized
            tag_data = self.tag_mapping[tag_info['id']]
            confidence = min(0.95, max(0.5, tag_info['confidence'] / 50.0))  # Normalize confidence
            
            return {
                'person_type': tag_data['type'],
                'name': tag_data['name'],
                'tag_id': tag_info['id'],
                'confidence': confidence,
                'position': f"x:{x}, y:{y}, w:{w}, h:{h}",
                'tag_center': tag_info['center'].tolist(),  # Convert numpy array to list
                'tag_confidence': tag_info['confidence'],
                'detection_method': 'apriltag'
            }
        elif tag_info:
            # AprilTag detected but unknown ID
            return {
                'person_type': 'unknown',
                'name': f'Unknown Tag ID {tag_info["id"]}',
                'tag_id': tag_info['id'],
                'confidence': 0.8,
                'position': f"x:{x}, y:{y}, w:{w}, h:{h}",
                'tag_center': tag_info['center'].tolist(),
                'detection_method': 'apriltag_unknown'
            }
        else:
            # No AprilTag detected
            return {
                'person_type': 'unknown',
                'name': 'Person without badge',
                'confidence': 0.3,
                'position': f"x:{x}, y:{y}, w:{w}, h:{h}",
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
                        match['tag']
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