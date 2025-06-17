#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import json
import numpy as np
import cv2

class VisionController(Node):
    def __init__(self):
        super().__init__('vision_controller')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
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

        # TODO: staff embeddings handling

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

    # # TODO: Face recognition placeholder
    # def recognize_face(self, image, face_rect):
    #     """Recognize face and return staff info if found"""
    #     # TODO: Implement actual face recognition with embeddings
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

    # classify_person method - badge detection instead of face detection
    def classify_person(self, image, person_rect):
        """Complete pipeline: person -> badge detection -> classification"""
        
        # Detect badge type instead of faces
        badge_type = self.detect_badge_type(image, person_rect)
        
        x, y, w, h = person_rect
        
        if badge_type == 'staff':
            # Staff classification based on badge
            return {
                'person_type': 'staff',
                'name': 'Staff Member',  # CHANGED: from face recognition result
                'role': 'Employee',
                'confidence': 0.9,       # CHANGED: high confidence for badge detection
                'position': f"x:{x}, y:{y}, w:{w}, h:{h}",
                'detection_method': 'badge'  # NEW: track detection method
            }
        elif badge_type == 'customer':
            # Customer classification based on badge
            return {
                'person_type': 'customer',
                'name': 'Customer',      # CHANGED: from "Unknown Customer"
                'confidence': 0.9,       # CHANGED: high confidence for badge detection
                'position': f"x:{x}, y:{y}, w:{w}, h:{h}",
                'detection_method': 'badge'  # NEW: track detection method
            }
        else:
            # Unknown badge/no badge detected
            return {
                'person_type': 'unknown',        # CHANGED: from 'customer'
                'name': 'Unknown Person',
                'confidence': 0.3,               # CHANGED: low confidence
                'position': f"x:{x}, y:{y}, w:{w}, h:{h}",
                'detection_method': 'no_badge'   # NEW: track detection method
            }
    
    def timer_callback(self):
        self.get_logger().info("Timer callback - node is alive")
    
    def image_callback(self, msg):
        """Process incoming camera images - Face Recognition Pipeline"""
        try:            
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Step 1: Detect persons
            person_rects = self.detect_persons(cv_image)

            if len(person_rects) > 0:
                self.get_logger().info(f'Detected {len(person_rects)} person(s)')

                # Step 2-4: Process each person through face pipeline
                for rect in person_rects:
                    result = self.classify_person(cv_image, rect)
                    
                    # Publish result
                    result_msg = String()
                    result_msg.data = json.dumps(result)
                    self.result_publisher.publish(result_msg)
                    
                    # Log result
                    person_type = result['person_type']
                    name = result.get('name', 'Unknown')
                    face_status = "with face" if result['face_detected'] else "no face"
                    self.get_logger().info(f'Classified {person_type}: {name} ({face_status})')

            else:
                self.get_logger().info(f'No persons detected, image shape: {cv_image.shape}')

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