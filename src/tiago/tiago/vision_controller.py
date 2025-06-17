#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import json
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

    # Face detection in person region
    def detect_faces_in_person(self, image, person_rect):
        """Detect faces within a person's bounding box"""
        if self.face_cascade is None:
            return []

        try:
            x, y, w, h = person_rect
            person_roi = image[y:y+h, x:x+w]
            gray_roi = cv2.cvtColor(person_roi, cv2.COLOR_BGR2GRAY)
            
            faces = self.face_cascade.detectMultiScale(gray_roi, 1.1, 4)
            
            # Convert face coordinates back to full image coordinates
            adjusted_faces = []
            for (fx, fy, fw, fh) in faces:
                adjusted_faces.append((x + fx, y + fy, fw, fh))
            
            return adjusted_faces
        except Exception as e:
            self.get_logger().error(f'Error during face detection: {e}')
            return []

    # TODO: Face recognition placeholder
    def recognize_face(self, image, face_rect):
        """Recognize face and return staff info if found"""
        # TODO: Implement actual face recognition with embeddings
        # For now: placeholder logic
        
        # Extract face region for future embedding comparison
        x, y, w, h = face_rect
        face_roi = image[y:y+h, x:x+w]
        
        # Placeholder: random staff recognition for demo
        # In real implementation: compare face_roi embedding with staff_database
        
        return None  # No recognition yet - placeholder

    # TODO: Person classification pipeline
    def classify_person(self, image, person_rect):
        """Complete pipeline: person -> face detection -> face recognition -> classification"""
        
        # Step 1: Detect faces in person region
        faces = self.detect_faces_in_person(image, person_rect)
        
        if len(faces) > 0:
            self.get_logger().info(f'Found {len(faces)} face(s) in person region')
            
            # Step 2: Try face recognition on first detected face
            face_rect = faces[0]  # Use first face
            staff_info = self.recognize_face(image, face_rect)
            
            if staff_info:
                # Step 3a: Recognized as staff
                x, y, w, h = person_rect
                return {
                    'person_type': 'staff',
                    'name': staff_info['name'],
                    'role': staff_info.get('role', 'Unknown'),
                    'confidence': staff_info.get('confidence', 0.9),
                    'position': f"x:{x}, y:{y}, w:{w}, h:{h}",
                    'face_detected': True
                }
            else:
                # Step 3b: Face detected but not recognized as staff
                x, y, w, h = person_rect
                return {
                    'person_type': 'customer',
                    'name': 'Unknown Customer',
                    'confidence': 0.7,
                    'position': f"x:{x}, y:{y}, w:{w}, h:{h}",
                    'face_detected': True
                }
        else:
            # Step 3c: No face detected - assume customer
            x, y, w, h = person_rect
            return {
                'person_type': 'customer',
                'name': 'Unknown Person',
                'confidence': 0.5,
                'position': f"x:{x}, y:{y}, w:{w}, h:{h}",
                'face_detected': False
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