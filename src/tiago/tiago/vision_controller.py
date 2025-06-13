#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2

class VisionController(Node):
    def __init__(self):
        super().__init__('vision_controller')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        # TODO: do we want to handle parameters as below in a config file?
        self.simulation_mode = True
        
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

        # Webcam accessed in simulation mode (pc webcam)
        if self.simulation_mode:
            try:
                self.webcam = cv2.VideoCapture(0)
                if self.webcam.isOpened():
                    self.get_logger().info('Webcam initialized for simulation mode')
                else:
                    self.get_logger().warn('Failed to open webcam')
                    selfg.webcam = None
            except Exception as e:
                self.get_logger().error(f'Error initializing webcam: {e}')
                self.webcam = None
        else:
            self.webcam = None

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
            return None

        try:
            # HOG detection
            (rects, weights) = self.person_detector.detectMultiScale(
                image, 
                winStride=(4, 4), 
                padding=(8, 8), 
                scale=1.05
            )

            return len(rects) > 0
        except Exception as e:
            self.get_logger().error(f'Error during person detection: {e}')
            return False

    def capture_webcam_frame(self):
        """Capture frame from PC webcam"""
        if self.webcam is None or not self.webcam.isOpened():
            return None

        try:
            ret, frame = self.webcam.read()
            if ret:
                return frame
            else:
                self.get_logger().warn('Failed to read frame from webcam')
                return None
        except Exception as e:
            self.get_logger().error(f'Error capturing webcam frame: {e}')
            return None

    def recognize_staff(self, image):
        """Recognize staff members from face embeddings"""
        # TODO: implement face recognition using embeddings
        # For now return a placeholder
        if self.face_cascade is None:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            faces = self.face_cascade.detectMultiScale(gray, 1.1, 4)
            if len(faces) > 0:
                return "Staff member detected"
        return "Unknown person"
    
    def timer_callback(self):
        self.get_logger().info("Timer callback - node is alive")
    
    def image_callback(self, msg):
        """Process incoming camera images"""
        try:            
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Person detection on tiago's camera
            people_detected = self.detect_persons(cv_image)

            if people_detected:
                self.get_logger().info('Person detected in TIAGo camera')

                # Choose image source based on simulation mode
                if self.simulation_mode:
                    # Use pc webcam in simulation mode
                    face_image = self.capture_webcam_frame()
                    if face_image is not None:
                        self.get_logger().info('Captured frame from webcam')
                    else:
                        self.get_logger().warn('No frame captured from webcam, using TIAGo camera image')
                        face_image = cv_image
                else:
                    # Use TIAGo's camera image in real mode
                    face_image = cv_image

                # Face detection and recognition
                if face_iamge is not None and self.face_cascade is not None:
                    gray = cv2.cvtColor(face_image, cv2.COLOR_BGR2GRAY)
                    faces = self.face_cascade.detectMultiScale(gray, 1.1, 4)

                    if len(faces) > 0:
                        # Face recognition
                        staff_id = self.recognize_staff(face_image)

                        # Publish result
                        result_msg = String()
                        result_msg.data = f"Person detected, faces: {len(faces)}, staff: {staff_id}"
                        self.result_publisher.publish(result_msg)
                        self.get_logger().info(f'Published: {result_msg.data}')
                    else:
                        # Person detected but no faces
                        result_msg = String()
                        result_msg.data = "Person detected, but no faces found"
                        self.result_publisher.publish(result_msg)
                else:
                    # Person detected but no face detection available
                    result_msg = String()
                    result_msg.data = "Person detected, but face detection not available"
                    self.result_publisher.publish(result_msg)
            else:
                # No person detected - publish status occasionally
                self.get_logger().info(f'No persons detected, image shape: {tiago_image.shape}', 
                                     throttle_duration_sec=10.0)
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')
                        

def main(args=None):
    rclpy.init(args=args)
    node = VisionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup webcam
        if hasattr(vision_controller, 'webcam') and vision_controller.webcam:
            vision_controller.webcam.release()
        
        vision_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()