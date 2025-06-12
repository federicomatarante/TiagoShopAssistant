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
        
        # Debug timer
        self.timer = self.create_timer(10.0, self.timer_callback)
        
        self.get_logger().info('Vision Controller initialized')
    
    def timer_callback(self):
        self.get_logger().info("Timer callback - node is alive")
    
    def image_callback(self, msg):
        """Process incoming camera images"""
        try:
            # Debug: Always publish something to test pipeline
            # debug_msg = String()
            # debug_msg.data = f"Callback triggered at {self.get_clock().now().nanoseconds}"
            # self.result_publisher.publish(debug_msg)
            
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Only do face detection if cascade loaded successfully
            if self.face_cascade is not None:
                gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
                faces = self.face_cascade.detectMultiScale(gray, 1.1, 4)
                
                if len(faces) > 0:
                    # Publish detection result
                    result_msg = String()
                    result_msg.data = f"Detected {len(faces)} faces"
                    self.result_publisher.publish(result_msg)
                    self.get_logger().info(f'Published: {result_msg.data}')
            else:
                # Placeholder - just confirm image processing works
                self.get_logger().info(f'Image processed: {cv_image.shape}', throttle_duration_sec=5.0)
                
                # Debug: publish image processing status
                result_msg = String()
                result_msg.data = f"Image processed: {cv_image.shape}, no face detection available"
                self.result_publisher.publish(result_msg)
                
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
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()