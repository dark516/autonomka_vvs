#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageClickCoordinator(Node):
    def __init__(self):
        super().__init__('image_click_coordinator')
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Variables to store the current image and click coordinates
        self.current_image = None
        self.click_x = None
        self.click_y = None
        
        # Subscribe to image topic (change topic name as needed)
        self.subscription = self.create_subscription(
            Image,
            '/camera_1/image_undistorted/compressed',  # Adjust this to your image topic
            self.image_callback,
            10)
        
        # Mouse callback setup
        self.window_name = "Image Click Coordinator"
        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)
        
        self.get_logger().info("Image click coordinator started. Click on the image and press 'q' to exit.")
        
    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.current_image = cv_image.copy()
            
            # If we have click coordinates, draw them on the image
            if self.click_x is not None and self.click_y is not None:
                self.draw_crosshair(self.current_image, self.click_x, self.click_y)
            
            # Display the image
            cv2.imshow(self.window_name, self.current_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")
    
    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            # Store click coordinates
            self.click_x = x
            self.click_y = y
            
            self.get_logger().info(f"Clicked at pixel coordinates: (x={x}, y={y})")
            
            # Update the displayed image with the crosshair
            if self.current_image is not None:
                display_image = self.current_image.copy()
                self.draw_crosshair(display_image, x, y)
                cv2.imshow(self.window_name, display_image)
    
    def draw_crosshair(self, image, x, y):
        """Draw a crosshair at the specified coordinates"""
        color = (0, 255, 0)  # Green color
        thickness = 2
        
        # Draw cross lines
        cv2.line(image, (x-15, y), (x+15, y), color, thickness)
        cv2.line(image, (x, y-15), (x, y+15), color, thickness)
        
        # Draw a circle at the center
        cv2.circle(image, (x, y), 5, color, thickness)
        
        # Add coordinates text
        coord_text = f"({x}, {y})"
        cv2.putText(image, coord_text, (x+20, y-20), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, thickness)
    
    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    node = ImageClickCoordinator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()