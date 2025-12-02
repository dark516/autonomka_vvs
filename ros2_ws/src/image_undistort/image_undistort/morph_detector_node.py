#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header
from cv_bridge import CvBridge
import time

class MorphDetector(Node):
    def __init__(self):
        super().__init__('morph_detector')
        
        self.declare_parameter('center_x', 960)
        self.declare_parameter('center_y', 540)
        self.declare_parameter('input_image_topic', '/camera_1/image_undistorted/compressed')
        self.declare_parameter('output_topic_1', '')  # Will be set based on marker type
        self.declare_parameter('output_topic_2', '')
        self.declare_parameter('output_topic_debug', '')
        
        self.input_topic       = self.get_parameter('input_image_topic').value
        self.output_topic_1      = self.get_parameter('output_topic_1').value
        self.output_topic_2      = self.get_parameter('output_topic_2').value
        self.output_topic_debug      = self.get_parameter('output_topic_debug').value
        self.center_x          = int(self.get_parameter('center_x').value)
        self.center_y          = int(self.get_parameter('center_y').value)
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Initialize background subtractor
        self.back_sub = cv2.createBackgroundSubtractorMOG2(history=15, varThreshold=30, detectShadows=False)
        
        # Subscriber for compressed image
        self.subscription = self.create_subscription(
            CompressedImage,
            self.input_topic,
            self.image_callback,
            10
        )
        
        # Publishers for blob centers as PointStamped
        self.point1_pub = self.create_publisher(PointStamped, self.output_topic_1, 10)
        self.point2_pub = self.create_publisher(PointStamped,  self.output_topic_2  , 10)
        
        # Publisher for RViz2-compatible uncompressed image
        self.debug_image_rviz_pub = self.create_publisher(Image, self.output_topic_debug , 10)
        
        self.get_logger().info('Morph detector node initialized')


    def clean_and_enlarge_blobs(self, binary_image):
        """
        Remove small white blobs and make remaining blobs larger
        """
        # Use larger kernel for more aggressive cleaning and enlargement
        kernel_ellipse_1 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        kernel_ellipse_2 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))

        binary_image = cv2.morphologyEx(binary_image, cv2.MORPH_CLOSE, kernel_ellipse_1)

        # Remove small blobs (noise)
        cleaned = cv2.morphologyEx(binary_image, cv2.MORPH_OPEN, kernel_ellipse_1)

        # Enlarge remaining blobs and fill holes
        enlarged = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, kernel_ellipse_2)

        return enlarged

    def detect_blob_centers(self, binary_image, min_area=0, max_blobs=2):
        """
        Detect blob centers with area filtering and maximum limit
        """
        # Find contours
        contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Filter contours by area and sort by size (largest first)
        valid_contours = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area >= min_area:
                valid_contours.append(contour)

        # Sort by area (largest first) and limit to max_blobs
        valid_contours = sorted(valid_contours, key=cv2.contourArea, reverse=True)[:max_blobs]

        # Calculate centers
        centers = []
        for contour in valid_contours:
            # Calculate moments for center
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                centers.append((cx, cy))

        return centers, valid_contours

    def draw_blob_centers(self, image, centers, contours):
        """
        Draw blob centers and contours on the image
        """
        result = image.copy()

        # Get frame dimensions
        height, width = result.shape[:2]
        
        # Draw frame center
        cv2.circle(result, (891, 776), 6, (255, 0, 255), -1)
        cv2.circle(result, (891, 776), 10, (255, 255, 255), 2)
        cv2.putText(result, "Frame Center", (891 - 40, 776 - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # Draw centers and relative coordinates
        for i, (cx, cy) in enumerate(centers):
            # Calculate relative coordinates
            rel_x = cx - self.center_x
            rel_y = self.center_y - cy  # Invert Y to match typical coordinate system
            
            # Center point
            cv2.circle(result, (cx, cy), 8, (0, 0, 255), -1)
            cv2.circle(result, (cx, cy), 12, (255, 255, 255), 2)
            
            # Line from frame center to blob center
            cv2.line(result, (self.center_x, self.center_y), (cx, cy), (255, 255, 0), 2)

            # ID label and coordinates
            cv2.putText(result, f"Blob {i + 1}", (cx - 25, cy - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(result, f"({rel_x}, {rel_y})", (cx - 35, cy + 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

        return result

    def image_callback(self, msg):
        """
        Callback function for processing incoming images
        """
        try:
            # Convert compressed image to OpenCV format
            frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Optional: Resize the image (adjust scaling factor as needed)
            frame = cv2.resize(frame, (frame.shape[1] // 3, frame.shape[0] // 3))
            
            # Get frame dimensions
            height, width = frame.shape[:2]
            self.frame_center = (width // 2, height // 2)
            
            # Smooth the original image (Gaussian blur)
            smoothed_frame = cv2.GaussianBlur(frame, (5, 5), 0)

            # Background subtraction
            fg_mask = self.back_sub.apply(smoothed_frame)

            # Convert to binary
            _, binary_mask = cv2.threshold(fg_mask, 127, 255, cv2.THRESH_BINARY)

            # Clean mask using morphology (remove small blobs, enlarge remaining)
            cleaned_mask = self.clean_and_enlarge_blobs(binary_mask)
            
            # Detect blob centers (max 2)
            centers, contours = self.detect_blob_centers(cleaned_mask, min_area=500, max_blobs=2)

            # Create result frame for debug
            result_frame = self.draw_blob_centers(frame, centers, contours)
            
            # Display blob count and frame center info
            cv2.putText(result_frame, f"Blobs detected: {len(centers)}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            # Publish blob centers as PointStamped (only if blobs are detected)
            self.publish_blob_centers(centers, msg.header)
            
            # Publish RViz2-compatible image
            self.publish_rviz_debug_image(result_frame)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def publish_blob_centers(self, centers, img_header):
        """
        Publish blob centers as PointStamped messages with coordinates relative to frame center
        Only publishes when blobs are actually detected
        """
        # Get frame center
        if hasattr(self, 'frame_center'):
            center_x, center_y = self.frame_center
        else:
            # Fallback if frame center not calculated
            center_x, center_y = 0, 0
        
        # Publish point1 if at least 1 blob is detected
        if len(centers) >= 1:
            point1 = PointStamped()
            point1.header = img_header
            
            # Calculate relative coordinates (inverted Y to match typical coordinate system)
            point1.point.x = float(centers[0][0] - self.center_x) * 3 * 0.004267
            point1.point.y = float(self.center_y - centers[0][1]) * 3 * 0.004267  # Invert Y axis
            point1.point.z = 0.2  # Fixed Z coordinate as requested
            
            self.point1_pub.publish(point1)
            self.get_logger().info(f'Published point1: ({point1.point.x:.1f}, {point1.point.y:.1f}, {point1.point.z:.1f})', 
                                  throttle_duration_sec=1.0)
        # else: No publication for point1
        
        # Publish point2 if at least 2 blobs are detected
        if len(centers) >= 2:
            point2 = PointStamped()
            point2.header = img_header
            
            point2.point.x = float(centers[1][0] - self.center_x) * 3 * 0.004267
            point2.point.y = float(self.center_y - centers[1][1]) * 3 * 0.004267 
            point2.point.z = 0.2  # Fixed Z coordinate as requested
            
            self.point2_pub.publish(point2)
            self.get_logger().info(f'Published point2: ({point2.point.x:.1f}, {point2.point.y:.1f}, {point2.point.z:.1f})')
        # else: No publication for point2

    def publish_compressed_debug_image(self, image):
        """
        Publish debug image as compressed image
        """
        try:
            debug_msg = self.bridge.cv2_to_compressed_imgmsg(image)
            debug_msg.header.stamp = self.get_clock().now().to_msg()
            self.debug_image_compressed_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing compressed debug image: {str(e)}')

    def publish_rviz_debug_image(self, image):
        """
        Publish debug image as uncompressed Image message for RViz2
        """
        try:
            # Convert BGR to RGB for better color representation in RViz2
            rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            
            # Create Image message
            debug_msg = self.bridge.cv2_to_imgmsg(rgb_image, encoding='rgb8')
            debug_msg.header.stamp = self.get_clock().now().to_msg()
            debug_msg.header.frame_id = 'camera_frame'  # Change this to your camera frame if different
            
            self.debug_image_rviz_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing RViz2 debug image: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    morph_detector = MorphDetector()
    
    try:
        rclpy.spin(morph_detector)
    except KeyboardInterrupt:
        pass
    finally:
        morph_detector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
