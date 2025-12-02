#!/usr/bin/env python3
import math
import traceback
import numpy as np
import cv2
import cv2.aruco as aruco
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseStamped, PointStamped, Point


class ArucoDetectorCompressed(Node):
    def __init__(self):
        super().__init__('aruco_detector_compressed')

        # -------- params --------
        self.declare_parameter('camera_name', 'camera_1')
        self.declare_parameter('input_image_topic', '/camera_1/image_undistorted/compressed')
        self.declare_parameter('output_topic', '')  # Will be set based on marker type
        self.declare_parameter('debug_image_topic', '')  # Separate debug for each instance
        self.declare_parameter('aruco_id', -1)  # Single marker ID to find
        self.declare_parameter('message_type', 'pose')  # 'pose' or 'point_stamped'
        self.declare_parameter('encode_format', 'jpeg')
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('png_compression', 3)
        self.declare_parameter('frame_id', '')
        self.declare_parameter('marker_name', 'marker')  # For visualization
        self.declare_parameter('center_dot_color', 'blue')
        self.declare_parameter('center_x', 960)
        self.declare_parameter('center_y', 540)


        self.camera_name       = self.get_parameter('camera_name').value
        self.input_topic       = self.get_parameter('input_image_topic').value
        self.output_topic      = self.get_parameter('output_topic').value
        self.debug_topic       = self.get_parameter('debug_image_topic').value
        self.aruco_id          = int(self.get_parameter('aruco_id').value)
        self.message_type      = str(self.get_parameter('message_type').value)
        self.marker_name       = str(self.get_parameter('marker_name').value)
        self.enc_fmt           = str(self.get_parameter('encode_format').value).lower()
        self.jpeg_q            = int(self.get_parameter('jpeg_quality').value)
        self.png_c             = int(self.get_parameter('png_compression').value)
        self.frame_id_param    = str(self.get_parameter('frame_id').value)
        self.center_dot_color  = str(self.get_parameter('center_dot_color').value).lower()
        self.center_x          = int(self.get_parameter('center_x').value)
        self.center_y          = int(self.get_parameter('center_y').value)
        # Validate parameters
        if self.aruco_id == -1:
            self.get_logger().error("Must specify aruco_id parameter!")
            raise ValueError("aruco_id must be specified")
        
        if not self.output_topic:
            self.get_logger().error("Must specify output_topic parameter!")
            raise ValueError("output_topic must be specified")

        # -------- aruco detector --------
        dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)
        detector_params = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(dictionary, detector_params)

        self.last_center = None
        self.last_heading = None

        # -------- pub/sub --------
        sub_qos = qos_profile_sensor_data
        pub_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE,
                             history=QoSHistoryPolicy.KEEP_LAST)

        # Create appropriate publisher based on message type
        if self.message_type == 'pose':
            self.publisher = self.create_publisher(PoseStamped, self.output_topic, pub_qos)
            self.get_logger().info(f"Created PoseStamped publisher for {self.output_topic}")
        elif self.message_type == 'point_stamped':
            self.publisher = self.create_publisher(PointStamped, self.output_topic, pub_qos)
            self.get_logger().info(f"Created PointStamped publisher for {self.output_topic}")
        else:
            self.get_logger().error(f"Unknown message_type: {self.message_type}")
            raise ValueError(f"Unknown message_type: {self.message_type}")

        # Only create debug publisher if debug topic is specified
        if self.debug_topic:
            self.debug_pub = self.create_publisher(CompressedImage, self.debug_topic, pub_qos)
        else:
            self.debug_pub = None

        self.sub = self.create_subscription(CompressedImage, self.input_topic, self.image_cb, sub_qos)

        self.get_logger().info(f"[init] camera={self.camera_name}")
        self.get_logger().info(f"[init] tracking {self.marker_name} ID: {self.aruco_id}")
        self.get_logger().info(f"[init] subscribe: {self.input_topic}")
        self.get_logger().info(f"[init] publish to: {self.output_topic} ({self.message_type})")
        if self.debug_topic:
            self.get_logger().info(f"[init] publish debug: {self.debug_topic}")
        self.get_logger().info(f"[init] center dot color: {self.center_dot_color}")

    # ---------- utils ----------
    @staticmethod
    def wrap_angle_deg(a):
        return (a + 360.0) % 360.0

    def heading_from_vector(self, dx, dy):
        return self.wrap_angle_deg(math.degrees(math.atan2(-dy, dx)))

    def heading_from_aruco_corners(self, corners):
        pts = corners.reshape(-1, 2).astype(np.float32)
        TL, TR = pts[0], pts[1]
        dx, dy = TR[0] - TL[0], TR[1] - TL[1]

        return self.heading_from_vector(dx, dy)

    def _decode(self, msg: CompressedImage):
        if not msg.data:
            return None
        arr = np.frombuffer(msg.data, np.uint8)
        return cv2.imdecode(arr, cv2.IMREAD_COLOR)

    def _encode_debug(self, img: np.ndarray):
        if self.enc_fmt in ('jpeg', 'jpg'):
            ok, enc = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, np.clip(self.jpeg_q, 0, 100)])
            fmt = 'jpeg'
        elif self.enc_fmt == 'png':
            ok, enc = cv2.imencode('.png', img, [cv2.IMWRITE_PNG_COMPRESSION, np.clip(self.png_c, 0, 9)])
            fmt = 'png'
        else:
            ok, enc = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, np.clip(self.jpeg_q, 0, 100)])
            fmt = 'jpeg'
        if not ok or enc is None:
            raise RuntimeError("cv2.imencode failed for debug image")
        return enc.tobytes(), fmt

    def get_color(self, color_name):
        """Convert color name to BGR tuple"""
        colors = {
            'blue': (255, 0, 0),
            'green': (0, 255, 0),
            'red': (0, 0, 255),
            'yellow': (0, 255, 255),
            'cyan': (255, 255, 0),
            'magenta': (255, 0, 255),
            'white': (255, 255, 255),
            'black': (0, 0, 0),
            'orange': (0, 165, 255),
            'purple': (128, 0, 128)
        }
        return colors.get(color_name, (255, 0, 0))  # Default to blue

    # ---------- main ----------
    def image_cb(self, msg: CompressedImage):
        try:
            frame = self._decode(msg)
            if frame is None:
                self.get_logger().warn("[decode] imdecode returned None — skip")
                return

            message = self.detect_aruco_and_make_message(frame, msg.header)
            if message is not None:
                self.publisher.publish(message)
                self.get_logger().debug(f"Published {type(message).__name__} to {self.output_topic}")
            
            # Always publish debug image if debug topic is set
            if self.debug_pub is not None:
                debug_image = self.create_debug_image(frame, msg.header)
                self.publish_debug_image(debug_image, msg.header)

        except Exception as e:
            self.get_logger().error(f"[error] {e}\n{traceback.format_exc()}")

    # ---------- detection & pose ----------
    def detect_aruco_and_make_message(self, image, header):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        message = None

        if ids is not None and len(ids) > 0:
            # Look for our specific marker
            for i, marker_id in enumerate(ids):
                if int(marker_id[0]) == self.aruco_id:
                    c = corners[i]
                    pts = c.reshape(-1, 2).astype(np.float32)
                    cx, cy = float(np.mean(pts[:, 0])), float(np.mean(pts[:, 1]))
                    
                    # Create appropriate message
                    if self.message_type == 'pose':
                        heading_deg = self.heading_from_aruco_corners(c)
                        self.last_center, self.last_heading = (cx, cy), heading_deg
                        message = self.pose_stamped_from_center_heading(
                            (cx, cy), heading_deg, image.shape[1], image.shape[0], header
                        )
                    else:  # point_stamped
                        self.last_center = (cx, cy)
                        message = self.point_stamped_from_center(
                            (cx, cy), image.shape[1], image.shape[0], header
                        )
                    
                    self.get_logger().info(f"Found {self.marker_name} ID:{self.aruco_id} at ({cx:.1f}, {cy:.1f})")
                    break
            else:
                self.get_logger().debug(f"Marker {self.aruco_id} not found in frame")

        return message

    def create_debug_image(self, image, header):
        debug_image = image.copy()
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        if ids is not None and len(ids) > 0:
            # Draw all markers
            debug_image = aruco.drawDetectedMarkers(debug_image, corners, ids)

            # Look for our specific marker to add custom visualization
            for i, marker_id in enumerate(ids):
                if int(marker_id[0]) == self.aruco_id:
                    c = corners[i]
                    pts = c.reshape(-1, 2).astype(np.float32)
                    cx, cy = float(np.mean(pts[:, 0])), float(np.mean(pts[:, 1]))

                    if self.message_type == 'pose':
                        heading_deg = self.heading_from_aruco_corners(c)
                        self.draw_pose_visualization(debug_image, (cx, cy), c, heading_deg)
                    else:  # point_stamped
                        self.draw_point_visualization(debug_image, (cx, cy))
                    break

        # Add camera and marker info
        cv2.putText(debug_image, f"{self.camera_name} - {self.marker_name}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        status = f"ID:{self.aruco_id} - Active" if self.last_center else f"ID:{self.aruco_id} - Not found"
        cv2.putText(debug_image, status, (10, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        # ADD CENTER DOT
        self.draw_center_dot(debug_image)

        return debug_image

    def draw_center_dot(self, image):
        """Draw a colored dot at the center of the image"""
        height, width = image.shape[:2]
        center_x, center_y = width // 2, height // 2
        
        # Get the color for the center dot
        color = self.get_color(self.center_dot_color)
        
        # Draw a filled circle at the center
        cv2.circle(image, (center_x, center_y), 8, color, -1)  # -1 means filled circle
        
        # Draw a white outline for better visibility
        cv2.circle(image, (center_x, center_y), 8, (255, 255, 255), 2)
        
        # Add text label near the center dot
        cv2.putText(image, "CENTER", (center_x + 15, center_y - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(image, "CENTER", (center_x + 15, center_y - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1)

    def draw_pose_visualization(self, image, center, corners, heading_deg):
        center_int = (int(round(center[0])), int(round(center[1])))
        # Orientation arrow (green for robot)
        heading_rad = math.radians(heading_deg)
        arrow_len = 50
        end_x = int(center_int[0] + arrow_len * math.cos(heading_rad))
        end_y = int(center_int[1] - arrow_len * math.sin(heading_rad))
        cv2.arrowedLine(image, center_int, (end_x, end_y), (0, 255, 0), 3, tipLength=0.3)

        # Center (green for robot)
        cv2.circle(image, center_int, 5, (0, 255, 0), -1)

        # Text
        cv2.putText(image, f"{self.marker_name} ID:{self.aruco_id}", 
                    (center_int[0] - 60, center_int[1] - 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        cv2.putText(image, f"{self.marker_name} ID:{self.aruco_id}", 
                    (center_int[0] - 60, center_int[1] - 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(image, f"Heading:{heading_deg:.1f}°", 
                    (center_int[0] - 60, center_int[1] - 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        cv2.putText(image, f"Heading:{heading_deg:.1f}°", 
                    (center_int[0] - 60, center_int[1] - 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    def draw_point_visualization(self, image, center):
        center_int = (int(round(center[0])), int(round(center[1])))
        # Center (red for enemy)
        cv2.circle(image, center_int, 5, (0, 0, 255), -1)
        
        # Cross for enemy
        cross_size = 15
        cv2.line(image, 
                (center_int[0] - cross_size, center_int[1] - cross_size),
                (center_int[0] + cross_size, center_int[1] + cross_size),
                (0, 0, 255), 3)
        cv2.line(image,
                (center_int[0] - cross_size, center_int[1] + cross_size),
                (center_int[0] + cross_size, center_int[1] - cross_size),
                (0, 0, 255), 3)

        # Text
        cv2.putText(image, f"{self.marker_name} ID:{self.aruco_id}", 
                    (center_int[0] - 60, center_int[1] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        cv2.putText(image, f"{self.marker_name} ID:{self.aruco_id}", 
                    (center_int[0] - 60, center_int[1] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

    def pose_stamped_from_center_heading(self, center, heading_deg, img_w, img_h, header):
        ps = PoseStamped()
        ps.header = header
        if not ps.header.frame_id:
            ps.header.frame_id = self.frame_id_param or f"{self.camera_name}_optical_frame"

        normalized_z = 0.3
        normalized_x = (center[0] - self.center_x) * 0.004267
        normalized_y = -(center[1] - self.center_y) * 0.004267

        ps.pose.position.x = normalized_x
        ps.pose.position.y = normalized_y
        ps.pose.position.z = normalized_z

        yaw_rad = math.radians(heading_deg)
        ps.pose.orientation.w = math.cos(yaw_rad / 2.0)
        ps.pose.orientation.x = 0.0
        ps.pose.orientation.y = 0.0
        ps.pose.orientation.z = math.sin(yaw_rad / 2.0)

        return ps

    def point_stamped_from_center(self, center, img_w, img_h, header):
        point_stamped = PointStamped()
        point_stamped.header = header
        if not point_stamped.header.frame_id:
            point_stamped.header.frame_id = self.frame_id_param or f"{self.camera_name}_optical_frame"

        normalized_z = 0.3
        normalized_x = (center[0] - self.center_x) * 0.004267
        normalized_y = -(center[1] - self.center_y) * 0.004267

        point_stamped.point.x = normalized_x
        point_stamped.point.y = normalized_y
        point_stamped.point.z = normalized_z

        return point_stamped


    def publish_debug_image(self, image, header):
        try:
            data_bytes, fmt = self._encode_debug(image)
            out = CompressedImage()
            out.header = header
            out.format = fmt
            out.data = data_bytes
            self.debug_pub.publish(out)
        except Exception as e:
            self.get_logger().error(f"[debug] publish failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorCompressed()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("shutdown")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
