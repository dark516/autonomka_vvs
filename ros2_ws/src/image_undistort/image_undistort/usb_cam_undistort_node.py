#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import CompressedImage, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import traceback
from threading import Lock


class USBCamUndistortNode(Node):
    def __init__(self):
        super().__init__('usb_cam_undistort_node')

        # ---------------- Parameters ----------------
        self.declare_parameter('camera_name', 'camera_1')
        self.declare_parameter('video_device', '/dev/video0')
        self.declare_parameter('output_topic', '/camera/image_undistorted/compressed')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('calibration_file', 'fisheye_calibration.npz')
        
        # Camera parameters
        self.declare_parameter('image_width', 1920)
        self.declare_parameter('image_height', 1080)
        self.declare_parameter('framerate', 60.0)
        self.declare_parameter('brightness', 108)
        self.declare_parameter('auto_white_balance', 0)
        self.declare_parameter('white_balance', 4050)
        self.declare_parameter('autoexposure', 0)
        self.declare_parameter('exposure', 250)
        self.declare_parameter('autofocus', 0)
        self.declare_parameter('K', [806.000000 / 2, 0.0, 1738.000000 / 2, 0.0, 762.000000 / 2, 1038.000000 / 2, 0.0, 0.0, 1.0])
        self.declare_parameter('D', [0.000001, 0.000000, 0.000431, 0.000043])
        
        # Output and encoding parameters
        self.declare_parameter('output_width', 0)  # 0 = same as input
        self.declare_parameter('output_height', 0)  # 0 = same as input
        self.declare_parameter('encode_format', 'jpeg')
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('png_compression', 3)

        # Get parameters
        self.camera_name = self.get_parameter('camera_name').value
        self.video_device = self.get_parameter('video_device').value
        output_topic = self.get_parameter('output_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        calibration_file = self.get_parameter('calibration_file').value
        
        # Camera params
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        self.framerate = self.get_parameter('framerate').value
        self.brightness = self.get_parameter('brightness').value
        self.auto_white_balance = self.get_parameter('auto_white_balance').value
        self.white_balance = self.get_parameter('white_balance').value
        self.autoexposure = self.get_parameter('autoexposure').value
        self.exposure = self.get_parameter('exposure').value
        self.autofocus = self.get_parameter('autofocus').value
        self.K = self.get_parameter('K').value
        self.D = self.get_parameter('D').value
        self.K = np.array([[self.K[0], self.K[1], self.K[2]], [self.K[3], self.K[4], self.K[5]], [self.K[6], self.K[7], self.K[8]]], dtype=np.float32)
        self.D = np.array(self.D, dtype=np.float32)
        
        # Output params
        self.out_w = int(self.get_parameter('output_width').value)
        self.out_h = int(self.get_parameter('output_height').value)
        self.encode_format = str(self.get_parameter('encode_format').value).lower()
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self.png_compression = int(self.get_parameter('png_compression').value)
        
        # self.K = np.array([[806.000000 / 2, 0.0, 1738.000000 / 2], [0.0, 762.000000 / 2, 1038.000000 / 2], [0.0, 0.0, 1.0]], dtype=np.float32)
        # self.D = np.array([0.000001, 0.000000, 0.000431, 0.000043], dtype=np.float32)
        self.calib_image_size = (1920, 1080)

        # ---------------- Camera setup ----------------
        self.cap = None
        self.cv_bridge = CvBridge()
        self.camera_lock = Lock()
        
        self._setup_camera()
        
        # Undistortion maps (will be created on first frame)
        self.map1 = None
        self.map2 = None
        self.output_size = None

        # ---------------- QoS ----------------
        pub_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST
        )

        # ---------------- Publishers ----------------
        self.image_pub = self.create_publisher(CompressedImage, output_topic, pub_qos)
        self.camera_info_pub = self.create_publisher(CameraInfo, camera_info_topic, pub_qos)
        
        # Camera info message (basic)
        self.camera_info = CameraInfo()
        self.camera_info.header.frame_id = f"{self.camera_name}_optical_frame"
        self.camera_info.width = self.out_w if self.out_w > 0 else self.image_width
        self.camera_info.height = self.out_h if self.out_h > 0 else self.image_height
        # You can populate the camera matrix from calibration if needed

        # ---------------- Timer for capture loop ----------------
        timer_period = 1.0 / self.framerate
        self.timer = self.create_timer(timer_period, self.capture_and_publish)
        
        self.get_logger().info(f"Started USB camera with undistortion for {self.camera_name}")
        self.get_logger().info(f"Device: {self.video_device}, Resolution: {self.image_width}x{self.image_height}")
        self.get_logger().info(f"Output: {output_topic}, Output size: {self.out_w}x{self.out_h}")

    def _setup_camera(self):
        """Initialize the USB camera with parameters"""
        try:
            self.cap = cv2.VideoCapture(self.video_device)
            
            if not self.cap.isOpened():
                raise RuntimeError(f"Could not open video device {self.video_device}")
            
            # Set camera parameters
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)
            self.cap.set(cv2.CAP_PROP_FPS, self.framerate)
            self.cap.set(cv2.CAP_PROP_BRIGHTNESS, self.brightness) # Normalize to 0-1
            self.cap.set(cv2.CAP_PROP_AUTO_WB, self.auto_white_balance)
            self.cap.set(cv2.CAP_PROP_WB_TEMPERATURE, self.white_balance)
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, self.autoexposure)
            self.cap.set(cv2.CAP_PROP_EXPOSURE, self.exposure)
            self.cap.set(cv2.CAP_PROP_AUTOFOCUS, self.autofocus )
            # Note: OpenCV doesn't always support all V4L2 controls directly
            # You might need to use v4l2-ctl commands for some parameters
            
            self.get_logger().info("Camera initialized successfully")
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize camera: {e}")
            raise

    def _ensure_maps(self, in_shape_wh):
        """Create undistortion maps on first frame"""
        in_w, in_h = in_shape_wh
        if self.out_w <= 0 or self.out_h <= 0:
            self.output_size = (in_w, in_h)  # Use input size
        else:
            self.output_size = (self.out_w, self.out_h)

        new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(self.K, self.D, (1920, 1080), 1, (1920, 1080))
        
        # Generate undistortion maps
        self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(self.K, self.D, np.eye(3), new_camera_matrix, (1920, 1080), cv2.CV_16SC2)
        
        self.get_logger().info(f"Undistortion maps created. Output size: {self.output_size}")
        
        if self.calib_image_size and (in_w, in_h) != tuple(self.calib_image_size):
            self.get_logger().warn(
                f"Input frame size {in_w}x{in_h} differs from calibration image_size {self.calib_image_size}. "
                f"Undistortion may be suboptimal."
            )

    def _encode_compressed(self, img: np.ndarray):
        """Encode image to compressed format"""
        if self.encode_format in ('jpeg', 'jpg'):
            params = [cv2.IMWRITE_JPEG_QUALITY, int(np.clip(self.jpeg_quality, 0, 100))]
            ok, enc = cv2.imencode('.jpg', img, params)
            fmt = 'jpeg'
        elif self.encode_format == 'png':
            params = [cv2.IMWRITE_PNG_COMPRESSION, int(np.clip(self.png_compression, 0, 9))]
            ok, enc = cv2.imencode('.png', img, params)
            fmt = 'png'
        else:
            # Fallback to jpeg
            params = [cv2.IMWRITE_JPEG_QUALITY, int(np.clip(self.jpeg_quality, 0, 100))]
            ok, enc = cv2.imencode('.jpg', img, params)
            fmt = 'jpeg'

        if not ok or enc is None:
            raise RuntimeError("cv2.imencode failed")
        return enc.tobytes(), fmt

    def capture_and_publish(self):
        """Main capture and processing loop"""
        try:
            with self.camera_lock:
                ret, frame = self.cap.read()
                
            if not ret:
                self.get_logger().warn("Failed to capture frame from camera")
                return

            # Convert BGR to RGB if needed (OpenCV uses BGR, ROS typically expects RGB)
            # But since we're publishing compressed, the format string will indicate
            frame_rgb = frame

            # Initialize maps on first frame
            h, w = frame.shape[:2]
            if self.map1 is None or self.map2 is None:
                self._ensure_maps((w, h))

            cam = np.eye(3, dtype=np.float32)
            cam[0, 2] = frame.shape[1] / 2.0  # define center x
            cam[1, 2] = frame.shape[0] / 2.0  # define center y
            cam[0, 0] = 10.  # define focal length x
            cam[1, 1] = 10.  # define focal length y
            nk = cam.copy()
            scale = 1.55
            nk[0, 0] = cam[0, 0] / scale  # scaling
            nk[1, 1] = cam[1, 1] / scale
            undistorter_2 = cv2.undistort(frame_rgb, cam, self.D, None, nk)

            # Apply undistortion using precomputed maps (fast for real-time)
            undistorted = cv2.remap( undistorter_2, self.map1, self.map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
            kernel = np.array([[-1, -1, -1],
                               [-1, 9, -1],
                               [-1, -1, -1]])

            # Apply the kernel to sharpen the image
            undistorted = cv2.filter2D(undistorted, -1, kernel)
            # Encode and publish
            data_bytes, fmt_out = self._encode_compressed(undistorted)

            # Create and publish compressed image message
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = f"{self.camera_name}_optical_frame"
            msg.format = fmt_out
            msg.data = data_bytes
            
            self.image_pub.publish(msg)

            # Publish camera info
            self.camera_info.header.stamp = msg.header.stamp
            self.camera_info_pub.publish(self.camera_info)

        except Exception as e:
            self.get_logger().error(f"Error in capture loop: {e}\n{traceback.format_exc()}")

    def destroy_node(self):
        """Cleanup on shutdown"""
        if self.cap and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = USBCamUndistortNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down by KeyboardInterrupt")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
