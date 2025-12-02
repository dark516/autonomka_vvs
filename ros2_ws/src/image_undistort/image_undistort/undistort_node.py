#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    qos_profile_sensor_data,
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
import traceback


class ImageUndistortCompressedNode(Node):
    def __init__(self):
        super().__init__('image_undistort_compressed_node')

        # ---------------- Parameters ----------------
        self.declare_parameter('input_topic', '/camera_1/image_raw/compressed')
        self.declare_parameter('output_image_topic', '/stupid')
        self.declare_parameter('calibration_file', 'fisheye_calibration.npz')

        # Если оставить 0/0 — возьмём размер первого кадра
        self.declare_parameter('output_width', 0)
        self.declare_parameter('output_height', 0)

        # Кодек и качество
        self.declare_parameter('encode_format', 'jpeg')   # 'jpeg' или 'png'
        self.declare_parameter('jpeg_quality', 80)        # 0..100
        self.declare_parameter('png_compression', 3)      # 0..9

        # Получаем параметры
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_image_topic').value
        calibration_file = self.get_parameter('calibration_file').value
        self.out_w = int(self.get_parameter('output_width').value)
        self.out_h = int(self.get_parameter('output_height').value)
        self.encode_format = str(self.get_parameter('encode_format').value).lower()
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self.png_compression = int(self.get_parameter('png_compression').value)

        # ---------------- Load calibration ----------------
        try:
            data = np.load(calibration_file)
            self.K = data['K'].astype(np.float64)
            self.D = data['D'].astype(np.float64)
            self.calib_image_size = tuple(data['image_size']) if 'image_size' in data else None
            self.get_logger().info(f"[init] Loaded calibration: {calibration_file}")
            self.get_logger().info(f"[init] K=\n{self.K}")
            self.get_logger().info(f"[init] D={self.D.ravel()}")
            if self.calib_image_size:
                self.get_logger().info(f"[init] calib image_size={self.calib_image_size}")
        except Exception as e:
            self.get_logger().error(f"[init] Failed to load calibration '{calibration_file}': {e}")
            raise

        # Карты создадим лениво при первом кадре, когда узнаем реальный размер, если out_w/out_h не заданы
        self.map1 = None
        self.map2 = None
        self.output_size = None  # (W, H)

        # ---------------- QoS ----------------
        # Подписчик: профиль датчиков (обычно совпадает с драйвером камеры)
        sub_qos = qos_profile_sensor_data

        # Паблишер: RELIABLE + буфер поглубже — чтобы rqt/image_view и многие подписчики наверняка подключились
        pub_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST
        )

        # ---------------- Pub/Sub ----------------
        self.publisher = self.create_publisher(CompressedImage, output_topic, pub_qos)
        self.subscription = self.create_subscription(
            CompressedImage, input_topic, self.image_callback, sub_qos
        )

        self.get_logger().info(
            f"[init] Subscribing: {input_topic} (CompressedImage, QoS=sensor_data)"

        )
        self.get_logger().info(
            f"[init] Publishing: {output_topic} (CompressedImage, QoS=RELIABLE depth=10)"
        )
        self.get_logger().info(
            f"[init] Encoding: {self.encode_format}, jpeg_quality={self.jpeg_quality}, png_compression={self.png_compression}"
        )
        self.get_logger().info(
            f"[init] Output size param: {self.out_w}x{self.out_h} (0=auto from first frame)"
        )

    # ---------- Helpers ----------
    def _decode_compressed(self, msg: CompressedImage) -> np.ndarray:
        np_arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        return img

    def _encode_compressed(self, img: np.ndarray):
        fmt = 'jpeg'
        if self.encode_format in ('jpeg', 'jpg'):
            params = [cv2.IMWRITE_JPEG_QUALITY, int(np.clip(self.jpeg_quality, 0, 100))]
            ok, enc = cv2.imencode('.jpg', img, params)
            fmt = 'jpeg'
        elif self.encode_format == 'png':
            params = [cv2.IMWRITE_PNG_COMPRESSION, int(np.clip(self.png_compression, 0, 9))]
            ok, enc = cv2.imencode('.png', img, params)
            fmt = 'png'
        else:
            # Fallback на jpeg
            params = [cv2.IMWRITE_JPEG_QUALITY, int(np.clip(self.jpeg_quality, 0, 100))]
            ok, enc = cv2.imencode('.jpg', img, params)
            fmt = 'jpeg'

        if not ok or enc is None:
            raise RuntimeError("cv2.imencode failed")
        return enc.tobytes(), fmt

    def _ensure_maps(self, in_shape_wh):
        """Создаёт карты undistort при первом кадре или при смене желаемого выхода."""
        in_w, in_h = in_shape_wh
        if self.out_w <= 0 or self.out_h <= 0:
            self.output_size = (in_w, in_h)  # берём размер входа
        else:
            self.output_size = (self.out_w, self.out_h)

        # Можно при необходимости отмасштабировать фокус (оставим как есть)
        nk = self.K.copy()

        self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(
            self.K, self.D, np.eye(3), nk, self.output_size, cv2.CV_16SC2
        )
        self.get_logger().info(
            f"[maps] initUndistortRectifyMap done. output_size={self.output_size}"
        )
        if self.calib_image_size and (in_w, in_h) != tuple(self.calib_image_size):
            self.get_logger().warn(
                f"[maps] Input frame size {in_w}x{in_h} differs from calibration image_size {self.calib_image_size}. "
                f"Undistortion may be suboptimal."
            )

    # ---------- Callback ----------
    def image_callback(self, msg: CompressedImage):
        try:
            # Базовые метаданные входа
            fmt_in = msg.format if msg.format else '(empty)'
            bytes_in = len(msg.data) if msg.data else 0
            self.get_logger().debug(
                f"[in] frame ts={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d} "
                f"format={fmt_in} bytes={bytes_in}"
            )

            if bytes_in == 0:
                self.get_logger().warn("[in] Empty compressed payload — skip")
                return

            # Декод
            img = self._decode_compressed(msg)
            if img is None:
                self.get_logger().warn("[decode] cv2.imdecode returned None — skip frame")
                return

            h, w = img.shape[:2]
            self.get_logger().debug(f"[decode] size={w}x{h}")

            # Инициализация карт при первом кадре
            if self.map1 is None or self.map2 is None or self.output_size is None:
                self._ensure_maps((w, h))

            # Undistort (выходной размер = self.output_size)
            undistorted = cv2.remap(
                img, self.map1, self.map2,
                interpolation=cv2.INTER_LINEAR,
                borderMode=cv2.BORDER_CONSTANT
            )

            uh, uw = undistorted.shape[:2]
            self.get_logger().debug(f"[remap] out_size={uw}x{uh}")

            # Кодирование обратно в CompressedImage
            data_bytes, fmt_out = self._encode_compressed(undistorted)
            bytes_out = len(data_bytes)

            out = CompressedImage()
            out.header = msg.header
            out.format = fmt_out
            out.data = data_bytes

            self.publisher.publish(out)

            subs = self.publisher.get_subscription_count()
            # self.get_logger().info(f"[pub] wrote frame: out_format={fmt_out} bytes={bytes_out} "f"subs={subs} topic={self.publisher.topic_name}")

        except Exception as e:
            self.get_logger().error(f"[error] {e}\n{traceback.format_exc()}")


def main(args=None):
    rclpy.init(args=args)
    node = ImageUndistortCompressedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down by KeyboardInterrupt")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
