#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class DWATestNode(Node):
    def __init__(self):
        super().__init__('dwa_test_node')
        self.get_logger().info("DWA Test Node - Comprehensive enemy position testing")

        # Publishers (ТОЛЬКО точка противника)
        self.enemy_point_pub = self.create_publisher(PointStamped, '/enemy_point', 10)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer for publishing test data
        self.timer = self.create_timer(0.1, self.publish_test_data)  # 10 Hz

        # Все возможные позиции противника для тестирования
        self.enemy_positions = [
            # БЛИЗКИЕ ПОЗИЦИИ (1-3 метра)
            # Прямо вперед
            [1.0, 0.0, "CLOSE: Straight ahead"],
            [2.0, 0.0, "CLOSE: Straight ahead medium"],
            [3.0, 0.0, "CLOSE: Straight ahead far"],

            # Под углами 45°
            [2.0, 2.0, "CLOSE: 45° right-forward"],
            [2.0, -2.0, "CLOSE: 45° left-forward"],
            [-2.0, 2.0, "CLOSE: 45° right-backward"],
            [-2.0, -2.0, "CLOSE: 45° left-backward"],

            # Сбоку
            [0.0, 2.0, "CLOSE: Right side"],
            [0.0, -2.0, "CLOSE: Left side"],

            # Сзади
            [-1.0, 0.0, "CLOSE: Directly behind"],
            [-2.0, 0.0, "CLOSE: Behind medium"],
            [-3.0, 0.0, "CLOSE: Behind far"],

            # ДАЛЬНИЕ ПОЗИЦИИ (4-10 метров)
            # Прямо вперед
            [5.0, 0.0, "FAR: Straight ahead"],
            [8.0, 0.0, "FAR: Straight ahead very far"],
            [10.0, 0.0, "FAR: Straight ahead extreme"],

            # Под углами
            [5.0, 5.0, "FAR: 45° right-forward"],
            [5.0, -5.0, "FAR: 45° left-forward"],
            [-5.0, 5.0, "FAR: 45° right-backward"],
            [-5.0, -5.0, "FAR: 45° left-backward"],

            # Сбоку
            [0.0, 5.0, "FAR: Right side"],
            [0.0, -5.0, "FAR: Left side"],
            [0.0, 8.0, "FAR: Right side very far"],
            [0.0, -8.0, "FAR: Left side very far"],

            # Сзади
            [-5.0, 0.0, "FAR: Behind"],
            [-8.0, 0.0, "FAR: Behind very far"],
            [-10.0, 0.0, "FAR: Behind extreme"],

            # ОСОБЫЕ СЛУЧАИ
            # Очень близкие
            [0.5, 0.0, "VERY CLOSE: Straight ahead"],
            [0.0, 0.5, "VERY CLOSE: Right side"],
            [-0.5, 0.0, "VERY CLOSE: Behind"],
            [0.0, -0.5, "VERY CLOSE: Left side"],

            # Диагонали очень близко
            [0.7, 0.7, "VERY CLOSE: 45° right-forward"],
            [0.7, -0.7, "VERY CLOSE: 45° left-forward"],
            [-0.7, 0.7, "VERY CLOSE: 45° right-backward"],
            [-0.7, -0.7, "VERY CLOSE: 45° left-backward"],

            # На границе зоны видимости
            [15.0, 0.0, "EDGE: Straight ahead limit"],
            [0.0, 15.0, "EDGE: Right side limit"],
            [-15.0, 0.0, "EDGE: Behind limit"],
            [0.0, -15.0, "EDGE: Left side limit"],

            # Случайные позиции для полноты тестирования
            [1.5, 3.0, "RANDOM: Mixed position 1"],
            [-2.0, 4.0, "RANDOM: Mixed position 2"],
            [3.0, -1.5, "RANDOM: Mixed position 3"],
            [-4.0, -3.0, "RANDOM: Mixed position 4"],
            [6.0, 2.0, "RANDOM: Mixed position 5"],
            [-7.0, 1.0, "RANDOM: Mixed position 6"]
        ]

        self.current_position_index = 0
        self.position_timer = self.create_timer(1.0, self.change_position)  # Меняем позицию каждые 3 секунды

    def change_position(self):
        """Change enemy position"""
        self.current_position_index = (self.current_position_index + 1) % len(self.enemy_positions)
        pos = self.enemy_positions[self.current_position_index]
        self.get_logger().info(f"Enemy position: {pos[2]} ({pos[0]:.1f}, {pos[1]:.1f})")

    def publish_test_data(self):
        """Publish ONLY enemy point"""
        current_time = self.get_clock().now()

        # Get current enemy position
        enemy_pos = self.enemy_positions[self.current_position_index]
        enemy_x, enemy_y, pos_description = enemy_pos

        # --- Publish Enemy Point ---
        point_msg = PointStamped()
        point_msg.header = Header()
        point_msg.header.stamp = current_time.to_msg()
        point_msg.header.frame_id = 'map'

        point_msg.point.x = enemy_x
        point_msg.point.y = enemy_y
        point_msg.point.z = 0.0

        self.enemy_point_pub.publish(point_msg)

        # --- Publish TF transform for enemy (для визуализации в RViz) ---
        enemy_tf = TransformStamped()
        enemy_tf.header.stamp = current_time.to_msg()
        enemy_tf.header.frame_id = 'map'
        enemy_tf.child_frame_id = 'enemy'

        enemy_tf.transform.translation.x = enemy_x
        enemy_tf.transform.translation.y = enemy_y
        enemy_tf.transform.translation.z = 0.0

        enemy_tf.transform.rotation.x = 0.0
        enemy_tf.transform.rotation.y = 0.0
        enemy_tf.transform.rotation.z = 0.0
        enemy_tf.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(enemy_tf)

        # Log detailed info every position change
        if current_time.nanoseconds % 3_000_000_000 < 100_000_000:
            distance = math.sqrt(enemy_x ** 2 + enemy_y ** 2)

            self.get_logger().info(
                f"\n═══════════════════════════════════════════════════\n"
                f"TEST SCENARIO:\n"
                f"  Enemy: ({enemy_x:.1f}, {enemy_y:.1f}) - {pos_description}\n"
                f"  Distance from origin: {distance:.1f}m\n"
                f"═══════════════════════════════════════════════════"
            )


def main(args=None):
    rclpy.init(args=args)
    node = DWATestNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
