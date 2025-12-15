#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math

from geometry_msgs.msg import PointStamped


class DataPublisherNode(Node):
    def __init__(self):
        super().__init__('data_publisher_node')
        
        # Параметры
        self.declare_parameter('publish_rate', 30.0)
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # Позиция противника - хранится как float
        self.enemy_x = 4.0
        self.enemy_y = 4.0
        
        # ИСПРАВЛЕНО: Переменная для хранения последней отправленной позиции
        self.last_published_x = None
        self.last_published_y = None
        
        # Публикатор
        self.enemy_point_pub = self.create_publisher(
            PointStamped,
            '/enemy_point',
            10
        )
        
        # Таймер для публикации
        self.publish_timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish_enemy_point
        )
        
        # Счетчик публикаций для отладки
        self.publish_count = 0
        
        self.get_logger().info('Публикатор данных запущен')
        self.get_logger().info(f'Публикует /enemy_point: ({self.enemy_x}, {self.enemy_y})')
    
    def publish_enemy_point(self):
        """Публикация позиции противника - всегда одна и та же позиция"""
        # ИСПРАВЛЕНО: Всегда отправляем одни и те же координаты (4, 4)
        # Не перезаписываем self.enemy_x и self.enemy_y
        
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        msg.point.x = 4.0  # Всегда 4.0
        msg.point.y = 4.0  # Всегда 4.0
        msg.point.z = 0.0
        
        self.enemy_point_pub.publish(msg)
        
        # Отладочная информация каждые 100 публикаций
        self.publish_count += 1
        if self.publish_count % 100 == 0:
            self.get_logger().debug(f'Опубликовано {self.publish_count} сообщений: ({msg.point.x}, {msg.point.y})')


def main(args=None):
    rclpy.init(args=args)
    node = DataPublisherNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Публикатор данных остановлен")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
