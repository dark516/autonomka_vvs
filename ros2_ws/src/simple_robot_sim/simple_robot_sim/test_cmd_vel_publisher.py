#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import time

from geometry_msgs.msg import TwistStamped


class TestCmdVelPublisher(Node):
    def __init__(self):
        super().__init__('test_cmd_vel_publisher')
        
        self.publisher = self.create_publisher(
            TwistStamped,
            '/cmd_vel',
            10
        )
        
        # Параметры тестового движения
        self.mode = 'circle'  # 'circle', 'forward', 'rotate', 'square'
        self.start_time = self.get_clock().now()
        self.last_publish_time = time.time()
        
        self.timer = self.create_timer(0.05, self.publish_cmd_vel)  # 20 Гц
        
        self.get_logger().info('Тестовый публикатор команд запущен')
        self.get_logger().info(f'Режим: {self.mode}')
    
    def publish_cmd_vel(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        
        if self.mode == 'circle':
            # Движение по кругу
            msg.twist.linear.x = 0.5  # 50% мощности вперед
            msg.twist.angular.z = 0.3  # 10% мощности на вращение
            
        elif self.mode == 'forward':
            # Движение вперед с плавным ускорением
            if elapsed < 2.0:
                msg.twist.linear.x = elapsed / 2.0 * 0.8  # плавное ускорение до 80%
            else:
                msg.twist.linear.x = 0.8
        
        elif self.mode == 'rotate':
            # Вращение на месте
            msg.twist.angular.z = 0.5  # 50% мощности на вращение
        
        elif self.mode == 'square':
            # Движение по квадрату
            cycle_time = elapsed % 8.0  # 8 секунд на полный цикл
            
            if cycle_time < 2.0:
                # Вперед
                msg.twist.linear.x = 0.6
            elif cycle_time < 2.5:
                # Поворот
                msg.twist.angular.z = 0.8
            elif cycle_time < 4.5:
                # Вперед
                msg.twist.linear.x = 0.6
            elif cycle_time < 5.0:
                # Поворот
                msg.twist.angular.z = 0.8
            elif cycle_time < 7.0:
                # Вперед
                msg.twist.linear.x = 0.6
            elif cycle_time < 7.5:
                # Поворот
                msg.twist.angular.z = 0.8
            else:
                # Вперед
                msg.twist.linear.x = 0.6
        
        # Ограничиваем публикацию до 20 Гц
        current_time = time.time()
        if current_time - self.last_publish_time >= 0.05:
            self.publisher.publish(msg)
            self.last_publish_time = current_time


def main(args=None):
    rclpy.init(args=args)
    node = TestCmdVelPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Тестовый публикатор остановлен")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
