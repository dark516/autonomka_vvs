import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32

class JoyController(Node):
    def __init__(self):
        super().__init__('joy_controller')
        
        # Параметры с максимальными скоростями
        self.declare_parameter('max_angular_vel', 1.0)
        self.declare_parameter('max_linear_vel', 1.0)
        self.declare_parameter('frame_id', 'base_link')  # frame_id для TwistStamped
        
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.frame_id = self.get_parameter('frame_id').value
        
        # Публикаторы
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.weapon_pub = self.create_publisher(Float32, '/robot/weapon', 10)
        
        # Подписка на джойстик
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        self.get_logger().info('Joy controller node started')
        self.get_logger().info(f'Max angular: {self.max_angular_vel}, Max linear: {self.max_linear_vel}')

    def joy_callback(self, msg):
        # Обработка осей джойстика
        axis1 = msg.axes[0]  # angular.z
        axis2 = msg.axes[1]  # linear.x
        axis3 = msg.axes[2]  # оружие
        
        # Создание сообщения TwistStamped
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = self.frame_id
        
        # Установка скоростей с учетом максимальных значений
        twist_stamped.twist.angular.z = -axis1 * self.max_angular_vel
        twist_stamped.twist.linear.x = axis2 * self.max_linear_vel
        
        # Преобразование оси оружия из [-1,1] в [0,1] с инверсией
        # Теперь: -1 -> 1, 1 -> 0
        weapon_value = (1 - axis3) / 2.0
        weapon_msg = Float32()
        weapon_msg.data = weapon_value
        
        # Публикация сообщений
        self.cmd_vel_pub.publish(twist_stamped)
        self.weapon_pub.publish(weapon_msg)

def main():
    rclpy.init()
    node = JoyController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
