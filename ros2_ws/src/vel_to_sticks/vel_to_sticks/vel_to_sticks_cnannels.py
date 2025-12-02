#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8, Float32
import serial
from geometry_msgs.msg import TwistStamped
import threading
from rclpy.qos import QoSProfile, ReliabilityPolicy

class WheelVelocityToSticks(Node):
    def __init__(self):
        super().__init__('wheel_vel_to_sticks')

        # Параметры
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('send_frequency', 50.0)
        self.declare_parameter('failsafe_timeout', 0.5)  # таймаут failsafe в секундах

        # Получение параметров
        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value
        send_frequency = self.get_parameter('send_frequency').value
        self.failsafe_timeout = self.get_parameter('failsafe_timeout').value

        # Открытие serial порта
        try:
            self.ser = serial.Serial(serial_port, baud_rate, timeout=0.1)
            self.get_logger().info(f'Успешно открыт serial порт: {serial_port} @ {baud_rate} бод')
        except Exception as e:
            self.get_logger().error(f'Не удалось открыть serial порт {serial_port}: {e}')
            raise

        # Состояние для потокобезопасного доступа
        self.lock = threading.Lock()
        self.l_velocity = 0.0
        self.r_velocity = 0.0
        self.l_velocity_keyboard = 0.0
        self.r_velocity_keyboard = 0.0
        self.l_velocity_help = 0.0
        self.r_velocity_help = 0.0
        self.weapon = -1.0  # начальное значение -1 (0 в терминах оружия)
        self.inverse = False  # режим инвертирования
        self.last_velocity_time = None

        # Подписки на топики
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.l_vel_sub = self.create_subscription(
            Float32,
            '/l_velocity',
            self.l_velocity_callback,
            qos
        )

        self.r_vel_sub = self.create_subscription(
            Float32,
            '/r_velocity',
            self.r_velocity_callback,
            qos
        )
        
        
        self.l_vel_sub = self.create_subscription(
            Float32,
            '/l_velocity_keyboard',
            self.l_velocity_callback_keyboard,
            qos
        )

        self.r_vel_sub = self.create_subscription(
            Float32,
            '/r_velocity_keyboard',
            self.r_velocity_callback_keyboard,
            qos
        )

        self.l_vel_sub = self.create_subscription(
            Float32,
            '/l_velocity_help',
            self.l_velocity_callback_help,
            qos
        )

        self.r_vel_sub = self.create_subscription(
            Float32,
            '/r_velocity_help',
            self.r_velocity_callback_help,
            qos
        )

        self.inverse_sub = self.create_subscription(
            UInt8,
            '/robot/inverse',
            self.inverse_callback,
            10
        )

        self.weapon_sub = self.create_subscription(
            Float32,
            '/robot/weapon',
            self.weapon_callback,
            10
        )
    
        # Поток для отправки данных
        self.stop_event = threading.Event()
        self.send_thread = threading.Thread(
            target=self.sender_loop,
            args=(send_frequency,),
            daemon=True
        )
        self.send_thread.start()

        self.get_logger().info('Нода wheel_vel_to_sticks запущена')

    def l_velocity_callback(self, msg: Float32):
        """Callback для обработки сообщений /l_velocity"""
        with self.lock:
            self.l_velocity = self.clamp(msg.data, -1.0, 1.0)
            self.last_velocity_time = self.get_clock().now()

        self.get_logger().debug(f'Получена скорость левого колеса: {self.l_velocity:.3f}')

    def r_velocity_callback(self, msg: Float32):
        """Callback для обработки сообщений /r_velocity"""
        with self.lock:
            self.r_velocity = self.clamp(msg.data, -1.0, 1.0)
            self.last_velocity_time = self.get_clock().now()

        self.get_logger().debug(f'Получена скорость правого колеса: {self.r_velocity:.3f}')    
        
    def l_velocity_callback_keyboard(self, msg: Float32):
        """Callback для обработки сообщений /l_velocity"""
        with self.lock:
            self.l_velocity_keyboard = self.clamp(msg.data, -1.0, 1.0)
            self.last_velocity_time = self.get_clock().now()

        self.get_logger().debug(f'Получена скорость левого колеса: {self.l_velocity:.3f}')

    def r_velocity_callback_keyboard(self, msg: Float32):
        """Callback для обработки сообщений /r_velocity"""
        with self.lock:
            self.r_velocity_keyboard = self.clamp(msg.data, -1.0, 1.0)
            self.last_velocity_time = self.get_clock().now()

        self.get_logger().debug(f'Получена скорость правого колеса: {self.r_velocity:.3f}')
    
    def l_velocity_callback_help(self, msg: Float32):
        """Callback для обработки сообщений /l_velocity"""
        with self.lock:
            self.l_velocity_help = self.clamp(msg.data, -1.0, 1.0)
            self.last_velocity_time = self.get_clock().now()

        self.get_logger().debug(f'Получена скорость левого колеса: {self.l_velocity:.3f}')

    def r_velocity_callback_help(self, msg: Float32):
        """Callback для обработки сообщений /r_velocity"""
        with self.lock:
            self.r_velocity_help = self.clamp(msg.data, -1.0, 1.0)
            self.last_velocity_time = self.get_clock().now()

        self.get_logger().debug(f'Получена скорость правого колеса: {self.r_velocity:.3f}')

    def inverse_callback(self, msg: UInt8):
        """Callback для обработки сообщений /robot/inverse"""
        inverse_mode = (msg.data == 1)
        with self.lock:
            self.inverse = inverse_mode

        self.get_logger().info(f'Режим инвертирования: {"включен" if inverse_mode else "выключен"}')

    def weapon_callback(self, msg: Float32):
        """Callback для обработки сообщений /robot/weapon"""
        weapon_value = self.clamp(msg.data, 0.0, 1.0)
        # Преобразуем [0, 1] в [-1, 1]
        weapon_mapped = weapon_value * 2.0 - 1.0

        with self.lock:
            self.weapon = weapon_mapped

        self.get_logger().debug(f'Получено значение оружия: {weapon_value:.3f} -> {weapon_mapped:.3f}')

    def clamp(self, x: float, lo: float = -1.0, hi: float = 1.0) -> float:
        """Ограничение значения в диапазоне"""
        return max(lo, min(hi, x))

    def to_csv(self, ax: float, ay: float, bx: float, by: float) -> bytes:
        """Преобразование в CSV формат для отправки"""
        ax = self.clamp(ax)
        ay = self.clamp(ay)
        bx = self.clamp(bx)
        by = self.clamp(by)
        return f"{ax:.3f},{ay:.3f},{bx:.3f},{by:.3f}\n".encode("utf-8")

    def sender_loop(self, send_frequency: float):
        """Цикл отправки данных с заданной частотой"""
        period = 1.0 / send_frequency

        while not self.stop_event.is_set() and rclpy.ok():
            # Получаем текущие данные
            with self.lock:
                l_velocity = self.l_velocity
                r_velocity = self.r_velocity
                weapon = self.weapon
                inverse = self.inverse
                last_velocity_time = self.last_velocity_time

            current_time = self.get_clock().now()

            # Проверяем failsafe
            if last_velocity_time is None:
                # Если команды еще не приходили, используем нули
                l_velocity = 0.0
                r_velocity = 0.0

            else:
                time_diff = (current_time - last_velocity_time).nanoseconds / 1e9
                if time_diff > self.failsafe_timeout:
                    # Если прошло больше таймаута, используем нули
                    l_velocity = 0.0
                    r_velocity = 0.0
                    self.l_velocity = 0.0
                    self.r_velocity = 0.0
                    self.get_logger().warning('Активирован failsafe: команды скоростей не поступают', once=True)

            # Применяем инвертирование если нужно
            if inverse:
                l_velocity, r_velocity = r_velocity, l_velocity

            # Формируем данные для отправки:
            # 0.0 -> первое число (не используется)
            # weapon -> второе число (оружие)
            # l_velocity -> третье число (левое колесо)
            # r_velocity -> четвертое число (правое колесо)


            # Отправляем по serial
            if self.l_velocity_keyboard!=0.0 or self.r_velocity_keyboard!=0.0:
                sticks_data = (0.0, weapon, self.l_velocity_keyboard, self.r_velocity_keyboard)
            elif self.l_velocity_help!=0.0 or self.r_velocity_help!=0.0:
                sticks_data = (0.0, weapon, self.l_velocity_help, self.r_velocity_help)
            else:
                sticks_data = (0.0, weapon, self.l_velocity, self.r_velocity)
                            
            try:
                self.ser.write(self.to_csv(*sticks_data))
            except Exception as e:
                self.get_logger().error(f'Ошибка отправки по serial: {e}')
                break

                

            # Задержка для поддержания частоты
            self.stop_event.wait(period)

    def destroy_node(self):
        """Корректное завершение работы"""
        self.get_logger().info('Завершение работы ноды...')

        # Останавливаем поток отправки
        self.stop_event.set()
        if self.send_thread.is_alive():
            self.send_thread.join(timeout=1.0)

        # Отправляем нули для остановки робота
        try:
            self.ser.write(self.to_csv(0.0, self.weapon, 0.0, 0.0))
        except Exception as e:
            self.get_logger().warning(f'Не удалось отправить нулевые значения: {e}')

        # Закрываем serial порт
        try:
            self.ser.close()
        except Exception as e:
            self.get_logger().warning(f'Ошибка при закрытии serial порта: {e}')

        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    try:
        node = WheelVelocityToSticks()
        rclpy.spin(node)
    except Exception as e:
        print(f"Ошибка при запуске ноды: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
