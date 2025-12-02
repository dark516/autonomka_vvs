#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import UInt8, Float32
import serial
import threading
from typing import Tuple
from rclpy.qos import QoSProfile, ReliabilityPolicy

class CmdVelToSticks(Node):
    def __init__(self):
        super().__init__('vel_to_sticks')

        # Параметры
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('send_frequency', 50.0)
        self.declare_parameter('drive_failsafe_timeout', 0.5)  # таймаут failsafe ходовой в секундах
        self.declare_parameter('weapon_failsafe_timeout', 0.5)  # таймаут failsafe орудия в секундах

        # Получение параметров
        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value
        send_frequency = self.get_parameter('send_frequency').value
        self.drive_failsafe_timeout = self.get_parameter('drive_failsafe_timeout').value
        self.weapon_failsafe_timeout = self.get_parameter('weapon_failsafe_timeout').value

        # Открытие serial порта
        try:
            self.ser = serial.Serial(serial_port, baud_rate, timeout=0.1)
            self.get_logger().info(f'Успешно открыт serial порт: {serial_port} @ {baud_rate} бод')
        except Exception as e:
            self.get_logger().error(f'Не удалось открыть serial порт {serial_port}: {e}')
            raise

        # Состояние для потокобезопасного доступа
        self.lock = threading.Lock()
        self.linear = 0.0 # линейная скорость
        self.angular = 0.0 # угловая скорость
        self.weapon = 0.0  # начальное значение 0
        self.inverted = False # перевернут?
        self.last_cmd_vel_time = None #
        self.last_weapon_time = None  # время последней команды орудия

        # Подписка на топики
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.cmd_vel_sub = self.create_subscription(
            TwistStamped,
            '/cmd_vel',
            self.cmd_vel_callback,
            qos
        )

        self.invert_sub = self.create_subscription(
            UInt8,
            '/robot/invert',
            self.invert_callback,
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

        self.get_logger().info('Нода vel_to_sticks запущена')
    #телеметрия всякого
    def cmd_vel_callback(self, msg: TwistStamped):
        """Callback для обработки сообщений /cmd_vel"""
        with self.lock:
            self.linear = msg.twist.linear.x
            self.angular = msg.twist.angular.z
            self.last_cmd_vel_time = self.get_clock().now()
        self.get_logger().debug(f'Получен cmd_vel: linear={msg.twist.linear.x:.3f}, angular={msg.twist.angular.z:.3f}')

    def invert_callback(self, msg: UInt8):
        """Callback для обработки сообщений /robot/invert"""
        inverted = (msg.data == 1)
        with self.lock:
            self.inverted = inverted

        self.get_logger().info(f'Режим инвертирования: {"включен" if inverted else "выключен"}')

    def weapon_callback(self, msg: Float32):
        """Callback для обработки сообщений /robot/weapon"""
        weapon_value = self.clamp(msg.data, 0.0, 1.0)
        with self.lock:
            self.weapon = weapon_value
            self.last_weapon_time = self.get_clock().now()

        self.get_logger().debug(f'Получено значение оружия: {weapon_value:.3f}')

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
                linear = self.linear
                angular = self.angular
                weapon = self.weapon
                inverted = self.inverted
                last_cmd_vel_time = self.last_cmd_vel_time
                last_weapon_time = self.last_weapon_time

            current_time = self.get_clock().now()

            # защита от глупых ситуаций
            # НЕЗАВИСИМЫЙ failsafe для ходовой части
            if last_cmd_vel_time is None:
                # Если команды еще не приходили, используем нули
                linear = 0.0
                angular = 0.0
            else:
                time_diff = (current_time - last_cmd_vel_time).nanoseconds / 1e9
                if time_diff > self.drive_failsafe_timeout:
                    # Если прошло больше таймаута, используем нули для ходовой
                    linear = 0.0
                    angular = 0.0
                    self.get_logger().warning('Активирован failsafe ходовой: команды не поступают', once=True)

            # НЕЗАВИСИМЫЙ failsafe для орудия
            if last_weapon_time is None:
                # Если команды орудия еще не приходили, используем 0
                weapon = 0.0
            else:
                time_diff_weapon = (current_time - last_weapon_time).nanoseconds / 1e9
                if time_diff_weapon > self.weapon_failsafe_timeout:
                    # Если прошло больше таймаута, используем 0 для орудия
                    weapon = 0.0
                    self.get_logger().warning('Активирован failsafe орудия: команды не поступают', once=True)

            # Применяем инвертирование если нужно
            if inverted:
                # При инвертировании меняем знак у angular
                angular = -angular

            # Формируем данные для отправки:
            # angular -> первое число (поворот)
            # weapon -> второе число (оружие)
            # 0.0 -> третье число (не используется)
            # linear -> четвертое число (движение вперед/назад)
            sticks_data = (linear, angular, weapon, 0.0)

            # Отправляем по serial
            try:
                self.ser.write(self.to_csv(*sticks_data))
            except Exception as e:
                self.get_logger().error(f'Ошибка отправки по serial: {e}')
                break

            # Задержка для поддержания частоты
            self.stop_event.wait(period)

    def destroy_node(self):
        """Корректное завершение работы без ошибок в консоли"""
        self.get_logger().info('Завершение работы ноды...')

        # Останавливаем поток отправки
        self.stop_event.set()
        if self.send_thread.is_alive():
            self.send_thread.join(timeout=1.0)

        # Отправляем нули для остановки робота и орудия
        try:
            self.ser.write(self.to_csv(0.0, 0.0, 0.0, 0.0))
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
        node = CmdVelToSticks()
        rclpy.spin(node)
    except Exception as e:
        print(f"Ошибка при запуске ноды: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
