# teleop_keyboard/teleop_keyboard/teleop_keyboard_node.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from pynput import keyboard
import threading

class TeleopKeyboardNode(Node):
    """
    Нода для управления роботом с клавиатуры.
    Постоянно публикует сообщения TwistStamped в топик /cmd_vel с высокой частотой.
    """
    
    def __init__(self):
        super().__init__('teleop_keyboard_node')

        # --- Параметры конфигурации ---
        # Позволяют настраивать поведение ноды без изменения кода
        self.declare_parameter('linear_speed', 0.5)      # м/с, скорость прямолинейного движения
        self.declare_parameter('angular_speed', 1.0)     # рад/с, скорость вращения
        self.declare_parameter('publish_rate', 50.0)     # Гц, частота публикации команд
        self.declare_parameter('frame_id', 'base_link')  # Идентификатор системы координат

        # Получаем значения параметров (можно переопределить через launch-файл)
        self.linear_speed = self.get_parameter('linear_speed').get_parameter_value().double_value
        self.angular_speed = self.get_parameter('angular_speed').get_parameter_value().double_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        # --- Инициализация паблишера ---
        # Публикуем TwistStamped вместо Twist для поддержки временных меток и frame_id
        self.publisher_ = self.create_publisher(TwistStamped, 'cmd_vel', 10)

        # --- Состояние управления ---
        # Эти переменные обновляются в реальном времени на основе нажатых клавиш
        self.target_linear_vel = 0.0      # Текущая целевая линейная скорость (X ось)
        self.target_angular_vel = 0.0     # Текущая целевая угловая скорость (Z ось)
        self.pressed_keys = set()         # Множество для отслеживания нажатых клавиш

        # Выводим инструкции и параметры в консоль
        self.print_instructions()

        # --- Многопоточная обработка клавиатуры ---
        # Слушатель клавиатуры запускается в отдельном потоке, чтобы не блокировать основной цикл ROS2
        self.key_listener_thread = threading.Thread(target=self.start_keyboard_listener)
        self.key_listener_thread.daemon = True  # Демонизируем поток для автоматического завершения
        self.key_listener_thread.start()

        # --- Таймер для постоянной публикации ---
        # Публикуем команды с фиксированной частотой, даже если клавиши не менялись
        # Это обеспечивает плавное управление и обработку потери соединения
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_twist_continuously)

    def print_instructions(self):
        """Выводит инструкции по управлению и текущие настройки."""
        self.get_logger().info("=" * 50)
        self.get_logger().info("Keyboard Teleoperation Node Started")
        self.get_logger().info("Controls: W-forward, S-backward, A-left, D-right")
        self.get_logger().info(f"Linear speed: {self.linear_speed} m/s")
        self.get_logger().info(f"Angular speed: {self.angular_speed} rad/s")
        self.get_logger().info(f"Publish rate: {self.publish_rate} Hz")
        self.get_logger().info(f"Frame ID: {self.frame_id}")
        self.get_logger().info("Press Ctrl+C to exit")
        self.get_logger().info("=" * 50)

    def on_press(self, key):
        """
        Обработчик нажатия клавиши.
        Добавляет клавишу в множество нажатых и обновляет скорости.
        """
        try:
            # Получаем символьное представление клавиши
            char_key = key.char
            
            # Добавляем клавишу только если ее еще нет в множестве
            # Это предотвращает множественные обновления при залипании клавиши
            if char_key not in self.pressed_keys:
                self.pressed_keys.add(char_key)
                self.update_target_velocities()
                
        except AttributeError:
            # Игнорируем специальные клавиши (Shift, Ctrl, Alt и т.д.)
            # Нас интересуют только символьные клавиши WASD
            pass

    def on_release(self, key):
        """
        Обработчик отпускания клавиши.
        Удаляет клавишу из множества и обновляет скорости.
        """
        try:
            char_key = key.char
            if char_key in self.pressed_keys:
                self.pressed_keys.remove(char_key)
                self.update_target_velocities()
        except AttributeError:
            pass
        
        # ESC можно использовать для аварийной остановки (опционально)
        if key == keyboard.Key.esc:
            self.get_logger().warn("ESC pressed - emergency stop recommended")

    def update_target_velocities(self):
        """
        Вычисляет целевые скорости на основе текущего набора нажатых клавиш.
        Реализует простую логику: одна клавиша - полная скорость, комбинации - смешанное управление.
        """
        # Сбрасываем скорости перед вычислением новых
        self.target_linear_vel = 0.0
        self.target_angular_vel = 0.0

        # Логика управления (можно расширить для более сложного поведения)
        # Каждая клавиша устанавливает соответствующую компоненту скорости
        if 'w' in self.pressed_keys:
            self.target_linear_vel = self.linear_speed
        if 's' in self.pressed_keys:
            self.target_linear_vel = -self.linear_speed
        if 'a' in self.pressed_keys:
            self.target_angular_vel = -self.angular_speed
        if 'd' in self.pressed_keys:
            self.target_angular_vel = self.angular_speed

    def publish_twist_continuously(self):
        """
        Основной метод публикации. Вызывается таймером с фиксированной частотой.
        Всегда публикует текущее состояние скоростей, обеспечивая непрерывный поток команд.
        """
        # Создаем сообщение TwistStamped с временной меткой
        twist_stamped = TwistStamped()
        
        # --- Заполняем заголовок ---
        # Временная метка важна для синхронизации и фильтрации старых сообщений
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = self.frame_id  # Указываем систему координат
        
        # --- Заполняем данные о движении ---
        # Используем только linear.x (вперед/назад) и angular.z (вращение)
        # Остальные компоненты оставляем нулевыми для дифференциального привода
        twist_stamped.twist.linear.x = self.target_linear_vel
        twist_stamped.twist.angular.z = self.target_angular_vel
        
        # Y и Z компоненты не используются в типовой схеме управления роботом
        twist_stamped.twist.linear.y = 0.0
        twist_stamped.twist.linear.z = 0.0
        twist_stamped.twist.angular.x = 0.0
        twist_stamped.twist.angular.y = 0.0

        # Публикуем сообщение (даже если скорости нулевые)
        self.publisher_.publish(twist_stamped)

        # Отладочный вывод (можно включить/выключить через параметр)
        if abs(self.target_linear_vel) > 0.01 or abs(self.target_angular_vel) > 0.01:
            self.get_logger().debug(
                f"Cmd: v={self.target_linear_vel:.2f} m/s, "
                f"ω={self.target_angular_vel:.2f} rad/s"
            )

    def start_keyboard_listener(self):
        """
        Запускает фоновый слушатель клавиатуры.
        Использует pynput для кроссплатформенного отслеживания клавиш.
        """
        # Создаем слушатель с привязкой к нашим обработчикам
        with keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release
        ) as listener:
            # Блокирующий вызов - поток будет работать пока не завершится программа
            listener.join()

    def destroy_node(self):
        """
        Переопределяем метод уничтожения ноды для безопасной остановки.
        Важно отправлять команды остановки перед завершением работы.
        """
        self.get_logger().info("Shutting down teleop node...")
        
        # Отправляем несколько команд остановки для надежности
        # Некоторые системы могут пропустить первое сообщение при быстром завершении
        stop_twist = TwistStamped()
        stop_twist.header.stamp = self.get_clock().now().to_msg()
        stop_twist.header.frame_id = self.frame_id
        # Все скорости уже нулевые, но явно инициализируем для ясности
        stop_twist.twist.linear.x = 0.0
        stop_twist.twist.angular.z = 0.0
        
        for i in range(5):  # 5 сообщений достаточно для большинства систем
            self.publisher_.publish(stop_twist)
            self.get_logger().debug(f"Stop command {i+1}/5 sent")
        
        # Вызываем родительский метод для корректного завершения
        super().destroy_node()


def main(args=None):
    """
    Точка входа ROS2 ноды.
    Обеспечивает корректную инициализацию и завершение работы.
    """
    # Инициализация ROS2 контекста
    rclpy.init(args=args)
    
    # Создаем экземпляр ноды
    node = TeleopKeyboardNode()
    
    try:
        # Основной цикл обработки - блокирует выполнение до получения сигнала завершения
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Обработка Ctrl+C - нормальный способ завершения
        node.get_logger().info("Keyboard interrupt received")
    except Exception as e:
        # Логируем любые другие исключения
        node.get_logger().error(f"Unexpected error: {e}")
    finally:
        # Гарантируем корректное завершение в любом случае
        node.get_logger().info("Cleaning up resources...")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
