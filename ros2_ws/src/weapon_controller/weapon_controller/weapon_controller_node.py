import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import Float32, String
import math
import time


class WeaponController(Node):
    def __init__(self):
        super().__init__('weapon_controller')

        # Скоростные параметры
        self.current_speed = 0.0
        self.target_speed = 0.40
        self.max_speed_limit = 0.40
        
        # Параметры для плавности регулировки
        self.ramp_rate = 0.009
        self.auto_ramp_target = 0.40

        # Конфигурация управления по расстоянию
        # При расстоянии < 1.5м скорость снижается до 0.30
        # При расстоянии ≥ 1.5м скорость равна 0.40
        self.close_distance = 1.5
        self.far_distance = 1.5
        self.close_speed = 0.30
        self.far_speed = 0.40
        
        # Таймеры для отслеживания актуальности данных
        # Если позиции не обновляются дольше pose_timeout - переходим в авторежим
        self.robot_pose_last_time = None
        self.enemy_point_last_time = None
        self.pose_timeout = 2.0
        
        # Система параболического разгона для плавного старта
        self.initial_acceleration_factor = 0.0
        self.initial_acceleration_complete = False
        self.acceleration_step = 0.3
        self.acceleration_power = 0.3
        
        # Механизм плавного выключения (graceful shutdown)
        self.shutdown_requested = False
        self.shutdown_start_time = None
        self.shutdown_duration = 10.0
        self.shutdown_complete = False
        
        # Подписка на входные данные
        self.robot_pose_subscriber = self.create_subscription(
            PoseStamped, '/robot/pose', self.robot_pose_callback, 10)
        self.enemy_point_subscriber = self.create_subscription(
            PointStamped, '/enemy/point', self.enemy_point_callback, 10)
        
        # Подписка на команду аварийного останова
        self.shutdown_subscriber = self.create_subscription(
            String, '/robot/weapon/shutdown', self.shutdown_callback, 10)

        # Публикация скорости оружия
        self.speed_publisher = self.create_publisher(Float32, '/robot/weapon', 10)

        # Хранение текущих позиций
        self.robot_pose = None
        self.enemy_point = None

        # Основной таймер обработки - работает на 10 Гц
        self.timer = self.create_timer(0.1, self.update_speed)
        
        self.get_logger().info("Weapon controller initialized")

    def robot_pose_callback(self, msg):
        # Игнорируем новые сообщения во время выключения
        if self.shutdown_requested:
            return
            
        self.robot_pose = msg.pose
        self.robot_pose_last_time = time.time()

    def enemy_point_callback(self, msg):
        if self.shutdown_requested:
            return
            
        self.enemy_point = msg.point
        self.enemy_point_last_time = time.time()
        
    def shutdown_callback(self, msg):
        """Обработчик команды экстренного останова"""
        # При получении 'stop' начинаем процесс плавного выключения
        if msg.data.lower() == "stop" and not self.shutdown_requested:
            self.initiate_shutdown()

    def calculate_distance(self, pose, point):
        """Вычисление Евклидова расстояния между роботом и противником"""
        if pose is None or point is None:
            return float('inf')
            
        dx = pose.position.x - point.x
        dy = pose.position.y - point.y
        
        return math.sqrt(dx**2 + dy**2)

    def has_valid_poses(self):
        """Проверка актуальности данных о позициях"""
        if self.shutdown_requested:
            return False
            
        current_time = time.time()
        
        # Позиция считается актуальной, если обновлялась не позднее pose_timeout секунд назад
        robot_valid = (self.robot_pose_last_time is not None and 
                      current_time - self.robot_pose_last_time <= self.pose_timeout)
        enemy_valid = (self.enemy_point_last_time is not None and 
                      current_time - self.enemy_point_last_time <= self.pose_timeout)
        
        return robot_valid and enemy_valid

    def parabolic_acceleration(self, target_speed):
        """Параболическая функция для плавного начального разгона"""
        if self.initial_acceleration_complete:
            return target_speed
            
        # Постепенно увеличиваем фактор ускорения
        self.initial_acceleration_factor += self.acceleration_step
        
        # Когда достигаем 1.0 - начальный разгон завершен
        if self.initial_acceleration_factor >= 1.0:
            self.initial_acceleration_factor = 1.0
            self.initial_acceleration_complete = True
            
        # Параболическая зависимость: скорость = target_speed * (factor^2)
        parabolic_factor = self.initial_acceleration_factor ** 2
        
        return target_speed * parabolic_factor

    def calculate_shutdown_speed(self):
        """Линейное снижение скорости во время выключения"""
        if self.shutdown_start_time is None:
            return 0.0
            
        elapsed_time = time.time() - self.shutdown_start_time
        
        # Проверка завершения процесса выключения
        if elapsed_time >= self.shutdown_duration:
            self.shutdown_complete = True
            return 0.0
        
        # Линейное уменьшение от текущей скорости до 0
        progress = elapsed_time / self.shutdown_duration
        shutdown_speed = self.current_speed * (1.0 - progress)
        
        return max(0.0, shutdown_speed)

    def calculate_target_speed_based_on_distance(self):
        """Основной алгоритм расчета скорости на основе расстояния"""
        if not self.has_valid_poses():
            # Если позиции устарели - используем авторазгон
            return self.auto_ramp_target

        distance = self.calculate_distance(self.robot_pose, self.enemy_point)

        # Логика регулировки скорости по расстоянию:
        # ≥1.5м: полная скорость (0.40)
        # <1.5м: линейное снижение до 0.30
        if distance >= self.far_distance:
            target_speed = self.far_speed
        elif distance <= 0.0:
            target_speed = self.close_speed
        else:
            # Линейная интерполяция в диапазоне [0, 1.5] метров
            ratio = distance / self.close_distance
            target_speed = self.close_speed + (self.far_speed - self.close_speed) * ratio

        # Ограничение максимальной скоростью
        return min(target_speed, self.max_speed_limit)

    def initiate_shutdown(self):
        """Инициализация процесса плавного выключения"""
        if not self.shutdown_requested:
            self.shutdown_requested = True
            self.shutdown_start_time = time.time()
            self.get_logger().info("Graceful shutdown initiated")

    def round_to_thousandths(self, value):
        """Утилитарная функция для округления до 3 знаков"""
        return round(value, 3)

    def update_speed(self):
        """Главный цикл управления скоростью - вызывается каждые 100 мс"""
        
        # === БЛОК ВЫКЛЮЧЕНИЯ ===
        # Приоритетная обработка команды останова
        if self.shutdown_requested:
            self.current_speed = self.calculate_shutdown_speed()
            
            # Публикация текущей скорости
            speed_msg = Float32()
            speed_msg.data = float(self.round_to_thousandths(self.current_speed))
            self.speed_publisher.publish(speed_msg)
            
            # Логирование процесса выключения
            if not self.shutdown_complete:
                elapsed = time.time() - self.shutdown_start_time
                remaining = max(0.0, self.shutdown_duration - elapsed)
                self.get_logger().info(
                    f"SHUTDOWN: Speed: {self.current_speed:.3f} | Remaining: {remaining:.1f}s",
                    throttle_duration_sec=1.0
                )
            else:
                self.get_logger().info("Shutdown complete")
            
            return

        # === БЛОК НОРМАЛЬНОЙ РАБОТЫ ===
        # Определение режима работы и расчет целевой скорости
        if self.has_valid_poses():
            # Режим регулировки по расстоянию
            calculated_speed = self.calculate_target_speed_based_on_distance()
            
            # Применяем параболический разгон только если нужен разгон
            if not self.initial_acceleration_complete and calculated_speed > self.current_speed:
                self.target_speed = self.parabolic_acceleration(calculated_speed)
            else:
                self.target_speed = calculated_speed
                
            mode = "DISTANCE"
        else:
            # Авторежим - разгон до безопасной скорости
            if not self.initial_acceleration_complete:
                self.target_speed = self.parabolic_acceleration(self.auto_ramp_target)
            else:
                self.target_speed = self.auto_ramp_target
            mode = "AUTO"
        
        # === ПЛАВНОЕ ИЗМЕНЕНИЕ СКОРОСТИ ===
        # Ramp-контроллер: плавное приближение к целевой скорости
        speed_diff = self.target_speed - self.current_speed
        
        if abs(speed_diff) <= self.ramp_rate:
            # Малая разница - устанавливаем точно целевую скорость
            self.current_speed = self.target_speed
        else:
            # Плавное изменение с шагом ramp_rate
            self.current_speed += math.copysign(self.ramp_rate, speed_diff)
        
        # Гарантия безопасного диапазона скорости
        self.current_speed = max(0.0, min(self.current_speed, self.max_speed_limit))

        # Публикация скорости в систему
        speed_msg = Float32()
        speed_msg.data = float(self.round_to_thousandths(self.current_speed))
        self.speed_publisher.publish(speed_msg)

        # === ДИАГНОСТИКА И ЛОГИРОВАНИЕ ===
        # Логируем состояние системы (с троттлингом 1 Гц)
        current_time = time.time()
        robot_time_diff = current_time - self.robot_pose_last_time if self.robot_pose_last_time else float('inf')
        enemy_time_diff = current_time - self.enemy_point_last_time if self.enemy_point_last_time else float('inf')
        
        distance = self.calculate_distance(self.robot_pose, self.enemy_point) if self.has_valid_poses() else float('inf')
        
        self.get_logger().info(
            f"Mode: {mode} | Speed: {self.current_speed:.3f} | Target: {self.target_speed:.3f} | "
            f"AccelFactor: {self.initial_acceleration_factor:.3f} | "
            f"Distance: {distance:.2f}m | Poses valid: {self.has_valid_poses()}",
            throttle_duration_sec=1.0
        )


def main(args=None):
    """Точка входа ROS2 узла"""
    rclpy.init(args=args)
    node = WeaponController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user")
    finally:
        # Гарантированная публикация нулевой скорости при завершении
        try:
            speed_msg = Float32()
            speed_msg.data = 0.000
            node.speed_publisher.publish(speed_msg)
            rclpy.spin_once(node, timeout_sec=0.1)
        except:
            pass
            
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
