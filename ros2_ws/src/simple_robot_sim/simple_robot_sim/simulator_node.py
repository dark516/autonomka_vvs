#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import pygame
import numpy as np
import math

from geometry_msgs.msg import PoseStamped, TwistStamped, PointStamped
from std_msgs.msg import Float32
from nav_msgs.msg import Path

from .robot import Robot


class SimulatorNode(Node):
    def __init__(self):
        super().__init__('simulator_node')
        
        # Параметры
        self.declare_parameter('field_size', 10.0)
        self.declare_parameter('window_size', 1000)
        self.declare_parameter('publish_rate', 30.0)
        
        self.field_size = self.get_parameter('field_size').value
        self.window_size = self.get_parameter('window_size').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # Масштаб
        self.scale = self.window_size / self.field_size
        
        # Инициализация PyGame
        pygame.init()
        self.screen = pygame.display.set_mode((self.window_size, self.window_size))
        pygame.display.set_caption('Robot Simulator 2D')
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont('Arial', 16)
        self.small_font = pygame.font.SysFont('Arial', 12)
        
        # Начальные позиции
        start_x = -4.0
        start_y = -4.0
        enemy_x = 4.0
        enemy_y = 4.0
        
        # ИСПРАВЛЕНО: Рассчитываем угол так, чтобы орудие смотрело на противника
        # Вектор от нашего робота к противнику
        dx = enemy_x - start_x
        dy = enemy_y - start_y
        
        # Угол направления на противника
        target_yaw = math.atan2(dy, dx)
        
        # Создание робота с правильным поворотом
        self.robot = Robot(start_x=start_x, start_y=start_y, start_yaw=target_yaw)
        
        # Противник
        self.enemy_x = enemy_x
        self.enemy_y = enemy_y
        self.enemy_radius = 0.075
        self.enemy_dragging = False
        
        # Путь
        self.planned_path = []
        
        # Состояние симуляции
        self.simulation_paused = False
        self.last_update_time = self.get_clock().now()
        
        # Подписки
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )
        
        self.cmd_vel_sub = self.create_subscription(
            TwistStamped,
            '/cmd_vel',
            self.cmd_vel_callback,
            qos_profile
        )
        
        self.weapon_sub = self.create_subscription(
            Float32,
            '/robot/weapon',
            self.weapon_callback,
            10
        )
        
        self.path_sub = self.create_subscription(
            Path,
            '/planned_path',
            self.path_callback,
            10
        )
        
        # ИСПРАВЛЕНО: Теперь симулятор тоже публикует координаты противника
        # при перетаскивании
        self.enemy_point_pub = self.create_publisher(
            PointStamped,
            '/enemy_point',
            10
        )
        
        # Публикатор позиции робота
        self.robot_pose_pub = self.create_publisher(
            PoseStamped,
            '/robot_pose',
            10
        )
        
        # Таймеры
        self.create_timer(1.0 / self.publish_rate, self.publish_robot_state)
        self.create_timer(1.0 / self.publish_rate, self.publish_enemy_state)
        
        self.get_logger().info('Симулятор инициализирован')
        self.get_logger().info(f'Наш робот: ({self.robot.x:.1f}, {self.robot.y:.1f})')
        self.get_logger().info(f'Угол: {math.degrees(self.robot.yaw):.1f}°')
        self.get_logger().info(f'Противник: ({self.enemy_x:.1f}, {self.enemy_y:.1f})')
        self.get_logger().info('Симулятор публикует /robot_pose и /enemy_point')
    
    def meter_to_pixel(self, x, y):
        """Преобразовать координаты в пиксели"""
        px = self.window_size / 2 + x * self.scale
        py = self.window_size / 2 - y * self.scale
        return int(px), int(py)
    
    def pixel_to_meter(self, px, py):
        """Преобразовать пиксели в метры"""
        x = (px - self.window_size / 2) / self.scale
        y = (self.window_size / 2 - py) / self.scale
        return x, y
    
    def cmd_vel_callback(self, msg):
        """Обработка команд движения"""
        vx_percent = msg.twist.linear.x
        vy_percent = msg.twist.linear.y
        v_yaw_percent = msg.twist.angular.z
        
        vx_percent = max(-1.0, min(1.0, vx_percent))
        vy_percent = max(-1.0, min(1.0, vy_percent))
        v_yaw_percent = max(-1.0, min(1.0, v_yaw_percent))
        
        self.robot.set_target_speeds(vx_percent, vy_percent, v_yaw_percent)
    
    def weapon_callback(self, msg):
        """Обработка значения орудия"""
        self.robot.set_weapon_value(msg.data)
    
    def path_callback(self, msg):
        """Обработка пути"""
        if msg.poses:
            self.planned_path = []
            for pose_stamped in msg.poses:
                x = pose_stamped.pose.position.x
                y = pose_stamped.pose.position.y
                self.planned_path.append((x, y))
    
    def publish_robot_state(self):
        """Публикация состояния робота"""
        if self.simulation_paused:
            return
        
        current_time = self.get_clock().now()
        
        # Позиция робота
        robot_pose = PoseStamped()
        robot_pose.header.stamp = current_time.to_msg()
        robot_pose.header.frame_id = 'map'
        robot_pose.pose.position.x = self.robot.x
        robot_pose.pose.position.y = self.robot.y
        robot_pose.pose.position.z = 0.0
        
        # Кватернион из угла
        cy = math.cos(self.robot.yaw * 0.5)
        sy = math.sin(self.robot.yaw * 0.5)
        
        robot_pose.pose.orientation.x = 0.0
        robot_pose.pose.orientation.y = 0.0
        robot_pose.pose.orientation.z = sy
        robot_pose.pose.orientation.w = cy
        
        self.robot_pose_pub.publish(robot_pose)
    
    def publish_enemy_state(self):
        """Публикация текущих координат противника"""
        if self.simulation_paused:
            return
        
        current_time = self.get_clock().now()
        
        # ИСПРАВЛЕНО: Всегда публикуем ТЕКУЩИЕ координаты противника
        enemy_point = PointStamped()
        enemy_point.header.stamp = current_time.to_msg()
        enemy_point.header.frame_id = 'map'
        enemy_point.point.x = self.enemy_x
        enemy_point.point.y = self.enemy_y
        enemy_point.point.z = 0.0
        
        self.enemy_point_pub.publish(enemy_point)
    
    def draw_robot(self):
        """Отрисовка робота"""
        # Корпус
        corners = self.robot.get_corners()
        pixel_corners = [self.meter_to_pixel(x, y) for x, y in corners]
        
        if len(pixel_corners) >= 3:
            pygame.draw.polygon(self.screen, (30, 136, 229), pixel_corners)
            pygame.draw.polygon(self.screen, (0, 0, 0), pixel_corners, 2)
        
        # Колеса (повернуты на 90 градусов)
        wheel_rectangles = self.robot.get_wheel_rectangles()
        for wheel_corners in wheel_rectangles:
            pixel_wheel = [self.meter_to_pixel(x, y) for x, y in wheel_corners]
            if len(pixel_wheel) >= 3:
                pygame.draw.polygon(self.screen, (50, 50, 50), pixel_wheel)
                pygame.draw.polygon(self.screen, (0, 0, 0), pixel_wheel, 1)
                
                # Протектор
                if len(pixel_wheel) >= 4:
                    mid1 = ((pixel_wheel[0][0] + pixel_wheel[3][0]) // 2,
                           (pixel_wheel[0][1] + pixel_wheel[3][1]) // 2)
                    mid2 = ((pixel_wheel[1][0] + pixel_wheel[2][0]) // 2,
                           (pixel_wheel[1][1] + pixel_wheel[2][1]) // 2)
                    pygame.draw.line(self.screen, (150, 150, 150), mid1, mid2, 2)
        
        # Орудие (горизонтальное)
        weapon_corners = self.robot.get_weapon_corners()
        pixel_weapon = [self.meter_to_pixel(x, y) for x, y in weapon_corners]
        
        # Цвет орудия (0.0 = зеленый, 0.5 = красный)
        weapon_value = self.robot.weapon_value
        green = int(76 * (1.0 - weapon_value * 2))
        red = int(244 * min(1.0, weapon_value * 2))
        
        if len(pixel_weapon) >= 3:
            pygame.draw.polygon(self.screen, (red, green, 50), pixel_weapon)
            pygame.draw.polygon(self.screen, (0, 0, 0), pixel_weapon, 2)
            
            # Значение орудия
            weapon_text = f"{weapon_value:.2f}"
            text = self.small_font.render(weapon_text, True, (255, 255, 255))
            if len(pixel_weapon) >= 4:
                center_x = sum(p[0] for p in pixel_weapon) // len(pixel_weapon)
                center_y = sum(p[1] for p in pixel_weapon) // len(pixel_weapon)
                self.screen.blit(text, (center_x - 15, center_y - 6))
        
        # ИСПРАВЛЕНО: Линия направления (совпадает с осью орудия)
        direction_start, direction_end = self.robot.get_direction_line()
        px1, py1 = self.meter_to_pixel(*direction_start)
        px2, py2 = self.meter_to_pixel(*direction_end)
        
        # Толстая красная линия направления
        pygame.draw.line(self.screen, (255, 0, 0), (px1, py1), (px2, py2), 3)
        
        # Стрелка на конце
        arrow_length = 10
        arrow_angle = math.atan2(py2 - py1, px2 - px1)
        
        arrow1 = (
            px2 - arrow_length * math.cos(arrow_angle - math.pi/6),
            py2 - arrow_length * math.sin(arrow_angle - math.pi/6)
        )
        arrow2 = (
            px2 - arrow_length * math.cos(arrow_angle + math.pi/6),
            py2 - arrow_length * math.sin(arrow_angle + math.pi/6)
        )
        
        pygame.draw.line(self.screen, (255, 0, 0), (px2, py2), arrow1, 2)
        pygame.draw.line(self.screen, (255, 0, 0), (px2, py2), arrow2, 2)
        
        # Текст "Наш робот"
        text = self.small_font.render("Наш робот", True, (0, 0, 0))
        self.screen.blit(text, (px1 - 25, py1 - 25))
    
    def draw_enemy(self):
        """Отрисовка противника"""
        px, py = self.meter_to_pixel(self.enemy_x, self.enemy_y)
        radius = int(self.enemy_radius * self.scale)
        
        pygame.draw.circle(self.screen, (255, 152, 0), (px, py), radius)
        pygame.draw.circle(self.screen, (0, 0, 0), (px, py), radius, 2)
        
        # Крестик
        pygame.draw.line(self.screen, (0, 0, 0), 
                        (px - radius//2, py), (px + radius//2, py), 2)
        pygame.draw.line(self.screen, (0, 0, 0), 
                        (px, py - radius//2), (px, py + radius//2), 2)
        
        # Текст
        text = self.small_font.render("Противник", True, (0, 0, 0))
        self.screen.blit(text, (px - 30, py - 25))
        
        coord_text = f"({self.enemy_x:.1f}, {self.enemy_y:.1f})"
        text = self.small_font.render(coord_text, True, (0, 0, 0))
        self.screen.blit(text, (px - 25, py + 15))
    
    def draw_path(self):
        """Отрисовка пути"""
        if self.planned_path and len(self.planned_path) > 1:
            pixel_points = []
            for x, y in self.planned_path:
                px, py = self.meter_to_pixel(x, y)
                pixel_points.append((px, py))
            
            if len(pixel_points) > 1:
                pygame.draw.lines(self.screen, (0, 200, 83), False, pixel_points, 3)
            
            for i, (px, py) in enumerate(pixel_points):
                pygame.draw.circle(self.screen, (100, 221, 23), (px, py), 5)
                pygame.draw.circle(self.screen, (0, 0, 0), (px, py), 5, 1)
    
    def draw_grid(self):
        """Отрисовка сетки"""
        self.screen.fill((245, 245, 245))
        
        grid_step = 1.0
        half_field = self.field_size / 2
        
        # Вертикальные линии
        for i in range(int(self.field_size / grid_step) + 1):
            x = -half_field + i * grid_step
            px, _ = self.meter_to_pixel(x, 0)
            color = (200, 200, 200) if i % 2 == 0 else (220, 220, 220)
            pygame.draw.line(self.screen, color, (px, 0), (px, self.window_size), 1)
            
            if i % 2 == 0 and x != 0:
                text = self.small_font.render(f"{x:.0f}", True, (150, 150, 150))
                self.screen.blit(text, (px - 10, self.window_size - 20))
        
        # Горизонтальные линии
        for i in range(int(self.field_size / grid_step) + 1):
            y = -half_field + i * grid_step
            _, py = self.meter_to_pixel(0, y)
            color = (200, 200, 200) if i % 2 == 0 else (220, 220, 220)
            pygame.draw.line(self.screen, color, (0, py), (self.window_size, py), 1)
            
            if i % 2 == 0 and y != 0:
                text = self.small_font.render(f"{y:.0f}", True, (150, 150, 150))
                self.screen.blit(text, (10, py - 10))
        
        # Центральные оси
        center_x, center_y = self.meter_to_pixel(0, 0)
        pygame.draw.line(self.screen, (150, 150, 150),
                        (center_x, 0), (center_x, self.window_size), 2)
        pygame.draw.line(self.screen, (150, 150, 150),
                        (0, center_y), (self.window_size, center_y), 2)
        
        # Границы
        border_rect = pygame.Rect(
            self.meter_to_pixel(-half_field, half_field)[0],
            self.meter_to_pixel(-half_field, half_field)[1],
            int(self.field_size * self.scale),
            int(self.field_size * self.scale)
        )
        pygame.draw.rect(self.screen, (0, 0, 0), border_rect, 3)
    
    def draw_info(self):
        """Отрисовка информации"""
        info_rect = pygame.Rect(5, 5, 320, 190)
        s = pygame.Surface((info_rect.width, info_rect.height), pygame.SRCALPHA)
        s.fill((255, 255, 255, 220))
        self.screen.blit(s, (info_rect.x, info_rect.y))
        pygame.draw.rect(self.screen, (0, 0, 0), info_rect, 1)
        
        # Угол к противнику
        dx = self.enemy_x - self.robot.x
        dy = self.enemy_y - self.robot.y
        distance = math.hypot(dx, dy)
        angle_to_enemy = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(angle_to_enemy - self.robot.yaw)
        
        info_lines = [
            "=== НАШ РОБОТ ===",
            f"Позиция: ({self.robot.x:.2f}, {self.robot.y:.2f})",
            f"Угол: {math.degrees(self.robot.yaw):.1f}°",
            f"Скорость: {self.robot.vx_local:.2f} м/с",
            f"Орудие: {self.robot.weapon_value:.2f}",
            "",
            "=== ПРОТИВНИК ===",
            f"Позиция: ({self.enemy_x:.2f}, {self.enemy_y:.2f})",
            f"Дистанция: {distance:.2f} м",
            f"Угол к нему: {math.degrees(angle_diff):.1f}°",
            "",
            "=== УПРАВЛЕНИЕ ===",
            "ПРОБЕЛ - пауза",
            "R - сброс",
            "C - очистить путь",
            f"FPS: {int(self.clock.get_fps())}"
        ]
        
        for i, line in enumerate(info_lines):
            color = (0, 0, 0)
            if "===" in line:
                color = (0, 100, 200)
            
            text = self.small_font.render(line, True, color)
            self.screen.blit(text, (10, 10 + i * 15))
    
    @staticmethod
    def normalize_angle(angle):
        """Нормализовать угол"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def handle_events(self):
        """Обработка событий"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    self.simulation_paused = not self.simulation_paused
                    status = "ПРИОСТАНОВЛЕНА" if self.simulation_paused else "ВОЗОБНОВЛЕНА"
                    self.get_logger().info(f"Симуляция {status}")
                
                elif event.key == pygame.K_r:
                    # Сброс с сохранением ориентации на противника
                    dx = self.enemy_x - (-4.0)
                    dy = self.enemy_y - (-4.0)
                    reset_yaw = math.atan2(dy, dx)
                    
                    self.robot.set_pose(-4.0, -4.0, reset_yaw)
                    self.enemy_x = 4.0
                    self.enemy_y = 4.0
                    self.planned_path = []
                    self.get_logger().info("Симуляция сброшена")
                
                elif event.key == pygame.K_c:
                    self.planned_path = []
                    self.get_logger().info("Путь очищен")
            
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:
                    mouse_x, mouse_y = event.pos
                    enemy_px, enemy_py = self.meter_to_pixel(self.enemy_x, self.enemy_y)
                    enemy_radius_px = int(self.enemy_radius * self.scale)
                    
                    distance = math.sqrt((mouse_x - enemy_px)**2 + (mouse_y - enemy_py)**2)
                    if distance <= enemy_radius_px:
                        self.enemy_dragging = True
                        self.get_logger().info(f"Начато перетаскивание противника")
            
            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1 and self.enemy_dragging:
                    self.enemy_dragging = False
                    self.get_logger().info(f"Противник перемещен в ({self.enemy_x:.2f}, {self.enemy_y:.2f})")
            
            elif event.type == pygame.MOUSEMOTION:
                if self.enemy_dragging:
                    mouse_x, mouse_y = event.pos
                    old_x, old_y = self.enemy_x, self.enemy_y
                    self.enemy_x, self.enemy_y = self.pixel_to_meter(mouse_x, mouse_y)
                    
                    # Ограничение
                    half_size = self.field_size / 2
                    self.enemy_x = max(-half_size + 0.1, min(half_size - 0.1, self.enemy_x))
                    self.enemy_y = max(-half_size + 0.1, min(half_size - 0.1, self.enemy_y))
        
        return True
    
    def run(self):
        """Главный цикл"""
        self.get_logger().info("Запуск симулятора...")
        
        try:
            running = True
            while running and rclpy.ok():
                running = self.handle_events()
                
                # Обновление физики
                if not self.simulation_paused:
                    current_time = self.get_clock().now()
                    dt = (current_time - self.last_update_time).nanoseconds / 1e9
                    
                    if dt > 0.1:
                        dt = 0.1
                    
                    self.robot.update_motion(dt)
                    self.robot.check_boundary_collision(self.field_size)
                    
                    self.last_update_time = current_time
                
                # Отрисовка
                self.draw_grid()
                self.draw_path()
                self.draw_enemy()
                self.draw_robot()
                self.draw_info()
                
                pygame.display.flip()
                self.clock.tick(60)
                
                rclpy.spin_once(self, timeout_sec=0.001)
        
        except KeyboardInterrupt:
            self.get_logger().info("Симулятор остановлен")
        finally:
            pygame.quit()
            self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SimulatorNode()
    
    try:
        node.run()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
