import math
import numpy as np


class Robot:
    def __init__(self, start_x=-4.0, start_y=-4.0, start_yaw=0.0):
        # Ширина - БОЛЬШАЯ сторона (боковые стороны с колесами)
        # Длина - МАЛЕНЬКАЯ сторона (перед/зад)
        self.width = 0.8  # 800мм - широкая сторона (боковые стороны с колесами)
        self.length = 0.7  # 700мм - узкая сторона (перед/зад)
        
        # Орудие - горизонтальное (вдоль оси X робота)
        self.weapon_length = 0.2  # толщина орудия (вдоль оси Y робота)
        self.weapon_width = 0.6   # длина орудия (вдоль оси X робота)
        
        # Текущее состояние
        self.x = start_x
        self.y = start_y
        
        # ИСПРАВЛЕНО: Робот повернут так, что орудие смотрит на противника
        # Ось X робота (вперед) направлена на противника
        self.yaw = start_yaw
        
        # Скорости
        self.vx_local = 0.0
        self.vy_local = 0.0
        self.v_yaw = 0.0
        
        # Коэффициенты движения
        self.max_linear_speed = 2.0
        self.max_angular_speed = 3.0
        self.acceleration_coef = 0.2
        self.deceleration_coef = 0.3
        self.slip_coefficient = 0.1
        
        # Целевые скорости
        self.target_vx_local = 0.0
        self.target_vy_local = 0.0
        self.target_v_yaw = 0.0
        
        # Орудие (0.0 = зеленый, 0.5+ = красный)
        self.weapon_value = 0.0
        
        # История позиций
        self.position_history = []
        self.max_history = 100
        self.is_moving = False
    
    def update_motion(self, dt):
        """Обновить движение"""
        # Линейная скорость
        if abs(self.target_vx_local - self.vx_local) > 0.01:
            if self.target_vx_local > self.vx_local:
                self.vx_local += self.acceleration_coef * dt * self.max_linear_speed
                self.vx_local = min(self.vx_local, self.target_vx_local)
            else:
                self.vx_local -= self.deceleration_coef * dt * self.max_linear_speed
                self.vx_local = max(self.vx_local, self.target_vx_local)
        else:
            if abs(self.vx_local) > 0.01:
                self.vx_local *= (1.0 - self.slip_coefficient * dt * 10)
            else:
                self.vx_local = 0.0
        
        # Угловая скорость
        if abs(self.target_v_yaw - self.v_yaw) > 0.01:
            if self.target_v_yaw > self.v_yaw:
                self.v_yaw += self.acceleration_coef * dt * self.max_angular_speed
                self.v_yaw = min(self.v_yaw, self.target_v_yaw)
            else:
                self.v_yaw -= self.deceleration_coef * dt * self.max_angular_speed
                self.v_yaw = max(self.v_yaw, self.target_v_yaw)
        else:
            if abs(self.v_yaw) > 0.01:
                self.v_yaw *= (1.0 - self.slip_coefficient * dt * 10)
            else:
                self.v_yaw = 0.0
        
        # Движение
        self.is_moving = abs(self.vx_local) > 0.01 or abs(self.v_yaw) > 0.01
        
        # Преобразование в глобальные скорости
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        
        vx_global = self.vx_local * cos_yaw - self.vy_local * sin_yaw
        vy_global = self.vx_local * sin_yaw + self.vy_local * cos_yaw
        
        # Интегрирование
        self.x += vx_global * dt
        self.y += vy_global * dt
        self.yaw += self.v_yaw * dt
        
        # Нормализация угла
        self.yaw = self.normalize_angle(self.yaw)
        
        # История
        if self.is_moving:
            self.position_history.append((self.x, self.y))
            if len(self.position_history) > self.max_history:
                self.position_history.pop(0)
    
    def set_target_speeds(self, vx_percent, vy_percent, v_yaw_percent):
        """Установить целевые скорости"""
        vx_percent = max(-1.0, min(1.0, vx_percent))
        vy_percent = max(-1.0, min(1.0, vy_percent))
        v_yaw_percent = max(-1.0, min(1.0, v_yaw_percent))
        
        self.target_vx_local = vx_percent * self.max_linear_speed
        self.target_vy_local = vy_percent * self.max_linear_speed
        self.target_v_yaw = v_yaw_percent * self.max_angular_speed
    
    def set_pose(self, x, y, yaw):
        """Установить позицию"""
        self.x = x
        self.y = y
        self.yaw = yaw
        # Сброс скоростей
        self.vx_local = 0.0
        self.vy_local = 0.0
        self.v_yaw = 0.0
        self.target_vx_local = 0.0
        self.target_vy_local = 0.0
        self.target_v_yaw = 0.0
    
    def set_weapon_value(self, value):
        """Установить значение орудия"""
        self.weapon_value = max(0.0, min(0.5, value))
    
    def get_corners(self):
        """Получить углы корпуса"""
        half_width = self.width / 2  # Большая сторона по X
        half_length = self.length / 2  # Малая сторона по Y
        
        # ИСПРАВЛЕНО: Ось X робота направлена вперед (на противника)
        # Робот повернут так, что его длинная сторона (width) теперь перпендикулярна направлению движения
        corners_local = [
            (half_length, half_width),    # правый передний
            (half_length, -half_width),   # левый передний
            (-half_length, -half_width),  # левый задний
            (-half_length, half_width)    # правый задний
        ]
        
        return self.transform_points(corners_local)
    
    def get_wheel_rectangles(self):
        """Получить прямоугольники колес"""
        # ИСПРАВЛЕНО: Колеса повернуты на 90 градусов
        # Теперь колеса вытянуты вдоль направления движения (по оси Y)
        wheel_length = 0.1    # вдоль оси Y (вперед-назад)
        wheel_width = 0.15    # вдоль оси X (влево-вправо)
        
        # Позиции колес - на углах широкой стороны
        wheel_offsets = [
            (self.length/2 - wheel_length/2, self.width/2 - wheel_width/2),   # переднее правое
            (self.length/2 - wheel_length/2, -self.width/2 + wheel_width/2),  # переднее левое
            (-self.length/2 + wheel_length/2, self.width/2 - wheel_width/2),  # заднее правое
            (-self.length/2 + wheel_length/2, -self.width/2 + wheel_width/2), # заднее левое
        ]
        
        wheel_rectangles = []
        for offset_x, offset_y in wheel_offsets:
            # Колеса вытянуты вдоль оси X робота
            wheel_corners_local = [
                (offset_x + wheel_length/2, offset_y + wheel_width/2),
                (offset_x + wheel_length/2, offset_y - wheel_width/2),
                (offset_x - wheel_length/2, offset_y - wheel_width/2),
                (offset_x - wheel_length/2, offset_y + wheel_width/2),
            ]
            
            wheel_corners_global = self.transform_points(wheel_corners_local)
            wheel_rectangles.append(wheel_corners_global)
        
        return wheel_rectangles
    
    def get_weapon_corners(self):
        """Получить углы орудия"""
        # Орудие расположено спереди, горизонтально (вдоль оси X)
        weapon_offset_x = self.length / 2  # Смещение вперед
        half_weapon_width = self.weapon_width / 2  # Длина орудия/2 (вдоль оси Y)
        half_weapon_length = self.weapon_length / 2  # Толщина орудия/2 (вдоль оси X)
        
        # ИСПРАВЛЕНО: Орудие горизонтальное, вытянуто вдоль широкой стороны
        corners_local = [
            (weapon_offset_x + half_weapon_length, half_weapon_width),   # правый передний
            (weapon_offset_x + half_weapon_length, -half_weapon_width),  # левый передний
            (weapon_offset_x - half_weapon_length, -half_weapon_width),  # левый задний
            (weapon_offset_x - half_weapon_length, half_weapon_width),   # правый задний
        ]
        
        return self.transform_points(corners_local)
    
    def get_direction_line(self):
        """Получить линию направления (от центра к переду)"""
        # ИСПРАВЛЕНО: Линия направления совпадает с осью орудия
        # Длина линии = половина длины робота + половина орудия
        line_length = self.length / 2 + self.weapon_length / 2 + 0.1
        
        start_local = (0, 0)
        end_local = (line_length, 0)  # Вдоль оси X робота
        
        start_global = self.transform_points([start_local])[0]
        end_global = self.transform_points([end_local])[0]
        
        return start_global, end_global
    
    def transform_points(self, points_local):
        """Преобразовать точки в глобальную систему"""
        points_global = []
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        
        for xl, yl in points_local:
            xg = self.x + xl * cos_yaw - yl * sin_yaw
            yg = self.y + xl * sin_yaw + yl * cos_yaw
            points_global.append((xg, yg))
        
        return points_global
    
    def check_boundary_collision(self, field_size):
        """Проверить столкновение с границами"""
        half_size = field_size / 2
        collision = False
        
        all_corners = self.get_corners() + self.get_weapon_corners()
        
        for x, y in all_corners:
            if abs(x) > half_size:
                self.x = np.sign(self.x) * (half_size - self.width/2 - 0.1)
                self.vx_local *= -0.5
                self.target_vx_local = 0.0
                collision = True
            
            if abs(y) > half_size:
                self.y = np.sign(self.y) * (half_size - self.length/2 - 0.1)
                self.vx_local *= 0.8
                self.target_vx_local = 0.0
                collision = True
        
        return collision
    
    @staticmethod
    def normalize_angle(angle):
        """Нормализовать угол"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
