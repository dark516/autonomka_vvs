import rclpy
from rclpy.node import Node
import math
import numpy as np
from enum import Enum
from typing import List, Tuple, Dict, Any

from geometry_msgs.msg import PoseStamped, PointStamped, Pose, Quaternion
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA


class RobotType(Enum):
    """Types of robot shapes."""
    circle = 0
    rectangle = 1


class Config:
    """Configuration for DWA."""
    def __init__(self):
        # DWA parameters
        self.max_speed: float = 0.5
        self.min_speed: float = 0.0
        self.max_yaw_rate: float = 100.0 * math.pi / 180.0
        self.max_accel: float = 0.5
        self.max_delta_yaw_rate: float = 120.0 * math.pi / 180.0
        self.v_resolution: float = 0.05
        self.yaw_rate_resolution: float = 1.0 * math.pi / 180.0
        self.dt: float = 0.45
        self.predict_time: float = 5.5

        # Cost gains
        self.to_goal_cost_gain: float = 3.0
        self.speed_cost_gain: float = 1.0
        self.obstacle_cost_gain: float = 1.0

        # Robot / world
        self.robot_radius: float = 0.4
        self.robot_type: RobotType = RobotType.circle

        # Known obstacles
        self.ob: np.ndarray = np.array([[]])  # shape=(1,0)


def euler_from_quaternion(q: Quaternion) -> float:
    """Converts a geometry_msgs.msg.Quaternion to a yaw angle."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def motion(x: np.ndarray, u: List[float], dt: float) -> np.ndarray:
    """
    Robot motion model.
    x = [x, y, yaw, v, omega]
    u = [v_cmd, omega_cmd]
    """
    x[2] += u[1] * dt
    x[0] += u[0] * math.cos(x[2]) * dt
    x[1] += u[0] * math.sin(x[2]) * dt
    x[3] = u[0]
    x[4] = u[1]
    return x


def calc_dynamic_window(x: np.ndarray, cfg: Config) -> List[float]:
    """Calculates the dynamic window (admissible velocities)."""
    # [min_v, max_v, min_yaw_rate, max_yaw_rate]
    Vs: List[float] = [cfg.min_speed, cfg.max_speed, -cfg.max_yaw_rate, cfg.max_yaw_rate]
    
    # Constraints based on acceleration
    Vd: List[float] = [x[3] - cfg.max_accel * cfg.dt,
                       x[3] + cfg.max_accel * cfg.dt,
                       x[4] - cfg.max_delta_yaw_rate * cfg.dt,
                       x[4] + cfg.max_delta_yaw_rate * cfg.dt]
                       
    # Intersection of Vs and Vd
    return [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
            max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]


def predict_trajectory(x_init: np.ndarray, v: float, y: float, cfg: Config) -> np.ndarray:
    """Predicts trajectory based on constant control [v, y]."""
    x = np.array(x_init)
    trajectory = np.array([x])
    t = 0.0
    while t <= cfg.predict_time:
        x = motion(x, [v, y], cfg.dt)
        trajectory = np.vstack((trajectory, x))
        t += cfg.dt
    return trajectory


def calc_obstacle_cost(trajectory: np.ndarray, ob: np.ndarray, cfg: Config) -> float:
    """Calculates the obstacle cost."""
    if ob.size == 0 or ob.shape[0] == 0:
        return 0.0
        
    ox, oy = ob[:, 0], ob[:, 1]
    
    # Calculate distances from all trajectory points to all obstacles
    dx = trajectory[:, 0].reshape(-1, 1) - ox.reshape(1, -1)
    dy = trajectory[:, 1].reshape(-1, 1) - oy.reshape(1, -1)
    r = np.hypot(dx, dy)
    
    # Infinite cost on collision
    if np.any(r <= cfg.robot_radius):
        return float("inf")
        
    # Cost is inversely proportional to the minimum safe distance
    min_r: float = float(np.min(r))
    if min_r == 0:
        return float("inf")
        
    return cfg.robot_radius / min_r


def calc_to_goal_cost(trajectory: np.ndarray, goal_xy: np.ndarray) -> float:
    """Calculates the cost to reach the goal (distance + orientation)."""
    # Distance component
    dist_cost: float = math.hypot(trajectory[-1, 0] - goal_xy[0],
                                  trajectory[-1, 1] - goal_xy[1])

    # Orientation component
    dx: float = goal_xy[0] - trajectory[-1, 0]
    dy: float = goal_xy[1] - trajectory[-1, 1]
    goal_angle: float = math.atan2(dy, dx)
    robot_angle: float = trajectory[-1, 2]

    # Normalize angle difference
    angle_diff: float = normalize_angle(goal_angle - robot_angle)

    cost_angle: float = abs(angle_diff)

    # Additional penalty if the robot is facing away from the goal
    if cost_angle > math.pi / 2:
        cost_angle *= 2.0

    return dist_cost + cost_angle


# --- Node ---------------------------------------------------------------------

class DWAPathPlannerNode(Node):
    """The path planning node using the Dynamic Window Approach (DWA)."""
    def __init__(self):
        super().__init__('dwa_path_planner_node')
        self.get_logger().info("DWA Path Planner node started.")

        self.cfg: Config = Config()

        # robot state [x, y, yaw, v, omega]
        self.state: np.ndarray = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        self.goal_xy: np.ndarray = np.array([10.0, 10.0])
        self.frame_id: str = 'map'
        self.has_pose: bool = False

        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped, '/robot_pose', self.pose_cb, 10)
        self.enemy_sub = self.create_subscription(
            PointStamped, '/enemy_point', self.enemy_cb, 10)

        # Publishers
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.traj_viz_pub = self.create_publisher(MarkerArray, '/dwa_trajectories', 10)

        # Timer
        self.timer = self.create_timer(self.cfg.dt, self.loop)

    def pose_cb(self, msg: PoseStamped) -> None:
        """Callback for incoming robot pose."""
        p: Pose = msg.pose
        self.frame_id = msg.header.frame_id or 'map'
        
        self.state[0] = p.position.x
        self.state[1] = p.position.y
        self.state[2] = euler_from_quaternion(p.orientation)
        
        self.has_pose = True

    def enemy_cb(self, msg: PointStamped) -> None:
        """Callback for incoming goal point (enemy/target)."""
        self.goal_xy[0] = msg.point.x
        self.goal_xy[1] = msg.point.y

    def dwa_control(self) -> Tuple[List[float], np.ndarray]:
        """
        The main DWA algorithm.
        Returns: [best_v, best_yaw_rate], best_trajectory
        """
        x: np.ndarray = self.state.copy()
        goal: np.ndarray = self.goal_xy.copy()
        ob: np.ndarray = self.cfg.ob

        dw: List[float] = calc_dynamic_window(x, self.cfg)

        x_init: np.ndarray = x[:]
        min_cost: float = float('inf')
        best_u: List[float] = [0.0, 0.0]
        best_traj: np.ndarray = np.array([x])

        # Baseline: safe stop trajectory to handle global collisions
        traj_stop: np.ndarray = predict_trajectory(x_init, 0.0, 0.0, self.cfg)
        ob_cost_stop: float = self.cfg.obstacle_cost_gain * calc_obstacle_cost(traj_stop, ob, self.cfg)
        
        if ob_cost_stop != float('inf'):
            # Set initial non-inf cost if stopping is safe
            min_cost = 1000.0
            best_u = [0.0, 0.0]
            best_traj = traj_stop

        # Iterate through all sampled velocities within the dynamic window
        for v in np.arange(dw[0], dw[1], self.cfg.v_resolution):
            for y in np.arange(dw[2], dw[3], self.cfg.yaw_rate_resolution):
                traj: np.ndarray = predict_trajectory(x_init, v, y, self.cfg)

                # Calculate cost components
                to_goal_cost: float = self.cfg.to_goal_cost_gain * calc_to_goal_cost(traj, goal)
                speed_cost: float = self.cfg.speed_cost_gain * (self.cfg.max_speed - v)
                ob_cost: float = self.cfg.obstacle_cost_gain * calc_obstacle_cost(traj, ob, self.cfg)

                final_cost: float = to_goal_cost + speed_cost + ob_cost

                if final_cost < min_cost:
                    min_cost = final_cost
                    best_u = [float(v), float(y)]
                    best_traj = traj

        if min_cost == float('inf'):
            self.get_logger().warn("DWA: all sampled paths collide; publishing stop path.")
            return [0.0, 0.0], np.array([x])

        return best_u, best_traj

    def trajectory_to_path(self, traj: np.ndarray) -> Path:
        """Converts the best trajectory (np.ndarray) into a ROS 2 Path message."""
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.frame_id

        # traj rows: [x, y, yaw, v, omega]
        for row in traj:
            ps = PoseStamped()
            ps.header.stamp = path.header.stamp
            ps.header.frame_id = self.frame_id
            ps.pose.position.x = float(row[0])
            ps.pose.position.y = float(row[1])
            ps.pose.position.z = 0.0
            ps.pose.orientation = yaw_to_quaternion(float(row[2]))
            path.poses.append(ps)
        return path

    def publish_traj_markers(self, traj: np.ndarray) -> None:
        """Visualizes the best trajectory and obstacles using MarkerArray."""
        ma = MarkerArray()
        
        # 1. Best trajectory line
        line = Marker()
        line.header.frame_id = self.frame_id
        line.header.stamp = self.get_clock().now().to_msg()
        line.ns = "dwa_best_trajectory"
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.15
        line.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.9)
        for row in traj:
            p = Pose()
            p.position.x = float(row[0])
            p.position.y = float(row[1])
            line.points.append(p.position)
        ma.markers.append(line)

        # 2. Known obstacles (if any)
        if self.cfg.ob.size > 0:
            obm = Marker()
            obm.header.frame_id = self.frame_id
            obm.header.stamp = line.header.stamp
            obm.ns = "known_traps"
            obm.id = 1
            obm.type = Marker.CUBE_LIST
            obm.action = Marker.ADD
            obm.scale.x = 0.5
            obm.scale.y = 0.5
            obm.scale.z = 0.5
            obm.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)
            for ob_pt in self.cfg.ob:
                p = Pose()
                p.position.x = float(ob_pt[0])
                p.position.y = float(ob_pt[1])
                obm.points.append(p.position)
            ma.markers.append(obm)

        self.traj_viz_pub.publish(ma)

    def loop(self) -> None:
        """Main node loop."""
        if not self.has_pose:
            self.get_logger().debug("Waiting for robot pose...")
            return

        _, best_traj = self.dwa_control()
        self.path_pub.publish(self.trajectory_to_path(best_traj))
        self.publish_traj_markers(best_traj)


def main(args: List[str] = None) -> None:
    """Entry point."""
    rclpy.init(args=args)
    node = DWAPathPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
