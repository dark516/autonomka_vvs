import rclpy
from rclpy.node import Node
import math
import numpy as np
from typing import Optional, List, Tuple

from geometry_msgs.msg import PoseStamped, TwistStamped, TransformStamped, Quaternion
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from tf2_ros import TransformBroadcaster


class FollowerConfig:
    """
    Configuration parameters for the path following algorithm.
    """
    def __init__(self):
        self.dt: float = 0.4

        # Angular PID controller parameters
        self.kp_angular: float = 0.3
        self.ki_angular: float = 0.05
        self.kd_angular: float = 0.2
        self.max_yaw_rate: float = 50.0 * math.pi / 180.0  # Maximum angular velocity [rad/s]

        self.base_linear_speed: float = 0.4  # Base linear speed [m/s]

        # Lookahead and stopping
        self.lookahead_distance: float = 0.6      # Lookahead distance for the target point [m]
        self.goal_tolerance: float = 0.00001         # Tolerance for stopping at the final goal [m]
        self.yaw_tolerance: float = math.radians(8)  # Yaw tolerance [rad] (not explicitly used for stopping)


def euler_from_quaternion(q: Quaternion) -> float:
    """
    Converts a geometry_msgs.msg.Quaternion message to a Euler yaw angle.
    Returns the yaw angle [rad].
    """
    x, y, z, w = q.x, q.y, q.z, q.w
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw


def normalize_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class PIDController:
    """
    A simple Proportional-Integral-Derivative (PID) controller.
    """
    def __init__(self, kp: float, ki: float, kd: float, max_output: float):
        self.kp: float = kp
        self.ki: float = ki
        self.kd: float = kd
        self.max_output: float = max_output
        self.prev_error: float = 0.0
        self.integral: float = 0.0
        self.integral_limit: float = 1.0

    def compute(self, error: float, dt: float) -> float:
        """
        Computes the controller output value.
        """
        self.integral += error * dt
        # Integral limit (anti-windup)
        self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))
        
        deriv: float = (error - self.prev_error) / dt if dt > 0.0 else 0.0
        
        out: float = self.kp * error + self.ki * self.integral + self.kd * deriv
        
        # Output limiting
        out = max(-self.max_output, min(self.max_output, out))
        
        self.prev_error = error
        return out

    def reset(self):
        """
        Resets the controller state (integral and previous error).
        """
        self.prev_error = 0.0
        self.integral = 0.0


# --- Node ---------------------------------------------------------------------

class PathFollowerPIDNode(Node):
    """
    ROS 2 node for path following using a PID controller for angular velocity
    and a lookahead strategy (similar to Pure Pursuit) for target selection.
    """
    def __init__(self):
        super().__init__('path_follower_pid_node')
        
        # Declare parameters with default values
        self.declare_parameters(
            namespace='',
            parameters=[
                ('kp_angular', 0.3),
                ('ki_angular', 0.05),
                ('kd_angular', 0.2),
                ('max_yaw_rate', 50.0),  # degrees/s
                ('base_linear_speed', 0.3),
                ('lookahead_distance', 0.6),
                ('goal_tolerance', 0.00001),
                ('yaw_tolerance', 8.0),  # degrees
                ('dt', 0.4)
            ]
        )
        
        # Get parameter values
        kp_angular: float = self.get_parameter('kp_angular').value
        ki_angular: float = self.get_parameter('ki_angular').value
        kd_angular: float = self.get_parameter('kd_angular').value
        max_yaw_rate_deg: float = self.get_parameter('max_yaw_rate').value
        base_linear_speed: float = self.get_parameter('base_linear_speed').value
        lookahead_distance: float = self.get_parameter('lookahead_distance').value
        goal_tolerance: float = self.get_parameter('goal_tolerance').value
        yaw_tolerance_deg: float = self.get_parameter('yaw_tolerance').value
        dt: float = self.get_parameter('dt').value
        
        self.get_logger().info("Path Follower (PID) node started.")
        self.get_logger().info(f"PID parameters: kp={kp_angular}, ki={ki_angular}, kd={kd_angular}")
        self.get_logger().info(f"Max yaw rate: {max_yaw_rate_deg} deg/s")
        self.get_logger().info(f"Lookahead distance: {lookahead_distance}m")

        self.cfg: FollowerConfig = FollowerConfig()
        
        # Override configuration with parameter values
        self.cfg.kp_angular = kp_angular
        self.cfg.ki_angular = ki_angular
        self.cfg.kd_angular = kd_angular
        self.cfg.max_yaw_rate = max_yaw_rate_deg * math.pi / 180.0  # Convert to rad/s
        self.cfg.base_linear_speed = base_linear_speed
        self.cfg.lookahead_distance = lookahead_distance
        self.cfg.goal_tolerance = goal_tolerance
        self.cfg.yaw_tolerance = yaw_tolerance_deg * math.pi / 180.0  # Convert to radians
        self.cfg.dt = dt

        # Robot state: [x, y, yaw]
        self.state: np.ndarray = np.array([0.0, 0.0, 0.0], dtype=np.float64)
        self.frame_id: str = 'map'

        # Last received path
        self.path_msg: Optional[Path] = None

        # Controller instance
        self.pid: PIDController = PIDController(self.cfg.kp_angular, self.cfg.ki_angular,
                                                self.cfg.kd_angular, self.cfg.max_yaw_rate)

        # TF broadcaster to publish the robot's base_link transform
        self.tf_broadcaster: TransformBroadcaster = TransformBroadcaster(self)

        # Subscriptions
        self.pose_sub = self.create_subscription(
            PoseStamped, '/robot_pose', self.pose_cb, 10)
        self.path_sub = self.create_subscription(
            Path, '/planned_path', self.path_cb, 10)

        # Publishers
        self.cmd_pub = self.create_publisher(TwistStamped, '/pid_follower_cmd_vel', 10)
        self.lookahead_pub = self.create_publisher(Marker, '/lookahead_marker', 10)

        # Timer for the main control loop
        self.timer = self.create_timer(self.cfg.dt, self.loop)

        # Logs and flags
        self._log_counter: int = 0
        self._goal_reached: bool = False

    # Callbacks ---------------------------------------------------------------

    def pose_cb(self, msg: PoseStamped):
        p = msg.pose.position
        q = msg.pose.orientation

        self.frame_id = msg.header.frame_id or 'map'
        self.state[0] = p.x
        self.state[1] = p.y
        self.state[2] = euler_from_quaternion(q)

        # Broadcast TF transform (<frame_id> -> base_link)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.frame_id
        t.child_frame_id = 'base_link'
        t.transform.translation.x = p.x
        t.transform.translation.y = p.y
        t.transform.translation.z = 0.0
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)

    def path_cb(self, msg: Path):
        self.path_msg = msg
        self._goal_reached = False
        self.pid.reset()

    # Utilities ---------------------------------------------------------------

    def pick_lookahead(self) -> Tuple[Optional[PoseStamped], Optional[int]]:
        if self.path_msg is None or len(self.path_msg.poses) == 0:
            return None, None

        poses: List[PoseStamped] = self.path_msg.poses
        rx, ry = float(self.state[0]), float(self.state[1])

        # 1. Find the closest point index
        dists: List[float] = [math.hypot(ps.pose.position.x - rx, ps.pose.position.y - ry) for ps in poses]
        nearest_idx: int = int(np.argmin(dists))

        # 2. Search forward for the first point beyond the lookahead distance
        for i in range(nearest_idx, len(poses)):
            d: float = math.hypot(poses[i].pose.position.x - rx, poses[i].pose.position.y - ry)
            if d >= self.cfg.lookahead_distance:
                return poses[i], i

        # 3. If no lookahead point is found (e.g., near the end of the path), use the last pose
        return poses[-1], len(poses) - 1

    def publish_lookahead_marker(self, target_ps: PoseStamped):
        """
        Publishes a marker to visualize the selected lookahead point.
        """
        m = Marker()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = target_ps.header.frame_id or self.frame_id
        m.ns = "follower_lookahead"
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose = target_ps.pose
        m.scale.x = m.scale.y = m.scale.z = 0.25
        m.color = ColorRGBA(r=0.0, g=0.5, b=1.0, a=0.9)
        self.lookahead_pub.publish(m)

    # Control loop ------------------------------------------------------------

    def loop(self):
        """
        The main control loop, triggered by a timer.
        """
        # 1. No path or path too short → Stop
        if self.path_msg is None or len(self.path_msg.poses) < 2:
            self._publish_cmd(0.0, 0.0)
            return

        # 2. Goal check (distance to the final waypoint)
        last_pose_stamped: PoseStamped = self.path_msg.poses[-1]
        dist_to_goal: float = math.hypot(last_pose_stamped.pose.position.x - self.state[0],
                                         last_pose_stamped.pose.position.y - self.state[1])
        
        if dist_to_goal < self.cfg.goal_tolerance:
            # Goal reached → full stop
            self._publish_cmd(0.0, 0.0)
            if not self._goal_reached:
                self.get_logger().info("Goal reached: stopping.")
                self._goal_reached = True
            return

        # 3. Choose lookahead target
        target_ps, idx = self.pick_lookahead()
        if target_ps is None:
            self._publish_cmd(0.0, 0.0)
            return

        self.publish_lookahead_marker(target_ps)

        # 4. Calculate heading error to the lookahead target
        dx: float = target_ps.pose.position.x - self.state[0]
        dy: float = target_ps.pose.position.y - self.state[1]
        
        desired_yaw: float = math.atan2(dy, dx)
        angle_error: float = normalize_angle(desired_yaw - self.state[2])

        # 5. PID → angular velocity (omega)
        omega_cmd: float = self.pid.compute(angle_error, self.cfg.dt)

        # 6. Linear speed (v_cmd): Base speed with slowdown factors
        dist_to_target: float = math.hypot(dx, dy)
        
        # Slowdown based on angle error [0.25..1]
        slow_by_angle: float = max(0.25, 1.0 - min(abs(angle_error), math.pi) / math.pi)  
        
        # Slowdown based on distance to lookahead/end [0.5..1]
        slow_by_dist: float = min(1.0, dist_to_target / (2.0 * self.cfg.lookahead_distance))
        
        v_cmd: float = self.cfg.base_linear_speed * slow_by_angle * max(0.5, slow_by_dist)

        # 7. Publish command
        self._publish_cmd(v_cmd, omega_cmd)

        # 8. Optional compact logging
        self._log_counter += 1
        if self._log_counter % 10 == 0:
            self.get_logger().info(
                f"Follow: target_idx={idx} dist={dist_to_target:.2f}m "
                f"yaw_err={math.degrees(angle_error):.1f}deg v={v_cmd:.2f} m/s ω={omega_cmd:.2f} rad/s"
            )

    def _publish_cmd(self, v: float, omega: float):
        ts = TwistStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = self.frame_id
        ts.twist.linear.x = float(v)
        ts.twist.angular.z = float(omega)
        self.cmd_pub.publish(ts)


def main(args=None):
    rclpy.init(args=args)
    node = PathFollowerPIDNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
