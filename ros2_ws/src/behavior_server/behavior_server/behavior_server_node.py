#!/usr/bin/env python3

import math
from collections import deque
from enum import Enum
from typing import Optional, List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped, TwistStamped


class RobotState(Enum):
    STARTUP_MOVE = 0
    STARTUP_STOPPING = 1
    NORMAL = 2
    UNSTUCK = 3
    UNSTUCK_STOPPING = 4
    STOP_SPINNING = 5


class SafetyController(Node):
    def __init__(self) -> None:
        super().__init__('safety_controller')

        # --- Parameters ---
        # Startup behavior
        self.declare_parameter('enable_startup_move', True)
        self.declare_parameter('startup_move_duration', 2.0)
        self.declare_parameter('startup_move_speed', 1.0)
        self.declare_parameter('startup_stop_duration', 0.0)

        # Stuck detection
        self.declare_parameter('stuck_time_window', 5.0)
        self.declare_parameter('stuck_threshold', 0.1)
        self.declare_parameter('stuck_angular_threshold', 0.3)
        self.declare_parameter('unstuck_duration', 1.0)
        self.declare_parameter('unstuck_speed', -0.35)
        self.declare_parameter('unstuck_stop_duration', 0.5)

        # Spinning detection
        self.declare_parameter('spin_threshold', 0.3)
        self.declare_parameter('spin_time_limit', 2.0)
        self.declare_parameter('stop_duration', 1.5)

        # System
        self.declare_parameter('debug_output_rate', 1.0)
        self.declare_parameter('min_samples', 10)

        # Load parameters
        self._load_parameters()

        # --- Infrastructure ---
        self.timer_cb_group = MutuallyExclusiveCallbackGroup()
        self.subscriber_cb_group = ReentrantCallbackGroup()

        # State management
        self.state: RobotState = RobotState.STARTUP_MOVE if self.enable_startup_move else RobotState.NORMAL
        self.state_start_time: Optional[Time] = None
        
        # Data buffers
        self.pose_history: deque = deque()
        self.last_pid_cmd: Optional[TwistStamped] = None
        self.last_published_cmd: Optional[TwistStamped] = None
        
        # Detection variables
        self.spin_start_time: Optional[Time] = None
        self.stuck_check_ready: bool = False
        
        # Metrics for debug
        self.current_metrics = {
            "dist": 0.0,
            "dyaw": 0.0,
            "ang_z": 0.0,
            "lin_x": 0.0,
            "hist_dur": 0.0
        }

        # --- Interfaces ---
        self.pose_sub = self.create_subscription(
            PoseStamped, '/robot_pose', self.pose_callback, 10,
            callback_group=self.subscriber_cb_group
        )
        self.pid_cmd_sub = self.create_subscription(
            TwistStamped, '/pid_follower_cmd_vel', self.pid_cmd_callback, 10,
            callback_group=self.subscriber_cb_group
        )
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        # Timers
        self.control_timer = self.create_timer(
            0.1, self.control_loop, callback_group=self.timer_cb_group
        )
        self.stuck_check_timer = self.create_timer(
            1.0, self.check_stuck_timer, callback_group=self.timer_cb_group
        )

        if self.debug_output_rate > 0:
            self.debug_timer = self.create_timer(
                1.0 / self.debug_output_rate, self.print_debug_info,
                callback_group=self.timer_cb_group
            )

        self.get_logger().info(f'Safety Controller initialized. Startup Mode: {self.enable_startup_move}')

    def _load_parameters(self) -> None:
        """Helper to load all parameters cleanly."""
        self.enable_startup_move = self.get_parameter('enable_startup_move').value
        self.startup_move_duration = self.get_parameter('startup_move_duration').value
        self.startup_move_speed = self.get_parameter('startup_move_speed').value
        self.startup_stop_duration = self.get_parameter('startup_stop_duration').value
        
        self.stuck_time_window = self.get_parameter('stuck_time_window').value
        self.stuck_threshold = self.get_parameter('stuck_threshold').value
        self.stuck_angular_threshold = self.get_parameter('stuck_angular_threshold').value
        self.unstuck_duration = self.get_parameter('unstuck_duration').value
        self.unstuck_speed = self.get_parameter('unstuck_speed').value
        self.unstuck_stop_duration = self.get_parameter('unstuck_stop_duration').value
        
        self.spin_threshold = self.get_parameter('spin_threshold').value
        self.spin_time_limit = self.get_parameter('spin_time_limit').value
        self.stop_duration = self.get_parameter('stop_duration').value
        
        self.debug_output_rate = self.get_parameter('debug_output_rate').value
        self.min_samples = self.get_parameter('min_samples').value

    def pose_callback(self, msg: PoseStamped) -> None:
        """Maintains a time-windowed history of robot poses."""
        current_time = self.get_clock().now()
        self.pose_history.append((current_time, msg))
        
        # Clean up old poses
        while self.pose_history:
            age = (current_time - self.pose_history[0][0]).nanoseconds / 1e9
            if age > self.stuck_time_window:
                self.pose_history.popleft()
            else:
                break

    def pid_cmd_callback(self, msg: TwistStamped) -> None:
        self.last_pid_cmd = msg

    @staticmethod
    def quaternion_to_yaw(q) -> float:
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def check_stuck_timer(self) -> None:
        """Periodic check to see if the robot is physically stuck despite commands."""
        # Stuck detection is only valid in NORMAL mode
        if self.state != RobotState.NORMAL:
            return

        if len(self.pose_history) < self.min_samples:
            self.stuck_check_ready = False
            return

        current_time = self.get_clock().now()
        oldest_age = (current_time - self.pose_history[0][0]).nanoseconds / 1e9
        
        # Ensure the history window is fully populated before making decisions
        if oldest_age < (self.stuck_time_window * 0.95):
            self.stuck_check_ready = False
            self.current_metrics["hist_dur"] = oldest_age
            return
        
        if not self.stuck_check_ready:
            self.stuck_check_ready = True
            self.get_logger().info('History window filled. Stuck monitoring active.')

        # Analysis
        first_pose = self.pose_history[0][1]
        last_pose = self.pose_history[-1][1]

        # Calculate displacements
        dx = last_pose.pose.position.x - first_pose.pose.position.x
        dy = last_pose.pose.position.y - first_pose.pose.position.y
        dz = last_pose.pose.position.z - first_pose.pose.position.z
        distance = math.sqrt(dx*dx + dy*dy + dz*dz)

        first_yaw = self.quaternion_to_yaw(first_pose.pose.orientation)
        last_yaw = self.quaternion_to_yaw(last_pose.pose.orientation)
        dyaw = abs(last_yaw - first_yaw)
        if dyaw > math.pi:
            dyaw = 2 * math.pi - dyaw

        # Get current commanded angular velocity
        current_angular_vel = 0.0
        if self.last_published_cmd:
            current_angular_vel = abs(self.last_published_cmd.twist.angular.z)

        # Update metrics for debug
        self.current_metrics.update({
            "dist": distance,
            "dyaw": dyaw,
            "ang_z": current_angular_vel,
            "hist_dur": oldest_age
        })

        # Decision logic
        is_pos_stuck = distance < self.stuck_threshold
        is_rot_stuck = dyaw < self.stuck_angular_threshold # Using angular threshold for yaw delta logic from original code
        is_not_spinning = current_angular_vel < self.stuck_angular_threshold

        if is_pos_stuck and is_rot_stuck and is_not_spinning:
            self.get_logger().warn(
                f'STUCK DETECTED: Dist={distance:.3f}, YawDelta={dyaw:.3f}. Initiating recovery.'
            )
            self.state = RobotState.UNSTUCK
            self.state_start_time = self.get_clock().now()
            self.stuck_check_ready = False
            self.pose_history.clear()

    def check_spinning(self) -> bool:
        """Detects if the robot has been spinning in place for too long."""
        if self.last_published_cmd is None:
            return False

        current_az = self.last_published_cmd.twist.angular.z
        
        if abs(current_az) > self.spin_threshold:
            if self.spin_start_time is None:
                self.spin_start_time = self.get_clock().now()
            else:
                duration = (self.get_clock().now() - self.spin_start_time).nanoseconds / 1e9
                if duration > self.spin_time_limit:
                    return True
        else:
            self.spin_start_time = None
        
        return False

    def control_loop(self) -> None:
        """Main control loop. Delegates to specific state handlers."""
        current_time = self.get_clock().now()
        
        if self.state == RobotState.STARTUP_MOVE:
            self._handle_startup_move(current_time)
        elif self.state == RobotState.STARTUP_STOPPING:
            self._handle_startup_stopping(current_time)
        elif self.state == RobotState.UNSTUCK:
            self._handle_unstuck(current_time)
        elif self.state == RobotState.UNSTUCK_STOPPING:
            self._handle_unstuck_stopping(current_time)
        elif self.state == RobotState.STOP_SPINNING:
            self._handle_stop_spinning(current_time)
        elif self.state == RobotState.NORMAL:
            self._handle_normal_mode(current_time)

    # --- State Handlers ---

    def _handle_startup_move(self, current_time: Time) -> None:
        if self.state_start_time is None:
            self.state_start_time = current_time
            self.get_logger().info('State: STARTUP_MOVE')

        elapsed = (current_time - self.state_start_time).nanoseconds / 1e9
        
        if elapsed >= self.startup_move_duration:
            self.state = RobotState.STARTUP_STOPPING
            self.state_start_time = current_time
            self.publish_stop_command()
        else:
            self.publish_cmd(linear=self.startup_move_speed, angular=0.0)

    def _handle_startup_stopping(self, current_time: Time) -> None:
        elapsed = (current_time - self.state_start_time).nanoseconds / 1e9
        
        if elapsed >= self.startup_stop_duration:
            self.get_logger().info('Startup sequence complete. Switching to NORMAL.')
            self.state = RobotState.NORMAL
            self.state_start_time = None
            self.pose_history.clear() # Clear history to avoid false positives after calib
        else:
            self.publish_stop_command()

    def _handle_unstuck(self, current_time: Time) -> None:
        elapsed = (current_time - self.state_start_time).nanoseconds / 1e9
        
        if elapsed >= self.unstuck_duration:
            self.state = RobotState.UNSTUCK_STOPPING
            self.state_start_time = current_time
            self.publish_stop_command()
        else:
            # Move backwards
            self.publish_cmd(linear=self.unstuck_speed, angular=0.0)

    def _handle_unstuck_stopping(self, current_time: Time) -> None:
        elapsed = (current_time - self.state_start_time).nanoseconds / 1e9
        
        if elapsed >= self.unstuck_stop_duration:
            self.get_logger().info('Recovery complete. Switching to NORMAL.')
            self.state = RobotState.NORMAL
            self.state_start_time = None
        else:
            self.publish_stop_command()

    def _handle_stop_spinning(self, current_time: Time) -> None:
        elapsed = (current_time - self.state_start_time).nanoseconds / 1e9
        
        if elapsed >= self.stop_duration:
            self.get_logger().info('Spin timeout cooldown complete. Switching to NORMAL.')
            self.state = RobotState.NORMAL
            self.state_start_time = None
            self.spin_start_time = None
        else:
            self.publish_stop_command()

    def _handle_normal_mode(self, current_time: Time) -> None:
        if self.check_spinning():
            self.get_logger().warn('Excessive spinning detected. Forcing stop.')
            self.state = RobotState.STOP_SPINNING
            self.state_start_time = current_time
            self.publish_stop_command()
            return

        # Forward PID commands
        if self.last_pid_cmd is not None:
            self.publish_cmd_msg(self.last_pid_cmd)

    # --- Publishing Helpers ---

    def publish_cmd(self, linear: float, angular: float) -> None:
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x = float(linear)
        cmd.twist.angular.z = float(angular)
        self.cmd_vel_pub.publish(cmd)
        self.last_published_cmd = cmd

    def publish_cmd_msg(self, msg: TwistStamped) -> None:
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = msg.header.frame_id
        cmd.twist = msg.twist
        self.cmd_vel_pub.publish(cmd)
        self.last_published_cmd = cmd

    def publish_stop_command(self) -> None:
        self.publish_cmd(0.0, 0.0)

    # --- Debug Output ---

    def print_debug_info(self) -> None:
        """Prints a concise status line."""
        state_str = self.state.name
        
        # Calculate state duration
        duration_str = ""
        if self.state_start_time:
            elapsed = (self.get_clock().now() - self.state_start_time).nanoseconds / 1e9
            duration_str = f"| T={elapsed:.1f}s"

        # Current velocity info
        vel_str = "Vel: N/A"
        if self.last_published_cmd:
            v_lin = self.last_published_cmd.twist.linear.x
            v_ang = self.last_published_cmd.twist.angular.z
            vel_str = f"Vel: x={v_lin:.2f}, z={v_ang:.2f}"

        # Stuck monitor info (only relevant in NORMAL)
        stuck_str = ""
        if self.state == RobotState.NORMAL:
            ready = "R" if self.stuck_check_ready else "Init"
            stuck_str = (f"| StuckMon[{ready}]: dPos={self.current_metrics['dist']:.3f}, "
                         f"dYaw={self.current_metrics['dyaw']:.2f}")

        log_msg = f"[{state_str}] {duration_str} | {vel_str} {stuck_str}"
        self.get_logger().info(log_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyController()
    
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
