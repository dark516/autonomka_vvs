#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, Point, Quaternion
from std_msgs.msg import Header
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from typing import Optional, Tuple
import math

class KalmanFilter:
    def __init__(self, dt: float = 0.1):
        # State: [x, y, z, vx, vy, vz, qw, qx, qy, qz, wx, wy, wz] - 13 dimensions
        self.dim_x = 13
        self.dim_z = 7  # Measurement: [x, y, z, qw, qx, qy, qz]
        
        # State vector
        self.x = np.zeros(self.dim_x)
        self.x[6] = 1.0  # Initialize quaternion as identity
        
        # State transition matrix (constant velocity model)
        self.F = np.eye(self.dim_x)
        # Position-velocity relationships
        for i in range(3):
            self.F[i, i+3] = dt
        
        # Process noise covariance
        self.Q = np.eye(self.dim_x) * 0.01
        # Higher uncertainty for velocities and angular velocities
        self.Q[3:6, 3:6] *= 10.0
        self.Q[10:13, 10:13] *= 10.0
        
        # Measurement matrix (we measure position and orientation directly)
        self.H = np.zeros((self.dim_z, self.dim_x))
        for i in range(7):
            self.H[i, i] = 1.0
        
        # Measurement noise covariance
        self.R = np.eye(self.dim_z) * 0.1
        
        # Estimation error covariance
        self.P = np.eye(self.dim_x) * 1.0
        
    def predict(self, dt: float) -> None:
        """Predict state forward in time"""
        # Update state transition matrix with new dt
        for i in range(3):
            self.F[i, i+3] = dt
        
        # Handle quaternion propagation based on angular velocity
        if np.any(self.x[10:13] != 0):  # If we have angular velocity
            omega = self.x[10:13]
            omega_norm = np.linalg.norm(omega)
            if omega_norm > 1e-6:
                # Quaternion update from angular velocity
                q_prev = self.x[6:10]
                omega_quat = np.zeros(4)
                omega_quat[1:] = omega
                omega_quat = 0.5 * omega_quat
                
                # Approximate quaternion update
                q_dot = self.quaternion_multiply(omega_quat, q_prev)
                self.x[6:10] = q_prev + q_dot * dt
                self.x[6:10] /= np.linalg.norm(self.x[6:10])  # Normalize
        
        # Predict state
        self.x = self.F @ self.x
        
        # Predict covariance
        self.P = self.F @ self.P @ self.F.T + self.Q
        
    def update(self, z: np.ndarray) -> None:
        """Update state with measurement"""
        # Kalman gain
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        
        # Update state
        y = z - self.H @ self.x
        self.x = self.x + K @ y
        
        # Normalize quaternion
        self.x[6:10] /= np.linalg.norm(self.x[6:10])
        
        # Update covariance
        I = np.eye(self.dim_x)
        self.P = (I - K @ self.H) @ self.P
        
    def quaternion_multiply(self, q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
        """Multiply two quaternions"""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        
        return np.array([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2
        ])
    
    def get_pose(self) -> Tuple[np.ndarray, np.ndarray]:
        """Get position and quaternion from state"""
        return self.x[0:3], self.x[6:10]
    
    def get_velocity(self) -> Tuple[np.ndarray, np.ndarray]:
        """Get linear and angular velocity from state"""
        return self.x[3:6], self.x[10:13]
    
    def get_position_covariance(self) -> np.ndarray:
        """Get position covariance matrix"""
        return self.P[0:3, 0:3]

class ArucoTracker(Node):
    def __init__(self):
        super().__init__('aruco_tracker')
        
        # Parameters
        self.declare_parameter('prediction_timeout', 2.0)
        self.declare_parameter('max_prediction_time', 2.0)
        self.declare_parameter('process_noise', 0.01)
        self.declare_parameter('measurement_noise', 0.1)
        self.declare_parameter('world_frame', 'map')
        self.declare_parameter('kf_dt', 0.1)
        self.declare_parameter('covariance_threshold', 0.5)  # Threshold for prediction warning
        
        # Initialize Kalman Filter
        dt = self.get_parameter('kf_dt').value
        self.kf = KalmanFilter(dt)
        self.kf_initialized = False
        
        self.prediction_timeout = self.get_parameter('prediction_timeout').value
        self.max_prediction_time = self.get_parameter('max_prediction_time').value
        self.world_frame = self.get_parameter('world_frame').value
        self.covariance_threshold = self.get_parameter('covariance_threshold').value
        
        # State variables
        self.last_update_time: Optional[float] = None
        self.marker_lost_time: Optional[float] = None
        self.is_tracking = False
        self.consecutive_misses = 0
        self.max_consecutive_misses = 5
        self.last_pose_was_predicted = False
        
        # QoS configuration
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Publishers and Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/robot/pose/camera_1',
            self.pose_callback,
            qos_profile
        )
        
        # Single publisher for both real and predicted poses
        self.tracked_pose_pub = self.create_publisher(
            PoseStamped,
            '/robot/pose/tracked',
            qos_profile
        )
        
        self.velocity_pub = self.create_publisher(
            Twist,
            '/robot/velocity/estimated',
            qos_profile
        )
        
        # Timer for prediction updates (10Hz)
        self.timer = self.create_timer(dt, self.update_tracking)
        
        self.get_logger().info('ArUco Tracker with Kalman Filter started')
        
    def pose_to_numpy(self, pose: PoseStamped) -> np.ndarray:
        """Convert PoseStamped to numpy measurement vector"""
        p = pose.pose.position
        q = pose.pose.orientation
        return np.array([p.x, p.y, p.z, q.w, q.x, q.y, q.z])
    
    def numpy_to_pose(self, position: np.ndarray, quaternion: np.ndarray) -> PoseStamped:
        """Convert numpy arrays to PoseStamped"""
        pose = PoseStamped()
        pose.header = Header()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.world_frame
        
        pose.pose.position = Point(x=position[0], y=position[1], z=position[2])
        pose.pose.orientation = Quaternion(w=quaternion[0], x=quaternion[1], 
                                         y=quaternion[2], z=quaternion[3])
        return pose
    
    def numpy_to_twist(self, linear: np.ndarray, angular: np.ndarray) -> Twist:
        """Convert numpy arrays to Twist"""
        twist = Twist()
        twist.linear.x = linear[0]
        twist.linear.y = linear[1]
        twist.linear.z = linear[2]
        twist.angular.x = angular[0]
        twist.angular.y = angular[1]
        twist.angular.z = angular[2]
        return twist
    
    def pose_callback(self, msg: PoseStamped):
        """Callback for new ArUco marker detections"""
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # Convert to measurement vector
        z = self.pose_to_numpy(msg)
        
        if not self.kf_initialized:
            # Initialize Kalman filter with first measurement
            self.kf.x[0:3] = z[0:3]  # Position
            self.kf.x[6:10] = z[3:7]  # Orientation
            self.kf_initialized = True
            self.get_logger().info('Kalman filter initialized with first measurement')
        else:
            # Update Kalman filter
            self.kf.update(z)
            self.consecutive_misses = 0
        
        self.last_update_time = current_time
        
        # Log if we're switching from predicted to real tracking
        if not self.is_tracking:
            self.get_logger().info('ArUco marker detected - switching to real tracking')
        
        self.is_tracking = True
        self.marker_lost_time = None
        self.last_pose_was_predicted = False
        
        # Get pose from Kalman filter and publish
        position, orientation = self.kf.get_pose()
        pose_msg = self.numpy_to_pose(position, orientation)
        self.tracked_pose_pub.publish(pose_msg)
        
        # Publish velocity
        linear_vel, angular_vel = self.kf.get_velocity()
        vel_msg = self.numpy_to_twist(linear_vel, angular_vel)
        self.velocity_pub.publish(vel_msg)
    
    def update_tracking(self):
        """Main tracking update loop - runs at fixed interval"""
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        if not self.kf_initialized:
            return
        
        # Check if we've lost the marker
        if (self.is_tracking and 
            self.last_update_time is not None and
            current_time - self.last_update_time > self.prediction_timeout):
            
            self.is_tracking = False
            if self.marker_lost_time is None:
                self.marker_lost_time = current_time
                self.consecutive_misses += 1
                self.get_logger().warn(f'ArUco marker lost - using prediction (miss #{self.consecutive_misses})')
        
        # Always run prediction step (this propagates the state forward)
        dt = self.get_parameter('kf_dt').value
        self.kf.predict(dt)
        
        # If marker is lost, use predicted state
        if not self.is_tracking and self.last_pose is not None:
            time_since_lost = current_time - self.marker_lost_time
            
            if time_since_lost <= self.max_prediction_time:
                # Use Kalman filter prediction
                position, orientation = self.kf.get_pose()
                pose_msg = self.numpy_to_pose(position, orientation)
                self.tracked_pose_pub.publish(pose_msg)
                
                # Log only when we first switch to prediction
                if not self.last_pose_was_predicted:
                    self.last_pose_was_predicted = True
                    pos_cov = np.trace(self.kf.get_position_covariance())
                    self.get_logger().warn(f'Publishing predicted pose (covariance: {pos_cov:.3f})')
                
                # Publish predicted velocity
                linear_vel, angular_vel = self.kf.get_velocity()
                vel_msg = self.numpy_to_twist(linear_vel, angular_vel)
                self.velocity_pub.publish(vel_msg)
                
                # Log occasional updates if covariance is high
                pos_cov = np.trace(self.kf.get_position_covariance())
                if pos_cov > self.covariance_threshold and int(current_time * 10) % 10 == 0:
                    self.get_logger().warn(f'High prediction uncertainty: {pos_cov:.3f}')
                    
            else:
                if self.consecutive_misses < self.max_consecutive_misses:
                    if int(current_time) % 5 == 0:  # Log every 5 seconds
                        self.get_logger().warn('Max prediction time exceeded - still waiting for marker')
                else:
                    self.get_logger().error('Too many consecutive misses - consider reinitializing')
        else:
            # We're tracking with real measurements
            self.last_pose_was_predicted = False
    
    def destroy_node(self):
        """Cleanup before destruction"""
        self.get_logger().info('Shutting down ArUco Tracker')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    tracker = ArucoTracker()
    
    try:
        rclpy.spin(tracker)
    except KeyboardInterrupt:
        pass
    finally:
        tracker.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()