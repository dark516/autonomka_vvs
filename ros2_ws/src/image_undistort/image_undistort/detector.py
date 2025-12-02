import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import Header
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from builtin_interfaces.msg import Time
import math

class RobotDetectionFusion(Node):
    def __init__(self):
        super().__init__('robot_detection_fusion')
        
        # QoS profiles
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribers
        self.aruco_sub = self.create_subscription(
            PoseStamped, '/robot/pose/camera_1', self.aruco_callback, reliable_qos
        )
        self.yolo_robot_sub = self.create_subscription(
            PointStamped, '/morph/point_1/camera_1', self.yolo_robot_callback, best_effort_qos
        )
        self.yolo_enemy_sub = self.create_subscription(
            PointStamped, '/enemy/point/camera_1', self.yolo_enemy_callback, best_effort_qos
        )
        
        # Publishers
        self.robot_pose_pub = self.create_publisher(PoseStamped, '/robot_pose', reliable_qos)
        self.enemy_point_pub = self.create_publisher(PointStamped, '/enemy_point', reliable_qos)
        
        # State variables
        self.last_aruco_pose = None
        self.last_yolo_robot_point = None
        self.last_yolo_enemy_point = None
        
        # Last published positions for fallback
        self.last_published_robot_pose = None
        self.last_published_enemy_point = None
        
        # Heading storage
        self.last_known_heading = 0.0  # Default heading (facing positive x)
        
        # Configuration
        self.max_fallback_age = 5.0  # seconds - how long to use last position as fallback
        self.position_epsilon = 0.00  # minimum position change to consider it new data
        
        # Timer for periodic publishing
        self.create_timer(0.016, self.publish_fused_data)  # 60Hz
        
        self.get_logger().info("Robot Detection Fusion Node Started - Simple heading persistence")

    def aruco_callback(self, msg):
        """Store latest ArUco marker pose and extract heading"""
        self.last_aruco_pose = msg
        # Update heading from ArUco (most reliable source)
        self._update_heading_from_pose(msg)
        self.get_logger().debug(f"ArUco pose received: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})")

    def yolo_robot_callback(self, msg):
        """Store latest YOLO robot detection"""
        self.last_yolo_robot_point = msg
        self.get_logger().debug(f"YOLO robot point received: ({msg.point.x:.2f}, {msg.point.y:.2f})")

    def yolo_enemy_callback(self, msg):
        """Store latest YOLO enemy detection"""
        self.last_yolo_enemy_point = msg
        self.get_logger().debug(f"YOLO enemy point received: ({msg.point.x:.2f}, {msg.point.y:.2f})")

    def _update_heading_from_pose(self, pose):
        """Extract and store heading from a PoseStamped message"""
        # Convert quaternion to yaw
        x = pose.pose.orientation.x
        y = pose.pose.orientation.y
        z = pose.pose.orientation.z
        w = pose.pose.orientation.w
        
        # Convert quaternion to Euler angles (yaw)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.last_known_heading = yaw

    def _time_to_float(self, time_msg):
        """Convert builtin_interfaces.msg.Time to float seconds"""
        return time_msg.sec + time_msg.nanosec / 1e9

    def _get_current_time_float(self):
        """Get current time as float seconds"""
        now = self.get_clock().now()
        return now.nanoseconds / 1e9

    def _get_message_time_float(self, msg):
        """Get message timestamp as float seconds"""
        if msg and hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
            return self._time_to_float(msg.header.stamp)
        return 0.0

    def _point_to_pose(self, point_stamped):
        """Convert PointStamped to PoseStamped using last known heading"""
        pose = PoseStamped()
        pose.header = point_stamped.header
        
        # Copy position
        pose.pose.position.x = point_stamped.point.x
        pose.pose.position.y = point_stamped.point.y
        pose.pose.position.z = point_stamped.point.z
        
        # Use last known heading (no trajectory estimation)
        pose.pose.orientation = self._yaw_to_quaternion(self.last_known_heading)
        
        return pose

    def _yaw_to_quaternion(self, yaw):
        """Convert yaw angle to quaternion"""
        from geometry_msgs.msg import Quaternion
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    def _get_newest_robot_pose(self):
        """
        Get the newest robot pose from available sources.
        Priority: Use whichever message is newest (ArUco or YOLO)
        """
        current_time = self._get_current_time_float()
        
        # Collect all valid poses with their timestamps
        valid_poses = []
        
        if self.last_aruco_pose is not None:
            aruco_time = self._get_message_time_float(self.last_aruco_pose)
            valid_poses.append((self.last_aruco_pose, aruco_time, "ArUco"))
            
        if self.last_yolo_robot_point is not None:
            yolo_time = self._get_message_time_float(self.last_yolo_robot_point)
            # Convert PointStamped to PoseStamped with last known heading
            yolo_pose = self._point_to_pose(self.last_yolo_robot_point)
            valid_poses.append((yolo_pose, yolo_time, "YOLO"))
        
        if not valid_poses:
            # No fresh data available, try fallback to last published position
            if self.last_published_robot_pose:
                last_pub_time = self._get_message_time_float(self.last_published_robot_pose)
                fallback_age = current_time - last_pub_time
                
                if fallback_age < self.max_fallback_age:
                    self.get_logger().warn(f"Using last known robot position (age: {fallback_age:.1f}s)")
                    
                    # Create updated pose with current timestamp
                    fallback_pose = PoseStamped()
                    fallback_pose.header.stamp = self._float_to_time(current_time)
                    fallback_pose.header.frame_id = self.last_published_robot_pose.header.frame_id
                    fallback_pose.pose = self.last_published_robot_pose.pose
                    return fallback_pose
                else:
                    self.get_logger().error(f"Robot position too old for fallback (age: {fallback_age:.1f}s)")
            return None
        
        # Find the newest pose
        newest_pose, newest_time, source = max(valid_poses, key=lambda x: x[1])
        
        # Check if the data is reasonably fresh (optional sanity check)
        pose_age = current_time - newest_time
        if pose_age > 10.0:  # Sanity check - reject very old data
            self.get_logger().warn(f"Rejecting very old {source} data (age: {pose_age:.1f}s)")
            return None
            
        self.get_logger().debug(f"Using {source} for robot pose (age: {pose_age:.3f}s)")
        return newest_pose

    def _get_newest_enemy_point(self):
        """
        Get the newest enemy point from available sources.
        Currently only YOLO provides enemy detection.
        """
        current_time = self._get_current_time_float()
        
        valid_points = []
        
        if self.last_yolo_enemy_point is not None:
            yolo_time = self._get_message_time_float(self.last_yolo_enemy_point)
            valid_points.append((self.last_yolo_enemy_point, yolo_time, "YOLO"))
        
        if not valid_points:
            # No fresh data available, try fallback to last published position
            if self.last_published_enemy_point:
                last_pub_time = self._get_message_time_float(self.last_published_enemy_point)
                fallback_age = current_time - last_pub_time
                
                if fallback_age < self.max_fallback_age:
                    self.get_logger().warn(f"Using last known enemy position (age: {fallback_age:.1f}s)")
                    
                    # Create updated point with current timestamp
                    fallback_point = PointStamped()
                    fallback_point.header.stamp = self._float_to_time(current_time)
                    fallback_point.header.frame_id = self.last_published_enemy_point.header.frame_id
                    fallback_point.point = self.last_published_enemy_point.point
                    return fallback_point
                else:
                    self.get_logger().error(f"Enemy position too old for fallback (age: {fallback_age:.1f}s)")
            return None
        
        # Find the newest point (only YOLO for now)
        newest_point, newest_time, source = max(valid_points, key=lambda x: x[1])
        
        # Check if the data is reasonably fresh
        point_age = current_time - newest_time
        if point_age > 10.0:  # Sanity check
            self.get_logger().warn(f"Rejecting very old {source} data (age: {point_age:.1f}s)")
            return None
            
        self.get_logger().debug(f"Using {source} for enemy point (age: {point_age:.3f}s)")
        return newest_point

    def _float_to_time(self, time_float):
        """Convert float seconds to builtin_interfaces.msg.Time"""
        time_msg = Time()
        time_msg.sec = int(time_float)
        time_msg.nanosec = int((time_float - time_msg.sec) * 1e9)
        return time_msg

    def _has_position_changed(self, new_pose, old_pose, threshold=None):
        """Check if position has changed significantly"""
        if threshold is None:
            threshold = self.position_epsilon
            
        if old_pose is None or new_pose is None:
            return True
            
        # Handle both PoseStamped and PointStamped
        if hasattr(new_pose, 'pose') and hasattr(old_pose, 'pose'):
            old_pos = np.array([old_pose.pose.position.x, old_pose.pose.position.y])
            new_pos = np.array([new_pose.pose.position.x, new_pose.pose.position.y])
        else:
            old_pos = np.array([old_pose.point.x, old_pose.point.y])
            new_pos = np.array([new_pose.point.x, new_pose.point.y])
        
        distance = np.linalg.norm(new_pos - old_pos)
        return distance > threshold

    def publish_fused_data(self):
        """Publish fused robot pose and enemy point"""
        current_time_float = self._get_current_time_float()
        current_time_msg = self._float_to_time(current_time_float)
        
        # Get robot pose (newest available)
        robot_pose = self._get_newest_robot_pose()
        if robot_pose is not None:
            # Update timestamp to current time
            robot_pose.header.stamp = current_time_msg
            robot_pose.header.frame_id = "map"
            
            # Only publish if position changed significantly or it's the first publication
            if (self._has_position_changed(robot_pose, self.last_published_robot_pose) or 
                self.last_published_robot_pose is None):
                
                self.robot_pose_pub.publish(robot_pose)
                self.last_published_robot_pose = robot_pose
        
        # Get enemy point (newest available)
        enemy_point = self._get_newest_enemy_point()
        if enemy_point is not None:
            # Update timestamp to current time
            enemy_point.header.stamp = current_time_msg
            enemy_point.header.frame_id = "map"
            
            # Only publish if position changed significantly or it's the first publication
            if (self._has_position_changed(enemy_point, self.last_published_enemy_point) or 
                self.last_published_enemy_point is None):
                
                self.enemy_point_pub.publish(enemy_point)
                self.last_published_enemy_point = enemy_point

def main(args=None):
    rclpy.init(args=args)
    node = RobotDetectionFusion()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
