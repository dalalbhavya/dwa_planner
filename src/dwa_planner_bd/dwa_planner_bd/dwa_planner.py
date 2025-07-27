import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import math
from transforms3d.euler import quat2euler
import tf2_ros
from tf2_ros import TransformException

class DWAPlanner(Node):
    def __init__(self):
        super().__init__('dwa_planner')
        self.get_logger().info("DWA Planner Node Initializing...")

        # TF2 Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Parameters
        self.max_speed = 0.22
        self.max_yaw_rate = 2.84
        self.max_accel = 1.0
        self.max_delta_yaw_rate = 3.2
        self.v_resolution = 0.01
        self.yaw_rate_resolution = 0.1
        self.dt = 0.1
        self.predict_time = 3.0
        self.to_goal_cost_gain = 1.0
        self.speed_cost_gain = 1.0
        self.obstacle_cost_gain = 1.5
        self.robot_radius = 0.2

        # State Variables
        self.current_pose = None
        self.current_velocity = Twist()
        self.goal_pose = None
        self.laser_scan = None
        
        # ROS2 Communications
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, qos_profile)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.traj_marker_pub = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)
        self.timer = self.create_timer(0.1, self.plan_and_execute)
        
        self.get_logger().info("DWA Planner Node Initialized Successfully.")

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.current_velocity = msg.twist.twist

    def scan_callback(self, msg):
        self.laser_scan = msg

    def goal_callback(self, msg):
        self.goal_pose = msg.pose
        self.get_logger().info(f"New goal received: x={self.goal_pose.position.x}, y={self.goal_pose.position.y}")

    def plan_and_execute(self):
        if self.current_pose is None or self.goal_pose is None or self.laser_scan is None:
            return

        try:
            self.tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time(seconds=0), timeout=Duration(seconds=0.1))
        except TransformException as ex:
            self.get_logger().warn(f'Could not transform odom to base_footprint: {ex}')
            return

        dist_to_goal = math.hypot(self.current_pose.position.x - self.goal_pose.position.x, self.current_pose.position.y - self.goal_pose.position.y)
        if dist_to_goal < 0.2:
            self.get_logger().info("Goal reached!")
            self.stop_robot()
            self.goal_pose = None
            return
            
        best_vel = self.dwa_control()
        
        if best_vel is None:
            self.get_logger().error("No valid trajectory found! Stopping robot.")
            self.stop_robot()
        else:
            cmd_vel = Twist()
            cmd_vel.linear.x = best_vel[0]
            cmd_vel.angular.z = best_vel[1]
            self.cmd_vel_pub.publish(cmd_vel)

    def stop_robot(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)

    def dwa_control(self):
        dw = self.calculate_dynamic_window()
        trajectories = []
        for v in np.arange(dw[0], dw[1], self.v_resolution):
            for w in np.arange(dw[2], dw[3], self.yaw_rate_resolution):
                trajectory = self.predict_trajectory(v, w)
                trajectories.append((v, w, trajectory))

        if not trajectories:
            return None

        best_traj = None
        min_cost = float('inf')

        for v, w, trajectory in trajectories:
            heading_cost = self.to_goal_cost_gain * self.calculate_heading_cost(trajectory)
            dist_cost = self.obstacle_cost_gain * self.calculate_dist_cost(trajectory)
            speed_cost = self.speed_cost_gain * (self.max_speed - v)
            total_cost = heading_cost + dist_cost + speed_cost
            
            if min_cost > total_cost:
                min_cost = total_cost
                best_traj = (v, w, trajectory)

        self.visualize_trajectories(trajectories, best_traj)
        return best_traj[0], best_traj[1] if best_traj else None

    def calculate_dynamic_window(self):
        v = self.current_velocity.linear.x
        w = self.current_velocity.angular.z
        vs = [v - self.max_accel * self.dt, v + self.max_accel * self.dt, w - self.max_delta_yaw_rate * self.dt, w + self.max_delta_yaw_rate * self.dt]
        vd = [0.0, self.max_speed, -self.max_yaw_rate, self.max_yaw_rate]
        return [max(vs[0], vd[0]), min(vs[1], vd[1]), max(vs[2], vd[2]), min(vs[3], vd[3])]

    def predict_trajectory(self, v, w):
        trajectory = []
        x, y, theta = 0.0, 0.0, 0.0
        time = 0.0
        while time <= self.predict_time:
            x += v * math.cos(theta) * self.dt
            y += v * math.sin(theta) * self.dt
            theta += w * self.dt
            trajectory.append((x, y, theta))
            time += self.dt
        return trajectory

    def calculate_heading_cost(self, trajectory):
        final_x, final_y, final_theta = trajectory[-1]
        dx = self.goal_pose.position.x - self.current_pose.position.x
        dy = self.goal_pose.position.y - self.current_pose.position.y
        _, _, current_yaw = quat2euler([self.current_pose.orientation.w, self.current_pose.orientation.x, self.current_pose.orientation.y, self.current_pose.orientation.z])
        angle_to_goal_global = math.atan2(dy, dx)
        angle_to_goal_local = angle_to_goal_global - current_yaw
        cost = abs(math.atan2(math.sin(angle_to_goal_local - final_theta), math.cos(angle_to_goal_local - final_theta)))
        return cost

    def calculate_dist_cost(self, trajectory):
        """
        Calculates a more robust cost related to obstacle clearance.
        This version filters invalid laser scan data and is more efficient.
        """
        min_dist_to_obstacle = float('inf')
        
        # Pre-calculate obstacle points from the laser scan to avoid redundant calculations.
        obstacle_points = []
        for i, scan_dist in enumerate(self.laser_scan.ranges):
            # Filter out invalid range values (inf, nan) which can cause errors.
            if np.isinf(scan_dist) or np.isnan(scan_dist):
                continue
            
            scan_angle = self.laser_scan.angle_min + i * self.laser_scan.angle_increment
            ox = scan_dist * math.cos(scan_angle)
            oy = scan_dist * math.sin(scan_angle)
            obstacle_points.append((ox, oy))

        # If there are no valid obstacles, the cost is zero.
        if not obstacle_points:
            return 0.0

        # Check each point in the trajectory against the list of obstacles.
        for traj_x, traj_y, _ in trajectory:
            for obs_x, obs_y in obstacle_points:
                dist = math.hypot(traj_x - obs_x, traj_y - obs_y)
                if dist < min_dist_to_obstacle:
                    min_dist_to_obstacle = dist

        # If the closest the trajectory ever gets to an obstacle is less than the robot's radius,
        # it's a collision, so the cost is infinite.
        if min_dist_to_obstacle <= self.robot_radius:
            return float('inf')
            
        # The cost is inversely proportional to the distance to the closest obstacle.
        return 1.0 / min_dist_to_obstacle

    def visualize_trajectories(self, trajectories, best_traj):
        marker_array = MarkerArray()
        delete_marker = Marker()
        delete_marker.header.frame_id = "base_footprint"
        delete_marker.header.stamp = self.get_clock().now().to_msg()
        delete_marker.ns = "dwa_trajectories"
        delete_marker.id = -1
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        
        for i, (v, w, trajectory) in enumerate(trajectories):
            marker = Marker()
            marker.header.frame_id = "base_footprint"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "dwa_trajectories"
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.03
            marker.color.a = 1.0
            
            if best_traj and v == best_traj[0] and w == best_traj[1]:
                marker.color.r, marker.color.g, marker.color.b = 0.0, 0.0, 1.0
                marker.scale.x = 0.05
            else:
                is_collision = self.calculate_dist_cost(trajectory) == float('inf')
                marker.color.r, marker.color.g, marker.color.b = (1.0, 0.0, 0.0) if is_collision else (0.0, 1.0, 0.0)

            marker.points = [Point(x=p[0], y=p[1], z=0.0) for p in trajectory]
            marker_array.markers.append(marker)

        self.traj_marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    dwa_planner = DWAPlanner()
    try:
        rclpy.spin(dwa_planner)
    except KeyboardInterrupt:
        dwa_planner.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        dwa_planner.stop_robot()
        dwa_planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
