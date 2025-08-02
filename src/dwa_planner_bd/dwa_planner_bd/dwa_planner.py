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
import matplotlib.pyplot as plt
import matplotlib

matplotlib.use('TkAgg')

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
        self.robot_radius = 0.2
        
        # Normal Cost Function Gains
        self.to_goal_cost_gain = 0.8
        self.path_cost_gain = 0.8
        self.obstacle_cost_gain = 1
        self.speed_cost_gain = 6

        # Stuck/Recovery Cost Function Gains
        self.recovery_to_goal_cost_gain = 0.2  # Reduced to allow turning
        self.recovery_path_cost_gain = 0.4
        self.recovery_obstacle_cost_gain = 5.0 # increased to force obstacle avoidance
        self.recovery_speed_cost_gain = 1.0

        # Stuck Detection Parameters
        self.stuck_counter = 0
        self.stuck_threshold = 30  # 3 seconds at 10Hz
        self.stuck_velocity_threshold = 0.01
        self.is_stuck = False

        # State Variables
        self.current_pose = None
        self.current_velocity = Twist()
        self.goal_pose = None
        self.laser_scan = None
        
        # ROS2 Communications
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, qos_profile)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.traj_marker_pub = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)
        self.timer = self.create_timer(0.1, self.plan_and_execute)

        self.debug_plot_enabled = False
        self.trajectories_array = []
        self.best_trajectory_array = []
        self.pose_array = []
        self.initial_pose = None

        self.get_logger().info("DWA Planner Node Initialized Successfully.")

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.current_velocity = msg.twist.twist

    def scan_callback(self, msg):
        self.laser_scan = msg

    def goal_callback(self, msg):
        self.goal_pose = msg.pose
        self.get_logger().info(f"New goal received: x={self.goal_pose.position.x}, y={self.goal_pose.position.y}")
        # Reset stuck status on new goal
        self.is_stuck = False
        self.stuck_counter = 0

    def check_if_stuck(self):
        """Checks if the robot is stuck and updates the recovery state."""
        if self.current_velocity.linear.x < self.stuck_velocity_threshold and self.current_velocity.angular.z < self.stuck_velocity_threshold:
            self.stuck_counter += 1
        else:
            # If we are moving, reset the counter and exit recovery mode
            self.stuck_counter = 0
            if self.is_stuck:
                self.get_logger().info("Robot is moving again, exiting recovery mode.")
                self.is_stuck = False

        if self.stuck_counter > self.stuck_threshold:
            if not self.is_stuck:
                self.get_logger().warn("Robot is stuck! Entering recovery mode.")
                self.is_stuck = True
    
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
            self.debug_plot(trajectories=None, best_traj=None, plot_now=True)
            self.get_logger().info("Goal reached!")
            self.stop_robot()
            self.goal_pose = None
            return
        
        # Check for stuck condition before planning
        self.check_if_stuck()
            
        best_vel = self.dwa_control()
        
        if best_vel is None:
            self.get_logger().error("No valid trajectory found! Forcing rotation to find a path.")
            # If no path is found, force a rotation to hopefully find a clear path
            cmd_vel = Twist()
            cmd_vel.angular.z = 0.5
            self.cmd_vel_pub.publish(cmd_vel)
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

        # Select gains based on whether we are in recovery mode
        if self.is_stuck:
            current_goal_gain = self.recovery_to_goal_cost_gain
            current_path_gain = self.recovery_path_cost_gain
            current_obs_gain = self.recovery_obstacle_cost_gain
            current_speed_gain = self.recovery_speed_cost_gain
        else:
            current_goal_gain = self.to_goal_cost_gain
            current_path_gain = self.path_cost_gain
            current_obs_gain = self.obstacle_cost_gain
            current_speed_gain = self.speed_cost_gain

        for v, w, trajectory in trajectories:
            heading_cost = current_goal_gain * self.calculate_heading_cost(trajectory)
            path_cost = current_path_gain * self.calculate_path_cost(trajectory)
            dist_cost = current_obs_gain * self.calculate_dist_cost(trajectory)
            speed_cost = current_speed_gain * (self.max_speed - v)
            
            total_cost = heading_cost + path_cost + dist_cost + speed_cost
            
            if min_cost > total_cost:
                min_cost = total_cost
                best_traj = (v, w, trajectory)
                self.get_logger().info(f"New best trajectory v:{best_traj[0]}, w:{best_traj[1]}")
                self.get_logger().info(f"Costs - Heading: {heading_cost}, Path: {path_cost}, Dist: {dist_cost}, Speed: {speed_cost}")

        self.visualize_trajectories(trajectories, best_traj)

        if self.debug_plot_enabled:
            self.debug_plot(trajectories, best_traj, self.current_pose, self.goal_pose, self.initial_pose)

        return (best_traj[0], best_traj[1]) if best_traj else None

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

    def calculate_path_cost(self, trajectory):
        dx = self.goal_pose.position.x - self.current_pose.position.x
        dy = self.goal_pose.position.y - self.current_pose.position.y
        _, _, current_yaw = quat2euler([self.current_pose.orientation.w, self.current_pose.orientation.x, self.current_pose.orientation.y, self.current_pose.orientation.z])
        goal_x_local = dx * math.cos(-current_yaw) - dy * math.sin(-current_yaw)
        goal_y_local = dx * math.sin(-current_yaw) + dy * math.cos(-current_yaw)
        traj_x, traj_y, _ = trajectory[-1]
        numerator = abs(goal_y_local * traj_x - goal_x_local * traj_y)
        denominator = math.hypot(goal_x_local, goal_y_local)
        return 0.0 if denominator < 0.01 else numerator / denominator

    def calculate_dist_cost(self, trajectory):
        min_dist_to_obstacle = float('inf')
        obstacle_points = []
        for i, scan_dist in enumerate(self.laser_scan.ranges):
            if np.isinf(scan_dist) or np.isnan(scan_dist):
                continue
            if scan_dist < 0.2:  # Ignore very close obstacles
                continue
            scan_angle = self.laser_scan.angle_min + i * self.laser_scan.angle_increment
            ox = scan_dist * math.cos(scan_angle)
            oy = scan_dist * math.sin(scan_angle)
            obstacle_points.append((ox, oy))

        if not obstacle_points:
            return 0.0

        for traj_x, traj_y, _ in trajectory:
            for obs_x, obs_y in obstacle_points:
                dist = math.hypot(traj_x - obs_x, traj_y - obs_y)
                if dist < min_dist_to_obstacle:
                    min_dist_to_obstacle = dist

        if min_dist_to_obstacle <= self.robot_radius:
            return float('inf') # Collision is infinite cost

        safe_distance = 1.0 # Obstacles beyond this distance have zero cost
        if min_dist_to_obstacle > safe_distance:
            return 0.0
        
        # Linearly scale the cost from a max value down to 0
        max_cost = 10.0 # The cost when an obstacle is at robot_radius
        cost = (safe_distance - min_dist_to_obstacle) / (safe_distance - self.robot_radius) * max_cost
        return cost
            
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

    def debug_plot(self, trajectories, best_traj, plot_now = False):

        if plot_now:
            self.initial_pose = self.pose_array[0] if self.pose_array else None
            x_array = [p.position.x for p in self.pose_array]
            y_array = [p.position.y for p in self.pose_array]
            plt.figure(figsize=(10, 10))
            plt.plot(x_array, y_array, 'ro', label='Robot Path')
            plt.plot(self.goal_pose.position.x, self.goal_pose.position.y, 'go', label='Goal')
            plt.xlim(-5, 5)
            plt.ylim(-5, 5)
            plt.xlabel('X Position (m)')
            plt.ylabel('Y Position (m)')
            plt.title('Robot Path and Goal Position')
            plt.legend()
            plt.grid()
            plt.show()
            self.trajectories_array.clear()
            self.best_trajectory_array.clear()  
        else:
            self.trajectories_array.append(trajectories)
            self.best_trajectory_array.append(best_traj)
            self.pose_array.append(self.current_pose)   
        
def main(args=None):
    rclpy.init(args=args)
    dwa_planner = DWAPlanner()
    debug = False

    if debug:
        dwa_planner.debug_plot_enabled = True

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
