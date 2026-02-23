#!/usr/bin/env python3
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, TwistStamped, AccelStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from px4_msgs.msg import VehicleOdometry 

# 引入已有模块
from trajectory_planner_py.min_snap_trajectory_generators import (
    optimizeTrajectory,
    YawMinAccTrajectory,
)

# 引入新模块
from trajectory_planner_py.grid_map import GridMap
from trajectory_planner_py.a_star_3d import AStar3D

class TrajectoryNode(Node):

    def __init__(self) -> None:
        super().__init__("trajectory_generator")

        # --- 发布器 ---
        self._pose_pub = self.create_publisher(PoseStamped, "/target_pose", 10)
        self._twist_pub = self.create_publisher(TwistStamped, "/target_twist", 10)
        self._accel_pub = self.create_publisher(AccelStamped, "/target_accel", 10)
        
        # 可视化发布器
        qos_vis = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self._vis_pub = self.create_publisher(MarkerArray, "/planning_vis", qos_vis)

        # --- 订阅器 ---
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.create_subscription(VehicleOdometry, "/fmu/out/vehicle_odometry", self._odom_cb, qos_profile)

        # 记录启动时间
        self._start_ns = self.get_clock().now().nanoseconds

        # --- 内部状态 ---
        self._curr_pos_enu = None 
        self._is_planning_done = False
        self._traj_start_time = 0.0
        
        # --- 初始化地图 ---
        map_bound = [-5, 5, -5, 5, 0, 5] 
        self.grid_map = GridMap(map_bound, resolution=0.5, num_blocks=5, seed=10)
        self.planner = AStar3D(self.grid_map)
        
        # 目标点 (ENU)
        self.goal_pt = np.array([4.0, 2.0, 2.0]) 

        # 定时器
        update_hz = 100.0
        self.create_timer(1.0 / update_hz, self._on_timer)
        
        self.get_logger().info("Planner initialized. Waiting for drone position (and 12s delay)...")

    def _odom_cb(self, msg: VehicleOdometry) -> None:
        # 1. 启动延时 (12秒)
        now_ns = self.get_clock().now().nanoseconds
        time_since_boot = (now_ns - self._start_ns) * 1e-9
        if time_since_boot < 12.0:
            return

        # 2. NED -> ENU
        self._curr_pos_enu = np.array([msg.position[0], -msg.position[1], -msg.position[2]])

    def _generate_plan(self):
        if self._curr_pos_enu is None:
            return

        start_pt = self._curr_pos_enu
        self.get_logger().info(f"Generating path from {start_pt.round(2)} to {self.goal_pt}...")
        
        # 1. A* 搜索 (得到折线路径点)
        path_points = self.planner.search(start_pt, self.goal_pt)
        if path_points is None:
            path_points = np.array([start_pt, self.goal_pt])

        # 2. 时间分配
        diffs = np.diff(path_points, axis=0)
        dists = np.linalg.norm(diffs, axis=1)
        dists = np.maximum(dists, 0.6) 
        avg_vel = 0.5
        segment_times = dists / avg_vel
        t_wp = np.concatenate(([0], np.cumsum(segment_times)))
        xyz_wp = path_points

        # 3. Minimum Snap 优化 (得到多项式参数)
        self._xyz_traj = optimizeTrajectory(
            xyz_wp,
            t_wp,
            optim_target="poly-coeff",
            poly_order=7,
            floating_cubes=None,
            t_cubes=None,
        )
        
        yaw_wp = np.zeros(len(t_wp))
        self._yaw_traj = YawMinAccTrajectory(yaw_waypoints=yaw_wp, t_waypoints=t_wp)
        self.traj_duration = t_wp[-1]

        # --- 新增：采样平滑轨迹用于可视化 ---
        smooth_points = []
        dt = 0.05  # 采样步长 0.05s
        for t in np.arange(0, self.traj_duration, dt):
            pos, _, _, _, _ = self._xyz_traj.eval(t)
            smooth_points.append(pos)
        # 补上终点
        pos, _, _, _, _ = self._xyz_traj.eval(self.traj_duration)
        smooth_points.append(pos)

        # 4. 可视化 (传入 A*原始点 和 优化后平滑点)
        self.publish_visualization(path_points, smooth_points)
        
        self._traj_start_time = self.get_clock().now().nanoseconds * 1e-9
        self._is_planning_done = True
        self.get_logger().info("Trajectory generated and visualized.")

    def _on_timer(self) -> None:
        if not self._is_planning_done:
            if self._curr_pos_enu is not None:
                self._generate_plan()
            return

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        t_eval = now_sec - self._traj_start_time

        if t_eval > self.traj_duration:
            t_eval = self.traj_duration

        pos, vel, acc, _, _ = self._xyz_traj.eval(t_eval)
        target_rpy = np.array([0.0, 0.0, 0.0])

        self._publish_pose(pos, target_rpy)
        self._publish_twist(vel)
        self._publish_accel(acc)

    def publish_visualization(self, a_star_points, smooth_points):
        marker_array = MarkerArray()
        timestamp = self.get_clock().now().to_msg()
        id_cnt = 0

        # 1. 障碍物
        occ_x, occ_y, occ_z = np.where(self.grid_map.grid == 1)
        for i in range(len(occ_x)):
            idx = (occ_x[i], occ_y[i], occ_z[i])
            center = self.grid_map.grid_to_coord(idx)
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = timestamp
            marker.ns = "obstacles"
            marker.id = id_cnt
            id_cnt += 1
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = center[0]
            marker.pose.position.y = center[1]
            marker.pose.position.z = center[2]
            marker.scale.x = self.grid_map.resolution
            marker.scale.y = self.grid_map.resolution
            marker.scale.z = self.grid_map.resolution
            marker.color.a = 0.5
            marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0
            marker_array.markers.append(marker)

        # 2. A* 原始路径 (红色细线)
        marker_astar = Marker()
        marker_astar.header.frame_id = "map"
        marker_astar.header.stamp = timestamp
        marker_astar.ns = "path_astar"
        marker_astar.id = 1000
        marker_astar.type = Marker.LINE_STRIP
        marker_astar.action = Marker.ADD
        marker_astar.scale.x = 0.03  # 细线
        marker_astar.color.a = 0.6   # 半透明
        marker_astar.color.r = 1.0; marker_astar.color.g = 0.0; marker_astar.color.b = 0.0 # 红色
        for pt in a_star_points:
            p = Point(); p.x, p.y, p.z = pt[0], pt[1], pt[2]
            marker_astar.points.append(p)
        marker_array.markers.append(marker_astar)

        # 3. Minimum Snap 平滑路径 (绿色粗线)
        marker_smooth = Marker()
        marker_smooth.header.frame_id = "map"
        marker_smooth.header.stamp = timestamp
        marker_smooth.ns = "path_smooth"
        marker_smooth.id = 1001
        marker_smooth.type = Marker.LINE_STRIP
        marker_smooth.action = Marker.ADD
        marker_smooth.scale.x = 0.1  # 粗线
        marker_smooth.color.a = 1.0
        marker_smooth.color.r = 0.0; marker_smooth.color.g = 1.0; marker_smooth.color.b = 0.0 # 绿色
        for pt in smooth_points:
            p = Point(); p.x, p.y, p.z = pt[0], pt[1], pt[2]
            marker_smooth.points.append(p)
        marker_array.markers.append(marker_smooth)

        self._vis_pub.publish(marker_array)

    def _publish_twist(self, lin_vel: np.ndarray) -> TwistStamped:
        twist = TwistStamped()
        twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z = lin_vel
        self._twist_pub.publish(twist)
        return twist

    def _publish_accel(self, lin_acc: np.ndarray) -> AccelStamped:
        accel = AccelStamped()
        accel.accel.linear.x, accel.accel.linear.y, accel.accel.linear.z = lin_acc
        self._accel_pub.publish(accel)
        return accel

    def _publish_pose(self, xyz: np.ndarray, rpy: np.ndarray) -> PoseStamped:
        quat_xyzw = Rotation.from_euler("XYZ", rpy).as_quat()
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = xyz
        msg.pose.orientation.w = quat_xyzw[3]
        msg.pose.orientation.x = quat_xyzw[0]
        msg.pose.orientation.y = quat_xyzw[1]
        msg.pose.orientation.z = quat_xyzw[2]
        self._pose_pub.publish(msg)
        return msg

def main(argv: list[str] | None = None) -> None:
    rclpy.init(args=argv)
    node = TrajectoryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()