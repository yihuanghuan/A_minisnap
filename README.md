# A* + Minimum Snap 轨迹规划器

基于 ROS 2 的无人机轨迹规划与几何控制工程。系统将 **A\* 三维路径搜索**、**Minimum Snap 轨迹优化**与**几何控制器**相结合，为四旋翼无人机生成平滑且动力学可行的飞行轨迹。

## 项目简介

本工程包含两个 ROS 2 功能包：

```
A_minisnap/
├── geometric_controller/     # C++ 几何控制器（内环 + 外环）
└── trajectory_planner_py/    # Python 轨迹规划器（A* + Min-Snap）
```

## 系统架构

```
trajectory_planner_py            geometric_controller
┌──────────────────────┐        ┌──────────────────────────┐
│  栅格地图构建         │        │   外环控制器              │
│  A* 三维路径搜索      │──────▶ │   （位置 + 速度控制）     │
│  Min-Snap 轨迹优化   │        │   内环控制器              │
│  偏航角轨迹生成       │        │   （姿态控制）            │
└──────────────────────┘        └──────────────────────────┘
```

## 功能包说明

### `trajectory_planner_py`（Python）

负责高层路径规划与轨迹生成。

| 模块 | 功能描述 |
|---|---|
| `a_star_3d.py` | 26 连通的三维 A\* 路径搜索 |
| `grid_map.py` | 占用栅格地图管理 |
| `min_snap_trajectory_generators.py` | Minimum Snap / Minimum Acceleration 轨迹优化 |
| `planner_node.py` | ROS 2 节点入口 |
| `waypoints.py` | 航路点管理工具 |
| `linear_type.py` | 线性轨迹多项式工具 |
| `optimize_type.py` | 多项式轨迹优化类型定义 |

**依赖：** `numpy`、`rclpy`、`geometry_msgs`、`px4_msgs`

### `geometric_controller`（C++）

实现用于轨迹跟踪的级联几何控制器。

| 源文件 | 功能描述 |
|---|---|
| `outer_loop_controller.cpp` | 位置/速度外环控制器，计算期望推力方向 |
| `inner_loop_controller.cpp` | 姿态内环控制器，控制横滚、俯仰、偏航 |
| `off_board_control.cpp` | Offboard 模式管理及 PX4 接口 |
| `trajectory_utils.cpp` | 轨迹采样与辅助工具函数 |

**依赖：** `rclcpp`、`px4_msgs`、`geometry_msgs`、`std_msgs`、`Eigen3`

## 环境依赖

- ROS 2（Humble 及以上版本）
- PX4 Autopilot（用于 SITL 仿真）
- Python 3.8+
- `numpy`、`scipy`
- Eigen3

## 编译

```bash
cd ~/learning/A*_minisnap
colcon build --symlink-install
source install/setup.bash
```

## 使用方法

### 启动轨迹规划器

```bash
ros2 launch trajectory_planner_py planner.launch.py
```

### 启动几何控制器

```bash
ros2 launch geometric_controller geometric_controller_launch.py
```

## 算法说明

### A\* 三维路径搜索

在占用栅格地图中，使用 26 连通邻域和欧氏距离启发函数进行三维 A\* 搜索，规划出无碰撞路径。

### Minimum Snap 轨迹优化

通过优化多项式轨迹系数，最小化位置的四阶导数（Snap），从而生成平滑、动力学可行的四旋翼轨迹。

### 几何控制器

采用级联控制结构：
- **外环**：PD 位置控制器，输出期望总推力与姿态
- **内环**：姿态控制器，跟踪期望横滚、俯仰和偏航角

## 许可证

各功能包许可证详见各自目录。其中 `geometric_controller` 包采用 **Apache-2.0** 许可证。
