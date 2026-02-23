import numpy as np

class GridMap:
    def __init__(self, map_bound, resolution=0.5, num_blocks=5, seed=10):
        """
        map_bound: [xmin, xmax, ymin, ymax, zmin, zmax]
        """
        self.resolution = resolution
        self.min_bound = np.array([map_bound[0], map_bound[2], map_bound[4]])
        self.max_bound = np.array([map_bound[1], map_bound[3], map_bound[5]])
        
        # 计算栅格尺寸
        self.grid_size = np.ceil((self.max_bound - self.min_bound) / resolution).astype(int)
        
        # 0: 无障碍, 1: 有障碍
        self.grid = np.zeros(self.grid_size, dtype=int)
        
        # 调用障碍物生成
        self._generate_obstacles()

    def _generate_obstacles(self):
        # --- 修改：手动设置障碍物以演示避障 ---
        # 场景假设：起点约为 [0,0,0], 终点 [4,3,2]
        
        # 我们放置一堵墙，强制无人机向上或者向侧面绕行
        # 墙的位置：X=[1.5, 2.5], Y=[-2.0, 3.0] (阻断了 y=x 的直线路径)
        # Z范围: [0, 5.0]
        
        # 1. 定义墙的真实坐标范围
        wall_x = [1.5, 2.5]
        wall_y = [-2.0, 3.0]
        wall_z = [0.0, 5.0]
        
        self._add_block(wall_x, wall_y, wall_z)
        
        # (可选) 你可以在这里添加更多障碍物...
        # self._add_block([3.0, 3.5], [3.0, 3.5], [0.0, 5.0])

    def _add_block(self, x_range, y_range, z_range):
        """ 辅助函数：根据真实坐标范围添加障碍物块 """
        p_min = np.array([x_range[0], y_range[0], z_range[0]])
        p_max = np.array([x_range[1], y_range[1], z_range[1]])
        
        # 转为栅格索引
        idx_min = self.coord_to_grid(p_min)
        idx_max = self.coord_to_grid(p_max)
        
        # 填充栅格
        self.grid[idx_min[0]:idx_max[0], idx_min[1]:idx_max[1], idx_min[2]:idx_max[2]] = 1

    def coord_to_grid(self, coord):
        """ 将真实坐标转为栅格索引 """
        idx = np.round((coord - self.min_bound) / self.resolution).astype(int)
        # 限制在范围内
        idx = np.maximum(0, np.minimum(self.grid_size - 1, idx))
        return tuple(idx)

    def grid_to_coord(self, idx):
        """ 将栅格索引转为真实坐标 """
        return np.array(idx) * self.resolution + self.min_bound

    def is_collision(self, idx):
        # 越界检查
        if (idx[0] < 0 or idx[0] >= self.grid_size[0] or
            idx[1] < 0 or idx[1] >= self.grid_size[1] or
            idx[2] < 0 or idx[2] >= self.grid_size[2]):
            return True
        # 障碍物检查
        return self.grid[idx[0], idx[1], idx[2]] == 1

    def clear_area(self, coord, safe_radius=2):
        """ 清理起点和终点周围的障碍物 (挖空) """
        idx = self.coord_to_grid(coord)
        x_min = max(0, idx[0] - safe_radius)
        x_max = min(self.grid_size[0], idx[0] + safe_radius + 1)
        y_min = max(0, idx[1] - safe_radius)
        y_max = min(self.grid_size[1], idx[1] + safe_radius + 1)
        z_min = max(0, idx[2] - safe_radius)
        z_max = min(self.grid_size[2], idx[2] + safe_radius + 1)
        
        self.grid[x_min:x_max, y_min:y_max, z_min:z_max] = 0