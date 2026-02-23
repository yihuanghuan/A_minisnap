import numpy as np
import heapq

class AStar3D:
    def __init__(self, grid_map):
        self.map = grid_map
        # 26-连通动作集
        self.actions = []
        for x in [-1, 0, 1]:
            for y in [-1, 0, 1]:
                for z in [-1, 0, 1]:
                    if x == 0 and y == 0 and z == 0:
                        continue
                    self.actions.append((x, y, z))

    def heuristic(self, a, b):
        return np.linalg.norm(np.array(a) - np.array(b))

    def search(self, start_world, goal_world):
        start_idx = self.map.coord_to_grid(start_world)
        goal_idx = self.map.coord_to_grid(goal_world)
        
        # 确保起点终点无障碍
        self.map.clear_area(start_world)
        self.map.clear_area(goal_world)

        # 优先队列 (f_score, g_score, current_node_idx)
        open_list = []
        start_h = self.heuristic(start_idx, goal_idx)
        heapq.heappush(open_list, (start_h, 0, start_idx))
        
        # 记录来源，用于回溯路径
        came_from = {}
        
        # 记录 g_score (从起点到当前的代价)
        g_score = {start_idx: 0}
        
        # 记录已经在 Closed List 的节点
        closed_set = set()
        
        found = False
        
        while open_list:
            _, current_g, current = heapq.heappop(open_list)
            
            if current == goal_idx:
                found = True
                break
            
            if current in closed_set:
                continue
            closed_set.add(current)
            
            for action in self.actions:
                neighbor = (current[0] + action[0], 
                            current[1] + action[1], 
                            current[2] + action[2])
                
                if self.map.is_collision(neighbor):
                    continue
                
                step_cost = np.linalg.norm(action)
                tentative_g = current_g + step_cost
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + self.heuristic(neighbor, goal_idx)
                    heapq.heappush(open_list, (f_score, tentative_g, neighbor))
                    
        if found:
            # 回溯路径
            path_indices = [goal_idx]
            while path_indices[-1] != start_idx:
                path_indices.append(came_from[path_indices[-1]])
            path_indices.reverse()
            
            # 转换为真实坐标
            path_world = [self.map.grid_to_coord(idx) for idx in path_indices]
            return np.array(path_world)
        else:
            print("A* failed to find a path!")
            return None