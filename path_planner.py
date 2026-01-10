"""
In this file, you should implement your own path planning class or function.
Within your implementation, you may call `env.is_collide()` and `env.is_outside()`
to verify whether candidate path points collide with obstacles or exceed the
environment boundaries.

You are required to write the path planning algorithm by yourself. Copying or calling 
any existing path planning algorithms from others is strictly
prohibited. Please avoid using external packages beyond common Python libraries
such as `numpy`, `math`, or `scipy`. If you must use additional packages, you
must clearly explain the reason in your report.
"""
import heapq
import numpy as np
import matplotlib.pyplot as plt
from math import sqrt

class AStarPathPlanner:
    def __init__(self, env):
        self.env = env
        # 定义动作（6个方向：上下左右前后）
        self.actions = [
            (1, 0, 0), (-1, 0, 0),  # x方向
            (0, 1, 0), (0, -1, 0),  # y方向
            (0, 0, 1), (0, 0, -1)   # z方向
        ]
        
    def heuristic(self, a, b):
        """
        计算两个点之间的启发式距离（欧几里得距离）
        """
        return sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2 + (a[2] - b[2])**2)
    
    def get_neighbors(self, point):
        """
        获取当前点的所有邻居节点
        """
        neighbors = []
        for action in self.actions:
            neighbor = (point[0] + action[0], point[1] + action[1], point[2] + action[2])
            # 检查是否在环境边界内且不与障碍物碰撞
            if not self.env.is_outside(neighbor) and not self.env.is_collide(neighbor):
                neighbors.append(neighbor)
        return neighbors
    
    def plan_path(self, start, goal):
        """
        使用A*算法规划从起点到终点的路径
        """
        # 初始化开放列表和关闭列表
        open_list = [(0, start)]
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        
        while open_list:
            # 获取f_score最小的节点
            current = heapq.heappop(open_list)[1]
            
            # 如果到达目标点，则重构路径
            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                return path
            
            # 遍历所有邻居节点
            for neighbor in self.get_neighbors(current):
                tentative_g_score = g_score[current] + self.heuristic(current, neighbor)
                
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + self.heuristic(neighbor, goal)
                    heapq.heappush(open_list, (f_score[neighbor], neighbor))
        
        # 如果找不到路径，返回空列表
        return []
    
    

def plan_path_astar(env, start, goal):
    """
    使用A*算法进行路径规划的函数接口
    """
    planner = AStarPathPlanner(env)
    path = planner.plan_path(start, goal)
    return path           