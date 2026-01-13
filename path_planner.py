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
from flight_environment import FlightEnvironment            
import heapq
import numpy as np
import matplotlib.pyplot as plt
import random
from math import sqrt
import math

class AStarPathPlanner:
    def __init__(self, env):
        self.env = env
        self.actions = [
            (1, 0, 0), (-1, 0, 0), # x
            (0, 1, 0), (0, -1, 0), # y
            (0, 0, 1), (0, 0, -1)  # z
        ] 

        

    def heuristic(self, a, b):
        '''
        欧几里得距离
        '''
        return sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)

    def get_neighbors(self, point):
        '''
        获取当前点所有相邻点
        '''
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

class RRTPathPlanner:
    def __init__(self, env, step_size=1.0, max_iter=2000):
        self.env = env
        self.step_size = step_size  # 步长，控制每次扩展的距离
        self.max_iter = max_iter    # 最大迭代次数
        
    def distance(self, point1, point2):
        """计算两点之间的欧几里得距离"""
        return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2 + (point1[2] - point2[2])**2)
    
    def nearest_node(self, nodes, target_point):
        """找到树中距离目标点最近的节点"""
        min_dist = float('inf')
        nearest = nodes[0]
        for node in nodes:
            dist = self.distance(node, target_point)
            if dist < min_dist:
                min_dist = dist
                nearest = node
        return nearest
    
    def steer(self, from_point, to_point):
        """从一个点向另一个点扩展一步，步长受限制"""
        dist = self.distance(from_point, to_point)
        if dist <= self.step_size:
            return to_point
        else:
            # 沿着方向扩展一个步长
            ratio = self.step_size / dist
            new_x = from_point[0] + ratio * (to_point[0] - from_point[0])
            new_y = from_point[1] + ratio * (to_point[1] - from_point[1])
            new_z = from_point[2] + ratio * (to_point[2] - from_point[2])
            return (new_x, new_y, new_z)
    
    def is_valid_path(self, point1, point2):
        """检查从point1到point2的路径是否有效（无障碍物）"""
        # 将路径离散化并检查中间点
        steps = int(self.distance(point1, point2) / 0.5)  # 使用较小的步长检查路径
        if steps == 0:
            steps = 1
            
        for i in range(steps + 1):
            t = i / steps
            x = point1[0] * (1 - t) + point2[0] * t
            y = point1[1] * (1 - t) + point2[1] * t
            z = point1[2] * (1 - t) + point2[2] * t
            if self.env.is_collide((x, y, z)) or self.env.is_outside((x, y, z)):
                return False
        return True
    
    def plan_path(self, start, goal):
        """
        使用RRT算法规划从起点到终点的路径
        """
        # 节点列表，每个节点是一个三元组(x, y, z)
        nodes = [start]
        # 父节点字典，用于重建路径
        parent_map = {start: None}
        
        for _ in range(self.max_iter):
            # 随机采样一个点
            rand_x = random.uniform(0, self.env.env_width)
            rand_y = random.uniform(0, self.env.env_length)
            rand_z = random.uniform(0, self.env.env_height)
            rand_point = (rand_x, rand_y, rand_z)
            
            # 有一定概率直接朝向目标点生长（偏向目标）
            if random.random() < 0.1:  # 10%的概率朝向目标
                rand_point = goal
            
            # 找到最近的节点
            nearest = self.nearest_node(nodes, rand_point)
            
            # 从最近节点向随机点扩展一步
            new_point = self.steer(nearest, rand_point)
            
            # 检查新点是否有效（不在障碍物内且在环境中）
            if not self.env.is_collide(new_point) and not self.env.is_outside(new_point):
                # 检查从最近点到新点的路径是否有效
                if self.is_valid_path(nearest, new_point):
                    nodes.append(new_point)
                    parent_map[new_point] = nearest
                    
                    # 检查是否到达目标附近
                    if self.distance(new_point, goal) <= self.step_size:
                        # 连接到目标点
                        if self.is_valid_path(new_point, goal):
                            parent_map[goal] = new_point
                            
                            # 重建路径
                            path = []
                            current = goal
                            while current is not None:
                                path.append(current)
                                current = parent_map[current]
                            path.reverse()
                            
                            return path
        
        # 如果未找到路径，尝试连接到最接近目标的节点
        nearest_to_goal = self.nearest_node(nodes, goal)
        if self.distance(nearest_to_goal, goal) <= self.step_size and self.is_valid_path(nearest_to_goal, goal):
            # 建立连接
            parent_map[goal] = nearest_to_goal
            
            # 重建路径
            path = []
            current = goal
            while current is not None:
                path.append(current)
                current = parent_map[current]
            path.reverse()
            
            return path
        
        # 仍未找到路径，返回空列表
        return []

def plan_path_rrt(env, start, goal):
    """
    使用RRT算法进行路径规划的函数接口
    """
    planner = RRTPathPlanner(env)
    path = planner.plan_path(start, goal)
    return path










