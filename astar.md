# 经典A*算法程序流程  
## （1）初始阶段，将起始节点S加入到开放列表（Openlist）中，准备进行探索
## （2）进入循环阶段，不断进行以下操作：
  - 遍历开放列表，找出具有最小 F(n)值的节点，将其作为当前处理的节点
  - 将当前处理的节点移动到关闭列表（Closelist）中，表示该节点已被探索
  - 对于当前节点的相邻节点（8个），进行以下检查：
    - 如果节点不在开放列表中，则将其添加到开放列表，并记录其父节点为当前节点，同时计算 F(n), G(n), H(n)的值；
    - 如果节点已在开放列表中，则评估新发现的路径是否更优，即比较 G(n)的值，若是新路径更优，更新该节点的父节点为当前节点，并重新计算其 F(n)和G(n)的值;
    - 当目标节点出现在开放列表中时，表示已经找到到达目标的路径，此时终止循环
## （3）路径回溯阶段，从终点开始，沿着每个节点的父节点回溯到起始节点，一次构成完整的路径

# 代码实现
```python
import heapq
import numpy as np
import matplotlib.pyplot as plt
from math import sqrt
import time

def a_star_search(start, goal, env):
  """
  start:起点（r,c）
  goal: 终点（r,c）
  grid:迷宫矩阵
  """

  width = env.env_width
  length = env.env_length
  hight = env.env_height

  neighbors = [
            (1, 0, 0), (-1, 0, 0),  # x方向
            (0, 1, 0), (0, -1, 0),  # y方向
            (0, 0, 1), (0, 0, -1)   # z方向
        ]
  open_heap = []
  heapq.heappush(open_heap, (0+manhattan_distance(start, goal), 0, start))
  
  #最小堆（Min Heap）：父节点的值总是小于或等于其子节点的值，因此，根节点（即heap ）是整个堆中最小的元素
  #以字典的形式记录节点的父节点（子节点：父节点），方便回溯
  came_from = {}
  g = {start: 0} #存储每个节点到起点的已知最小代价G(n)的值
  expand_count = 0  #扩展节点数
  frontier_max = 1  #存放待扩展节点的最大长度，佐证启发式函数的优劣

  t0 = time.time()
  while open_heap:
    f, g_curr, current = heapq.heappop(open_heap)
    expand_count += 1

    if current == goal:
      #回溯路径
      path = []
      node = current
      while node in came_from:
        path.append(node)
        node = came_from[node]
      path.append(start)
      path.reverse()
      search_time = time.time() - t0
      return path, expand_count, frontier_max, search_time

    #遍历可能移动的方向
    for dx, dy, dz in neighbors:
      nx, ny, nz = current[0] + dx, current[1] + dy, current[2] + dz #邻居节点的坐标

      #判断邻居节点是否在地图内，且不是障碍物
      if (0 <= nx < width and 0 <= ny < length and 0 <= nz < hight and
        not env.is_collide((nx, ny, nz)) and not env.is_outside((nx, ny, nz))):
        #从起点到邻居节点（nx, ny）的代价（当前G值＋1）
        tentative_g = g[current] + 1

        #如果第一次访问这个邻居节点，或找到了一条更短路径
        if tentative_g < g.get((nx, ny, nz), float('inf')):
          #更新came_from用于回溯路径
          #表示到达（nx, ny, nz）的最优前驱节点是current节点
          came_from[(nx, ny, nz)] = current

          #更新从起点到邻居节点（nx, ny, nz）的最小代价G值
          g[(nx, ny, nz)] = tentative_g

          #计算f=g+h
          #G是从起点到邻居节点地的代价，h是邻居节点到目标点的启发式评价值
          f_neighbor = tentative_g + manhattan_distance((nx, ny, nz), goal)

          #将邻居节点加入openlist（优先队列），按f值排序
          #元组(f, g, node)中f用于排序，g可用作tie-break
          heapq.heappush(open_heap, (f_neighbor, tentative_g, (nx, ny, nz)))

          #更新frontier_max,用于统计open_list(frontier)最大长度
          frontier_max = max(frontier_max, len(open_heap))

  search_time = time.time() - t0
  return None, expand_count, frontier_max, search_time



def manhattan_distance(a, goal):
    return abs(a[0] - goal[0]) + abs(a[1] - goal[1]) + abs(a[2] - goal[2])

```