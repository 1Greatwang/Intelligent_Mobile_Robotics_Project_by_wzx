"""
In this file, you should implement your trajectory generation class or function.
Your method must generate a smooth 3-axis trajectory (x(t), y(t), z(t)) that 
passes through all the previously computed path points. A positional deviation 
up to 0.1 m from each path point is allowed.

You should output the generated trajectory and visualize it. The figure must
contain three subplots showing x, y, and z, respectively, with time t (in seconds)
as the horizontal axis. Additionally, you must plot the original discrete path 
points on the same figure for comparison.

You are expected to write the implementation yourself. Do NOT copy or reuse any 
existing trajectory generation code from others. Avoid using external packages 
beyond general scientific libraries such as numpy, math, or scipy. If you decide 
to use additional packages, you must clearly explain the reason in your report.
"""

import matplotlib.pyplot as plt
import numpy as np

def plot_path_coordinates(self, path, algorithm_name="Path"):
        """
        绘制路径在各坐标轴上的变化曲线
        """
        
        # 转换为numpy数组以便处理
        path_array = np.array(path)
        
        # 创建图形和子图
        fig, axs = plt.subplots(3, 1, figsize=(10, 8))
        time_steps = range(len(path_array))
        
        # 绘制x坐标变化
        axs[0].plot(time_steps, path_array[:, 0], marker='o', linestyle='-', color='r', label='X Coordinate')
        axs[0].set_ylabel('X Coordinate')
        axs[0].set_title(f'{algorithm_name} - X Coordinate vs. Time Step')
        axs[0].grid(True)
        axs[0].legend()
        
        # 绘制y坐标变化
        axs[1].plot(time_steps, path_array[:, 1], marker='o', linestyle='-', color='g', label='Y Coordinate')
        axs[1].set_ylabel('Y Coordinate')
        axs[1].set_title(f'{algorithm_name} - Y Coordinate vs. Time Step')
        axs[1].grid(True)
        axs[1].legend()
        
        # 绘制z坐标变化
        axs[2].plot(time_steps, path_array[:, 2], marker='o', linestyle='-', color='b', label='Z Coordinate')
        axs[2].set_xlabel('Time Step')
        axs[2].set_ylabel('Z Coordinate')
        axs[2].set_title(f'{algorithm_name} - Z Coordinate vs. Time Step')
        axs[2].grid(True)
        axs[2].legend()
        
        plt.tight_layout()
        plt.show()
