from flight_environment import FlightEnvironment
from path_planner import a_star_search

env = FlightEnvironment(50)
start = (1,2,0)
goal = (18,18,3)

# --------------------------------------------------------------------------------------------------- #
# Call your path planning algorithm here. 
# The planner should return a collision-free path and store it in the variable `path`. 
# `path` must be an N×3 numpy array, where:
#   - column 1 contains the x-coordinates of all path points
#   - column 2 contains the y-coordinates of all path points
#   - column 3 contains the z-coordinates of all path points
# This `path` array will be provided to the `env` object for visualization.

#path = [[0,0,0],[1,1,1],[2,2,2],[3,3,3]]

# --------------------------------------------------------------------------------------------------- #

# 调用A*搜索算法进行路径规划
result = a_star_search(start, goal, env)
if result[0] is not None:
    path = result[0]  # 获取路径
    print(f"找到路径，共 {len(path)} 个点")
    print(f"扩展节点数: {result[1]}")
    print(f"存放待扩展节点的最大长度: {result[2]}")
    print(f"搜索时间: {result[3]}")
else:
    print("未找到路径")
    # 如果找不到路径，使用默认路径保证程序继续运行



env.plot_cylinders(path)


# --------------------------------------------------------------------------------------------------- #
#   Call your trajectory planning algorithm here. The algorithm should
#   generate a smooth trajectory that passes through all the previously
#   planned path points.
#
#   After generating the trajectory, plot it in a new figure.
#   The figure should contain three subplots showing the time histories of
#   x, y, and z respectively, where the horizontal axis represents time (in seconds).
#
#   Additionally, you must also plot the previously planned discrete path
#   points on the same figure to clearly show how the continuous trajectory
#   follows these path points.




# --------------------------------------------------------------------------------------------------- #



# You must manage this entire project using Git. 
# When submitting your assignment, upload the project to a code-hosting platform 
# such as GitHub or GitLab. The repository must be accessible and directly cloneable. 
#
# After cloning, running `python3 main.py` in the project root directory 
# should successfully execute your program and display:
#   1) the 3D path visualization, and
#   2) the trajectory plot.
#
# You must also include the link to your GitHub/GitLab repository in your written report.