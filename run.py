from Astar import AStar
import matplotlib.pyplot as plt
from controller import Controller
import numpy as np

################# Code A Plots #################
# Testing Offline
run = AStar([0.5,-1.5],[0.5,1.5],1)
run.run_Offline_AStar()
run.path_planning()
fig = plt.figure("Offline A* 1m #1")
ax = fig.add_subplot()
ax.set_title("Offline A* - Condition 1")
run.plotting(fig, ax)

run = AStar([4.5,3.5],[4.5,-1.5],1)
run.run_Offline_AStar()
run.path_planning()
fig = plt.figure("Offline A* 1m #2")
ax = fig.add_subplot()
ax.set_title("Offline A* - Condition 2")
run.plotting(fig, ax)

run = AStar([-0.5,5.5],[1.5,-3.5],1)
run.run_Offline_AStar()
run.path_planning()
fig = plt.figure("Offline A* 1m #3")
ax = fig.add_subplot()
ax.set_title("Offline A* - Condition 3")
run.plotting(fig, ax)

# Testing Online
run = AStar([0.5,-1.5],[0.5,1.5],1)
run.run_Online_AStar()
run.path_planning()
fig = plt.figure("Online A* 1m #1")
ax = fig.add_subplot()
ax.set_title("Online A* - Condition 1")
run.plotting(fig, ax)

run = AStar([4.5,3.5],[4.5,-1.5],1)
run.run_Online_AStar()
run.path_planning()
fig = plt.figure("Online A* 1m #2")
ax = fig.add_subplot()
ax.set_title("Online A* - Condition 2")
run.plotting(fig, ax)

run = AStar([-0.5,5.5],[1.5,-3.5],1)
run.run_Online_AStar()
run.path_planning()
fig = plt.figure("Online A* 1m #3")
ax = fig.add_subplot()
ax.set_title("Online A* - Condition 3")
run.plotting(fig, ax)

# Testing Smaller Grids - Offline
run = AStar([2.45,-3.55],[0.95,-1.55],0.1)
run.run_Offline_AStar()
run.path_planning()
fig = plt.figure("Offline A* 0.1m #1")
ax = fig.add_subplot()
ax.set_title("Offline A* - Condition 1")
run.plotting(fig, ax)

run = AStar([4.95,-0.05],[2.45,0.25],0.1)
run.run_Offline_AStar()
run.path_planning()
fig = plt.figure("Offline A* 0.1m #2")
ax = fig.add_subplot()
ax.set_title("Offline A* - Condition 2")
run.plotting(fig, ax)

run = AStar([-0.55,1.45],[1.95,3.95],0.1)
run.run_Offline_AStar()
run.path_planning()
fig = plt.figure("Offline A* 0.1m #3")
ax = fig.add_subplot()
ax.set_title("Offline A* - Condition 3")
run.plotting(fig, ax)

# Testing Smaller Grids - Online
run = AStar([2.45,-3.55],[0.95,-1.55],0.1)
run.run_Online_AStar()
run.path_planning()
fig = plt.figure("Online A* 0.1m #1")
ax = fig.add_subplot()
ax.set_title("Online A* - Condition 1")
run.plotting(fig, ax)

run = AStar([4.95,-0.05],[2.45,0.25],0.1)
run.run_Online_AStar()
run.path_planning()
fig = plt.figure("Online A* 0.1m #2")
ax = fig.add_subplot()
ax.set_title("Online A* - Condition 2")
run.plotting(fig, ax)

run = AStar([-0.55,1.45],[1.95,3.95],0.1)
run.run_Online_AStar()
run.path_planning()
fig = plt.figure("Online A* 0.1m #3")
ax = fig.add_subplot()
ax.set_title("Online A* - Condition 3")
run.plotting(fig, ax)

################ Code B Plots #################
# Testing Online Path Planning then Controlling Robot
run = AStar([2.45,-3.55],[0.95,-1.55],0.1)
run.run_Online_AStar()
run.path_planning()
fig = plt.figure("Path then Control - Condition #1")
ax = fig.add_subplot()
ax.set_title("Path then Control - Condition 1")
run.plotting(fig, ax)
ut = Controller(run.col_x,run.row_y,[run.start[0], run.start[1]])
ut.control_path()
ut.plotting(fig,ax)

run = AStar([4.95,-0.05],[2.45,0.25],0.1)
run.run_Online_AStar()
run.path_planning()
fig = plt.figure("Path then Control - Condition #2")
ax = fig.add_subplot()
ax.set_title("Path then Control - Condition 2")
run.plotting(fig, ax)
ut = Controller(run.col_x,run.row_y,[run.start[0], run.start[1]])
ut.control_path()
ut.plotting(fig,ax)

run = AStar([-0.55,1.45],[1.95,3.95],0.1)
run.run_Online_AStar()
run.path_planning()
fig = plt.figure("Path then Control - Condition #3")
ax = fig.add_subplot()
ax.set_title("Path then Control - Condition 3")
run.plotting(fig, ax)
ut = Controller(run.col_x,run.row_y,[run.start[0], run.start[1]])
ut.control_path()
ut.plotting(fig,ax)

# Testing Online Path Planning with Controller
run = AStar([2.45,-3.55],[0.95,-1.55],0.1)
run.run_Online_AStar_Control()
run.path_planning()
fig = plt.figure("Path with Control - Condition #1")
ax = fig.add_subplot()
ax.set_title("Path with Control - Condition #1")
run.plotting_control(fig, ax)

run = AStar([4.95,-0.05],[2.45,0.25],0.1)
run.run_Online_AStar_Control()
run.path_planning()
fig = plt.figure("Path with Control - Condition #2")
ax = fig.add_subplot()
ax.set_title("Path with Control - Condition 2")
run.plotting_control(fig, ax)

run = AStar([-0.55,1.45],[1.95,3.95],0.1)
run.run_Online_AStar_Control()
run.path_planning()
fig = plt.figure("Path with Control - Condition #3")
ax = fig.add_subplot()
ax.set_title("Path with Control - Condition 3")
run.plotting_control(fig, ax)

# Testing Online Path Planning with Control - Large Grid
run = AStar([2.45,-3.55],[0.95,-1.55],1.0)
run.run_Online_AStar_Control()
run.path_planning()
fig = plt.figure("Path with Control - Condition #1 Large Grid")
ax = fig.add_subplot()
ax.set_title("Path with Control - Condition #1")
run.plotting_control(fig, ax)

run = AStar([4.95,-0.05],[2.45,0.25],1.0)
run.run_Online_AStar_Control()
run.path_planning()
fig = plt.figure("Path with Control - Condition #2 Large Grid")
ax = fig.add_subplot()
ax.set_title("Path with Control - Condition 2")
run.plotting_control(fig, ax)

run = AStar([-0.55,1.45],[1.95,3.95],1.0)
run.run_Online_AStar_Control()
run.path_planning()
fig = plt.figure("Path with Control - Condition #3 Large Grid")
ax = fig.add_subplot()
ax.set_title("Path with Control - Condition 3")
run.plotting_control(fig, ax)

plt.show()
