# AStar-Algorithm
AStar algorithm from scratch

This repositiory includes an A* algorithm coded from scratch. The algorithm takes in a grid of 1's and 0's indicating occupancy and a start and goal position. Then depending on desired functionality, the algorithm can plan a path in various scenarios:
1. varying grid sizes
2. offline and online implementations
3. plan then move, plan and move simulataneously using a PID controlled motion model

Files:

`Astar.py` - Astar class contains functions for each functionality
    `run_Offline_Astar` - runs classic offline Astar
    `run_Online_Astar` - runs simple online Astar moving between grids
    `run_Online_AStar_Control` - runs online Astar on a mobile robot

`controller.py` - contains two classic for the motion controller when the robot moves after path has been found `Controller` and simultaneous path and motion `Controller_Live`

`buildGrid.py` -  Creates the occupancy grid from given landscape data

`run.py` - runs all examples and combinations of the model is capable of and prints figures of each
