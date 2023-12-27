import numpy as np
import matplotlib.pyplot as plt
from buildGrid import Grid
from node import Node
from functions import world_to_grid, grid_to_world
from controller import Controller_Live

class AStar() :

    def __init__(self, start, goal, cell_size) :

        # initial conditions and grid properties
        self.start = start
        self.goal = goal
        self.cell_size = cell_size
        self.grid_obj = Grid(cell=cell_size)
        self.grid = self.grid_obj.grid
        self.x_range = self.grid_obj.x_range
        self.y_range = self.grid_obj.y_range

        # start and goal grid positions
        row, col = world_to_grid(start[0],start[1],self.x_range,self.y_range,self.cell_size)
        self.start_grid = [row, col] # [row, column]
        row, col = world_to_grid(goal[0],goal[1],self.x_range,self.y_range,self.cell_size)
        self.goal_grid = [row, col] # [row, column]
        
        # initial node and sets
        h = self.heuristic_function(self.start_grid[0],self.start_grid[1])
        g = 0
        t = h+g
        self.base_node = Node(self.start_grid[0],self.start_grid[1],h,g,t)
        self.open_set = [self.base_node]
        self.closed_set = []

        # initialize controller
        self.controller = Controller_Live(start)

        # begin searching for goal
        self.searching = True

    def heuristic_function(self, row, col) :
        # row and col of neighbor

        goal_row = self.goal_grid[0]
        goal_col = self.goal_grid[1]

        # # manhattan distance divide by 3
        # man_dist = abs(goal_row-row) + abs(goal_col-col)
        # heur = man_dist/3.0

        # euclidian distance divide by 2
        dist = np.sqrt((goal_row-row)**2 + (goal_col-col)**2)
        heur = dist/2.0

        # # chebyshev distance
        # row_diff = abs(goal_row-row)
        # col_diff = abs(goal_col-col)
        # heur = max(row_diff,col_diff)

        return heur
    
    def cost_function(self,row,col):

        if self.grid[row][col] == 0 :
            cost = 1
        else :
            cost = 1000
        
        return cost
    
    def find_neighbors(self):

        curr_row = self.base_node.row
        curr_col = self.base_node.col

        total_rows = int((self.y_range[1]-self.y_range[0])/self.cell_size)
        total_cols = int((self.x_range[1]-self.x_range[0])/self.cell_size)

        if curr_row == 0 :
            row = [0,1]
        elif curr_row == (total_rows-1) :
            row = [0,-1]
        else :
            row = [0,1,-1]
        
        if curr_col == 0 :
            col = [0,1]
        elif curr_col == (total_cols-1) :
            col = [0,-1]
        else :
            col = [0,1,-1]
        
        neighbors = np.array([])
        for aa in col :
            for ii in row :
                neighbors = np.append(neighbors, [curr_row+ii, curr_col+aa])
        
        neighbors = np.reshape(neighbors,(len(row)*len(col),2))
        self.neighbors = neighbors[1:]

    def run_Offline_AStar(self):

        while self.searching:

            # find node with lowest total cost :
            t_min = float('inf')
            for node in self.open_set :
    
                if node.t < t_min :
                    t_min = node.t
                    min_node = node
            
            # store node as current node
            self.base_node = min_node
            self.open_set.remove(min_node)

            # check if node is goal
            if self.base_node.row == self.goal_grid[0] and self.base_node.col == self.goal_grid[1] :
                self.closed_set.append(self.base_node)
                self.searching = False
                continue

            # find neighbors
            self.find_neighbors()

            # check if neighbors already in open or closed set
            for nn in self.neighbors:

                # evaluate g
                g_neigh = self.cost_function(int(nn[0]),int(nn[1]))
                h_neigh = self.heuristic_function(nn[0],nn[1])
                t_neigh = g_neigh + self.base_node.g + h_neigh 
                neigh_node = Node(nn[0],nn[1],h_neigh,total=t_neigh)

                if neigh_node in self.closed_set :
                    continue

                if neigh_node not in self.open_set :
                    neigh_node.g = g_neigh
                    neigh_node.parent = self.base_node
                    self.open_set.append(neigh_node)
                else :
                    ind = self.open_set.index(neigh_node)
                    g = self.open_set[ind].g

                    if g_neigh < g:
                        self.open_set[ind].g = g_neigh
                        self.open_set[ind].t = self.open_set[ind].h + g_neigh
                        self.open_set[ind].parent = self.base_node

            # append base_node to closed set
            self.closed_set.append(self.base_node)

    def run_Online_AStar(self) :

        while self.searching:

            # find node with lowest total cost :
            t_min = float('inf')
            for node in self.open_set :
    
                if node.t < t_min :
                    t_min = node.t
                    min_node = node
            
            # store node as current node
            self.base_node = min_node
            self.open_set = []

            # check if node is goal
            if self.base_node.row == self.goal_grid[0] and self.base_node.col == self.goal_grid[1] :
                self.closed_set.append(self.base_node)
                self.searching = False
                continue

            # find neighbors
            self.find_neighbors()

            # check if neighbors already in open or closed set
            for nn in self.neighbors:

                # evaluate g
                g_neigh = self.cost_function(int(nn[0]),int(nn[1]))
                h_neigh = self.heuristic_function(nn[0],nn[1])
                t_neigh = g_neigh + self.base_node.g + h_neigh 
                neigh_node = Node(nn[0],nn[1],h_neigh,total=t_neigh)

                if neigh_node in self.closed_set :
                    continue

                if neigh_node not in self.open_set :
                    neigh_node.g = g_neigh
                    neigh_node.parent = self.base_node
                    self.open_set.append(neigh_node)

            # append base_node to closed set
            self.closed_set.append(self.base_node)

            if len(self.open_set) < 1 :
                print('Model Stuck')
                self.searching = False

    def run_Online_AStar_Control(self) :

        while self.searching:

            current_node_x,current_node_y = grid_to_world(self.base_node.row, self.base_node.col, self.x_range[0], self.y_range[0], self.cell_size)
            
            # find node with lowest total cost :
            t_min = float('inf')
            for node in self.open_set :
    
                if node.t < t_min :
                    t_min = node.t
                    min_node = node
            
            # store node as current node
            self.base_node = min_node
            self.open_set = []

            goal_node_x, goal_node_y = grid_to_world(self.base_node.row, self.base_node.col, self.x_range[0], self.y_range[0], self.cell_size)
            
            # run controller
            # get new current position
            new_x, new_y = self.controller.control_planning([goal_node_x, goal_node_y])
    
            # transfer to grid coordinates
            new_row, new_col = world_to_grid(new_x,new_y, self.x_range, self.y_range, self.cell_size)
            
            self.base_node.row = new_row
            self.base_node.col = new_col

            # check if node is goal
            if self.base_node.row == self.goal_grid[0] and self.base_node.col == self.goal_grid[1] :
                self.closed_set.append(self.base_node)
                final_x,final_y = self.controller.control_planning([self.goal[0],self.goal[1]])
                self.searching = False
                continue

            # find neighbors
            self.find_neighbors()

            # check if neighbors already in open or closed set
            for nn in self.neighbors:

                # evaluate g
                g_neigh = self.cost_function(int(nn[0]),int(nn[1]))
                h_neigh = self.heuristic_function(nn[0],nn[1])
                t_neigh = g_neigh + self.base_node.g + h_neigh 
                neigh_node = Node(nn[0],nn[1],h_neigh,total=t_neigh)

                if neigh_node in self.closed_set :
                    continue

                if neigh_node not in self.open_set :
                    neigh_node.g = g_neigh
                    neigh_node.parent = self.base_node
                    self.open_set.append(neigh_node)

            # append base_node to closed set
            self.closed_set.append(self.base_node)

            if len(self.open_set) < 1 :
                print('Model Stuck')
                self.searching = False

    def path_planning(self):

        ending = self.closed_set[-1]
        curr_node = ending
        planning = True
        grids = [[curr_node.row,curr_node.col]]
        total_costs = [curr_node.t]
        
        while planning :

            parent = curr_node.parent
            grids.append([parent.row,parent.col])
            total_costs.append([parent.t])
            curr_node = parent

            if curr_node.row == self.start_grid[0] and curr_node.col == self.start_grid[1] :
                self.path = grids
                self.total_costs = total_costs
                planning = False

    def plotting(self, fig, ax) :

        columns_x = np.array(self.path)[:,1]
        rows_y = np.array(self.path)[:,0]
        col_range = self.x_range
        row_range = self.y_range

        row_y = []
        col_x = []
        for ii in range(len(columns_x)) :
            x,y = grid_to_world(rows_y[ii], columns_x[ii], col_range[0], row_range[0], self.cell_size)
            row_y.append(y)
            col_x.append(x)
        
        self.row_y = row_y
        self.col_x = col_x

        # ranges of x and y
        extent = [col_range[0], col_range[1], row_range[0], row_range[1]]

        # plot landmarks
        ax.imshow(self.grid_obj.grid, cmap='Blues', extent=extent, origin='lower')

        # grid positions
        x_grid = self.cell_size*np.arange(int(col_range[0]/self.cell_size),int(col_range[1]/self.cell_size),1)
        y_grid = self.cell_size*np.arange(int(row_range[0]/self.cell_size),int(row_range[1]/self.cell_size),1)
        # grid labels
        x_grid_labels = [str(int(i)) if i % 1 ==0 else '' for i in x_grid]
        y_grid_labels = [str(int(i)) if i % 1 ==0 else '' for i in y_grid]

        # plot grid
        ax.grid(True, color='black', linestyle='-', linewidth=0.5)
        ax.set_xticks(x_grid, x_grid_labels)
        ax.set_yticks(y_grid, y_grid_labels)
        ax.set_xlabel("X axis")
        ax.set_ylabel("Y axis")

        # plot start and goal
        ax.scatter(self.start[0], self.start[1], color='red', label='Start')
        ax.scatter(self.goal[0], self.goal[1], color='green', label='Goal')
        ax.legend()

        # plot path
        ax.plot(col_x,row_y, color='red', label="A* path")

    def plotting_control(self, fig, ax) :

        columns_x = np.array(self.path)[:,1]
        rows_y = np.array(self.path)[:,0]
        col_range = self.x_range
        row_range = self.y_range

        row_y = []
        col_x = []
        for ii in range(len(columns_x)) :
            x,y = grid_to_world(rows_y[ii], columns_x[ii], col_range[0], row_range[0], self.cell_size)
            row_y.append(y)
            col_x.append(x)
        
        self.row_y = row_y
        self.col_x = col_x

        # ranges of x and y
        extent = [col_range[0], col_range[1], row_range[0], row_range[1]]

        # plot landmarks
        ax.imshow(self.grid_obj.grid, cmap='Blues', extent=extent, origin='lower')

        # grid positions
        x_grid = self.cell_size*np.arange(int(col_range[0]/self.cell_size),int(col_range[1]/self.cell_size),1)
        y_grid = self.cell_size*np.arange(int(row_range[0]/self.cell_size),int(row_range[1]/self.cell_size),1)
        # grid labels
        x_grid_labels = [str(int(i)) if i % 1 ==0 else '' for i in x_grid]
        y_grid_labels = [str(int(i)) if i % 1 ==0 else '' for i in y_grid]

        # plot grid
        ax.grid(True, color='black', linestyle='-', linewidth=0.5)
        ax.set_xticks(x_grid, x_grid_labels)
        ax.set_yticks(y_grid, y_grid_labels)
        ax.set_xlabel("X axis")
        ax.set_ylabel("Y axis")

        # plot start and goal
        ax.scatter(self.start[0], self.start[1], color='red', label='Start')
        ax.scatter(self.goal[0], self.goal[1], color='green', label='Goal')
        ax.legend()

        # plot path
        ax.plot(col_x,row_y, color='red', label="A* path")

        ax.plot(self.controller.control_path_x, self.controller.control_path_y)
        for pp in range(len(self.controller.control_path_h)) : 

            if (pp % 100) == 0 :
                angle = self.controller.control_path_h[pp]
                x = self.controller.control_path_x[pp]
                y = self.controller.control_path_y[pp]

                ddx = 0.25*np.cos(angle)
                ddy = 0.25*np.sin(angle)

                plt.arrow(x,y,ddx,ddy,color="orange")