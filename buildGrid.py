import numpy as np
from data import Landmarks
from functions import world_to_grid

class Grid() :

    def __init__(self, cell=1.0) :

        self.x_range = [-2,5]
        self.y_range = [-6,6]
        self.cell_size = cell
        rows = int((self.y_range[1]-self.y_range[0])/self.cell_size)
        cols = int((self.x_range[1]-self.x_range[0])/self.cell_size)
        self.grid = np.zeros((rows,cols))
        self.landmarks = Landmarks()
        if cell < 1 :
            self.obs_small_cells()
        else :
            self.add_obstacles()

    def add_obstacles(self):

        for ii in range(len(self.landmarks.subjects)) :

            x = self.landmarks.x_pos[ii]
            y = self.landmarks.y_pos[ii]

            row, col = world_to_grid(x,y,self.x_range,self.y_range,self.cell_size)

            self.grid[row,col] = 1
    
    def obs_small_cells(self) :

        for ii in range(len(self.landmarks.subjects)) :

            x = self.landmarks.x_pos[ii]
            y = self.landmarks.y_pos[ii]
            total_rows = int((self.y_range[1]-self.y_range[0])/self.cell_size)
            total_cols = int((self.x_range[1]-self.x_range[0])/self.cell_size)

            row, col = world_to_grid(x,y,self.x_range,self.y_range,self.cell_size)

            if row == 0 :
                row_inc = [0,1,2,3]
            if row == 1 :
                row_inc = [0,1,-1,2,3]
            if row == 2 :
                row_inc = [0,1,2,-1,2,3]
            elif row == (total_rows-1) :
                row_inc = [0,-1,-2,-3]
            elif row == (total_rows-2) :
                row_inc = [0,1,-1,-2,-3]
            elif row == (total_rows-3) :
                row_inc = [0,1,2,-1,-2,-3]
            else :
                row_inc = [0,1,-1,2,-2,3,-3]
            
            if col == 0 :
                col_inc = [0,1,2,3]
            if col == 1 :
                col_inc = [0,1,-1,2,3]
            if col == 2 :
                col_inc = [0,1,2,-1,2,3]
            elif col == (total_cols-1) :
                col_inc = [0,-1,-2,-3]
            elif col == (total_cols-2) :
                col_inc = [0,1,-1,-2,-3]
            elif col == (total_cols-3) :
                col_inc = [0,1,2,-1,-2,-3]
            else :
                col_inc = [0,1,-1,2,-2,3,-3]

            for rr in row_inc :
                for cc in col_inc :

                    self.grid[row+rr,col+cc] = 1
            

