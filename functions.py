
def world_to_grid(x,y,x_range,y_range,cell_size) :
    column = int((x - x_range[0])/cell_size)
    row = int((y - y_range[0])/cell_size)
    return row, column

def grid_to_world(row,column,x_range,y_range,cell_size) :
    x = column*cell_size + x_range + cell_size/2
    y = row*cell_size + y_range + cell_size/2
    return x, y