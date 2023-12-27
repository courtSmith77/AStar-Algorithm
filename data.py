import numpy as np

class Landmarks():

    def __init__(self):

        self.file = np.loadtxt("./ds1_Landmark_Groundtruth.dat")
        self.subjects = self.file[:,0]
        self.x_pos = self.file[:,1]
        self.y_pos = self.file[:,2]