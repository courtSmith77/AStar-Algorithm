
class Node() :

    def __init__(self,row,col,heur=None,g=None,total=None,parent=None) :
        self.parent = parent
        self.row = row
        self.col = col
        self.h = heur
        self.g = g
        self.t = total
    
    def __eq__(self, other):
        return self.row == other.row and self.col == other.col