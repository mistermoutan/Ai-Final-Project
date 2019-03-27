import state
import numpy as np
from collections import deque

"""
Converts Level to Graph (only non walls cells are considered)

    Methods
     - BFS_ShortestPath : Returns shortest path between a source vertice and target vertice in a deque with directions on the grid (N,S,W,E) (deques can be indexed as lists)
     - getNeighbours: Get neighbouring vertices of a particular vertice
     - BackTrack: Used to return the shortest path in terms of directions, called once BFS finds a solution

"""
class Graph (object) :

    def __init__(self,maze):

        row,col = np.asarray(maze).shape
        self.vertices = [(i,j) for i in range(row) for j in range(col) if maze[i][j] == True]      #vertices of graph representation of level

    def getNeighbours(self,vertice):

        (x,y) = vertice
        neighbours = [(x,y+1),(x,y-1),(x-1,y),(x+1,y)]
        neighbours = [n for n in neighbours if n in self.vertices]

        return neighbours

    def Backtrack (self, source_vertice, target_vertice, parent_dict):
        """Used to Return shortest path in terms of directions"""

        path = [target_vertice]
        parent = parent_dict[target_vertice]

        #get path, composed of vertices
        while parent != source_vertice:
            path.insert(0,parent) # appent parent to beggining of path array
            parent = parent_dict[parent]  # get next parent

        #still missing the source vertice (it has no parent)
        path.insert(0,source_vertice)

        #turn path into directions:
        directions = deque()
        for i in range(len(path) - 1):

            subtract = tuple(np.subtract(path[i+1],path[i]))

            if subtract == (1,0):
                direction = "S"
            elif subtract == (-1,0):
                direction = "N"
            elif subtract == (0,1):
                direction = "E"
            elif subtract == (0,-1):
                direction = "W"
            
            directions.append(direction)

        return directions

    def BFS_ShortestPath (self,source_vertice,target_vertice):
        """ Returns Shortest in terms of  S,E,W,N directions"""

        assert source_vertice in self.vertices and target_vertice in self.vertices, "Insert coordinates that are part of the level"
        
        queue = deque() 
        queue.append(source_vertice) #adds to the right
        explored_set = set()
        parent = {} #children/neighbours as keys and their parent vertice as values
        

        while len(queue) > 0:
            current_vertice = queue.popleft()

            if current_vertice == target_vertice:
                path = self.Backtrack(source_vertice,target_vertice,parent)
                return path

            neighbours = self.getNeighbours(current_vertice)
            
            for neighbour in neighbours:
                if neighbour not in explored_set:
                    explored_set.add(neighbour)
                    parent[neighbour] = current_vertice
                    queue.append(neighbour)

            if len(queue) == 0:
                print("Failed to find Shortest path, make sure there is a path between the source and target vertice")
                return



            
"""

maze = [[True,True,False,True],
        [True,True,True,True],
        [True,False,False,True],
        [True,False,True,True]]

g = Graph(maze)
g.BFS_ShortestPath((3,2),(3,0))

"""





                

            






        








