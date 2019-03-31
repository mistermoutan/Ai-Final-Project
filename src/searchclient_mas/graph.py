import numpy as np
from collections import deque
from action import *
from collections import defaultdict

"""
Converts Level to Graph (only non walls cells are considered)

    Constructor:

        - Requires an instance of StateMA or StateSA
        
        The precompute parameter allows for:

                - Precompute distance from all vertices of graph to each goal
                - Precompute distance from all boxes to each goal


    Methods

     - BFS_ShortestPath : Returns shortest path between a source vertex and target vertex in a deque with directions on the grid (N,S,W,E) (deques can be indexed as lists)
     - getNeighbours: Get neighbouring vertices of a particular vertex
     - BackTrack: Used to return the shortest path in terms of directions, called once BFS finds a solution

"""
class Graph (object) :

    def __init__(self,state,precompute = None):

        row,col = np.asarray(state.maze).shape
        self.vertices = {(i,j) for i in range(row) for j in range(col) if maze[i][j] == True}     

        #dict of shortest paths from all vertices of graph to the goal vertices
        if "all_to_goals" in precompute:
            self.shortest_path_to_goals = defaultdict(list) # goals:(vertex,shortest past from that vertex to goal)

            for goal_position in state.goal_positions:
                for vertex in self.vertices if vertex not in self.goals:  #for all vertices that are not goals
                    shortest_path = self.BFS_ShortestPath(vertex,goal_position)
                        self.shortest_path_to_goals[goal_position].append((vertex,shortest_path))
                    
        
        # build dict of shortest path from each box to all goals
        elif "boxes_to_goals" in precompute:
            self.shortest_path_boxes_to_goals = defaultdict(list) #goals (box_position,shortest past from that vertex to goal)

            for goal_position in state.goal_positions:
                for box_positon in state.box_positions:
                    shortest_path = self.BFS_ShortestPath((box_positon,goal_position))
                        self.shortest_path_boxes_to_goals[goal_position].append((box_positon,shortest_path))

    def getNeighbours(self,vertex):

        (x,y) = vertex
        neighbours = {(x,y+1),(x,y-1),(x-1,y),(x+1,y)}
        neighbours = {n for n in neighbours if n in self.vertices}

        return neighbours

    def Backtrack (self, source_vertice, target_vertice, parent_dict):
        """Used to Return shortest path in terms of directions"""

        path = [target_vertice]
        parent = parent_dict[target_vertice]

        #get path, composed of vertices
        while parent != source_vertice:
            path.insert(0,parent) 
            parent = parent_dict[parent] 

        #still missing the source vertex (it has no parent)
        path.insert(0,source_vertice)

        directions = deque()
        for i in range(len(path) - 1):

            subtract = tuple(np.subtract(path[i+1],path[i]))

            if subtract == (1,0):
                direction = Dir.S
            elif subtract == (-1,0):
                direction = Dir.N
            elif subtract == (0,1):
                direction = Dir.E
            elif subtract == (0,-1):
                direction = Dir.W
            
            directions.append(direction)

        return directions

    def BFS_ShortestPath (self,source_vertice,target_vertice):
        """ Returns Shortest in terms of S,E,W,N directions"""

        assert source_vertice in self.vertices and target_vertice in self.vertices, "Insert coordinates that are part of the state"
        
        queue = deque() 
        queue.append(source_vertice) 
        explored_set = set()
        self.parent = {} # neighbours:parent 
        

        while len(queue) > 0:
            current_vertice = queue.popleft()

            if current_vertice == target_vertice:
                path = self.Backtrack(source_vertice,target_vertice,parent)
                print(path)
                return path

            neighbours = self.getNeighbours(current_vertice)
            
            for neighbour in neighbours:
                if neighbour not in explored_set:
                    explored_set.add(neighbour)
                    parent[neighbour] = current_vertice
                    queue.append(neighbour)

            if len(queue) == 0:
                print("Failed to find Shortest path")
                return None



            

"""
maze = [[True,True,False,True],
        [True,True,True,True],
        [True,False,False,True],
        [True,False,True,True]]



g = Graph(maze)
g.BFS_ShortestPath((3,2),(3,0))

"""





                

            






        








