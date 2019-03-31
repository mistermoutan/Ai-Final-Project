import numpy as np
import random
from collections import defaultdict , deque
from action import *
import test_utilities as util


"""
Converts Level to Graph (only non walls cells are considered)

    Constructor:

        - Requires an instance of StateMA or StateSA
        
        The precompute parameter allows for:

                - Precompute distance from all vertices of graph to each goal
                - Precompute distance from all boxes to each goal


    Methods

     - bfs_shortestpath : Returns shortest path between a source vertex and target vertex in a deque with directions on the grid (N,S,W,E) (deques can be indexed as lists)
     - get_neighbours: Get neighbouring vertices of a particular vertex
     - backTrack: Used to return the shortest path in terms of directions, called once BFS finds a solution

"""

class Graph (object) :

    def __init__(self,state,precompute = ""):

        row,col = np.asarray(state.maze).shape
        self.vertices = {(i,j) for i in range(row) for j in range(col) if state.maze[i][j] == True}     

        #dict of shortest paths from all vertices of graph to the goal vertices
        if "all_to_goals" in precompute:
            self.shortest_path_to_goals = defaultdict(list) # goals:(vertex,shortest past from that vertex to goal)
            non_goal_vertices = [vertex for vertex in self.vertices if vertex not in state.goal_positions]
            
            for goal_position in state.goal_positions:
                for vertex in non_goal_vertices:  #for all vertices that are not goals
                    shortest_path = self.bfs_shortestpath(vertex,goal_position)
                    self.shortest_path_to_goals[goal_position].append((vertex,shortest_path))
                    
        
        # build dict of shortest path from each box to all goals
        elif "boxes_to_goals" in precompute:
            self.shortest_path_boxes_to_goals = defaultdict(list) #goals (box_position,shortest past from that vertex to goal)
            for goal_position in state.goal_positions:
                for box_positon in state.box_positions:
                    shortest_path = self.bfs_shortestpath(box_positon,goal_position)
                    self.shortest_path_boxes_to_goals[goal_position].append((box_positon,shortest_path))
        


    def get_neighbours(self,vertex):

        (x,y) = vertex
        neighbours = {(x,y+1),(x,y-1),(x-1,y),(x+1,y)}
        neighbours = {n for n in neighbours if n in self.vertices}

        return neighbours

    def path_to_directions(self,path):
        """ Turns Path composed of vertices to directions"""
    
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
        
    def backtrack(self, source_vertice, target_vertice, parent_dict):
        """Used to Return shortest path between two vertices, does it in terms of directions"""

        path = [target_vertice]
        parent = parent_dict[target_vertice]

        #get path, composed of vertices
        while parent != source_vertice:
            path.insert(0,parent) 
            parent = parent_dict[parent] 

        path.insert(0,source_vertice) #still missing the source vertex (it has no parent)
    
        directions = self.path_to_directions(path)

        return directions

    
    def bfs_shortestpath(self,source_vertice,target_vertice):
        """ Returns Shortest between two vertices in terms of S,E,W,N directions"""

        assert source_vertice in self.vertices and target_vertice in self.vertices ,  "Insert coordinates that are part of the state or not walls"
        
        #Deal with the case when the source and target of the search is the same vertex
        if source_vertice == target_vertice:
            return deque()

        queue = deque() 
        queue.append(source_vertice) 
        explored_set = set()
        self.parent = {} # vertex:parent 
        

        while len(queue) > 0:
            current_vertice = queue.popleft()

            if current_vertice == target_vertice:
                path = self.backtrack(source_vertice,target_vertice,self.parent)
                return path

            neighbours = self.get_neighbours(current_vertice)
            
            for neighbour in neighbours:
                if neighbour not in explored_set:
                    explored_set.add(neighbour)
                    self.parent[neighbour] = current_vertice
                    queue.append(neighbour)

            if len(queue) == 0:
                print("Failed to find Shortest path")
                return None

    def bfs_tree(self,source_vertice):
        """Returns complete bfs tree with source_vertex as root. May lead to congestion due to path similarity as it is used to get all shortest paths from all vertices to the source vertex """

        assert source_vertice in self.vertices, "Root/source is not part of the state"
        
        queue = deque() 
        queue.append(source_vertice) 
        explored_set = set()
        parent = {} # vertex:parent 
        
        while len(queue) > 0:
            current_vertice = queue.popleft()
            neighbours = self.get_neighbours(current_vertice)
            
            for neighbour in neighbours:
                if neighbour not in explored_set:
                    explored_set.add(neighbour)
                    parent[neighbour] = current_vertice
                    queue.append(neighbour)

            if len(queue) == 0:
                #print("Tree Complete")
                return parent

    def bfs_shortestpath_tree(self, source_vertex):  # take into account that we may encounter other desired roots only accounting for one
        """Calculates shortest path for every vertex in tree to source/root"""

        tree = self.bfs_tree(source_vertex) # vertex:parent
        shortest_path_to_source_vertex = defaultdict(list)
        parents = set(tree.values())
        leaf_nodes = [vertex for vertex in self.vertices if vertex not in parents] #nodes with no children (at bottom of tree)
        random_leaf_node = random.choice(leaf_nodes) #select one of the leaf nodes randomly to backtrack to root
        path = [random_leaf_node] 
        parent = tree[random_leaf_node]
        explored_leaf_nodes = set()
        explored_set = set()
        explored_leaf_nodes.add(random_leaf_node)

        while len(explored_leaf_nodes) != len(leaf_nodes):

            while parent != source_vertex:
                path.insert(0,parent) 
                parent = tree[parent] 

            path.insert(0,source_vertex) #still missing the source vertex (it has no parent)
            
            for index,vertex in enumerate(path): # keeps overwriting paths if vertexes are the same

                shortest_path_to_source_vertex[vertex] = self.path_to_directions(path[:index + 1]) # vertex: path as directions to source vertex
                explored_set.add(vertex)

            random_leaf_node = self.get_random(leaf_nodes,explored_leaf_nodes)
            explored_leaf_nodes.add(random_leaf_node)
            path = [random_leaf_node]
            parent = tree[random_leaf_node]

        print(shortest_path_to_source_vertex)
        return shortest_path_to_source_vertex


    def get_random(self, from_container, not_in_container):

        random_choice = random.choice(from_container)

        if random_choice in not_in_container:
            while random_choice in not_in_container:
                random_choice = random.choice(from_container)
                break
        return random_choice

            



            #add to explored
            #get the paths



    





    #def BFS_ShortestPath_Occupied()


agt0 = util.agent(0,"red")
agt1 = util.agent(1,"blue")
box0  = util.box("A", "blue")
level = [
        [True,True,True],
        [False,True,False],
        [True,True,True]
    ]

initial_state = util.make_state(level)
g = Graph(initial_state)
g.bfs_shortestpath_tree((0,0))

#tree = g.bfs_tree((2,1))
#for key,value in tree.items():
#    print(key,value)





                

            






        








