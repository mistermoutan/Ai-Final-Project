import numpy as np
import random
from collections import defaultdict , deque
import copy
from action import *
import test_utilities as util


"""
Converts Level to Graph 

    Constructor:
        - Requires an instance of StateMA or StateSA

        The precompute parameter allows for:
                - Precompute bfs_trees for goals or boxes. 

    Methods

    - run_bfs : run_bfs for a particular vertex as root therefore building its bfs tree -> {vertex:parent}
    - bfs_tree: used to select tree with certain vertex as root. If tree is not buit, builds it. Returns tree
    - shortest_path_between: returns shortest path between source_vertex and target_vertex by using the bfs_tree that has target_vertex as root
    - bfs_shortestpath_no_tree : returns shortest path between source_vertex and target_vertex without having a bfs_tree pre built or storing it in self.bfs_trees. May come in handy

"""

class Graph (object) :

    def __init__(self,maze,precompute = ""):

        self.bfs_trees = {}  # store bfs trees from specific root vertices
        row,col = np.asarray(state.maze).shape
        self.vertices = {(i,j) for i in range(row) for j in range(col) if maze[i][j]}
      
        #precompute bfs_trees with goals as root
        if "goal_trees" in precompute:
            for goal in state.goal_positions:
                self.bfs_tree(goal)

        #precompute bfs_trees with box positions as root
        elif "box_trees" in precompute:
            for box in state.box_positions:
                self.bfs_tree(box)

            
    def run_bfs(self,source_vertex,cutoff_vertex=None,cutoff_branch = None): #add self.bfs_trees_cut?
        """
        Builds complete bfs tree with source_vertex as root, adds it so self.bfs_trees. 
        Tree is in the foar of a dictionary structured in the following way: {vertex:(parent)}
        A cutoff_vertex may be passed in order to stop building the tree once that vertex is reached.
        May lead to congestion due to path similarity as it is used to get shortest paths from vertices to the source/root vertex
        """
        assert source_vertex in self.vertices, "Root/source is not part of the state"
        #if tree is already built
        if source_vertex in self.bfs_trees:
            return
        
        queue = deque([source_vertex]) 
        explored_set = set([source_vertex])
        parent = {} # {vertex:(parent,(path to root in terms of directions)}

        while queue:
            current_vertex = queue.popleft()
            if current_vertex == cutoff_vertex:
                break
            #filter out explored neighbours
            unexplored_neighbours = [v for v in self.get_neighbours(current_vertex) if not v in explored_set]  
            #mark them as explored
            explored_set.update(unexplored_neighbours)
            #add them to queue
            queue.extend(unexplored_neighbours)
            #Add a new entry to the dictionary for each neighbour, potining to the current 
            #node as their parent
            node_parent_pairs = [(v,current_vertex) for v in unexplored_neighbours]
            parent.update(node_parent_pairs)
        
        self.bfs_trees[source_vertex] = parent

    def bfs_tree(self,source_vertex):
        """Checks if tree with source_vertex as root is in self.bfs_trees, if not builds the tree, returns tree"""
        assert source_vertex in self.vertices
        #if tree is not built, build it and store it
        if source_vertex not in self.bfs_trees:
            self.run_bfs(source_vertex)    
        return  self.bfs_trees[source_vertex]      
 
    def shortest_path_between(self,source_vertex,target_vertex):
        """
        Returns shortest path between two vertices using bfs tree, if the tree with target_vertice as root is not built, builds it and adds it to self.bfs_trees
        If there is no path between the vertices returns None
        """
        assert target_vertex in self.vertices #source_vertex is already checked in self.bfs_tree
        if target_vertex == source_vertex:
            return deque()
        tree = self.bfs_tree(source_vertex)
        path = self.reconstruct_path_from_root_in_bfs_tree_to_target(tree,target_vertex)
        return path

    def reconstruct_path_from_root_in_bfs_tree_to_target(self, tree, target_vertex):
        """Not to be called directly"""
        path = deque()
        current_vertex = target
        while current_vertex:
            path.appendleft(current_vertex)
            current_vertex = tree.get(current_vertex,None) #get parent of current_vertex, none if it's not in keys (the source_vertex)
        return path


    def bfs_shortestpath_notree(self,source_vertex,target_vertex):
        """ 
        Returns Shortest path between two vertices without having a tree pre built or building one and storing it in self.bfs_trees
        If there is no path between the two vertices, returns None. May be useful if memory/time problems arise.
        """
        assert source_vertex in self.vertices and target_vertex in self.vertices,  "Insert coordinates that are part of the state or not walls"
        if source_vertex == target_vertex:
            return deque()

        queue = deque([source_vertex]) 
        explored_set = set(source_vertex)
        parent = {} # vertex:parent 

        while queue:
            current_vertex = queue.popleft()
            if current_vertex == target_vertex:
                path = self.backtrack(source_vertex,target_vertex,parent)
                return path

        #filter out explored neighbours
        unexplored_neighbours = [v for v in self.get_neighbours(current_vertex) if not v in explored_set]  
        #mark them as explored
        explored_set.update(unexplored_neighbours)
        #add them to queue
        queue.extend(unexplored_neighbours)
        #Add a new entry to the dictionary for each neighbour, potining to the current 
        #node as their parent
        node_parent_pairs = [(v,current_vertex) for v in unexplored_neighbours]
        parent.update(node_parent_pairs)

        return None #if no path is found


    def backtrack(self, source_vertex, target_vertex, parent_dict):
        """Used to Return shortest path between two vertices,  in terms of directions, used in bfs_shortestpath_notree"""

        path = [target_vertex]
        parent = parent_dict[target_vertex]
        #get path, composed of vertices
        while parent != source_vertex:
            path.insert(0,parent) 
            parent = parent_dict[parent] 
        path.insert(0,source_vertex) #still missing the source vertex (it has no parent)
        return path


    ##########################################################
    ###############           UTILS               ############
    ##########################################################

    def get_neighbours(self,vertex,in_vertices = True):
        """Returns neighbours as set"""

        (x,y) = vertex
        neighbours = {(x,y+1),(x,y-1),(x-1,y),(x+1,y)}
        if in_vertices:
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
            else:
                raise ValueError("Vertices in path are not adjacent")

            directions.append(direction)
        return directions

    def direction_between_two_adjacent_vertices(self,_from,to):

        subtract = tuple(np.subtract(_from,to))

        if subtract == (1,0):
            direction = Dir.S
        elif subtract == (-1,0):
            direction = Dir.N
        elif subtract == (0,1):
            direction = Dir.E
        elif subtract == (0,-1):
            direction = Dir.W
        else:
            raise ValueError("Vertices are not adjacent")

        return direction

    def deep_copy(self,x):
        return copy.deepcopy(x)  
     


#def trim_tree()
    
#def occupy_vertices_in_tree(self,tree,vertices_to_occupy): #{vertex:parent,path}


     
    


            



"""
agt0 = util.agent(0,"red")
agt1 = util.agent(1,"blue")
box0  = util.box("A", "blue")
level = [
        [True,False,True,False,True],
        [False,True,False,True,False],
        [True,True,True,False,True]
    ]





                

            






        








"""