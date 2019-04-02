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

    - bfs_tree : builds bfs_tree for a particular vertex as root, this trees have the shortest path (in terms of directions on the grid) from each vertice to root stored. -> {vertex:(parent,[path_from_vertex_to_root])}
    - shortest_path_between: returns shortest path between source_vertex and target_vertex by using the bfs_tree that has target_vertex as root
    - bfs_shortestpath_no_tree : returns shortest path between source_vertex and target_vertex without having a bfs_tree pre built or storing it in self.bfs_trees. May come in handy

"""

class Graph (object) :

    def __init__(self,state,precompute = ""):

        self.bfs_trees = {}  # store bfs trees from specific root vertices
        row,col = np.asarray(state.maze).shape
        self.vertices = set()
        self.walls = set()
        for i in range(row):
             for j in range(col):
                if state.maze[i][j] == True:
                    self.vertices.add((i,j))
                else:
                    self.walls.add((i,j))
      
        #precompute bfs_trees with goals as root
        if "goal_trees" in precompute:
            for goal in state.goal_positions:
                self.bfs_tree(goal)

        #precompute bfs_trees with box positions as root
        elif "box_trees" in precompute:
            for box in state.box_positions:
                self.bfs_tree(box)

            
    def bfs_tree(self,source_vertex,cutoff_vertex=None,cutoff_branch = None): #add self.bfs_trees_cut?
        """
        Builds complete bfs tree with source_vertex as root, adds it so self.bfs_trees. 
        Tree is a dictionary structured in the following way: {vertex:(parent,(path to root in terms of directions)}
        A cutoff_vertex may be passed in order to stop building the tree once that vertex is reached. That vertex is still stored
        If the source_vertex has no neighbours (tree can't be built) returns None.
        May lead to congestion due to path similarity as it is used to get shortest paths from vertices to the source/root vertex
        """
        assert source_vertex in self.vertices, "Root/source is not part of the state"
        #if tree is already built
        if source_vertex in self.bfs_trees:
            return
        
        queue = deque() 
        queue.append(source_vertex) 
        explored_set = set()
        explored_set.add(source_vertex)
        parent = {} # {vertex:(parent,(path to root in terms of directions)}
        parent[source_vertex] = None

        while len(queue) > 0:
            current_vertex = queue.popleft()
            neighbours = self.get_neighbours(current_vertex)
            if neighbours:
                for neighbour in neighbours:
                    if neighbour not in explored_set:
                        explored_set.add(neighbour)
                        queue.append(neighbour)
                        direction = self.direction_between_two_adjacent_vertices(current_vertex,neighbour)
                        if parent[current_vertex]: # if the parent is not the root, append the direction to the path that already exists
                            path_until_now = self.deep_copy(parent[current_vertex][1]) 
                            path_until_now.insert(0,direction)
                            parent[neighbour] = (current_vertex,path_until_now)
                        else: # for the children of the root
                            parent[neighbour] = (current_vertex,[direction])
                        if cutoff_vertex == neighbour: # exit inner loop (stop building tree)
                            break
                if cutoff_vertex == neighbour: #exit outter loop
                    break
            else:
                parent = None
                break


        #if it is not possible to build the tree from source_vertex (it has no neighbours)
        #if len(parent.keys()) == 1:
        #    parent = None
        
        self.bfs_trees[source_vertex] = parent


    def shortest_path_between(self,source_vertex,target_vertex):
        """
        Returns shortest path between two vertices using bfs tree, if the tree with target_vertice as root is not built, builds it and adds it to self.bfs_trees
        If there is no path between the vertices returns None
        """
        assert source_vertex in self.vertices and target_vertex in self.vertices, "Insert coordinates that are part of the state or not walls"

        if target_vertex == source_vertex:
            return deque()
        if target_vertex not in self.bfs_trees:
            self.bfs_tree(target_vertex)

        tree = self.bfs_trees[target_vertex]

        if tree:
            return self.shortest_path_from_bfs_tree(tree,source_vertex)
        else:
            return tree

    def shortest_path_from_bfs_tree(self, tree, source):
        """
        Returns shortest path from source to root of tree, not to be called directly
        """
        return tree[source][1]


    def bfs_shortestpath_notree(self,source_vertex,target_vertex):
        """ 
        Returns Shortest between two vertices in terms of S,E,W,N directions without having a tree pre built or building one and storing it in self.bfs_trees
        If there is no path between the two vertices, returns None. May be useful if memory/time problems arise.
        """
        assert source_vertex in self.vertices and target_vertex in self.vertices ,  "Insert coordinates that are part of the state or not walls"

        if source_vertex == target_vertex:
            return deque()

        queue = deque() 
        queue.append(source_vertex) 
        explored_set = set()
        self.parent = {} # vertex:parent 
        
        while len(queue) > 0:
            current_vertex = queue.popleft()

            if current_vertex == target_vertex:
                path = self.backtrack(source_vertex,target_vertex,self.parent)
                return path

            neighbours = self.get_neighbours(current_vertex)
            
            for neighbour in neighbours:
                if neighbour not in explored_set:
                    explored_set.add(neighbour)
                    self.parent[neighbour] = current_vertex
                    queue.append(neighbour)

            if len(queue) == 0:
                return None


    def backtrack(self, source_vertex, target_vertex, parent_dict):
        """Used to Return shortest path between two vertices,  in terms of directions, used in bfs_shortestpath_notree"""

        path = [target_vertex]
        parent = parent_dict[target_vertex]

        #get path, composed of vertices
        while parent != source_vertex:
            path.insert(0,parent) 
            parent = parent_dict[parent] 

        path.insert(0,source_vertex) #still missing the source vertex (it has no parent)
        directions = self.path_to_directions(path)
        return directions


    ##########################################################
    ###############           UTILS               ############
    ##########################################################


    def get_neighbours(self,vertex,in_vertices = True):
        """Returns neighbours, returns none if there are no neighbours"""

        (x,y) = vertex
        neighbours = {(x,y+1),(x,y-1),(x-1,y),(x+1,y)}

        if in_vertices:
            neighbours = {n for n in neighbours if n in self.vertices}
        if len(neighbours) == 0:
            return None
        return neighbours

    def get_vertical_neighbours(self,vertex):
        """Returns vertical neighbours, none if there are no neighbours"""

        (x,y) = vertex
        vertical_neighbours = {(x,y+1),(x,y-1)}
        vertical_neighbours = {n for n in vertical_neighbours if n in self.vertices}
        if len(neighbours) == 0:
            return None
        return vertical_neighbours

    def get_horizontal_neighbours(self,vertex):
        """Returns horizontal neighbours, none if there are no neighbours"""

        (x,y) = vertex
        horizontal_neighbours = {(x,y+1),(x,y-1)}
        horizontal_neighbours = {n for n in horizontal_neighbours if n in self.vertices}
        if len(neighbours) == 0:
            return None
        return horizontal_neighbours

    def number_neighbouring_walls_of_vertex(self,vertex):
        """Returns amount of neighbours of partical vertex that are walls (min:0 ; max:4) """

        #self.number_neighbouring_walls_of_vertex = {}
        neighbours = self.get_neighbours(vertex, in_vertices = None)
        n_neighbouring_walls = 0
        if neighbours:
            for neighbour in neighbours:
                if neighbour in self.walls:
                    n_neighbouring_walls += 1

            assert n_neighbouring_walls >= 0 and n_neighbouring_walls <= 4, "Neighbouring walls under 0 or over 4"
            return n_neighbouring_walls
        else:
            return n_neighbouring_walls
    
        #self.number_neighbouring_walls_of_vertex[vertex] = n_neighbouring_walls


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

    def sum_len_elements_of_list (self,list_):
        lenght = 0
        for element in list_:
            lenght += len(element)
        return lenght      

    def from_not_in (self, from_container, not_in_containers):
        """Get an element in from_container that isn't in any of the not_in_containers,
        Returns None if not possibe to do so"""

        explored_elements = set()
        for element in from_container:
            for not_in_container in not_in_containers:
                if element in not_in_container:
                    explored_elements.add(element)
                    break
            if element not in explored_elements:
                return element
        
        if len(explored_elements) == len(from_container):
            return None
        else:
            print("I fucked up somewhere")

    def tree_of_vertex(self,vertex):
        """Checks if vertex is in self.bfs_trees, if not builds the tree, returns pointer to tree"""
        if vertex not in self.bfs_trees:
            self.bfs_tree(vertex)    
        tree = self.bfs_trees[vertex]      

        return tree 

#def trim_tree()
    
#def occupy_vertices_in_tree(self,tree,vertices_to_occupy): #{vertex:parent,path}


     
    


            



agt0 = util.agent(0,"red")
agt1 = util.agent(1,"blue")
box0  = util.box("A", "blue")
level = [
        [True,False,True,False,True],
        [False,True,False,True,False],
        [True,True,True,False,True]
    ]

initial_state = util.make_state(level)
g = Graph(initial_state)
g.separate_rooms_exist()
g.locate_separate_rooms()
#g.locate_corridors(3)
g.number_neighbouring_walls_of_vertex((1,1))





                

            






        








