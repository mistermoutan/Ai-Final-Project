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

    def __init__(self,is_vertex_matrix):

        self.bfs_trees = {}  # store bfs trees from specific root vertices
        self.bfs_trees_vertices = {}
        row,col = np.asarray(is_vertex_matrix).shape
        self.vertices = set([(r,c) for r in range(row) for c in range(col) if is_vertex_matrix[r][c]])
    

    #Runs a BFS searc in the graph from source, and returns the BFS tree 
    #in terms of a dictionary where every vertex maps to its parent in the BFS tree
    #The root of the tree has no parent / does not occur as a key in the dictionary
    def run_bfs(self, source, cutoff_vertex=None):
        assert source in self.vertices
        #Initialize variables
        queue = deque([source])
        explored_set = set([source])
        parent = {}

        while queue:
            current_vertex = queue.popleft()
            #If there is a specific vertex we're looking for, we've found it 
            #and need look no further
            if current_vertex == cutoff_vertex:
                break
            #Filter out explored neighbours
            unexplored_neighbours = [v for v in self.get_neighbours(current_vertex) if not v in explored_set]
            #Marked neighbours as explored
            explored_set.update(unexplored_neighbours)
            #All all neighbours to queue
            queue.extend(unexplored_neighbours)
            #Add a new entry to the dictionary for each neighbour, potining to the current 
            #node as their parent
            node_parent_pairs = [(v,current_vertex) for v in unexplored_neighbours]
            parent.update(node_parent_pairs)
        
        return parent

    def build_bfs_tree_if_it_doesnt_exist(self, source):
        #Check that source is valid
        assert source in self.vertices
        #Exit if BFS tree for source has been made before
        if source in self.bfs_trees_vertices:
            return self.bfs_trees_vertices[source]
        #Compute the BFS tree and store it
        self.bfs_trees_vertices[source] = self.run_bfs(source)
        #Return the newly computed BFS tree
        return self.bfs_trees_vertices[source]

    #Given a BFS tree and a target vertex, returns the path from the root in that BFS 
    #tree to the target vertex
    def reconstruct_path_from_root_in_bfs_tree_to_target(self, tree, target):
        path = deque()
        current = target
        while current:
            path.appendleft(current)
            current = tree.get(current, None)
        return path
    
    def shortest_path_between(self,source_vertex,target_vertex):
        """
        Returns shortest path between two vertices using bfs tree, if the tree with target_vertice as root is not built, builds it and adds it to self.bfs_trees
        If there is no path between the vertices returns None
        """
        bfs_tree = self.build_bfs_tree_if_it_doesnt_exist(source_vertex)
        path = self.reconstruct_path_from_root_in_bfs_tree_to_target(bfs_tree, target_vertex)
        return path


    def bfs_shortestpath_notree(self,source_vertex,target_vertex):
        """ 
        Returns Shortest between two vertices in terms of vertices on the path without having a tree pre built or building one and storing it in self.bfs_trees
        If there is no path between the two vertices, returns None. May be useful if memory/time problems arise.
        """
        assert source_vertex in self.vertices and target_vertex in self.vertices ,  "Insert coordinates that are part of the state or not walls"
        tree = self.run_bfs(source_vertex)
        return self.reconstruct_path_from_root_in_bfs_tree_to_target(tree, target_vertex)

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

    def number_of_neighbours(self, vertex):
        return len(self.get_neighbours(vertex))

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
            self.build_bfs_tree_if_it_doesnt_exist(vertex)
        tree = self.bfs_trees[vertex]      

        return tree 

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

initial_state = util.make_state(level)
g = Graph(initial_state)
g.separate_rooms_exist()
g.locate_separate_rooms()
#g.locate_corridors(3)
g.number_neighbouring_walls_of_vertex((1,1))
"""
