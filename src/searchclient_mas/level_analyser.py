import numpy as np
import random
from collections import defaultdict , deque
import copy
from action import *
import test_utilities as util
import utilities as u
from graph import Graph
from itertools import groupby

class LevelAnalyser:

    def __init__(self,state):

        self.bfs_trees = {}
        self.explored = {}
        self.vertices = set()
        self.walls = set()
        self.rooms = None
        row,col = np.asarray(state.maze).shape
        for i in range(row):
             for j in range(col):
                if state.maze[i][j]:
                    self.vertices.add((i,j))
                else:
                    self.walls.add((i,j))

    def run_bfs(self,source_vertex,cutoff_vertex=None,save_explored=None): #add self.bfs_trees_cut?
        """
        Builds complete bfs tree with source_vertex as root, adds it so self.bfs_trees. 
        Tree is in the form of a dictionary structured in the following way: {vertex:(parent)}
        A cutoff_vertex may be passed in order to stop building the tree once that vertex is reached.
        May lead to congestion due to path similarity as it is used to get shortest paths from vertices to the source/root vertex
        """
        assert source_vertex in self.vertices
        if source_vertex in self.bfs_trees: #if tree is already built
            return
        
        queue = deque([source_vertex]) 
        explored_set = set([source_vertex])
        parent = {} # {vertex:(parent,(path to root in terms of directions)}
        parent[source_vertex] = None #this is different from the graph class! Facilitates some of this methods

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
        if save_explored:
            self.explored[source_vertex] = explored_set

    def bfs_tree(self,source_vertex):
        """Checks if tree with source_vertex as root is in self.bfs_trees, if not builds the tree and stores it in self.bfs_trees, returns tree"""
        assert source_vertex in self.vertices
        #if tree is not built, build it and store it
        if source_vertex not in self.bfs_trees:
            self.run_bfs(source_vertex)    
        return self.bfs_trees[source_vertex]      
 
    def separate_rooms_exist(self):
        """True if there are separate rooms (isolated parts of the level), False otherwise"""

        initial_vertex = self.vertices.pop() 
        self.vertices.add(initial_vertex) 
        tree = self.bfs_tree(initial_vertex) 
        return len(self.vertices) != len(tree.keys())

    def locate_separate_rooms(self):
        """Creates self.rooms: list containing each separate room as sets"""

        #assert self.separate_rooms_exist(), "There are no isolated rooms"

        initial_vertex = self.vertices.pop() 
        self.vertices.add(initial_vertex)
        tree = self.bfs_tree(initial_vertex) 
        vertices_list = list(self.vertices) #will need to iterate
        rooms = []

        #initialize first room
        initial_room = {(i,j) for i,j in tree.keys()}
        rooms.append(initial_room)

        #so while we haven't accounted for all vertices to be in their room
        while self.sum_len_elements_of_list(rooms) != len(self.vertices): 
            vertex_not_in_initial_room = self.from_not_in(vertices_list,rooms)
            tree = self.bfs_tree(vertex_not_in_initial_room) 
            new_room = []
            new_room = {(i,j) for i,j in tree.keys()}
            rooms.append(new_room)
            new_room = {}

        self.rooms = rooms
 
    def locate_corridors(self):
        """Returns list of all corridors for each room 
        For now it is also returning edges (vertices) eg:corners of square rooms """

        self.locate_separate_rooms()
        self.corridors = {}
        number_of_rooms = len(self.rooms)
        for room_index in range(number_of_rooms):
            corridor_vertices = self.corridor_vertices_of_room(self.rooms[room_index])
            corridors_in_room = self.break_container_into_adjacent_vertices(corridor_vertices)
            self.corridors[room_index] = corridors_in_room
            print(self.corridors)
            #print("In room {0}, the corridors are: {1}" .format(room_index,corridors_in_room))        
    
    #def room_is_corridor(self):


    ##########################################################
    ###############           UTILS               ############
    ##########################################################

    def break_container_into_adjacent_vertices(self,container):
        """Goes through vertices in container and returns list of deques with vertices grouped with adjacent vertices"""

        list_of_deques = []
        while container:
            current_vertex = container.pop()
            adjacent_to_current_vertex = self.find_adjacent_vertices_in_container(current_vertex,container) 
            adjacent_to_current_vertex.update([current_vertex]) #we'll say the current vertex is adjacent to himself because we use this for determining corridors
            container.difference_update(adjacent_to_current_vertex)
            list_of_deques.append(adjacent_to_current_vertex)
        return list_of_deques

    def find_adjacent_vertices_in_container(self,vertex,container):
        """Find adjacent vertices of adjacent vertices and so on for a particular vertex in a container"""

        explored_set = set([vertex])
        adjacent_to_vertex = {v for v in container if u.are_adjacent(vertex,v)}
        non_explored = {v for v in adjacent_to_vertex if v not in explored_set}
        while non_explored:
            vertex = non_explored.pop()
            explored_set.add(vertex)
            adjacent_to_an_adjacent = {v for v in container if u.are_adjacent(vertex,v)}
            adjacent_to_vertex.update(adjacent_to_an_adjacent)
            non_explored = {v for v in adjacent_to_vertex if v not in explored_set}
        return adjacent_to_vertex

    def corridor_vertices_of_room(self,room):
        """Returns all vertices in a room which are part of corridors"""
        corridor_vertices = {vertex for vertex in room if self.is_corridor_candidate(vertex)}
        return corridor_vertices

    def get_children_dictionary(self,parent_dictionary):
        """Turn dictionary in form children:(parent) to parent:(children)"""

        children = defaultdict(list)
        for child, parent in parent_dictionary.items():
            children[parent].append(child)
        return children

    def get_neighbours(self,vertex,in_vertices = True):
        """Returns neighbours, returns none if there are no neighbours"""

        assert vertex in self.vertices
        (x,y) = vertex
        neighbours = {(x,y+1),(x,y-1),(x-1,y),(x+1,y)}

        if in_vertices:
            neighbours = {n for n in neighbours if n in self.vertices}
        return neighbours

    def number_neighbouring_walls_of_vertex(self,vertex):
        """Returns amount of neighbours of partical vertex that are walls (min:0 ; max:4) """

        neighbours = self.get_neighbours(vertex, in_vertices = None)
        n_neighbouring_walls = 0
        neighbours_in_walls = {n for n in neighbours if n in self.walls}
        n_neighbouring_walls += len(neighbours_in_walls)
        assert n_neighbouring_walls >= 0 and n_neighbouring_walls <= 4, "Neighbouring walls under 0 or over 4"
        return n_neighbouring_walls
    
    ######################################################## neighbour_condition FIX
    def is_corridor_candidate(self,vertex):
        """For a vertex to possibly be in a corridor he should have 2 or 3 walls as neighbours, one of hs neighbours should too"""

        vertex_condition = self.corridor_vertex_condition(vertex)
        neighbours = self.get_neighbours(vertex)   
        neighbours_condition = False
        neighbours_that_verify_condition = {n for n in neighbours if self.corridor_vertex_condition(neighbours)}
        if neighbours_that_verify_condition:
            neighbours_condition = True
        return vertex_condition and neighbours_condition

    def corridor_vertex_condition(self,vertex):
         
        return 2 <= self.number_neighbouring_walls_of_vertex(vertex) <=3

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

    def get_goal_room_graph(self, goals):
        rooms = [set((goal,)) for goal in goals]
        vert_in_room = {goal:i for i,goal in enumerate(goals)}
        seen = set(goals)
        seen_edges = set()

        # TODO: move this to appropriate place
        def get_neighbours(vertex):
            """Returns neighbours as set"""
            (x, y) = vertex
            neighbours = {(x, y + 1), (x, y - 1), (x - 1, y), (x + 1, y)}
            return neighbours

        edges = []
        for i, g in enumerate(goals):
            # for every goal we will check adjacent squares and if we haven't seen them before they are now a new room
            # which we grow with dfs
            neighbors = get_neighbours(g)
            for n in neighbors:
                if n in seen:
                    edge = (i, vert_in_room[n])
                    if not(edge in seen_edges or (vert_in_room[n],i) in seen_edges):
                        edges.append(edge)
                        seen_edges.add(edge)
                if n in self.vertices and n not in seen:
                    idx = len(rooms)
                    room = set((n,))
                    rooms.append(room)
                    # TODO: use bfs instead of dfs? (only necessary if we wanna get some min distances between nodes)
                    stack = [n]
                    while stack:
                        curr = stack.pop()
                        if curr in seen and vert_in_room[curr] != idx:
                            edge = (vert_in_room[curr], idx)
                            edges.append(edge)
                            seen_edges.add(edge)
                        elif curr not in seen:
                            if curr not in self.vertices:
                                continue
                            seen.add(curr)
                            room.add(curr)
                            vert_in_room[curr] = idx

                            stack.extend(get_neighbours(curr))
        # TODO: use different edge representation?
        # TODO: get some extra info?
        return rooms, vert_in_room, edges



def test_get_goal_room_graph():
    import test_utilities as tu

    maze = tu.create_maze()
    goal = tu.goal('a')

    for i in range(len(maze)-2):
        maze[i][4] = False
        maze[i][6] = False

    maze[8][4] = goal
    maze[8][5] = goal
    maze[8][6] = goal
    maze[5][5] = goal

    state = tu.make_state(maze)
    analyzer = LevelAnalyser(state)
    rooms, room_by_cord, edges = analyzer.get_goal_room_graph(state.goal_positions)

    if len(rooms) != 8:
        print("number of rooms is wrong")
        return False
    if len(edges) != len(rooms)-1:
        print("number of edges is not correct")
        return False
    spaces = 0
    for r in rooms:
        for _ in r:
            spaces += 1

    if spaces != sum([sum(i) for i in state.maze]):
        print("number of spaces in rooms are not equal to actual number of spaces")
        return False
    return True


if __name__ == '__main__':
    if not test_get_goal_room_graph():
        print("Test failed")

#def locate_high_density_areas:

#def locate_high_density_clustered_goals_in_corridor:
    
#def locate_complicated_corners:

#class Goal_Rooms_Tree:




agt0 = util.agent(0,"red")
agt1 = util.agent(1,"blue")
box0  = util.box("A", "blue")
level = [
        [False, False,False, False,False,False, False,False, False,False],
        [False, True,False, True,False,False, False,False, True,False],
        [False, True,True, True,False,True, True,True, False,False],
        [False, True,True, True,False,True, True,True, False,False],
        [False, True,False, True,False,False, False,False, False,False],
        [False, False,False, False,False,False, False,False, False,False]
    ]

initial_state = util.make_state(level)

L = LevelAnalyser(initial_state)
L.separate_rooms_exist()
#L.locate_corridors()

