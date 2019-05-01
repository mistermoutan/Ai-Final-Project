import numpy as np
import random
from collections import defaultdict , deque
import copy
from action import *
import test_utilities as util
import utilities as u
from graph import Graph
from itertools import groupby
from collections import Counter
from state import StateMA,StateBuilder
import time

class LevelAnalyser:

    def __init__(self,state: StateMA):
        self.bfs_trees = {}
        self.vertices = set()
        self.walls = set()
        self.goal_positions = {goal_pos for goal_pos in state.goal_positions}
        self.box_positions = {box_pos for box_pos in state.box_positions} 
        self.agent_positions = [agent_pos for agent_pos in state.agent_positions] #agents are identified by the order their positions appear in the array
        self.rooms = None #list
        self.corridors = None
        self.open_areas = None
        self.goals_per_room = None
        self.boxes_per_room = None
        self.agents_per_room = None
        self.safe_storage = None
        
        row,col = np.asarray(state.maze).shape
        for i in range(row):
             for j in range(col):
                if state.maze[i][j]:
                    self.vertices.add((i,j))
                else:
                    self.walls.add((i,j))

 
    def separate_rooms_exist(self):
        '''True if there are separate rooms (isolated parts of the level), False otherwise'''
        return self.is_connected_component(self.vertices)

    def locate_separate_rooms(self):
        '''Creates self.rooms: list containing each separate room as sets, this sets are all the vertices that correspond to a room'''

        #assert self.separate_rooms_exist(), "There are no isolated rooms"
        if self.rooms: #rooms already built
            return 
        initial_vertex = self.vertices.pop() 
        self.vertices.add(initial_vertex)
        tree = self.bfs_tree(initial_vertex) 
        vertices_list = list(self.vertices) #will need to iterate
        rooms = []

        #initialize first room
        initial_room = {(i,j) for i,j in tree.keys()}
        rooms.append(initial_room)

        #while we haven't accounted for all vertices to be in their room
        while self.sum_len_elements_of_list(rooms) != len(self.vertices): 
            vertex_not_in_initial_room = self.from_not_in(vertices_list,rooms)
            tree = self.bfs_tree(vertex_not_in_initial_room) 
            new_room = []
            new_room = {(i,j) for i,j in tree.keys()}
            rooms.append(new_room)
            new_room = {}

        self.rooms = rooms

    def get_agent_distribution_per_room(self):
        """More efficient way of doing this? depends on the balance of nr of rooms vr nr of agents
        Builds self.agents_per_room dictionary ->  room: agents in room
        """
        self.locate_separate_rooms()
        if self.agents_per_room:
            return
        self.agents_per_room = {} # room: agents in room
        accounted_for_agents = set()

        for room_index , room in enumerate(self.rooms):
            if len(accounted_for_agents) != len(self.agent_positions): # if there are stills agents that are not assigned to a room
                agents_in_room = set()
                unnacounted_for_agents = {agent for agent in range(len(self.agent_positions)) if agent not in accounted_for_agents}
                for agent in unnacounted_for_agents: #agent identifier
                    if self.agent_positions[agent] in room:
                        agents_in_room.add(agent)
                        accounted_for_agents.add(agent)
                self.agents_per_room[room_index] = agents_in_room or None

        assert len(self.agent_positions) == len(accounted_for_agents), "Not all agents were accounted for"

        #if None in self.agents_per_room.values():
        #    print("There are rooms with no agents")

        #agent:room alternative
        #for agent in range(len(self.agent_positions)): # agent identifier
        #    for room_index in range(len(self.rooms)):
        #        if self.agent_positions[agent] in self.rooms[room_index]:
        #            self.agents_per_room[agent] = room_index
        #            break
        #    assert self.agents_per_room[agent], "Agent was not assigned a room"

    def get_goals_distribution_per_room(self):
        """get dict of goal distribution in each room; 
        self.goals_per_room dictionary ->  room:goal_positions"""

        self.locate_separate_rooms()
        if self.goals_per_room:
            return
        self.goals_per_room = {}
        for room_index in range(len(self.rooms)):
            goals_in_room = {goal_pos for goal_pos in self.goal_positions if goal_pos in self.rooms[room_index]}
            self.goals_per_room[room_index] = goals_in_room or None

    def get_box_distribution_per_room(self):
        """get dict of box distribution in each room; 
        self.boxes_per_room dictionary->  room:box_positions"""

        self.locate_separate_rooms()
        if self.boxes_per_room:
            return        
        self.boxes_per_room = {}
        for room_index in range(len(self.rooms)):
            boxes_in_room = {box_pos for box_pos in self.box_positions if box_pos in self.rooms[room_index]}
            self.boxes_per_room[room_index] = boxes_in_room or None        

    def detect_useless_elements(self,delete = True):
        self.get_goals_distribution_per_room()
        self.get_agent_distribution_per_room()
        self.get_box_distribution_per_room()
        if self.useless_rooms:
            return
        self.useless_rooms = [] #indexes of useless rooms
        self.useless_agents= [] #indexes of useless agents
        self.useless_boxes = []

        for room_index, room in self.rooms:
            # no goals in room
            if not self.goals_per_room[room_index]:
                self.useless_rooms.append(room_index)
                #and the room has agents
                if self.agents_per_room[room_index]:
                    self.useless_agents.append(self.agents_per_room[room_index])
                if self.boxes_per_room[room_index]:
                    #not relevant
                    pass

            # no agents in room
            if not self.agents_per_room[room_index]
        
       # self.delete_useless_elements

 
    def locate_corridors(self):
        '''Finds corridors for each room and stores them in self.corridors -> {room:list_of_corridors}  '''
        # Consider: if corridor is of size 1 it's just a "door", still say it a corridor?
        self.locate_separate_rooms()
        if self.corridors:
            return
        self.corridors = {}
        number_of_rooms = len(self.rooms)
        for room_index in range(number_of_rooms):
            corridor_vertices_of_room = self.corridor_vertices_of_room(self.rooms[room_index])
            if not corridor_vertices_of_room:
                self.corridors[room_index] = None #room has no corridors
            else:
                corridors_in_room = self.break_container_into_adjacent_vertices(corridor_vertices_of_room)
                self.corridors[room_index] = corridors_in_room
                
        print("\nSelf.corridors: ",self.corridors)

    def locate_open_areas(self):
        '''Dependent on finding corridors first'''

        self.locate_corridors()
        if self.open_areas:
            return
        self.open_areas = {}
        number_of_rooms = len(self.rooms)
        for room_index in range(number_of_rooms):
            #get vertices that are corridors of a room as a single set 
            corridors_of_room = self.corridors[room_index]
            if corridors_of_room:
                corridor_vertices_of_room = self.union_of_sets(corridors_of_room)
                open_area_vertices = {v for v in self.rooms[room_index] if v not in corridor_vertices_of_room}
                open_areas = self.break_container_into_adjacent_vertices(open_area_vertices)
                if open_areas:
                    self.open_areas[room_index] = open_areas
                else: #open areas is []
                    self.open_areas[room_index] = None

            else:
                #if there are no corridors, the room is an open area
                self.open_areas[room_index] = self.rooms[room_index]

        print("open areas",self.open_areas)

    def locate_safe_storage(self):
        "Locates safe storage in each room"

        #TODO: overlooking placement near goals?
        #TODO: corridors areas that result in dead end (don't connect areas of level)?

        self.locate_corridors()
        self.get_goals_distribution_per_room()
        self.get_box_distribution_per_room()
        if self.safe_storage:
            return
        self.safe_storage = {}
        walls = self.deep_copy(self.walls)

        for room_index, room in enumerate(self.rooms):

            number_of_boxes_in_room = len(self.boxes_per_room[room_index])
            goals_of_room = self.goals_per_room[room_index]
            safe_storage_of_room = set()
            #to use self.union_of_sets
            if self.corridors[room_index]:
                corridors = self.deep_copy(self.corridors[room_index])
                corridors.append(goals_of_room)  
                list_sets_illegal_vertices = corridors              
            else:
                list_sets_illegal_vertices = [goals_of_room]


            print("\nROOM %s" %(room_index))
            print("GOALS OF ROOM")
            print(self.goals_per_room[room_index])
            print("NUMBER BOXES IN ROOM")
            print(number_of_boxes_in_room)
            print("List set illegal vertices")
            print(list_sets_illegal_vertices)
            illegal_vertices = self.union_of_sets(list_sets_illegal_vertices)
            print("illegal vertices")
            print(illegal_vertices)
            safe_storage_candidates = {v for v in self.rooms[room_index] if v not in illegal_vertices}  # safe storage can't be on goals or choke points
            print("safe storage candidates")
            print(safe_storage_candidates)
            vertices_to_add_again = set()

            while len(safe_storage_of_room) < number_of_boxes_in_room and safe_storage_candidates: # we either exhaust the candidates or reach a satisfying number of storage
                safe_storage_candidate = safe_storage_candidates.pop()
                condition1 = self.vertex_is_easily_accessible(safe_storage_candidate,walls)
                if condition1:
                    room.remove(safe_storage_candidate)
                    self.vertices.remove(safe_storage_candidate)
                    vertices_to_add_again.add(safe_storage_candidate)
                    #Condition 2: if the room is not fully connnected without the safe_storage_candidate, we see the connected components it was split in
                    #if one of them holds all the goals, condition2 is True
                    condition2 = self.is_connected_component(room,particular_vertices_are_all_in_a_component=True,vertices=goals_of_room) 
                    if condition2:
                        safe_storage_of_room.add(safe_storage_candidate) #if it verifies both conditions it is storage
                        walls.add(safe_storage_candidate) #it is treated as a wall for following candidates
                    else:
                        room.add(safe_storage_candidate)  # if not, it remains part of the room as a normal vertex and not a wall 

            print("safe storage of room")
            print(safe_storage_of_room)
            self.safe_storage[room_index] = safe_storage_of_room or None
            # add the vertices that were removed to both data sructures
            if safe_storage_of_room:
                room.update(safe_storage_of_room) 
            if vertices_to_add_again:
                self.vertices.update(vertices_to_add_again) 


    #def room_is_corridor(self):
    #def locate_open_areas()
    # open_areas_high_density_goals
    # open_areas_high_density_boxes
    # corridors_high_density_goals
    # corridors_high_densite_boxes
    # agents in corridors --> corridor occupied
    # agent in room --> quick access - say in which room the agent is
    # open spaces separated by corridor


    ##########################################################
    ###############          BFS UTILS            ############
    ##########################################################

    def run_bfs(self,source_vertex,cutoff_vertex=None,store_tree = True): #add self.bfs_trees_cut?
        '''
        Builds complete bfs tree with source_vertex as root, adds it so self.bfs_trees. 
        Tree is in the form of a dictionary structured in the following way: {vertex:(parent)}
        A cutoff_vertex may be passed in order to stop building the tree once that vertex is reached.
        May lead to congestion due to path similarity as it is used to get shortest paths from vertices to the source/root vertex
        '''
        assert source_vertex in self.vertices
        if source_vertex in self.bfs_trees: #if tree is already built
            return
        
        queue = deque([source_vertex]) 
        explored_set = set([source_vertex])
        parent = {} # vertex:parent
        parent[source_vertex] = None #this is different from the graph class! Facilitates some of this methods, for keys and values of dict comparison

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

        if store_tree:
            self.bfs_trees[source_vertex] = parent
        else:
            return parent


    def bfs_tree(self,source_vertex):
        '''Checks if tree with source_vertex as root is in self.bfs_trees, if not builds the tree and stores it in self.bfs_trees, returns tree'''
        assert source_vertex in self.vertices
        #if tree is not built, build it and store it
        if source_vertex not in self.bfs_trees:
            self.run_bfs(source_vertex)    
        return self.bfs_trees[source_vertex]

    def bfs_tree_no_store(self,source_vertex):
        """ As to not override trees"""
        assert source_vertex in self.vertices
        tree = self.run_bfs(source_vertex,store_tree=False)
        return tree

    def bfs_shortestpath_notree(self,source_vertex,target_vertex,illegal_vertices = {}, cutoff_branch = None):
        ''' 
        Returns Shortest path between two vertices without having a tree pre built or building one and storing it in self.bfs_trees
        If there is no path between the two vertices, returns None. May be useful if memory/time problems arise.
        '''
        assert source_vertex in self.vertices and target_vertex in self.vertices,  "Insert coordinates that are part of the state or not walls"
        if source_vertex == target_vertex:
            return deque()
        queue = deque([source_vertex]) 
        explored_set = set([source_vertex])
        explored_set.update(illegal_vertices)
        parent = {} # vertex:parent 
        counter = 0
        while queue:
            if counter == cutoff_branch:
                return None
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
            counter += 1

        return None #if no path is found


    def backtrack(self, source_vertex, target_vertex, parent_dict):
        '''Used to Return shortest path between two vertices, used in bfs_shortestpath_notree'''

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

    def corridor_vertices_of_room(self,room):
        '''Returns all vertices in a room which are part of corridors'''
        corridor_vertices = {vertex for vertex in room if self.is_corridor_candidate(vertex)}
        return corridor_vertices

    def corridor_vertex_condition(self,vertex):
        '''neighbours are quicly acesssible among them without having to go through vertex'''
        neighbours = deque(self.get_neighbours(vertex))
        if neighbours:
            neighbour = neighbours.pop()
            neighbours_shortest_paths = [self.bfs_shortestpath_notree(neighbour,n,cutoff_branch=20,illegal_vertices={vertex}) for n in neighbours]
            if None in neighbours_shortest_paths:
                    return True
            #if it is accessible but takes more than 3 steps (path includes initial and final vertex)
            for path in neighbours_shortest_paths:
                if len(path) > 5:
                    return True
        
        return False

    def is_corridor_candidate(self,vertex):
        neighbours = [n for n in self.get_neighbours(vertex)]
        if len(neighbours) == 1: #3 walls around him
            return self.corridor_vertex_condition(neighbours[0]) or False 
            
        elif self.corridor_vertex_condition(vertex):
            return True
        else: 
            return False

    def is_connected_component(self,container_of_vertices: set, particular_vertices_are_all_in_a_component= None, vertices = None):
        """
        True if the container is a connected component
        For special cases you can set particular_vertices_are_all_in_a_component to True,
        If the container_of_vertices is not fully connected but one of its individual components holds all of the vertices in the vertices argument, will return True
        """
        if particular_vertices_are_all_in_a_component:
            assert vertices, "Provide the vertices that should all be located in the same component"
            connected_components = self.locate_separate_connected_components(container_of_vertices,store_tree=False)            
            #so now that we have all the connected components, we will check if any of them holds all vertices desired
            for cc in connected_components:
                cc_holds_vertices = 0
                for vertex in vertices:
                    if vertex not in cc:
                        break
                    else:
                        cc_holds_vertices += 1 
                # if a component holds only a part of the vertices
                if 0 < cc_holds_vertices < len(vertices):
                    return False
                # if the component holds all vertices
                elif cc_holds_vertices == len(vertices):
                    return True

        else:
            vertex = container_of_vertices.pop()
            container_of_vertices.add(vertex)
            tree = self.bfs_tree(vertex)
            return len(container_of_vertices) != len(tree.keys())


    def locate_separate_connected_components(self,container_of_vertices: set,store_tree = True):
        '''Detect connected components of container of vertices'''

        initial_vertex = container_of_vertices.pop() 
        container_of_vertices.add(initial_vertex)
        if store_tree:
            tree = self.bfs_tree(initial_vertex) 
        else:
            tree = self.bfs_tree_no_store(initial_vertex)
        
        connected_components = []

        #initialize first room
        initial_room = {(i,j) for i,j in tree.keys()}
        connected_components.append(initial_room)

        #while we haven't accounted for all vertices to be in their room
        while self.sum_len_elements_of_list(connected_components) != len(container_of_vertices): 
            vertex_not_in_initial_room = self.from_not_in(container_of_vertices,connected_components)
            if store_tree:
                tree = self.bfs_tree(initial_vertex) 
            else:
                tree = self.bfs_tree_no_store(initial_vertex)
            new_connected_component = {(i,j) for i,j in tree.keys()}
            connected_components.append(new_room)
            new_connected_component = {}

        return connected_components

    def vertex_is_easily_accessible(self,vertex,walls_container):
        assert vertex
        neighbours = self.get_neighbours(vertex)
        neighbours_not_in_wall_container = {n for n in neighbours if n not in walls_container}
        return len(neighbours_not_in_wall_container) > 0
    
    def delete_useless_elements(self,rooms:list = None,agents:list = None,boxes:list = None):
        """Removes useless room information"""
        
        if rooms:
            for room_index in rooms:
                del self.rooms[room_index] # we are sure the room exists
                self.agents_per_room.pop(room_index)# not that is has agents or boxes
                self.boxes_per_room.pop(room_index)

        if agents:
            for agent_index in agents:
                del self.agents_per_room[room_index]

        
    def union_of_sets(self,list_of_sets):

        assert list_of_sets
        set_union = set()
        number_of_sets = len(list_of_sets)
        for set_index in range(number_of_sets):
            set_union = set_union.union(list_of_sets[set_index])
        return set_union

    def break_container_into_adjacent_vertices(self,container):
        '''Goes through vertices in container and returns list of deques with vertices grouped with adjacent vertices'''

        list_of_deques = []
        while container:
            current_vertex = container.pop()
            adjacent_to_current_vertex = self.find_adjacent_vertices_in_container(current_vertex,container) 
            adjacent_to_current_vertex.update([current_vertex]) #we'll say the current vertex is adjacent to himself because we use this for determining corridors
            container.difference_update(adjacent_to_current_vertex)
            list_of_deques.append(adjacent_to_current_vertex)
        return list_of_deques

    def find_adjacent_vertices_in_container(self,vertex,container):
        '''Find adjacent vertices of adjacent vertices and so on for a particular vertex in a container'''

        #assert container , "container is empty"
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

    def container_is_composed_of_adjacent_vertices(self,container):
        '''True if container is composed of only adjacent vertices, same as previous function but used to check condition instead,'''
        # TODO: merge it into previous one as argument
        assert container , "container is empty"
        vertex = container.pop()
        container.add(vertex)
        explored_set = set([vertex])
        adjacent_to_vertex = {v for v in container if u.are_adjacent(vertex,v)}
        non_explored = {v for v in adjacent_to_vertex if v not in explored_set}
        while non_explored:
            vertex = non_explored.pop()
            explored_set.add(vertex)
            adjacent_to_an_adjacent = {v for v in container if u.are_adjacent(vertex,v)}
            adjacent_to_vertex.update(adjacent_to_an_adjacent)
            non_explored = {v for v in adjacent_to_vertex if v not in explored_set}
        adjacent_to_vertex.add(vertex)
        #print(len(adjacent_to_vertex) == len(container))
        return len(adjacent_to_vertex) == len(container)

        
    def is_corner(self,vertex):
        (x,y) = vertex
        opt1,opt2,opt3,opt4 = {(x-1,y),(x,y+1)},{(x+1,y),(x,y+1)},{(x+1,y),(x,y-1)},{(x-1,y),(x,y-1)}
        neighbouring_walls = self.get_neighbours(vertex,in_vertices=False,in_walls=True)
        if neighbouring_walls == opt1 or neighbouring_walls == opt2 or neighbouring_walls == opt3 or neighbouring_walls == opt4:
            return True
        else:
            return False

    def is_corridor_corner(self,vertex):
        (x,y) = vertex
        opt1,opt2,opt3,opt4 = {(x-1,y),(x,y+1),(x+1,y-1)},{(x+1,y),(x,y+1),(x-1,y-1)},{(x+1,y),(x,y-1),(x-1,y+1)},{(x-1,y),(x,y-1),(x+1,y+1)}
        neighbouring_walls = self.get_neighbours(vertex,in_vertices=False,in_walls=True)
        if opt1.issubset(self.walls) or opt2.issubset(self.walls) or opt3.issubset(self.walls) or opt4.issubset(self.walls):
            return True
        else:
            return False

    def get_neighbours(self,vertex,in_vertices = True, in_walls = False, in_container = None, not_in_container = None):
        '''Returns neighbours, returns none if there are no neighbours'''

        assert vertex in self.vertices
        (x,y) = vertex
        neighbours = {(x,y+1),(x,y-1),(x-1,y),(x+1,y)}

        if in_vertices:
            neighbours = {n for n in neighbours if n in self.vertices}
        if in_walls:
            neighbours = {n for n in neighbours if n in self.walls}
        if in_container:
            neighbours = {n for n in neighbours if n in in_container}
        if not_in_container:
            neighbours = {n for n in neighbours if n not in not_in_container}   

        return neighbours

    def get_specific_neighbours(self,vertex,in_vertices = True, in_walls = False, S_ = None,N_ = None, W_ = None, E_ = None):
        '''Returns neighbours, returns none if there are no neighbours'''

        assert vertex in self.vertices
        (x,y) = vertex
        N,S,W,E = (x,y+1),(x,y-1),(x-1,y),(x+1,y)
        neighbours = set()
        if S_:
            neighbours.add(S)
        if N_:
            neighbours.add(N)
        if W_:
            neighbours.add(W)
        if E_:
            neighbours.add(E)

        if in_vertices:
            neighbours = {n for n in neighbours if n in self.vertices}
        if in_walls:
            neighbours = {n for n in neighbours if n in self.walls}
        return neighbours
    
    def get_neighbours_2coordinates_away(self,vertex):
        assert vertex in self.vertices
        (x,y) = vertex
        neighbours = {(x,y+2),(x,y-2),(x-2,y),(x+2,y)}
        neighbours = {n for n in neighbours if n in self.vertices}
        #assert len(neighbours) == 1 #only for depression edge case
        return neighbours

    def number_neighbouring_walls_of_vertex(self,vertex):
        '''Returns amount of neighbours of partical vertex that are walls (min:0 ; max:4) '''

        neighbours = self.get_neighbours(vertex, in_vertices = None)
        n_neighbouring_walls = 0
        neighbours_in_walls = {n for n in neighbours if n in self.walls}
        n_neighbouring_walls += len(neighbours_in_walls)
        assert n_neighbouring_walls >= 0 and n_neighbouring_walls <= 4, "Neighbouring walls must be between 0 and 4"
        return n_neighbouring_walls


    def sum_len_elements_of_list (self,list_):
        lenght = 0
        for element in list_:
            lenght += len(element)
        return lenght      

    def from_not_in (self, from_container, not_in_containers):
        '''Get an element in from_container that isn't in any of the not_in_containers,
        Returns None if not possibe to do so'''
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

    def direction_between_two_adjacent_vertices(self,_from,to):

        subtract = tuple(np.subtract(_from,to))

        if subtract == (1,0):
            direction = "S"
        elif subtract == (-1,0):
            direction = "N"
        elif subtract == (0,1):
            direction = "E"
        elif subtract == (0,-1):
            direction = "W"
        else:
            raise ValueError("Vertices are not adjacent")

        return (_from,direction)    

    def get_children_dictionary(self,parent_dictionary):
        '''Turn dictionary in form children:(parent) to parent:(children)'''
        children = defaultdict(list)
        for child, parent in parent_dictionary.items():
            children[parent].append(child)
        return children

    def is_neighbour_of_vertex(self,is_neighbour,of_vertex):
        return is_neighbour in self.get_neighbours(of_vertex)

    def is_wall (self,vertex):
        return vertex in self.walls

    def are_walls (self,vertices):
        are_walls = {v for v in vertices if self.is_wall(v)}
        return are_walls
    
    def deep_copy(self,x):
        return copy.deepcopy(x)  

#def locate_high_density_areas:
#def locate_high_density_clustered_goals_in_corridor:
#def locate_complicated_corners:
#class Goal_Rooms_Tree:
#or just block and see if connection from room to otther rooms is not blocked
# in choke points : see if they connect different rooms




agt0 = util.agent(0,"red")
agt1 = util.agent(1,"blue")
box0  = util.box("A", "blue")
maze = [                                        #6            #8
        [False,False, False,False, False,False, False, False, False, False,False],
        [False,True, True, False, False, False, False, False, True,  False, False],#1
        [False,True, True, False, False, False, False, False,  True,  False, False],
        [False,True, True, False, False, False, False, True,  True,  True, False], #3
        [False,True, True, False, False, False, False, True,  True,  True, False],
        [False,False,True,False, False, False,  False, True,  True,  True, False],
        [False,False, False,False, False,False, False, False, False, False,False]
                             #3                
    ]

builder = StateBuilder()
builder.set_maze(maze)
builder.add_agent(0,(1,1),0)
builder.add_agent(1,(2,7),1)
builder.add_agent(2,(5,8),2)

box_list = [(1,(3,7),1),(1,(3,9),1),(1,(1,2),0),(1,(5,7),2),(1,(2,2),0)]
goal_list = [(1,(1,8)),(1,(4,7)),(2,(3,2))]
for t,pos,color in box_list:
    builder.add_box(t,pos,color)
for t,pos in goal_list:
    builder.add_goal(t,pos)

state = builder.build_StateMA()
start = time.time()

L = LevelAnalyser(state)
L.locate_safe_storage()
#print(L.corridors)
#print(L.boxes_per_room)
#print()
#L.locate_safe_storage()


end = time.time()
print(end-start)