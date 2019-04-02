class LevelAnalyser:

    def __init__(self,state):

        self.vertices = {}




    def separate_rooms_exist(self):
        """True if there are separate rooms (isolated parts of the level), False otherwise"""

        initial_vertex = self.vertices.pop() 
        self.vertices.add(initial_vertex)
        tree = self.tree_of_vertex(initial_vertex) 

        if tree:
            return len(self.vertices) != len(tree.keys())
        else:
            return True #if tree is none, that vertex is isolated and therefore we have different rooms in the level

    def locate_separate_rooms(self):

        """Creates self.rooms: list containing each separate room as sets"""

        assert self.separate_rooms_exist(), "There are no isolated rooms"

        initial_vertex = self.vertices.pop() 
        self.vertices.add(initial_vertex)
        tree = self.tree_of_vertex(initial_vertex) 
        vertices_list = list(self.vertices) #will need to iterate
        rooms = []

        if tree:
            initial_room = {(i,j) for i,j in tree.keys()}
            rooms.append(initial_room)
        else: # case where room is just a single isolated vertex
            rooms.append({initial_vertex})

        #so while we haven't account for all vertexes to be in their room
        while self.sum_len_elements_of_list(rooms) != len(self.vertices): 
            vertex_not_in_initial_room = self.from_not_in(vertices_list,rooms)
            tree = self.tree_of_vertex(vertex_not_in_initial_room) 
            new_room = []
            if tree:
                new_room = {(i,j) for i,j in tree.keys()}
                rooms.append(new_room)
            else: # case where room is just a single isolated vertex
                rooms.append({vertex_not_in_initial_room})
            new_room = []

        self.rooms = rooms
 
    def locate_corridors(self, min_lenght):
        """Returns list of all corridors with minimum lenght specified """

        vertical_corridors = []
        horizontal_corridors = []
        explored_set = set()
        vertices_list = list(self.vertices) #will need to iterate

        # choose initial vertex that is relevant 
        for v in vertices_list:
            n_neigh_walls = self.number_neighbouring_walls_of_vertex(v)
            if n_neigh_walls < 2:
                explored_set.add(v)
            elif n_neigh_walls == 4:
                explored_set.add(v)
            else:
                vertex = v


    #def locate_high_density_areas:

    #def locate_high_density_clustered_goals_in_corridor:

    
    #def locate_complicated_corners:

    #class Goal_Rooms_Tree:



      """
        while len(explored_set) != len(self.vertices):

            vertical_neighbours = self.get_vertical_neighbours(vertex)
            is_corridor = 0
            possible_corridor = []
            possible_corridor.append(vertex)

            if vertical_neighbours: # if there are vertical neighbours
                for vertical_neighbour in vertical_neighbours:
                    n_neigh_walls = self.number_neighbouring_walls_of_vertex(vertical_neighbour) #number of neighbouring walls that neighbour has
                    if n_neigh_walls < 2:
                        explored_set.add(vertical_neighbour)
                    else:
                        explored_set.add(vertical_neighbour)
                        is_corridor += 1
                        possible_corridor.append(vertical_neighbour)
                    
                if is_corridor == 0:
                    vertex = self.from_not_in
                    


                    
            else:
                explored_set.add(vertex)
                vertex = self.from_not_in


            for n in range(min_lenght):
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
