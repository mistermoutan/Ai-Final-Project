import numpy as np
from state import StateMA, StateSA
from collections import defaultdict
import queue

# TODO: move this to appropriate place
def get_neighbours(vertex):
    """Returns neighbours as set"""
    (x, y) = vertex
    return {(x, y + 1), (x, y - 1), (x - 1, y), (x + 1, y)}


class GoalAnalyzer:
    def __init__(self, state: StateMA):
        self.vertices = set()
        row, col = np.asarray(state.maze).shape
        for i in range(row):
            for j in range(col):
                if state.maze[i][j]:
                    self.vertices.add((i, j))

        self.state = state
        self.goals = state.goal_positions

        self.rooms, self.vert_in_room, self.edges = self.get_goal_room_graph(state.goal_positions)
        self.connections = defaultdict(set)

        for v1, v2 in self.edges:
            i = self.vert_in_room[v1]
            j = self.vert_in_room[v2]
            self.connections[i].add(j)
            self.connections[j].add(i)

        self.n_goals = len(state.goal_positions)
        self.storage = dict()


    def get_goal_room_graph(self, goals):
        rooms = [set((goal,)) for goal in goals]
        vert_in_room = {goal: i for i,goal in enumerate(goals)}
        seen = set(goals)
        edges = []
        # TODO: compute corridors into rooms aswell?
        for i, g in enumerate(goals):
            # for every goal we will check adjacent squares and if we haven't seen them before they are now a new room
            # which we grow with dfs
            neighbors = get_neighbours(g)
            for n in neighbors:
                if n in seen:
                    edge = (g, n)
                    edges.append(edge)
                if n in self.vertices and n not in seen:
                    idx = len(rooms)
                    room = set((n,))
                    rooms.append(room)
                    # TODO: use bfs instead of dfs? (only necessary if we wanna get some min distances between nodes)
                    stack = [n]
                    while stack:
                        curr = stack.pop()
                        if curr in seen and vert_in_room[curr] != idx:
                            edge = (curr, n)
                            edges.append(edge)
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

    def print_rooms(self):
        s = ""
        for i in range(self.state.rows):
            for j in range(self.state.cols):
                if not self.state.maze[i][j]:
                    s += "+"
                else:
                    # Deadspace gets turned into walls
                    if (i,j) not in self.vert_in_room:
                        s += "+"
                        continue
                    val = self.vert_in_room[(i,j)]
                    # to avoid having 2 character per grid space we convert goals greater than 9 to characters
                    if val > 9:
                        val = 87 + val
                        s += chr(val)
                    else:
                        s += str(val)

            s += "\n"
        print(s)

    def compute_goal_eccentricity(self):
        ecc = [0]*self.n_goals
        for i in range(len(self.state.goal_positions)):
            seen = set()
            q = queue.Queue()
            m = 0
            for j in self.connections[i]:
                q.put((j, 0))
                seen.add(j)

            while not q.empty():
                room, depth = q.get()

                # goals are always the first rooms
                if room < self.n_goals:
                    depth += 1
                    m = max(m, depth)
                for j in self.connections[room]:
                    if j not in seen:
                        q.put((j, depth))
                        seen.add(j)
            ecc[i] = m
        return ecc

    def is_goal(self, id):
        return id < self.n_goals

    def get_leaf_loss(self, id, removed):
        # TODO: make loss number of seep storage space available
        neighbours = self.connections[id]

        important_neighbours = 0
        loss = 0
        losses = []

        for i in neighbours:
            if i not in removed:
                if self.is_goal(id):
                    important_neighbours += 1
                else:
                    important = False
                    for n in self.connections[i]:
                        if n == id:
                            continue
                        if n not in removed:
                            important = True
                            important_neighbours += 1
                    if not important:
                        loss += len(self.rooms[i])
                        losses.append(i)

        return important_neighbours < 2, loss, losses

    def free_space(self, id):
        return len(self.rooms[id])

    def isolated(self,i, id, removed):
        # returns true if a room is isolated from all other vertices
        # given the removed set and removed have beeen removed form the graph
        for n in self.connections[i]:
            if n not in removed and n != id:
                return False
        return True

    def compute_loss(self, id, removed):
        # computes the amount of lost free space from adjacent rooms if id is removed
        loss = 0
        for n in self.connections[id]:
            if n not in removed:
                if self.isolated(n, id, removed):
                    loss += self.free_space(n)
        return loss

    def check_cutsafe_cycle(self, id, removed, is_tree):
        neighbours = self.connections[id]

        # check if current id is a leaf node, viable will be a set of nodes that are not singular rooms
        viable = set()
        for i in neighbours:
            if i not in removed:
                if self.is_goal(i):
                    viable.add(i)
                else:
                    if not self.isolated(i, id , removed):
                        viable.add(i)

        # if there is less than 2 non isolated rooms next to our current we are at a leaf
        if len(viable) < 2:
            return True, False

        # if whatever is left of the graph is a tree we don't need to look for cycles
        if is_tree:
            return False, False

        cycle = False

        found = set()
        seen = set((id,))
        first_pass = True

        # see what happens if id is remove from the graph:
        # can we reach every neighbour? (if so it is cut safe)
        # are at least 2 neighbours that are still connected? (if so there is a cycle)
        for i in viable:
            found.add(i)
            seen.add(i)
            stack = [i]
            while stack:
                curr = stack.pop()
                seen.add(curr)
                neighbours = self.connections[curr]
                for n in neighbours:
                    if n not in seen and n not in removed:
                        seen.add(n)
                        stack.append(n)
                        if n in viable and n != i:
                            found.add(n)
                            cycle = True
            # if we manage to find every neighbour through the first one we have found a cutsafe cycle
            if first_pass:
                first_pass = False
                if len(found) == len(viable):
                    return True, True

        return False, cycle

    def compute_goal_order_plan(self):
        # computes a plan on how to order to goals in order to complete them, operates under the assumption that
        # every box is mobile and currently assumes every room has infinite space and goal cells are empty

        # TODO use different metric? perhaps use distance from largest room?
        #eccentricities = self.compute_goal_eccentricity()
        #eccentricity_order = [(eccentricities[i], pos) for i, pos in enumerate(self.state.goal_positions)]
        #eccentricity_order = sorted(eccentricity_order,key=lambda x: -x[0])
        #eccentricity_order = [x[1] for x in eccentricity_order]

        # TODO: find space left  and use as metric as well

        removed = set()
        plan = []
        cycle_cuts = []

        incomplete_goals = {i for i, _ in enumerate(self.state.goal_positions)}


        # TODO: add clears as a part of the plan
        # TODO: create some sort of goal partial orders
        is_tree = False
        while incomplete_goals:

            best = -1
            lowest = 99999999999
            in_cycle = False
            easy_removals = []
            cycle_found = False
            for i in incomplete_goals:
                cutsafe, cycle = self.check_cutsafe_cycle(i, removed, is_tree)
                cycle_found = cycle or cycle_found
                if cutsafe:
                    loss = self.compute_loss(i, removed)
                    if loss == 0 and not cycle:
                        easy_removals.append(i)
                    if loss < lowest:
                        best = i
                        lowest = loss
                        in_cycle = cycle

            if len(easy_removals) > 0:
                for i in easy_removals:
                    plan.append(i)
                    incomplete_goals.remove(i)
                    removed.add(i)
                    cycle_cuts.append(False)
            else:
                plan.append(best)
                cycle_cuts.append(in_cycle)
                incomplete_goals.remove(best)
                removed.add(best)

        return plan

    def get_viable_goals(self, completed):
        incomplete_goals = {i for i, _ in enumerate(self.state.goal_positions) if i not in completed}
        available = []

        for i in incomplete_goals:
            cutsafe, cycle = self.check_cutsafe_cycle(i, completed, False)
            if cutsafe:
                available.append(i)

        return available

    def get_isolated_by_goal_completion(self, goal, completed):
        """ computes which adjacent rooms will be isolated after given goal is completed given previously completed goals """
        rooms = []
        for n in self.connections[goal]:
            if n not in completed:
                if self.isolated(n, goal, completed):
                    rooms.append(n)
        return rooms

    def get_storage_spaces_for_room(self, room_id):
        if room_id in self.storage:
            return self.storage[room_id]

        room = self.rooms[room_id]
        if len(room) == 1:
            self.storage[room_id] = set()
            return self.storage[room_id]

        unseen = set(room)
        path = set()
        undecided = set()
        storage = set()

        start = None
        # neighbours to rooms should be goal spots so we just grab the space and find its neighbours
        # if those spaces are in our room they are neighbour entrance, we also pick one at random as our start
        neighbors = self.connections[room_id]
        neighbor_entrances = set()
        for neighbor in neighbors:
            space = set(self.rooms[neighbor]).pop()
            for n in get_neighbours(space):
                if n in room:
                    start = n
                    neighbor_entrances.add(n)

        unseen.remove(start)
        path.add(start)
        for i in get_neighbours(start):
            if i in room:
                unseen.remove(i)
                undecided.add(i)

        # TODO: this can be done more efficiently than this brute force, but it is more complicated
        while len(undecided) > 0:
            best = None
            best_score = -9999
            decided_storage = []
            for i in undecided:
                neighbors = get_neighbours(i)
                gain = 0
                undecided_bonus = 0
                for n in neighbors:
                    if n in room and n in unseen:
                        gain += 1
                    if n in room and n in undecided:
                        undecided_bonus += 0.01
                if gain == 0:
                    decided_storage.append(i)
                else:
                    score = 0
                    # TODO: find better values for penalizing boxes and agents in path
                    if self.state.box_at(i[0], i[1]):
                        score -= 4
                    if i in self.state.agent_by_cords:
                        score -= 0.5
                    if i in neighbor_entrances:
                        score += 0.1
                    score += gain + undecided_bonus

                    if score > best_score:
                        best = i
                        best_score = score
                        if gain == 3 and score > 0: # we cant gain access to more than 3 new spaces
                            break

            for i in decided_storage:
                storage.add(i)
                undecided.remove(i)

            if best is not None:
                undecided.remove(best)
                path.add(best)
                for n in get_neighbours(best):
                    if n in unseen and n in room:
                        undecided.add(n)
                        unseen.remove(n)
            # self.debug_storage_print(storage, path)



        # we now must ensure that all neighbors are reachable from the given path
        # so we check if this is the case, if not we can fix it by turning 1 storage space to path space
        for neighbor in self.connections[room_id]:
            potential = None
            found = False
            assert len(self.rooms[neighbor]) == 1, "we should only have goals adjacent to the current room"

            # TODO: avoid picking spaces with boxes on them
            for space in self.rooms[neighbor]:
                for n in get_neighbours(space):
                    if n in room:
                        potential = n
                        if n in path:
                            found = True
                            break
                if not found:
                    storage.remove(potential)
                    path.add(potential)

        self.storage[room_id] = storage
        return storage

    def is_storage(self, cord):
        if not cord in self.vert_in_room:
            return False
        room = self.vert_in_room[cord]
        storage = self.get_storage_spaces_for_room(room)
        if cord in storage:
            return True
        return False

    def print_storage_spaces(self, path_symbol=" ", storage_symbol="0"):
        s = ""
        for i in range(self.state.rows):
            for j in range(self.state.cols):
                if not self.state.maze[i][j]:
                    s += "+"
                else:
                    if self.is_storage((i,j)):
                        s += storage_symbol
                    else:
                        s += path_symbol

            s += "\n"
        print(s)

    def debug_storage_print(self, storage, path):
        s = ""
        for i in range(self.state.rows):
            for j in range(self.state.cols):
                if not self.state.maze[i][j]:
                    s += "+"
                else:
                    if (i, j) in path:
                        s += " "
                    elif (i, j) in storage:
                        s += "0"
                    else:
                        s += "?"

            s += "\n"
        print(s)



def try_get_goal_room_graph():
    import test_utilities as tu

    maze = tu.create_maze(16)

    for i in range(len(maze)-2):
        maze[i][4] = False
        maze[i][6] = False

    maze[7][2] = tu.goal('a')
    maze[8][4] = tu.goal('b')
    maze[8][5] = tu.goal('c')
    maze[8][6] = tu.goal('a')
    maze[5][5] = tu.goal('b')
    maze[4][5] = tu.goal('a')
    maze[3][6] = tu.goal('f')
    maze[9][5] = tu.goal('g')
    maze[10][5] = tu.goal('h')
    maze[11][5] = tu.goal('h')
    maze[12][5] = tu.goal('h')
    maze[13][5] = tu.goal('h')
    maze[14][5] = tu.goal('h')
    maze[14][6] = tu.goal('h')

    # [row][col]
    maze[2][5] = tu.box('a', 'lul')
    maze[7][3] = tu.box('b', 'lul')
    maze[6][3] = tu.box('c', 'lul')
    maze[5][3] = tu.box('a', 'lul')
    maze[7][8] = tu.box('b', 'lul')
    maze[8][8] = tu.box('a', 'lul')
    maze[6][8] = tu.box('f', 'lul')


    maze[1][1] = tu.agent(0, 'lul')

    state = tu.make_state(maze)
    analyzer = GoalAnalyzer(state)
    print(state, "\n")
    analyzer.print_rooms()
    analyzer.compute_goal_order_plan()
    plan = analyzer.compute_goal_order_plan()
    print(plan)
    partial_plan = plan[:6]
    print(partial_plan)
    print("viable goal nodes:", analyzer.get_viable_goals(set(partial_plan)))
    curr = 13
    print("cutoff rooms given", curr, ":", analyzer.get_isolated_by_goal_completion(curr, partial_plan))

    analyzer.print_storage_spaces()
    if len(analyzer.rooms) != 18:
        print("number of rooms is wrong")
        return False
    spaces = 0
    for r in analyzer.rooms:
        for _ in r:
            spaces += 1


    if spaces != sum([sum(i) for i in state.maze]):
        print("number of spaces in rooms are not equal to actual number of spaces")
        return False
    return True

if __name__ == '__main__':
    if not try_get_goal_room_graph():
        print("Test failed")