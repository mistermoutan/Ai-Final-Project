import numpy as np
from state import StateMA, StateSA
from collections import defaultdict

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

        self.rooms, self.vert_in_room, self.edges = self.get_goal_room_graph(state.goal_positions)
        self.connections = defaultdict(set)

        for v1,v2 in self.edges:
            i = self.vert_in_room[v1]
            j = self.vert_in_room[v2]
            self.connections[i].add(j)
            self.connections[j].add(i)

    def get_goal_room_graph(self, goals):
        rooms = [set((goal,)) for goal in goals]
        vert_in_room = {goal: i for i,goal in enumerate(goals)}
        seen = set(goals)
        edges = []
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
                    s += str(self.vert_in_room[(i,j)])

            s += "\n"
        print(s)


def try_get_goal_room_graph():
    import test_utilities as tu

    maze = tu.create_maze()
    goal = tu.goal('a')

    for i in range(len(maze)-2):
        maze[i][4] = False
        maze[i][6] = False

    maze[7][2] = goal
    maze[8][4] = goal
    maze[8][5] = goal
    maze[8][6] = goal
    maze[5][5] = goal

    state = tu.make_state(maze)
    analyzer = GoalAnalyzer(state)
    analyzer.print_rooms()

    if len(analyzer.rooms) != 8:
        print("number of rooms is wrong")
        return False
    print(state)
    if len(analyzer.edges) != len(analyzer.rooms)-1:
        print("number of edges is not correct")
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