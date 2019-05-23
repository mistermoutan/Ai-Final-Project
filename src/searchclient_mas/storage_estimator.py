from state import StateMA
import test_utilities as tu

offsets = [(1,1),(1,0),(1,-1),(0,-1),(-1,-1),(-1,0),(-1,1),(0,1)]
imm_neighbors = {(i, j) for i, j in offsets if (i+j) % 2 == 1}
diag_neighbors = {(i, j) for i, j in offsets if (i+j) % 2 == 0}
next_dict = dict()
prev_dict = dict()

for i in range(len(offsets)):
    p = i - 1
    n = (i + 1) % len(offsets)

    next_dict[offsets[i]] = offsets[n]
    prev_dict[offsets[i]] = offsets[p]

free = 0
agent = 1
box = 2
goal = 3
wall = 4

def get_adjacency_map(pos, state: StateMA, ignore_spaces, blocked_spaces):
    # 0 indicates free space
    # 1 indicates agent space
    # 2 indicates box space
    # 3 indicates goal space
    # 4 indicates blocked/wall space

    adj_map = dict()
    for offset in offsets:
        off_pos = (pos[0] + offset[0], pos[1] + offset[1])
        if off_pos in ignore_spaces:
            adj_map[offset] = free
        elif not state.maze[off_pos[0]][off_pos[1]]:
            adj_map[offset] = wall
        elif off_pos in blocked_spaces:
            adj_map[offset] = wall
        elif off_pos in state.agent_by_cords:
            adj_map[offset] = agent
        elif off_pos in state.box_by_cords:
            adj_map[offset] = box
        elif off_pos in state.goal_by_cords:
            adj_map[offset] = goal
        else:
            adj_map[offset] = free

    return adj_map


def storage_value(pos, state: StateMA, ignore_spaces=set(), blocked_spaces=set(), is_box=True):
    # returns high values for good storage spaces and low values for bad ones
    # TODO: everything but the adjacency map can be precomputed before even seeing the state and loaded from file
    adj_map = get_adjacency_map(pos, state, ignore_spaces, blocked_spaces)
    on_goal_penalty = 0
    if pos in state.goal_positions:
        on_goal_penalty = 20
    occupied = [space for space in adj_map.keys() if adj_map[space] > 0]
    n_free = 8 - len(occupied)

    value = 100
    value -= on_goal_penalty
    if n_free == 8:
        return value - 1
    elif n_free == 7:
        return value - 5
    elif n_free == 6:
        p1, p2 = occupied

        dx = abs(p1[0] - p2[0])
        dy = abs(p1[1] - p2[1])

        if dx < 2 and dy < 2:
            return value - 10

        if adj_map[p1] == wall and adj_map[p2] == wall:
            return value - 70

    curr = 1
    if is_box:
        curr = 2

    n_blocked = 0
    walls = 0
    same = 0
    diff = 0
    goals = 0
    for i in imm_neighbors:
        x, y = i
        type = adj_map[i]
        if type != free and type != goal:
            n_blocked += 1
            if type == wall:
                walls += 1
            elif type == goal:
                goals += 1
            elif type == curr:
                same += 1
            else:
                diff += 1
        elif type == goal:
            goals += 1

    # TODO: check if we are blocking stuff off
    # ex: * is being blocked off from the rest
    # #
    # #0#
    # #*#
    if n_blocked == 3:
        return value - 10 - 5*same - 10*diff - 15*goals

    splits = 0

    seen = set()

    # here we check if it is the
    #
    # #0# or #0
    #        ##
    # case

    for i in imm_neighbors:
        if (adj_map[i] == free or adj_map[i] == goal)and i not in seen:
            splits += 1
            n = i
            p = prev_dict[i]

            while n not in seen and (adj_map[n] == free or adj_map[n] == goal):
                seen.add(n)
                n = next_dict[n]

            while p not in seen and (adj_map[p] == free or adj_map[n] == goal):
                seen.add(p)
                p = prev_dict[p]

    # there will always be at least one split
    splits -= 1
    penalty = 0
    if splits > 0:
        penalty = 50

    return value - penalty - splits*20 - 5*same - 10*diff - 15*goals

def print_state_storage_values(state: StateMA):
    print("original:")
    print(state)
    print("")

    s = ""
    for i in range(state.rows):
        for j in range(state.cols):
            if not state.maze[i][j]:
                s += "+"
            else:
                if not state.maze[i][j]:
                    s += "+"
                else:
                    val = storage_value((i,j), state)
                    if val < 10:
                        val = 10
                    #print(val)
                    s += str(val//10 - 1)

        s += "\n"
    print(s)


def storage_value_test():

    maze = tu.create_maze(10)
    for i in range(1, 9):
        maze[1][i] = tu.box('a', 'blue')
        maze[2][i] = tu.box('a', 'blue')

    for i in range(2, 9):
        maze[7][i] = False


    maze[8][5] = tu.goal('a')

    maze[3][5] = tu.agent(0, 'blue')

    state = tu.make_state(maze)

    print_state_storage_values(state)

if __name__ == '__main__':
    storage_value_test()