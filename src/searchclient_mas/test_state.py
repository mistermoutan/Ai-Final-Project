import test_utilities as tu
from test_utilities import north,east,west,south

def create_maze(size = 10):

    level = [[False for _ in range(size)] for _ in range(size)]

    for i in range(1, size-1):
        for j in range(1,size-1):
            level[i][j] = True
    return level

def test_swap():
    maze = create_maze(10)
    agt1 = tu.agent(0,'r')
    agt2 = tu.agent(1,'r')

    maze[5][5] = agt1
    maze[5][4] = agt2

    state = tu.make_state(maze)
    res = state.get_child([tu.move(west), tu.move(east)])
    if res is None:
        return True
    return False

def test_move():
    maze = create_maze(10)
    agt1 = tu.agent(0,'r')
    agt2 = tu.agent(1,'r')
    agt3 = tu.agent(2,'r')

    maze[5][5] = agt1
    maze[5][4] = agt2
    maze[4][5] = agt3

    state = tu.make_state(maze)
    # ++++++++++
    # +        +
    # +        +
    # +        +
    # +    2   +
    # +   10   +
    # +        +
    # +        +
    # +        +
    # ++++++++++
    res = state.get_child([tu.move(south), tu.move(east), None])
    if not (res.agent_by_cords[(4,5)] == 2 and res.agent_by_cords[(5,5)] == 1 and res.agent_by_cords[(6,5)] == 0):
        print("Failed move test 1")
        return False

    res = res.get_child([tu.move(south), tu.move(south), tu.move(south)])

    if not (res.agent_by_cords[(5,5)] == 2 and res.agent_by_cords[(6,5)] == 1 and res.agent_by_cords[(7,5)] == 0):
        print("Failed move test 1")
        return False

    res = res.get_child([tu.move(south), tu.move(north), tu.move(south)])

    if res is not None:
        print("Failed move test 3")
        return False

    return True


def test_push_pull():
    maze = create_maze(10)
    agt0 = tu.agent(0,'g')
    agt1 = tu.agent(1,'r')
    agt2 = tu.agent(2,'r')

    box1 = tu.box('a','r')

    maze[5][5] = agt0
    maze[5][4] = agt1
    maze[4][5] = agt2
    maze[4][4] = box1

    state = tu.make_state(maze)
    # ++++++++++
    # +        +
    # +        +
    # +        +
    # +   a2   +
    # +   10   +
    # +        +
    # +        +
    # +        +
    # ++++++++++
    res = state.get_child([None, tu.push(north, north), tu.pull(east, west)])
    if res is not None:
        print("failed push pull test 1")
        return False
    res = state.get_child([None, tu.push(north, north), tu.move(east)])
    if not (res.box_at(3,4) and res.agent_by_cords[(4,4)] == 1):
        print("failed push test 1")
        return False
    res = state.get_child([tu.move(west), tu.pull(south, north), tu.move(east)])
    if res is not None:
        print("failed pull test 1")
        return False
    res = state.get_child([None, tu.pull(south, north), tu.move(east)])
    if not (res.box_at(5,4) and  res.agent_by_cords[(6,4)] == 1):
        print("failed pull test 2")
        return False

    res = res.get_child([tu.pull(east, west), None, None])

    if res is not None:
        print("failed pull color test")
