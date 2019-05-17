from Parallel_planner import ParallelPlanner
from parallel_realizer import ParallelRealizer
import test_utilities as tu


def prepare_storage_test():

    maze = tu.create_maze(10)
    for i in range(1, 9):
        maze[1][i] = tu.box('a', 'blue')
        maze[2][i] = tu.box('a', 'blue')

    maze[8][5] = tu.goal('a')

    maze[3][5] = tu.agent(0, 'blue')

    state = tu.make_state(maze)

    planner = ParallelPlanner(state)
    print(state)
    planner.goal_analyzer.print_storage_spaces()

    storage_plan, storage_state = planner.prepare_storage()

    print(storage_state)

def storage_realization_test():
    maze = tu.create_maze(10)
    for i in range(1, 9):
        maze[1][i] = tu.box('a', 'blue')
        maze[2][i] = tu.box('b', 'green')

    maze[2][2] = tu.box('b', 'blue')
    maze[8][5] = tu.goal('a')

    maze[3][5] = tu.agent(0, 'blue')
    maze[4][5] = tu.agent(1, 'green')

    state = tu.make_state(maze)

    planner = ParallelPlanner(state)
    planner.goal_analyzer.print_storage_spaces()

    storage_plan, expected_state = planner.prepare_storage()
    realizer = ParallelRealizer(state)
    action_plan = realizer.realize_plan(storage_plan)

    result = state
    for a in action_plan:
        print(a)
        #print(state)
        result = result.get_child(a)
        assert result is not None, "something has gone wrong in realization :("

    # TODO: something is wrong with StateMA.__eq__() causing this to not be correct
    if str(result) != str(expected_state):
        print("target:\n", expected_state)
        print("result:\n", result)
        assert False, "something has gone wrong in plan realization or partial plan was never valid :("


def compute_plan_test():
    maze = tu.create_maze(10)
    for i in range(1, 9):
        maze[1][i] = tu.box('a', 'blue')
        maze[2][i] = tu.box('b', 'green')


    maze[8][5] = tu.goal('a')
    maze[7][1] = tu.goal('b')
    maze[8][2] = tu.goal('b')
    maze[8][1] = tu.goal('a')
    maze[8][3] = tu.goal('a')
    maze[7][2] = tu.goal('a')
    maze[6][1] = tu.goal('a')

    maze[3][5] = tu.agent(0, 'blue')
    maze[4][5] = tu.agent(1, 'green')

    maze[8][8] = tu.goal(0, True)
    maze[8][7] = tu.goal(1, True)

    state = tu.make_state(maze)


    planner = ParallelPlanner(state)
    #planner.goal_analyzer.print_storage_spaces()

    action_plan = planner.solve()
    print("Initial:")
    print(state)
    result = state
    count = 0
    for a in action_plan:
        # print(a)
        # print(state)
        old = result
        result = result.get_child(a)
        if result is None:
            print(old)
            print(count)
            print(a)
            old.get_child(a)
            assert False, "something has gone wrong in realization :("
        count += 1

    print("result:\n", result)
    print("solution_length:", len(action_plan))


if __name__ == '__main__':
    #prepare_storage_test()
    #storage_realization_test()
    compute_plan_test()