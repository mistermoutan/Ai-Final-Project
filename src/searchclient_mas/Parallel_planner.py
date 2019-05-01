from parallel_realizer import HighLevelPartialPlan
from goal_analyzer import GoalAnalyzer, get_neighbours
from queue import Queue
from state import StateMA
from parallel_realizer import ParallelRealizer
import test_utilities as tu
import heapq

class PathNode:
    def __init__(self, pos, parent=None):
        self.parent = parent
        self.pos = pos

class ParallelPlanner:
    def __init__(self, state: StateMA):
        self.initial_state= state
        self.state = state.copy()
        self.goal_analyzer = GoalAnalyzer(state)
        self.goal_order = self.goal_analyzer.compute_goal_order_plan()
        self.immovable_boxes = {}
        self.current_plan = []

    def find_box_and_agent_for_goal(self, goal_id):
        # TODO: do this in a smart way, not at random
        # TODO: have this function yield? in case found match is not usable we don't wanna restart the process
        # TODO: what if this is agent goal?
        g_type = self.state.goal_types[goal_id]
        viable_boxes = []

        for b_type, i in enumerate(self.state.box_types):
            if i in self.immovable_boxes:
                continue
            if g_type == b_type:
                viable_boxes.append(i)

        for a_color, i in enumerate(self.state.agent_colors):
            for b in viable_boxes:
                if self.state.box_colors[b] == self.state.agent_colors:
                    return b, i

    def get_node_before_position(self, node: PathNode, pos):
        curr = node
        while curr is not None:
            if curr.parent.pos == pos:
                return curr.pos
            curr = curr.parent

    def move_box_to_storage(self, pos):
        box_id = self.state.box_by_cords[pos]
        color = self.state.box_colors[box_id]

        # returns true if position is a free storage space
        def check_storage(x):
            return self.state.is_free(x) and self.goal_analyzer.is_storage(x)

        # returns true if position is agent at the position is same color as the box
        def check_agent(x):
            if x in self.state.agent_by_cords:
                agent_id = self.state.agent_by_cords[x]
                return color == self.state.agent_colors[agent_id]
            return False

        # check if it is possible to change the orientation of agent around the box at given position
        def check_turning(x):
            neighbors = get_neighbours(x)
            viable = 0

            for n in neighbors:
                if self.state.is_free(n):
                    viable += 1

            return viable > 2

        # find nearest storage possible
        storage_node = self.find_path_to_condition(pos, check_storage)
        if storage_node is None:
            return False

        # find nearest available agent
        agent_node = self.find_path_to_condition(pos, check_agent)
        if agent_node is None:
            return False

        agent_pre_box = self.get_node_before_position(agent_node, pos)

        # check if we can orient the agent however we want
        can_turn = self.find_path_to_condition(pos, check_turning)
        if can_turn is None:
            # TODO: we may still be able to come up with a valid plan even if we cant orient the agent
            return False

        box_final_pos = storage_node.pos
        agent_origin = agent_node.pos

        self.state.set_box_position(pos, box_final_pos)
        # Temporarily remove agent from the board before deciding where to put him around the box
        self.state.set_agent_position(agent_origin, box_final_pos)

        agent_node_finished = self.find_path_to_condition(box_final_pos, check_storage)
        if agent_node_finished is None:
            # TODO: this should be a very rare case, we could return the agent back to his origin if we want instead

            # undo state changes made for search
            self.state.set_agent_position(box_final_pos, agent_origin)
            self.state.set_box_position(box_final_pos, pos)

            return False

        agent_final = agent_node_finished.pos
        self.state.set_agent_position(box_final_pos, agent_final)

        self.current_plan.append(HighLevelPartialPlan(self.state.agent_by_cords[agent_final], agent_origin, agent_final,
                                                      box_id, pos, box_final_pos))

        return True

    def move_agent_to_storage(self, pos):

        def check_storage(x):
            return self.state.is_free(x) and self.goal_analyzer.is_storage(x)

        # move box to storage may affect agents which is why we must ensure that agent is still present ingiven position
        if pos not in self.state.agent_by_cords:
            return False

        storage_node = self.find_path_to_condition(pos, check_storage)
        if storage_node is None:
            return False

        final_pos = storage_node.pos

        self.current_plan.append(HighLevelPartialPlan(self.state.agent_by_cords[pos], pos, final_pos))
        self.state.set_agent_position(pos, final_pos)

        return True

    # finds shortest path from pos to a square given by the condition function using BFS
    def find_path_to_condition(self, pos, condition) -> PathNode:
        seen = set()
        seen.add(pos)
        start_node = PathNode(pos, None)

        q = Queue()
        q.put(start_node)

        # use BFS to find nearest available storage node
        while not q.empty():
            curr = q.get()
            neighbors = get_neighbours(curr.pos)
            for n in neighbors:
                if self.state.in_bounds(n) and condition(n):
                    return PathNode(n, curr)
                if n not in seen and self.state.is_free(n):
                    seen.add(n)
                    q.put(PathNode(n, curr))

        return None

    def get_not_stored_objects(self):
        not_stored = set()
        for i in self.state.box_positions:
            if not self.goal_analyzer.is_storage(i):
                not_stored.add(("box", i))

        for i in self.state.agent_positions:
            if not self.goal_analyzer.is_storage(i):
                not_stored.add(("agent", i))

        return not_stored

    def prepare_storage(self):

        not_stored = self.get_not_stored_objects()

        while len(not_stored) > 0:
            changed = False
            for type, pos in not_stored:
                if type == "box":
                    changed = changed | self.move_box_to_storage(pos)
                if type == "agent":
                    changed = changed | self.move_agent_to_storage(pos)

            if not changed:
                break

            not_stored = self.get_not_stored_objects()
        # TODO: clean this up to not use internal structures?
        return self.current_plan, self.state


    def compute_plan(self):
        # TODO : change this to work for agent goals

        for goal in self.goal_order:
            #TODO: move required stuff out of rooms getting blocked when goal is completed
            box, agent = self.find_box_and_agent_for_goal(goal)

            path = self.find_path_to_box(agent, box)
            # TODO: for item on path move to storage

            path = self.move_box_to_goal(agent, box, goal)
            # TODO: for item on path move to storage



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

    # TODO: something is wrong with StateMa.__eq__() causing this to not be correct
    if str(result) != str(expected_state):
        print("target:\n", expected_state)
        print("result:\n", result)
        assert False, "something has gone wrong in plan realization or partial plan was never valid :("



from state import StateBuilder

if __name__ == '__main__':
    #prepare_storage_test()
    storage_realization_test()