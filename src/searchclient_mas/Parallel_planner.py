from goal_analyzer import GoalAnalyzer, get_neighbours
from queue import Queue
from state import StateMA
from action import Action,ActionType,Dir
import test_utilities as tu
import heapq
import copy

Dir.N = Dir('N', -1,  0)
Dir.S = Dir('S',  1,  0)
Dir.E = Dir('E',  0,  1)
Dir.W = Dir('W',  0, -1)

dir_dict = {
    (-1,0) : Dir.N,
    (1,0) : Dir.S,
    (0,1) : Dir.E,
    (0,-1) : Dir.W,
    (0,0) : None
}
def dir_from_to(frm, to):
    dx = to[0] - frm[0]
    dy = to[1] - frm[1]
    return dir_dict[(dx,dy)]


def manhattan_dist(p1, p2):
    return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])

class PathNode:
    def __init__(self, pos, parent=None):
        self.parent = parent
        self.pos = pos

class PlanState:
    def __init__(self, time, agent, box, parent=None ):
        self.time = time
        self.agent = agent
        self.box = box
        self.parent = parent

    def to_position_list(self):
        li = []
        curr = self
        while curr is not None:
            node = [curr.agent]
            if curr.box is not None:
                node.append(curr.box)
            li.append(node)
            curr = curr.parent

        li.reverse()
        return li

    def __lt__(self, other):
        return False

    def __eq__(self, other):
        if not isinstance(other, PlanState):
            return False
        return self.time == other.time and self.agent == other.agent and self.box == other.box

    def __hash__(self):
        return hash((self.time, self.agent, self.box))

    def __repr__(self):
        return str(self.time) + " a:" + str(self.agent) + " b:" + str(self.box)

class HighLevelPartialPlan:
    def __init__(self, agent_id,  agent_origin, agent_end, box_id=None, box_pos_origin=None, box_pos_end=None):
        self.agent_id = agent_id
        self.agent_origin = agent_origin
        self.agent_end = agent_end
        self.box_id = box_id
        self.box_pos_origin = box_pos_origin
        self.box_pos_end = box_pos_end

    def __repr__(self):
        plan = "agent:" + str(self.agent_origin) + "," + str(self.agent_end)
        if self.box_pos_origin is not None:
            plan += "; " + "box:" + str(self.box_pos_origin) + "," + str(self.box_pos_end)

        return plan

class SpaceTracker:
    def __init__(self, state: StateMA):
        initial = [[False for _ in range(state.cols)] for _ in range(state.rows)]

        # TODO: is a dictionary solution better?
        for i in range(state.rows):
            for j in range(state.cols):
                initial[i][j] = state.is_free(i, j)

        self.spaces = [initial]
        self.ignore_set = set()

    def set_position_to_end(self, time_step, pos, val):
        for t in range(time_step, len(self.spaces)):
            row, col = pos
            self.spaces[t][row][col] = val

    def set_position(self, t, pos, val):
        row, col = pos
        self.spaces[t][row][col] = val

    def print_time_step(self,t):
        step = self.spaces[t]

        s = ""
        for i in step:
            for val in i:
                if not val:
                    s += "+"
                else:
                    s += " "

            s += "\n"
        print(s)


    def update(self, time_step, plan: PlanState):
        # TODO: implement when plan format is decided
        multi_positions = plan.to_position_list()
        # if a plan is only hte initial state we have nothing to do
        if len(multi_positions) < 2:
            return

        initial = multi_positions[0]
        prev = None

        for p in initial:
            self.set_position_to_end(time_step, p, True)

        final = copy.deepcopy(self.spaces[-1])

        for positions in multi_positions:
            if len(self.spaces) <= time_step:
                self.spaces.append(copy.deepcopy(final))

            # find "after image" of agents/boxes moved last turn because they cannot be occupied this turn
            if prev is not None:
                for afterimage in prev:
                    self.set_position(time_step, afterimage, False)

            for pos in positions:
                self.set_position(time_step, pos, False)

            time_step += 1
            prev = positions

        if len(self.spaces) <= time_step:
            self.spaces.append(copy.deepcopy(final))
        # these are the final positions of our plan so objects will be there until end of plan
        for p in positions:
            self.set_position_to_end(time_step, p, False)

        pass

    def print_tracker(self):
        for i in range(len(self.spaces)):
            self.print_time_step(i)

    # sometimes we set stuff to ignore when we know these items have are on the move
    def set_ignore_set(self, items):
        self.ignore_set = set(items)

    def is_free(self, time_step, pos):
        if time_step >= len(self.spaces):
            time_step = len(self.spaces) - 1

        row, col = pos

        return self.spaces[time_step][row][col] or pos in self.ignore_set

    def will_be_free(self, time_step, pos):
        # TODO: returns true if space won't be occupied from given time step until the end, could be useful
        pass


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

        return self.current_plan, self.state



    def realize_high_level_plan(self, partial_plan: HighLevelPartialPlan, spaces: SpaceTracker, time=0):
        agent_curr = partial_plan.agent_origin
        box_curr = partial_plan.box_pos_origin

        agent_target = partial_plan.agent_end
        box_target = partial_plan.box_pos_end

        initial_time = time

        spaces.set_ignore_set([agent_curr, box_curr])

        goal_diff = 0
        if box_target is not None:
            goal_diff = manhattan_dist(agent_target, box_target)

        def heuristic(state: PlanState):

            steps = state.time - initial_time

            if state.box is None or manhattan_dist(state.box, box_target) == 0:
                return steps + manhattan_dist(agent_target, state.agent)
            else:
                dist_to_box = manhattan_dist(state.agent, state.box) - 1
                box_to_goal = manhattan_dist(state.box, box_target)

                # TODO: find some coefficients for the different metrics
                return steps + 2*dist_to_box + 2*box_to_goal + goal_diff

        def get_children(state: PlanState):
            timestep = state.time
            agent_pos = state.agent
            box_pos = state.box
            timestep += 1
            children = []
            # if something is just about to occupy spaces that are already occupied now we know this cant happen
            if not spaces.is_free(timestep, agent_pos):
                return []
            if box_pos is not None and not spaces.is_free(timestep, box_pos):
                return []

            in_range = False
            for a in get_neighbours(agent_pos):
                if a == box_pos:
                    # if a neighbouring space is a box we can perform push and pull actions
                    in_range = True
                    for b in get_neighbours(box_pos):
                        if b != agent_pos and spaces.is_free(timestep, b):
                            children.append(PlanState(timestep, a, b, state))

                elif spaces.is_free(timestep, a):
                    children.append(PlanState(timestep, a, box_pos, state))

            # this means we can perform pull actions
            if in_range:
                for a in get_neighbours(agent_pos):
                    if spaces.is_free(timestep, a) and a != box_pos:
                        children.append(PlanState(timestep, a, agent_pos, state))

            # noop
            children.append(PlanState(timestep, agent_pos, box_pos, state))
            return children


        initial = PlanState(initial_time, agent_curr, box_curr, None)
        pq = []
        seen = set()

        heapq.heappush(pq, (heuristic(initial), initial))
        goal = None
        best = 99999
        #print("target: a:", agent_target , " b:", box_target)
        while pq:
            _, state = heapq.heappop(pq)

            children = get_children(state)
            done = False
            for c in children:
                if c not in seen:
                    seen.add(c)
                    h = heuristic(c)
                    delta = (c.time - initial_time)
                    goal_dist = h - delta
                    if goal_dist < best or goal_dist < 0:
                        print(goal_dist,": ",c)
                        h = heuristic(c)
                        best = goal_dist
                    if goal_dist == 0:
                        done = True
                        goal = c
                        break
                    heapq.heappush(pq, (h, c))
            if done:
                break

        assert goal is not None, "goal should always be completable here"

        return goal

    def state_to_action(self, state: PlanState):
        parent = state.parent

        # agent didnt move then nothing happened so it was noop
        if state.agent == parent.agent:
            return None

        type = ActionType.Move
        # we need to handle box dir separately cause it behaves weird compared to agent
        box_dir = None
        # agent was moved to box position so it must have been a push
        if state.agent == parent.box:
            type = ActionType.Push
            box_dir = dir_from_to(parent.box, state.box)

        # box was moved ontop of agent so it must have been pull
        if state.box == parent.agent:
            type = ActionType.Pull
            box_dir = dir_from_to(parent.agent, parent.box)

        return Action(type, dir_from_to(parent.agent, state.agent), box_dir)

    def plan_to_actions(self, plan: PlanState):

        actions = []
        while plan.parent is not None:
            actions.append(self.state_to_action(plan))
            plan = plan.parent

        # since we are iterating the plan backwards we need to reverse actions before we are done
        actions.reverse()

        return actions

    def realize_current_plan(self):
        state = self.initial_state.copy()
        agent_free = [0 for _ in self.state.agent_positions]
        action_plan = []
        spaces = SpaceTracker(state)

        counter = 0
        for partial in self.current_plan:
            #spaces.print_tracker()
            print("step:", counter)
            print("plan:", partial)
            counter += 1
            id = partial.agent_id
            plan = self.realize_high_level_plan(partial, spaces, agent_free[id])
            spaces.update(agent_free[id], plan)
            actions = self.plan_to_actions(plan)
            start = agent_free[id]
            end = agent_free[id] + len(actions)
            for i in range(start, end):
                if i == len(action_plan):
                    action_plan.append([None for _ in agent_free])

                action_plan[i][id] = actions[i - start]
            agent_free[id] = end

        for a in action_plan:
            print(a)
            print(state)
            state = state.get_child(a)
            assert state is not None, "something hase gone wrong in realization :("

        print("target:\n", self.state)
        print("result:\n", state)

        if self.state == state:
            assert False, "something has gone wrong in plan realization or partial plan was never valid :("

        return action_plan




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

def realizer_test():
    maze = tu.create_maze(10)
    for i in range(1, 9):
        maze[1][i] = tu.box('a', 'blue')
        maze[2][i] = tu.box('b', 'green')

    maze[8][5] = tu.goal('a')

    maze[3][5] = tu.agent(0, 'blue')
    maze[4][5] = tu.agent(1, 'green')

    state = tu.make_state(maze)

    planner = ParallelPlanner(state)
    planner.goal_analyzer.print_storage_spaces()

    storage_plan, storage_state = planner.prepare_storage()
    action_list = planner.realize_current_plan()


from state import StateBuilder

if __name__ == '__main__':
    #prepare_storage_test()
    realizer_test()