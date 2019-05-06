from parallel_realizer import HighLevelPartialPlan
from goal_analyzer import GoalAnalyzer,GoalMetric, get_neighbours
from queue import Queue
from state import StateMA
from parallel_realizer import ParallelRealizer
from collections import defaultdict
from typing import List
from parallel_realizer import manhattan_dist
from storage_estimator import storage_value
import test_utilities as tu
import heapq



class PathNode:
    def __init__(self, pos, parent=None, value=0):
        self.parent = parent
        self.pos = pos
        self.value = value

    def to_set(self):
        res = set()
        curr = self
        while curr is not None:
            res.add(curr.pos)
            curr = curr.parent
        return res

    def __repr__(self):
        return "{}:{}".format(self.pos, self.value)

    def __lt__(self, other):
        return self.value < other.value

class ParallelPlanner:
    def __init__(self, state: StateMA):
        self.initial_state = state
        self.state = state.copy()
        self.goal_analyzer = GoalAnalyzer(state)
        self.current_plan = []
        self.blocked = set()
        self.completed = set()
        self.agent_business = [0 for _ in state.agent_positions]
        self.unusable_boxes = set()
        self.unusable_agents = set()

    def find_boxes_and_agents_for_goal(self, goal_id):
        g_type = self.state.goal_types[goal_id]
        if self.state.goal_agent[goal_id]:
            return ([None], [int(g_type)])
        else:
            viable_boxes = []
            viable_agents = []
            b_color = None
            for i, b_type in enumerate(self.state.box_types):
                if g_type == b_type and i not in self.unusable_boxes:
                    viable_boxes.append(i)
                    b_color = self.state.box_colors[i]

            for i, a_color in enumerate(self.state.agent_colors):
                if a_color == b_color and i not in self.unusable_agents:
                    viable_agents.append(i)

            return viable_boxes, viable_agents

    def get_node_before_position(self, node: PathNode, pos):
        curr = node
        while curr is not None:
            if curr.parent.pos == pos:
                return curr.pos
            curr = curr.parent

    def move_box_to_storage(self, pos, state, forbidden=set(), custom_agent_func=None):
        box_id = state.box_by_cords[pos]
        color = state.box_colors[box_id]
        ignore = {pos}
        # returns true if position is a free storage space
        def check_storage(x):
            return state.is_free(x) and x not in forbidden and storage_value(x, state, ignore, self.blocked) > 80

        # returns true if position is agent at the position is same color as the box
        def check_agent(x):
            if x in state.agent_by_cords:
                agent_id = state.agent_by_cords[x]
                return color == state.agent_colors[agent_id]
            return None

        # check if it is possible to change the orientation of agent around the box at given position
        def check_turning(x):
            neighbors = get_neighbours(x)
            viable = 0

            for n in neighbors:
                if state.is_free(n):
                    viable += 1

            return viable > 2

        # find nearest storage possible
        storage_node = self.find_path_to_condition(pos,state, check_storage)
        if storage_node is None:
            return None

        # find nearest available agent
        agent_node = self.find_path_to_condition(pos, state, check_agent)
        if agent_node is None:
            return None

        agent_pre_box = self.get_node_before_position(agent_node, pos)

        # check if we can orient the agent however we want, this is possible if we can find a layout like this:
        # o
        # xo
        # o
        # where x is a node with 3 or more neighbors
        # so we do 3 checks 1 search from the agent, 1 search from the goal and
        # finally we do 1 simple check at the box position
        can_turn = self.find_path_to_condition(pos, state, check_turning)
        if can_turn is None:
            can_turn = self.find_path_to_condition(storage_node.pos, state, check_turning)
            if can_turn is None:
                if not check_turning(pos):
                    # TODO: we may still be able to come up with a valid plan even if we cant orient the agent
                    return None

        box_final_pos = storage_node.pos
        agent_origin = agent_node.pos

        state.set_box_position(pos, box_final_pos)
        # Temporarily remove agent from the board before deciding where to put him around the box
        state.set_agent_position(agent_origin, box_final_pos)

        ignore = {agent_origin}
        agent_node_finished = self.find_path_to_condition(box_final_pos, state, check_storage)
        if agent_node_finished is None:
            # TODO: this should be a very rare case, we could return the agent back to his origin if we want instead

            # undo state changes made for search
            state.set_agent_position(box_final_pos, agent_origin)
            state.set_box_position(box_final_pos, pos)

            return None

        agent_final = agent_node_finished.pos
        state.set_agent_position(box_final_pos, agent_final)

        return HighLevelPartialPlan(state.agent_by_cords[agent_final], agent_origin, agent_final,
                                                      box_id, pos, box_final_pos)

    def move_agent_to_storage(self, pos, state, forbidden=set()):
        ignore = {pos}

        def check_storage(x):
            return state.is_free(x) and x not in forbidden and storage_value(x, state, ignore, self.blocked) > 80

        # move box to storage may affect agents which is why we must ensure that agent is still present ingiven position
        if pos not in state.agent_by_cords:
            return None

        storage_node = self.find_path_to_condition(pos,state, check_storage)
        if storage_node is None:
            return None

        final_pos = storage_node.pos

        state.set_agent_position(pos, final_pos)

        return HighLevelPartialPlan(state.agent_by_cords[final_pos], pos, final_pos)

    # finds shortest path from pos to a square given by the condition function using BFS
    def find_path_to_condition(self, pos, state, condition) -> PathNode:
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
                if state.in_bounds(n) and condition(n):
                    return PathNode(n, curr)
                if n not in seen and state.is_free(n):
                    seen.add(n)
                    q.put(PathNode(n, curr))

        return None

    def get_not_stored_objects(self, state):
        not_stored = set()
        for i in state.box_positions:
            if not self.goal_analyzer.is_storage(i):
                not_stored.add(("box", i))

        for i in state.agent_positions:
            if not self.goal_analyzer.is_storage(i):
                not_stored.add(("agent", i))

        return not_stored

    def prepare_storage(self):


        plan = []
        state = self.state.copy()
        not_stored = self.get_not_stored_objects(state)
        while len(not_stored) > 0:
            changed = False
            for type, pos in not_stored:
                if type == "box":
                    res = self.move_box_to_storage(pos, state)
                    if res is not None:
                        plan.append(res)
                        changed = True
                if type == "agent":
                    res = self.move_agent_to_storage(pos, state)
                    if res is not None:
                        plan.append(res)
                        changed = True

            if not changed:
                break

            not_stored = self.get_not_stored_objects(state)
        return plan, state

    def get_state_color_info(self):
        color_agents = defaultdict(int)
        for color in self.state.agent_colors:
            color_agents[color] += 1
        color_goals = defaultdict(int)
        type_color_map = {}

        for i in range(len(self.state.box_types)):
            type = self.state.box_types[i]
            color = self.state.box_colors[i]
            type_color_map[type] = color

        for type, agent_goal in zip(self.state.goal_types, self.state.goal_agent):
            if not agent_goal:
                color = type_color_map[type]
                color_goals[color] += 1

        return color_agents, color_goals, type_color_map

    def goal_value(self, gm: GoalMetric):
        val = gm.loss
        if not gm.leaf:
            val += 5
        if gm.agent_goal:
            # we want to complete agent goals as late as possible because agents are mobile
            # and may be required to clear for other goals
            val += 5000
        return val

    def clear_rooms(self, path: set, state: StateMA, room_ids: List[int]):
        # TODO: remove stuff that you need from this room
        # TODO: put useless stuff in this room?
        return state, []

    def required(self, item, is_box=True):
        # TODO: determine if we will need this to be accessible after goal is completed
        return True

    def find_easiest_path(self, begin, end) -> PathNode:
        initial = PathNode(begin)
        pq = []
        # TODO: there may be some inefficiencies here
        heapq.heappush(pq, (initial.value, initial))
        seen = set()
        while pq:
            res = heapq.heappop(pq)
            _, state = res
            if state.pos in seen:
                continue
            if state.pos == end:
                return state
            seen.add(state.pos)

            for n in get_neighbours(state.pos):
                pn = None
                if n in self.blocked:
                    continue
                if self.state.is_free(n):
                    pn = PathNode(n, state, state.value + 1)
                elif n in self.state.agent_by_cords:
                    pn = PathNode(n, state, state.value + 3)  # penalize for passing an agent
                elif n in self.state.box_by_cords:
                    pn = PathNode(n, state, state.value + 18) # penalize for passing a box
                else:
                    continue
                heapq.heappush(pq, (pn.value, pn))
        # This can only happen if the given coordinates are not in the same connected component
        assert False, "agent, goal, box combination was invalid and should never have been considered"
        return None

    def clear_path(self, path, agent, box):
        boxes_in_path = set()
        agents_in_path = set()

        for pos in path:
            if pos in self.state.box_by_cords:
                box_id = self.state.box_by_cords[pos]
                # TODO: in some cases this box may be blocking other agenst from clearing the path so we actually have to move it
                # TODO: in some cases this box may be blocking other agenst from clearing the path so we actually have to move it
                # TODO: in some cases this box may be blocking other agenst from clearing the path so we actually have to move it
                # TODO: in some cases this box may be blocking other agenst from clearing the path so we actually have to move it
                if box_id != box:
                    boxes_in_path.add(box_id)
            elif pos in self.state.agent_by_cords:
                agent_id = self.state.agent_by_cords[pos]
                if agent_id != agent:
                    agents_in_path.add(agent_id)

        plan_index = 0
        current_plan = []
        state = self.state.copy()
        while len(boxes_in_path) > 0 or len(agents_in_path) > 0:
            # update what needs to be done
            while plan_index < len(current_plan):
                pln = current_plan[plan_index]
                if pln.box_id is not None:
                    boxes_in_path.remove(pln.box_id)
                if pln.agent_id in agents_in_path:
                    agents_in_path.remove(pln.agent_id)
                plan_index += 1

            if len(boxes_in_path) == 0 and len(agents_in_path) == 0:
                break

            changed = False
            for b in boxes_in_path:
                box_pos = state.box_positions[b]
                partial_plan = self.move_box_to_storage(box_pos, state, path)
                if partial_plan is not None:
                    changed = True
                    current_plan.append(partial_plan)
                    # TODO: agent assigned to the box may be used to move the undesired box, we must insure that he can return to his path!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                    # TODO: agent assigned to the box may be used to move the undesired box, we must insure that he can return to his path!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                    # TODO: agent assigned to the box may be used to move the undesired box, we must insure that he can return to his path!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                    # TODO: agent assigned to the box may be used to move the undesired box, we must insure that he can return to his path!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                    # TODO: agent assigned to the box may be used to move the undesired box, we must insure that he can return to his path!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                    # TODO: agent assigned to the box may be used to move the undesired box, we must insure that he can return to his path!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                    # TODO: agent assigned to the box may be used to move the undesired box, we must insure that he can return to his path!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                    # TODO: agent assigned to the box may be used to move the undesired box, we must insure that he can return to his path!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                    # TODO: we also want to try to ensure that other agents used to move the boxes can return to the path for more work
            if changed:
                # we wanna retry some boxes before we start trying to move the agents
                continue

            for a in agents_in_path:
                a_pos = state.agent_positions[a]
                partial_plan = self.move_agent_to_storage(a_pos, state, path)
                if partial_plan is not None:
                    changed = True
                    current_plan.append(partial_plan)

            if not changed:
                return None

        return state, current_plan


    def find_goal_plan(self, goal, agent, box):
        plan = []
        path = set()
        agent_pos = self.state.agent_positions[agent]
        goal_pos = self.state.goal_positions[goal]
        box_pos = None
        
        if box is not None:
            box_pos = self.state.box_positions[box]
            if box_pos == goal_pos:
                # TODO: if goal is already solved we may not need to do anything, but we also might want to clear rooms
                return plan

            box_plan = self.find_easiest_path(box_pos, goal_pos)
            agent_plan = self.find_easiest_path(agent_pos, box_pos)
            path = box_plan.to_set().union(agent_plan.to_set())
        else:
            path = self.find_easiest_path(agent_pos, goal_pos).to_set()

        res = self.clear_path(path, agent, box)

        if res is None:
            return None

        state, plan = res

        isolated_rooms = self.goal_analyzer.get_isolated_by_goal_completion(goal, self.completed)

        state, room_clearing_plan = self.clear_rooms(path, state, isolated_rooms)

        plan.extend(room_clearing_plan)

        # these positions may have changed so we need them updated
        agent_pos = state.agent_positions[agent]
        if box_pos is not None:
            box_pos = state.box_positions[box]

        def is_goal(x):
            return x == goal_pos

        def is_box(x):
            return x == box_pos

        # check if it is possible to change the orientation of agent around the box at given position
        def check_turning(x):
            neighbors = get_neighbours(x)
            viable = 0

            for n in neighbors:
                if state.is_free(n):
                    viable += 1

            return viable > 2

        if box is None:
            path = self.find_path_to_condition(agent_pos, state, is_goal)
            if path is None:
                return None
            else:
                plan.append(HighLevelPartialPlan(agent, agent_pos, goal_pos))
                state.set_agent_position(agent_pos, goal_pos)
        else:

            agent_to_box = self.find_path_to_condition(agent_pos,state, is_box)
            if agent_to_box is None:
                return None
            box_to_goal = self.find_path_to_condition(box_pos, state, is_goal)
            if box_to_goal is None:
                # if the box is unable to reach the goal the agent may be able to reach it
                agent_to_goal = self.find_path_to_condition(agent_pos, state, is_goal)
                if agent_to_goal is None:
                    return None

            can_turn = self.find_path_to_condition(goal_pos, state, check_turning)
            if can_turn is None:
                can_turn = self.find_path_to_condition(agent_pos, state, check_turning)
                if can_turn is None:
                    if not check_turning(box_pos):
                        # TODO: we may still be able to come up with a valid plan even if we cant orient the agent
                        return None

            # TODO: check if agent will be useful after goal is complete

            agent_final = None
            for n in get_neighbours(goal_pos):
                if state.is_free(n):
                    agent_final = n
                else:
                    continue
                good = True
                for room in isolated_rooms:
                    if n in self.goal_analyzer.rooms[room]:
                        good = False
                if good:
                    break
            assert agent_final is not None, "something fucky happened when choosing final position for agent after delivering box"
            partial = HighLevelPartialPlan(agent, agent_pos, agent_final, box, box_pos, goal_pos)
            state.set_agent_position(agent_pos, agent_final)
            state.set_box_position(box_pos, goal_pos)
            plan.append(partial)

        self.state = state
        return plan

    def get_agent_order(self, box, agents: List[int]):
        if box is None:
            return agents
        box_pos = self.state.box_positions[box]
        agent_positions = self.state.agent_positions
        agent_costs = [self.agent_business[i] + manhattan_dist(agent_positions[i], box_pos) for i in agents]
        agent_temp = [(i,j) for i,j in zip(agent_costs, agents)]
        return [j for i, j in sorted(agent_temp, key=lambda x: x[0])]


    def complete_goal(self, goal):
        boxes, agents = self.find_boxes_and_agents_for_goal(goal)
        for box in boxes:
            # TODO: sort agents by business and distance
            agents = self.get_agent_order(box, agents)
            for agent in agents:
                goal_plan = self.find_goal_plan(goal, agent, box)
                if goal_plan is not None:
                    return goal_plan
        return None

    def compute_plan(self):

        # TODO: make use of this info and maybe add one for boxes?
        color_agents_left, color_goals_left, type_color_map = self.get_state_color_info()
        complete_plan = []
        while len(self.completed) < len(self.state.goal_positions):
            goal_order = self.goal_analyzer.get_viable_goals(self.completed)
            goal_order = sorted(goal_order, key=self.goal_value)
            goal_order = [g.id for g in goal_order]
            goal_plan = None
            goal = None
            for goal in goal_order:
                goal_plan = self.complete_goal(goal)
                if goal_plan is not None:
                    break

            if goal_plan is None:
                # TODO: by setting some stuff as immovable we may be able to change path choice to one that is solvable
                # TODO: by moving some stuff to storage first we may also be able to solve it
                assert False, "no solution could be found"
            else:
                for p in goal_plan:
                    business = 0
                    if p.box_pos_end is None:
                        business += manhattan_dist(p.agent_origin, p.agent_end)
                    else:
                        business += manhattan_dist(p.agent_origin, p.box_pos_origin)
                        business += manhattan_dist(p.box_pos_origin, p.box_pos_end)
                        business += manhattan_dist(p.box_pos_end, p.agent_end)
                    self.agent_business[p.agent_id] += business
                complete_plan.extend(goal_plan)
                blocked_rooms = self.goal_analyzer.get_isolated_by_goal_completion(goal, self.completed)
                for room in blocked_rooms:
                    room_verts = self.goal_analyzer.rooms[room]
                    self.blocked = self.blocked.union(room_verts)
                self.completed.add(goal)
                self.blocked.add(self.state.goal_positions[goal])
                goal_pos = self.state.goal_positions[goal]
                if self.state.goal_agent[goal]:
                    self.unusable_agents.add(self.state.agent_by_cords[goal_pos])
                else:
                    self.unusable_boxes.add(self.state.box_by_cords[goal_pos])
                #print(self.state)

        return complete_plan

    def solve(self):
        plan = self.compute_plan()
        realizer = ParallelRealizer(self.initial_state)
        return realizer.realize_plan(plan)


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