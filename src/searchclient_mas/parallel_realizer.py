from goal_analyzer import get_neighbours
from state import StateMA
from action import Action,ActionType,Dir
from typing import List
import heapq
import copy


def manhattan_dist(p1, p2):
    return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])

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


class PlanState:
    def __init__(self, time, agent, box, parent=None):
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
        self.latest_update = [[0 for _ in range(state.cols)] for _ in range(state.rows)]
        # TODO: is a dictionary solution better?
        for i in range(state.rows):
            for j in range(state.cols):
                initial[i][j] = state.is_free(i, j)

        self.spaces = [initial]
        self.ignore_set = set()

    def set_position_to_end(self, time_step, pos, val):
        row, col = pos
        for t in range(time_step, len(self.spaces)):
            self.spaces[t][row][col] = val
        if time_step < len(self.spaces):
            self.latest_update[row][col] = time_step

    def set_position(self, t, pos, val):
        row, col = pos
        self.spaces[t][row][col] = val
        if t > self.latest_update[row][col]:
            self.latest_update[row][col] = t

    def changes(self, time_step, pos):
        row, col = pos
        return self.latest_update[row][col] > time_step

    def print_time_step(self, t):
        if t >= len(self.spaces):
            t = len(self.spaces) - 1
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


class ParallelRealizer:
    def __init__(self, state: StateMA):
        self.state = state


    def realize_partial_plan(self, partial_plan: HighLevelPartialPlan, spaces: SpaceTracker, time=0):
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

            # we dont wanna leave the box or agent in spaces that will be occupied in the near future
            # TODO: test if this works properly
            changes = 0
            if spaces.changes(state.time, state.agent):
                changes += 1
            if state.box is not None and spaces.changes(state.time, state.box):
                changes += 1

            if state.box is None or manhattan_dist(state.box, box_target) == 0:
                return steps + 5*manhattan_dist(agent_target, state.agent) + changes
            else:
                dist_to_box = manhattan_dist(state.agent, state.box) - 1
                box_to_goal = manhattan_dist(state.box, box_target) + changes

                # TODO: find some coefficients for the different metrics
                return steps + 5*dist_to_box + 5*box_to_goal + goal_diff

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
                        s = PlanState(timestep, a, agent_pos, state)
                        children.append(s)

            # noop
            children.append(PlanState(timestep, agent_pos, box_pos, state))
            return children


        initial = PlanState(initial_time, agent_curr, box_curr, None)
        pq = []
        seen = set()
        # TODO: all stuff at time_step > spaces is the same in terms of seen

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
                    # if goal_dist < best or goal_dist < 0:
                    #     print(goal_dist,": ",c)
                    #     h = heuristic(c)
                    #     best = goal_dist
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

    def realize_plan(self, high_level_plan: List[HighLevelPartialPlan]):
        agent_free = [0 for _ in self.state.agent_positions]
        action_plan = []
        spaces = SpaceTracker(self.state)

        counter = 0
        for partial in high_level_plan:
            #spaces.print_tracker()
            #print("step:", counter)
            #print("plan:", partial)
            counter += 1
            id = partial.agent_id
            plan = self.realize_partial_plan(partial, spaces, agent_free[id])
            spaces.update(agent_free[id], plan)
            actions = self.plan_to_actions(plan)
            start = agent_free[id]
            end = agent_free[id] + len(actions)
            for i in range(start, end):
                if i == len(action_plan):
                    action_plan.append([None for _ in agent_free])

                action_plan[i][id] = actions[i - start]
            agent_free[id] = end

        return action_plan
