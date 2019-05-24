from parallel_realizer import HighLevelPartialPlan
from goal_analyzer import GoalAnalyzer,GoalMetric, get_neighbours
from queue import Queue
from state import StateMA
from parallel_realizer import ParallelRealizer
from collections import defaultdict
from typing import List
import time
#from parallel_realizer import manhattan_dist
from storage_estimator import storage_value, print_state_storage_values
from level_analyser import LevelAnalyser
from distance_comp import DistanceComputer
import test_utilities as tu
import heapq
import sys



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
        self.state = state
        self.level_analyzer = LevelAnalyser(state)

        while self.level_analyzer.useless_boxes:
            self.state.remove_immovable_boxes(self.level_analyzer.useless_boxes)
            self.level_analyzer = LevelAnalyser(self.state)

        self.initial_state = state.copy()
        self.goal_analyzer = GoalAnalyzer(state)
        self.current_plan = []
        self.blocked = set()
        self.completed = set()
        self.agent_business = [0 for _ in state.agent_positions]

        self.solo_agents = [False for _ in range(len(self.state.agent_positions))]
        for i in range(len(self.state.agent_positions)):
            room = self.level_analyzer.room_of_agent[i]
            if len(self.level_analyzer.agents_per_room[room]) == 1:
                self.solo_agents[i] = True

        self.unusable_boxes = set()
        self.unusable_agents = set()
        self.dist = DistanceComputer(self.state)
        self.color_dict = defaultdict(set)

        for c,t in zip(self.state.box_colors, self.state.box_types):
            self.color_dict[c].add(t)

        self.discouraged_storage = defaultdict(int)


    def find_boxes_and_agents_for_goal(self, goal_id):
        g_type = self.state.goal_types[goal_id]
        relevant_boxes, relevant_agents = self.level_analyzer.get_relevant_elements_to_goals(goal_id)
        if self.state.goal_agent[goal_id]:
            return ([None], [int(g_type)])
        else:
            viable_boxes = []
            viable_agents = []
            b_color = None
            for i, b_type in enumerate(self.state.box_types):
                if g_type == b_type and i not in self.unusable_boxes and i in relevant_boxes:
                    viable_boxes.append(i)
                    b_color = self.state.box_colors[i]

            for i, a_color in enumerate(self.state.agent_colors):
                if a_color == b_color and i not in self.unusable_agents and i in relevant_agents:
                    if self.state.agent_positions[i] not in self.blocked:
                        viable_agents.append(i)

            g_pos = self.state.goal_positions[goal_id]
            viable_boxes.sort(key=lambda x: self.dist.dist(g_pos, self.state.box_positions[x]))
            return viable_boxes, viable_agents

    def get_node_before_position(self, node: PathNode, pos):
        curr = node
        while curr is not None:
            if curr.parent.pos == pos:
                return curr.pos
            curr = curr.parent

    def move_box_to_storage(self, pos, state, forbidden=set(), allowed_agent=-1):
        """
        :param pos: current position of the box
        :param state: the current state (will get changed if we successfully store the box)
        :param forbidden: set of vertices that should not be considered for storage
        :return: High level partial plan storing the box if it is possible in current position
        """
        box_id = state.box_by_cords[pos]
        color = state.box_colors[box_id]
        ignore = {pos}

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
            if not state.is_free(x) and x not in ignore:
                return False
            for n in neighbors:
                if state.is_free(n) or n in ignore:
                    viable += 1

            return viable > 2

        def is_free(x):
            return x in ignore or state.is_free(x)

        # find nearest available agent
        agent_node = self.find_path_to_condition(pos, state, check_agent)
        if agent_node is None:
            return None

        agent_origin = agent_node.pos
        ignore.add(agent_origin)

        # find nearest storage possible
        storage_node = self.find_path_to_storage(pos, state, True, ignore, forbidden,cutoff=90)
        if storage_node is None:
            return None


        agent_pre_box = self.get_node_before_position(agent_node, pos)

        # check if we can orient the agent however we want, this is possible if we can find a layout like this:
        # o
        # xo
        # o
        # where x is a node with 3 or more neighbors
        # so we do 3 checks 1 search from the agent, 1 search from the goal and
        # finally we do 1 simple check at the box position
        can_turn = self.find_path_to_condition(pos, state, check_turning, is_free)
        box_final_pos = storage_node.pos
        if can_turn is None:
            path = storage_node.to_set()
            push = True
            if agent_origin in path:
                push = False
            agent_final = None
            ignorable = set()
            ignorable.add(pos)
            ignorable.add(agent_origin)
            for n in get_neighbours(box_final_pos):
                if n in ignorable or state.is_free(n):
                    if push and n in path:
                        agent_final = n
                    elif not push and n not in path:
                        agent_final = n


            if agent_final is None:
                return None
            else:
                state.set_box_position(pos, box_final_pos)
                state.set_agent_position(agent_origin, agent_final)
                return HighLevelPartialPlan(state.agent_by_cords[agent_final], agent_origin, agent_final,
                                                      box_id, pos, box_final_pos)

        state.set_box_position(pos, box_final_pos)
        # Temporarily remove agent from the board before deciding where to put him around the box
        state.set_agent_position(agent_origin, box_final_pos)

        ignore = {agent_origin}
        # box may have been placed where agent is currently in which case we cannot ignore it
        if box_final_pos in ignore:
            ignore.remove(box_final_pos)

        if allowed_agent == state.agent_by_cords[box_final_pos]:
            forbidden = set()
        agent_node_finished = self.find_path_to_storage(box_final_pos, state, False, ignore, forbidden, cutoff=80)
        #self.level_analyzer.room_of_agent[]
        if agent_node_finished is None:

            # undo state changes made for search
            state.set_agent_position(box_final_pos, agent_origin)
            state.set_box_position(box_final_pos, pos)

            return None
        agent_id = state.agent_by_cords[box_final_pos]
        agent_final = agent_node_finished.pos

        # if the agent is alone in the room, he has no need to go to storage
        if self.solo_agents[agent_id]:
            while agent_node_finished.pos != box_final_pos:
                agent_final = agent_node_finished.pos
                agent_node_finished = agent_node_finished.parent

        state.set_agent_position(box_final_pos, agent_final)

        return HighLevelPartialPlan(state.agent_by_cords[agent_final], agent_origin, agent_final,
                                                      box_id, pos, box_final_pos)

    def move_agent_to_storage(self, pos, state, forbidden=set()):
        ignore = {pos}

        # move box to storage may affect agents which is why we must ensure that agent is still present ingiven position
        if pos not in state.agent_by_cords:
            return None

        storage_node = self.find_path_to_storage(pos, state, False, ignore, forbidden)
        if storage_node is None:
            return None

        final_pos = storage_node.pos

        state.set_agent_position(pos, final_pos)

        return HighLevelPartialPlan(state.agent_by_cords[final_pos], pos, final_pos)

    def find_path_to_storage(self, pos, state, is_box=True, empty=set(), forbidden=set(), cutoff=80):
        seen = set()
        seen.add(pos)
        start_node = PathNode(pos, None)

        q = Queue()
        q.put(start_node)
        best_val = -100
        best_node = None
        # use BFS to find nearest/best available storage node
        while not q.empty():
            curr = q.get()
            neighbors = get_neighbours(curr.pos)
            for n in neighbors:
                if n in seen:
                    continue
                if state.is_free(n) or n in empty:
                    node = PathNode(n, curr)
                    if n not in forbidden:
                        # TODO: what if we are sitting on top of a goal?
                        # TODO: balance this with distance somehow?
                        # TODO: reduce value if going to be blocked and is needed
                        value = storage_value(n, state, empty, self.blocked, is_box) - self.discouraged_storage[n]*2
                        if value >= cutoff:
                            return node
                        if value > best_val:
                            best_val = value
                            best_node = node

                    seen.add(n)
                    q.put(node)

        return best_node



    # finds shortest path from pos to a square given by the condition function using BFS
    def find_path_to_condition(self, pos, state, condition, is_free=None) -> PathNode:
        if is_free is None:
            is_free = state.is_free
        if condition(pos):
            return PathNode(pos, None)
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
                if n not in seen and is_free(n):
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

    def goal_value(self, gm: GoalMetric):
        val = gm.storage_loss
        if not gm.leaf:
            val += 5
        if gm.agent_goal:
            # we want to complete agent goals as late as possible because agents are mobile
            # and may be required to clear for other goals
            val += 5000
        return val

    def clear_rooms(self, path: set, state: StateMA, room_ids: List[int],goal,agent,box):
        #rooms that will be blocked off

        if len (room_ids) == 0:
            return state, []

        rooms_to_be_deleted = [self.goal_analyzer.rooms[room_id] for room_id in room_ids]
        rooms_to_be_deleted = set.union(*rooms_to_be_deleted) 

        needed_box_types, needed_agent_ids, needed_agents_colors = self.level_analyzer.salvage_elements(rooms_to_be_deleted, state)
        all_agent_ids, all_box_ids = set(), set() # of rooms to be deleted
        total_boxes_needed = sum([value for value in needed_box_types.values()])
        #print('\n',needed_box_types, needed_agent_ids, needed_agents_colors,'\n',file=sys.stderr, flush=True)

        for vertex in rooms_to_be_deleted:
            if vertex in state.agent_by_cords:
                all_agent_ids.add(state.agent_by_cords[vertex])
            elif vertex in state.box_by_cords:
                all_box_ids.add(state.box_by_cords[vertex])

        # if our goal box is in here we dont wanna move it out of the room
        if box in all_box_ids:
            all_box_ids.remove(box)
            b_type = state.box_types[box]
            if needed_box_types[b_type] > 0:
                needed_box_types[b_type] -= 1
                total_boxes_needed -= 1

        color_changed = True
        illegal = path.union(rooms_to_be_deleted)
        done_boxes = set()
        current_plan = []

        while total_boxes_needed > 0 and color_changed:
            color_changed = False
            for box_id in all_box_ids:
                box_type = state.box_types[box_id]
                if needed_box_types[box_type] > 0:
                    pos = state.box_positions[box_id]
                    partial_plan = self.move_box_to_storage(pos,state,illegal)
                    if partial_plan:
                        needed_box_types[box_type] -= 1
                        total_boxes_needed -= 1
                        color_changed = True
                        current_plan.append(partial_plan)
                        done_boxes.add(box_id)
                else:
                    done_boxes.add(box_id)

            all_box_ids -= done_boxes
            done_boxes = set()
            if not color_changed:
            #TODO: fall back strategy when can't move boxes
                return None

        assert sum([value for value in needed_box_types.values()]) == 0, "Did  not get all needed boxes"

        # TODO: improve this:
        for pos in rooms_to_be_deleted:
            if pos in state.agent_by_cords:
                a_id = state.agent_by_cords[pos]
                if a_id != agent:
                    partial = self.move_agent_to_storage(pos, state, illegal)
                    if partial is not None:
                        current_plan.append(partial)


        return state, current_plan
        #return state, must_salvage_elements
                
        # TODO: remove stuff that you need from this room
        # TODO: put useless stuff in this room?

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
        print(self.state, file=sys.stderr, flush=True)
        assert False, "agent, goal, box combination was invalid and should never have been considered"

    def get_items_in_path(self,path, agent=None, box=None):
        boxes_in_path = set()
        agents_in_path = set()

        for pos in path:
            if pos in self.state.box_by_cords:
                box_id = self.state.box_by_cords[pos]
                if box_id != box:
                    boxes_in_path.add(box_id)
            elif pos in self.state.agent_by_cords:
                agent_id = self.state.agent_by_cords[pos]
                if agent_id != agent:
                    agents_in_path.add(agent_id)

        return boxes_in_path, agents_in_path

    def clear_path(self, path, agent=None, box=None, goal_pos=None):

        boxes_in_path, agents_in_path = self.get_items_in_path(path, agent, box)
        plan_index = 0
        current_plan = []
        state = self.state.copy()

        agent_moved = agent is None
        box_moved = box is None
        agent_target = goal_pos
        if not box_moved:
            agent_target = state.box_positions[box]

        while len(boxes_in_path) > 0 or len(agents_in_path) > 0:
            # update what needs to be done
            while plan_index < len(current_plan):
                pln = current_plan[plan_index]
                if pln.box_id is not None and pln.box_id in boxes_in_path:
                    boxes_in_path.remove(pln.box_id)
                if pln.agent_id in agents_in_path:
                    agents_in_path.remove(pln.agent_id)
                plan_index += 1

            if len(boxes_in_path) == 0 and len(agents_in_path) == 0:
                break

            changed = False
            for b in boxes_in_path:
                box_pos = state.box_positions[b]
                if agent is not None:
                    partial_plan = self.move_box_to_storage(box_pos, state, path, allowed_agent=agent)
                    #agent_moved = False # after moving box to storage agent may be put in the path again blockign other agents
                else:
                    partial_plan = self.move_box_to_storage(box_pos, state, path)

                if partial_plan is not None:
                    changed = True
                    current_plan.append(partial_plan)
                    # TODO: agent assigned to the goal box may be used to move the undesired box, we must ensure that he can return to his path!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                    # TODO: agent assigned to the goal box may be used to move the undesired box, we must ensure that he can return to his path!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                    # TODO: agent assigned to the goal box may be used to move the undesired box, we must ensure that he can return to his path!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                    # TODO: agent assigned to the goal box may be used to move the undesired box, we must ensure that he can return to his path!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                    # TODO: agent assigned to the goal box may be used to move the undesired box, we must ensure that he can return to his path!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
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
                if goal_pos is None:
                    #TODO: figure out what to do here if anything
                    return None
                if agent is not None:
                    agent_pos = state.agent_positions[agent]
                    agent_reach = self.reachable(agent_pos, agent_target)
                    if not agent_reach:
                        if agent_moved:
                            return None
                        agent_moved = True
                        res = self.move_agent_to_storage(agent_pos, state)
                        if res is None:
                            return None
                        changed = True
                        current_plan.append(res)
                    else:
                        if box_moved:
                            return None
                        box_moved = True
                        # TODO: this may actually pick a different agent to do the work which would be more suitable
                        # for solving the goal and might fuck us up
                        res = self.move_box_to_storage(state.box_positions[box], state)
                        if res is None:
                            return None
                        changed = True
                        current_plan.append(res)

                    # now the agent and/or the box has been moved so we must recompute the path
                    path = self.find_easiest_path(agent_pos, agent_target).to_set()
                    if box is not None:
                        box_pos = state.box_positions[box]
                        path = path.union(self.find_easiest_path(box_pos, goal_pos).to_set())
                    boxes_in_path, agents_in_path = self.get_items_in_path(path, agent, box)


                    continue

                # TODO: maybe the box/agent itself is blocking the path so we should move it out of the way
                return None
        return state, current_plan

    def need_to_clear_rooms(self, goal):
        room_ids = self.goal_analyzer.get_isolated_by_goal_completion(goal, self.completed)
        if len(room_ids) == 0:
            return False
        rooms_to_be_deleted = [self.goal_analyzer.rooms[room_id] for room_id in room_ids]
        rooms_to_be_deleted = set.union(*rooms_to_be_deleted)
        needed_box_types, needed_agent_ids, needed_agents_colors = self.level_analyzer.salvage_elements(
            rooms_to_be_deleted, self.state)

        for v in needed_box_types.values():
            if v > 0:
                return True
        if len(needed_agent_ids) > 0 or len(needed_agents_colors) > 0:
            return True
        return False

    def find_goal_plan(self, goal, agent, box):
        plan = []
        path = set()
        agent_pos = self.state.agent_positions[agent]
        goal_pos = self.state.goal_positions[goal]
        box_pos = None

        if box is not None:
            box_pos = self.state.box_positions[box]
            if box_pos == goal_pos:
                if not self.need_to_clear_rooms(goal):
                    return plan

            box_plan = self.find_easiest_path(box_pos, goal_pos)
            agent_plan = self.find_easiest_path(agent_pos, box_pos)
            path = box_plan.to_set().union(agent_plan.to_set())
        else:
            path = self.find_easiest_path(agent_pos, goal_pos).to_set()

        # TODO: While clearing path we might have to disallow the agent to return to it by setting the agent parameter as -1
        # TODO: While clearing path we might have to disallow the agent to return to it by setting the agent parameter as -1
        # TODO: While clearing path we might have to disallow the agent to return to it by setting the agent parameter as -1
        # TODO: While clearing path we might have to disallow the agent to return to it by setting the agent parameter as -1
        res = self.clear_path(path, agent, box, goal_pos)
        if res is None:
            # this is a shitty fix that might not always work
            res = self.clear_path(path, None, box, goal_pos)
            if res is None:
                return None

        state, plan = res
        isolated_rooms = self.goal_analyzer.get_isolated_by_goal_completion(goal, self.completed)

        # TODO: if the path has changed during clearing we need to recompute it here
        # TODO: if the path has changed during clearing we need to recompute it here
        # TODO: if the path has changed during clearing we need to recompute it here
        # TODO: if the path has changed during clearing we need to recompute it here
        # TODO: if the path has changed during clearing we need to recompute it here
        # TODO: if the path has changed during clearing we need to recompute it here
        
        res = self.clear_rooms(path, state, isolated_rooms,goal,agent,box)

        if res is None:
            return None

        state, room_clearing_plan = res

        plan.extend(room_clearing_plan)
        
        # these positions may have changed so we need them updated
        agent_pos = state.agent_positions[agent]
        if box_pos is not None:
            box_pos = state.box_positions[box]

        def is_goal(x):
            return x == goal_pos

        def is_box(x):
            return x == box_pos
        ignorable = {box_pos, agent_pos}
        # check if it is possible to change the orientation of agent around the box at given position
        def check_turning(x):
            neighbors = get_neighbours(x)
            viable = 0

            if not state.is_free(x) and x not in ignorable:
                return False
            for n in neighbors:
                if state.is_free(n) or n in ignorable:
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
            agent_to_goal = None
            if box_to_goal is None:
                # if the box is unable to reach the goal the agent may be able to reach it
                pushable = False
                agent_to_goal = self.find_path_to_condition(agent_pos, state, is_goal)
                if agent_to_goal is None:
                    return None
            else:
                agent_path_set = agent_to_box.to_set()
                if goal_pos not in agent_path_set:
                    pushable = state.is_free(box_to_goal.pos) or box_pos == goal_pos
                else:
                    pushable = False

            can_turn = self.find_path_to_condition(goal_pos, state, check_turning)
            if can_turn is None:
                can_turn = self.find_path_to_condition(agent_pos, state, check_turning)
                if can_turn is None:
                    can_turn = self.find_path_to_condition(box_pos, state, check_turning)

            agent_final = None

            if can_turn is None:
                if pushable:
                    agent_final = box_to_goal.parent.pos
                elif agent_to_box is not None and agent_to_goal is not None:
                    box_path = agent_to_box.to_set().union(agent_to_goal.to_set())

                    g_neighbors = get_neighbours(goal_pos)
                    for n in g_neighbors:
                        if n not in box_path and self.state.is_free(n):
                            agent_final = n
                    if agent_final is None:
                        return None
                else:
                    return None

            # TODO: check if agent will be useful after goal is complete/ check which space is best storage
            # if agent_final is None:
            #     for n in get_neighbours(goal_pos):
            #         if state.is_free(n) or n == box_pos or n == agent_pos:
            #             agent_final = n
            #         else:
            #             continue
            #         good = True
            #         for room in isolated_rooms:
            #             if n in self.goal_analyzer.rooms[room]:
            #                 good = False
            #         if good:
            #             break

            if agent_final is None:
                best = None
                for n in get_neighbours(goal_pos):
                    if state.is_free(n) or n == box_pos or n == agent_pos:
                        agent_final = n
                    else:
                        continue
                    good = True
                    for room in isolated_rooms:
                        if n in self.goal_analyzer.rooms[room]:
                            good = False
                    if good and n not in self.state.goal_by_cords:
                        best = n
                        break
                    elif good:
                        if best is None:
                            best = n
                if best is not None:
                    agent_final = best

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
        agent_costs = [self.agent_business[i] + self.dist.dist(agent_positions[i], box_pos) for i in agents]
        agent_temp = [(i,j) for i,j in zip(agent_costs, agents)]
        return [j for i, j in sorted(agent_temp, key=lambda x: x[0])]


    def complete_goal(self, goal):
        boxes, agents = self.find_boxes_and_agents_for_goal(goal)
        for box in boxes:
            agents = self.get_agent_order(box, agents)
            for agent in agents:
                goal_plan = self.find_goal_plan(goal, agent, box)
                if goal_plan is not None:
                    return goal_plan

        return self.complete_goal_aggressive(goal)

    def complete_goal_aggressive(self, goal):
        boxes, agents = self.find_boxes_and_agents_for_goal(goal)
        for box in boxes:
            agents = self.get_agent_order(box, agents)
            for agent in agents:
                goal_plan = self.find_goal_plan(goal, agent, box)
                if goal_plan is not None:
                    return goal_plan


        original_state = self.state.copy()
        # we could not complete the goal but we might after we unblock some agents
        unblocking_plan = self.mediocre_unblock_agents(goal)

        if unblocking_plan is None:
            # we are fucked here
            return None

        for box in boxes:
            agents = self.get_agent_order(box, agents)
            for agent in agents:
                goal_plan = self.find_goal_plan(goal, agent, box)
                if goal_plan is not None:
                    unblocking_plan.extend(goal_plan)
                    return unblocking_plan

        # we are fucked here
        self.state = original_state
        return None

    def reachable(self, frm, to, state: StateMA=None, ignore_boxes=False, ignore_agents=False):
        if state is None:
            state = self.state
        # TODO: use A* instead of bfs
        q = Queue()
        seen = set()
        q.put(frm)

        # use BFS to find nearest available storage node
        while not q.empty():
            curr = q.get()
            neighbors = get_neighbours(curr)
            for n in neighbors:
                if n == to:
                    return True
                if state.in_bounds(n) and n not in seen:
                    seen.add(n)
                    if not ignore_boxes and n in state.box_by_cords:
                        continue
                    elif not ignore_agents and n in state.agent_by_cords:
                        continue
                    else:
                        q.put(n)

    def mediocre_unblock_agents(self, goal):
        room_id = 0
        # this is just so that the level analyzer is initialized
        _ = self.level_analyzer.get_relevant_elements_to_goals(goal)
        goal_pos = self.state.goal_positions[goal]
        for i, spaces in enumerate(self.level_analyzer.rooms):
            if goal_pos in spaces:
                room_id = i
                break

        room_agents = list(self.level_analyzer.agents_per_room[room_id])
        if len(room_agents) <= 1:
            # nothing to be done
            return None

        # TODO: is this the ideal number?
        max_count = 10
        clearing_plan = []
        #TODO: maybe we wanna restore this state?
        changed = True
        agent_set_dict = dict()
        for i in room_agents:
            agent_set_dict[i] = {i}
        while changed and max_count > 0:
            changed = False
            for a1 in room_agents:
                for a2 in room_agents:
                    if a1 in agent_set_dict[a2]:
                        continue
                    a1_pos = self.state.agent_positions[a1]
                    a2_pos = self.state.agent_positions[a2]
                    if a1_pos in self.blocked or a2_pos in self.blocked:
                        continue
                    if self.reachable(a1_pos, a2_pos, self.state, ignore_agents=True):
                        res = (self.state, [])
                    else:
                        path = self.find_easiest_path(a1_pos, a2_pos)
                        # we don't really wanna clear the agent itself
                        path = path.parent
                        res = self.clear_path(path.to_set(), a1)
                    if res is not None:
                        state, plan = res
                        clearing_plan.extend(plan)
                        self.state = state
                        changed = True
                        union = agent_set_dict[a2].union(agent_set_dict[a1])
                        for i in union:
                            agent_set_dict[i] = union
            max_count -= 1

        if len(clearing_plan) == 0:
            return None
        return clearing_plan


    def rearranged_free_goals(self, goals: List[GoalMetric]):
        freebies = [g for g in goals if g.true_loss == 0 and not g.agent_goal]

        if len(freebies) == 0:
            return []
        else:
            agent_room = self.level_analyzer.room_of_agent
            for g in freebies:
                g.storage_loss = 99999
                g_type = self.state.goal_types[g.id]
                g_pos = self.state.goal_positions[g.id]
                g_room = self.level_analyzer.room_of_goal[g.id]
                for i, c in enumerate(self.state.agent_colors):
                    if g_type in self.color_dict[c] and i in agent_room and agent_room[i] == g_room:
                        a_pos = self.state.agent_positions[i]
                        # path = self.find_easiest_path(a_pos, g_pos)
                        # dist = path.value + self.agent_business[i]
                        dist = self.dist.dist(g_pos, a_pos) + self.agent_business[i]
                        if dist < g.storage_loss:
                            g.storage_loss = dist


            freebies.sort(key=lambda x: x.storage_loss)
            return [g for g in freebies]

    def do_desperate_random_shit(self):
        random_plan = []
        # TODO: import random? and randomly select boxes to be moved?
        for b, b_pos in enumerate(self.state.box_positions):
            if b_pos in self.blocked:
                continue

            forbidden = set()
            forbidden.add(b_pos)
            partial = self.move_box_to_storage(b_pos, self.state, forbidden)
            if partial is not None:
                random_plan.append(partial)
                self.discouraged_storage[partial.box_pos_end] += 1

        for a, a_pos in enumerate(self.state.agent_positions):
            if a_pos in self.blocked:
                continue

            forbidden = set()
            forbidden.add(a_pos)
            partial = self.move_agent_to_storage(a_pos, self.state, forbidden)
            if partial is not None:
                random_plan.append(partial)
        return random_plan


    def compute_plan(self):
        complete_plan = []
        rand_max = 10
        ScrambledInitial = False
        while len(self.completed) < len(self.state.goal_positions):
            # print(len(self.completed), file=sys.stderr,flush=True)
            viable_goals = self.goal_analyzer.get_viable_goals(self.completed)
            freebies = self.rearranged_free_goals(viable_goals)
            if len(freebies) > 0:
                goal_order = [g.id for g in freebies]
            else:
                goal_order = sorted(viable_goals, key=self.goal_value)
                goal_order = [g.id for g in goal_order]
            goal_plan = None
            goal = None
            for goal in goal_order:
                #unblock_plan = self.unblock_agents(goal)
                #assert False, "delete the line above to run properly, this is jsut for debu"
                goal_plan = self.complete_goal_aggressive(goal)
                if goal_plan is not None:
                    break

            if goal_plan is None:
                # TODO: by setting some stuff as immovable we may be able to change path choice to one that is solvable
                # TODO: by moving some stuff to storage first we may also be able to solve it
                #print(self.state, file=sys.stderr, flush=True)
                rand_max -= 1
                random_plan = self.do_desperate_random_shit()
                if len(random_plan) > 0 and rand_max >= 0:
                    #sys.stdout.write("# could not finish the plan :(, trying random stuff\n")
                    #print(self.state)
                    #sys.stdout.flush()
                    complete_plan.extend(random_plan)
                else:
                    if ScrambledInitial or len(complete_plan) == 0:
                        sys.stdout.write("# RNG-sus has failed us :(, realizing plan with {} goals completed\n".format(len(self.completed)))
                        sys.stdout.flush()
                        return complete_plan
                    else:
                        self.__init__(self.initial_state)
                        #print(self.state, file=sys.stderr)
                        sys.stdout.write("# checking if randomizing initial state helps\n".format(len(self.completed)))
                        sys.stdout.flush()
                        complete_plan = self.do_desperate_random_shit()
                        ScrambledInitial = True
                        #complete_plan = []
                        rand_max = 5

                #assert False, "no solution could be found"
            else:
                for p in goal_plan:
                    business = 0
                    if p.box_pos_end is None:
                        business += self.dist.dist(p.agent_origin, p.agent_end)
                    else:
                        business += self.dist.dist(p.agent_origin, p.box_pos_origin)
                        business += self.dist.dist(p.box_pos_origin, p.box_pos_end)
                        business += self.dist.dist(p.box_pos_end, p.agent_end)
                    self.agent_business[p.agent_id] += business
                complete_plan.extend(goal_plan)
                blocked_rooms = self.goal_analyzer.get_isolated_by_goal_completion(goal, self.completed)
                if len(blocked_rooms) == 0:
                    blocked_rooms = set()
                else:
                    blocked_rooms = set.union(*[self.goal_analyzer.rooms[room] for room in blocked_rooms])

                blocked_rooms.add(self.state.goal_positions[goal])
                #print(self.state.goal_types[goal],file=sys.stderr,flush = True)
                #print(self.level_analyzer.inventory,file=sys.stderr,flush = True)

                self.level_analyzer.subtract_from_inventory(blocked_rooms, self.state)
                

                self.blocked = self.blocked.union(blocked_rooms) #cells that are being blocked
                #self.level_analyzer.subtract_from_inventory
                self.completed.add(goal)

                ##LA: stuff is blocked


                goal_pos = self.state.goal_positions[goal]
                if self.state.goal_agent[goal]:
                    self.unusable_agents.add(self.state.agent_by_cords[goal_pos])
                else:
                    self.unusable_boxes.add(self.state.box_by_cords[goal_pos])
                #print(self.state)

        return complete_plan

    def solve(self):
        print("planning starting:", file=sys.stderr, flush=True)
        start = time.time()
        plan = self.compute_plan()
        end = time.time()
        print("plan computed in:", end - start, ", realizing...", file=sys.stderr, flush=True)
        start = end
        realizer = ParallelRealizer(self.initial_state, self.dist, self.solo_agents)
        master_plan = realizer.realize_plan(plan)
        end = time.time()
        print("plan realized in:", end - start, file=sys.stderr, flush=True)

        return master_plan

