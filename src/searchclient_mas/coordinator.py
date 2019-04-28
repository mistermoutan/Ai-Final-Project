from state import StateMA,StateSA
from action import north,south,west,east,move,push,pull
from master_plan import MasterPlan
from planner import Planner
from graph import Graph
from problemDecomposer import problemDecomposer
import sys
import copy

class Coordinator:
    def __init__(self, state:StateMA):
        self.state = state
        self.bfs_counter = 0
        self.graph_of_level = Graph(state.maze, precompute = "all_shortest_paths")

    def bfs_heuristic(self, state):
        self.bfs_counter += 1
        return self.bfs_counter

    def distance_to(self, x, y):
        ### current error here -> differnce of 2 for last step
        # distance_to((1,1),(1,2)) = 2
        # distance_to((1,1),(1,1)) = 0

        #resolve here:
        path_length = len(self.graph_of_level.shortest_path_between(x,y))
        if path_length > 0:
            return path_length-1
        else:
            return path_length

    def min_distance_to_position_in_list(self, box, goals):
        distances = [self.distance_to(box, goal) for goal in goals]
        return min(distances)

    def distances_to_position_in_list(self, pos1, poslist):
        return [self.distance_to(pos1, pos2) for pos2 in poslist]

    def ind_n_dis_goals_to_closest_box(self, state, boxes, goals):
        '''
        for every goal return the index of closest box that can satisfy that goal
        (and is not assigned to another goal already) and the distance to it
        (we have to calculate the distance anyway so we might aswell keep it)
        '''
        closest_boxes = []
        distances = []

        for i, goal in enumerate(goals):
            d = []
            for j, box in enumerate(boxes):
                if j not in closest_boxes:
                    if state.goal_types[i] == state.box_types[j]:
                        d.append(self.distance_to(box, goal))
                    else:
                        d.append(2500)
                else:
                    d.append(2500)

            min_index = d.index(min(d))
            closest_boxes.append(min_index)
            distances.append(d[min_index])

        return closest_boxes, distances

    #Simple heuristic which minimizes the value (agent_to_box + box_to_goal) distance
    def heuristic(self, state):
        agent = (state.agent_row, state.agent_col)
        boxes = state.box_positions
        goals = state.goal_positions

        agent_to_box_distances = [self.distance_to(agent, box) for box in boxes]
        box_to_goals_distances = [self.min_distance_to_position_in_list(box, goals) for box in boxes]
        agent_to_box_goal_goal_distances = [agent_to_box_distances[i] + box_to_goals_distances[i] for i in range(len(agent_to_box_distances))]
        return min(agent_to_box_goal_goal_distances)

    def heuristic_adv(self, state, alpha = 1):
        '''
        Idea is to combine different heurisitcs here and wheight them differently, sqaure them, etc.
        Work in progress
        '''
        agent = (state.agent_row, state.agent_col)
        boxes = state.box_positions
        goals = state.goal_positions

        alpha = 5.5 #penalizing factor for distance goals_to_box
        square_goals2box = True #all goals will be solved almost in "parrallel"
        square_agt2boxes = False #boxes will be pushed to their goals "sequentially"

        #closest box for every goal and the distance to it
        closest_boxes, dist_goals_to_box = self.ind_n_dis_goals_to_closest_box(state, boxes, goals)

        #distances form agent to all boxes that are not in goal state
        dist_agent_to_boxes = self.distances_to_position_in_list(agent, [boxes[cb] for i,cb in enumerate(closest_boxes) if dist_goals_to_box[i] != 0])
        dist_agent_to_boxes = [d-1 for d in dist_agent_to_boxes] #currently error of 1 #TODO resolve this?

        # not enough boxes for goals
        if 2500 in dist_goals_to_box:
            raise ValueError("Not enough boxes to satisfy all goals")

        #avoid error in goal state - every goal is satisfied
        if dist_agent_to_boxes == []:
            dist_agent_to_boxes = [0]

        if square_goals2box:
            dist_goals_to_box = [d*d for d in dist_goals_to_box]

        if square_agt2boxes:
            dist_agent_to_boxes = [d*d for d in dist_agent_to_boxes]

        #print("dist_goals_to_box: {}, dist_agent_to_boxes: {} ".format(dist_goals_to_box, dist_agent_to_boxes), file= sys.stderr,flush=True)
        h = alpha * sum(dist_goals_to_box) + min(dist_agent_to_boxes) + state.g

        return h


    def make_single_agent_plan(self, initial_state):
        return Planner(initial_state, heuristic=self.heuristic_adv, g_value=lambda x: 1,cutoff_solution_length=300).make_plan()

    def solve(self):

        number_of_agents = len(self.state.agent_positions)
        agents = range(number_of_agents)

        single_agent_states = [self.state.get_StateSA(i, True) for i in agents]

        single_agent_plans = [self.make_single_agent_plan(s) for s in single_agent_states]

        master_plan = MasterPlan(number_of_agents, self.state)

        for agent,plan in enumerate(single_agent_plans):
            master_plan.merge_plan_into_master(agent,plan)

        return master_plan.plan

    def solve_greedy_decomposition(self):

        number_of_agents = len(self.state.agent_positions)
        agents = range(number_of_agents)

        #print("box_types:{}".format(self.state.box_types),file= sys.stderr,flush=True)
        pd = problemDecomposer(self.state)
        agt_tasks = pd.assign_tasks_greedy()
        single_agent_states = [self.state.get_greedy_StateSA(i,agt_tasks[i], True) for i in agents]
        #print("single_agent_states",file=sys.stderr,flush=True)
        #print(single_agent_states,file=sys.stderr,flush=True)

        single_agent_plans = [self.make_single_agent_plan(s) for s in single_agent_states]

        master_plan = MasterPlan(number_of_agents, self.state)

        for agent,plan in enumerate(single_agent_plans):
            master_plan.merge_plan_into_master(agent,plan)

        return master_plan.plan
