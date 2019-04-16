from state import StateMA,StateSA
from action import north,south,west,east,move,push,pull
from master_plan import MasterPlan
from planner import Planner
from graph import Graph
from problemDecomposer import HTN, subtask, problemDecomposer
import sys
import copy

class Coordinator:
    def __init__(self, state:StateMA):
        self.state = state
        self.bfs_counter = 0
        self.graph_of_level = Graph(state.maze)

    def bfs_heuristic(self, state):
        self.bfs_counter += 1
        return self.bfs_counter

    def distance_to(self, x, y):
        return len(self.graph_of_level.shortest_path_between(x,y))

    def min_distance_to_position_in_list(self, box, goals):
        distances = [self.distance_to(box, goal) for goal in goals]
        return min(distances)


    #Simple heuristic which minimizes the value (agent_to_box + box_to_goal) distance
    def heuristic(self, state):
        agent = (state.agent_row, state.agent_col)
        boxes = state.box_positions
        goals = state.goal_positions

        agent_to_box_distances = [self.distance_to(agent, box) for box in boxes]
        box_to_goals_distances = [self.min_distance_to_position_in_list(box, goals) for box in boxes]
        agent_to_box_goal_goal_distances = [agent_to_box_distances[i] + box_to_goals_distances[i] for i in range(len(agent_to_box_distances))]
        return min(agent_to_box_goal_goal_distances)

    def heuristic_2(self, state):

        # Work in Progress!!

        agent = (state.agent_row, state.agent_col)
        boxes = state.box_positions
        goals = state.goal_positions

        alpha = 10 #penalizing factor for distance goals_to_box

        seen = []
        goals_to_box = []
        agent_to_box = []

        # distance from every goal to closest box that can satisfy that goal and is not assigned to other goal alread
        for i, goal in enumerate(goals):
            distances = []
            for j, box in enumerate(boxes):
                if j not in seen:
                    if state.goal_types[i] == state.box_types[j]:
                        distances.append(self.distance_to(box, goal))
                    else:
                        distances.append(1000)

            min_index = distances.index(min(distances))
            seen.append(min_index)
            goals_to_box.append(distances[min_index]-1)

             # distance from agent to closest box that is not at its goal
            if distances[min_index]-1 != 0:
                agent_to_box.append(self.distance_to(agent, boxes[min_index])-1)

        # not enough boxes for goals
        assert 999 not in goals_to_box

        #avoid error in goal state
        if agent_to_box == []:
            agent_to_box = [0]

        return alpha * sum(goals_to_box) + min(agent_to_box)


    def make_single_agent_plan(self, initial_state):
        return Planner(initial_state, heuristic=self.heuristic, g_value=lambda x: 1,cutoff_solution_length=30).make_plan()

    def solve(self):

        number_of_agents = len(self.state.agent_positions)
        agents = range(number_of_agents)

        greedy_decomposition = True

        if greedy_decomposition:
            #print("box_types:{}".format(self.state.box_types),file= sys.stderr,flush=True)
            pd = problemDecomposer(self.state)
            agt_tasks = pd.assign_tasks_greedy()
            single_agent_states = [self.state.get_greedy_StateSA(i,agt_tasks[i], True) for i in agents]
            #print("single_agent_states",file=sys.stderr,flush=True)
            #print(single_agent_states,file=sys.stderr,flush=True)
        else:
            single_agent_states = [self.state.get_StateSA(i, True) for i in agents]

        single_agent_plans = [self.make_single_agent_plan(s) for s in single_agent_states]

        master_plan = MasterPlan(number_of_agents, self.state)

        for agent,plan in enumerate(single_agent_plans):
            master_plan.merge_plan_into_master(agent,plan)

        return master_plan.plan
