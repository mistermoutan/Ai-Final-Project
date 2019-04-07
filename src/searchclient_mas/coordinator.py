from state import StateMA,StateSA
from action import north,south,west,east,move,push,pull
from master_plan import MasterPlan
from planner import Planner
from graph_alternative import Graph
from problemDecomposer import problemDecomposer
import sys

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

    def make_single_agent_plan(self, initial_state):
        return Planner(initial_state, heuristic=self.heuristic, g_value=lambda x: 1,cutoff_solution_length=30).make_plan()

    def solve(self):

        number_of_agents = len(self.state.agent_positions)
        agents = range(number_of_agents)

        greedy_decomposition = True

        if greedy_decomposition:
            print("box_types:{}".format(self.state.box_types),file= sys.stderr,flush=True)
            pd = problemDecomposer(self.state)
            agt_tasks = pd.assign_tasks_greedy()
            single_agent_states = [self.state.get_greedy_StateSA(i,agt_tasks[i], False) for i in agents]

        else:
            single_agent_states = [self.state.get_StateSA(i, True) for i in agents]

        single_agent_plans = [self.make_single_agent_plan(s) for s in single_agent_states]

        master_plan = MasterPlan(number_of_agents, self.state)

        for agent,plan in enumerate(single_agent_plans):
            master_plan.merge_plan_into_master(agent,plan)

        return master_plan.plan
