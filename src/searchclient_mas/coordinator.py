from state import StateMA,StateSA
from action import north,south,west,east,move,push,pull
from master_plan import MasterPlan
from planner import Planner
import sys

class Coordinator:
    def __init__(self, state:StateMA):
        self.state = state
        self.bfs_counter = 0
    
    def bfs_heuristic(self, state):
        self.bfs_counter += 1
        return self.bfs_counter

    def solve(self):
        

        number_of_agents = len(self.state.agent_positions)
        agents = range(number_of_agents)
        single_agent_states = [self.state.get_StateSA(i, True) for i in agents]
        
        make_plan = lambda s : Planner(s, heuristic=self.bfs_heuristic, g_value=lambda x: 1,cutoff_solution_length=30).make_plan()
        single_agent_plans = [make_plan(s) for s in single_agent_states]

        master_plan = MasterPlan(number_of_agents, self.state)
        
        for agent,plan in enumerate(single_agent_plans):
            master_plan.merge_plan_into_master(agent,plan)
        
        return master_plan.plan

        """        #s0 = self.state.get_StateSA(0, True)
        #s1 = self.state.get_StateSA(1, True)
        
        #plan_0 = Planner(s0, heuristic=self.bfs_heuristic, g_value=lambda x: 1,cutoff_solution_length=30).make_plan()
        #plan_1 = Planner(s1, heuristic=self.bfs_heuristic, g_value=lambda x: 1,cutoff_solution_length=30).make_plan()
        
        #For some reason, when we include another agent, everything just goes straight to shit. 
        #s2 = self.state.get_StateSA(2, True)
        #plan_2 = Planner(s2, heuristic=self.bfs_heuristic, g_value=lambda x: 1,cutoff_solution_length=30).make_plan()

        
        #plan.merge_plan_into_master(0,plan_0)
        #plan.merge_plan_into_master(1,plan_1)
        #plan.merge_plan_into_master(2,plan_2)

        return plan.plan"""