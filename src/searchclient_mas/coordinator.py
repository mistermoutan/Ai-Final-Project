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
        
        s0 = self.state.get_StateSA(0, True)
        s1 = self.state.get_StateSA(1, True)
        
        plan_0 = Planner(s0, heuristic=self.bfs_heuristic, g_value=lambda x: 1,cutoff_solution_length=30).make_plan()
        plan_1 = Planner(s1, heuristic=self.bfs_heuristic, g_value=lambda x: 1,cutoff_solution_length=30).make_plan()
        
        #For some reason, when we include another agent, everything just goes straight to shit. 
        #s2 = self.state.get_StateSA(2, True)
        #plan_2 = Planner(s2, heuristic=self.bfs_heuristic, g_value=lambda x: 1,cutoff_solution_length=30).make_plan()

        plan = MasterPlan(2,self.state)
        plan.merge_plan_into_master(0,plan_0)
        plan.merge_plan_into_master(1,plan_1)
        #plan.merge_plan_into_master(2,plan_2)

        return plan.plan