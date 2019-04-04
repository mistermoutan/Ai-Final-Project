from action import ActionType
from action import Action
from action import Dir
import copy
from merger import merge
from state import StateMA
from typing import List
import sys      


class MasterPlan(object):
    def noop_action_vector(self):
        return [ActionType.Wait] * self.number_of_agents

    def __init__(self, number_of_agents: int, initial_state: StateMA):
        self.number_of_agents = number_of_agents
        
        #Initialize the empty plan 
        self.current_plan_length = 1
        self.plan = [self.noop_action_vector() for _ in range(self.current_plan_length)] 
        
        self.states = [None]*self.current_plan_length
        self.states[0] = initial_state

        #The index of the last action that each agent commited to
        self.index_after_last_action = [0]*number_of_agents

    def copy(self):
        return copy.deepcopy(self)

    def pretty_print_master_plan(self):
        agents = range(self.number_of_agents)
        plans = [[] for i in agents]
        for action_vector in self.plan:
            for agent,action in enumerate(action_vector):
                plans[agent].append(action)
        
        print("MASTER PLAN", file=sys.stderr)
        for plan in plans:
            print(plan, file=sys.stderr)

    def merge_plan_into_master(self, agent_id: int, agent_plan: List[Action]):
        first_index_in_plan = self.index_after_last_action[agent_id]
        revised_agent_plan = merge(agent_id, agent_plan, self.plan, first_index_in_plan, self.states[first_index_in_plan])
        
        if not revised_agent_plan:
            return False
                
        #Increase the length of the master plan if necessary
        difference_in_length = (len(revised_agent_plan) + first_index_in_plan) - len(self.plan)
        if difference_in_length > 0:
            self.plan.extend([self.noop_action_vector() for _ in range(difference_in_length)])
            self.states.extend([None]*difference_in_length)
            
        #Update the action vectors in the master plan to reflect the change to the plan
        for i,action in enumerate(revised_agent_plan):
            index_of_action_vector = first_index_in_plan + i
            self.plan[index_of_action_vector][agent_id] = action

        #Update the states to reflect the changes to the master plan
        #Note that all states after a changed action vector must be updated, not just
        #those at the same index as changed action vector
        for i in range(first_index_in_plan, len(self.plan)-1):
            self.states[i+1] = self.states[i].get_child(self.plan[i])

        #Update the index of where to start the next plan for this agent
        self.index_after_last_action[agent_id] += len(revised_agent_plan)

        return True

    def get_plan_of_agent(self, agent_id):
        end_of_agents_plan = self.index_after_last_action[agent_id]
        return [self.plan[i][agent_id] for i in range(end_of_agents_plan)]
