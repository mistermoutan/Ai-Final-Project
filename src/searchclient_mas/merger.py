from action import *
from planner import Planner
from state import StateMA
import sys

class MergeState(object):

    def __init__(self, game_state, agent_id, agent_plan, master_plan, master_plan_index, agent_plan_index=None, g_value=0):
        self.game_state = game_state
        self.agent_id = agent_id
        self.agent_plan = agent_plan
        self.master_plan = master_plan
        self.master_plan_index = master_plan_index
        self.agent_plan_index = agent_plan_index
        self.g_value = g_value
        self.parent = None
        self.action_performed = None

    def _copy(self):
        return MergeState(
            self.game_state, 
            self.agent_id, 
            self.agent_plan, 
            self.master_plan, 
            self.master_plan_index, 
            self.agent_plan_index, 
            self.g_value
        )
    
    #Get the next uncompleted action in the agents plan in this merge state
    def get_next_action_of_agent(self):
        return self.agent_plan[self.agent_plan_index]

    #Get the next uncompleted action vector from the master plan in this merge state
    def get_next_action_vector(self):
        return self.master_plan[self.master_plan_index].copy()

    #Produces the next merge state from the current merge state
    #You can choose to take the next action in the agents plan or to wait for one step
    def get_next_merge_state(self, should_wait_for_one_step):
        
        #Determine what the next action of the agent is
        next_action = ActionType.Wait if should_wait_for_one_step \
                      else self.get_next_action_of_agent()

        #Based on next action, determine next action vector
        action_vector = self.get_next_action_vector()
        action_vector[self.agent_id] = next_action

        #Return None if action_vector causes a conflict
        next_game_state = self.game_state.get_child(action_vector)
        if not next_game_state:
            return None
        
        #Generate the next merge state
        next_state = self._copy()

        #Update values common for both waiting and moving
        next_state.game_state = next_game_state
        next_state.master_plan_index += 1
        next_state.g_value += 1
        next_state.parent = self
        next_state.action_performed = next_action
        
        #We only increment agent_plan_index when we complete an action in the agents plan
        #If the agent waited for a step, we don't inrement
        if not should_wait_for_one_step:
            next_state.agent_plan_index += 1

        return next_state

    def _merge_state_after_next_action(self):
        return self.get_next_merge_state(should_wait_for_one_step=False)

    def _merge_state_after_waiting_one_step(self):
        return self.get_next_merge_state(should_wait_for_one_step=True)

    def get_children(self):
        #if self.agent_id == 1:
        #    print("------------------------------------", file=sys.stderr)
        #    print(self.extract_plan(), file=sys.stderr)
        #    print(self.game_state, file=sys.stderr)
        #input(">")
        # If the agent plan cannot fit within the master plan, it might extend beyond the 
        # length of the action plan. In this case we add a NoOp action vector for each move
        # in the action plan that extends beyond the master plan
        difference_in_length = (self.master_plan_index + 1) - len(self.master_plan)
        if difference_in_length > 0:
            action_vector_length = len(self.master_plan[0]) 
            empty_action_vector = [ActionType.Wait]*action_vector_length
            self.master_plan.extend([empty_action_vector.copy() for _ in range(difference_in_length)])

        #Return the children in which no conflict happened
        children = [self._merge_state_after_next_action(),self._merge_state_after_waiting_one_step()]        
        return [x for x in children if x]

    #The heuristic is the number of actions left in the action plan of the agent. 
    def heuristic(self):
        
        return self.get_g_value() + len(self.agent_plan) - self.agent_plan_index
    
    def get_g_value(self):
        return self.g_value

    #The merge state is a goal state if the entire agent_plan has been merged into the master plan
    def is_goal_state(self):
        return self.agent_plan_index >= len(self.agent_plan)

    #To extract the plan, backtrack from the goal state while building the list of actions performed by the agent
    def extract_plan(self):
        new_agent_plan = []
        current = self
        
        while current.parent:
            new_agent_plan.append(current.action_performed)
            current = current.parent

        #Since we are backtracking from goal, the last actions are added first, and the plan must be reversed.
        new_agent_plan.reverse()
        return new_agent_plan
    
    def __lt__(self, other):
        return False



#Merge a plan for agent_id into the plan 'master_plan' for all agents, starting at master_plan_index. 
#initial_game_state is the state of the level after all the actions vectors before master_plan_index 
#have been applied 
def merge(agent_id : int, agent_plan, master_plan, master_plan_index, initial_game_state):
        
        #Make defensive copies to avoid side effects
        #Better safe than sorry
        master_plan_copy = [action_vector.copy() for action_vector in master_plan]
        agent_plan_copy = agent_plan.copy()
        
        #Create the initial merge state
        initial_state = MergeState(
            game_state=initial_game_state, 
            agent_id = agent_id, 
            agent_plan =agent_plan_copy,
            master_plan = master_plan_copy, 
            master_plan_index=master_plan_index, 
            agent_plan_index=0
        )
        
        #Get the revised plan for the agent, if it exists. If the plans are (naively) mergable, it is at least possible to append the
        #agent plan to the end of the master plan. Thus the sum of the length of the two plans is an upper bound for the 
        #length of the solution and can be used as a cutoff value for the planner
        #maximum_length_of_solution = len(master_plan) + len(agent_plan)
        maximum_length_of_solution = 200000
        merge_planner = Planner(initial_state, cutoff_solution_length=maximum_length_of_solution)
        
        return merge_planner.make_plan()




