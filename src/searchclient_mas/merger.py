from action import ActionType
from planner import Planner

class MergeState(object):
    def __init__(self, game_state, agent_id, agent_plan, master_plan, master_plan_index, agent_plan_index=None, g_value = 0):
        self.game_state = game_state
        self.agent_id = agent_id
        self.agent_plan = agent_plan
        self.master_plan = master_plan
        self.master_plan_index = master_plan_index
        self.agent_plan_index = agent_plan_index
        self.g_value = g_value
        self.parent = None
        self.performed_move = False

    def copy(self):
        return MergeState(self.game_state, self.agent_id, self.agent_plan, self.master_plan, self.master_plan_index, self.agent_plan_index, self.g_value)

    def get_children(self):
        action_vector = self.master_plan[self.master_plan_index]
        
        children = []

        #produce the state resulting from making the next move in the action plan
        next_action = action_vector[:]
        next_action[self.agent_id] = self.agent_plan[self.agent_plan_index]
        
        #If there is a conflict after this action, game_state_after_move is None
        game_state_after_move = self.game_state.next_state(next_action)
        #Make a new MergeState if next game state isn't None
        if game_state_after_move:
            s1 = self.copy()
            s1.game_state = game_state_after_move
            s1.master_plan_index += 1
            s1.agent_plan_index +=1
            s1.g_value += 1
            s1.parent = self
            s1.performed_move = True
            children.append(s1)


        #produce the state resulting from waiting a step 
        wait_action = action_vector[:]
        wait_action[self.agent_id] = ActionType.Wait
        game_state_after_wait = self.game_state.next_state(wait_action)
        if game_state_after_wait:        
            s2 = self.copy()
            s2.game_state = game_state_after_wait
            s2.master_plan_index += 1
            #Note that the agent_plan_index remains the same because we inserted a wai action
            s2.g_value += 1
            s2.parent = self
            s2.performed_move = False
            children.append(s2)

        return children

    def heuristic(self):
        return self.g_value + len(self.agent_plan) - self.agent_plan_index

    def is_goal_state(self):
        return self.agent_plan_index >= len(self.agent_plan)

    def extract_plan(self):
        performed_moves = []
        current = self
        i = len(self.agent_plan) - 1
        while(current.parent != None):
            if current.performed_move:
                performed_moves.append(self.agent_plan[i])
                i -= 1
            else:
                performed_moves.append(ActionType.Wait)
            current = current.parent
        
        return performed_moves.reverse()


def merge(agent_id, agent_plan, master_plan, master_plan_index, initial_game_state):        
        initial_state = MergeState(initial_game_state, agent_id, agent_plan, master_plan, master_plan_index=master_plan_index)
        merge_planner = Planner(initial_state)
        revised_agent_plan = merge_planner.make_plan()
        print(revised_agent_plan)

