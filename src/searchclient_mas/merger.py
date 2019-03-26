from action import *
from planner import Planner
from state import StateMA

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

    def merge_state_after_next_move(self):
        #Determine the next action vector
        action = self.master_plan[self.master_plan_index]
        action[self.agent_id] = self.agent_plan[self.agent_plan_index]
        
        #Determine the state resulting from applying this action vector
        next_game_state = self.game_state.get_child(action)

        if next_game_state == None:
            return None
        
        next_merge_state = self.copy()
        next_merge_state.game_state = next_game_state
        next_merge_state.master_plan_index += 1
        next_merge_state.agent_plan_index += 1
        next_merge_state.g_value += 1
        next_merge_state.parent = self
        next_merge_state.performed_move = True

        return next_merge_state

    def merge_state_after_wait(self):
        #Determine the next action vector
        action = self.master_plan[self.master_plan_index]
        action[self.agent_id] = None
        #ActionType.Wait
        
        #Determine the state resulting from applying this action vector
        next_game_state = self.game_state.get_child(action)

        if next_game_state == None:
            return None
        
        #Note that we do not increment agent_plan_index, since we wait instead of performing the action
        next_merge_state = self.copy()
        next_merge_state.game_state = next_game_state
        next_merge_state.master_plan_index += 1
        next_merge_state.g_value += 2
        next_merge_state.parent = self
        next_merge_state.performed_move = False

        return next_merge_state



    def get_children(self):
        children = []
        
        state_after_move = self.merge_state_after_next_move()
        if state_after_move:
            children.append(state_after_move)
        
        state_after_wait = self.merge_state_after_wait()
        if state_after_wait:
            children.append(state_after_wait)
        
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
        
        performed_moves.reverse()
        return performed_moves
    
    def __lt__(self, other):
        return False


def merge(agent_id, agent_plan, master_plan, master_plan_index, initial_game_state):        
        initial_state = MergeState(
            game_state=initial_game_state, 
            agent_id = agent_id, 
            agent_plan =agent_plan, 
            master_plan = master_plan, 
            master_plan_index=master_plan_index, 
            agent_plan_index=0)
        merge_planner = Planner(initial_state)
        revised_agent_plan = merge_planner.make_plan()
        print(revised_agent_plan)


maze = [
        [False,False,False,False,False],
        [False,True,True,True,False],
        [False,True,True,True,False],
        [False,True,True,True,False],
        [False,False,False,False,False]
    ]

boxes = [(3,(2,3),0)]
agent = [((2,1),0), ((3,2),1)]
goals = []

initial_state = StateMA(maze,boxes,goals,agent)

ME = Action(ActionType.Move, Dir.E, None)
PN = Action(ActionType.Push, Dir.N, Dir.N)
PNW = Action(ActionType.Push, Dir.N, Dir.W)
PWW = Action(ActionType.Push, Dir.W, Dir.W)

master_plan = [[ME, None], [PN, None], [PNW, None], [PWW, None]]
agent_plan = [ME, ME]

merge(1, agent_plan, master_plan, 0, initial_state)

