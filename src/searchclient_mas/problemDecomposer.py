from state import StateSA,StateMA
from typing import List, Tuple

class problemDecomposer():
    def __init__(self,state:StateMA()):
        self.Tasks = []
        self.state = state

        pass
    def createSetOfTasks(self):
        self.checkGoals()
        self.getPossibleAgents()
        for i in range(len(self.goal_need_task)):
            if self.goal_need_task[i]:
                pass
                #createTask()
    '''
    creates a top level task for an agent to transport a box from the initial state to the goal state
    '''
    def createTask(self,box,from_cor,to_cor,agent):
        for i in self.goal_need_task:

        Task: List[Tuple[int, Tuple[int, int], Tuple[int, int],List[int]]] = None
        self.Tasks.append(Task)
    '''Checks if all boxes are at their goal state
    returns list of boolean where true represents a need for a task
    '''
    def checkGoals(self):
        self.goal_need_task =[]
        #iterate over all goals:
        for i in range(len(self.state.goal_types)):
            self.goal_need_task.append(False)
            p = self.state.goal_position[i]
            if p in self.state.box_by_cords:
                box_id = self.state.box_by_cords[p]
                if self.state.goal_types[i] != self.state.box_types[box_id]:
                    self.goal_need_task[i]=True
            else:
                self.goal_need_task[i]=True
    '''Multiple agents for one box possible?'''
    def getPossibleAgents(self):
        self.PossibleAgents = []
        for i in range(len(self.state.box_types)):
            agents = []
            if self.goal_need_task[i]:
                for c in self.state.agent_colors:
                    if c==i:
                        agents.append(c)
                self.PossibleAgents.append(agents)
            else:
                self.PossibleAgents.append(None)

    def getTasks(self):
        return self.Tasks
 