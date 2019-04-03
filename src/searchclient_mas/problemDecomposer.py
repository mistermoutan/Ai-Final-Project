from state import StateSA,StateMA
from typing import List, Tuple
import sys
'''
This class decomposes a level into high level problems where
'''
class problemDecomposer():
    def __init__(self,state):
        self.Tasks  = []
        self.state = state
        #print(self.state.box_types,file= sys.stderr, flush=True)
        self.createSetOfTasks()
    def createSetOfTasks(self):
        pass
        '''self.checkGoals()
        print(self.goal_need_task,file= sys.stderr, flush=True)

        #self.getPossibleAgents()
        for i in range(len(self.goal_need_task)):
            if self.goal_need_task[i]:
                Task = (i, self.state.box_positions[i],self.state.goal_positions[i],self.PossibleAgents[i])
                self.Tasks.extend(Task)
        '''

    '''Checks if all boxes are at their goal state
    returns list of boolean where true represents a need for a task
    '''
    def checkGoals(self):
        pass

        '''self.goal_need_task =[]
        #iterate over all goals:
        for i in range(len(self.state.goal_types)):
            self.goal_need_task.append(False)
            p = self.state.goal_positions[i]
            if p in self.state.box_by_cords:
                box_id = self.state.box_by_cords[p]
                if self.state.goal_types[i] != self.state.box_types[box_id]:
                    self.goal_need_task[i]=True
            else:
                self.goal_need_task[i]=True
    '''
    '''Multiple agents for one box possible?'''
    def searchPossibleAgentsForBox(self,box_idx):
        '''returns a list of lists of agent indexes that are able to move the boxes at pos i in the box list'''
        return [idx for idx in range(len(self.state.agent_colors)) if self.state.box_colors[box_idx]==self.state.agent_colors[idx]]
    def searchPossibleBoxesForGoals(self):
        '''returns a list of lists of box-indexes that are able to satisfy the goal at pos i in the goal list'''
        self.pos_boxes=[]
        for i in range(len(self.state.goal_types)):
            self.pos_boxes.append([idx for idx in range(len(self.state.box_types)) if self.state.goal_types[i] ==self.state.box_types[idx]])

    def searchPossibleGoalsForBoxes(self):
        '''returns a list of lists of goal-indexes that are able to store the box at pos i in the goal list'''
        self.pos_goals=[]
        for i in range(len(self.state.box_types)):
            self.pos_goals.append([idx for idx in range(len(self.state.goal_types)) if self.state.goal_types[idx] ==self.state.box_types[i]])

    def getTasks(self):
        return self.Tasks
