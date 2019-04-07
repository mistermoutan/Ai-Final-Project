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
    def getGoalOrientedProblems(self):

        g = []
        for i in range(len(self.state.goal_types)):
            g.append((i,self.searchPossibleBoxesForGoal(i)))

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
    def searchPossibleAgentsForBox(self,box_idx):
        '''returns a list of agent indexes that are able to move the box at pos box_idx in the box list'''
        return [idx for idx in range(len(self.state.agent_colors)) if self.state.box_colors[box_idx]==self.state.agent_color[idx]]

    def searchAgentsForAllBoxes(self):
        '''links all boxes in list of boxes to agents that can move the box'''
        self.agts_for_boxes = []
        for c in self.state.box_colors:
            self.agts_for_boxes.append([i for i,x in enumerate(self.state.agent_colors) if x ==c])
    def searchPossibleBoxesForGoal(self,goal_idx):
        '''returns a list of box-indexes that are able to satisfy the goal at pos goad_idx in the goal list'''
        return [idx for idx in range(len(self.state.box_types)) if self.state.goal_types[goal_idx] ==self.state.box_types[idx]]
    def searchPossibleGoalsForBox(self,box_idx):
        '''returns a list of goal-indexes that are able to store the box at index box_idx'''
        return [idx for idx in range(len(self.state.goal_types)) if self.state.goal_types[idx] ==self.state.box_types[box_idx]]
    '''Multiple agents for one box possible?'''
    def searchPossibleAgentsForBoxIndex(self,box_idx):
        '''returns a list of lists of agent indexes that are able to move the boxes at pos i in the box list'''
        return [idx for idx in range(len(self.state.agent_colors)) if self.state.box_colors[box_idx]==self.state.agent_colors[idx]]

    def assign_tasks_greedy(self):
        '''- greedely assings tasks to agents with lowest workload that can fullfill the task
           - every box represents a task
           - returns list of lists of tasks for agents at position i
        '''
        self.agt_boxes = [[] for i in range(len(self.state.agent_colors))]

        for i,b in enumerate(self.state.box_colors):
            p_agts = self.searchPossibleAgentsForBoxIndex(i)
            if p_agts:
                ls = [len(self.agt_boxes[p]) for p in p_agts]
                if ls:
                    m = ls.index(min(ls))
                    self.agt_boxes[p_agts[m]].append(i)

                else:
                    self.agt_boxes[p_agts[-1]].append(i)

        self.agt_goals = [[] for i in range(len(self.state.agent_colors))]

        seen = []

        for i,bxs in enumerate(self.agt_boxes):
            for b in bxs:
                for goal in self.searchPossibleGoalsForBox(b):
                    if goal not in seen:
                        self.agt_goals[i].append(goal)
                        seen.append(goal)
                        break

        self.agt_tasks=[(self.agt_boxes[i],self.agt_goals[i]) for i in range(0,len(self.agt_boxes))]

        return self.agt_tasks

        #extension:
        # - pick agent that has lowest workload and is closest to box
        # - estimate workload of agents based on distance box to goal

    def getTasks(self):
        return self.Tasks
