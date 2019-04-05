from state import StateSA,StateMA
from typing import List, Tuple
import sys
''' 
idea
    - get list of tasks/problems
    - decompose to primitive tasks
    - use planner plan pathes for tasks
    - weight tasks
    - sort tasks by weighting
    - decompose tasks to actions
    - send actions
'''

class HTN():
    def __init__(self,state):
        self.decomposer = problemDecomposer(state)
        self.Tasks = self.decomposer.getGoalOrientedProblems()
    def selectAgent(self):
        #Test: select first agent in dict
        pass
    def selectBox(self):
        pass
        #Test: Select first box in dict
    def sortbyWeight(self,obj):
        pass
    def planTask(self):
        pass
    def heuristic(self):
        pass

'''
This class decomposes a level into high level problems where
'''
class problemDecomposer():
    def __init__(self,state):
        self.Tasks  = []
        self.Plan = []
        self.state = state
        #print(self.state.box_types,file= sys.stderr, flush=True)
    def createCompoundTask(self):
        '''
        (isPrimitive(bool),name(str),from(Tuple(x,y)),to(List(Tuple)),with(List(int),precond[],effect[]'
        (False,'TransportTo',(x,y),[x1(x,y),(x,y),...,[1,2],precond[x1[],x2[]],effect:[x1][x2]]
        '''
        pass
    def decomposeToActions(self,task):
        pass
    def decomposeToPrimitiveTask(self,task):
        pass
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
        self.Plan.insert(g)
        return self.Plan
    def searchPossibleAgentsForBox(self,box_idx):
        '''returns a dict of agent indexes that are able to move the box at pos box_idx in the box list and adds an initial value for the weight of an agent'''
        return {idx:0 for idx in range(len(self.state.agent_colors)) if self.state.box_colors[box_idx]==self.state.agent_color[idx]}
    def searchPossibleBoxesForGoal(self,goal_idx):
        '''returns a list of box-indexes that are able to satisfy the goal at pos goad_idx in the goal list'''
        return [idx for idx in range(len(self.state.box_types)) if self.state.goal_types[goal_idx] ==self.state.box_types[idx]]
    def searchPossibleGoalsForBox(self,box_idx):
        '''returns a list of goal-indexes that are able to store the box at index box_idx'''
        return [idx for idx in range(len(self.state.goal_types)) if self.state.goal_types[idx] ==self.state.box_types[box_idx]]
    '''Multiple agents for one box possible?'''
    def searchPossibleAgentsForBoxIndex(self,box_idx):
        '''returns a list of agent indexes that are able to move the boxes at pos i in the box list'''
        return [idx for idx in range(len(self.state.agent_colors)) if self.state.box_colors[box_idx]==self.state.agent_colors[idx]]



    def assign_tasks_greedy(self):
        '''- greedely assings tasks to agents with lowest workload that can fullfill the task
           - every box represents a task
           - returns list of lists of tasks for agents at position i
        '''
        self.agt_tasks = [[] for i in range(len(self.state.agent_colors))]

        for i,b in enumerate(self.state.box_colors):
            p_agts = self.searchPossibleAgentsForBoxIndex(i)
            if p_agts:
                ls = [len(self.agt_tasks[p]) for p in p_agts]
                if ls:
                    m = ls.index(min(ls))
                    self.agt_tasks[p_agts[m]].append(i)
                else:
                    self.agt_tasks[p_agts[-1]].append(i)

        #extension:
        # - pick agent that has lowest workload and is closest to box
        # - estimate workload of agents based on distance box to goal

    def getTasks(self):
        return self.Tasks
